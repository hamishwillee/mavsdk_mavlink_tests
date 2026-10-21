"""
Shared fixtures for all tests.

MAVSDK
------
The suite runs on MAVSDK 4 (the native, in-process binding) through
tests/mavsdk_compat.py: an ``Endpoint`` is one MAVSDK instance with one MAVLink
identity and one connection; a ``SystemShim`` gives tests the old
``system.telemetry.position()`` call style. Endpoints are session-lifetime and
synchronous to create; v4 subscriptions capture whichever event loop is running
when they start, so one endpoint serves every test's (function- or
class-scoped) pytest-asyncio loop. There are no ``mavsdk_server`` processes.

Connection strategy
-------------------
* ``--drone-address`` supplied → standalone mode: the GCS endpoint connects to
  that address (e.g. a PX4 SITL). All client tests run against the real
  flight stack.
* ``--drone-address`` omitted  → paired mode: client tests run against
  MockFlightStack over loopback.

Fixture sets
------------
``gcs_mavsdk_server`` / ``gcs_system`` / ``mock_stack``
    Mode-aware GCS for client tests (names kept from the gRPC era).
    Standalone: ``gcs_mavsdk_server`` is an endpoint on ``--drone-address``;
    ``mock_stack`` is a no-op.
    Paired: ``gcs_mavsdk_server`` is ``paired_gcs_server``; ``mock_stack``
    runs MockFlightStack on the paired drone endpoint.

``paired_gcs_server`` / ``paired_drone_server``
``paired_gcs_system`` / ``paired_drone_system``
    Loopback pair, created on first use (session-scoped). Used by client tests
    in paired mode and by server tests. Port 14560 is used deliberately (not
    14540) to avoid interference from PX4 SITL.

Ports
-----
  GCS_MAVLINK_PORT      = 14560  (MAVLink UDP — paired loopback;
                                  deliberately NOT 14540 to avoid interference
                                  from a concurrently running PX4 SITL)

Identity
--------
  GCS:   MAVSDK GROUND_STATION (sysid=245, compid=190) — see mavsdk_compat.GCS_SYSID
  Drone: sysid=1,   compid=1
"""

import re
import shutil
import signal
import subprocess
import logging
import asyncio
import json
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from tests.mavsdk_compat import ENDPOINTS, GCS_COMPID, GCS_SYSID, Endpoint, SystemShim as System, open_system, open_paired_drone

from tests.mock_flight_stack import MockFlightStack
from tests.message_watcher import (
    ARDUPILOT_MESSAGES, HARNESS_MESSAGES, MAV_AUTOPILOT_ARDUPILOTMEGA, MessageWatcher,
    ensure_harness_streams,
)

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# MAVLink enum lookups — loaded from the bundled XML submodule at import time,
# with hardcoded fallback tables if the submodule is absent.
# ---------------------------------------------------------------------------

_MAV_AUTOPILOT_FALLBACK: dict[int, str] = {
    0: "GENERIC", 1: "PIXHAWK", 3: "ARDUPILOTMEGA", 8: "INVALID", 12: "PX4",
}
_MAV_TYPE_FALLBACK: dict[int, str] = {
    0: "GENERIC", 1: "FIXED_WING", 2: "QUADROTOR", 3: "COAXIAL", 4: "HELICOPTER",
    6: "GCS", 10: "GROUND_ROVER", 13: "HEXAROTOR", 14: "OCTOROTOR", 15: "TRICOPTER",
    19: "VTOL_TAILSITTER_DUOROTOR", 20: "VTOL_TAILSITTER_QUADROTOR",
    21: "VTOL_TILTROTOR", 22: "VTOL_FIXEDROTOR", 23: "VTOL_TAILSITTER",
    24: "VTOL_TILTWING",
}
_MAV_FIRMWARE_TYPE_FALLBACK: dict[int, str] = {
    0: "dev", 64: "alpha", 128: "beta", 192: "rc", 255: "official",
}

# Directory containing common.xml (two levels up from tests/conftest.py)
_MAVLINK_DEFINITIONS_DIR = Path(__file__).parent.parent / "mavlink" / "message_definitions" / "v1.0"


def _load_mavlink_enum(
    enum_name: str,
    fallback: dict[int, str],
    strip_prefix: str | None = None,
    lowercase: bool = False,
) -> dict[int, str]:
    """
    Load a MAVLink enum from the bundled mavlink/ git submodule XML.

    Parses common.xml and follows <include> chains to find ``enum_name``.
    Strips ``strip_prefix`` from entry names if given, then optionally
    lowercases.  Falls back to ``fallback`` with a warning if the XML is
    not present or the enum is not found.
    """
    common_xml = _MAVLINK_DEFINITIONS_DIR / "common.xml"
    if not common_xml.exists():
        log.warning(
            "MAVLink XML not found at %s — using hardcoded %s table. "
            "Run: git submodule update --init mavlink",
            _MAVLINK_DEFINITIONS_DIR,
            enum_name,
        )
        return dict(fallback)

    result: dict[int, str] = {}
    seen: set[str] = set()

    def _parse(filename: str) -> None:
        if filename in seen:
            return
        seen.add(filename)
        fpath = _MAVLINK_DEFINITIONS_DIR / filename
        if not fpath.exists():
            return
        try:
            tree = ET.parse(fpath)
        except ET.ParseError as exc:
            log.warning("Failed to parse MAVLink XML %s: %s", fpath, exc)
            return
        for inc in tree.findall(".//include"):
            if inc.text:
                _parse(inc.text.strip())
        for enum_el in tree.findall(f'.//enum[@name="{enum_name}"]'):
            for entry in enum_el.findall("entry"):
                val = entry.get("value")
                name = entry.get("name")
                if val is not None and name is not None:
                    display = name
                    if strip_prefix and display.startswith(strip_prefix):
                        display = display[len(strip_prefix):]
                    if lowercase:
                        display = display.lower()
                    result[int(val)] = display

    _parse("common.xml")

    if not result:
        log.warning(
            "Enum '%s' not found in MAVLink XML at %s — using hardcoded fallback.",
            enum_name,
            _MAVLINK_DEFINITIONS_DIR,
        )
        return dict(fallback)

    return result


_MAV_AUTOPILOT = _load_mavlink_enum(
    "MAV_AUTOPILOT", _MAV_AUTOPILOT_FALLBACK, strip_prefix="MAV_AUTOPILOT_"
)
_MAV_TYPE = _load_mavlink_enum(
    "MAV_TYPE", _MAV_TYPE_FALLBACK, strip_prefix="MAV_TYPE_"
)
_MAV_FIRMWARE_TYPE = _load_mavlink_enum(
    "FIRMWARE_VERSION_TYPE",
    _MAV_FIRMWARE_TYPE_FALLBACK,
    strip_prefix="FIRMWARE_VERSION_TYPE_",
    lowercase=True,
)


# ---------------------------------------------------------------------------
# Autopilot probing helpers
# ---------------------------------------------------------------------------


async def _probe_autopilot_async(endpoint: Endpoint, timeout_s: int) -> dict:
    """
    Probe the flight stack connected to *endpoint* (the session's GCS
    tests/mavsdk_compat.Endpoint): reads one HEARTBEAT for autopilot/vehicle-type,
    then requests AUTOPILOT_VERSION for firmware version and git hash.

    Returns a dict with keys: autopilot, vehicle_type, firmware_version,
    git_hash, capabilities (int).
    """
    from mavsdk.plugins.mavlink_direct import MavlinkMessage

    try:
        system = await open_system(endpoint, timeout_s)
    except asyncio.TimeoutError:
        return {
            "autopilot": "TIMEOUT", "vehicle_type": "TIMEOUT",
            "firmware_version": "N/A", "git_hash": "N/A", "capabilities": 0,
        }

    info: dict = {"capabilities": 0, "autopilot": "UNKNOWN", "vehicle_type": "UNKNOWN"}

    # --- Firmware version via system.info (MAVSDK high-level API) ---
    # system.info.get_version() internally requests AUTOPILOT_VERSION and decodes it.
    try:
        ver = await asyncio.wait_for(system.info.get_version(), timeout=10.0)
        # MAVSDK 4 reports its own enum (UNKNOWN/DEV/ALPHA/BETA/RC/RELEASE), not
        # MAVLink's raw FIRMWARE_VERSION_TYPE value — map it onto the same
        # labels the raw-message fallback below produces.
        vt = ver.flight_sw_version_type
        fw_type = {"DEV": "dev", "ALPHA": "alpha", "BETA": "beta", "RC": "rc", "RELEASE": "official"}.get(
            getattr(vt, "name", ""), _MAV_FIRMWARE_TYPE.get(int(vt), f"type{int(vt)}"),
        )
        info["firmware_version"] = (
            f"{ver.flight_sw_major}.{ver.flight_sw_minor}.{ver.flight_sw_patch}-{fw_type}"
        )
        info["git_hash"] = ver.flight_sw_git_hash or "N/A"
    except Exception:
        # Fallback: probe AUTOPILOT_VERSION directly via mavlink_direct
        av_seen = asyncio.Event()

        async def _listen_av():
            async for msg in system.mavlink_direct.message("AUTOPILOT_VERSION"):
                fields = json.loads(msg.fields_json)
                v = int(fields.get("flight_sw_version", 0))
                major = (v >> 24) & 0xFF
                minor = (v >> 16) & 0xFF
                patch = (v >> 8) & 0xFF
                fw_type = v & 0xFF
                type_str = _MAV_FIRMWARE_TYPE.get(fw_type, f"type{fw_type}")
                info["firmware_version"] = f"{major}.{minor}.{patch}-{type_str}"
                git_raw = fields.get("flight_custom_version", [])
                if isinstance(git_raw, list) and any(b != 0 for b in git_raw):
                    try:
                        decoded = bytes(git_raw[:8]).decode("ascii").rstrip("\x00").strip()
                        info["git_hash"] = decoded if decoded.isprintable() and decoded else (
                            "".join(f"{b:02x}" for b in git_raw[:8])
                        )
                    except (UnicodeDecodeError, ValueError):
                        info["git_hash"] = "".join(f"{b:02x}" for b in git_raw[:8])
                else:
                    info["git_hash"] = "N/A"
                info["capabilities"] = int(fields.get("capabilities", 0))
                av_seen.set()

        av_task = asyncio.create_task(_listen_av())
        await asyncio.sleep(0.2)
        try:
            await system.mavlink_direct.send_message(
                MavlinkMessage(
                    message_name="COMMAND_LONG",
                    system_id=0, component_id=0,
                    target_system_id=1, target_component_id=1,
                    fields_json=json.dumps({
                        "target_system": 1, "target_component": 1,
                        "command": 512, "confirmation": 0,
                        "param1": 148.0, "param2": 0.0, "param3": 0.0,
                        "param4": 0.0, "param5": 0.0, "param6": 0.0, "param7": 0.0,
                    }),
                )
            )
        except Exception as exc:
            log.warning("AUTOPILOT_VERSION request failed: %s", exc)
        try:
            await asyncio.wait_for(av_seen.wait(), timeout=10.0)
        except asyncio.TimeoutError:
            info.setdefault("firmware_version", "N/A")
            info.setdefault("git_hash", "N/A")
        finally:
            av_task.cancel()
            try:
                await av_task
            except asyncio.CancelledError:
                pass

    # --- Vendor/product name via system.info.get_product() (best-effort) ---
    try:
        prod = await asyncio.wait_for(system.info.get_product(), timeout=5.0)
        if prod.vendor_name:
            info["autopilot"] = prod.vendor_name
        if prod.product_name:
            info["product_name"] = prod.product_name
    except Exception:
        pass

    # --- Autopilot type via HEARTBEAT (best-effort; MAVSDK may filter these) ---
    hb_seen = asyncio.Event()

    async def _listen_hb():
        async for msg in system.mavlink_direct.message("HEARTBEAT"):
            fields = json.loads(msg.fields_json)
            vt_id = int(fields.get("type", 0))
            if vt_id == 6:  # skip GCS heartbeats
                continue
            ap_id = int(fields.get("autopilot", 0))
            info["autopilot_id"] = ap_id
            info["vehicle_type_id"] = vt_id
            info["autopilot"] = _MAV_AUTOPILOT.get(ap_id, f"AUTOPILOT({ap_id})")
            info["vehicle_type"] = _MAV_TYPE.get(vt_id, f"TYPE({vt_id})")
            hb_seen.set()
            return

    hb_task = asyncio.create_task(_listen_hb())
    try:
        await asyncio.wait_for(hb_seen.wait(), timeout=3.0)
    except asyncio.TimeoutError:
        pass  # MAVSDK may filter HEARTBEAT — vehicle type stays UNKNOWN
    finally:
        hb_task.cancel()
        try:
            await hb_task
        except asyncio.CancelledError:
            pass

    return info


def _format_autopilot_header(info: dict, drone_address: str | None) -> str:
    """Format probed autopilot info as a multi-line header string for log files."""
    lines = [
        "=" * 70,
        "FLIGHT STACK PROBE",
        "=" * 70,
        f"  Connection:       {drone_address or 'Mock (paired loopback)'}",
        f"  Autopilot:        {info.get('autopilot', 'N/A')}",
        f"  Vehicle type:     {info.get('vehicle_type', 'N/A')}",
        f"  Firmware version: {info.get('firmware_version', 'N/A')}",
        f"  Git hash:         {info.get('git_hash', 'N/A')}",
        f"  Capabilities:     0x{info.get('capabilities', 0):08x}",
        "=" * 70,
    ]
    return "\n".join(lines)


def _derive_log_prefix(config) -> str:
    """
    Derive a log prefix from the test paths passed on the command line.

    ``pytest tests/mission/test_frame_types.py``               → ``mission_frame_types``
    ``pytest tests/mission/test_frame_types.py::SomeClass``    → ``mission_frame_types``
    ``pytest tests/command/test_protocol.py``                  → ``command_protocol``
    ``pytest tests/``                                          → ``tests``
    """
    args = getattr(config, "args", [])
    # Strip ::NodeId suffixes so "tests/foo/test_bar.py::Class::test" → "tests/foo/test_bar.py"
    py_paths: set[str] = set()
    for a in args:
        base = a.split("::")[0]
        if base.endswith(".py"):
            py_paths.add(base)
    if len(py_paths) == 1:
        p = Path(next(iter(py_paths))).with_suffix("")
        parts = [s for s in p.parts if s != "tests"]
        if parts:
            parts[-1] = parts[-1].removeprefix("test_")
        return "_".join(parts) if parts else "tests"
    return "tests"


def suggest_log_filename(info: dict, config=None) -> str:
    """
    Return a suggested log filename:
      <test_type>_<autopilot>_<vehicle_type>_<version>_<YYYYMMDD_HHMMSS>.log

    Example: mission_frame_types_ardupilot_quadplane_4.8.0-dev_20260526_153000.log
    Filename-safe: dots kept, slashes/spaces replaced with underscores.
    """
    def _safe(s: str) -> str:
        return s.replace("/", "_").replace(" ", "_").replace("\\", "_")

    prefix = _derive_log_prefix(config) if config is not None else "tests"
    ap = _safe(info.get("autopilot", "unknown").lower().replace("ardupilotmega", "ardupilot"))
    vt = _safe(info.get("vehicle_type", "unknown").lower())
    ver_raw = info.get("firmware_version", "")
    ver = f"_{_safe(ver_raw)}" if ver_raw and ver_raw != "N/A" else ""
    ts = time.strftime("%Y%m%d_%H%M%S")
    return f"{prefix}_{ap}_{vt}{ver}_{ts}.log"

GCS_MAVLINK_PORT = 14560  # not 14540 — avoids PX4 SITL interference


def _ardu_model_and_defaults(
    binary_path: Path,
    model_override: str | None,
    vehicle_type: str | None,
) -> tuple[str, list[Path]]:
    """
    Return (model_string, defaults_paths) for an ArduPilot SITL binary — passed
    to the binary as one comma-separated ``--defaults=`` list, later files
    overriding earlier ones.

    Model detection order:
      1. ``--ardupilot-model`` CLI option (explicit override)
      2. Inferred from binary name + ``--vehicle-type``

    Defaults detection:
      - copter → copter.parm, then tests/sitl_defaults/copter_gcs_auto.parm
        (AUTO_OPTIONS=3 — takeoff without RC throttle; see that file)
      - rover  → rover.parm
      - quadplane → quadplane.parm
      - plane → tests/sitl_defaults/plane_ins_cal.parm (upstream ships no
        plane.parm — see that file's own comment for why one is needed)

    Upstream's .parm files come from the checkout the binary was built in
    (``<checkout>/build/sitl/bin/<binary>``), falling back to
    ``~/github/ArduPilot/ardupilot`` for a binary that lives elsewhere.

    All four set small nonzero INS_ACC*OFFS/SCAL values so ArduPilot's arming
    check reads the INS as already calibrated. Without this, a freshly-booted
    SITL instance never reports armable (PreArm: "Arm: 3D Accel calibration
    needed") — confirmed 2026-09-15 against the official ArduPlane 4.7.1
    release SITL binary, which has no compiled-in calibration the way the
    4.8.0-dev build apparently does. An earlier version of this function
    passed ``None`` for plane/quadplane, citing "a .parm file crashes
    quadplane SITL" — re-tested 2026-09-15 with the official quadplane.parm
    against the same release binary and it did NOT crash (armable at ~25s);
    whatever caused that finding didn't reproduce, so it's re-enabled.
    """
    binary_name = binary_path.stem
    if model_override:
        model = model_override
    elif "copter" in binary_name:
        model = "+"
    elif "plane" in binary_name:
        vt = (vehicle_type or "").lower()
        model = "quadplane" if ("quad" in vt or "vtol" in vt) else "plane"
    elif "rover" in binary_name:
        model = "rover"
    elif "sub" in binary_name:
        model = "vectored"
    else:
        model = "+"

    ardu_defaults = binary_path.resolve().parents[3] / "Tools/autotest/default_params"
    if not ardu_defaults.is_dir():
        ardu_defaults = Path("~/github/ArduPilot/ardupilot/Tools/autotest/default_params").expanduser()
    repo_defaults = Path(__file__).parent / "sitl_defaults"
    defaults_map: dict[str, Path] = {
        "+": ardu_defaults / "copter.parm",
        "quad": ardu_defaults / "copter.parm",
        "rover": ardu_defaults / "rover.parm",
        "rover-skid": ardu_defaults / "rover-skid.parm",
        "quadplane": ardu_defaults / "quadplane.parm",
        "plane": repo_defaults / "plane_ins_cal.parm",
    }
    extra_map: dict[str, list[Path]] = {
        "+": [repo_defaults / "copter_gcs_auto.parm"],
        "quad": [repo_defaults / "copter_gcs_auto.parm"],
    }
    candidates = [defaults_map[model]] if model in defaults_map else []
    candidates += extra_map.get(model, [])
    return model, [c for c in candidates if c.exists()]


def _ardu_cmdline_instance(cmdline: str) -> int:
    """Parse the attached `-IN` instance argument out of an ArduPilot process's cmdline (default: 0)."""
    for tok in cmdline.split():
        if tok.startswith("-I") and len(tok) > 2:
            try:
                return int(tok[2:])
            except ValueError:
                pass
    return 0


@pytest.fixture(scope="session", autouse=True)
def _manage_ardupilot_sitl(request):
    """
    Start and stop any ArduPilot SITL binary when ``--ardupilot-sitl`` is given.

    Handles arducopter, arduplane (plane / quadplane), and ardurover.
    The SITL model is determined from ``--ardupilot-model`` (explicit) or
    inferred from the binary name and ``--vehicle-type`` (automatic).
    The defaults .parm file is selected automatically based on the model.
    ``--sitl-instance`` (default 0) selects which ArduPilot SITL instance to
    run, passed straight through as the binary's own native ``-I N`` flag
    (confirmed via ``--help``: "adds 10*instance to all port numbers") — see
    root CLAUDE.md's CI section for the full multi-instance design.

    Kills any existing process for *this same instance* (not other
    instances — see root CLAUDE.md CI section) with the same binary name
    first to clear stale CLOSE-WAIT TCP connections — ArduPilot does not
    close its socket when clients disconnect abruptly, which prevents new
    connections from receiving MAVLink heartbeats.

    Readiness: waits for "Waiting for connection" in the SITL log (the point
    at which the MAVLink TCP stack is up and the binary is accepting clients),
    not just for the TCP socket to be bound.
    """
    binary = request.config.getoption("--ardupilot-sitl")
    if binary is None:
        yield None
        return

    state = _start_ardupilot_process(request)
    yield state
    _stop_ardupilot_process(state)


def _start_ardupilot_process(request) -> dict:
    """
    Start a fresh ArduPilot SITL process for the configured
    --ardupilot-sitl/--ardupilot-model/--sitl-instance, killing any existing
    same-instance process first. Returns a state dict consumed by
    _stop_ardupilot_process() and mutated in place by restart_flight_stack()
    on a mid-session restart (see that fixture's docstring for why a restart
    capability exists — root CLAUDE.md's 2026-09-15 ArduPlane finding).
    Extracted from _manage_ardupilot_sitl's body so the exact same startup
    sequence can run either once (session start) or repeatedly (restart).
    """
    binary = request.config.getoption("--ardupilot-sitl")
    binary_path = Path(binary).expanduser()
    if not binary_path.exists():
        pytest.fail(f"--ardupilot-sitl: binary not found at {binary_path}")

    binary_name = binary_path.stem  # arducopter / arduplane / ardurover
    instance = request.config.getoption("--sitl-instance") or 0
    port = 5760 + 10 * instance

    model, defaults = _ardu_model_and_defaults(
        binary_path,
        request.config.getoption("--ardupilot-model"),
        request.config.getoption("--vehicle-type"),
    )

    # Kill any existing *same-instance* process by name to clear CLOSE-WAIT
    # connections — pgrep -x matches only the exact "comm" name (never our
    # own pytest process, whose argv happens to contain "--ardupilot-sitl");
    # -a additionally prints the full cmdline so _ardu_cmdline_instance can
    # filter by -IN. A different concurrently-running instance is left
    # alone — see root CLAUDE.md's CI section.
    # SIGKILL (not SIGTERM) so the process exits immediately and the OS
    # releases the port before we start a fresh instance, then poll until
    # the port is actually free — SIGTERM can take several seconds to flush.
    result = subprocess.run(["pgrep", "-xa", binary_name], capture_output=True, text=True)
    killed_any = False
    for line in result.stdout.splitlines():
        pid_str, _, cmdline = line.partition(" ")
        if _ardu_cmdline_instance(cmdline) == instance:
            log.warning(
                "Killing existing %s instance %d PID %s (CLOSE-WAIT cleanup)",
                binary_name, instance, pid_str,
            )
            subprocess.run(["kill", "-9", pid_str], check=False)
            killed_any = True
    if killed_any:
        # Wait until this instance's port is fully released (up to 10 s).
        for _ in range(20):
            time.sleep(0.5)
            result2 = subprocess.run(["ss", "-tlnp"], capture_output=True, text=True)
            if f":{port}" not in result2.stdout:
                break
        else:
            log.warning("Port %d still in use after 10 s — proceeding anyway", port)

    # Per-model (and, for a non-zero instance, per-instance too — so two
    # concurrent same-model instances don't collide on the same working
    # directory) working directory.
    safe_model = model.replace("+", "quad")
    instance_suffix = f"_i{instance}" if instance else ""
    work_dir = binary_path.parent / f"{binary_name.replace('ardu', '')}_{safe_model}_working{instance_suffix}"
    work_dir.mkdir(parents=True, exist_ok=True)

    home_lat = request.config.getoption("--home-lat")
    home_lon = request.config.getoption("--home-lon")
    home_alt = request.config.getoption("--home-alt")
    home = f"{home_lat},{home_lon},{home_alt},270"

    logs_dir = Path("logs")
    logs_dir.mkdir(exist_ok=True)
    log_path = logs_dir / f"{binary_name}_{safe_model}{instance_suffix}_{time.strftime('%Y%m%d_%H%M%S')}.log"
    log_fh = open(log_path, "w")

    cmd = [str(binary_path), "-S", f"-I{instance}", "--model", model, f"--home={home}"]
    if defaults:
        cmd.append("--defaults=" + ",".join(str(d) for d in defaults))
    log.info("Starting %s SITL (model=%s, instance=%d): %s", binary_name, model, instance, " ".join(cmd))
    proc = subprocess.Popen(cmd, cwd=str(work_dir), stdout=log_fh, stderr=log_fh)

    # Wait for "Waiting for connection" in the SITL log — this is the point at which
    # the MAVLink TCP stack is up and the binary will send heartbeats to new clients.
    ready = False
    for _ in range(30):
        time.sleep(1)
        if proc.poll() is not None:
            log_fh.flush()
            log_fh.close()
            pytest.fail(
                f"{binary_name} SITL exited unexpectedly. Check {log_path}"
            )
        try:
            if "Waiting for connection" in log_path.read_text():
                ready = True
                break
        except OSError:
            pass

    if not ready:
        proc.terminate()
        proc.wait()
        log_fh.close()
        pytest.fail(
            f"{binary_name} SITL (model={model}, instance={instance}) did not reach "
            f"'Waiting for connection' within 30 s. Check {log_path}"
        )

    log.info("%s SITL ready (model=%s, instance=%d, log: %s)", binary_name, model, instance, log_path)
    return {
        "proc": proc, "log_fh": log_fh, "log_path": log_path,
        "binary_name": binary_name, "model": model, "instance": instance,
    }


def _stop_ardupilot_process(state: dict) -> None:
    """Stop an ArduPilot SITL process previously started by _start_ardupilot_process()."""
    log.info(
        "Stopping %s SITL (model=%s, instance=%d)",
        state["binary_name"], state["model"], state["instance"],
    )
    proc = state["proc"]
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    state["log_fh"].close()


def _px4_cmdline_instance(cmdline: str) -> int:
    """Parse the `-i <N>` instance argument out of a px4 process's cmdline (default: 0)."""
    tokens = cmdline.split()
    for i, tok in enumerate(tokens):
        if tok == "-i" and i + 1 < len(tokens):
            try:
                return int(tokens[i + 1])
            except ValueError:
                pass
    return 0


@pytest.fixture(scope="session", autouse=True)
def _manage_px4_sitl(request):
    """
    Start and stop PX4 SITL when ``--px4-sitl`` is given.

    ``--px4-sitl`` must point to the PX4-Autopilot repository root containing
    ``build/px4_sitl_default/bin/px4``.  ``--px4-model`` sets PX4_SIM_MODEL
    (default: sihsim_quadx).  A ``gz_``-prefixed model (e.g. gz_standard_vtol)
    runs Gazebo instead of SIH — see _start_px4_process().  ``--sitl-instance`` (default 0) selects which
    PX4 SIH instance to run — see root CLAUDE.md's CI section for the full
    multi-instance design (port formula, working-directory isolation, etc.).

    SIH (Software In the Loop Hardware) is built into PX4 — no external
    simulator process is required.  MAVLink is broadcast on UDP port
    14540+instance by default.

    Kills any existing PX4 process for *this same instance* first (not other
    instances — see root CLAUDE.md CI section) to avoid port conflicts and
    stale state.  Readiness: waits for the mavlink module startup message in
    the PX4 log.
    """
    px4_dir = request.config.getoption("--px4-sitl")
    if px4_dir is None:
        yield None
        return

    state = _start_px4_process(request)
    yield state
    _stop_px4_process(state)


def _start_px4_process(request) -> dict:
    """
    Start a fresh PX4 SIH process for the configured --px4-model/
    --sitl-instance, killing any existing same-instance process first.
    Returns a state dict consumed by _stop_px4_process() and mutated in
    place by restart_flight_stack() on a mid-session restart (see that
    fixture's docstring for why a restart capability exists — root
    CLAUDE.md's 2026-09-15 ArduPlane finding). Extracted from
    _manage_px4_sitl's body so the exact same startup sequence can run
    either once (session start) or repeatedly (restart).
    """
    import os as _os

    px4_dir = request.config.getoption("--px4-sitl")
    px4_path = Path(px4_dir).expanduser()
    model = request.config.getoption("--px4-model") or "sihsim_quadx"
    instance = request.config.getoption("--sitl-instance") or 0
    # Gazebo (gz sim) instead of the built-in SIH simulator, selected purely by
    # model name — the same PX4_SIM_MODEL value `make px4_sitl <model>` uses.
    # PX4's own px4-rc.gzsim startup script does the rest: it launches a
    # headless gz server if no world is running yet (or attaches to one that
    # is, which is how a non-zero --sitl-instance joins instance 0's world),
    # waits for the world, and spawns the model. SIH stays the default.
    gz = model.startswith("gz_")

    build_dir = px4_path / "build/px4_sitl_default"
    binary = build_dir / "bin/px4"
    rcS = build_dir / "etc/init.d-posix/rcS"
    px4_bin_dir = build_dir / "bin"

    if not binary.exists():
        pytest.fail(f"--px4-sitl: px4 binary not found at {binary}")
    if gz and shutil.which("gz") is None:
        pytest.fail(f"--px4-model={model} needs Gazebo, but `gz` is not on PATH")

    # Kill any running PX4 *for this instance only* — pgrep -x (exact name
    # "px4", not a substring match) so this never matches our own pytest
    # process even though its argv contains "--px4-sitl"; -a additionally
    # prints the full cmdline so _px4_cmdline_instance can filter by -i N.
    # Other concurrently-running instances (different -i N) are left alone —
    # this is the one change from the pre-multi-instance blind kill-every-px4
    # behaviour, and is what makes concurrent instances viable at all.
    result = subprocess.run(["pgrep", "-xa", "px4"], capture_output=True, text=True)
    killed_any = False
    for line in result.stdout.splitlines():
        pid_str, _, cmdline = line.partition(" ")
        if _px4_cmdline_instance(cmdline) == instance:
            log.warning("Killing existing PX4 instance %d PID %s", instance, pid_str)
            subprocess.run(["kill", pid_str], check=False)
            killed_any = True
    if killed_any:
        time.sleep(2)
    if gz and instance == 0:
        # A gz server left over from an earlier (crashed/killed) run would be
        # re-used by px4-rc.gzsim, still holding that run's spawned model, and
        # the new spawn would collide with it. Instance 0 owns the world; a
        # non-zero instance deliberately joins it, so never kill it there.
        _kill_gz_servers(px4_path)

    # PX4 rootfs: the directory that contains etc/init.d-posix/airframes/.
    # SIH: the build directory itself. Gazebo: build/.../rootfs, the same
    # working directory `make px4_sitl gz_*` uses — px4-rc.gzsim sources
    # ./gz_env.sh (or ../gz_env.sh from an instance_N subdirectory) for the
    # model/world resource paths, and that file only exists there.
    rootfs = build_dir / "rootfs" if gz else build_dir

    logs_dir = Path("logs")
    logs_dir.mkdir(exist_ok=True)
    instance_suffix = f"_i{instance}" if instance else ""
    log_path = logs_dir / f"px4_{model}{instance_suffix}_{time.strftime('%Y%m%d_%H%M%S')}.log"
    log_fh = open(log_path, "w")

    env = _os.environ.copy()
    env["PX4_SIM_MODEL"] = model
    if gz:
        env["HEADLESS"] = "1"  # server only, no gz GUI
        env.setdefault("GZ_IP", "127.0.0.1")  # as `make px4_sitl gz_*` sets it
    # Ensure px4-alias.sh is findable via PATH
    env["PATH"] = str(px4_bin_dir) + ":" + env.get("PATH", "")

    if instance == 0 and gz:
        # rootfs/etc doesn't exist in a fresh build (only after something has
        # run PX4 there once). Passing the shared etc/ as the positional
        # <rootfs_directory> makes PX4's create_symlinks_if_needed() create
        # rootfs/etc -> build/etc itself — same mechanism as the non-zero
        # instance branch below.
        cmd = [str(binary), "-s", str(rcS), str(build_dir / "etc"), "-w", str(rootfs)]
    elif instance == 0:
        # Byte-identical to pre-multi-instance behaviour: runtime state
        # (dataman, eeprom, logs) lives directly in the shared build dir, no
        # -i flag passed (PX4 defaults to instance 0 either way).
        cmd = [str(binary), "-s", str(rcS), "-w", str(rootfs)]
    else:
        # A non-zero instance needs its OWN writable runtime-state directory
        # (dataman/eeprom/parameters), not the shared build dir, or two
        # concurrent instances would corrupt each other's state. Passing
        # `-w <instance_dir>` alone is not enough: PX4 resolves
        # etc/init.d-posix/airframes relative to -w's directory after
        # chdir'ing into it (verified empirically — passing the shared
        # rootfs only as the positional <rootfs_directory> argument does
        # NOT help, since the airframe lookup ignores it and reads
        # `${cwd}/etc/...` regardless). The fix PX4 itself provides:
        # passing the shared etc/ dir as the POSITIONAL <rootfs_directory>
        # argument makes PX4's own create_symlinks_if_needed() (main.cpp)
        # auto-symlink "etc" inside the instance's working dir back to the
        # shared one — confirmed by inspection of the resulting directory
        # (etc -> <shared>/etc, plus its own private dataman/eeprom/log/
        # parameters.bson). `-i N` on top of this is what makes the
        # rc.mavlink startup script (px4-rc.mavlink) derive every MAVLink
        # port as base+N — see the CI section in root CLAUDE.md for the
        # full port table this produces.
        etc_dir = build_dir / "etc"
        instance_dir = rootfs / f"instance_{instance}"
        instance_dir.mkdir(parents=True, exist_ok=True)
        cmd = [str(binary), "-i", str(instance), "-s", str(rcS), str(etc_dir), "-w", str(instance_dir)]

    log.info("Starting PX4 SITL (model=%s, instance=%d): %s", model, instance, " ".join(cmd))
    proc = subprocess.Popen(
        cmd, cwd=str(px4_path), stdout=log_fh, stderr=log_fh, env=env
    )

    # The pxh> prompt refresh loop PX4 enters after startup grows this file at
    # several GB/min (see 4b's comment above and root CLAUDE.md design decision
    # 4b) — normally harmless since only the first 64 KB is ever read back, but
    # a Tier 2 test session can run long enough for this to reach multiple GB
    # and put real memory pressure on the host (observed: ~4 GB in ~5 minutes
    # against a VTOL model, severe enough to trigger the sandbox's low-memory
    # kill; growth can start well before the "ready" wait loop below finishes,
    # so this watchdog starts immediately, not after readiness). Cap it with a
    # background thread rather than removing the file entirely, so
    # startup/module-init content (the only part ever useful for debugging
    # "PX4 SITL exited unexpectedly") survives.
    #
    # Read-head/truncate/reseek as three separate syscalls on the shared fd is
    # NOT safe: PX4's stdout fd is shared (via dup2, not a fresh open) with
    # this process's own log_fh, so both sides see the same file offset. In
    # the gap between truncating the file and repositioning that shared
    # offset, PX4 can slip in one more write at its old (pre-truncation, huge)
    # offset — which instantly recreates a multi-hundred-MB sparse file (an
    # observed failure mode: 117 truncation cycles ran in one minute yet the
    # file still ended up at 1.1 GB). SIGSTOP'ing PX4 for the handful of
    # syscalls below closes that window — verified race-free in isolation
    # (60 s, 0 unwanted growth) after this fix, vs. ballooning to 1.1 GB
    # without it.
    _PX4_LOG_CAP_BYTES = 5 * 1024 * 1024  # 5 MB — generous headroom over the 64 KB readiness read
    stop_cap_thread = threading.Event()

    def _cap_log_size() -> None:
        fd = log_fh.fileno()
        while not stop_cap_thread.is_set():
            try:
                if log_path.stat().st_size > _PX4_LOG_CAP_BYTES:
                    # log_fh is write-only (opened "w"), so the head must be read via
                    # a separate read-only fd — reading from log_fh's own fd fails
                    # with EBADF. This read doesn't touch the shared write offset,
                    # so it's safe to do before (or after) the SIGSTOP.
                    with open(log_path, "rb") as _rf:
                        head = _rf.read(65536)
                    trailer = b"\n... [truncated by test harness to cap pxh> prompt-loop growth] ...\n"
                    _os.kill(proc.pid, signal.SIGSTOP)
                    try:
                        _os.lseek(fd, 0, _os.SEEK_SET)
                        _os.write(fd, head)
                        _os.write(fd, trailer)
                        _os.ftruncate(fd, len(head) + len(trailer))
                        # Deliberately no further seek: the writes above already
                        # leave the shared offset exactly at the new EOF, which
                        # is where PX4's next write should continue from.
                    finally:
                        _os.kill(proc.pid, signal.SIGCONT)
            except Exception:
                # Deliberately broad, and logged rather than silently passed: a
                # narrower `except OSError: pass` here previously hid a real bug
                # (log_fh's write-only fd can't be read from — EBADF) across
                # several debugging cycles by masking every failed truncation
                # attempt as if nothing had gone wrong.
                log.exception("PX4 log-size watchdog failed on this cycle")
            stop_cap_thread.wait(0.5)

    cap_thread = threading.Thread(target=_cap_log_size, daemon=True)
    cap_thread.start()

    # Wait for "INFO  [mavlink]" (note: two spaces) — the mavlink module reporting
    # its UDP address, which means MAVLink is up and listening on port 14540.
    # Read only the first 64 KB of the log to avoid blocking on the growing
    # pxh> shell prompt loop that PX4 enters after startup (can reach GB/s).
    # Gazebo needs longer: gz server start + world load (px4-rc.gzsim itself
    # waits up to 30 s for the world) + model spawn, all before mavlink starts.
    ready_timeout_s = 120 if gz else 60
    ready = False
    for _ in range(ready_timeout_s):
        time.sleep(1)
        if proc.poll() is not None:
            stop_cap_thread.set()
            log_fh.flush()
            log_fh.close()
            pytest.fail(f"PX4 SITL exited unexpectedly. Check {log_path}")
        try:
            with open(log_path, "rb") as _f:
                head = _f.read(65536).decode("utf-8", errors="replace")
            if "INFO  [mavlink]" in head:
                ready = True
                break
        except OSError:
            pass

    if not ready:
        stop_cap_thread.set()
        proc.terminate()
        proc.wait()
        log_fh.close()
        pytest.fail(
            f"PX4 SITL (model={model}, instance={instance}) did not reach mavlink "
            f"startup within {ready_timeout_s} s. Check {log_path}"
        )

    # Additional stabilisation: SIH needs a moment after mavlink starts
    # before the EKF is converged enough for MAVSDK to declare is_connected.
    time.sleep(5)
    log.info("PX4 SITL ready (model=%s, instance=%d, log: %s)", model, instance, log_path)

    return {
        "proc": proc, "log_fh": log_fh, "log_path": log_path,
        "stop_cap_thread": stop_cap_thread, "cap_thread": cap_thread,
        "model": model, "instance": instance, "gz": gz, "px4_path": px4_path,
    }


def _gz_server_pids(px4_path: Path) -> list[int]:
    """
    PIDs of gz sim processes launched from this PX4 checkout — matched on the
    checkout's own worlds path in the cmdline (px4-rc.gzsim passes
    <px4>/Tools/simulation/gz/worlds/<world>.sdf), so an unrelated Gazebo
    session, or one from a different PX4 checkout, is never touched.
    """
    worlds = str(px4_path / "Tools/simulation/gz/worlds")
    result = subprocess.run(["pgrep", "-fa", "gz sim"], capture_output=True, text=True)
    pids = []
    for line in result.stdout.splitlines():
        pid_str, _, cmdline = line.partition(" ")
        if worlds in cmdline:
            pids.append(int(pid_str))
    return pids


def _kill_gz_servers(px4_path: Path) -> None:
    pids = _gz_server_pids(px4_path)
    for pid in pids:
        log.warning("Killing Gazebo server PID %d", pid)
        subprocess.run(["kill", str(pid)], check=False)
    for _ in range(20):
        if not _gz_server_pids(px4_path):
            return
        time.sleep(0.5)
    for pid in _gz_server_pids(px4_path):
        subprocess.run(["kill", "-9", str(pid)], check=False)


def _stop_px4_process(state: dict) -> None:
    """Stop a PX4 process previously started by _start_px4_process()."""
    state["stop_cap_thread"].set()
    state["cap_thread"].join(timeout=2)
    log.info("Stopping PX4 SITL (model=%s, instance=%d)", state["model"], state["instance"])
    proc = state["proc"]
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    state["log_fh"].close()
    # px4-rc.gzsim backgrounds the gz server, so it outlives PX4. Instance 0
    # owns (and started, or restarted) the world; a non-zero instance only
    # joined it, so leave it running for instance 0.
    if state.get("gz") and state["instance"] == 0:
        _kill_gz_servers(state["px4_path"])


@pytest.fixture(scope="session", autouse=True)
def _clear_px4_if_paired(request):
    """
    Kill any running *instance-0* PX4 SITL before starting paired-mode tests.

    PX4 uses sysid=1/compid=1 — the same identity as the mock drone mavsdk_server.
    If PX4 is running it will send heartbeats that reach the GCS loopback listener
    on port 14560, causing _wait_for_connection to see two sysid=1 peers and hang.

    In standalone mode (``--drone-address`` supplied) this fixture is a no-op;
    PX4 must be running to serve as the drone under test.

    Scoped to instance 0 only (not every running PX4): the paired loopback
    only conflicts with PX4's *default*-port broadcast, which is what
    instance 0 uses (see root CLAUDE.md's CI section for the per-instance
    port formula) — a concurrently-running non-zero instance uses a disjoint
    port set and is not a candidate for this conflict, so it is left alone.
    """
    if request.config.getoption("--drone-address") is not None:
        return

    result = subprocess.run(["pgrep", "-xa", "px4"], capture_output=True, text=True)
    if result.returncode != 0:
        return  # no PX4 running

    pids = []
    for line in result.stdout.splitlines():
        pid_str, _, cmdline = line.partition(" ")
        if _px4_cmdline_instance(cmdline) == 0:
            pids.append(pid_str)
    if not pids:
        return  # only non-zero-instance PX4 running — not a conflict, leave it be

    log.warning(
        "PX4 SITL instance 0 detected (PID %s) while starting paired-mode tests. "
        "PX4 uses sysid=1 which conflicts with the mock drone — killing it now.",
        ", ".join(pids),
    )
    for pid in pids:
        subprocess.run(["kill", pid], check=False)
    time.sleep(1.5)


async def _wait_for_connection(system: System, timeout_s: int) -> None:
    """
    Raise TimeoutError if the drone does not connect within *timeout_s*.

    The gRPC ``connection_state()`` stream does not respond to asyncio
    cancellation: wrapping the ``async for`` loop in ``asyncio.wait_for``
    causes ``wait_for`` to block indefinitely after the timeout fires because
    ``await task`` (post-cancel) never returns.

    Fix: run the gRPC subscription in a fire-and-forget background task and
    only wait on a plain asyncio.Event, which *does* respond to wait_for
    cancellation.  On success the event is set and we return; on timeout the
    event never fires and wait_for raises TimeoutError (cleanly, without
    waiting for the gRPC task to acknowledge cancellation).
    """
    connected = asyncio.Event()

    async def _subscription() -> None:
        try:
            async for state in system.core.connection_state():
                if state.is_connected:
                    connected.set()
                    return
        except asyncio.CancelledError:
            pass
        except Exception:
            pass  # never propagate from a background helper task

    task = asyncio.create_task(_subscription())
    try:
        await asyncio.wait_for(connected.wait(), timeout=timeout_s)
    except asyncio.TimeoutError:
        raise asyncio.TimeoutError(f"Could not connect within {timeout_s}s")
    finally:
        if not task.done():
            task.cancel()
            # Do NOT await task here — the gRPC stream won't acknowledge
            # cancellation until its next I/O event, which may be seconds away.
            # The task will be cleaned up by the event loop on its own schedule.


# ---------------------------------------------------------------------------
# Session-level autopilot identity probe
# ---------------------------------------------------------------------------


@pytest.fixture(scope="session", autouse=True)
def _autopilot_header(gcs_mavsdk_server, request):
    """
    Probe the flight stack identity once at session start and write a header
    block to the log.

    Standalone mode: connects to the already-running GCS mavsdk_server,
    reads one HEARTBEAT (autopilot type, vehicle type) and requests
    AUTOPILOT_VERSION (firmware version, git hash, capabilities).

    Paired (mock) mode: logs mock identity with N/A for version fields.

    Stores the probed info dict in ``request.config._autopilot_info``.
    Logs a suggested log filename of the form:
      <purpose>_<autopilot>_<vehicle>_<version>_<timestamp>.log
    """
    drone_address = request.config.getoption("--drone-address")
    vehicle_type_override = request.config.getoption("--vehicle-type")
    if drone_address is None:
        info: dict = {
            "autopilot": request.config.getoption("--autopilot") or "MOCK",
            "vehicle_type": vehicle_type_override or "MOCK",
            "firmware_version": "N/A",
            "git_hash": "N/A",
            "capabilities": 0,
        }
    else:
        timeout_s = int(request.config.getoption("--connection-timeout"))
        try:
            info = asyncio.run(_probe_autopilot_async(gcs_mavsdk_server, timeout_s))
        except RuntimeError as exc:
            # asyncio.run() raises RuntimeError if a loop is already running.
            # Fall back gracefully rather than aborting the session.
            log.warning("Autopilot probe skipped (event loop already running): %s", exc)
            info = {
                "autopilot": "UNKNOWN", "vehicle_type": "UNKNOWN",
                "firmware_version": "N/A", "git_hash": "N/A", "capabilities": 0,
            }
        except Exception as exc:
            log.warning("Autopilot probe failed: %s", exc)
            info = {
                "autopilot": "ERROR", "vehicle_type": "ERROR",
                "firmware_version": "N/A", "git_hash": "N/A", "capabilities": 0,
            }
        autopilot_override = request.config.getoption("--autopilot")
        if autopilot_override:
            info["autopilot"] = autopilot_override
        if vehicle_type_override:
            info["vehicle_type"] = vehicle_type_override

    request.config._autopilot_info = info
    header = _format_autopilot_header(info, drone_address)
    log.info("\n%s", header)
    suggested = suggest_log_filename(info, config=request.config)
    log.info("Suggested log filename: logs/%s", suggested)
    return info


# ---------------------------------------------------------------------------
# Mode-aware GCS (standalone against real drone, or paired against mock)
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def gcs_mavsdk_server(
    request,
    _manage_ardupilot_sitl,
    _manage_px4_sitl,
):
    """
    The session's GCS MAVSDK endpoint (tests/mavsdk_compat.Endpoint) —
    MAVSDK 4 runs in-process, so this is a connection, not a server process
    (the name is kept so fixture signatures didn't change).

    Standalone mode (``--drone-address`` given): its own endpoint connected to
    the real drone, as MAVSDK's GROUND_STATION identity (see
    tests/mavsdk_compat.GCS_SYSID for why not the old 255/1). Paired mode (no ``--drone-address``): the paired GCS
    endpoint talking to the MockFlightStack loopback.

    Depends on ``_manage_ardupilot_sitl`` and ``_manage_px4_sitl`` so the
    flight stack is guaranteed to be up before the GCS connects.
    """
    drone_address = request.config.getoption("--drone-address")
    if drone_address is None:
        yield request.getfixturevalue("paired_gcs_server")
        return

    endpoint = Endpoint("gcs", GCS_SYSID, GCS_COMPID, drone_address)
    ENDPOINTS["gcs"] = endpoint
    yield endpoint
    endpoint.close()
    log.info("Standalone GCS MAVSDK endpoint closed")


@pytest.fixture(scope="session")
def restart_flight_stack(request, _manage_px4_sitl, _manage_ardupilot_sitl, gcs_mavsdk_server):
    """
    Returns a synchronous callable that kills and restarts whichever SITL
    process is active (PX4 or ArduPilot, instance-aware) and reconnects the
    GCS endpoint, for Tier 2 flight tests to call instead of forcing a
    disarm when RTL doesn't reliably ground the vehicle.

    Why this exists (root CLAUDE.md's 2026-09-15 ArduPlane finding):
    ArduPlane's RTL_AUTOLAND defaults to disabled, and a plain RTL with no
    DO_LAND_START mission item just loiters at home forever rather than
    landing — and ArduPilot's own AP_Arming_Plane::disarm() correctly
    *refuses* a MAVLink-originated disarm while is_flying() (a real safety
    check, not a bug), so trying to force-disarm an aircraft that never
    landed silently fails, leaving it armed and airborne for the rest of the
    session. Every subsequent test then inherits that broken "already
    flying" state instead of "on the ground, ready to arm". A cold restart
    is the only fully reliable way to guarantee a clean state, and is
    typically no slower than the RTL-then-wait-out-a-doomed-timeout it
    replaces (SITL startup is itself a ~15-45s operation, comparable to or
    faster than RTL_LAND_TIMEOUT_S=120s of waiting for a landing that will
    never come).

    Also reopens the GCS MAVSDK endpoint: PX4's UDP connection self-heals
    once the new process starts broadcasting heartbeats again, but
    ArduPilot's TCP connection does not reliably redial a closed peer (see
    this file's own documented CLOSE-WAIT pitfalls) — reconnecting
    unconditionally is simpler and safer than branching on transport.

    No-op in mock/paired mode (nothing to restart).
    """
    drone_address = request.config.getoption("--drone-address")

    def _restart() -> None:
        if drone_address is None:
            return  # mock mode: nothing to restart

        if request.config.getoption("--px4-sitl") is not None:
            _stop_px4_process(_manage_px4_sitl)
            new_state = _start_px4_process(request)
            _manage_px4_sitl.clear()
            _manage_px4_sitl.update(new_state)
        elif request.config.getoption("--ardupilot-sitl") is not None:
            _stop_ardupilot_process(_manage_ardupilot_sitl)
            new_state = _start_ardupilot_process(request)
            _manage_ardupilot_sitl.clear()
            _manage_ardupilot_sitl.update(new_state)

        # Reconnect the GCS endpoint too: ArduPilot's TCP link doesn't reliably
        # redial a closed peer, so a fresh connection is simpler than branching
        # on transport.
        gcs_mavsdk_server.reopen()
        log.info(
            "Flight stack restarted (instance=%d)",
            request.config.getoption("--sitl-instance") or 0,
        )

    return _restart


@pytest.fixture
async def mock_stack(request):
    """
    Start MockFlightStack on the paired drone in paired mode.

    In standalone mode (``--drone-address`` given) this is a no-op; the real
    flight stack provides protocol handling.  In paired mode a fresh
    MockFlightStack is started for each test and cancelled on teardown.

    Tests that need to inspect or reconfigure the mock can request this
    fixture directly; ``gcs_system`` depends on it automatically.
    """
    drone_address = request.config.getoption("--drone-address")
    if drone_address is not None:
        yield None
        return

    system = await open_paired_drone()

    stack = MockFlightStack()
    task = asyncio.create_task(stack.run(system))
    # Give the subscriptions a moment to establish before the test starts.
    await asyncio.sleep(0.5)
    yield stack
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass


@pytest.fixture
async def gcs_system(gcs_mavsdk_server, mock_stack, request):
    """
    A fresh MAVSDK System (GCS side) for each client test.

    Works in both standalone mode (connects to real drone via
    ``--drone-address``) and paired mode (connects to MockFlightStack over
    loopback).
    """
    timeout_s = int(request.config.getoption("--connection-timeout"))
    system = await open_system(gcs_mavsdk_server, timeout_s)
    await _wait_for_connection(system, timeout_s)
    if request.config.getoption("--drone-address") is None:
        yield system  # mock mode: MockFlightStack streams nothing, nothing to watch
        return

    # Standalone: watch every message this test's data comes from
    # (tests/message_watcher.py) and, on ArduPilot, ask for them — it doesn't
    # send them unrequested. The autopilot type comes from the vehicle's own
    # HEARTBEAT (never --autopilot), cached on the endpoint. The request is
    # repeated every test, and again after a mid-test restart
    # (ensure_harness_streams, called from the flight helpers), since a
    # restart/reboot forgets it.
    required = {**HARNESS_MESSAGES, **getattr(request.module, "REQUIRED_MESSAGES", {})}
    watcher = MessageWatcher(system, required)
    await watcher.start()
    autopilot = getattr(gcs_mavsdk_server, "autopilot_type", None)
    if autopilot is None and await watcher.wait_for_heartbeat():
        autopilot = gcs_mavsdk_server.autopilot_type = watcher.autopilot
    if autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA:
        watcher.add(ARDUPILOT_MESSAGES)
    system.autopilot_type = autopilot
    system.watcher = watcher
    await ensure_harness_streams(system, force=True)
    request.node._message_watcher = watcher
    try:
        yield system
    finally:
        await watcher.stop()
        system.watcher = None
        missing = watcher.missing()
        if missing:
            log.warning("Message watcher: never received during this test: %s\n%s",
                        ", ".join(missing), watcher.report())


# ---------------------------------------------------------------------------
# Paired loopback (always available, independent of --drone-address)
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def paired_drone_server(request):
    """
    The paired-mode mock drone's MAVSDK endpoint (sysid 1 / compid 1, sending
    to the paired GCS over UDP loopback). MockFlightStack runs on top of it.
    Paired mode only — created lazily by gcs_mavsdk_server, so a standalone
    session never opens the loopback at all. The port is instance-offset
    (``--sitl-instance``) so concurrent sessions don't collide.
    """
    instance = request.config.getoption("--sitl-instance") or 0
    mavlink_port = GCS_MAVLINK_PORT + 10 * instance
    endpoint = Endpoint("paired_drone", 1, 1, f"udpout://127.0.0.1:{mavlink_port}")
    ENDPOINTS["paired_drone"] = endpoint
    yield endpoint
    endpoint.close()
    log.info("Paired drone MAVSDK endpoint closed (instance=%d)", instance)


@pytest.fixture(scope="session")
def paired_gcs_server(request, paired_drone_server):
    """
    The paired-mode GCS MAVSDK endpoint (GROUND_STATION identity, listening on
    the loopback). Created after the drone endpoint so the peer is already
    sending heartbeats when the GCS starts listening.
    """
    instance = request.config.getoption("--sitl-instance") or 0
    mavlink_port = GCS_MAVLINK_PORT + 10 * instance
    endpoint = Endpoint("paired_gcs", GCS_SYSID, GCS_COMPID, f"udpin://0.0.0.0:{mavlink_port}")
    ENDPOINTS["paired_gcs"] = endpoint
    yield endpoint
    endpoint.close()
    log.info("Paired GCS MAVSDK endpoint closed (instance=%d)", instance)


@pytest.fixture
async def paired_gcs_system(paired_gcs_server, request):
    """A MAVSDK System (GCS side) for each paired-mode test."""
    timeout_s = int(request.config.getoption("--connection-timeout"))
    system = await open_system(paired_gcs_server, timeout_s)
    await _wait_for_connection(system, timeout_s)
    yield system


@pytest.fixture
async def paired_drone_system(paired_drone_server, request):
    """A MAVSDK System (drone side — its view of the GCS) for each paired-mode test."""
    timeout_s = int(request.config.getoption("--connection-timeout"))
    system = await open_system(paired_drone_server, timeout_s)
    yield system


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    """
    Stash each phase's TestReport on the item (``item._report_setup`` /
    ``_report_call`` / ``_report_teardown``) so a fixture can read a test's own
    outcome (including whether it was skipped, xfailed, or xpassed) at
    teardown time. Currently consumed by tests/flight_helpers.py's Tier 2
    auto-recording fixture; generic and protocol-neutral, so any future
    reporting need can reuse it rather than adding another hook of this name
    (pytest allows only one implementation of a given hook to see the report
    unless chained via hookwrapper, which this already is).
    """
    outcome = yield
    rep = outcome.get_result()
    setattr(item, f"_report_{rep.when}", rep)
    watcher = getattr(item, "_message_watcher", None)
    if watcher is not None and rep.failed:
        # Which data-bearing messages actually arrived (tests/message_watcher.py).
        rep.sections.append(("message watcher", watcher.report()))


def pytest_terminal_summary(terminalreporter, exitstatus, config):
    """
    End-of-session param-coverage check (tests/report.py's param_coverage()):
    list every command report written this session that still has a param
    with no explicit verdict — supported/not supported/not applicable/
    untestable. General requirement: no param may end a run silently unknown.
    """
    from tests import report

    gaps = report.coverage_gaps()
    if not gaps:
        return
    terminalreporter.section("Param coverage gaps")
    for (protocol, cmd_name), params in sorted(gaps.items()):
        terminalreporter.write_line(
            f"{protocol} MAV_CMD_{cmd_name}: no verdict for {', '.join(params)} "
            "— needs an ACK result, an execution test, or a recorded untestable reason"
        )


def pytest_configure(config):
    """Tell tests/report.py whether this session has a real flight stack (see report.compat_fail)."""
    from tests import report

    report.MOCK_MODE = config.getoption("--drone-address") is None
