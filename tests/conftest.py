"""
Shared fixtures for all tests.

Event-loop strategy
-------------------
pytest-asyncio gives each test function its own asyncio event loop.  gRPC
channels (used by MAVSDK) are tied to the event loop that created them, so a
session-scoped *async* fixture that creates gRPC channels cannot be safely
shared across test functions.

Solution: start ``mavsdk_server`` once as a *synchronous* session fixture
(no event-loop ownership), then create a fresh ``System`` object per test.
Each System connects to the already-running server over gRPC, establishing
channels in the test's own event loop.  The MAVLink connection to the drone
is maintained by mavsdk_server for the whole session.

Connection strategy
-------------------
* ``--drone-address`` supplied → standalone mode: GCS mavsdk_server connects
  to that address (e.g. a PX4 SITL).  All client tests run against the real
  flight stack.
* ``--drone-address`` omitted  → paired mode: client tests run against
  MockFlightStack over loopback.  Server tests also run (they always use the
  paired loopback independently of this flag).

Fixture sets
------------
``gcs_mavsdk_server`` / ``gcs_system`` / ``mock_stack``
    Mode-aware GCS for client tests.
    Standalone: ``gcs_mavsdk_server`` starts a dedicated server pointing to
    ``--drone-address``; ``mock_stack`` is a no-op.
    Paired: ``gcs_mavsdk_server`` reuses ``paired_gcs_server``; ``mock_stack``
    starts MockFlightStack against the paired drone mavsdk_server.

``paired_gcs_server`` / ``paired_drone_server``
``paired_gcs_system`` / ``paired_drone_system``
    Loopback-only pair — always started, regardless of ``--drone-address``.
    Used by server tests and ``TestDeprecatedMessageHandling``.  Port 14560
    is used deliberately (not 14540) to avoid interference from PX4 SITL.

Ports
-----
  GCS_GRPC_PORT         = 50051  (gRPC — standalone GCS mavsdk_server)
  PAIRED_GCS_GRPC_PORT  = 50053  (gRPC — paired GCS mavsdk_server)
  DRONE_GRPC_PORT       = 50052  (gRPC — paired drone mavsdk_server)
  GCS_MAVLINK_PORT      = 14560  (MAVLink UDP — paired loopback;
                                  deliberately NOT 14540 to avoid interference
                                  from a concurrently running PX4 SITL)

Identity
--------
  GCS:   sysid=255, compid=1 — GCS must have compid=1 (autopilot-class) so the
         drone's mavsdk_server fires "System discovered" and starts its gRPC
         server.  If the GCS uses compid=190 (ground station), the drone's gRPC
         never starts.
  Drone: sysid=1,   compid=1
"""

import re
import subprocess
import logging
import asyncio
import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from mavsdk import System

from tests.mock_flight_stack import MockFlightStack

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


async def _probe_autopilot_async(grpc_port: int, timeout_s: int) -> dict:
    """
    Connect to an already-running mavsdk_server on *grpc_port* and probe the
    connected flight stack: reads one HEARTBEAT for autopilot/vehicle-type,
    then requests AUTOPILOT_VERSION for firmware version and git hash.

    Returns a dict with keys: autopilot, vehicle_type, firmware_version,
    git_hash, capabilities (int).
    """
    from mavsdk.mavlink_direct import MavlinkMessage

    system = System(mavsdk_server_address="localhost", port=grpc_port)
    await system.connect()

    try:
        await asyncio.wait_for(_wait_for_connection(system, timeout_s), timeout=timeout_s + 5)
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
        fw_type = _MAV_FIRMWARE_TYPE.get(int(ver.flight_sw_version_type), f"type{ver.flight_sw_version_type}")
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

GCS_GRPC_PORT = 50051
PAIRED_GCS_GRPC_PORT = 50053
DRONE_GRPC_PORT = 50052
GCS_MAVLINK_PORT = 14560  # not 14540 — avoids PX4 SITL interference


def _ardu_model_and_defaults(
    binary_name: str,
    model_override: str | None,
    vehicle_type: str | None,
) -> tuple[str, Path | None]:
    """
    Return (model_string, defaults_path_or_None) for an ArduPilot SITL binary.

    Model detection order:
      1. ``--ardupilot-model`` CLI option (explicit override)
      2. Inferred from binary name + ``--vehicle-type``

    Defaults detection:
      - copter → copter.parm
      - rover  → rover.parm
      - plane / quadplane → None (ROMFS-only; a .parm file crashes quadplane SITL)
    """
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

    ardu_defaults = (
        Path("~/github/ArduPilot/ardupilot/Tools/autotest/default_params").expanduser()
    )
    defaults_map: dict[str, str] = {
        "+": "copter.parm",
        "quad": "copter.parm",
        "rover": "rover.parm",
        "rover-skid": "rover-skid.parm",
    }
    defaults_name = defaults_map.get(model)
    defaults: Path | None = None
    if defaults_name:
        candidate = ardu_defaults / defaults_name
        defaults = candidate if candidate.exists() else None

    return model, defaults


@pytest.fixture(scope="session", autouse=True)
def _manage_ardupilot_sitl(request):
    """
    Start and stop any ArduPilot SITL binary when ``--ardupilot-sitl`` is given.

    Handles arducopter, arduplane (plane / quadplane), and ardurover.
    The SITL model is determined from ``--ardupilot-model`` (explicit) or
    inferred from the binary name and ``--vehicle-type`` (automatic).
    The defaults .parm file is selected automatically based on the model.

    Kills any existing process with the same binary name first to clear
    stale CLOSE-WAIT TCP connections — ArduPilot does not close its socket
    when clients disconnect abruptly, which prevents new connections from
    receiving MAVLink heartbeats.

    Readiness: waits for "Waiting for connection" in the SITL log (the point
    at which the MAVLink TCP stack is up and the binary is accepting clients),
    not just for the TCP socket to be bound.
    """
    binary = request.config.getoption("--ardupilot-sitl")
    if binary is None:
        yield
        return

    import os as _os

    binary_path = Path(binary).expanduser()
    if not binary_path.exists():
        pytest.fail(f"--ardupilot-sitl: binary not found at {binary_path}")

    binary_name = binary_path.stem  # arducopter / arduplane / ardurover

    model, defaults = _ardu_model_and_defaults(
        binary_name,
        request.config.getoption("--ardupilot-model"),
        request.config.getoption("--vehicle-type"),
    )

    # Kill any existing instance by name to clear CLOSE-WAIT connections.
    # Use SIGKILL (not SIGTERM) so the process exits immediately and the OS
    # releases port 5760 before we start a fresh instance.  Then poll until
    # the port is actually free — SIGTERM can take several seconds to flush.
    result = subprocess.run(["pgrep", "-x", binary_name], capture_output=True, text=True)
    if result.returncode == 0:
        for pid in result.stdout.split():
            log.warning(
                "Killing existing %s PID %s (CLOSE-WAIT cleanup)", binary_name, pid
            )
            subprocess.run(["kill", "-9", pid], check=False)
        # Wait until port 5760 is fully released (up to 10 s).
        for _ in range(20):
            time.sleep(0.5)
            result2 = subprocess.run(["ss", "-tlnp"], capture_output=True, text=True)
            if ":5760" not in result2.stdout:
                break
        else:
            log.warning("Port 5760 still in use after 10 s — proceeding anyway")

    # Per-model working directory so multiple vehicle types can coexist on disk.
    safe_model = model.replace("+", "quad")
    work_dir = binary_path.parent / f"{binary_name.replace('ardu', '')}_{safe_model}_working"
    work_dir.mkdir(parents=True, exist_ok=True)

    home_lat = request.config.getoption("--home-lat")
    home_lon = request.config.getoption("--home-lon")
    home_alt = request.config.getoption("--home-alt")
    home = f"{home_lat},{home_lon},{home_alt},270"

    logs_dir = Path("logs")
    logs_dir.mkdir(exist_ok=True)
    log_path = logs_dir / f"{binary_name}_{safe_model}_{time.strftime('%Y%m%d_%H%M%S')}.log"
    log_fh = open(log_path, "w")

    cmd = [str(binary_path), "-S", "-I0", "--model", model, f"--home={home}"]
    if defaults:
        cmd.append(f"--defaults={defaults}")
    log.info("Starting %s SITL (model=%s): %s", binary_name, model, " ".join(cmd))
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
            f"{binary_name} SITL (model={model}) did not reach 'Waiting for connection' "
            f"within 30 s. Check {log_path}"
        )

    log.info("%s SITL ready (model=%s, log: %s)", binary_name, model, log_path)
    yield

    log.info("Stopping %s SITL (model=%s)", binary_name, model)
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    log_fh.close()


@pytest.fixture(scope="session", autouse=True)
def _manage_px4_sitl(request):
    """
    Start and stop PX4 SIH SITL when ``--px4-sitl`` is given.

    ``--px4-sitl`` must point to the PX4-Autopilot repository root containing
    ``build/px4_sitl_default/bin/px4``.  ``--px4-model`` sets PX4_SIM_MODEL
    (default: sihsim_quadx).

    SIH (Software In the Loop Hardware) is built into PX4 — no external
    simulator process is required.  MAVLink is broadcast on UDP port 14540
    by default.

    Kills any existing ``px4`` process first to avoid port conflicts and stale
    state.  Readiness: waits for the mavlink module startup message in the
    PX4 log.
    """
    px4_dir = request.config.getoption("--px4-sitl")
    if px4_dir is None:
        yield
        return

    import os as _os

    px4_path = Path(px4_dir).expanduser()
    model = request.config.getoption("--px4-model") or "sihsim_quadx"

    binary = px4_path / "build/px4_sitl_default/bin/px4"
    rcS = px4_path / "build/px4_sitl_default/etc/init.d-posix/rcS"
    px4_bin_dir = px4_path / "build/px4_sitl_default/bin"

    if not binary.exists():
        pytest.fail(f"--px4-sitl: px4 binary not found at {binary}")

    # Kill any running PX4 to clear state and port bindings.
    result = subprocess.run(["pgrep", "-x", "px4"], capture_output=True, text=True)
    if result.returncode == 0:
        for pid in result.stdout.split():
            log.warning("Killing existing PX4 PID %s", pid)
            subprocess.run(["kill", pid], check=False)
        time.sleep(2)

    # PX4 rootfs: the directory that contains etc/init.d-posix/airframes/.
    # Using the build directory ensures PX4 finds its airframe files.
    # Runtime state (dataman, logs) is written there as well.
    rootfs = px4_path / "build/px4_sitl_default"

    logs_dir = Path("logs")
    logs_dir.mkdir(exist_ok=True)
    log_path = logs_dir / f"px4_{model}_{time.strftime('%Y%m%d_%H%M%S')}.log"
    log_fh = open(log_path, "w")

    env = _os.environ.copy()
    env["PX4_SIM_MODEL"] = model
    # Ensure px4-alias.sh is findable via PATH
    env["PATH"] = str(px4_bin_dir) + ":" + env.get("PATH", "")

    cmd = [str(binary), "-s", str(rcS), "-w", str(rootfs)]
    log.info("Starting PX4 SITL (model=%s): %s", model, " ".join(cmd))
    proc = subprocess.Popen(
        cmd, cwd=str(px4_path), stdout=log_fh, stderr=log_fh, env=env
    )

    # Wait for "INFO  [mavlink]" (note: two spaces) — the mavlink module reporting
    # its UDP address, which means MAVLink is up and listening on port 14540.
    # Read only the first 64 KB of the log to avoid blocking on the growing
    # pxh> shell prompt loop that PX4 enters after startup (can reach GB/s).
    ready = False
    for _ in range(60):
        time.sleep(1)
        if proc.poll() is not None:
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
        proc.terminate()
        proc.wait()
        log_fh.close()
        pytest.fail(
            f"PX4 SITL (model={model}) did not reach mavlink startup within 45 s. "
            f"Check {log_path}"
        )

    # Additional stabilisation: SIH needs a moment after mavlink starts
    # before the EKF is converged enough for MAVSDK to declare is_connected.
    time.sleep(5)
    log.info("PX4 SITL ready (model=%s, log: %s)", model, log_path)
    yield

    log.info("Stopping PX4 SITL (model=%s)", model)
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    log_fh.close()


@pytest.fixture(scope="session", autouse=True)
def _clear_stale_mavsdk_servers():
    """
    Kill ALL mavsdk_server processes before starting a new test session.

    Stale processes accumulate when a previous test run was interrupted (e.g. by
    pytest-timeout) without running session-level teardown.  Port-scanning with
    ``ss -tlnp`` only catches processes in the LISTEN state; a stale server that
    lost its port (or is in the process of reconnecting to the drone) does not
    appear in the listen table and can steal the MAVLink connection from the new
    server, causing ``_wait_for_connection`` to hang indefinitely.

    Killing every mavsdk_server process is safe because the test suite is the
    only thing that starts mavsdk_server on this machine.
    """
    result = subprocess.run(
        ["pgrep", "-x", "mavsdk_server"], capture_output=True, text=True
    )
    if result.returncode == 0:
        pids = result.stdout.split()
        log.warning(
            "Killing %d stale mavsdk_server process(es) (PIDs: %s) from a previous "
            "interrupted test session.",
            len(pids), ", ".join(pids),
        )
        for pid in pids:
            subprocess.run(["kill", "-9", pid], check=False)
        # Brief pause to let the kernel release the ports before we start new servers.
        time.sleep(1.0)


@pytest.fixture(scope="session", autouse=True)
def _clear_px4_if_paired(request):
    """
    Kill any running PX4 SITL before starting paired-mode tests.

    PX4 uses sysid=1/compid=1 — the same identity as the mock drone mavsdk_server.
    If PX4 is running it will send heartbeats that reach the GCS loopback listener
    on port 14560, causing _wait_for_connection to see two sysid=1 peers and hang.

    In standalone mode (``--drone-address`` supplied) this fixture is a no-op;
    PX4 must be running to serve as the drone under test.
    """
    if request.config.getoption("--drone-address") is not None:
        return

    result = subprocess.run(["pgrep", "-x", "px4"], capture_output=True, text=True)
    if result.returncode != 0:
        return  # no PX4 running

    pids = result.stdout.split()
    log.warning(
        "PX4 SITL detected (PID %s) while starting paired-mode tests. "
        "PX4 uses sysid=1 which conflicts with the mock drone — killing it now.",
        ", ".join(pids),
    )
    for pid in pids:
        subprocess.run(["kill", pid], check=False)
    time.sleep(1.5)


def _find_mavsdk_server() -> Path:
    """Return the path to the mavsdk_server binary bundled with MAVSDK-Python."""
    import mavsdk as _m
    candidate = Path(_m.__file__).parent / "bin" / "mavsdk_server"
    if candidate.exists():
        return candidate
    raise FileNotFoundError(f"mavsdk_server not found at {candidate}")


def _start_mavsdk_server(
    grpc_port: int,
    mavlink_url: str,
    sysid: int = 245,
    compid: int = 190,
) -> subprocess.Popen:
    binary = _find_mavsdk_server()
    cmd = [
        str(binary),
        "-p", str(grpc_port),
        "--sysid", str(sysid),
        "--compid", str(compid),
        mavlink_url,
    ]
    log.info("Starting mavsdk_server: %s", " ".join(cmd))
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    # Give the server a moment to bind its ports.
    time.sleep(1.5)
    if proc.poll() is not None:
        out = proc.stdout.read().decode(errors="replace")
        raise RuntimeError(f"mavsdk_server exited immediately:\n{out}")
    return proc


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
    paired_gcs_server,
    _manage_ardupilot_sitl,
    _manage_px4_sitl,
):
    """
    GCS mavsdk_server for client tests.

    Standalone mode (``--drone-address`` given): starts a dedicated
    mavsdk_server on port 50051 connected to the real drone.

    Paired mode (no ``--drone-address``): reuses the already-started
    ``paired_gcs_server`` (port 50053).  No skip — MockFlightStack provides
    the drone-side protocol handlers.

    Explicitly depends on ``_manage_ardupilot_sitl`` and ``_manage_px4_sitl``
    so the flight stack is guaranteed to be up before the GCS connects.
    """
    drone_address = request.config.getoption("--drone-address")
    if drone_address is None:
        yield PAIRED_GCS_GRPC_PORT
        return

    proc = _start_mavsdk_server(
        grpc_port=GCS_GRPC_PORT,
        mavlink_url=drone_address,
        sysid=255,
        compid=1,
    )
    yield GCS_GRPC_PORT
    proc.kill()
    proc.wait()
    log.info("Standalone GCS mavsdk_server stopped")


@pytest.fixture
async def mock_stack(request, paired_drone_server):
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

    system = System(mavsdk_server_address="localhost", port=paired_drone_server)
    await system.connect()

    stack = MockFlightStack()
    task = asyncio.create_task(stack.run(system))
    # Give the gRPC subscriptions a moment to establish before the test starts.
    # _wait_for_connection is intentionally skipped on the drone System: the
    # MAVLink session-level servers may already be connected (connection_state()
    # only fires on *changes*), so waiting for it would hang.
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
    system = System(mavsdk_server_address="localhost", port=gcs_mavsdk_server)
    await system.connect()
    await _wait_for_connection(system, timeout_s)
    yield system


# ---------------------------------------------------------------------------
# Paired loopback (always available, independent of --drone-address)
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def paired_drone_server(_clear_stale_mavsdk_servers):
    """
    Drone-side mavsdk_server for paired-mode tests.

    Always started on a fixed loopback port, regardless of whether
    ``--drone-address`` is supplied.

    Depends on ``_clear_stale_mavsdk_servers`` to guarantee the cleanup runs
    before we bind the gRPC ports — not after.
    """
    proc = _start_mavsdk_server(
        grpc_port=DRONE_GRPC_PORT,
        mavlink_url=f"udpout://127.0.0.1:{GCS_MAVLINK_PORT}",
        sysid=1,
        compid=1,
    )
    yield DRONE_GRPC_PORT
    proc.kill()
    proc.wait()
    log.info("Drone mavsdk_server stopped")


@pytest.fixture(scope="session")
def paired_gcs_server(paired_drone_server):
    """
    GCS-side mavsdk_server for paired-mode tests.

    Started after the drone server so the peer is already sending heartbeats
    when the GCS begins listening.
    """
    proc = _start_mavsdk_server(
        grpc_port=PAIRED_GCS_GRPC_PORT,
        mavlink_url=f"udpin://0.0.0.0:{GCS_MAVLINK_PORT}",
        sysid=255,
        compid=1,
    )
    yield PAIRED_GCS_GRPC_PORT
    proc.kill()
    proc.wait()
    log.info("Paired GCS mavsdk_server stopped")


@pytest.fixture
async def paired_gcs_system(paired_gcs_server, request):
    """
    A fresh MAVSDK System (GCS side) for each paired-mode test.
    """
    timeout_s = int(request.config.getoption("--connection-timeout"))
    system = System(mavsdk_server_address="localhost", port=paired_gcs_server)
    await system.connect()
    await _wait_for_connection(system, timeout_s)
    yield system


@pytest.fixture
async def paired_drone_system(paired_drone_server, request):
    """
    A fresh MAVSDK System (drone side) for each paired-mode test.
    """
    timeout_s = int(request.config.getoption("--connection-timeout"))
    system = System(mavsdk_server_address="localhost", port=paired_drone_server)
    await system.connect()
    await _wait_for_connection(system, timeout_s)
    yield system
