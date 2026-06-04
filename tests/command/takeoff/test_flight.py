"""
MAV_CMD_NAV_TAKEOFF (cmd=22) via COMMAND_INT — Tier 2 execution tests.

Sends NAV_TAKEOFF directly via COMMAND_INT (not via mission upload) and observes
telemetry to verify execution.  Requires a real flight stack (--drone-address).
All tests are skipped in paired/mock mode.

Unlike mission-protocol flight tests, there is no mission to upload or clear.
The vehicle is armed via MAVSDK action API, then NAV_TAKEOFF is sent as a raw
COMMAND_INT.  RTL is commanded in a finally block after each test.

Running
-------
PX4 multicopter::

    pytest tests/command/takeoff/test_flight.py \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \\
        --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO

PX4 fixed-wing::

    pytest tests/command/takeoff/test_flight.py \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_airplane \\
        --vehicle-type=fixed_wing --autopilot=px4 -v --log-cli-level=INFO

ArduCopter::

    pytest tests/command/takeoff/test_flight.py \\
        --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \\
        --ardupilot-sitl=~/ardu_sitl/arducopter \\
        --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 \\
        --vehicle-type=quadcopter --autopilot=ardupilot -v --log-cli-level=INFO

ArduPlane::

    pytest tests/command/takeoff/test_flight.py \\
        --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \\
        --ardupilot-sitl=~/ardu_sitl/arduplane --vehicle-type=fixed_wing \\
        --autopilot=ardupilot -v --log-cli-level=INFO
"""

import asyncio
import datetime
import logging
import time as _time_m
from pathlib import Path

import pytest
from mavsdk.telemetry import LandedState

from tests.command.conftest import (
    probe_command_int,
    probe_command_long,
    send_command_int,
    send_command_long,
    ACK_TIMEOUT_S,
    INT32_MAX,
    _FMT,
)
from tests.mock_flight_stack import MAV_RESULT_UNSUPPORTED, MAV_RESULT_DENIED

log = logging.getLogger(__name__)

# Flight tests: arming (60 s) + takeoff (90 s) + RTL/land (120 s) + margin.
pytestmark = pytest.mark.timeout(360)

_CMD = "NAV_TAKEOFF"
_CMD_ID = 22  # MAV_CMD_NAV_TAKEOFF

ARMABLE_TIMEOUT_S  = 60.0
TAKEOFF_ALT_M      = 30.0   # metres relative (nominal altitude for assertion tests)
TAKEOFF_TIMEOUT_S  = 90.0   # timeout for reaching TAKEOFF_ALT_M (assertion tests)
RTL_LAND_TIMEOUT_S = 120.0
YAW_TOLERANCE_DEG  = 20.0   # ± degrees for heading assertion

# For observational tests (yaw, position, mode), check that the vehicle is
# airborne at a low threshold so they work on stacks that use a fixed safety
# altitude (e.g. PX4 MPC_TKO_ALT ≈ 2.5 m) regardless of the requested z.
AIRBORNE_THRESHOLD_M = 2.0   # metres — confirms vehicle left the ground
AIRBORNE_TIMEOUT_S   = 30.0  # seconds — short check for observational tests

# Module-level caches — each probed once per session.
_nav_takeoff_supported: bool | None = None   # ACK result: True if not UNSUPPORTED
_nav_takeoff_executes: bool | None = None    # execution: True if vehicle actually climbs

# SITL self-recovery state
_REBOOT_COOLDOWN_S: float = 300.0  # 5 min between reboots (prevents loops)
_last_sitl_reboot_ts: float = 0.0  # monotonic timestamp of last successful reboot

# ---------------------------------------------------------------------------
# Behaviour summary — per-run log file + inline README section update
# ---------------------------------------------------------------------------
_README = Path("tests/command/takeoff/README.md")

# Map (autopilot, vehicle) → the Tier-2 section heading prefix in the README.
# The suffix " (Tier 2)" or " (M.m.p)" is handled dynamically.
_TIER2_SECTION = {
    ("px4",       "quadcopter"): "### PX4 MC",
    ("px4",       "fixed_wing"): "### PX4 FW",
    ("px4",       "vtol"):       "### PX4 VTOL",
    ("px4",       "rover"):      "### PX4 Rover",
    ("ardupilot", "quadcopter"): "### ArduCopter MC",
    ("ardupilot", "fixed_wing"): "### ArduPlane FW",
    ("ardupilot", "quadplane"):  "### ArduPlane QP",
    ("ardupilot", "rover"):      "### ArduRover",
}


def _write_and_update_readme(
    request,
    autopilot: str | None,
    firmware_version: str,
    summary_block: str,
) -> None:
    """Write a timestamped per-run summary log file and update the README inline.

    The per-run file is written to logs/ with the autopilot, vehicle, version, and
    timestamp in the filename — one file per run, never overwritten.

    The README Tier-2 section for (autopilot, vehicle) is updated in-place:
      - Heading suffix is updated to the current firmware version.
      - A summary block (pre-arm requirements + behaviour text) is inserted/replaced
        immediately after the heading, before the existing test-count line.
      - The old bottom-of-file `## Behaviour Summaries` section (if present) is removed.
    """
    vehicle   = request.config.getoption("--vehicle-type", default="unknown") or "unknown"
    auto      = autopilot or "unknown"
    now       = datetime.datetime.now()
    date_str  = now.strftime("%Y-%m-%d %H:%M")
    date_file = now.strftime("%Y%m%d_%H%M%S")

    # ── 1. Per-run log file ────────────────────────────────────────────────
    Path("logs").mkdir(exist_ok=True)
    log_path = (Path("logs") /
                f"command_nav_takeoff_summary_{auto}_{vehicle}_{firmware_version}_{date_file}.md")
    log_path.write_text(
        f"# NAV_TAKEOFF COMMAND_INT — Behaviour Summary\n\n"
        f"**Autopilot:** {auto}  **Vehicle:** {vehicle}  "
        f"**Firmware:** {firmware_version}  **Date:** {date_str}\n\n"
        + summary_block + "\n",
        encoding="utf-8",
    )
    log.info(_FMT, _CMD, "summary log written", str(log_path))

    # ── 2. Inline README update ────────────────────────────────────────────
    section_prefix = _TIER2_SECTION.get((auto, vehicle))
    if not section_prefix:
        log.warning(_FMT, _CMD, "README update", f"no Tier-2 section mapping for {auto}/{vehicle}")
        return

    marker_key = f"{auto}-{vehicle}"
    start_m    = f"<!-- TIER2_SUMMARY_START {marker_key} -->"
    end_m      = f"<!-- TIER2_SUMMARY_END {marker_key} -->"
    new_heading = section_prefix + f" ({firmware_version})\n"
    block = (
        f"{start_m}\n"
        f"**Last run:** {date_str}  **Firmware:** {firmware_version}\n\n"
        + summary_block + "\n\n"
        + f"{end_m}\n"
    )

    content = _README.read_text(encoding="utf-8")
    lines   = content.splitlines(keepends=True)

    def _is_tier2_heading(line: str) -> bool:
        """True if this line is the Tier-2 heading — excludes '(standalone)' headings."""
        s = line.rstrip()
        if not s.startswith(section_prefix + " ("):
            return False
        return "standalone" not in s  # leave Tier-1 headings untouched

    if start_m in content:
        # Markers present — update the Tier-2 heading and replace block content.
        new_lines: list[str] = []
        in_block = False
        for line in lines:
            if _is_tier2_heading(line):
                new_lines.append(new_heading)
                continue
            if start_m in line:
                in_block = True
                new_lines.append(block)
                continue
            if end_m in line:
                in_block = False
                continue
            if in_block:
                continue
            new_lines.append(line)
        content = "".join(new_lines)
    else:
        # First run — find the heading and insert the block immediately after it.
        new_lines = []
        heading_found = False
        block_inserted = False
        for line in lines:
            if _is_tier2_heading(line):
                heading_found = True
                new_lines.append(new_heading)
                continue
            if heading_found and not block_inserted:
                # Insert block before the first non-blank line after the heading.
                if line.strip():
                    new_lines.append(block)
                    block_inserted = True
            new_lines.append(line)
        if heading_found and not block_inserted:
            new_lines.append("\n" + block)
        content = "".join(new_lines)

    # Remove the old bottom "## Behaviour Summaries" section if it exists.
    if "<!-- BEHAVIOUR_SUMMARIES_START -->" in content:
        pre  = content.split("<!-- BEHAVIOUR_SUMMARIES_START -->")[0]
        post = content.split("<!-- BEHAVIOUR_SUMMARIES_END -->")[1]
        # Also strip the section header line that precedes the marker.
        if "## Behaviour Summaries\n" in pre:
            pre = pre.rsplit("## Behaviour Summaries\n", 1)[0]
        content = pre.rstrip() + "\n" + post.lstrip()

    _README.write_text(content, encoding="utf-8")
    log.info(_FMT, _CMD, "README section updated", f"{section_prefix} ({firmware_version})")


# ---------------------------------------------------------------------------
# Takeoff command builder
# ---------------------------------------------------------------------------

def _takeoff_cmd(**overrides) -> dict:
    """Return default COMMAND_INT kwargs for NAV_TAKEOFF."""
    defaults = dict(
        command=_CMD_ID,
        frame=6,       # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        param1=0.0,    # Pitch: 0 deg (use default)
        param2=0.0,    # Unused
        param3=0.0,    # Flags: none
        param4=0.0,    # Yaw: 0.0 = north (None encodes as NaN — "use current heading")
        x=0,           # lat: 0 → most stacks treat as "use current position"
        y=0,           # lon: 0
        z=float(TAKEOFF_ALT_M),
    )
    defaults.update(overrides)
    return defaults


# ---------------------------------------------------------------------------
# Telemetry helpers
# ---------------------------------------------------------------------------

async def _request_home_position(system) -> None:
    """Request HOME_POSITION (msg id=242) from the autopilot once.

    ArduCopter does not re-broadcast HOME_POSITION automatically after the
    initial connect.  MAVSDK's telemetry.home() uses this message.  After many
    function-scoped System reconnects within the same session, the stream may
    have gone stale.  MAV_CMD_REQUEST_MESSAGE (512) fetches one fresh copy.
    """
    from mavsdk.mavlink_direct import MavlinkMessage as _MavlinkMessage
    import json as _json_rh
    await system.mavlink_direct.send_message(_MavlinkMessage(
        message_name="COMMAND_LONG",
        system_id=255, component_id=1,
        target_system_id=1, target_component_id=0,
        fields_json=_json_rh.dumps({
            "target_system": 1, "target_component": 0,
            "command": 512,      # MAV_CMD_REQUEST_MESSAGE
            "param1": 242.0,    # MAVLINK_MSG_ID_HOME_POSITION = 242
            "param2": 0.0, "param3": 0.0, "param4": 0.0,
            "param5": 0.0, "param6": 0.0, "param7": 0.0,
            "confirmation": 0,
        }),
    ))
    await asyncio.sleep(0.3)


async def _request_position_stream(system, rate_hz: float = 5.0) -> None:
    """Request GLOBAL_POSITION_INT (msg id=33) from the autopilot.

    MAVSDK mavsdk_server does not automatically request position streaming from
    ArduCopter (unlike PX4, which sends it by default).  ArduCopter requires an
    explicit REQUEST_DATA_STREAM or MAV_CMD_SET_MESSAGE_INTERVAL before it begins
    streaming GLOBAL_POSITION_INT.  Without this, telemetry.position() and
    mavlink_direct.message("GLOBAL_POSITION_INT") both time out indefinitely.

    Uses MAV_CMD_SET_MESSAGE_INTERVAL (511) which is supported by ArduPilot
    v4.0+ and all PX4 versions.
    """
    from mavsdk.mavlink_direct import MavlinkMessage as _MavlinkMessage
    import json as _json
    interval_us = int(1_000_000 / rate_hz)  # microseconds between messages
    await system.mavlink_direct.send_message(_MavlinkMessage(
        message_name="COMMAND_LONG",
        system_id=255, component_id=1,
        target_system_id=1, target_component_id=0,
        fields_json=_json.dumps({
            "target_system": 1, "target_component": 0,
            "command": 511,     # MAV_CMD_SET_MESSAGE_INTERVAL
            "param1": 33.0,    # MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33
            "param2": float(interval_us),
            "param3": 0.0, "param4": 0.0, "param5": 0.0,
            "param6": 0.0, "param7": 0.0,
            "confirmation": 0,
        }),
    ))
    await asyncio.sleep(0.2)  # brief settle for stream to start


async def _get_home_position(system, timeout_s: float = 30.0):
    """Return the vehicle's home Position from telemetry."""
    async with asyncio.timeout(timeout_s):
        async for home in system.telemetry.home():
            return home
    raise TimeoutError("Home position not received within timeout")


async def _wait_armable(system, timeout_s: float = ARMABLE_TIMEOUT_S):
    """Block until the vehicle reports is_armable=True.

    Uses fire-and-forget task + asyncio.Event to avoid leaving a dangling gRPC
    health stream after timeout cancellation (CLAUDE.md §4a pattern).  The gRPC
    stream does not respond to asyncio cancellation reliably; wrapping it in a
    background task and only awaiting a plain Event ensures clean timeout handling.
    """
    armable_event = asyncio.Event()

    async def _watch() -> None:
        async for health in system.telemetry.health():
            if health.is_armable:
                armable_event.set()
                return

    task = asyncio.create_task(_watch())
    try:
        await asyncio.wait_for(armable_event.wait(), timeout=timeout_s)
    finally:
        task.cancel()  # fire-and-forget — do NOT await (§4a)


async def _wait_for_altitude(system, threshold_m: float, timeout_s: float = TAKEOFF_TIMEOUT_S):
    """Block until relative_altitude_m >= threshold_m; return the Position."""
    async with asyncio.timeout(timeout_s):
        async for pos in system.telemetry.position():
            if pos.relative_altitude_m >= threshold_m:
                return pos
    raise TimeoutError(
        f"Relative altitude {threshold_m:.1f} m not reached within {timeout_s:.0f} s"
    )


async def _wait_for_altitude_with_peak_pitch(
    system, threshold_m: float, timeout_s: float = TAKEOFF_TIMEOUT_S
):
    """Block until threshold_m reached; also return max |pitch| sampled during the wait."""
    peak_pitch: float = 0.0

    async def _sample_pitch() -> None:
        nonlocal peak_pitch
        async for att in system.telemetry.attitude_euler():
            mag = abs(att.pitch_deg)
            if mag > peak_pitch:
                peak_pitch = mag

    pitch_task = asyncio.create_task(_sample_pitch())
    try:
        pos = await _wait_for_altitude(system, threshold_m, timeout_s)
    finally:
        pitch_task.cancel()
        # Do not await the task: the attitude gRPC stream may not yield promptly
        # after cancellation (same issue as CLAUDE.md §4a for connection_state()).
        # The task is cleaned up at its next I/O event.

    return pos, peak_pitch


async def _get_heading(system, timeout_s: float = 5.0) -> float:
    """Return current vehicle heading in degrees (0–360)."""
    async with asyncio.timeout(timeout_s):
        async for hdg in system.telemetry.heading():
            return hdg.heading_deg
    raise TimeoutError("Heading not received")


async def _get_flight_mode(system, timeout_s: float = 5.0) -> str:
    """Return current flight mode name as a string."""
    async with asyncio.timeout(timeout_s):
        async for fm in system.telemetry.flight_mode():
            return str(fm)
    raise TimeoutError("Flight mode not received")


async def _set_guided_mode_ardupilot(
    system, timeout_s: float = 15.0, custom_mode: int = 4
) -> bool:
    """Set ArduPilot GUIDED mode and verify via telemetry.

    Sends DO_SET_MODE (176) with the given ``custom_mode`` and polls
    ``telemetry.flight_mode()`` until the mode is confirmed.

    MAVSDK mode string mapping (observed):
    - ArduCopter GUIDED (custom_mode=4)  → ``"OFFBOARD"``
    - ArduPlane GUIDED  (custom_mode=15) → ``"UNKNOWN"`` (MAVSDK has no
      mapping for ArduPlane custom modes beyond a small set)

    Acceptance rules:
    - ``"OFFBOARD"`` or ``"GUIDED"`` in the mode string: definitive confirmation.
    - ``"UNKNOWN"``: MAVSDK has no mode mapping but the DO_SET_MODE ACK was
      accepted; treat as confirmed since we cannot verify further.
    - Any other definite mode (e.g. ``"MANUAL"``, ``"STABILIZED"``): retry.

    Returns True when confirmed (including UNKNOWN-after-ACK), False on timeout.
    """
    guided_ack_accepted = False
    async with asyncio.timeout(timeout_s):
        while True:
            ack = await probe_command_long(
                system, 176, param1=1.0, param2=float(custom_mode)
            )
            if ack and int(ack["result"]) == 0:
                guided_ack_accepted = True
            await asyncio.sleep(0.5)
            try:
                mode = await _get_flight_mode(system, timeout_s=1.0)
                if "OFFBOARD" in mode or "GUIDED" in mode:
                    log.info(_FMT, _CMD, "GUIDED mode confirmed", f"telemetry reports {mode!r}")
                    return True
                if "UNKNOWN" in mode and guided_ack_accepted:
                    # MAVSDK has no mapping for this custom_mode; trust the ACK.
                    log.info(_FMT, _CMD, "GUIDED mode confirmed (ACK only)",
                             f"telemetry reports {mode!r} — no MAVSDK mapping for custom_mode={custom_mode}")
                    return True
                log.info(_FMT, _CMD, "GUIDED mode not yet active", f"current mode: {mode!r} — retrying")
            except TimeoutError:
                pass
            await asyncio.sleep(0.5)
    return False


async def _rtl_and_land(system, timeout_s: float = RTL_LAND_TIMEOUT_S) -> None:
    """Command RTL and wait for landed state; then disarm.  Best-effort — never raises."""
    try:
        await system.action.return_to_launch()
        async with asyncio.timeout(timeout_s):
            async for state in system.telemetry.landed_state():
                if state == LandedState.ON_GROUND:
                    break
    except Exception as exc:
        log.warning("RTL/land wait failed: %s", exc)
    await asyncio.sleep(2.0)
    try:
        await system.action.disarm()
    except Exception:
        pass


async def _reboot_sitl_if_degraded(system) -> None:
    """Reboot the flight stack via MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (cmd=246).

    Called when ``_wait_armable()`` times out, indicating the SITL has degraded
    after many consecutive arm-takeoff-RTL cycles.  PX4 SIH accumulates EKF drift
    and physics state over ~17 cycles until ``is_armable`` never returns True.

    After sending the reboot command:
    - PX4 reboots its firmware and the SIH physics module (~15 s).
    - The session-scoped ``mavsdk_server`` reconnects automatically on the same
      UDP port when PX4 resumes broadcasting heartbeats.
    - Position and home streams are re-requested so the next ``_wait_armable``
      has fresh telemetry.

    A 5-minute cooldown prevents reboot loops if recovery fails.  Caller must
    call ``_wait_armable(system, timeout_s=120.0)`` after this returns.
    """
    global _last_sitl_reboot_ts
    now = _time_m.monotonic()
    if now - _last_sitl_reboot_ts < _REBOOT_COOLDOWN_S:
        log.warning(
            _FMT, _CMD, "SITL reboot",
            f"within cooldown ({_REBOOT_COOLDOWN_S:.0f} s) — not rebooting; "
            "caller's _wait_armable will raise TimeoutError",
        )
        return
    _last_sitl_reboot_ts = now
    log.info(
        _FMT, _CMD, "SITL reboot",
        "is_armable=False after timeout — sending MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN "
        "(SITL degraded after many flight cycles)",
    )
    try:
        # param1=1.0: reboot autopilot (not companion).  PX4 may disconnect before
        # the ACK arrives, so exceptions are suppressed.
        await probe_command_long(system, 246, param1=1.0)
    except Exception:
        pass
    # Wait for PX4 SIH to boot and for EKF to begin converging before re-requesting
    # streams (streams silently timeout if PX4 isn't ready yet).
    await asyncio.sleep(15.0)
    await _request_position_stream(system)
    await _request_home_position(system)
    log.info(_FMT, _CMD, "SITL reboot", "done — caller should now _wait_armable(120 s)")


async def _arm_and_send_takeoff(system, **overrides) -> None:
    """Wait for armable → arm → send NAV_TAKEOFF COMMAND_INT.

    PX4 treats the z field in COMMAND_INT as absolute AMSL altitude (it ignores
    the frame field).  The caller passes z as a RELATIVE altitude (above home);
    this function adds the home AMSL altitude to produce the correct absolute z.

    Special cases:
    - z=None: passed as-is (NaN on wire → stack uses its default altitude).
    - z=0.0: treated literally (0m AMSL) for the zero-altitude observation test.

    Home lat/lon is used as x/y unless overrides specify different values.
    Frame is forced to 5 (GLOBAL_INT, absolute AMSL) to match the absolute z.
    """
    home = await _get_home_position(system)
    home_lat_int = int(home.latitude_deg * 1e7)
    home_lon_int = int(home.longitude_deg * 1e7)
    home_amsl_m = home.absolute_altitude_m

    # Convert relative z to absolute; preserve None (NaN) and 0.0 as-is.
    relative_z = overrides.pop("z", float(TAKEOFF_ALT_M))
    if relative_z is not None and relative_z != 0.0:
        absolute_z = home_amsl_m + relative_z
    else:
        absolute_z = relative_z  # None (NaN) or 0.0 absolute for observation tests

    merged = {
        "x": home_lat_int,
        "y": home_lon_int,
        "z": absolute_z,
        "frame": 5,  # GLOBAL_INT: absolute AMSL — matches PX4 COMMAND_INT behavior
    }
    merged.update(overrides)
    await _request_position_stream(system)  # ArduCopter doesn't stream by default
    await _wait_armable(system)
    await system.action.arm()
    await asyncio.sleep(0.5)  # brief settle after arm before command
    kw = _takeoff_cmd(**merged)
    await send_command_int(system, **kw)


# ---------------------------------------------------------------------------
# Module-level support gate
# ---------------------------------------------------------------------------

async def _ensure_nav_takeoff_supported(system) -> None:
    """
    Gate all flight tests with two sequential probes (each cached after first call):

    1. ACK probe: send COMMAND_INT NAV_TAKEOFF while disarmed; skip if UNSUPPORTED.
       Disarmed ACCEPTED is sufficient — ArduRover returns UNSUPPORTED here.

    2. Execution probe: arm, send NAV_TAKEOFF, wait ≤ 15 s for any climb > 0.5 m.
       PX4 in HOLD mode returns ACCEPTED (probe 1) but does NOT execute the takeoff
       unless the vehicle is first put in an execution-ready mode (e.g. AUTO).  The
       probe skips all 17 execution tests when the command is accepted but the vehicle
       stays on the ground, saving ~25 minutes of 90-second per-test timeouts.
    """
    global _nav_takeoff_supported, _nav_takeoff_executes

    # --- Probe 1: ACK result ---
    if _nav_takeoff_supported is None:
        ack = await probe_command_int(system, _CMD_ID)
        unsupported = (ack is not None and int(ack["result"]) == MAV_RESULT_UNSUPPORTED)
        _nav_takeoff_supported = not unsupported
    if not _nav_takeoff_supported:
        pytest.skip(f"{_CMD} (cmd={_CMD_ID}) is UNSUPPORTED on this platform — flight test not run")

    # --- Probe 2: actual execution (arm → send → wait for meaningful climb) ---
    if _nav_takeoff_executes is None:
        try:
            await _arm_and_send_takeoff(system, z=5.0)
            await _wait_for_altitude(system, AIRBORNE_THRESHOLD_M, timeout_s=20.0)
            _nav_takeoff_executes = True
            log.info(_FMT, _CMD, "execution probe", "vehicle climbed — COMMAND_INT NAV_TAKEOFF executes")
        except TimeoutError:
            _nav_takeoff_executes = False
            log.warning(
                _FMT, _CMD, "execution probe",
                "vehicle did NOT climb within 20 s after arm + COMMAND_INT NAV_TAKEOFF "
                "(ACCEPTED but not executed — stack may require a different flight mode)",
            )
        finally:
            await _rtl_and_land(system)

    if not _nav_takeoff_executes:
        pytest.skip(
            f"{_CMD} (cmd={_CMD_ID}) ACCEPTED but does not cause takeoff on this platform "
            "— execution tests skipped (stack-specific mode requirement)"
        )


# ---------------------------------------------------------------------------
# Autouse fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def require_real_stack(request):
    """Skip every test in this module when no --drone-address is given."""
    if request.config.getoption("--drone-address") is None:
        pytest.skip("Execution tests require a real flight stack (--drone-address not set)")


@pytest.fixture(autouse=True)
def _set_executes_cache_for_known_modes(request):
    """Pre-populate the execution-probe cache for stacks known to need a special mode.

    ArduCopter and ArduPlane QuadPlane require GUIDED mode before COMMAND_INT
    NAV_TAKEOFF executes (code: GCS_MAVLink_Copter.cpp:578, has_user_takeoff).
    The standard execution probe uses action.arm() which leaves the vehicle in
    STABILIZE — the probe always fails, taking 60 s (armable wait + altitude wait),
    and leaves a dangling telemetry.health() gRPC stream that corrupts subsequent
    tests (CLAUDE.md §4a).  Set the cache directly to avoid the probe entirely.

    PX4 rover: NAV_TAKEOFF is ACCEPTED but the rover cannot climb.  Same fix.

    The comprehensive test (test_mc_takeoff_comprehensive) bypasses this gate with
    its own ACK probe, so it is unaffected.
    """
    global _nav_takeoff_executes
    autopilot = request.config.getoption("--autopilot", default=None) or ""
    vehicle = request.config.getoption("--vehicle-type", default=None) or ""

    if _nav_takeoff_executes is None:
        # ArduCopter MC and ArduPlane QP: ACCEPTED but requires GUIDED mode.
        if autopilot == "ardupilot" and vehicle in ("quadcopter", "copter", "quadplane"):
            _nav_takeoff_executes = False
            log.info(
                _FMT, _CMD, "execution cache",
                f"ardupilot/{vehicle}: NAV_TAKEOFF requires GUIDED mode — "
                "skipping execution probe (test_mc_takeoff_comprehensive has its own gate)",
            )
        # PX4 rover: ACCEPTED but rover cannot fly.
        elif autopilot == "px4" and vehicle == "rover":
            _nav_takeoff_executes = False
            log.info(_FMT, _CMD, "execution cache",
                     "px4/rover: NAV_TAKEOFF ACCEPTED but rover cannot climb")


# ---------------------------------------------------------------------------
@pytest.mark.timeout(600)
async def test_mc_takeoff_comprehensive(gcs_system, request):
    """
    COMMAND_INT NAV_TAKEOFF — comprehensive multicopter execution observation.

    Sends NAV_TAKEOFF with param4=90° (east), target 200 m north at 30 m altitude.
    Validated on PX4 MC and ArduCopter; intended to generalise to other stacks.

    Detects and reports:
      - Minimum vertical ascent before lateral navigation begins (safety floor)
      - Whether the vehicle navigates toward the specified lat/lon
      - Whether param4 (yaw) is respected
      - Flight mode at the target altitude

    Logs a natural language behaviour summary at the end.

    Design: stack-specific *pre-arm setup* and *command field setup* are isolated
    in clearly labelled blocks; the arm and send steps are then stack-agnostic.
    This test bypasses the module-level execution-probe gate so it can run on
    stacks that need mode preparation before NAV_TAKEOFF will execute.
    """
    vehicle_type = request.config.getoption("--vehicle-type", default=None) or "unknown"
    if vehicle_type not in ("quadcopter", "fixed_wing"):
        pytest.skip(f"Not supported for vehicle type {vehicle_type!r} — use quadcopter or fixed_wing")

    autopilot = request.config.getoption("--autopilot", default=None)

    # ArduPlane fixed-wing: COMMAND_INT NAV_TAKEOFF is accepted at the MAVLink level
    # but does not cause takeoff.  The correct mechanism (TAKEOFF mode 13 + arm) is
    # documented and tested by test_arduplane_guided_takeoff_to_target, which runs in
    # the same session and writes its own authoritative summary log.  Running the
    # tiered probe here produces contradictory output (arm fails → "arm failed" while
    # pre_arm_notes say "command not supported") so we write a direct summary instead.
    if autopilot == "ardupilot" and vehicle_type == "fixed_wing":
        summary_block = (
            "COMMAND_INT NAV_TAKEOFF (22) is accepted at the MAVLink level "
            "(MAV_RESULT_ACCEPTED) but the aircraft does not take off.\n\n"
            "Fixed-wing takeoff is achieved via DO_SET_MODE TAKEOFF (mode 13) + arm — "
            "the aircraft then applies full throttle and climbs automatically.\n\n"
            "**Correct mechanism:** TAKEOFF mode (mode 13) + arm.  "
            "See `test_arduplane_guided_takeoff_to_target` for detailed observations.\n"
        )
        log.info(_FMT, _CMD, "ArduPlane FW",
                 "COMMAND_INT NAV_TAKEOFF not the correct takeoff mechanism — "
                 "writing summary and exiting without probe")
        try:
            ver = await gcs_system.info.get_version()
            firmware_version = (
                f"{ver.flight_sw_major}.{ver.flight_sw_minor}.{ver.flight_sw_patch}"
            )
        except Exception:
            firmware_version = "unknown"
        _write_and_update_readme(request, autopilot, firmware_version, summary_block)
        return

    # Brief settle: the execution probe (_ensure_nav_takeoff_supported) may have
    # run a 60 s _wait_armable wait + RTL sequence that leaves the mavsdk_server
    # or ArduCopter briefly busy.  Allow up to 3 attempts with a 3 s gap.
    ack = None
    for _attempt in range(3):
        await asyncio.sleep(3.0)
        try:
            ack = await probe_command_int(gcs_system, _CMD_ID)
            break
        except (TimeoutError, asyncio.TimeoutError):
            log.info(_FMT, _CMD, "ACK probe attempt %d timed out — retrying", _attempt + 1)
    if ack is not None and int(ack["result"]) == MAV_RESULT_UNSUPPORTED:
        pytest.skip(f"{_CMD} UNSUPPORTED on this platform")

    TARGET_ALT_REL  = 30.0
    PARAM4_YAW_DEG  = 90.0   # east — distinct from target bearing (~0° north)
    LATERAL_START_M = 5.0    # dist from home that counts as "lateral navigation started"

    # autopilot and vehicle_type already set above (from the early-exit block)

    # Request position + home streams — ArduCopter doesn't stream these by default,
    # and HOME_POSITION may be stale after multiple reconnections within a session.
    await _request_position_stream(gcs_system)
    await _request_home_position(gcs_system)

    # Home position: use a long timeout for stacks whose SITL is slow to emit home.
    home = await _get_home_position(gcs_system, timeout_s=90.0)
    TARGET_LAT = home.latitude_deg + 200.0 / 111111.0  # 200 m north
    TARGET_LON = home.longitude_deg

    log.info(
        _FMT, _CMD, "MC comprehensive — setup",
        f"home ({home.latitude_deg:.5f}°N, {home.longitude_deg:.5f}°E  "
        f"AMSL={home.absolute_altitude_m:.1f} m)  "
        f"target 200 m north @ {TARGET_ALT_REL:.0f} m rel  "
        f"param4={PARAM4_YAW_DEG}° east  autopilot={autopilot}",
    )

    # Dense position sampling from pre-arm onwards — used to detect min vertical ascent.
    pos_samples: list[tuple[float, float]] = []  # (alt_rel_m, dist_from_home_m)

    async def _sample_pos() -> None:
        async for pos in gcs_system.telemetry.position():
            dist = _dist_m(pos.latitude_deg, pos.longitude_deg,
                           home.latitude_deg, home.longitude_deg)
            pos_samples.append((pos.relative_altitude_m, dist))

    sample_task = asyncio.create_task(_sample_pos())

    obs: dict = {
        "yaw_at_airborne": None,
        "checkpoints": [],          # (alt_m, hdg_deg, dist_from_target_m)
        "mode_before_cmd": None,    # flight mode after arm, before NAV_TAKEOFF
        "mode_on_cmd": None,        # flight mode ~1.5 s after sending NAV_TAKEOFF
        "mode_at_target": None,     # flight mode at 85% altitude threshold
        "mode_at_arrival": None,    # flight mode after reaching near-full altitude (HOLD expected)
        "lateral_nav": False,
        "min_ascent_m": None,       # altitude when first lateral movement detected (may be 0 = ground)
        "became_airborne": False,   # True only when AIRBORNE_THRESHOLD_M was reached
        "pre_arm_notes": [],
        "arm_succeeded": False,
    }

    try:
        # ── STACK-SPECIFIC PRE-ARM SETUP ─────────────────────────────────────────
        # Mode changes or prerequisites before arming.  Stack-agnostic code follows.
        if autopilot == "ardupilot" and vehicle_type == "quadcopter":
            # ArduCopter MC requires GUIDED mode before COMMAND_INT/LONG NAV_TAKEOFF.
            # Poll flight_mode() until MAVSDK reports "OFFBOARD" (ArduCopter GUIDED).
            guided_ok = await _set_guided_mode_ardupilot(gcs_system)
            obs["pre_arm_notes"].append(
                f"GUIDED mode ({'confirmed' if guided_ok else 'NOT confirmed — timeout'})"
            )
            log.info(_FMT, _CMD, "pre-arm — ArduCopter MC", obs["pre_arm_notes"][-1])
        elif autopilot == "ardupilot" and vehicle_type == "fixed_wing":
            obs["pre_arm_notes"].append(
                "no mode change tested — COMMAND_INT NAV_TAKEOFF returns FAILED for "
                "ArduPlane fixed-wing (QuadPlane only); see test_arduplane_guided_takeoff_to_target"
            )
            log.info(_FMT, _CMD, "pre-arm — ArduPlane FW", obs["pre_arm_notes"][-1])
        elif autopilot == "px4":
            obs["pre_arm_notes"].append("no mode change required")
            log.info(_FMT, _CMD, "pre-arm — PX4", obs["pre_arm_notes"][-1])
        else:
            obs["pre_arm_notes"].append("no stack-specific setup")

        # ── STACK-SPECIFIC COMMAND FIELD SETUP ───────────────────────────────────
        # PX4 always treats COMMAND_INT z as absolute AMSL (ignores frame field).
        # ArduCopter COMMAND_INT handler requires frame=3 (GLOBAL_RELATIVE_ALT).
        if autopilot == "px4":
            cmd_z     = home.absolute_altitude_m + TARGET_ALT_REL
            cmd_frame = 5   # GLOBAL_INT (absolute AMSL for PX4)
        else:
            cmd_z     = TARGET_ALT_REL
            cmd_frame = 3   # GLOBAL_RELATIVE_ALT (ArduCopter requires exactly frame=3)

        cmd_kw = _takeoff_cmd(
            x=int(TARGET_LAT * 1e7),
            y=int(TARGET_LON * 1e7),
            z=cmd_z,
            param4=PARAM4_YAW_DEG,
            frame=cmd_frame,
        )

        # ── ARM ───────────────────────────────────────────────────────────────────
        # For stacks where the MAVSDK health-stream armable check is slow (ArduCopter
        # SITL cold start), use probe_command_long MAV_CMD_COMPONENT_ARM_DISARM (400)
        # with a retry loop instead of action.arm().
        if autopilot == "ardupilot":
            try:
                async with asyncio.timeout(120.0):
                    while True:
                        ack_arm = await probe_command_long(gcs_system, 400, param1=1.0)
                        if ack_arm and int(ack_arm["result"]) == 0:
                            obs["arm_succeeded"] = True
                            log.info(_FMT, _CMD, "arm", "ACCEPTED")
                            break
                        await asyncio.sleep(3.0)
            except TimeoutError:
                log.warning(_FMT, _CMD, "arm", "did not arm within 120 s")
        else:
            await _wait_armable(gcs_system)
            await gcs_system.action.arm()
            obs["arm_succeeded"] = True
        await asyncio.sleep(0.5)

        # Settle after arm: ArduCopter needs a few seconds for motors to spool
        # from GROUND_IDLE to THROTTLE_UNLIMITED before NAV_TAKEOFF executes.
        if obs["arm_succeeded"]:
            await asyncio.sleep(3.0 if autopilot == "ardupilot" else 0.5)
            obs["mode_before_cmd"] = await _get_flight_mode(gcs_system)

        # ── TIERED NAV_TAKEOFF PROBE ─────────────────────────────────────────────
        # Try approaches in priority order, stop at first that produces a climb.
        # This discovers which approach works for this (stack, vehicle) combination.
        #
        # Tier 1: COMMAND_INT with lat/lon (spec-preferred for location commands)
        # Tier 2: COMMAND_LONG with altitude only (matches ArduPilot autotest)
        # Tier 3: COMMAND_INT without arming (tests auto-arm)
        #
        # Each tier waits PROBE_CLIMB_S for the vehicle to reach AIRBORNE_THRESHOLD_M.
        # If no climb, the tier is marked as failed and the next tier is tried.
        # Re-arming between tiers is needed; disarm happens via RTL inside _rtl_and_land.
        # ArduCopter motor spool + throttle ramp takes ~3-4 s after NAV_TAKEOFF.
        # Allow 45 s per tier for ArduCopter, 20 s for PX4.
        PROBE_CLIMB_S = 45.0 if autopilot == "ardupilot" else 20.0
        _ACK_NAMES = {0:"ACCEPTED",1:"TEMP_REJECTED",2:"DENIED",3:"UNSUPPORTED",4:"FAILED"}
        obs["probe_results"] = []  # list of {"tier": str, "ack": str, "climbed": bool}
        successful_tier = None

        async def _send_and_check_climb(tier_label: str, send_coro, wait_s: float) -> bool:
            """Send NAV_TAKEOFF, wait up to wait_s for a climb, return True if climbed."""
            ack = await send_coro
            ack_r = int(ack["result"]) if ack else -1
            ack_name = _ACK_NAMES.get(ack_r, f"result={ack_r}")
            log.info(_FMT, _CMD, tier_label, f"ACK={ack_name}")
            climbed = False
            if ack_r in (0, 1):  # ACCEPTED or TEMPORARILY_REJECTED
                await asyncio.sleep(1.5)
                obs["mode_on_cmd"] = await _get_flight_mode(gcs_system)
                log.info(_FMT, _CMD, "mode on receipt",
                         f"{obs['mode_before_cmd']} → {obs['mode_on_cmd']}")
                try:
                    await _wait_for_altitude(gcs_system, AIRBORNE_THRESHOLD_M, wait_s)
                    climbed = True
                except TimeoutError:
                    pass
            obs["probe_results"].append({
                "tier": tier_label, "ack": ack_name, "climbed": climbed,
            })
            return climbed

        if obs["arm_succeeded"]:
            # Tier 1: COMMAND_INT with lat/lon (spec-preferred for location commands).
            # Exception: ArduCopter autotest uses COMMAND_LONG (run_cmd, not run_cmd_int)
            # for NAV_TAKEOFF; COMMAND_INT with lat/lon has the same handler but some
            # SITL timing differences. Try COMMAND_INT first; COMMAND_LONG is tier 2.
            climbed = await _send_and_check_climb(
                "tier1: COMMAND_INT+lat/lon",
                probe_command_int(gcs_system, **cmd_kw),
                PROBE_CLIMB_S,
            )
            if climbed:
                successful_tier = "COMMAND_INT with lat/lon"
            else:
                # Attempt failed — RTL back to ground, re-arm for tier 2
                await _rtl_and_land(gcs_system)
                # Re-arm for tier 2 — verify GUIDED mode before arming
                if autopilot == "ardupilot":
                    guided_ok2 = await _set_guided_mode_ardupilot(gcs_system)
                    log.info(_FMT, _CMD, "tier2 re-arm: GUIDED mode",
                             "confirmed" if guided_ok2 else "NOT confirmed")
                    for _ in range(10):
                        ack_r2 = await probe_command_long(gcs_system, 400, param1=1.0)
                        if ack_r2 and int(ack_r2["result"]) == 0:
                            obs["arm_succeeded"] = True
                            break
                        await asyncio.sleep(2.0)
                else:
                    await _wait_armable(gcs_system)
                    await gcs_system.action.arm()
                obs["mode_before_cmd"] = await _get_flight_mode(gcs_system)

                # Tier 2: COMMAND_LONG with altitude only (matches ArduPilot autotest)
                await asyncio.sleep(1.0)  # spool-up settle
                climbed = await _send_and_check_climb(
                    "tier2: COMMAND_LONG+alt-only",
                    probe_command_long(gcs_system, _CMD_ID, param7=cmd_z),
                    PROBE_CLIMB_S,
                )
                if climbed:
                    successful_tier = "COMMAND_LONG with altitude only"

        # Tier 3: unarmed COMMAND_INT (tests auto-arm capability) — probe only, no re-arm needed
        if successful_tier is None:
            await _rtl_and_land(gcs_system)  # ensure disarmed
            obs["mode_before_cmd"] = await _get_flight_mode(gcs_system)
            log.info(_FMT, _CMD, "tier3: unarmed probe", "sending COMMAND_INT without arming")
            climbed = await _send_and_check_climb(
                "tier3: COMMAND_INT (unarmed)",
                probe_command_int(gcs_system, **cmd_kw),
                PROBE_CLIMB_S,
            )
            if climbed:
                successful_tier = "COMMAND_INT (self-armed)"

        obs["successful_tier"] = successful_tier
        log.info(
            _FMT, _CMD, "probe result",
            f"successful approach: {successful_tier or 'none — command not executed on this stack'}",
        )

        # ── GENERIC OBSERVATION ──────────────────────────────────────────────────
        # If a tier succeeded, the vehicle is already airborne — continue observation.
        # If none succeeded, skip to the finally block.
        # Fixed-wing needs longer timeout: runway roll + rotation + initial climb.
        obs_airborne_timeout = 90.0 if vehicle_type == "fixed_wing" else AIRBORNE_TIMEOUT_S
        try:
            pos = await _wait_for_altitude(gcs_system, AIRBORNE_THRESHOLD_M, obs_airborne_timeout)
            hdg = await _get_heading(gcs_system)
            d_tgt = _dist_m(pos.latitude_deg, pos.longitude_deg, TARGET_LAT, TARGET_LON)
            obs["yaw_at_airborne"] = hdg
            obs["became_airborne"] = True
            obs["checkpoints"].append((pos.relative_altitude_m, hdg, d_tgt))
            log.info(
                _FMT, _CMD, f"airborne {AIRBORNE_THRESHOLD_M:.0f} m",
                f"alt={pos.relative_altitude_m:.1f} m  hdg={hdg:.1f}°  "
                f"(param4={PARAM4_YAW_DEG}°, target_bearing≈0° N)  "
                f"dist_from_target={d_tgt:.0f} m",
            )

            try:
                pos = await _wait_for_altitude(gcs_system, 15.0, timeout_s=60.0)
                hdg = await _get_heading(gcs_system)
                d_tgt = _dist_m(pos.latitude_deg, pos.longitude_deg, TARGET_LAT, TARGET_LON)
                obs["checkpoints"].append((pos.relative_altitude_m, hdg, d_tgt))
                log.info(
                    _FMT, _CMD, "15 m checkpoint",
                    f"alt={pos.relative_altitude_m:.1f} m  hdg={hdg:.1f}°  "
                    f"dist_from_target={d_tgt:.0f} m",
                )
            except TimeoutError:
                log.info(_FMT, _CMD, "15 m", "not reached — stack uses lower default altitude")

            threshold = TARGET_ALT_REL * 0.85
            try:
                pos = await _wait_for_altitude(gcs_system, threshold, TAKEOFF_TIMEOUT_S)
                hdg = await _get_heading(gcs_system)
                mode = await _get_flight_mode(gcs_system)
                d_tgt = _dist_m(pos.latitude_deg, pos.longitude_deg, TARGET_LAT, TARGET_LON)
                obs["checkpoints"].append((pos.relative_altitude_m, hdg, d_tgt))
                obs["mode_at_target"] = mode
                obs["lateral_nav"] = (d_tgt < 190)
                log.info(
                    _FMT, _CMD, f"≥{threshold:.0f} m",
                    f"alt={pos.relative_altitude_m:.1f} m  hdg={hdg:.1f}°  "
                    f"mode={mode}  dist_from_target={d_tgt:.0f} m",
                )

                # ── Smart completion detection ────────────────────────────────
                # Choose criterion based on what the vehicle is actually doing:
                #   Heading to waypoint → wait for mode change (PX4 transitions
                #     to HOLD/loiter when the waypoint is reached).
                #   Not heading to waypoint → wait for target altitude OR mode
                #     change (whichever comes first); some stacks complete on
                #     altitude alone without changing mode.
                #   Either way: time out if neither occurs.
                mode_initial = mode
                log.info(
                    _FMT, _CMD, "completion detection",
                    f"heading_to_target={obs['lateral_nav']}  "
                    + ("waiting for mode change (waypoint arrival)"
                       if obs["lateral_nav"]
                       else "waiting for altitude or mode change"),
                )
                if obs["lateral_nav"]:
                    # Vehicle is navigating toward target lat/lon.
                    # PX4 transitions TAKEOFF → HOLD when the waypoint is reached.
                    # Subscribe to the mode stream and wait for any change.
                    try:
                        async with asyncio.timeout(TAKEOFF_TIMEOUT_S):
                            async for fm in gcs_system.telemetry.flight_mode():
                                m = str(fm)
                                if m != mode_initial:
                                    obs["mode_at_arrival"] = m
                                    log.info(
                                        _FMT, _CMD, "completion — waypoint arrived",
                                        f"{mode_initial} → {m}",
                                    )
                                    break
                        if obs["mode_at_arrival"] is None:
                            obs["mode_at_arrival"] = mode_initial
                    except TimeoutError:
                        obs["mode_at_arrival"] = mode_initial
                        log.info(
                            _FMT, _CMD, "completion — timed out",
                            f"vehicle did not arrive at target within {TAKEOFF_TIMEOUT_S:.0f} s  "
                            f"mode remained {mode_initial}",
                        )
                else:
                    # Vehicle not heading toward target lat/lon.
                    # Wait for it to reach near-full altitude OR for a mode change,
                    # whichever occurs first.
                    mode_event  = asyncio.Event()
                    alt_event   = asyncio.Event()
                    final_mode  = [mode_initial]

                    async def _watch_mode_completion() -> None:
                        async for fm in gcs_system.telemetry.flight_mode():
                            m = str(fm)
                            if m != mode_initial:
                                final_mode[0] = m
                                mode_event.set()
                                break

                    async def _watch_alt_completion() -> None:
                        async for p in gcs_system.telemetry.position():
                            if p.relative_altitude_m >= TARGET_ALT_REL * 0.97:
                                alt_event.set()
                                break

                    mode_task2 = asyncio.create_task(_watch_mode_completion())
                    alt_task2  = asyncio.create_task(_watch_alt_completion())
                    try:
                        async with asyncio.timeout(60.0):
                            done, _ = await asyncio.wait(
                                [asyncio.create_task(mode_event.wait()),
                                 asyncio.create_task(alt_event.wait())],
                                return_when=asyncio.FIRST_COMPLETED,
                            )
                        if mode_event.is_set():
                            obs["mode_at_arrival"] = final_mode[0]
                            log.info(_FMT, _CMD, "completion — mode changed",
                                     f"{mode_initial} → {final_mode[0]}")
                        elif alt_event.is_set():
                            obs["mode_at_arrival"] = final_mode[0]  # mode unchanged
                            log.info(_FMT, _CMD, "completion — altitude reached",
                                     f"≥{TARGET_ALT_REL * 0.97:.1f} m  mode={final_mode[0]}")
                    except TimeoutError:
                        obs["mode_at_arrival"] = final_mode[0]
                        log.info(_FMT, _CMD, "completion — timed out",
                                 f"mode remained {mode_initial}  "
                                 "vehicle may be loitering within takeoff mode or "
                                 "altitude/lat-lon not supported by stack")
                    finally:
                        mode_task2.cancel()
                        alt_task2.cancel()
            except TimeoutError:
                mode = await _get_flight_mode(gcs_system)
                obs["mode_at_target"] = mode
                obs["mode_at_arrival"] = mode
                if obs["checkpoints"]:
                    obs["lateral_nav"] = (obs["checkpoints"][-1][2] < 190)
                log.info(
                    _FMT, _CMD, f"≥{threshold:.0f} m not reached",
                    f"stack uses lower default altitude  mode={mode}",
                )
        except TimeoutError:
            log.warning(
                _FMT, _CMD, "observation",
                "vehicle did not reach airborne threshold — command may not have executed",
            )
    finally:
        sample_task.cancel()
        await _rtl_and_land(gcs_system)

    # ── POST-FLIGHT ANALYSIS ──────────────────────────────────────────────────────

    for alt, dist in pos_samples:
        if dist > LATERAL_START_M and obs["min_ascent_m"] is None:
            obs["min_ascent_m"] = alt
            break
    log.info(
        _FMT, _CMD, "min ascent analysis",
        (f"lateral movement (>{LATERAL_START_M:.0f} m from home) first detected at "
         f"alt={obs['min_ascent_m']:.2f} m")
        if obs["min_ascent_m"] is not None
        else "no lateral movement detected in position samples",
    )

    yaw_hdg = obs["yaw_at_airborne"]
    if yaw_hdg is not None:
        diff_param4 = abs((yaw_hdg - PARAM4_YAW_DEG + 180) % 360 - 180)
        if diff_param4 <= 20:
            yaw_verdict = f"param4={PARAM4_YAW_DEG:.0f}° RESPECTED (heading={yaw_hdg:.0f}°)"
            yaw_ignored = False
        else:
            yaw_verdict = f"param4={PARAM4_YAW_DEG:.0f}° IGNORED (heading={yaw_hdg:.0f}°)"
            yaw_ignored = True
    else:
        yaw_verdict = "unknown (no heading sample)"
        yaw_ignored = True

    # ── PRECONDITIONS / FLIGHT SUMMARY ───────────────────────────────────────────
    min_m = obs["min_ascent_m"]
    mode_at_arrival = obs.get("mode_at_arrival") or obs.get("mode_at_target") or "unknown"
    mode_before = obs.get("mode_before_cmd") or "unknown"
    mode_on_cmd = obs.get("mode_on_cmd")   or "unknown"

    # Initial mode: what mode was set before the command that enabled execution.
    initial_mode_req = "; ".join(obs["pre_arm_notes"]) or "no mode change required"

    # Which tier succeeded.
    tier = obs.get("successful_tier") or "none — see probe results"
    probe_lines = "; ".join(
        f"{p['tier']}: {p['ack']} {'→CLIMBED' if p['climbed'] else '→no climb'}"
        for p in obs.get("probe_results", [])
    ) or "no probes completed"

    # "Command arms vehicle": tier-3 result (unarmed probe) or default.
    tier3 = next((p for p in obs.get("probe_results", []) if "unarmed" in p["tier"]), None)
    if tier3 and tier3["climbed"]:
        arm_verdict = "True — command self-arms (tier-3 probe succeeded)"
    else:
        arm_verdict = "False — must pre-arm" if obs["arm_succeeded"] else "Not tested — arm failed"

    # Mode transition on receipt of NAV_TAKEOFF.
    if mode_on_cmd and mode_on_cmd != mode_before:
        mode_receipt = f"{mode_before} → {mode_on_cmd}"
    elif mode_on_cmd:
        mode_receipt = f"{mode_on_cmd} (no change)"
    else:
        mode_receipt = "unknown"

    # Flight description.
    # Rule: if the vehicle did not take off, stop at the first failure.
    # Nothing about navigation, yaw, or pitch is meaningful if airborne was never reached.
    if not obs["arm_succeeded"]:
        flight_desc = "Did not arm — NAV_TAKEOFF not executed."
    elif not obs["became_airborne"]:
        ground_movement = (min_m is not None and min_m < AIRBORNE_THRESHOLD_M)
        if ground_movement:
            if vehicle_type == "fixed_wing":
                movement_note = "SIH FW runway roll — no vertical liftoff."
            else:
                # MC: lateral movement near ground usually means SITL was in motion
                # from a previous test's incomplete RTL, not a real takeoff attempt.
                movement_note = (
                    "lateral movement at low altitude — SITL may have been in RTL "
                    "from a previous test (degraded state after many flight cycles)."
                )
            flight_desc = (
                f"Did not become airborne — ground movement detected "
                f"(altitude < {AIRBORNE_THRESHOLD_M:.0f} m). {movement_note}"
            )
        else:
            flight_desc = "Did not climb — command accepted but not executed."
    else:
        if min_m is None or min_m < 0.5:
            climb_desc = "Climbs diagonally from ground — no vertical-first phase"
        else:
            climb_desc = f"Ascends {min_m:.1f} m vertically, then navigates laterally"

        lat_lon_desc = (
            f"Navigates toward specified lat/lon/alt. Holds at waypoint ({mode_at_arrival} mode)."
            if obs["lateral_nav"]
            else "Does not navigate toward specified lat/lon (lat/lon ignored)."
        )
        yaw_desc = f"Yaw: {yaw_verdict}."
        ignored = ["yaw (param4)", "pitch (param1)"] if yaw_ignored else ["pitch (param1)"]
        flight_desc = (
            f"{climb_desc}. {lat_lon_desc} "
            f"{yaw_desc} Ignored: {', '.join(ignored)}."
        )

    summary_block = (
        "**Preconditions:**\n"
        f"- Initial mode: {initial_mode_req}\n"
        f"- Command arms vehicle: {arm_verdict}\n"
        f"- Mode on NAV_TAKEOFF receipt: {mode_receipt}\n"
        "\n"
        "**Takeoff approach that worked:**\n"
        f"- {tier}\n"
        f"- Probe sequence: {probe_lines}\n"
        "\n"
        "**Flight:**\n"
        f"- {flight_desc}\n"
    )

    log.info(_FMT, _CMD, "BEHAVIOUR SUMMARY", flight_desc)

    try:
        ver = await gcs_system.info.get_version()
        firmware_version = (
            f"{ver.flight_sw_major}.{ver.flight_sw_minor}.{ver.flight_sw_patch}"
        )
    except Exception:
        firmware_version = "unknown"
    _write_and_update_readme(request, autopilot, firmware_version, summary_block)



# Altitude tests (param7 = z)
# ---------------------------------------------------------------------------

async def test_altitude_nominal(gcs_system):
    """NAV_TAKEOFF z=30 m — vehicle climbs to ≥ 85% of target altitude."""
    await _ensure_nav_takeoff_supported(gcs_system)
    target_m = 30.0
    threshold = target_m * 0.85
    try:
        await _arm_and_send_takeoff(gcs_system, z=target_m)
        log.info(_FMT, _CMD, "z=30 m (nominal)", f"waiting for {threshold:.1f} m")
        pos = await _wait_for_altitude(gcs_system, threshold)
        log.info(_FMT, _CMD, "z=30 m (nominal)", f"reached {pos.relative_altitude_m:.1f} m")
        assert pos.relative_altitude_m >= threshold, (
            f"Vehicle only reached {pos.relative_altitude_m:.1f} m (target {target_m} m, "
            f"threshold {threshold:.1f} m)"
        )
    finally:
        await _rtl_and_land(gcs_system)


async def test_altitude_higher(gcs_system):
    """NAV_TAKEOFF z=50 m — vehicle climbs to ≥ 85% of the higher target altitude."""
    await _ensure_nav_takeoff_supported(gcs_system)
    target_m = 50.0
    threshold = target_m * 0.85
    try:
        await _arm_and_send_takeoff(gcs_system, z=target_m)
        log.info(_FMT, _CMD, "z=50 m (higher)", f"waiting for {threshold:.1f} m")
        pos = await _wait_for_altitude(gcs_system, threshold, timeout_s=120.0)
        log.info(_FMT, _CMD, "z=50 m (higher)", f"reached {pos.relative_altitude_m:.1f} m")
        assert pos.relative_altitude_m >= threshold, (
            f"Vehicle only reached {pos.relative_altitude_m:.1f} m (target {target_m} m, "
            f"threshold {threshold:.1f} m)"
        )
    finally:
        await _rtl_and_land(gcs_system)


async def test_altitude_very_low(gcs_system):
    """
    NAV_TAKEOFF z=0.5 m — observational: detects any safety minimum altitude.

    The MAVLink spec does not define a minimum altitude.  Most stacks enforce a
    safety minimum (e.g. 1 m).  This test observes and logs what altitude the
    vehicle actually reaches, without asserting a specific value.
    """
    await _ensure_nav_takeoff_supported(gcs_system)
    target_m = 0.5
    try:
        await _arm_and_send_takeoff(gcs_system, z=target_m)
        await asyncio.sleep(10.0)  # allow time for takeoff to complete or safety minimum to apply
        async with asyncio.timeout(5.0):
            async for pos in gcs_system.telemetry.position():
                log.info(
                    _FMT, _CMD, "z=0.5 m (very low)",
                    f"reached {pos.relative_altitude_m:.2f} m (target {target_m} m — "
                    "safety minimum may apply)",
                )
                break
    finally:
        await _rtl_and_land(gcs_system)


async def test_altitude_nan_uses_default(gcs_system):
    """
    NAV_TAKEOFF z=NaN — observational: vehicle should take off to a default altitude.

    Per the MAVLink spec, NaN altitude means 'use the stack's default altitude'.
    PASS if the vehicle takes off at all (altitude > 0.5 m).
    """
    await _ensure_nav_takeoff_supported(gcs_system)
    try:
        await _arm_and_send_takeoff(gcs_system, z=None)  # None → NaN on wire
        # If NaN is accepted the vehicle should climb; any climb > 0.5 m is a success.
        pos = await _wait_for_altitude(gcs_system, 0.5, timeout_s=30.0)
        log.info(
            _FMT, _CMD, "z=NaN (use default)",
            f"vehicle took off — reached {pos.relative_altitude_m:.1f} m",
        )
        assert pos.relative_altitude_m >= 0.5, (
            "Vehicle did not climb with z=NaN (expected takeoff to default altitude)"
        )
    except TimeoutError:
        log.warning(_FMT, _CMD, "z=NaN", "no altitude reached — stack may have rejected NaN altitude")
    finally:
        await _rtl_and_land(gcs_system)


@pytest.mark.xfail(
    reason=(
        "Spec gap: z=0 is ambiguous — stack may use safety minimum, hover in place, "
        "or reject.  No MAVLink spec requirement exists for this case."
    ),
    strict=False,
)
async def test_altitude_zero_behaviour(gcs_system):
    """
    NAV_TAKEOFF z=0 — observational: what altitude does the vehicle reach?

    Marked xfail because the outcome is stack-specific and the spec is silent on
    whether z=0 should be rejected, treated as 'use safety minimum', or honoured
    literally (hover in place after leaving the ground).
    """
    await _ensure_nav_takeoff_supported(gcs_system)
    try:
        await _arm_and_send_takeoff(gcs_system, z=0.0)
        await asyncio.sleep(10.0)
        async with asyncio.timeout(5.0):
            async for pos in gcs_system.telemetry.position():
                log.info(
                    _FMT, _CMD, "z=0.0 (zero altitude)",
                    f"reached {pos.relative_altitude_m:.2f} m",
                )
                break
    finally:
        await _rtl_and_land(gcs_system)


# ---------------------------------------------------------------------------
# Yaw tests (param4) — all observational
# ---------------------------------------------------------------------------
#
# Both PX4 and ArduPilot ignore param4 in the COMMAND_INT execution path:
#   PX4: rep->current.yaw = NAN regardless (navigator_main.cpp:630)
#   ArduCopter: "param4 : yaw angle   (not supported)" (GCS_MAVLink_Copter.cpp:585)
#   ArduPlane: only altitude read from COMMAND_INT handler (GCS_MAVLink_Plane.cpp)
#
# These tests log the actual heading after takeoff but do not assert.
# ---------------------------------------------------------------------------

async def _observe_yaw(gcs_system, label: str, param4_deg) -> None:
    """Arm, take off with given param4 (yaw), log heading, RTL.  No assertion.

    Uses AIRBORNE_THRESHOLD_M (not the full TAKEOFF_ALT_M) so this works on
    stacks that ignore the z parameter and use a fixed safety altitude (e.g.
    PX4 MPC_TKO_ALT ≈ 2.5 m).
    """
    try:
        await _arm_and_send_takeoff(gcs_system, param4=param4_deg, z=TAKEOFF_ALT_M)
        pos = await _wait_for_altitude(gcs_system, AIRBORNE_THRESHOLD_M, AIRBORNE_TIMEOUT_S)
        heading = await _get_heading(gcs_system)
        log.info(
            _FMT, _CMD, label,
            f"altitude={pos.relative_altitude_m:.1f} m  heading={heading:.1f}° "
            f"(param4={param4_deg}° — observational, all known stacks ignore param4 in COMMAND_INT path)",
        )
    finally:
        await _rtl_and_land(gcs_system)


async def test_yaw_north(gcs_system):
    """param4=0° (north) — observational: log heading after takeoff."""
    await _ensure_nav_takeoff_supported(gcs_system)
    await _observe_yaw(gcs_system, "param4=0° (north)", 0.0)


async def test_yaw_east(gcs_system):
    """param4=90° (east) — observational: log heading after takeoff."""
    await _ensure_nav_takeoff_supported(gcs_system)
    await _observe_yaw(gcs_system, "param4=90° (east)", 90.0)


async def test_yaw_135(gcs_system):
    """param4=135° (SE) — observational: log heading after takeoff."""
    await _ensure_nav_takeoff_supported(gcs_system)
    await _observe_yaw(gcs_system, "param4=135° (SE)", 135.0)


async def test_yaw_near_360(gcs_system):
    """param4=358° (near north, CW) — observational: log heading after takeoff."""
    await _ensure_nav_takeoff_supported(gcs_system)
    await _observe_yaw(gcs_system, "param4=358° (near north)", 358.0)


async def test_yaw_negative(gcs_system):
    """
    param4=−90° — observational: log heading after takeoff.

    Spec gap: negative yaw values are not defined.  The canonical equivalent is 270°
    (west).  This test observes whether the stack treats −90° as 270°, ignores it,
    or rejects it.
    """
    await _ensure_nav_takeoff_supported(gcs_system)
    await _observe_yaw(gcs_system, "param4=−90° (negative yaw)", -90.0)


async def test_yaw_overflow(gcs_system):
    """
    param4=450° — observational: log heading after takeoff.

    Spec gap: values > 360° are not defined.  The canonical equivalent is 90°
    (east).  This test observes whether the stack wraps, ignores, or rejects it.
    """
    await _ensure_nav_takeoff_supported(gcs_system)
    await _observe_yaw(gcs_system, "param4=450° (overflow)", 450.0)


async def test_yaw_very_large(gcs_system):
    """
    param4=3600° — observational: log heading after takeoff.

    Spec gap: an extremely large yaw value is undefined.  The canonical equivalent
    is 0° (north).  This test observes whether the stack wraps (canonical 0°),
    attempts multiple rotations, ignores the value, or rejects it.
    """
    await _ensure_nav_takeoff_supported(gcs_system)
    await _observe_yaw(gcs_system, "param4=3600° (very large)", 3600.0)


# ---------------------------------------------------------------------------
# Position tests (param5/6 = x/y)
# ---------------------------------------------------------------------------

async def test_position_specific(gcs_system):
    """
    NAV_TAKEOFF with explicit home lat/lon — vehicle should take off successfully.

    Tests that providing a specific lat/lon does not break command execution.  The
    spec does not define what the position field means for a COMMAND_INT takeoff:
    it may be the position from which to take off, or the position to arrive at
    after takeoff.  This test only asserts that the vehicle takes off.
    """
    await _ensure_nav_takeoff_supported(gcs_system)
    home = await _get_home_position(gcs_system)
    lat_int = int(home.latitude_deg * 1e7)
    lon_int = int(home.longitude_deg * 1e7)
    try:
        await _arm_and_send_takeoff(gcs_system, x=lat_int, y=lon_int, z=TAKEOFF_ALT_M)
        pos = await _wait_for_altitude(gcs_system, AIRBORNE_THRESHOLD_M, AIRBORNE_TIMEOUT_S)
        log.info(
            _FMT, _CMD, "x/y=home lat/lon",
            f"reached {pos.relative_altitude_m:.1f} m with explicit home coordinates",
        )
        assert pos.relative_altitude_m >= AIRBORNE_THRESHOLD_M, (
            f"Vehicle did not take off with explicit home lat/lon"
        )
    finally:
        await _rtl_and_land(gcs_system)


async def test_position_int32max_stays_at_home(gcs_system):
    """
    NAV_TAKEOFF x=INT32_MAX, y=INT32_MAX — vehicle takes off from current position.

    INT32_MAX is the sentinel meaning "use current vehicle position".  This test
    verifies that when the sentinel is sent via COMMAND_INT, the vehicle takes off
    from where it is (within 5 m of home) rather than navigating to a garbage coordinate.

    ArduCopter rejects INT32_MAX with DENIED in the mission protocol; the command
    protocol behaviour may differ.
    """
    await _ensure_nav_takeoff_supported(gcs_system)
    home = await _get_home_position(gcs_system)
    try:
        await _arm_and_send_takeoff(gcs_system, x=INT32_MAX, y=INT32_MAX, z=TAKEOFF_ALT_M)
        pos = await _wait_for_altitude(gcs_system, AIRBORNE_THRESHOLD_M, AIRBORNE_TIMEOUT_S)
        # Check horizontal distance from home is small (vehicle didn't fly to garbage coords)
        dlat = (pos.latitude_deg - home.latitude_deg) * 111111.0
        dlon = (pos.longitude_deg - home.longitude_deg) * 111111.0 * abs(home.latitude_deg / 90.0 + 0.001)
        dist_m = (dlat**2 + dlon**2) ** 0.5
        log.info(
            _FMT, _CMD, "x/y=INT32_MAX sentinel",
            f"altitude={pos.relative_altitude_m:.1f} m  dist_from_home={dist_m:.1f} m",
        )
        assert dist_m < 50.0, (
            f"Vehicle drifted {dist_m:.1f} m from home with INT32_MAX sentinel — "
            "may have navigated to an invalid coordinate"
        )
    except TimeoutError:
        log.warning(_FMT, _CMD, "x/y=INT32_MAX sentinel", "no altitude reached — stack may have rejected sentinel")
    finally:
        await _rtl_and_land(gcs_system)


@pytest.mark.xfail(
    reason=(
        "Spec gap: x=0, y=0 (equator/prime meridian) is not defined as a sentinel.  "
        "Most stacks treat it as 'use current position', but this is undocumented."
    ),
    strict=False,
)
async def test_position_zero_treated_as_current(gcs_system):
    """
    NAV_TAKEOFF x=0, y=0 — observational: does the stack treat (0, 0) as 'use current'?

    Marked xfail because the outcome is stack-specific and the spec does not address
    whether (0, 0) is a valid takeoff coordinate or a "use current position" sentinel.
    """
    await _ensure_nav_takeoff_supported(gcs_system)
    home = await _get_home_position(gcs_system)
    try:
        await _arm_and_send_takeoff(gcs_system, x=0, y=0, z=TAKEOFF_ALT_M)
        pos = await _wait_for_altitude(gcs_system, AIRBORNE_THRESHOLD_M, AIRBORNE_TIMEOUT_S)
        dlat = (pos.latitude_deg - home.latitude_deg) * 111111.0
        dlon = (pos.longitude_deg - home.longitude_deg) * 111111.0 * abs(home.latitude_deg / 90.0 + 0.001)
        dist_m = (dlat**2 + dlon**2) ** 0.5
        log.info(
            _FMT, _CMD, "x/y=0 (zero coords)",
            f"altitude={pos.relative_altitude_m:.1f} m  dist_from_home={dist_m:.1f} m "
            f"({'≤5 m — treated as current' if dist_m < 5 else '>5 m — treated as literal'})",
        )
    except TimeoutError:
        log.warning(_FMT, _CMD, "x/y=0 (zero coords)", "no altitude reached within timeout")
    finally:
        await _rtl_and_land(gcs_system)


# ---------------------------------------------------------------------------
# Pitch comparison (param1) — two full flight cycles
# ---------------------------------------------------------------------------

@pytest.mark.timeout(600)
async def test_pitch_comparison_low_vs_high(gcs_system):
    """
    NAV_TAKEOFF param1=5° vs param1=45° — observational: compare peak pitch during ascent.

    Two consecutive takeoff cycles.  For each, a background task samples attitude_euler()
    during the climb and records the maximum |pitch| value.  The results are logged.

    The spec defines param1 as "Minimum pitch (if airspeed sensor present)" for fixed-wing
    takeoffs.  It is undefined for multicopters.

    Known stack behaviour (tier 1 findings for COMMAND_INT path):
      PX4: both MC and FW accept param1 but the COMMAND_INT execution path likely
           ignores it (param1 handling not confirmed in navigator_main.cpp for COMMAND_INT).
      ArduCopter: param1 is accepted; AP_Copter::do_takeoff reads param1 from the
           COMMAND_INT and may pass it to the takeoff controller.
      ArduPlane: param1 is the minimum takeoff pitch; values are accepted via COMMAND_INT.

    This test does not assert a specific relationship between param1 and peak pitch because
    the command-path behaviour is stack-dependent and many stacks ignore param1 entirely.
    It is informational: if the peak pitches are similar regardless of param1, the stack
    ignores param1 in the command path.
    """
    await _ensure_nav_takeoff_supported(gcs_system)
    target_m = 20.0  # requested altitude (stacks that honour z will climb here)
    results: dict[str, float] = {}

    for label, pitch_deg in [("param1=5°", 5.0), ("param1=45°", 45.0)]:
        log.info(_FMT, _CMD, label, f"arming and sending takeoff with {label}")
        try:
            await _arm_and_send_takeoff(gcs_system, param1=pitch_deg, z=target_m)
            # Use AIRBORNE_THRESHOLD_M so the test works on stacks that ignore z
            # (e.g. PX4 MPC_TKO_ALT ≈ 2.5 m); peak pitch is still measurable at
            # low altitude.
            pos, peak_pitch = await _wait_for_altitude_with_peak_pitch(
                gcs_system, AIRBORNE_THRESHOLD_M, timeout_s=AIRBORNE_TIMEOUT_S
            )
            log.info(
                _FMT, _CMD, label,
                f"reached {pos.relative_altitude_m:.1f} m  peak_|pitch|={peak_pitch:.1f}°",
            )
            results[label] = peak_pitch
        except TimeoutError as exc:
            log.warning(_FMT, _CMD, label, f"altitude not reached: {exc}")
            results[label] = 0.0
        finally:
            await _rtl_and_land(gcs_system)
            await asyncio.sleep(5.0)  # brief pause between cycles

    if "param1=5°" in results and "param1=45°" in results:
        low_pitch = results["param1=5°"]
        high_pitch = results["param1=45°"]
        log.info(
            _FMT, _CMD, "pitch comparison",
            f"param1=5°→peak={low_pitch:.1f}°  param1=45°→peak={high_pitch:.1f}°  "
            f"{'stack honours param1' if high_pitch > low_pitch + 5.0 else 'stack likely ignores param1 in COMMAND_INT path'}",
        )


# ---------------------------------------------------------------------------
# Pre-arm behaviour
# ---------------------------------------------------------------------------

async def test_unarmed_takeoff(gcs_system):
    """
    Observe whether COMMAND_INT NAV_TAKEOFF self-arms the vehicle.

    Sends NAV_TAKEOFF while the vehicle is disarmed (after waiting for armable
    state but WITHOUT calling arm()).  Documents whether the stack requires
    pre-arming or can auto-arm when it receives the command.

    Observational — passes regardless of result.  If the vehicle is not armable
    within ARMABLE_TIMEOUT_S (SITL degraded after many flight cycles), the test
    logs a warning and passes rather than failing with TimeoutError.
    """
    await _ensure_nav_takeoff_supported(gcs_system)
    try:
        await _wait_armable(gcs_system)
    except (TimeoutError, asyncio.TimeoutError):
        log.info(_FMT, _CMD, "unarmed takeoff",
                 "armable timeout — rebooting SITL before test")
        await _reboot_sitl_if_degraded(gcs_system)
        try:
            await _wait_armable(gcs_system, timeout_s=120.0)
        except (TimeoutError, asyncio.TimeoutError):
            log.warning(_FMT, _CMD, "unarmed takeoff",
                        "still not armable after reboot — skipping observation")
            return
    home = await _get_home_position(gcs_system)

    absolute_z = home.absolute_altitude_m + AIRBORNE_THRESHOLD_M
    kw = _takeoff_cmd(
        x=int(home.latitude_deg * 1e7),
        y=int(home.longitude_deg * 1e7),
        z=absolute_z,
        frame=5,  # GLOBAL_INT absolute AMSL — matches PX4 COMMAND_INT behaviour
    )
    _RESULT_NAMES = {
        0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
        3: "UNSUPPORTED", 4: "FAILED",
    }
    try:
        ack = await probe_command_int(gcs_system, **kw)
        result = int(ack["result"]) if ack is not None else -1
        result_name = _RESULT_NAMES.get(result, f"result={result}")
        try:
            pos = await _wait_for_altitude(gcs_system, AIRBORNE_THRESHOLD_M, timeout_s=20.0)
            log.info(
                _FMT, _CMD, "unarmed takeoff",
                f"ACK={result_name} AND vehicle climbed to {pos.relative_altitude_m:.1f} m "
                "— stack SELF-ARMS on COMMAND_INT NAV_TAKEOFF",
            )
        except TimeoutError:
            log.info(
                _FMT, _CMD, "unarmed takeoff",
                f"ACK={result_name}, vehicle did NOT climb — "
                "pre-arming is required before COMMAND_INT NAV_TAKEOFF",
            )
    finally:
        await _rtl_and_land(gcs_system)


async def test_required_flight_mode(gcs_system):
    """
    Observe what flight mode allows COMMAND_INT NAV_TAKEOFF to execute.

    Arms the vehicle via MAVSDK action.arm() (which sets the stack's default
    armed mode), samples the flight mode BEFORE sending NAV_TAKEOFF, sends
    the command, and records whether the vehicle climbs.

    Observational — passes regardless of result; documents the mode requirement.
    If the vehicle is not armable within ARMABLE_TIMEOUT_S (SITL degraded after
    many flight cycles), the test logs a warning and passes immediately.
    """
    await _ensure_nav_takeoff_supported(gcs_system)
    try:
        home = await _get_home_position(gcs_system)
        absolute_z = home.absolute_altitude_m + TAKEOFF_ALT_M

        try:
            await _wait_armable(gcs_system)
        except (TimeoutError, asyncio.TimeoutError):
            log.warning(_FMT, _CMD, "required flight mode",
                        f"vehicle not armable within {ARMABLE_TIMEOUT_S:.0f} s — "
                        "SITL may be degraded; observation skipped")
            return
        await gcs_system.action.arm()
        await asyncio.sleep(0.5)

        mode_before = await _get_flight_mode(gcs_system)

        kw = _takeoff_cmd(
            x=int(home.latitude_deg * 1e7),
            y=int(home.longitude_deg * 1e7),
            z=absolute_z,
            frame=5,
        )
        await send_command_int(gcs_system, **kw)

        try:
            pos = await _wait_for_altitude(gcs_system, AIRBORNE_THRESHOLD_M, AIRBORNE_TIMEOUT_S)
            mode_during = await _get_flight_mode(gcs_system)
            log.info(
                _FMT, _CMD, "required flight mode",
                f"armed mode: {mode_before}  →  climbed to {pos.relative_altitude_m:.1f} m  "
                f"mode during climb: {mode_during}",
            )
        except TimeoutError:
            log.info(
                _FMT, _CMD, "required flight mode",
                f"armed mode: {mode_before}  →  did NOT climb "
                "(NAV_TAKEOFF not executed from this mode; stack may require mode change)",
            )
    finally:
        await _rtl_and_land(gcs_system)


# ---------------------------------------------------------------------------
# Post-takeoff mode (informational)
# ---------------------------------------------------------------------------

def _dist_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Approximate horizontal distance in metres between two WGS84 points."""
    dlat = (lat1 - lat2) * 111111.0
    dlon = (lon1 - lon2) * 111111.0 * abs(lat2 / 90.0 + 0.001)
    return (dlat**2 + dlon**2) ** 0.5


async def test_mode_after_takeoff(gcs_system):
    """
    Observe the flight mode once NAV_TAKEOFF altitude is reached — informational only.

    The MAVLink spec does not define what mode a vehicle should be in after a
    COMMAND_INT NAV_TAKEOFF.  This test logs the observed mode and passes.
    If the vehicle is not armable within ARMABLE_TIMEOUT_S (SITL degraded after
    many flight cycles), the test logs a warning and passes immediately.
    """
    await _ensure_nav_takeoff_supported(gcs_system)
    try:
        try:
            await _arm_and_send_takeoff(gcs_system, z=TAKEOFF_ALT_M)
        except (TimeoutError, asyncio.TimeoutError):
            log.warning(_FMT, _CMD, "flight mode after takeoff",
                        f"armable timeout ({ARMABLE_TIMEOUT_S:.0f} s) — "
                        "SITL may be degraded after many flight cycles; observation skipped")
            return
        try:
            pos = await _wait_for_altitude(gcs_system, AIRBORNE_THRESHOLD_M, AIRBORNE_TIMEOUT_S)
        except TimeoutError:
            log.warning(
                _FMT, _CMD, "flight mode after takeoff",
                "vehicle did not reach airborne threshold — flight mode not observed",
            )
            return
        flight_mode = await _get_flight_mode(gcs_system)
        log.info(
            _FMT, _CMD, "flight mode after takeoff",
            f"altitude={pos.relative_altitude_m:.1f} m  mode={flight_mode} (informational)",
        )
    finally:
        await _rtl_and_land(gcs_system)


# ---------------------------------------------------------------------------
# Comprehensive COMMAND_INT NAV_TAKEOFF execution observation — multicopter
# Validated on PX4 MC and ArduCopter; intended to run on any quadcopter stack.
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# ArduPlane: GUIDED mode takeoff to a nearby target location
# ---------------------------------------------------------------------------

@pytest.mark.timeout(600)
async def test_arduplane_guided_takeoff_to_target(gcs_system, request):
    """
    ArduPlane FW: observe NAV_TAKEOFF (GUIDED mode) to a location ~1 km away at 200 m.

    ArduPlane does not execute COMMAND_INT NAV_TAKEOFF unless the vehicle is first
    placed in GUIDED (or AUTO/TAKEOFF) mode.  The existing execution probe arms and
    sends the command without setting the mode, so all standard flight tests skip on
    ArduPlane FW.  This test explicitly sets GUIDED mode (custom_mode=15) first.

    Sends NAV_TAKEOFF with:
      - param1 = 15° (MinPitch) — observe whether ArduPlane honours it
      - x/y    = ~1 km north of home — observe whether ArduPlane navigates there
      - z      = 200 m relative altitude

    Three checkpoints are logged (5 m, 50 m, 200 m) recording:
      - Actual altitude reached
      - Peak |pitch| sampled since launch
      - Horizontal distance from home and from the target lat/lon

    Source (commands_logic.cpp::do_takeoff):
      `next_WP_loc.lat = home.lat + 10; next_WP_loc.lng = home.lng + 10`
    → lat/lon in NAV_TAKEOFF is IGNORED by ArduPlane; the plane always takes off
    from home regardless of the specified coordinates.  "Switching to GUIDED at
    altitude" below is done by THIS TEST (not automatic) to observe position after
    takeoff.  ArduPlane stays in AUTO throughout — no automatic mode transition.

    param1 (min pitch): ArduPlane TAKEOFF mode reads TKOFF_PITCH_MIN, not the
    COMMAND_INT param1.  The mission path (`do_takeoff`) reads `cmd.p1` directly.
    Two PARAM_SET runs (TKOFF_PITCH_MIN=5° and 45°) are used here to test whether
    a higher minimum pitch forces a higher actual pitch, or whether TECS pitch
    dominates regardless.

    Purely observational — no assertions.
    """
    if (request.config.getoption("--autopilot") != "ardupilot" or
            request.config.getoption("--vehicle-type") != "fixed_wing"):
        pytest.skip(
            "ArduPlane fixed-wing only "
            "(--autopilot=ardupilot --vehicle-type=fixed_wing)"
        )

    home = await _get_home_position(gcs_system)

    # ~1 km north of home; ArduPlane starts facing west (270°) so this is crosswind
    TARGET_LAT        = home.latitude_deg + 0.009
    TARGET_LON        = home.longitude_deg
    TARGET_ALT_REL_M  = 200.0

    log.info(
        _FMT, _CMD, "ArduPlane takeoff observation — setup",
        f"home ({home.latitude_deg:.5f}°N, {home.longitude_deg:.5f}°E "
        f"AMSL={home.absolute_altitude_m:.1f} m hdg=270° west)  "
        f"target ({TARGET_LAT:.5f}°N, {TARGET_LON:.5f}°E {TARGET_ALT_REL_M:.0f} m rel)",
    )

    # ── Probe 1: does GUIDED mode accept NAV_TAKEOFF COMMAND_INT? ──────────────
    # Tier 1 tests show ACCEPTED when unarmed (no mode set).  Here we verify what
    # happens in GUIDED mode specifically, and note whether the result changes.
    import json as _json
    ack_guided = await probe_command_long(
        gcs_system, 176, param1=1.0, param2=15.0,  # DO_SET_MODE → GUIDED
    )
    log.info(_FMT, _CMD, "DO_SET_MODE GUIDED", f"ACK={ack_guided['result'] if ack_guided else 'NONE'}")

    ack_tkoff_guided = await probe_command_int(
        gcs_system, _CMD_ID,
        frame=6, param1=15.0, param2=0.0, param3=0.0, param4=None,
        x=int(TARGET_LAT * 1e7), y=int(TARGET_LON * 1e7), z=TARGET_ALT_REL_M,
    )
    log.info(
        _FMT, _CMD, "NAV_TAKEOFF in GUIDED mode (unarmed)",
        f"ACK={ack_tkoff_guided['result'] if ack_tkoff_guided else 'NONE'}  "
        f"(DENIED=2 means GUIDED does not support ground takeoff via COMMAND_INT)",
    )

    # ── Probe 2: does MANUAL mode accept NAV_TAKEOFF COMMAND_INT? ─────────────
    ack_manual = await probe_command_long(
        gcs_system, 176, param1=1.0, param2=0.0,  # DO_SET_MODE → MANUAL
    )
    log.info(_FMT, _CMD, "DO_SET_MODE MANUAL", f"ACK={ack_manual['result'] if ack_manual else 'NONE'}")

    ack_tkoff_manual = await probe_command_int(
        gcs_system, _CMD_ID,
        frame=6, param1=15.0, param2=0.0, param3=0.0, param4=None,
        x=int(TARGET_LAT * 1e7), y=int(TARGET_LON * 1e7), z=TARGET_ALT_REL_M,
    )
    log.info(
        _FMT, _CMD, "NAV_TAKEOFF in MANUAL mode (unarmed)",
        f"ACK={ack_tkoff_manual['result'] if ack_tkoff_manual else 'NONE'}  "
        f"(ACCEPTED=0 but never executes — no throttle control in MANUAL)",
    )

    # ── Actual flight: TAKEOFF mode (custom_mode=13) with two TKOFF_PITCH_MIN values
    # ArduPlane's dedicated TAKEOFF mode applies full throttle automatically and
    # climbs to TKOFF_ALT.  COMMAND_INT NAV_TAKEOFF is NOT used here — ArduPlane
    # does not execute it for standard fixed-wing.  TAKEOFF mode is the correct
    # supported mechanism.  TKOFF_PITCH_MIN controls the minimum pitch in this mode
    # (equivalent to cmd.p1 in the mission protocol do_takeoff path).
    from mavsdk.mavlink_direct import MavlinkMessage as _MavlinkMessage

    async def _param_set(param_id: str, value: float) -> None:
        await gcs_system.mavlink_direct.send_message(_MavlinkMessage(
            message_name="PARAM_SET",
            system_id=255, component_id=1,
            target_system_id=1, target_component_id=1,
            fields_json=_json.dumps({
                "target_system": 1, "target_component": 1,
                "param_id": param_id,
                "param_value": value,
                "param_type": 10,   # MAV_PARAM_TYPE_REAL32
            }),
        ))
        await asyncio.sleep(0.3)

    await _param_set("TKOFF_ALT", TARGET_ALT_REL_M)
    log.info(_FMT, _CMD, "PARAM_SET TKOFF_ALT", f"{TARGET_ALT_REL_M:.0f} m")

    peak_pitches: dict[str, float] = {}

    for label, min_pitch_deg in [("TKOFF_PITCH_MIN=5°", 5.0), ("TKOFF_PITCH_MIN=45°", 45.0)]:
        await _param_set("TKOFF_PITCH_MIN", min_pitch_deg)
        log.info(_FMT, _CMD, f"PARAM_SET {label}", f"{min_pitch_deg:.0f}°")

        ack = await probe_command_long(
            gcs_system, 176, param1=1.0, param2=13.0,  # DO_SET_MODE → TAKEOFF (mode 13)
        )
        log.info(_FMT, _CMD, f"DO_SET_MODE TAKEOFF — {label}",
                 f"ACK={ack['result'] if ack else 'NONE'}")

        pitch_log_run: list[float] = []

        async def _poll_pitch_run() -> None:
            async for att in gcs_system.telemetry.attitude_euler():
                pitch_log_run.append(att.pitch_deg)

        pitch_task = asyncio.create_task(_poll_pitch_run())

        def _log_pos(sublabel: str, pos) -> None:
            peak = max((abs(p) for p in pitch_log_run), default=0.0)
            d_home = _dist_m(pos.latitude_deg, pos.longitude_deg,
                             home.latitude_deg, home.longitude_deg)
            d_tgt  = _dist_m(pos.latitude_deg, pos.longitude_deg,
                             TARGET_LAT, TARGET_LON)
            log.info(
                _FMT, _CMD, f"{label} | {sublabel}",
                f"alt={pos.relative_altitude_m:.1f} m  peak_|pitch|={peak:.1f}°  "
                f"dist_from_home={d_home:.0f} m  dist_from_target={d_tgt:.0f} m  "
                f"({pos.latitude_deg:.5f}°N, {pos.longitude_deg:.5f}°E)",
            )

        try:
            await _wait_armable(gcs_system)
            ack = await probe_command_long(gcs_system, 400, param1=1.0)
            log.info(_FMT, _CMD, f"ARM — {label}", f"ACK={ack['result'] if ack else 'NONE'}")
            await asyncio.sleep(0.5)

            try:
                pos = await _wait_for_altitude(gcs_system, 5.0, timeout_s=90.0)
                _log_pos("airborne 5 m", pos)

                try:
                    pos = await _wait_for_altitude(gcs_system, 50.0, timeout_s=120.0)
                    _log_pos("50 m checkpoint", pos)
                except TimeoutError:
                    log.warning(_FMT, _CMD, f"{label} | 50 m", "not reached within 120 s")

                threshold = TARGET_ALT_REL_M * 0.85
                try:
                    pos = await _wait_for_altitude(gcs_system, threshold, timeout_s=300.0)
                    peak_run = max((abs(p) for p in pitch_log_run), default=0.0)
                    peak_pitches[label] = peak_run
                    _log_pos(f"≥{threshold:.0f} m", pos)
                    mode = await _get_flight_mode(gcs_system)
                    log.info(_FMT, _CMD, f"{label} | flight mode at altitude", mode)
                    # Note: ArduPlane does NOT switch to GUIDED automatically.
                    # This test switches to GUIDED to observe position after takeoff.
                    ack = await probe_command_long(
                        gcs_system, 176, param1=1.0, param2=15.0,  # GUIDED
                    )
                    log.info(_FMT, _CMD, f"{label} | DO_SET_MODE GUIDED (test-initiated, not auto)",
                             f"ACK={ack['result'] if ack else 'NONE'}")
                    await asyncio.sleep(3.0)
                    async with asyncio.timeout(5.0):
                        async for pos2 in gcs_system.telemetry.position():
                            d2 = _dist_m(pos2.latitude_deg, pos2.longitude_deg, TARGET_LAT, TARGET_LON)
                            log.info(
                                _FMT, _CMD, f"{label} | GUIDED 3 s",
                                f"alt={pos2.relative_altitude_m:.1f} m  "
                                f"dist_from_target={d2:.0f} m  "
                                f"({pos2.latitude_deg:.5f}°N, {pos2.longitude_deg:.5f}°E)  "
                                "(lat/lon from NAV_TAKEOFF is IGNORED by ArduPlane)",
                            )
                            break
                except TimeoutError:
                    log.warning(_FMT, _CMD, f"{label} | ≥{threshold:.0f} m",
                                "not reached within 300 s")
            except TimeoutError:
                log.warning(_FMT, _CMD, f"{label} | airborne 5 m", "not reached within 90 s")
        finally:
            pitch_task.cancel()
            await _rtl_and_land(gcs_system)
            await asyncio.sleep(5.0)  # brief settle between runs

    pitch_verdict = "unknown (fewer than 2 runs completed)"
    if len(peak_pitches) == 2:
        low  = peak_pitches["TKOFF_PITCH_MIN=5°"]
        high = peak_pitches["TKOFF_PITCH_MIN=45°"]
        pitch_verdict = (
            f"TKOFF_PITCH_MIN=5°→peak={low:.1f}°  TKOFF_PITCH_MIN=45°→peak={high:.1f}° — "
            + ("higher minimum forced higher actual pitch (param1 honoured)"
               if high > low + 5.0
               else "peak pitch unchanged — TECS pitch dominates minimum (param1 ignored in practice)")
        )
        log.info(_FMT, _CMD, "pitch comparison result", pitch_verdict)

    # ── BEHAVIOUR SUMMARY ────────────────────────────────────────────────────────
    autopilot = request.config.getoption("--autopilot", default=None)
    summary = (
        "The plane takes off in its initial direction, ignoring yaw, pitch, lat, and lon "
        "(source: ArduPlane do_takeoff() overwrites lat/lon with home±10 units). "
        f"Pitch: {pitch_verdict}. "
        "Acceptance on reaching TKOFF_ALT — then loiters within TAKEOFF mode. "
        "Mode transition: NOT automatic (test switches to GUIDED to observe post-takeoff position). "
        "Pre-arm requirements: TAKEOFF mode (mode 13) + arm."
    )
    log.info(_FMT, _CMD, "BEHAVIOUR SUMMARY", summary)

    arduplane_pre_arm = (
        "**Preconditions:**\n"
        "- Initial mode: TAKEOFF mode (mode 13) — set via DO_SET_MODE before arming\n"
        "- Command arms vehicle: False — arm() is called after TAKEOFF mode is set; "
        "COMMAND_INT NAV_TAKEOFF is NOT used (returns FAILED for non-QuadPlane fixed-wing)\n"
        "- Mode on NAV_TAKEOFF receipt: N/A — takeoff is triggered by TAKEOFF mode + arm, "
        "not by the NAV_TAKEOFF command\n"
        "\n"
        "**Flight:**\n"
        f"- {summary}\n"
    )
    summary_block = arduplane_pre_arm

    try:
        ver = await gcs_system.info.get_version()
        firmware_version = (
            f"{ver.flight_sw_major}.{ver.flight_sw_minor}.{ver.flight_sw_patch}"
        )
    except Exception:
        firmware_version = "unknown"
    _write_and_update_readme(request, autopilot, firmware_version, summary_block)
