"""
Generic Tier 2 (flight/execution) test helpers — vehicle-state scaffolding
shared across both the command-protocol and mission-protocol test trees.

Deliberately protocol-neutral (lives at tests/, not tests/command/ or
tests/mission/): both trees need "get a real vehicle into a flyable state and
observe telemetry" as a precondition for testing their own, genuinely
command/mission-specific execution semantics, and keeping that split by
protocol would make one tree depend on the other for no reason.

Consolidates what used to be three independently-drifted copies of this same
scaffolding: tests/command/nav_takeoff/test_flight.py (source of most of
what's here), tests/command/do_reposition/test_flight.py, and
tests/mission/nav_takeoff/test_flight.py — the latter two had each
reimplemented arm/wait/observe logic from scratch rather than importing it,
and had drifted (most notably two incompatible _dist_m() formulas; see its
docstring below). Callers needing NAV_TAKEOFF only as a way to reach a
flyable state (not testing NAV_TAKEOFF itself) should use
_arm_and_send_takeoff(); a command's own execution-semantics tests (touchdown
analysis, DO_JUMP tallying, WIND_COV comparison, etc.) stay in that command's
own test_flight.py — nothing command-specific belongs here.
"""

import asyncio
import json
import logging
import math
import time as _time_m

import pytest
from mavsdk.mavlink_direct import MavlinkMessage
from mavsdk.telemetry import LandedState

from tests.command.conftest import probe_command_long, send_command_int

log = logging.getLogger(__name__)

ARMABLE_TIMEOUT_S = 60.0
TAKEOFF_ALT_M = 30.0        # metres relative — nominal altitude for _arm_and_send_takeoff()
TAKEOFF_TIMEOUT_S = 90.0    # default timeout for _wait_for_altitude()
RTL_LAND_TIMEOUT_S = 120.0
AIRBORNE_THRESHOLD_M = 2.0  # metres — confirms the vehicle left the ground

_NAV_TAKEOFF_CMD_ID = 22  # MAV_CMD_NAV_TAKEOFF

# SITL self-recovery cooldown state (module-level: the concern is "has any
# caller rebooted the SITL recently", not per-command).
_REBOOT_COOLDOWN_S: float = 300.0
_last_sitl_reboot_ts: float = 0.0


@pytest.fixture(autouse=True)
def require_real_stack(request):
    """Skip every test in a module using this fixture when no --drone-address is given."""
    if request.config.getoption("--drone-address") is None:
        pytest.skip("Execution tests require a real flight stack (--drone-address not set)")


# ---------------------------------------------------------------------------
# Telemetry request helpers
# ---------------------------------------------------------------------------


async def _request_home_position(system) -> None:
    """
    Request HOME_POSITION (msg id=242) from the autopilot once.

    ArduCopter does not re-broadcast HOME_POSITION automatically after the
    initial connect. MAVSDK's telemetry.home() uses this message. After many
    function-scoped System reconnects within the same session, the stream may
    have gone stale. MAV_CMD_REQUEST_MESSAGE (512) fetches one fresh copy.
    """
    await system.mavlink_direct.send_message(MavlinkMessage(
        message_name="COMMAND_LONG",
        system_id=255, component_id=1,
        target_system_id=1, target_component_id=0,
        fields_json=json.dumps({
            "target_system": 1, "target_component": 0,
            "command": 512,      # MAV_CMD_REQUEST_MESSAGE
            "param1": 242.0,    # MAVLINK_MSG_ID_HOME_POSITION
            "param2": 0.0, "param3": 0.0, "param4": 0.0,
            "param5": 0.0, "param6": 0.0, "param7": 0.0,
            "confirmation": 0,
        }),
    ))
    await asyncio.sleep(0.3)


async def _request_position_stream(system, rate_hz: float = 5.0) -> None:
    """
    Request GLOBAL_POSITION_INT (msg id=33) from the autopilot.

    MAVSDK mavsdk_server does not automatically request position streaming
    from ArduCopter (unlike PX4, which sends it by default). ArduCopter
    requires an explicit MAV_CMD_SET_MESSAGE_INTERVAL before it begins
    streaming GLOBAL_POSITION_INT — without this, telemetry.position() and
    mavlink_direct.message("GLOBAL_POSITION_INT") both time out indefinitely.
    """
    interval_us = int(1_000_000 / rate_hz)
    await system.mavlink_direct.send_message(MavlinkMessage(
        message_name="COMMAND_LONG",
        system_id=255, component_id=1,
        target_system_id=1, target_component_id=0,
        fields_json=json.dumps({
            "target_system": 1, "target_component": 0,
            "command": 511,     # MAV_CMD_SET_MESSAGE_INTERVAL
            "param1": 33.0,    # MAVLINK_MSG_ID_GLOBAL_POSITION_INT
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


async def _get_position(system, timeout_s: float = 5.0):
    """Return the current Position from telemetry."""
    async with asyncio.timeout(timeout_s):
        async for pos in system.telemetry.position():
            return pos
    raise TimeoutError("Position not received")


async def _get_heading(system, timeout_s: float = 5.0) -> float:
    """Return current vehicle heading in degrees (0-360)."""
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


# ---------------------------------------------------------------------------
# Wait helpers
# ---------------------------------------------------------------------------


async def _wait_armable(system, timeout_s: float = ARMABLE_TIMEOUT_S):
    """
    Block until the vehicle reports is_armable=True.

    Uses fire-and-forget task + asyncio.Event to avoid leaving a dangling gRPC
    health stream after timeout cancellation (CLAUDE.md §4a pattern) — the
    gRPC stream does not respond to asyncio cancellation reliably; wrapping it
    in a background task and only awaiting a plain Event ensures clean timeout
    handling.
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

    return pos, peak_pitch


async def _wait_for_horizontal_position(
    system, target_lat: float, target_lon: float, threshold_m: float, timeout_s: float = 60.0,
):
    """Block until the vehicle is within threshold_m of target lat/lon; return the Position."""
    async with asyncio.timeout(timeout_s):
        async for pos in system.telemetry.position():
            if _dist_m(pos.latitude_deg, pos.longitude_deg, target_lat, target_lon) <= threshold_m:
                return pos
    raise TimeoutError(f"Did not reach within {threshold_m:.1f} m of target within {timeout_s:.0f} s")


# ---------------------------------------------------------------------------
# Mode / action helpers
# ---------------------------------------------------------------------------


async def _set_guided_mode_ardupilot(system, timeout_s: float = 15.0, custom_mode: int = 4) -> bool:
    """
    Set ArduPilot GUIDED mode and verify via telemetry.

    Sends DO_SET_MODE (176) with the given ``custom_mode`` and polls
    ``telemetry.flight_mode()`` until the mode is confirmed.

    MAVSDK mode string mapping (observed):
    - ArduCopter GUIDED (custom_mode=4)  -> "OFFBOARD"
    - ArduPlane GUIDED  (custom_mode=15) -> "UNKNOWN" (MAVSDK has no mapping
      for ArduPlane custom modes beyond a small set)

    Acceptance rules:
    - "OFFBOARD" or "GUIDED" in the mode string: definitive confirmation.
    - "UNKNOWN": MAVSDK has no mode mapping but the DO_SET_MODE ACK was
      accepted; treat as confirmed since we cannot verify further.
    - Any other definite mode (e.g. "MANUAL", "STABILIZED"): retry.

    Returns True when confirmed (including UNKNOWN-after-ACK), False on timeout.
    """
    guided_ack_accepted = False
    async with asyncio.timeout(timeout_s):
        while True:
            ack = await probe_command_long(system, 176, param1=1.0, param2=float(custom_mode))
            if ack and int(ack["result"]) == 0:
                guided_ack_accepted = True
            await asyncio.sleep(0.5)
            try:
                mode = await _get_flight_mode(system, timeout_s=1.0)
                if "OFFBOARD" in mode or "GUIDED" in mode:
                    log.info("GUIDED mode confirmed: telemetry reports %r", mode)
                    return True
                if "UNKNOWN" in mode and guided_ack_accepted:
                    log.info(
                        "GUIDED mode confirmed (ACK only): telemetry reports %r — "
                        "no MAVSDK mapping for custom_mode=%d", mode, custom_mode,
                    )
                    return True
                log.info("GUIDED mode not yet active: current mode %r — retrying", mode)
            except TimeoutError:
                pass
            await asyncio.sleep(0.5)
    return False


async def _rtl_and_land(system, timeout_s: float = RTL_LAND_TIMEOUT_S) -> None:
    """Command RTL and wait for landed state; then disarm. Best-effort — never raises."""
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


async def _reboot_sitl_if_degraded(system, label: str = "SITL") -> None:
    """
    Reboot the flight stack via MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (cmd=246).

    Intended to be called when ``_wait_armable()`` times out, indicating the
    SITL has degraded after many consecutive arm-takeoff-RTL cycles (PX4 SIH
    accumulates EKF drift and physics state over many cycles until
    ``is_armable`` never returns True again).

    After sending the reboot command:
    - PX4 reboots its firmware and the SIH physics module (~15 s).
    - The session-scoped ``mavsdk_server`` reconnects automatically on the
      same UDP port when PX4 resumes broadcasting heartbeats.
    - Position and home streams are re-requested so the caller's next
      ``_wait_armable`` has fresh telemetry.

    A 5-minute cooldown prevents reboot loops if recovery fails. Caller must
    call ``_wait_armable(system, timeout_s=120.0)`` after this returns.
    `label` is used only for logging (e.g. the calling command's name).
    """
    global _last_sitl_reboot_ts
    now = _time_m.monotonic()
    if now - _last_sitl_reboot_ts < _REBOOT_COOLDOWN_S:
        log.warning(
            "%s SITL reboot: within cooldown (%.0f s) — not rebooting; "
            "caller's _wait_armable will raise TimeoutError", label, _REBOOT_COOLDOWN_S,
        )
        return
    _last_sitl_reboot_ts = now
    log.info(
        "%s SITL reboot: is_armable=False after timeout — sending "
        "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (SITL degraded after many flight cycles)", label,
    )
    try:
        # param1=1.0: reboot autopilot (not companion). PX4 may disconnect
        # before the ACK arrives, so exceptions are suppressed.
        await probe_command_long(system, 246, param1=1.0)
    except Exception:
        pass
    # Wait for PX4 SIH to boot and for EKF to begin converging before
    # re-requesting streams (streams silently timeout if PX4 isn't ready yet).
    await asyncio.sleep(15.0)
    await _request_position_stream(system)
    await _request_home_position(system)
    log.info("%s SITL reboot: done — caller should now _wait_armable(120 s)", label)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def _dist_m(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float) -> float:
    """
    Flat-earth distance in metres between two WGS84 points (accurate to
    < 0.1% for distances < 10 km).

    Canonical version, adopted from do_reposition/test_flight.py — replaces a
    cruder `abs(lat2/90 + 0.001)` longitude-scale approximation that had
    independently drifted into tests/command/nav_takeoff/test_flight.py.
    """
    mid_lat = math.radians((lat1_deg + lat2_deg) / 2)
    dlat = (lat2_deg - lat1_deg) * 111111.0
    dlon = (lon2_deg - lon1_deg) * 111111.0 * math.cos(mid_lat)
    return math.sqrt(dlat**2 + dlon**2)


def _offset_lat_lon(lat_deg: float, lon_deg: float, north_m: float, east_m: float) -> tuple[float, float]:
    """Return (lat, lon) displaced by north_m and east_m from the given point."""
    lat = lat_deg + north_m / 111111.0
    lon = lon_deg + east_m / (111111.0 * math.cos(math.radians(lat_deg)))
    return lat, lon


def _north_of(lat_deg: float, metres: float) -> int:
    """Return latitude as int×1e7 for a position metres north of lat_deg."""
    return int((lat_deg + metres / 111111.0) * 1e7)


# ---------------------------------------------------------------------------
# "Get airborne via NAV_TAKEOFF" utility
# ---------------------------------------------------------------------------
#
# Used by other commands' Tier 2 tests purely as a way to reach a flyable
# state, not to test NAV_TAKEOFF itself — NAV_TAKEOFF's own execution
# semantics (param behaviour, per-vehicle-type support, etc.) are tested in
# tests/command/nav_takeoff/test_flight.py, not here.


def _takeoff_cmd(**overrides) -> dict:
    """Return default COMMAND_INT kwargs for MAV_CMD_NAV_TAKEOFF."""
    defaults = dict(
        command=_NAV_TAKEOFF_CMD_ID,
        frame=6,       # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        param1=0.0,    # Pitch: 0 deg (use default)
        param2=0.0,    # Unused
        param3=0.0,    # Flags: none
        param4=0.0,    # Yaw: 0.0 = north (None encodes as NaN — "use current heading")
        x=0,           # lat: 0 -> most stacks treat as "use current position"
        y=0,           # lon: 0
        z=float(TAKEOFF_ALT_M),
    )
    defaults.update(overrides)
    return defaults


async def _arm_and_send_takeoff(system, **overrides) -> None:
    """
    Wait for armable -> arm -> send NAV_TAKEOFF COMMAND_INT.

    PX4 treats the z field in COMMAND_INT as absolute AMSL altitude (it
    ignores the frame field). The caller passes z as a RELATIVE altitude
    (above home); this function adds the home AMSL altitude to produce the
    correct absolute z.

    Special cases:
    - z=None: passed as-is (NaN on wire -> stack uses its default altitude).
    - z=0.0: treated literally (0m AMSL) for a zero-altitude observation test.

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
