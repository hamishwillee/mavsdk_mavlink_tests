"""
MAV_CMD_DO_REPOSITION (cmd=192) via COMMAND_INT — Tier 2 execution tests.

Sends DO_REPOSITION with various parameter combinations and observes vehicle
behaviour.  All tests skip in paired/mock mode.

Flight test structure
---------------------
The primary test ``test_do_reposition_comprehensive`` arms the vehicle, takes off,
then runs all observations in a single flight cycle without landing between stages.
This minimises total test time.

Each observation stage also exists as a standalone test function that can be run
independently (does its own arm/takeoff/RTL cycle).

Mode discovery
--------------
Before each test the suite finds a "hold-equivalent" mode using the standard modes
protocol (AVAILABLE_MODES, MAV_CMD_DO_SET_STANDARD_MODE) without hardcoding stack-
specific mode names.  If the standard modes protocol is not supported, it falls back
to scanning custom mode names for "hold", "guided", "loiter", "course".

Running
-------
PX4 MC (patched dakejahl/do-reposition-ack)::

    pytest tests/command/do_reposition/test_flight.py \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \\
        --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO

ArduCopter::

    pytest tests/command/do_reposition/test_flight.py \\
        --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \\
        --ardupilot-sitl=~/ardu_sitl/arducopter \\
        --home-lat=47.3977 --home-lon=8.5456 --home-alt=0 \\
        --vehicle-type=quadcopter --autopilot=ardupilot -v --log-cli-level=INFO
"""

import asyncio
import json
import logging
import math
import time as _time_m

import pytest
from mavsdk import System
from mavsdk.mavlink_direct import MavlinkMessage
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
from tests.mock_flight_stack import MAV_RESULT_ACCEPTED, MAV_RESULT_DENIED, MAV_RESULT_UNSUPPORTED

log = logging.getLogger(__name__)

pytestmark = pytest.mark.timeout(900)

_CMD    = "DO_REPOSITION"
_CMD_ID = 192  # MAV_CMD_DO_REPOSITION

# Takeoff parameters for getting vehicle airborne before reposition tests
_INITIAL_ALT_M    = 20.0   # relative altitude for initial takeoff
_INITIAL_THRESH_M = 17.0   # 85% of _INITIAL_ALT_M
_INITIAL_TIMEOUT_S = 90.0

# Reposition parameters
_NEAR_DIST_M    = 5.0    # "arrived" threshold for position tests
_MOVE_DIST_M    = 50.0   # lateral movement target for position tests
_ARRIVE_TIMEOUT_S = 60.0
_YAW_TOLERANCE_RAD = 0.3  # ~17°

# Speed test parameters
_SPEED_DIST_M  = 100.0  # total distance for speed tests
_SPEED_HALF_M  = 50.0   # measure time until vehicle covers half distance
_SLOW_SPEED    = 2.0    # m/s (slow)
_FAST_SPEED    = 10.0   # m/s (fast)

# MAV_STANDARD_MODE values (from common.xml)
_MAV_STANDARD_MODE_NON_STANDARD = 0
_MAV_STANDARD_MODE_POSITION_HOLD = 1

# MAV_DO_REPOSITION_FLAGS
_FLAG_CHANGE_MODE  = 1
_FLAG_RELATIVE_YAW = 2

# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _dist_m(lat1_deg: float, lon1_deg: float,
            lat2_deg: float, lon2_deg: float) -> float:
    """Flat-earth distance in metres (accurate to < 0.1% for distances < 10 km)."""
    mid_lat = math.radians((lat1_deg + lat2_deg) / 2)
    dlat = (lat2_deg - lat1_deg) * 111111.0
    dlon = (lon2_deg - lon1_deg) * 111111.0 * math.cos(mid_lat)
    return math.sqrt(dlat**2 + dlon**2)


def _offset_lat_lon(lat_deg: float, lon_deg: float,
                    north_m: float, east_m: float) -> tuple[float, float]:
    """Return (lat, lon) displaced by north_m and east_m from the given point."""
    lat = lat_deg + north_m / 111111.0
    lon = lon_deg + east_m / (111111.0 * math.cos(math.radians(lat_deg)))
    return lat, lon


# ---------------------------------------------------------------------------
# Telemetry helpers (shared with takeoff/test_flight.py pattern)
# ---------------------------------------------------------------------------

async def _request_position_stream(system: System, rate_hz: float = 5.0) -> None:
    """Request GLOBAL_POSITION_INT streaming (ArduCopter doesn't stream by default)."""
    interval_us = int(1_000_000 / rate_hz)
    await system.mavlink_direct.send_message(MavlinkMessage(
        message_name="COMMAND_LONG",
        system_id=255, component_id=1,
        target_system_id=1, target_component_id=0,
        fields_json=json.dumps({
            "target_system": 1, "target_component": 0,
            "command": 511,   # MAV_CMD_SET_MESSAGE_INTERVAL
            "param1": 33.0,  # MAVLINK_MSG_ID_GLOBAL_POSITION_INT
            "param2": float(interval_us),
            "param3": 0.0, "param4": 0.0, "param5": 0.0,
            "param6": 0.0, "param7": 0.0,
            "confirmation": 0,
        }),
    ))
    await asyncio.sleep(0.2)


async def _get_home_position(system: System, timeout_s: float = 30.0):
    """Return the vehicle's home Position from telemetry."""
    async with asyncio.timeout(timeout_s):
        async for home in system.telemetry.home():
            return home
    raise TimeoutError("Home position not received")


async def _get_position(system: System, timeout_s: float = 5.0):
    """Return current Position."""
    async with asyncio.timeout(timeout_s):
        async for pos in system.telemetry.position():
            return pos
    raise TimeoutError("Position not received")


async def _get_heading(system: System, timeout_s: float = 5.0) -> float:
    """Return current heading in degrees."""
    async with asyncio.timeout(timeout_s):
        async for hdg in system.telemetry.heading():
            return hdg.heading_deg
    raise TimeoutError("Heading not received")


async def _get_flight_mode(system: System, timeout_s: float = 5.0) -> str:
    """Return current flight mode as string."""
    async with asyncio.timeout(timeout_s):
        async for fm in system.telemetry.flight_mode():
            return str(fm)
    raise TimeoutError("Flight mode not received")


async def _wait_armable(system: System, timeout_s: float = 60.0) -> None:
    """Block until is_armable=True (fire-and-forget pattern per §4a)."""
    event = asyncio.Event()

    async def _watch() -> None:
        async for health in system.telemetry.health():
            if health.is_armable:
                event.set()
                return

    task = asyncio.create_task(_watch())
    try:
        await asyncio.wait_for(event.wait(), timeout=timeout_s)
    finally:
        task.cancel()


async def _wait_for_altitude(system: System, threshold_m: float,
                              timeout_s: float = 60.0):
    """Block until relative_altitude_m >= threshold_m; return Position."""
    async with asyncio.timeout(timeout_s):
        async for pos in system.telemetry.position():
            if pos.relative_altitude_m >= threshold_m:
                return pos
    raise TimeoutError(f"Altitude {threshold_m:.1f} m not reached in {timeout_s:.0f} s")


async def _wait_for_horizontal_position(
    system: System,
    target_lat: float, target_lon: float,
    threshold_m: float,
    timeout_s: float = _ARRIVE_TIMEOUT_S,
):
    """Block until vehicle is within threshold_m of target lat/lon; return Position."""
    async with asyncio.timeout(timeout_s):
        async for pos in system.telemetry.position():
            d = _dist_m(pos.latitude_deg, pos.longitude_deg, target_lat, target_lon)
            if d <= threshold_m:
                return pos
    raise TimeoutError(
        f"Did not reach target within {threshold_m:.0f} m in {timeout_s:.0f} s"
    )


async def _rtl_and_land(system: System, timeout_s: float = 120.0) -> None:
    """Command RTL and wait for landed state; then disarm. Best-effort."""
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


# ---------------------------------------------------------------------------
# Standard modes protocol helper
# ---------------------------------------------------------------------------

async def _get_available_modes(system: System, timeout_s: float = 3.0) -> list[dict]:
    """
    Request AVAILABLE_MODES (msg 435) via MAV_CMD_REQUEST_MESSAGE(512) param1=435.

    Returns list of mode dicts with keys: standard_mode, custom_mode, mode_name.
    Returns empty list if the stack does not support the standard modes protocol.
    """
    modes: list[dict] = []
    expected_count: int | None = None
    complete = asyncio.Event()

    async def _collect() -> None:
        nonlocal expected_count
        async for msg in system.mavlink_direct.message("AVAILABLE_MODES"):
            fields = json.loads(msg.fields_json)
            modes.append({
                "standard_mode": int(fields.get("standard_mode", 0)),
                "custom_mode":   int(fields.get("custom_mode", 0)),
                "mode_name":     str(fields.get("mode_name", "")).rstrip("\x00"),
                "number_modes":  int(fields.get("number_modes", 0)),
                "mode_index":    int(fields.get("mode_index", 0)),
            })
            if expected_count is None:
                expected_count = int(fields.get("number_modes", 0))
            if len(modes) >= expected_count:
                complete.set()
                return

    task = asyncio.create_task(_collect())
    await asyncio.sleep(0.05)

    await system.mavlink_direct.send_message(MavlinkMessage(
        message_name="COMMAND_LONG",
        system_id=255, component_id=1,
        target_system_id=1, target_component_id=0,
        fields_json=json.dumps({
            "target_system": 1, "target_component": 0,
            "command": 512,    # MAV_CMD_REQUEST_MESSAGE
            "param1": 435.0,  # MAVLINK_MSG_ID_AVAILABLE_MODES
            "param2": 0.0,    # 0 = emit for all available modes
            "param3": 0.0, "param4": 0.0, "param5": 0.0,
            "param6": 0.0, "param7": 0.0,
            "confirmation": 0,
        }),
    ))

    try:
        await asyncio.wait_for(complete.wait(), timeout=timeout_s)
    except asyncio.TimeoutError:
        pass
    finally:
        task.cancel()

    return modes


async def _find_and_enter_hold_mode(system: System) -> dict | None:
    """
    Find a hold-equivalent mode and switch to it.

    Protocol:
    1. Request AVAILABLE_MODES to enumerate supported modes.
    2. Prefer MAV_STANDARD_MODE_POSITION_HOLD (value=1) — switch via DO_SET_STANDARD_MODE(262).
    3. If not found by standard mode, scan mode names for "hold", "guided", "loiter", "course"
       (case-insensitive) — switch via DO_SET_MODE (176).
    4. Verify the mode switch was ACCEPTED.

    Returns dict with the selected mode info, or None if no suitable mode found.
    """
    _HOLD_NAMES = ("hold", "guided", "loiter", "course", "posctl")

    modes = await _get_available_modes(system)

    if modes:
        log.info(_FMT, _CMD, "AVAILABLE_MODES", f"received {len(modes)} mode(s)")
        for m in modes:
            log.info(_FMT, _CMD, "  mode",
                     f"standard={m['standard_mode']}  custom={m['custom_mode']}  "
                     f"name={m['mode_name']!r}")

        # Step 2: look for standard POSITION_HOLD first
        for m in modes:
            if m["standard_mode"] == _MAV_STANDARD_MODE_POSITION_HOLD:
                ack = await probe_command_long(
                    system, 262, param1=float(_MAV_STANDARD_MODE_POSITION_HOLD)
                )
                if ack and int(ack["result"]) == MAV_RESULT_ACCEPTED:
                    log.info(_FMT, _CMD, "hold mode", "entered via DO_SET_STANDARD_MODE POSITION_HOLD")
                    return {**m, "method": "DO_SET_STANDARD_MODE"}
                break

        # Step 3: scan custom mode names
        for m in modes:
            name_lower = m["mode_name"].lower()
            if any(kw in name_lower for kw in _HOLD_NAMES):
                ack = await probe_command_long(
                    system, 176,
                    param1=1.0,                   # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                    param2=float(m["custom_mode"]),
                )
                if ack and int(ack["result"]) == MAV_RESULT_ACCEPTED:
                    log.info(_FMT, _CMD, "hold mode",
                             f"entered custom mode {m['mode_name']!r} via DO_SET_MODE")
                    return {**m, "method": "DO_SET_MODE"}

    else:
        log.info(_FMT, _CMD, "AVAILABLE_MODES", "no response — trying DO_SET_STANDARD_MODE directly")
        # Try DO_SET_STANDARD_MODE POSITION_HOLD directly even without the list
        ack = await probe_command_long(
            system, 262, param1=float(_MAV_STANDARD_MODE_POSITION_HOLD)
        )
        if ack and int(ack["result"]) == MAV_RESULT_ACCEPTED:
            log.info(_FMT, _CMD, "hold mode", "entered via DO_SET_STANDARD_MODE (no AVAILABLE_MODES)")
            return {
                "standard_mode": _MAV_STANDARD_MODE_POSITION_HOLD,
                "custom_mode": 0,
                "mode_name": "POSITION_HOLD",
                "method": "DO_SET_STANDARD_MODE",
            }

    log.warning(_FMT, _CMD, "hold mode", "no hold-equivalent mode found on this stack")
    return None


# ---------------------------------------------------------------------------
# Arm and takeoff helper
# ---------------------------------------------------------------------------

async def _arm_and_takeoff(system: System, alt_m: float = _INITIAL_ALT_M) -> "Position":
    """
    Arm the vehicle and take off to alt_m relative altitude.

    Returns the home Position (for computing reposition targets).
    Raises TimeoutError if the vehicle does not reach the threshold altitude.
    """
    await _request_position_stream(system)
    home = await _get_home_position(system, timeout_s=60.0)
    log.info(_FMT, _CMD, "home",
             f"lat={home.latitude_deg:.5f}° lon={home.longitude_deg:.5f}° "
             f"amsl={home.absolute_altitude_m:.1f} m")

    await _wait_armable(system, timeout_s=60.0)
    await system.action.arm()
    await asyncio.sleep(0.5)

    # PX4: COMMAND_INT with absolute AMSL z (ignores frame); ArduCopter: relative z
    cmd_z     = home.absolute_altitude_m + alt_m
    cmd_frame = 5  # GLOBAL_INT absolute (PX4); also accepted by ArduCopter

    await send_command_int(
        system,
        command=22,    # MAV_CMD_NAV_TAKEOFF
        frame=cmd_frame,
        param1=0.0, param2=0.0, param3=0.0, param4=None,
        x=int(home.latitude_deg * 1e7),
        y=int(home.longitude_deg * 1e7),
        z=cmd_z,
    )

    pos = await _wait_for_altitude(system, alt_m * 0.85, _INITIAL_TIMEOUT_S)
    log.info(_FMT, _CMD, "airborne", f"altitude={pos.relative_altitude_m:.1f} m")
    return home


# ---------------------------------------------------------------------------
# DO_REPOSITION send helpers
# ---------------------------------------------------------------------------

def _reposition_cmd(**overrides) -> dict:
    """Return default COMMAND_INT kwargs for DO_REPOSITION."""
    defaults = dict(
        command = _CMD_ID,
        frame   = 6,       # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        param1  = -1.0,
        param2  = 1.0,     # CHANGE_MODE
        param3  = 0.0,
        param4  = None,    # NaN yaw
        x       = 0,
        y       = 0,
        z       = _INITIAL_ALT_M,
    )
    defaults.update(overrides)
    return defaults


async def _send_reposition(system: System, **overrides) -> dict | None:
    """Probe DO_REPOSITION with given params; return ACK dict or None."""
    return await probe_command_int(system, **_reposition_cmd(**overrides))


# ---------------------------------------------------------------------------
# Autouse skip fixture
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def require_real_stack(request):
    """Skip every test in this module when no --drone-address is given."""
    if request.config.getoption("--drone-address") is None:
        pytest.skip("Execution tests require a real flight stack (--drone-address not set)")


# ---------------------------------------------------------------------------
# Comprehensive single-flight test
# ---------------------------------------------------------------------------

@pytest.mark.timeout(900)
async def test_do_reposition_comprehensive(gcs_system, request):
    """
    DO_REPOSITION comprehensive single-flight test.

    Arms, takes off, then runs all parameter observations in one flight:
      Stage 1: mode gating — confirm param2=0 DENIED in AUTO (post-takeoff) mode
      Stage 2: CHANGE_MODE flag — enter hold-equivalent mode
      Stage 3: reposition to new lat/lon (same altitude)
      Stage 4: NaN altitude (keep current altitude)
      Stage 5: altitude-only reposition (INT32_MAX lat/lon)
      Stage 6: NaN lat/lon via COMMAND_LONG (keep current position)
      Stage 7: speed — confirm param1=-1 and NaN both use default
      Stage 8: speed — slow vs fast ratio test
      Stage 9: yaw (vehicle-type specific: MC absolute/relative; FW loiter direction)
      Stage 10: loiter radius (FW: honoured; MC: DENIED)
      Stage 11: all-NaN pause
      Stage 12: RTL
    """
    vehicle_type = request.config.getoption("--vehicle-type", default=None) or "unknown"
    autopilot    = request.config.getoption("--autopilot", default=None) or "unknown"
    is_fw        = vehicle_type in ("fixed_wing", "plane")
    is_mc        = vehicle_type in ("quadcopter", "copter", "helicopter", "multirotor")

    # ACK probe — skip if UNSUPPORTED on this stack
    ack_probe = await probe_command_int(gcs_system, **_reposition_cmd())
    if ack_probe is not None and int(ack_probe["result"]) == MAV_RESULT_UNSUPPORTED:
        pytest.skip(f"{_CMD} UNSUPPORTED on this platform")

    obs: dict = {
        "stages": [],
        "hold_mode": None,
        "yaw_supported": None,
        "radius_fw_honoured": None,
    }

    def _record(stage: str, detail: str, passed: bool | None = None) -> None:
        marker = "PASS" if passed is True else ("FAIL" if passed is False else "OBS")
        log.info(_FMT, _CMD, f"[{marker}] {stage}", detail)
        obs["stages"].append({"stage": stage, "marker": marker, "detail": detail})

    try:
        # ── Stage 0: Arm and initial takeoff ──────────────────────────────────
        home = await _arm_and_takeoff(gcs_system, _INITIAL_ALT_M)
        home_lat = home.latitude_deg
        home_lon = home.longitude_deg

        # ── Stage 1: Mode gating — param2=0 should be DENIED in auto mode ─────
        # After takeoff, vehicle is in some AUTO mode (not Hold).
        ack1 = await _send_reposition(gcs_system, param2=0.0,
                                       x=int(home_lat * 1e7), y=int(home_lon * 1e7),
                                       z=_INITIAL_ALT_M)
        if ack1 is not None:
            r1 = int(ack1["result"])
            passed1 = (r1 == MAV_RESULT_DENIED)
            _record("stage1:mode-gating param2=0",
                    f"result={r1}  (expected DENIED in non-hold mode)", passed1)
            if not passed1:
                log.warning(_FMT, _CMD, "stage1", f"expected DENIED but got {r1} — "
                            "stack may not gate DO_REPOSITION on mode")
        else:
            _record("stage1:mode-gating param2=0", "UNKNOWN — no ACK")

        # ── Stage 2: CHANGE_MODE flag — enter hold mode ────────────────────────
        hold_mode = await _find_and_enter_hold_mode(gcs_system)
        obs["hold_mode"] = hold_mode
        if hold_mode is None:
            _record("stage2:find-hold-mode", "no hold-equivalent mode found — skip reposition", False)
            return

        ack2 = await _send_reposition(gcs_system, param2=float(_FLAG_CHANGE_MODE),
                                       x=int(home_lat * 1e7), y=int(home_lon * 1e7),
                                       z=_INITIAL_ALT_M)
        r2 = int(ack2["result"]) if ack2 else -1
        passed2 = (r2 == MAV_RESULT_ACCEPTED)
        _record("stage2:CHANGE_MODE accepted", f"result={r2}", passed2)
        if not passed2:
            _record("stage2", "CHANGE_MODE not accepted — abort remaining stages", False)
            return
        await asyncio.sleep(2.0)  # allow mode switch to settle

        # ── Stage 3: Reposition to new lat/lon (same altitude) ─────────────────
        pos_before = await _get_position(gcs_system)
        tgt_lat, tgt_lon = _offset_lat_lon(pos_before.latitude_deg, pos_before.longitude_deg,
                                            _MOVE_DIST_M, 0.0)  # 50 m north
        tgt_z = pos_before.relative_altitude_m
        ack3 = await _send_reposition(gcs_system, param2=0.0,  # already in Hold
                                       x=int(tgt_lat * 1e7), y=int(tgt_lon * 1e7),
                                       z=tgt_z)
        r3 = int(ack3["result"]) if ack3 else -1
        if r3 == MAV_RESULT_ACCEPTED:
            try:
                pos_after = await _wait_for_horizontal_position(
                    gcs_system, tgt_lat, tgt_lon, _NEAR_DIST_M, _ARRIVE_TIMEOUT_S
                )
                d = _dist_m(pos_after.latitude_deg, pos_after.longitude_deg, tgt_lat, tgt_lon)
                alt_drift = abs(pos_after.relative_altitude_m - tgt_z)
                passed3 = (d <= _NEAR_DIST_M)
                _record("stage3:reposition-to-location",
                        f"dist_from_target={d:.1f} m  alt_drift={alt_drift:.1f} m",
                        passed3)
            except TimeoutError:
                _record("stage3:reposition-to-location",
                        f"vehicle did not reach target ({_MOVE_DIST_M:.0f} m N) in {_ARRIVE_TIMEOUT_S:.0f} s",
                        False)
        else:
            _record("stage3:reposition-to-location", f"ACK={r3} (not ACCEPTED)", None)
        await asyncio.sleep(1.0)

        # ── Stage 4: NaN altitude — keep current altitude ──────────────────────
        pos_before4 = await _get_position(gcs_system)
        tgt4_lat, tgt4_lon = _offset_lat_lon(pos_before4.latitude_deg, pos_before4.longitude_deg,
                                              0.0, _MOVE_DIST_M)  # 50 m east
        ack4 = await probe_command_long(
            gcs_system, _CMD_ID,
            param1=-1.0, param2=float(_FLAG_CHANGE_MODE), param3=0.0,
            param4=None,                                        # NaN yaw
            param5=tgt4_lat, param6=tgt4_lon, param7=None,     # NaN alt
        )
        r4 = int(ack4["result"]) if ack4 else -1
        if r4 not in (MAV_RESULT_ACCEPTED, 1):
            _record("stage4:NaN-altitude", f"ACK={r4} — cannot test altitude preservation", None)
        else:
            try:
                pos_after4 = await _wait_for_horizontal_position(
                    gcs_system, tgt4_lat, tgt4_lon, _NEAR_DIST_M, _ARRIVE_TIMEOUT_S
                )
                alt_drift4 = abs(pos_after4.relative_altitude_m - pos_before4.relative_altitude_m)
                passed4_pos  = _dist_m(pos_after4.latitude_deg, pos_after4.longitude_deg,
                                       tgt4_lat, tgt4_lon) <= _NEAR_DIST_M
                passed4_alt  = alt_drift4 <= 3.0
                _record("stage4:NaN-altitude (keep current alt)",
                        f"moved_east={passed4_pos}  alt_drift={alt_drift4:.1f} m "
                        f"(should be ≤3 m)", passed4_alt and passed4_pos)
            except TimeoutError:
                _record("stage4:NaN-altitude", "vehicle did not move to east target", False)
        await asyncio.sleep(1.0)

        # ── Stage 5: Altitude-only (INT32_MAX lat/lon) ─────────────────────────
        pos_before5 = await _get_position(gcs_system)
        new_alt5 = pos_before5.relative_altitude_m + 20.0
        ack5 = await _send_reposition(gcs_system, param2=float(_FLAG_CHANGE_MODE),
                                       x=INT32_MAX, y=INT32_MAX, z=new_alt5)
        r5 = int(ack5["result"]) if ack5 else -1
        if r5 != MAV_RESULT_ACCEPTED:
            _record("stage5:altitude-only (INT32_MAX lat/lon)", f"ACK={r5} — not tested", None)
        else:
            try:
                pos_after5 = await _wait_for_altitude(gcs_system, new_alt5 * 0.85, 30.0)
                pos_drift5 = _dist_m(pos_after5.latitude_deg, pos_after5.longitude_deg,
                                     pos_before5.latitude_deg, pos_before5.longitude_deg)
                passed5_alt  = pos_after5.relative_altitude_m >= new_alt5 * 0.85
                passed5_pos  = pos_drift5 <= 10.0
                _record("stage5:altitude-only",
                        f"alt={pos_after5.relative_altitude_m:.1f}/{new_alt5:.1f} m  "
                        f"pos_drift={pos_drift5:.1f} m (should be ≤10 m)",
                        passed5_alt and passed5_pos)
            except TimeoutError:
                _record("stage5:altitude-only", "altitude not reached in 30 s", False)
        await asyncio.sleep(1.0)

        # ── Stage 6: NaN lat/lon COMMAND_LONG — keep current position ──────────
        pos_before6 = await _get_position(gcs_system)
        ack6 = await probe_command_long(
            gcs_system, _CMD_ID,
            param1=-1.0, param2=float(_FLAG_CHANGE_MODE), param3=0.0,
            param4=None, param5=None, param6=None,
            param7=pos_before6.relative_altitude_m,
        )
        r6 = int(ack6["result"]) if ack6 else -1
        await asyncio.sleep(10.0)
        if r6 != MAV_RESULT_UNSUPPORTED:
            pos_after6 = await _get_position(gcs_system)
            drift6 = _dist_m(pos_after6.latitude_deg, pos_after6.longitude_deg,
                             pos_before6.latitude_deg, pos_before6.longitude_deg)
            passed6 = drift6 <= 5.0
            _record("stage6:NaN-lat/lon (keep position)",
                    f"drift={drift6:.1f} m (should be ≤5 m)  ACK={r6}",
                    passed6)
        else:
            _record("stage6:NaN-lat/lon COMMAND_LONG", "UNSUPPORTED", None)
        await asyncio.sleep(1.0)

        # ── Stage 7: Speed — param1=-1 and NaN should use same default ─────────
        pos7 = await _get_position(gcs_system)
        tgt7_lat, tgt7_lon = _offset_lat_lon(pos7.latitude_deg, pos7.longitude_deg,
                                              _SPEED_DIST_M, 0.0)  # 100 m north
        tgt7_half_lat, _ = _offset_lat_lon(pos7.latitude_deg, pos7.longitude_deg,
                                            _SPEED_HALF_M, 0.0)

        t_start7 = _time_m.monotonic()
        await _send_reposition(gcs_system, param2=float(_FLAG_CHANGE_MODE), param1=-1.0,
                                x=int(tgt7_lat * 1e7), y=int(tgt7_lon * 1e7),
                                z=pos7.relative_altitude_m)
        try:
            await _wait_for_horizontal_position(gcs_system, tgt7_half_lat, pos7.longitude_deg,
                                                 5.0, timeout_s=40.0)
            t_default = _time_m.monotonic() - t_start7
        except TimeoutError:
            t_default = float("nan")

        # Return to start (roughly), then test NaN speed
        await _send_reposition(gcs_system, param2=float(_FLAG_CHANGE_MODE),
                                x=int(pos7.latitude_deg * 1e7), y=int(pos7.longitude_deg * 1e7),
                                z=pos7.relative_altitude_m)
        await asyncio.sleep(10.0)

        t_start7n = _time_m.monotonic()
        await probe_command_long(gcs_system, _CMD_ID,
                                  param1=None,   # NaN
                                  param2=float(_FLAG_CHANGE_MODE),
                                  param3=0.0, param4=None,
                                  param5=tgt7_lat, param6=tgt7_lon,
                                  param7=pos7.relative_altitude_m)
        try:
            await _wait_for_horizontal_position(gcs_system, tgt7_half_lat, pos7.longitude_deg,
                                                 5.0, timeout_s=40.0)
            t_nan = _time_m.monotonic() - t_start7n
        except TimeoutError:
            t_nan = float("nan")

        if math.isfinite(t_default) and math.isfinite(t_nan):
            ratio7 = max(t_default, t_nan) / min(t_default, t_nan)
            passed7 = ratio7 < 1.3   # within 30% = same default speed
            _record("stage7:speed -1 vs NaN",
                    f"t_default={t_default:.1f}s  t_nan={t_nan:.1f}s  ratio={ratio7:.2f}",
                    passed7)
        else:
            _record("stage7:speed -1 vs NaN",
                    f"t_default={t_default:.1f}s  t_nan={t_nan:.1f}s — could not measure", None)
        await asyncio.sleep(1.0)

        # ── Stage 8: Speed — slow vs fast ratio ────────────────────────────────
        pos8 = await _get_position(gcs_system)
        tgt8_lat, tgt8_lon = _offset_lat_lon(pos8.latitude_deg, pos8.longitude_deg,
                                              _SPEED_DIST_M, 0.0)
        tgt8_half_lat, _ = _offset_lat_lon(pos8.latitude_deg, pos8.longitude_deg,
                                            _SPEED_HALF_M, 0.0)

        t_start8s = _time_m.monotonic()
        await _send_reposition(gcs_system, param2=float(_FLAG_CHANGE_MODE), param1=_SLOW_SPEED,
                                x=int(tgt8_lat * 1e7), y=int(tgt8_lon * 1e7),
                                z=pos8.relative_altitude_m)
        try:
            await _wait_for_horizontal_position(gcs_system, tgt8_half_lat, pos8.longitude_deg,
                                                 5.0, timeout_s=60.0)
            t_slow = _time_m.monotonic() - t_start8s
        except TimeoutError:
            t_slow = float("nan")

        await _send_reposition(gcs_system, param2=float(_FLAG_CHANGE_MODE),
                                x=int(pos8.latitude_deg * 1e7), y=int(pos8.longitude_deg * 1e7),
                                z=pos8.relative_altitude_m)
        await asyncio.sleep(10.0)

        t_start8f = _time_m.monotonic()
        await _send_reposition(gcs_system, param2=float(_FLAG_CHANGE_MODE), param1=_FAST_SPEED,
                                x=int(tgt8_lat * 1e7), y=int(tgt8_lon * 1e7),
                                z=pos8.relative_altitude_m)
        try:
            await _wait_for_horizontal_position(gcs_system, tgt8_half_lat, pos8.longitude_deg,
                                                 5.0, timeout_s=30.0)
            t_fast = _time_m.monotonic() - t_start8f
        except TimeoutError:
            t_fast = float("nan")

        if math.isfinite(t_slow) and math.isfinite(t_fast) and t_fast > 0:
            ratio8 = t_slow / t_fast
            passed8 = ratio8 > 1.5
            _record("stage8:speed slow vs fast",
                    f"t_slow={t_slow:.1f}s ({_SLOW_SPEED} m/s)  "
                    f"t_fast={t_fast:.1f}s ({_FAST_SPEED} m/s)  ratio={ratio8:.2f}",
                    passed8)
        else:
            _record("stage8:speed slow vs fast",
                    f"t_slow={t_slow:.1f}s  t_fast={t_fast:.1f}s — could not measure", None)
        await asyncio.sleep(1.0)

        # ── Stage 9: Yaw ──────────────────────────────────────────────────────
        if is_mc:
            # MC: param4 is absolute yaw heading in radians
            pos9 = await _get_position(gcs_system)
            heading_before9 = await _get_heading(gcs_system)

            # 9a: absolute yaw (param4 = π/2 = East)
            yaw_target_rad = math.pi / 2   # East
            ack9a = await _send_reposition(gcs_system,
                                            param2=float(_FLAG_CHANGE_MODE),
                                            param4=yaw_target_rad,
                                            x=int(pos9.latitude_deg * 1e7),
                                            y=int(pos9.longitude_deg * 1e7),
                                            z=pos9.relative_altitude_m)
            await asyncio.sleep(10.0)
            heading9a = await _get_heading(gcs_system)
            yaw_target_deg = math.degrees(yaw_target_rad)
            diff9a = abs((heading9a - yaw_target_deg + 180) % 360 - 180)
            passed9a = diff9a <= math.degrees(_YAW_TOLERANCE_RAD)
            obs["yaw_supported"] = passed9a
            _record("stage9a:yaw absolute (π/2=East)",
                    f"commanded={yaw_target_deg:.0f}°  actual={heading9a:.1f}°  "
                    f"diff={diff9a:.1f}° (tolerance={math.degrees(_YAW_TOLERANCE_RAD):.0f}°)",
                    passed9a)

            # 9b: NaN yaw — should not force a specific heading
            ack9b = await _send_reposition(gcs_system,
                                            param2=float(_FLAG_CHANGE_MODE),
                                            param4=None,   # NaN
                                            x=int(pos9.latitude_deg * 1e7),
                                            y=int(pos9.longitude_deg * 1e7),
                                            z=pos9.relative_altitude_m)
            await asyncio.sleep(8.0)
            heading9b = await _get_heading(gcs_system)
            _record("stage9b:yaw NaN (current heading mode)",
                    f"heading={heading9b:.1f}° (system-defined; not asserted)", None)

            # 9c: relative yaw (RELATIVE_YAW flag) — param4 = π/4 offset from current
            heading_before9c = await _get_heading(gcs_system)
            yaw_offset_rad = math.pi / 4
            ack9c = await _send_reposition(gcs_system,
                                            param2=float(_FLAG_CHANGE_MODE | _FLAG_RELATIVE_YAW),
                                            param4=yaw_offset_rad,
                                            x=int(pos9.latitude_deg * 1e7),
                                            y=int(pos9.longitude_deg * 1e7),
                                            z=pos9.relative_altitude_m)
            await asyncio.sleep(10.0)
            heading9c = await _get_heading(gcs_system)
            expected9c_deg = (heading_before9c + math.degrees(yaw_offset_rad)) % 360
            diff9c = abs((heading9c - expected9c_deg + 180) % 360 - 180)
            abs_diff9c = abs((heading9c - math.degrees(yaw_offset_rad) + 180) % 360 - 180)
            if diff9c <= math.degrees(_YAW_TOLERANCE_RAD):
                _record("stage9c:RELATIVE_YAW",
                        f"before={heading_before9c:.1f}°  offset={math.degrees(yaw_offset_rad):.0f}°  "
                        f"expected={expected9c_deg:.1f}°  actual={heading9c:.1f}° → RELATIVE honoured",
                        True)
            elif abs_diff9c <= math.degrees(_YAW_TOLERANCE_RAD):
                _record("stage9c:RELATIVE_YAW",
                        f"heading={heading9c:.1f}° matches ABSOLUTE π/4 — "
                        "RELATIVE_YAW flag ignored (spec violation)",
                        False)
            else:
                _record("stage9c:RELATIVE_YAW",
                        f"before={heading_before9c:.1f}°  actual={heading9c:.1f}° — "
                        "heading did not match either relative or absolute expectation", None)

        elif is_fw:
            # FW: param4 indicates loiter direction (0=CW, 1=CCW)
            pos9fw = await _get_position(gcs_system)

            # 9a: CW loiter (param4=0)
            await _send_reposition(gcs_system, param2=float(_FLAG_CHANGE_MODE),
                                    param4=0.0,
                                    x=int(pos9fw.latitude_deg * 1e7),
                                    y=int(pos9fw.longitude_deg * 1e7),
                                    z=pos9fw.relative_altitude_m)
            await asyncio.sleep(15.0)
            # Sample bearing change to determine direction
            p_a = await _get_position(gcs_system)
            await asyncio.sleep(5.0)
            p_b = await _get_position(gcs_system)
            bearing_a = math.degrees(math.atan2(p_a.longitude_deg - pos9fw.longitude_deg,
                                                 p_a.latitude_deg - pos9fw.latitude_deg))
            bearing_b = math.degrees(math.atan2(p_b.longitude_deg - pos9fw.longitude_deg,
                                                 p_b.latitude_deg - pos9fw.latitude_deg))
            d_bearing_cw = (bearing_b - bearing_a + 360) % 360
            cw = d_bearing_cw < 180
            _record("stage9a-fw:loiter direction param4=0 (CW expected)",
                    f"bearing change = {d_bearing_cw:.0f}° → {'CW' if cw else 'CCW'}",
                    cw)

            # 9b: CCW loiter (param4=1)
            await _send_reposition(gcs_system, param2=float(_FLAG_CHANGE_MODE),
                                    param4=1.0,
                                    x=int(pos9fw.latitude_deg * 1e7),
                                    y=int(pos9fw.longitude_deg * 1e7),
                                    z=pos9fw.relative_altitude_m)
            await asyncio.sleep(15.0)
            p_c = await _get_position(gcs_system)
            await asyncio.sleep(5.0)
            p_d = await _get_position(gcs_system)
            bearing_c = math.degrees(math.atan2(p_c.longitude_deg - pos9fw.longitude_deg,
                                                 p_c.latitude_deg - pos9fw.latitude_deg))
            bearing_d = math.degrees(math.atan2(p_d.longitude_deg - pos9fw.longitude_deg,
                                                 p_d.latitude_deg - pos9fw.latitude_deg))
            d_bearing_ccw = (bearing_d - bearing_c + 360) % 360
            ccw = d_bearing_ccw > 180
            _record("stage9b-fw:loiter direction param4=1 (CCW expected)",
                    f"bearing change = {d_bearing_ccw:.0f}° → {'CCW' if ccw else 'CW'}",
                    ccw)

            # 9c: NaN yaw — default direction
            await _send_reposition(gcs_system, param2=float(_FLAG_CHANGE_MODE),
                                    param4=None,
                                    x=int(pos9fw.latitude_deg * 1e7),
                                    y=int(pos9fw.longitude_deg * 1e7),
                                    z=pos9fw.relative_altitude_m)
            await asyncio.sleep(5.0)
            _record("stage9c-fw:loiter direction param4=NaN (system default)", "observational", None)

        else:
            _record("stage9:yaw", f"skipped for vehicle_type={vehicle_type!r}", None)
        await asyncio.sleep(1.0)

        # ── Stage 10: Loiter radius ────────────────────────────────────────────
        pos10 = await _get_position(gcs_system)

        if is_mc:
            # MC: non-zero/non-NaN radius cannot be honoured → expect DENIED
            ack10 = await _send_reposition(gcs_system,
                                            param2=float(_FLAG_CHANGE_MODE),
                                            param3=100.0,
                                            x=int(pos10.latitude_deg * 1e7),
                                            y=int(pos10.longitude_deg * 1e7),
                                            z=pos10.relative_altitude_m)
            r10 = int(ack10["result"]) if ack10 else -1
            passed10 = (r10 == MAV_RESULT_DENIED)
            _record("stage10-mc:radius 100 m (expect DENIED on MC)",
                    f"result={r10}", passed10)
            if not passed10:
                log.warning(_FMT, _CMD, "stage10", "MC accepted loiter radius — spec gap")

        elif is_fw:
            # FW: check that 0 and NaN do not change the radius; positive radius is honoured

            # 10a: radius=0 should leave radius unchanged
            await _send_reposition(gcs_system, param2=float(_FLAG_CHANGE_MODE),
                                    param3=0.0, x=int(pos10.latitude_deg * 1e7),
                                    y=int(pos10.longitude_deg * 1e7), z=pos10.relative_altitude_m)
            await asyncio.sleep(15.0)
            positions_r0 = []
            for _ in range(5):
                positions_r0.append(await _get_position(gcs_system))
                await asyncio.sleep(2.0)
            radii_r0 = [_dist_m(p.latitude_deg, p.longitude_deg,
                                 pos10.latitude_deg, pos10.longitude_deg)
                        for p in positions_r0]
            mean_r0 = sum(radii_r0) / len(radii_r0)

            # 10b: radius=200 m should produce a loiter at ~200 m
            await _send_reposition(gcs_system, param2=float(_FLAG_CHANGE_MODE),
                                    param3=200.0, x=int(pos10.latitude_deg * 1e7),
                                    y=int(pos10.longitude_deg * 1e7), z=pos10.relative_altitude_m)
            await asyncio.sleep(30.0)  # allow loiter to stabilise
            positions_r200 = []
            for _ in range(5):
                positions_r200.append(await _get_position(gcs_system))
                await asyncio.sleep(2.0)
            radii_r200 = [_dist_m(p.latitude_deg, p.longitude_deg,
                                   pos10.latitude_deg, pos10.longitude_deg)
                          for p in positions_r200]
            mean_r200 = sum(radii_r200) / len(radii_r200)
            passed10_r200 = abs(mean_r200 - 200.0) <= 50.0

            # 10c: radius=NaN should not change radius (same as before)
            await _send_reposition(gcs_system, param2=float(_FLAG_CHANGE_MODE),
                                    param3=None, x=int(pos10.latitude_deg * 1e7),
                                    y=int(pos10.longitude_deg * 1e7), z=pos10.relative_altitude_m)
            await asyncio.sleep(15.0)
            positions_rnan = []
            for _ in range(5):
                positions_rnan.append(await _get_position(gcs_system))
                await asyncio.sleep(2.0)
            radii_rnan = [_dist_m(p.latitude_deg, p.longitude_deg,
                                   pos10.latitude_deg, pos10.longitude_deg)
                          for p in positions_rnan]
            mean_rnan = sum(radii_rnan) / len(radii_rnan)

            _record("stage10-fw:radius=0 (unchanged from default)",
                    f"mean_radius={mean_r0:.0f} m", None)
            _record("stage10-fw:radius=200 m (honoured)",
                    f"mean_radius={mean_r200:.0f} m (expected ≈200±50 m)", passed10_r200)
            obs["radius_fw_honoured"] = passed10_r200
            # NaN should produce similar radius to 0 (both are "ignored")
            r_diff = abs(mean_rnan - mean_r0) / max(mean_r0, 1.0)
            passed10_nan = r_diff < 0.3
            _record("stage10-fw:radius=NaN (unchanged — same as 0)",
                    f"mean_radius_nan={mean_rnan:.0f} m  mean_radius_0={mean_r0:.0f} m  "
                    f"diff={r_diff:.0%}", passed10_nan)
        else:
            _record("stage10:radius", f"skipped for vehicle_type={vehicle_type!r}", None)
        await asyncio.sleep(1.0)

        # ── Stage 11: All-NaN pause ────────────────────────────────────────────
        # Get vehicle moving first (reposition to a target)
        pos11 = await _get_position(gcs_system)
        tgt11_lat, tgt11_lon = _offset_lat_lon(pos11.latitude_deg, pos11.longitude_deg,
                                                _MOVE_DIST_M, 0.0)
        await _send_reposition(gcs_system, param2=float(_FLAG_CHANGE_MODE),
                                x=int(tgt11_lat * 1e7), y=int(tgt11_lon * 1e7),
                                z=pos11.relative_altitude_m)
        await asyncio.sleep(5.0)  # let vehicle start moving

        pos_at_pause = await _get_position(gcs_system)
        await probe_command_long(
            gcs_system, _CMD_ID,
            param1=-1.0, param2=float(_FLAG_CHANGE_MODE), param3=0.0,
            param4=None, param5=None, param6=None, param7=None,
        )
        await asyncio.sleep(15.0)
        pos_after_pause = await _get_position(gcs_system)
        pause_drift = _dist_m(pos_after_pause.latitude_deg, pos_after_pause.longitude_deg,
                               pos_at_pause.latitude_deg, pos_at_pause.longitude_deg)
        passed11 = pause_drift <= 10.0
        _record("stage11:all-NaN pause",
                f"drift_after_pause={pause_drift:.1f} m (should be ≤10 m)",
                passed11)

    finally:
        await _rtl_and_land(gcs_system)

    # ── Summary ────────────────────────────────────────────────────────────────
    passed = sum(1 for s in obs["stages"] if s["marker"] == "PASS")
    failed = sum(1 for s in obs["stages"] if s["marker"] == "FAIL")
    observed = sum(1 for s in obs["stages"] if s["marker"] == "OBS")
    log.info(_FMT, _CMD, "COMPREHENSIVE SUMMARY",
             f"{passed} PASS  {failed} FAIL  {observed} OBS  "
             f"hold_mode={obs['hold_mode']['mode_name'] if obs['hold_mode'] else 'none'}  "
             f"yaw_supported={obs['yaw_supported']}  "
             f"fw_radius_honoured={obs['radius_fw_honoured']}")

    assert failed == 0, (
        f"{failed} stage(s) FAILED — see log above for details"
    )


# ---------------------------------------------------------------------------
# Individual standalone tests (arm/takeoff/RTL each)
# ---------------------------------------------------------------------------

async def _ensure_supported(system: System) -> None:
    """Skip if DO_REPOSITION is UNSUPPORTED on this stack."""
    ack = await probe_command_int(system, command=_CMD_ID, param2=1.0,
                                   x=0, y=0, z=20.0, frame=6)
    if ack is not None and int(ack["result"]) == MAV_RESULT_UNSUPPORTED:
        pytest.skip(f"{_CMD} UNSUPPORTED on this platform")


async def test_reposition_to_location(gcs_system):
    """Standalone: arm, take off, reposition 50 m north, assert vehicle arrives."""
    await _ensure_supported(gcs_system)
    try:
        home = await _arm_and_takeoff(gcs_system, _INITIAL_ALT_M)
        await _find_and_enter_hold_mode(gcs_system)
        pos = await _get_position(gcs_system)
        tgt_lat, tgt_lon = _offset_lat_lon(pos.latitude_deg, pos.longitude_deg, _MOVE_DIST_M, 0.0)
        ack = await _send_reposition(gcs_system, param2=0.0,
                                      x=int(tgt_lat * 1e7), y=int(tgt_lon * 1e7),
                                      z=pos.relative_altitude_m)
        result = int(ack["result"]) if ack else -1
        assert result == MAV_RESULT_ACCEPTED, f"Expected ACCEPTED; got {result}"
        pos_final = await _wait_for_horizontal_position(
            gcs_system, tgt_lat, tgt_lon, _NEAR_DIST_M, _ARRIVE_TIMEOUT_S
        )
        d = _dist_m(pos_final.latitude_deg, pos_final.longitude_deg, tgt_lat, tgt_lon)
        log.info(_FMT, _CMD, "reposition-to-location", f"dist_from_target={d:.1f} m")
        assert d <= _NEAR_DIST_M, f"Vehicle only got {d:.1f} m from target"
    finally:
        await _rtl_and_land(gcs_system)


async def test_altitude_only_reposition(gcs_system):
    """Standalone: INT32_MAX lat/lon with altitude change — altitude-only reposition."""
    await _ensure_supported(gcs_system)
    try:
        home = await _arm_and_takeoff(gcs_system, _INITIAL_ALT_M)
        await _find_and_enter_hold_mode(gcs_system)
        pos = await _get_position(gcs_system)
        new_alt = pos.relative_altitude_m + 20.0
        ack = await _send_reposition(gcs_system, param2=float(_FLAG_CHANGE_MODE),
                                      x=INT32_MAX, y=INT32_MAX, z=new_alt)
        result = int(ack["result"]) if ack else -1
        assert result == MAV_RESULT_ACCEPTED, f"Expected ACCEPTED; got {result}"
        pos_final = await _wait_for_altitude(gcs_system, new_alt * 0.85, 30.0)
        log.info(_FMT, _CMD, "altitude-only", f"alt={pos_final.relative_altitude_m:.1f} m")
        assert pos_final.relative_altitude_m >= new_alt * 0.85
    finally:
        await _rtl_and_land(gcs_system)


async def test_nan_altitude_keeps_current(gcs_system):
    """Standalone: NaN altitude in COMMAND_LONG should keep current altitude."""
    await _ensure_supported(gcs_system)
    try:
        home = await _arm_and_takeoff(gcs_system, _INITIAL_ALT_M)
        await _find_and_enter_hold_mode(gcs_system)
        pos = await _get_position(gcs_system)
        tgt_lat, tgt_lon = _offset_lat_lon(pos.latitude_deg, pos.longitude_deg, _MOVE_DIST_M, 0.0)
        ack = await probe_command_long(
            gcs_system, _CMD_ID,
            param1=-1.0, param2=float(_FLAG_CHANGE_MODE), param3=0.0,
            param4=None, param5=tgt_lat, param6=tgt_lon, param7=None,
        )
        result = int(ack["result"]) if ack else -1
        assert result not in (MAV_RESULT_UNSUPPORTED,), f"Command UNSUPPORTED"
        pos_final = await _wait_for_horizontal_position(
            gcs_system, tgt_lat, tgt_lon, _NEAR_DIST_M, _ARRIVE_TIMEOUT_S
        )
        alt_drift = abs(pos_final.relative_altitude_m - pos.relative_altitude_m)
        log.info(_FMT, _CMD, "NaN-altitude", f"alt_drift={alt_drift:.1f} m (should be ≤3 m)")
        assert alt_drift <= 3.0, f"Altitude drifted {alt_drift:.1f} m with NaN altitude"
    finally:
        await _rtl_and_land(gcs_system)


async def test_param2_zero_denied_in_auto_mode(gcs_system):
    """Standalone: param2=0 should be DENIED immediately after takeoff (not in Hold)."""
    await _ensure_supported(gcs_system)
    try:
        home = await _arm_and_takeoff(gcs_system, _INITIAL_ALT_M)
        pos = await _get_position(gcs_system)
        ack = await _send_reposition(gcs_system, param2=0.0,
                                      x=int(pos.latitude_deg * 1e7),
                                      y=int(pos.longitude_deg * 1e7),
                                      z=pos.relative_altitude_m)
        result = int(ack["result"]) if ack else -1
        log.info(_FMT, _CMD, "param2=0 after takeoff (not in Hold)", f"result={result}")
        assert result == MAV_RESULT_DENIED, (
            f"Expected DENIED when not in hold-equivalent mode; got {result}"
        )
    finally:
        await _rtl_and_land(gcs_system)
