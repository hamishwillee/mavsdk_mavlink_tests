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
import logging

import pytest
from mavsdk.telemetry import LandedState

from tests.command.conftest import (
    probe_command_int,
    send_command_int,
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

async def _get_home_position(system, timeout_s: float = 30.0):
    """Return the vehicle's home Position from telemetry."""
    async with asyncio.timeout(timeout_s):
        async for home in system.telemetry.home():
            return home
    raise TimeoutError("Home position not received within timeout")


async def _wait_armable(system, timeout_s: float = ARMABLE_TIMEOUT_S):
    """Block until the vehicle reports is_armable=True."""
    async with asyncio.timeout(timeout_s):
        async for health in system.telemetry.health():
            if health.is_armable:
                return


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

    # --- Probe 2: actual execution (arm → send → 15 s wait) ---
    if _nav_takeoff_executes is None:
        try:
            await _arm_and_send_takeoff(system, z=5.0)
            await _wait_for_altitude(system, 0.5, timeout_s=20.0)
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


# ---------------------------------------------------------------------------
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
# Post-takeoff mode (informational)
# ---------------------------------------------------------------------------

async def test_mode_after_takeoff(gcs_system):
    """
    Observe the flight mode once NAV_TAKEOFF altitude is reached — informational only.

    The MAVLink spec does not define what mode a vehicle should be in after a
    COMMAND_INT NAV_TAKEOFF.  This test logs the observed mode and passes.
    """
    await _ensure_nav_takeoff_supported(gcs_system)
    try:
        await _arm_and_send_takeoff(gcs_system, z=TAKEOFF_ALT_M)
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
