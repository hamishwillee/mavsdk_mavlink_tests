"""
Baseline takeoff tests — one per (stack, vehicle-type) combination.

Each test verifies the minimum working sequence to get a vehicle airborne using
MAVLink commands.  These tests serve as **pre-conditions** for all other
execution tests: if the baseline for your platform fails, the higher-level
tests will also fail.

Command selection for VTOL frames
-----------------------------------
Two takeoff commands exist for VTOL capable vehicles:

``MAV_CMD_NAV_TAKEOFF (22)``
    General-purpose takeoff for all vehicle types.  On multicopters and
    QuadPlanes, the vehicle climbs vertically.  On fixed-wing, requires a
    runway roll.

``MAV_CMD_NAV_VTOL_TAKEOFF (84)``
    VTOL-specific takeoff.  On PX4 VTOL, executes a full VTOL sequence:
    MC hover → heading alignment → fixed-wing transition → fixed-wing climb
    to loiter altitude.  On ArduPlane QuadPlane, this is a **mission protocol
    command** (executed in AUTO mode) and cannot be sent as a direct
    COMMAND_INT — use NAV_TAKEOFF (22) in GUIDED mode instead.

Which ArduCopter mode allows NAV_TAKEOFF (22)
-----------------------------------------------
``GCS_MAVLink_Copter.cpp:578`` checks ``has_user_takeoff(must_navigate)``
in the current flight mode.  With the default param3=0 (``must_navigate=True``):

=========   =======  =========================  ===========================
Mode        Number   Accepts cmd  (param3=0)    Autonomous climb (no RC)
=========   =======  =========================  ===========================
STABILIZE   0        ❌                         —
ALT_HOLD    2        ❌ (needs param3=1)         —
GUIDED      4        ✅                         ✅ ``_AutoTakeoff::run()``
LOITER      5        ✅                         ❌ pilot controller
POSHOLD     16       ✅                         ❌ pilot controller
=========   =======  =========================  ===========================

Only GUIDED (4) overrides ``do_user_takeoff_start_m()`` to use the autonomous
``_AutoTakeoff::run()`` controller (spool motors + ramp throttle without RC).
LOITER / POSHOLD accept the command but use ``_TakeOff::do_pilot_takeoff_ms()``
which reads the RC throttle channel — unreliable without RC input.

ArduPlane QuadPlane (GUIDED = mode 15)
-----------------------------------------
QuadPlane's GUIDED mode path calls ``guided_start()`` + sets
``guided_takeoff=true`` in ``quadplane.cpp:4039``, activating the VTOL
position controller for a vertical climb.  The quadplane autotest always
uses ``takeoff(height, mode="GUIDED")`` for autonomous tests — never QHOVER
or QLOITER (those require RC throttle).

MAVSDK quirks for ArduPilot
------------------------------
- ArduCopter GUIDED (custom_mode=4) → MAVSDK reports as ``"OFFBOARD"``.
- ArduPlane GUIDED (custom_mode=15) → MAVSDK may report as ``"GUIDED"`` or
  ``"OFFBOARD"``; ``_set_guided_mode_ardupilot()`` accepts both.
- ArduCopter does NOT stream ``GLOBAL_POSITION_INT`` by default.  Call
  ``_request_position_stream()`` before any altitude polling.
- MAVSDK ``action.arm()`` is unreliable on ArduCopter SITL cold start.
  Use raw ``COMMAND_LONG 400`` retry loop instead.

Running
-------
ArduCopter MC::

    pytest tests/command/baseline_takeoff/test_baseline.py::test_ardupilot_mc_takeoff_baseline \\
        --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \\
        --ardupilot-sitl=~/ardu_sitl/arducopter \\
        --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 \\
        --vehicle-type=quadcopter --autopilot=ardupilot -v --log-cli-level=INFO

PX4 MC::

    pytest tests/command/baseline_takeoff/test_baseline.py::test_px4_mc_takeoff_baseline \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \\
        --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO

PX4 VTOL::

    pytest tests/command/baseline_takeoff/test_baseline.py::test_px4_vtol_takeoff_baseline \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_standard_vtol \\
        --vehicle-type=vtol --autopilot=px4 -v --log-cli-level=INFO

ArduPlane QuadPlane::

    pytest tests/command/baseline_takeoff/test_baseline.py::test_ardupilot_quadplane_takeoff_baseline \\
        --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \\
        --ardupilot-sitl=~/ardu_sitl/arduplane --ardupilot-model=quadplane \\
        --vehicle-type=quadplane --autopilot=ardupilot -v --log-cli-level=INFO
"""

import asyncio
import logging

import pytest

from tests.command.conftest import (
    probe_command_int,
    probe_command_long,
    _FMT,
)
from tests.command.takeoff.test_flight import (
    _get_home_position,
    _wait_armable,
    _rtl_and_land,
    _wait_for_altitude,
    _request_position_stream,
    _set_guided_mode_ardupilot,
    _takeoff_cmd,
)

log = logging.getLogger(__name__)

_NAV_TAKEOFF      = 22   # MAV_CMD_NAV_TAKEOFF
_NAV_VTOL_TAKEOFF = 84   # MAV_CMD_NAV_VTOL_TAKEOFF

_ACK_NAMES = {
    0: "ACCEPTED", 1: "TEMP_REJECTED", 2: "DENIED",
    3: "UNSUPPORTED", 4: "FAILED",
}

pytestmark = pytest.mark.timeout(300)


@pytest.fixture(autouse=True)
def require_real_stack(request):
    """Skip every test when no --drone-address is given (paired/mock mode)."""
    if request.config.getoption("--drone-address") is None:
        pytest.skip("Baseline tests require a real flight stack (--drone-address not set)")


# ---------------------------------------------------------------------------
# ArduCopter MC
# ---------------------------------------------------------------------------

async def test_ardupilot_mc_takeoff_baseline(gcs_system, request):
    """
    ArduCopter MC — minimum working takeoff sequence via COMMAND_LONG.

    Sequence (mirrors ArduPilot autotest ``arducopter.py``)::

        change_mode("GUIDED")   # custom_mode=4; only mode with autonomous takeoff
        arm()                   # MAV_CMD_COMPONENT_ARM_DISARM (400) p1=1
        user_takeoff(alt=20)    # MAV_CMD_NAV_TAKEOFF (22) COMMAND_LONG p7=20

    Why GUIDED (4) is the only mode that works for autonomous takeoff without RC
    is documented in the module docstring above.

    Command used: NAV_TAKEOFF (22) via COMMAND_LONG.
    """
    if (request.config.getoption("--autopilot") != "ardupilot" or
            request.config.getoption("--vehicle-type") not in ("quadcopter", "copter")):
        pytest.skip(
            "ArduCopter multicopter only "
            "(--autopilot=ardupilot --vehicle-type=quadcopter)"
        )

    TARGET_ALT_REL = 20.0
    threshold = TARGET_ALT_REL * 0.85

    home = await _get_home_position(gcs_system, timeout_s=60.0)
    log.info(
        _FMT, "NAV_TAKEOFF", "ArduCopter MC — setup",
        f"home ({home.latitude_deg:.5f}°N, {home.longitude_deg:.5f}°E "
        f"AMSL={home.absolute_altitude_m:.1f} m)  target={TARGET_ALT_REL:.0f} m rel",
    )

    try:
        # GUIDED mode: polls telemetry.flight_mode() until "OFFBOARD" is confirmed
        # (MAVSDK reports ArduCopter GUIDED / custom_mode=4 as "OFFBOARD").
        guided_ok = await _set_guided_mode_ardupilot(gcs_system)
        log.info(_FMT, "NAV_TAKEOFF", "GUIDED mode",
                 "confirmed ('OFFBOARD')" if guided_ok else "TIMEOUT — proceeding")

        # ArduCopter does not stream GLOBAL_POSITION_INT by default.
        await _request_position_stream(gcs_system)

        # Arm: raw COMMAND_LONG retry bypasses MAVSDK's unreliable health-stream
        # gate on ArduCopter SITL cold start.
        armed = False
        async with asyncio.timeout(120.0):
            while True:
                ack = await probe_command_long(gcs_system, 400, param1=1.0)
                if ack and int(ack["result"]) == 0:
                    armed = True
                    log.info(_FMT, "NAV_TAKEOFF", "arm", "ACCEPTED")
                    break
                await asyncio.sleep(3.0)
        assert armed, "Vehicle did not arm within 120 s"

        # Settle: GUIDED VelAccel mode calls make_safe_ground_handling() (spool=GROUND_IDLE).
        # After NAV_TAKEOFF switches guided_mode=TakeOff, _AutoTakeoff::run() requests
        # THROTTLE_UNLIMITED.  3 s allows spool-up before the throttle ramp starts.
        await asyncio.sleep(3.0)

        ack = await probe_command_long(gcs_system, _NAV_TAKEOFF, param7=TARGET_ALT_REL)
        assert ack is not None, "No ACK for NAV_TAKEOFF COMMAND_LONG"
        result = int(ack["result"])
        log.info(_FMT, "NAV_TAKEOFF", "NAV_TAKEOFF COMMAND_LONG",
                 f"ACK={_ACK_NAMES.get(result, result)}  param7={TARGET_ALT_REL:.0f} m")
        assert result in (0, 1), (
            f"NAV_TAKEOFF returned {_ACK_NAMES.get(result, result)} — "
            "ensure vehicle is in GUIDED mode before sending"
        )

        pos = await _wait_for_altitude(gcs_system, threshold, timeout_s=90.0)
        log.info(_FMT, "NAV_TAKEOFF", "ArduCopter MC — PASS",
                 f"reached {pos.relative_altitude_m:.1f} m (threshold {threshold:.1f} m)")
        assert pos.relative_altitude_m >= threshold
    finally:
        await _rtl_and_land(gcs_system)


# ---------------------------------------------------------------------------
# PX4 MC
# ---------------------------------------------------------------------------

@pytest.mark.timeout(180)
async def test_px4_mc_takeoff_baseline(gcs_system, request):
    """
    PX4 MC — minimum working takeoff sequence via COMMAND_INT.

    Sequence::

        action.arm()           # MAVSDK; PX4 transitions to HOLD
        COMMAND_INT NAV_TAKEOFF (22)   # frame=5, z=home_amsl+target_rel

    PX4 ignores the ``frame`` field and always reads ``z`` as absolute AMSL.
    ``x``/``y`` (lat/lon ×1e7) specify the target position; PX4 climbs
    diagonally (no vertical-first phase).  Mode transition on arrival: TAKEOFF→HOLD.

    No mode pre-set required: PX4 does not check the current mode.

    Command used: NAV_TAKEOFF (22) via COMMAND_INT.
    """
    if (request.config.getoption("--autopilot") != "px4" or
            request.config.getoption("--vehicle-type") not in
            ("quadcopter", "multicopter")):
        pytest.skip(
            "PX4 multicopter only (--autopilot=px4 --vehicle-type=quadcopter)"
        )

    TARGET_ALT_REL = 20.0
    threshold = TARGET_ALT_REL * 0.85

    home = await _get_home_position(gcs_system, timeout_s=30.0)
    target_z = home.absolute_altitude_m + TARGET_ALT_REL
    log.info(_FMT, "NAV_TAKEOFF", "PX4 MC — setup",
             f"home AMSL={home.absolute_altitude_m:.1f} m  target_z={target_z:.1f} m abs")

    try:
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await asyncio.sleep(0.5)

        kw = _takeoff_cmd(
            x=int(home.latitude_deg * 1e7),
            y=int(home.longitude_deg * 1e7),
            z=target_z,
            frame=5,      # GLOBAL_INT (absolute AMSL)
            param4=None,  # NaN → current heading
        )
        ack = await probe_command_int(gcs_system, **kw)
        assert ack is not None, "No ACK for NAV_TAKEOFF COMMAND_INT"
        result = int(ack["result"])
        log.info(_FMT, "NAV_TAKEOFF", "NAV_TAKEOFF COMMAND_INT",
                 f"ACK={_ACK_NAMES.get(result, result)}  z={target_z:.1f} m abs")
        assert result in (0, 1)

        pos = await _wait_for_altitude(gcs_system, threshold, timeout_s=60.0)
        log.info(_FMT, "NAV_TAKEOFF", "PX4 MC — PASS",
                 f"reached {pos.relative_altitude_m:.1f} m (threshold {threshold:.1f} m)")
        assert pos.relative_altitude_m >= threshold
    finally:
        await _rtl_and_land(gcs_system)


# ---------------------------------------------------------------------------
# PX4 VTOL
# ---------------------------------------------------------------------------

@pytest.mark.timeout(300)
async def test_px4_vtol_takeoff_baseline(gcs_system, request):
    """
    PX4 VTOL — preferred takeoff via NAV_VTOL_TAKEOFF (84) with NAV_TAKEOFF fallback.

    VTOL preference
    ---------------
    ``MAV_CMD_NAV_VTOL_TAKEOFF (84)`` is the preferred command for VTOL vehicles.
    PX4 VTOL's ``vtol_takeoff.cpp`` executes a full VTOL sequence:

    1. **TAKEOFF_HOVER**: MC hover climb to takeoff altitude.
    2. **ALIGN_HEADING**: align heading toward the loiter position.
    3. **TRANSITION**: transition to fixed-wing flight.
    4. **CLIMB**: continue climbing to ``loiter_altitude_msl + LOITER_ALT_OFFSET``
       in fixed-wing mode.  Then transitions to loiter.

    This test probes NAV_VTOL_TAKEOFF (84) first.  If the vehicle reaches the
    airborne threshold during the MC hover phase (step 1), it is considered a
    pass — we do not wait for the full FW transition which may take 60–120 s.

    Fallback: if NAV_VTOL_TAKEOFF returns UNSUPPORTED (e.g. non-VTOL frame),
    the test falls back to NAV_TAKEOFF (22) with the same parameters.

    Command used: NAV_VTOL_TAKEOFF (84) preferred, NAV_TAKEOFF (22) fallback.

    Source: ``src/modules/navigator/vtol_takeoff.cpp`` (PX4).
    """
    if (request.config.getoption("--autopilot") != "px4" or
            request.config.getoption("--vehicle-type") != "vtol"):
        pytest.skip(
            "PX4 VTOL only (--autopilot=px4 --vehicle-type=vtol)"
        )

    TARGET_ALT_REL = 20.0
    AIRBORNE_THRESHOLD_M = 5.0   # check that MC hover phase at least lifts off
    threshold = max(TARGET_ALT_REL * 0.85, AIRBORNE_THRESHOLD_M)

    home = await _get_home_position(gcs_system, timeout_s=30.0)
    target_z = home.absolute_altitude_m + TARGET_ALT_REL
    log.info(
        _FMT, "VTOL_TAKEOFF", "PX4 VTOL — setup",
        f"home AMSL={home.absolute_altitude_m:.1f} m  target_z={target_z:.1f} m abs  "
        f"airborne_threshold={threshold:.1f} m",
    )

    try:
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await asyncio.sleep(0.5)

        # Common COMMAND_INT parameters for both cmd=84 and cmd=22.
        # PX4 ignores frame and always treats z as absolute AMSL.
        base_kw = dict(
            x=int(home.latitude_deg * 1e7),
            y=int(home.longitude_deg * 1e7),
            z=target_z,
            frame=5,      # GLOBAL_INT (absolute AMSL)
            param4=None,  # NaN → current heading
        )

        # ── Tier 1: NAV_VTOL_TAKEOFF (84) ────────────────────────────────────
        log.info(_FMT, "VTOL_TAKEOFF", "probe NAV_VTOL_TAKEOFF (84)",
                 "preferred for VTOL frames")
        kw84 = _takeoff_cmd(command=_NAV_VTOL_TAKEOFF, **base_kw)
        ack84 = await probe_command_int(gcs_system, **kw84)
        result84 = int(ack84["result"]) if ack84 else -1
        log.info(_FMT, "VTOL_TAKEOFF", "NAV_VTOL_TAKEOFF ACK",
                 _ACK_NAMES.get(result84, f"result={result84}"))

        if result84 in (0, 1):
            # NAV_VTOL_TAKEOFF accepted — wait for MC hover phase to lift off.
            # The full VTOL sequence (FW transition + climb) may take 2-3 min;
            # we pass once the initial MC hover reaches AIRBORNE_THRESHOLD_M.
            log.info(_FMT, "VTOL_TAKEOFF", "waiting for lift-off",
                     f"threshold={threshold:.1f} m (MC hover phase; FW transition may follow)")
            pos = await _wait_for_altitude(gcs_system, threshold, timeout_s=120.0)
            log.info(_FMT, "VTOL_TAKEOFF", "PX4 VTOL — PASS (NAV_VTOL_TAKEOFF)",
                     f"reached {pos.relative_altitude_m:.1f} m")
            assert pos.relative_altitude_m >= threshold
            return

        # ── Tier 2: NAV_TAKEOFF (22) fallback ────────────────────────────────
        log.info(_FMT, "VTOL_TAKEOFF", "NAV_VTOL_TAKEOFF not accepted",
                 "falling back to NAV_TAKEOFF (22)")
        kw22 = _takeoff_cmd(**base_kw)   # default command=22
        ack22 = await probe_command_int(gcs_system, **kw22)
        result22 = int(ack22["result"]) if ack22 else -1
        log.info(_FMT, "VTOL_TAKEOFF", "NAV_TAKEOFF (22) ACK",
                 _ACK_NAMES.get(result22, f"result={result22}"))
        assert result22 in (0, 1), (
            f"Both NAV_VTOL_TAKEOFF and NAV_TAKEOFF failed: "
            f"84→{_ACK_NAMES.get(result84, result84)}  "
            f"22→{_ACK_NAMES.get(result22, result22)}"
        )
        pos = await _wait_for_altitude(gcs_system, threshold, timeout_s=60.0)
        log.info(_FMT, "VTOL_TAKEOFF", "PX4 VTOL — PASS (NAV_TAKEOFF fallback)",
                 f"reached {pos.relative_altitude_m:.1f} m")
        assert pos.relative_altitude_m >= threshold
    finally:
        await _rtl_and_land(gcs_system)


# ---------------------------------------------------------------------------
# ArduPlane QuadPlane
# ---------------------------------------------------------------------------

@pytest.mark.timeout(300)
async def test_ardupilot_quadplane_takeoff_baseline(gcs_system, request):
    """
    ArduPlane QuadPlane — VTOL takeoff via GUIDED (15) + NAV_TAKEOFF (22).

    Why not NAV_VTOL_TAKEOFF (84)?
    --------------------------------
    ``MAV_CMD_NAV_VTOL_TAKEOFF (84)`` on ArduPlane is a **mission protocol
    command** — it is handled in ``commands_logic.cpp`` during AUTO mode
    mission execution only.  It cannot be sent as a direct COMMAND_INT while on
    the ground.  Sending it outside AUTO mode returns FAILED.

    Why GUIDED (mode 15)?
    ----------------------
    ArduPlane QuadPlane's GUIDED mode activates the VTOL position controller:
    ``quadplane.cpp:4039`` sets ``guided_takeoff=true`` when NAV_TAKEOFF is
    received in GUIDED mode, causing ``in_vtol_mode()`` to return true and the
    VTOL attitude controller to run.  The vehicle climbs vertically using the
    quadplane motors, identical to QHOVER / QLOITER takeoff except without RC
    throttle input.

    The quadplane autotest (``quadplane.py``) uses::

        takeoff(height, mode="GUIDED")  →  change_mode("GUIDED") + user_takeoff()

    for autonomous tests.  QHOVER (18) and QLOITER (19) require RC throttle
    input and are not suitable for autonomous MAVLink takeoff.

    Note: ArduPlane GUIDED (custom_mode=15) may report as ``"OFFBOARD"`` or
    ``"GUIDED"`` in MAVSDK telemetry — ``_set_guided_mode_ardupilot()`` accepts
    both strings.

    Command used: NAV_TAKEOFF (22) via COMMAND_LONG in GUIDED mode.

    Source: ``quadplane.cpp``, ``mode.h``, ``quadplane.py`` autotest.
    """
    if (request.config.getoption("--autopilot") != "ardupilot" or
            request.config.getoption("--vehicle-type") != "quadplane"):
        pytest.skip(
            "ArduPlane QuadPlane only "
            "(--autopilot=ardupilot --vehicle-type=quadplane)"
        )

    TARGET_ALT_REL = 20.0
    threshold = TARGET_ALT_REL * 0.85

    home = await _get_home_position(gcs_system, timeout_s=60.0)
    log.info(
        _FMT, "NAV_TAKEOFF", "ArduPlane QuadPlane — setup",
        f"home ({home.latitude_deg:.5f}°N, {home.longitude_deg:.5f}°E "
        f"AMSL={home.absolute_altitude_m:.1f} m)  target={TARGET_ALT_REL:.0f} m rel",
    )

    try:
        # GUIDED mode for ArduPlane QuadPlane = custom_mode=15.
        # _set_guided_mode_ardupilot polls until "OFFBOARD" or "GUIDED" is confirmed.
        # ArduPlane GUIDED (15) may show as "GUIDED" or "OFFBOARD" in MAVSDK.
        guided_ok = await _set_guided_mode_ardupilot(gcs_system, custom_mode=15)
        log.info(_FMT, "NAV_TAKEOFF", "GUIDED mode (custom=15)",
                 "confirmed" if guided_ok else "TIMEOUT — proceeding anyway")

        # QuadPlane also needs GLOBAL_POSITION_INT streaming.
        await _request_position_stream(gcs_system)

        # Arm: raw COMMAND_LONG to bypass MAVSDK health-stream issues on cold start.
        armed = False
        async with asyncio.timeout(120.0):
            while True:
                ack = await probe_command_long(gcs_system, 400, param1=1.0)
                if ack and int(ack["result"]) == 0:
                    armed = True
                    log.info(_FMT, "NAV_TAKEOFF", "arm", "ACCEPTED")
                    break
                await asyncio.sleep(3.0)
        assert armed, "Vehicle did not arm within 120 s"

        await asyncio.sleep(3.0)  # motor spool settle

        # NAV_TAKEOFF COMMAND_LONG — QuadPlane GUIDED mode routes to VTOL controller.
        # param7 = altitude relative to home.  lat/lon in COMMAND_LONG are ignored
        # (ArduPlane do_vtol_takeoff always uses current XY location).
        ack = await probe_command_long(gcs_system, _NAV_TAKEOFF, param7=TARGET_ALT_REL)
        assert ack is not None, "No ACK for NAV_TAKEOFF COMMAND_LONG"
        result = int(ack["result"])
        log.info(_FMT, "NAV_TAKEOFF", "NAV_TAKEOFF COMMAND_LONG",
                 f"ACK={_ACK_NAMES.get(result, result)}  param7={TARGET_ALT_REL:.0f} m")
        assert result in (0, 1), (
            f"NAV_TAKEOFF returned {_ACK_NAMES.get(result, result)} — "
            "ensure GUIDED mode (15) is set before sending on QuadPlane"
        )

        pos = await _wait_for_altitude(gcs_system, threshold, timeout_s=90.0)
        log.info(_FMT, "NAV_TAKEOFF", "ArduPlane QuadPlane — PASS",
                 f"reached {pos.relative_altitude_m:.1f} m (threshold {threshold:.1f} m)")
        assert pos.relative_altitude_m >= threshold
    finally:
        await _rtl_and_land(gcs_system)
