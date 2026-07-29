"""
MAV_CMD_DO_SET_MISSION_CURRENT (cmd=224) param2 — Tier 2 jump-counter reset test.

Implements the design outlined in README.md § "Design: verifying param2 resets
DO_JUMP repeat counters".  Jump counters are internal mission-executor state,
not exposed via any MAVLink telemetry field, so this can only be verified
*behaviourally*: fly a mission containing a MAV_CMD_DO_JUMP loop, tally how
many times the jump target is revisited, and show that sending
DO_SET_MISSION_CURRENT(param1=-1, param2=1) mid-loop causes MORE revisits than
a control run without the reset.

Skipped:
  - in paired/mock mode (no --drone-address) — MockFlightStack stores/ACKs
    mission items but has no mission executor at all (confirmed: no
    "current item" tracking, no MISSION_CURRENT emission, no DO_JUMP
    handling), so nothing would ever advance past the first item.
  - when DO_SET_MISSION_CURRENT (cmd=224) is UNSUPPORTED on the connected
    stack (probed once per test via COMMAND_LONG).

Mission shape (built at runtime from the vehicle's actual home position, NOT
a static plan file — a fixed absolute-coordinate plan would be impractical to
fly to from an arbitrarily-configured SITL home; reuses
tests/mission/nav_takeoff/test_flight.py's _get_home_position()/_north_of()
pattern via cross-module import, the same precedent CLAUDE.md documents for
nav_land/test_flight.py)::

    seq  command         purpose
    0    NAV_TAKEOFF     climb to JUMP_ALT_M
    1    NAV_WAYPOINT A  loop target (20 m north of home)
    2    NAV_WAYPOINT B  loop far point (40 m north of home)
    3    DO_JUMP(target=seq(A), repeat=JUMP_REPEAT)
    4    NAV_WAYPOINT C  only reached once the loop is exhausted (60 m north)
    5    NAV_RETURN_TO_LAUNCH

With JUMP_REPEAT=2, an untouched run visits A exactly 3 times (repeat+1).  The
reset run sends the reset after the 2nd A-visit (i.e. after the counter has
already decremented once) — restoring it from 1 back to 2 — so a working
reset must produce strictly more A-visits than the control run.  This is a
*relative*, not absolute, assertion: exact visit counts are execution-order
and timing dependent (see README.md for the full rationale).

Running
-------
Against PX4 SIH::

    pytest tests/command/do_set_mission_current/test_flight.py \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \\
        --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO

Against ArduCopter SITL::

    pytest tests/command/do_set_mission_current/test_flight.py \\
        --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \\
        --ardupilot-sitl=~/ardu_sitl/arducopter \\
        --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 \\
        --vehicle-type=copter --autopilot=ardupilot -v --log-cli-level=INFO
"""

import asyncio
import logging

import pytest
from mavsdk.mission_raw import MissionItem

from tests.command.conftest import probe_command_long
from tests.mission.conftest import (  # noqa: F401
    clear_all_mission_types,
    home_item_for_mission,
    requires_home_slot,  # dependency of home_item_for_mission
)
from tests.mission.nav_takeoff.test_flight import (
    _get_home_position,
    _north_of,
    _rtl_and_land,
    _wait_armable,
    require_real_stack,  # noqa: F401 — registers the real-stack skip gate in this module
)
from tests.mock_flight_stack import MAV_RESULT_UNSUPPORTED

log = logging.getLogger(__name__)

# Arming (60s) + up to ~7 loop passes (fast, small offsets) + RTL/land (120s) x2 runs + margin.
pytestmark = pytest.mark.timeout(900)

_CMD_ID = 224  # MAV_CMD_DO_SET_MISSION_CURRENT

TRANSFER_TIMEOUT_S = 30.0
JUMP_ALT_M = 15.0
JUMP_REPEAT = 2         # 3 total A-visits per untouched run (repeat + 1)
LOOP_TIMEOUT_S = 240.0  # budget for one full mission run (takeoff..RTL)
NAN = float("nan")


# ---------------------------------------------------------------------------
# Mission builder
# ---------------------------------------------------------------------------

def _jump_mission_items(home_item, home):
    """
    Build the seq0-5 jump-loop mission described in the module docstring.

    Returns (items, wp_a_seq, rtl_seq).  If home_item is not None (ArduCopter,
    which reserves seq=0 for home), everything is shifted by one seq and the
    home item is prepended with current=0 — mirrors
    tests/mission/nav_takeoff/test_flight.py's _build_mission().
    """
    items = []
    if home_item is not None:
        items.append(MissionItem(
            seq=0, frame=home_item.frame, command=home_item.command,
            current=0, autocontinue=1,
            param1=home_item.param1, param2=home_item.param2,
            param3=home_item.param3, param4=home_item.param4,
            x=home_item.x, y=home_item.y, z=home_item.z,
            mission_type=0,
        ))
    offset = len(items)

    takeoff = MissionItem(
        seq=offset + 0, frame=6, command=22, current=(1 if offset == 0 else 0), autocontinue=1,
        param1=0.0, param2=0.0, param3=0.0, param4=NAN,
        x=int(home.latitude_deg * 1e7), y=int(home.longitude_deg * 1e7), z=JUMP_ALT_M,
        mission_type=0,
    )
    wp_a = MissionItem(
        seq=offset + 1, frame=6, command=16, current=0, autocontinue=1,
        param1=0.0, param2=5.0, param3=0.0, param4=NAN,   # param2 = acceptance radius (m)
        x=_north_of(home.latitude_deg, 20), y=int(home.longitude_deg * 1e7), z=JUMP_ALT_M,
        mission_type=0,
    )
    wp_b = MissionItem(
        seq=offset + 2, frame=6, command=16, current=0, autocontinue=1,
        param1=0.0, param2=5.0, param3=0.0, param4=NAN,
        x=_north_of(home.latitude_deg, 40), y=int(home.longitude_deg * 1e7), z=JUMP_ALT_M,
        mission_type=0,
    )
    do_jump = MissionItem(
        seq=offset + 3, frame=2, command=177, current=0, autocontinue=1,
        param1=float(offset + 1), param2=float(JUMP_REPEAT), param3=0.0, param4=0.0,
        x=0, y=0, z=0.0,
        mission_type=0,
    )
    wp_c = MissionItem(
        seq=offset + 4, frame=6, command=16, current=0, autocontinue=1,
        param1=0.0, param2=5.0, param3=0.0, param4=NAN,
        x=_north_of(home.latitude_deg, 60), y=int(home.longitude_deg * 1e7), z=JUMP_ALT_M,
        mission_type=0,
    )
    rtl = MissionItem(
        seq=offset + 5, frame=2, command=20, current=0, autocontinue=1,
        param1=0.0, param2=0.0, param3=0.0, param4=0.0,
        x=0, y=0, z=0.0,
        mission_type=0,
    )
    items.extend([takeoff, wp_a, wp_b, do_jump, wp_c, rtl])
    return items, offset + 1, offset + 2, offset + 5


# ---------------------------------------------------------------------------
# Run helper
# ---------------------------------------------------------------------------

async def _fly_jump_mission(system, home_item, home, *, send_reset_after_visit: int | None) -> tuple[int, int | None]:
    """
    Upload, arm, and fly the jump-loop mission; return
    (visits, reset_ack_result).

    visits is the number of times wp_a (the jump target) was genuinely
    visited. reset_ack_result is the COMMAND_ACK result of the
    DO_SET_MISSION_CURRENT reset (None if send_reset_after_visit was None, or
    if no ACK was received).

    If send_reset_after_visit is not None, sends
    DO_SET_MISSION_CURRENT(param1=-1, param2=1) via COMMAND_LONG immediately
    after the Nth genuine visit to wp_a — the spec-correct form, confirmed
    ACCEPTED mid-flight on PX4 MC 1.18.0-beta. If a stack DENIES the -1
    sentinel (a spec violation — it's explicitly defined for exactly this
    "keep current item" use case), this falls back to an explicit current-seq
    retry with param2=1, so a stack's -1-sentinel bug doesn't mask the signal
    this test actually cares about: whether param2 itself resets the jump
    counter. reset_ack_result reflects whichever attempt succeeded (or the
    last attempt's result if neither did).

    Visit counting requires wp_b to have been observed as current since the
    last counted wp_a visit before counting the next one. This is NOT
    redundant with the raw MISSION_CURRENT.seq-change dedup below: live
    tracing against PX4 (1.18.0-beta) showed MISSION_CURRENT oscillating
    rapidly between wp_a's seq and the DO_JUMP item's seq several times per
    real jump (e.g. seq 1,3,1,3,1,3... at ~1 Hz, with no wp_b seq=2 in
    between and no real vehicle movement) — a MISSION_CURRENT reporting
    artifact around DO_JUMP, not genuine re-execution. Without the
    "wp_b seen since last count" gate, a JUMP_REPEAT=2 mission (expected: 3
    total wp_a visits) tallied 13 — confirmed via manual mission_progress()
    tracing (see tests/command/do_set_mission_current/README.md). Requiring
    an intervening wp_b filters this out and reproduces the expected count.
    """
    items, wp_a_seq, wp_b_seq, rtl_seq = _jump_mission_items(home_item, home)

    async with asyncio.timeout(TRANSFER_TIMEOUT_S):
        await system.mission_raw.upload_mission(items)
    log.info("Jump mission uploaded (%d items, wp_a seq=%d); waiting for armable", len(items), wp_a_seq)

    await _wait_armable(system)
    await system.action.arm()
    await system.mission_raw.start_mission()
    log.info("Armed and mission started")

    visits = 0
    reset_sent = False
    reset_ack_result: int | None = None
    last_seq = None
    seen_b_since_last_a = True  # the first wp_a visit (post-takeoff) always counts

    async with asyncio.timeout(LOOP_TIMEOUT_S):
        async for progress in system.mission_raw.mission_progress():
            current = progress.current
            if current == last_seq:
                continue
            last_seq = current
            if current == wp_b_seq:
                seen_b_since_last_a = True
            elif current == wp_a_seq and seen_b_since_last_a:
                visits += 1
                seen_b_since_last_a = False
                log.info("wp_a (seq=%d) genuine visit #%d", wp_a_seq, visits)
                if send_reset_after_visit is not None and visits == send_reset_after_visit and not reset_sent:
                    reset_sent = True
                    ack = await probe_command_long(system, _CMD_ID, param1=-1.0, param2=1.0)
                    reset_ack_result = int(ack["result"]) if ack is not None else None
                    log.info("Sent DO_SET_MISSION_CURRENT(param1=-1, param2=1) reset — ACK result=%s",
                              reset_ack_result)
                    if reset_ack_result != 0:
                        log.warning(
                            "DOC DISCREPANCY: DO_SET_MISSION_CURRENT reset (param1=-1, param2=1) "
                            "was not ACCEPTED mid-mission (result=%s) — the param1=-1 sentinel is "
                            "spec-defined but this stack rejects it", reset_ack_result,
                        )
                        ack2 = await probe_command_long(system, _CMD_ID, param1=float(wp_a_seq), param2=1.0)
                        reset_ack_result = int(ack2["result"]) if ack2 is not None else reset_ack_result
                        log.info(
                            "Retried with explicit param1=%d (current seq) instead of -1 — ACK result=%s",
                            wp_a_seq, reset_ack_result,
                        )
            if current >= rtl_seq or current == -1:
                break

    return visits, reset_ack_result


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

async def test_param2_resets_jump_counter(gcs_system, home_item_for_mission):
    """
    param2=1 should reset a DO_JUMP repeat counter — verified behaviourally.

    Flies the jump-loop mission twice (control, then with a mid-loop reset)
    and asserts the reset run visits the jump target strictly more times.
    """
    ack = await probe_command_long(gcs_system, _CMD_ID, param1=-1.0, param2=0.0)
    if ack is not None and int(ack["result"]) == MAV_RESULT_UNSUPPORTED:
        pytest.skip(f"DO_SET_MISSION_CURRENT (cmd={_CMD_ID}) is UNSUPPORTED on this platform")

    home = await _get_home_position(gcs_system)

    control_visits = None
    test_visits = None
    reset_ack_result = None
    try:
        log.info("=== Control run: no reset ===")
        control_visits, _ = await _fly_jump_mission(gcs_system, home_item_for_mission, home, send_reset_after_visit=None)
    finally:
        await _rtl_and_land(gcs_system)
        await clear_all_mission_types(gcs_system)

    try:
        log.info("=== Test run: reset after 2nd genuine wp_a visit ===")
        test_visits, reset_ack_result = await _fly_jump_mission(
            gcs_system, home_item_for_mission, home, send_reset_after_visit=2
        )
    finally:
        await _rtl_and_land(gcs_system)
        await clear_all_mission_types(gcs_system)

    log.info("Jump-counter reset result: control_visits=%d test_visits=%d reset_ack=%s (JUMP_REPEAT=%d)",
              control_visits, test_visits, reset_ack_result, JUMP_REPEAT)

    if reset_ack_result != 0:
        pytest.xfail(
            f"DO_SET_MISSION_CURRENT(param1=-1, param2=1) reset was not ACCEPTED mid-mission "
            f"(result={reset_ack_result}) — a rejected reset command cannot be evidence either "
            f"way for jump-counter behaviour (control={control_visits}, test={test_visits})"
        )

    assert test_visits > control_visits, (
        f"param2=1 should reset the DO_JUMP repeat counter, causing MORE wp_a visits "
        f"than an untouched run; got control={control_visits}, test={test_visits}"
    )
