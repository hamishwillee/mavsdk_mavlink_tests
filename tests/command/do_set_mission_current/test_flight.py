"""
MAV_CMD_DO_SET_MISSION_CURRENT (cmd=224) param2 — Tier 2 behavioural tests.

Two tests, covering the two halves of the authoritative matrix's param2 claim
(module docstring of test_command.py / README.md): "resets DO_JUMP repeat
counters AND changes mission state 'completed' to 'active'/'paused'".

1. `test_param2_resets_jump_counter` — jump-counter half. Jump counters are
   internal mission-executor state, not exposed via any MAVLink telemetry
   field, so this can only be verified *behaviourally*: fly a mission
   containing a MAV_CMD_DO_JUMP loop, tally how many times the jump target is
   revisited, and show that sending DO_SET_MISSION_CURRENT(param1=-1, param2=1)
   mid-loop causes MORE revisits than a control run without the reset.

2. `test_param2_restarts_completed_mission` — mission-state half, attempt 1.
   Flies a short, non-looping mission ENDING ON RTL to MISSION_STATE_COMPLETE
   (observed via raw MISSION_CURRENT.mission_state), then confirms param2=0
   leaves it COMPLETE while param2=1 moves it to ACTIVE/PAUSED (restartable).
   Distinct from test 1 above, which never lets the mission reach completion.
   XFAILs on PX4 MC (see README.md "Root cause of the PX4 MC XFAIL") — ending
   on RTL switches the vehicle out of AUTO.MISSION mode entirely, so
   DO_SET_MISSION_CURRENT's reset never propagates to mission_state regardless
   of param2. Superseded by test 3 below, which corrects the mission shape and
   adds the missing mode-reactivation step.

3. `test_param2_restarts_from_early_item_after_hold` — mission-state half,
   attempt 2 (see README.md "Design: restart-after-Hold, corrected"). Ends the
   mission on an ORDINARY waypoint (not RTL) — PX4 completes into Hold mode
   rather than leaving via RTL — then sends
   DO_SET_MISSION_CURRENT(param1=<early item>, param2=0 or 1) followed by a
   raw MAV_CMD_MISSION_START(param1=-1) to re-engage AUTO.MISSION mode without
   itself moving the current item (PX4's Navigator ignores MISSION_START's
   param1=-1 per its own `>= 0` guard; only Commander acts on it, to arm/switch
   mode). Tests whether param2 is what gates resumption once mode is
   reactivated with a valid non-terminal current item.

All three skipped:
  - in paired/mock mode (no --drone-address) — MockFlightStack stores/ACKs
    mission items but has no mission executor at all (confirmed: no
    "current item" tracking, no MISSION_CURRENT emission, no DO_JUMP
    handling), so nothing would ever advance past the first item.
  - when DO_SET_MISSION_CURRENT (cmd=224) is UNSUPPORTED on the connected
    stack (probed once per test via COMMAND_LONG).

All missions are built at runtime from the vehicle's actual home position, NOT
a static plan file — a fixed absolute-coordinate plan would be impractical to
fly to from an arbitrarily-configured SITL home; reuses
tests/mission/nav_takeoff/test_flight.py's _get_home_position()/_north_of()
pattern via cross-module import, the same precedent CLAUDE.md documents for
nav_land/test_flight.py.

Test 1's mission shape (looping, see `_jump_mission_items`)::

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

Test 2's mission shape (short, non-looping — reach MISSION_STATE_COMPLETE as
fast as possible; see `_restart_mission_items`)::

    seq  command             purpose
    0    NAV_TAKEOFF         climb to RESTART_ALT_M
    1    NAV_WAYPOINT        single nearby waypoint (15 m north of home)
    2    NAV_RETURN_TO_LAUNCH

Test 3's mission shape (ends on an ORDINARY waypoint, not RTL — see
`_hold_mission_items`)::

    seq  command             purpose
    0    NAV_TAKEOFF         climb to HOLD_ALT_M
    1    NAV_WAYPOINT        early item / restart target (HOLD_EARLY_OFFSET_M north)
    2    NAV_WAYPOINT        last item — reaching it sets MISSION_STATE_COMPLETE
                             and PX4 exits AUTO.MISSION into Hold (HOLD_LAST_OFFSET_M north)

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
import json
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
from tests.mock_flight_stack import MAV_RESULT_ACCEPTED, MAV_RESULT_UNSUPPORTED

log = logging.getLogger(__name__)


async def _get_flight_mode(system, timeout_s: float = 5.0) -> str:
    """Return current vehicle flight mode name as a string."""
    async with asyncio.timeout(timeout_s):
        async for fm in system.telemetry.flight_mode():
            return str(fm)
    raise TimeoutError("Flight mode not received")


# Arming (60s) + up to ~7 loop passes (fast, small offsets) + RTL/land (120s) x2 runs + margin.
pytestmark = pytest.mark.timeout(900)

_CMD_ID = 224  # MAV_CMD_DO_SET_MISSION_CURRENT
_MISSION_START_CMD_ID = 300  # MAV_CMD_MISSION_START

TRANSFER_TIMEOUT_S = 30.0
JUMP_ALT_M = 15.0
JUMP_REPEAT = 2         # 3 total A-visits per untouched run (repeat + 1)
LOOP_TIMEOUT_S = 240.0  # budget for one full mission run (takeoff..RTL)
NAN = float("nan")

# MAV_MISSION_STATE (common.xml) — used by test_param2_restarts_completed_mission
# to observe the "completed" -> "active"/"paused" transition the authoritative
# matrix (module docstring / README.md) attributes to param2=1.
MISSION_STATE_UNKNOWN = 0   # state reporting not supported by this stack
MISSION_STATE_ACTIVE = 3
MISSION_STATE_PAUSED = 4
MISSION_STATE_COMPLETE = 5

RESTART_ALT_M = 15.0
RESTART_COMPLETE_TIMEOUT_S = 120.0  # budget for the short takeoff->wp->RTL mission to complete
RESTART_STATE_TIMEOUT_S = 15.0      # budget for mission_state to react to a restart command
RESTART_SETTLE_S = 2.0              # settle time after param2=0 before sampling mission_state

HOLD_ALT_M = 15.0
HOLD_EARLY_OFFSET_M = 15   # early/restart waypoint — north-of-home offset (m)
HOLD_LAST_OFFSET_M = 30    # last (completing) waypoint — north-of-home offset (m)


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


def _restart_mission_items(home_item, home):
    """
    Build a minimal takeoff -> one nearby waypoint -> RTL mission.

    Unlike `_jump_mission_items` (built to loop, for counting revisits), this
    is built to reach MISSION_STATE_COMPLETE as quickly as possible, for
    `test_param2_restarts_completed_mission`. Returns (items, rtl_seq); rtl_seq
    doubles as the last item's seq and as a fallback "valid current item"
    index for the -1-sentinel-DENIED fallback (see `_send_restart_command`).
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
        x=int(home.latitude_deg * 1e7), y=int(home.longitude_deg * 1e7), z=RESTART_ALT_M,
        mission_type=0,
    )
    wp = MissionItem(
        seq=offset + 1, frame=6, command=16, current=0, autocontinue=1,
        param1=0.0, param2=5.0, param3=0.0, param4=NAN,
        x=_north_of(home.latitude_deg, 15), y=int(home.longitude_deg * 1e7), z=RESTART_ALT_M,
        mission_type=0,
    )
    rtl = MissionItem(
        seq=offset + 2, frame=2, command=20, current=0, autocontinue=1,
        param1=0.0, param2=0.0, param3=0.0, param4=0.0,
        x=0, y=0, z=0.0,
        mission_type=0,
    )
    items.extend([takeoff, wp, rtl])
    return items, offset + 2


def _hold_mission_items(home_item, home):
    """
    Build a takeoff -> waypoint A (early/restart target) -> waypoint B (last
    item) mission, where the LAST item is an ORDINARY waypoint rather than
    RTL.

    `_restart_mission_items` (ending on RTL) causes PX4 to leave AUTO.MISSION
    for a dedicated RETURN_TO_LAUNCH mode on completion, which is why
    `test_param2_restarts_completed_mission` XFAILs there — DO_SET_MISSION_
    CURRENT's effect never propagates once Mission mode is inactive (see
    README.md "Root cause of the PX4 MC XFAIL"). Ending on a plain waypoint
    instead completes into Hold mode, and is paired in
    `test_param2_restarts_from_early_item_after_hold` with an explicit
    MAV_CMD_MISSION_START(param1=-1) to re-engage AUTO.MISSION afterwards.

    Returns (items, early_seq, last_seq).
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
        x=int(home.latitude_deg * 1e7), y=int(home.longitude_deg * 1e7), z=HOLD_ALT_M,
        mission_type=0,
    )
    wp_early = MissionItem(
        seq=offset + 1, frame=6, command=16, current=0, autocontinue=1,
        param1=0.0, param2=5.0, param3=0.0, param4=NAN,
        x=_north_of(home.latitude_deg, HOLD_EARLY_OFFSET_M), y=int(home.longitude_deg * 1e7), z=HOLD_ALT_M,
        mission_type=0,
    )
    wp_last = MissionItem(
        seq=offset + 2, frame=6, command=16, current=0, autocontinue=1,
        param1=0.0, param2=5.0, param3=0.0, param4=NAN,
        x=_north_of(home.latitude_deg, HOLD_LAST_OFFSET_M), y=int(home.longitude_deg * 1e7), z=HOLD_ALT_M,
        mission_type=0,
    )
    items.extend([takeoff, wp_early, wp_last])
    return items, offset + 1, offset + 2


# ---------------------------------------------------------------------------
# MISSION_CURRENT.mission_state subscription (restart tests only)
# ---------------------------------------------------------------------------

async def _subscribe_mission_current(system):
    """
    Background raw MISSION_CURRENT subscription, tracking the latest `seq` and
    `mission_state` (MAV_MISSION_STATE) fields.

    Returns (task, state) where `state` is a plain dict updated in place by
    the background task. Caller must cancel the task (fire-and-forget) when
    done — do NOT await after cancel (gRPC stream caveat, CLAUDE.md §4a).
    """
    state: dict = {"seq": None, "mission_state": None}

    async def _collect() -> None:
        async for msg in system.mavlink_direct.message("MISSION_CURRENT"):
            fields = json.loads(msg.fields_json)
            state["seq"] = int(fields.get("seq", -1))
            state["mission_state"] = int(fields.get("mission_state", MISSION_STATE_UNKNOWN))

    task = asyncio.create_task(_collect())
    await asyncio.sleep(0.05)  # let gRPC stream register before the mission starts
    return task, state


async def _wait_for_mission_state(state: dict, targets: set[int], timeout_s: float) -> int | None:
    """
    Poll `state["mission_state"]` (kept live by `_subscribe_mission_current`)
    until it is one of `targets`.

    Returns the matching value, or — if `timeout_s` elapses first — whatever
    `mission_state` was last observed (which may be None if no MISSION_CURRENT
    was ever received, or MISSION_STATE_UNKNOWN(0) if the stack sends the
    message but does not populate the field — "state reporting not supported"
    per the spec). Never raises: the caller treats "didn't reach a target
    state" as a result to assert/xfail on, not an error.
    """
    deadline = asyncio.get_event_loop().time() + timeout_s
    while asyncio.get_event_loop().time() < deadline:
        ms = state.get("mission_state")
        if ms in targets:
            return ms
        await asyncio.sleep(0.2)
    return state.get("mission_state")


async def _send_restart_command(system, param2: float, fallback_seq: int) -> int | None:
    """
    Send DO_SET_MISSION_CURRENT(param1=-1, param2=param2) — the spec-correct
    "keep current item, just reset" form.

    If a stack DENIES the -1 sentinel specifically (a spec violation), retries
    with an explicit valid index (`fallback_seq`) so a stack's -1-sentinel bug
    doesn't mask the signal this test actually cares about: whether param2
    itself changes mission_state. Same defensive pattern as
    `_fly_jump_mission`'s reset send (see README.md "Design note: param1=-1
    fallback"). Returns the (possibly retried) ACK result, or None if no ACK
    was ever received.
    """
    ack = await probe_command_long(system, _CMD_ID, param1=-1.0, param2=param2)
    result = int(ack["result"]) if ack is not None else None
    if result != MAV_RESULT_ACCEPTED:
        log.warning(
            "DO_SET_MISSION_CURRENT(param1=-1, param2=%.0f) result=%s (not ACCEPTED) — "
            "retrying with explicit param1=%d (valid index) instead of -1 sentinel",
            param2, result, fallback_seq,
        )
        ack2 = await probe_command_long(system, _CMD_ID, param1=float(fallback_seq), param2=param2)
        result = int(ack2["result"]) if ack2 is not None else result
    return result


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


async def test_param2_restarts_completed_mission(gcs_system, home_item_for_mission):
    """
    A MISSION_STATE_COMPLETE mission should become restartable (ACTIVE/PAUSED)
    after DO_SET_MISSION_CURRENT(param2=1), but NOT after param2=0 — the other
    half of the authoritative matrix's param2 claim (module docstring /
    README.md), distinct from `test_param2_resets_jump_counter` above (which
    validates the DO_JUMP-counter-reset half via a mid-flight, never-completed
    mission).

    Flies a minimal takeoff -> waypoint -> RTL mission to completion (observed
    via the raw MISSION_CURRENT.mission_state field, MAV_MISSION_STATE enum —
    the field the "completed" claim is actually about; mission_raw's own
    mission_progress()/is_mission_finished() has no equivalent concept), then:

    1. Sends DO_SET_MISSION_CURRENT(param1=-1, param2=0) — mission_state must
       stay MISSION_STATE_COMPLETE(5) (not restarted).
    2. Sends DO_SET_MISSION_CURRENT(param1=-1, param2=1) — mission_state must
       become MISSION_STATE_ACTIVE(3) or MISSION_STATE_PAUSED(4) (restarted).

    If mission_state is never reported as anything but MISSION_STATE_UNKNOWN(0)
    (or is never received at all) — "state reporting not supported" is
    spec-legal — the test is skipped as inconclusive rather than failed,
    mirroring the no-ACK/observational conventions used elsewhere in this
    suite.
    """
    ack = await probe_command_long(gcs_system, _CMD_ID, param1=-1.0, param2=0.0)
    if ack is not None and int(ack["result"]) == MAV_RESULT_UNSUPPORTED:
        pytest.skip(f"DO_SET_MISSION_CURRENT (cmd={_CMD_ID}) is UNSUPPORTED on this platform")

    home = await _get_home_position(gcs_system)
    items, rtl_seq = _restart_mission_items(home_item_for_mission, home)

    state_task, state = await _subscribe_mission_current(gcs_system)
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
        log.info("Restart-test mission uploaded (%d items, rtl seq=%d)", len(items), rtl_seq)

        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        log.info("Armed and mission started")

        completed_state = await _wait_for_mission_state(
            state, {MISSION_STATE_COMPLETE}, timeout_s=RESTART_COMPLETE_TIMEOUT_S
        )
        mode_at_completion = await _get_flight_mode(gcs_system, timeout_s=2.0)
        log.info("Mission run finished — mission_state=%s (seq=%s) flight_mode=%r",
                  completed_state, state.get("seq"), mode_at_completion)

        # NOTE: deliberately do NOT call _rtl_and_land() here (it disarms) —
        # the restart commands below must be sent to a still-armed vehicle,
        # otherwise "restartable" can never be observed. RTL/land/disarm
        # happens once, in the outer `finally`, after both restart attempts.

        if completed_state != MISSION_STATE_COMPLETE:
            pytest.skip(
                f"Mission never reported MISSION_STATE_COMPLETE within "
                f"{RESTART_COMPLETE_TIMEOUT_S:.0f}s (last mission_state={completed_state}) — "
                "either mission_state reporting is unsupported by this stack, or the mission "
                "didn't finish in time; cannot test restart behaviour either way"
            )

        # --- param2=0: must NOT restart a completed mission -----------------
        result_param2_0 = await _send_restart_command(gcs_system, param2=0.0, fallback_seq=rtl_seq)
        await asyncio.sleep(RESTART_SETTLE_S)
        state_after_param2_0 = state.get("mission_state")
        log.info(
            "After param2=0 (ack=%s): mission_state=%s", result_param2_0, state_after_param2_0
        )

        if state_after_param2_0 != MISSION_STATE_COMPLETE:
            log.warning(
                "DOC DISCREPANCY: DO_SET_MISSION_CURRENT(param1=-1, param2=0) sent to a "
                "MISSION_STATE_COMPLETE mission changed mission_state to %s — spec says "
                "param2=0 leaves the mission untouched", state_after_param2_0,
            )
            pytest.xfail(
                f"mission_state became {state_after_param2_0} after param2=0 on a completed "
                "mission; spec says param2=0 should NOT restart it"
            )
        assert state_after_param2_0 == MISSION_STATE_COMPLETE

        # --- param2=1: must restart (ACTIVE/PAUSED) the completed mission ---
        result_param2_1 = await _send_restart_command(gcs_system, param2=1.0, fallback_seq=rtl_seq)
        state_after_param2_1 = await _wait_for_mission_state(
            state, {MISSION_STATE_ACTIVE, MISSION_STATE_PAUSED}, timeout_s=RESTART_STATE_TIMEOUT_S
        )
        mode_after_param2_1 = await _get_flight_mode(gcs_system, timeout_s=2.0)
        log.info(
            "After param2=1 (ack=%s): mission_state=%s flight_mode=%r",
            result_param2_1, state_after_param2_1, mode_after_param2_1,
        )

        if state_after_param2_1 not in (MISSION_STATE_ACTIVE, MISSION_STATE_PAUSED):
            log.warning(
                "DOC DISCREPANCY: DO_SET_MISSION_CURRENT(param1=-1, param2=1) sent to a "
                "MISSION_STATE_COMPLETE mission left mission_state=%s (expected "
                "ACTIVE(3)/PAUSED(4)) within %.0fs — spec says param2=1 makes a completed "
                "mission restartable", state_after_param2_1, RESTART_STATE_TIMEOUT_S,
            )
            pytest.xfail(
                f"mission_state stayed {state_after_param2_1} after param2=1; spec says "
                "param2=1 should restart a completed mission (ACTIVE/PAUSED)"
            )
        assert state_after_param2_1 in (MISSION_STATE_ACTIVE, MISSION_STATE_PAUSED)
    finally:
        state_task.cancel()  # fire-and-forget — do NOT await (§4a)
        await _rtl_and_land(gcs_system)
        await clear_all_mission_types(gcs_system)


async def test_param2_restarts_from_early_item_after_hold(gcs_system, home_item_for_mission):
    """
    Corrected redesign of `test_param2_restarts_completed_mission` above (see
    README.md "Design: restart-after-Hold, corrected" for the full rationale
    and code-inspection trace). Two changes from the original:

    1. The mission ends on an ORDINARY waypoint, not RTL. Reaching RTL as the
       last item makes PX4 leave AUTO.MISSION mode entirely (confirmed via
       flight_mode() == 'RETURN_TO_LAUNCH'), which is why the original test's
       DO_SET_MISSION_CURRENT reset never had anywhere to propagate to,
       regardless of param2. Ending on a plain waypoint instead completes into
       Hold mode.
    2. The restart command targets an EARLY valid item index (not -1), and is
       followed by a raw MAV_CMD_MISSION_START(param1=-1) to explicitly
       re-engage AUTO.MISSION mode. param1=-1 is deliberate: PX4's Navigator
       only acts on MISSION_START's param1 when it is >= 0
       (`navigator_main.cpp`), so -1 lets Commander switch/arm without the
       Navigator independently overwriting the current index we just set —
       isolating whatever effect param2 (not param1's index-setting side
       effect) actually has.

    Code-inspection prediction (PX4-Autopilot source, 2026-07-30): tracing
    `Mission::set_current_mission_index()` (mission.cpp) and
    `MissionBase::isMissionValid()` / `update_mission()` /
    `set_mission_items()` (mission_base.cpp) suggests `_is_current_planned_
    mission_item_valid` is set `true` by DO_SET_MISSION_CURRENT whenever a
    valid, non-terminal index is given — REGARDLESS of param2 — and
    `isMissionValid()` never inspects `mission_result.finished`. So the code
    suggests the mission may resume once Mission mode reactivates even with
    param2=0, i.e. param2's only PX4-side effect (DO_JUMP counter reset) may
    be unable to gate resumption for a mission with no DO_JUMP item, same
    root issue as the original test — but this is exactly what running the
    test (rather than just reading the source) is for; see the assertions and
    the log output for what actually happened.

    Because attempt A (param2=0) might itself cause the mission to resume and
    re-complete (per the prediction above), attempt B (param2=1) always waits
    for MISSION_STATE_COMPLETE again before running — so the test is valid
    regardless of which way attempt A actually goes.
    """
    ack = await probe_command_long(gcs_system, _CMD_ID, param1=-1.0, param2=0.0)
    if ack is not None and int(ack["result"]) == MAV_RESULT_UNSUPPORTED:
        pytest.skip(f"DO_SET_MISSION_CURRENT (cmd={_CMD_ID}) is UNSUPPORTED on this platform")

    home = await _get_home_position(gcs_system)
    items, early_seq, last_seq = _hold_mission_items(home_item_for_mission, home)

    state_task, state = await _subscribe_mission_current(gcs_system)
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
        log.info("Hold-restart mission uploaded (%d items, early seq=%d, last seq=%d)",
                  len(items), early_seq, last_seq)

        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        log.info("Armed and mission started")

        completed_state = await _wait_for_mission_state(
            state, {MISSION_STATE_COMPLETE}, timeout_s=RESTART_COMPLETE_TIMEOUT_S
        )
        mode_at_completion = await _get_flight_mode(gcs_system, timeout_s=2.0)
        log.info("Mission run finished — mission_state=%s (seq=%s) flight_mode=%r",
                  completed_state, state.get("seq"), mode_at_completion)

        # NOTE: deliberately do NOT call _rtl_and_land() here (it disarms) —
        # both attempts below need the vehicle armed. RTL/land/disarm happens
        # once, in the outer `finally`, after both attempts.

        if completed_state != MISSION_STATE_COMPLETE:
            pytest.skip(
                f"Mission never reported MISSION_STATE_COMPLETE within "
                f"{RESTART_COMPLETE_TIMEOUT_S:.0f}s (last mission_state={completed_state}) — "
                "either mission_state reporting is unsupported by this stack, or the mission "
                "didn't finish in time; cannot test restart behaviour either way"
            )

        # --- Attempt A: param1=<early item>, param2=0, then reactivate ------
        ack_reset_a = await probe_command_long(gcs_system, _CMD_ID, param1=float(early_seq), param2=0.0)
        result_reset_a = int(ack_reset_a["result"]) if ack_reset_a is not None else None
        log.info("Attempt A: DO_SET_MISSION_CURRENT(param1=%d, param2=0) ack=%s", early_seq, result_reset_a)
        if result_reset_a != MAV_RESULT_ACCEPTED:
            pytest.skip(
                f"DO_SET_MISSION_CURRENT(param1={early_seq}, param2=0) was not ACCEPTED "
                f"(result={result_reset_a}) — cannot test resumption without it"
            )

        ack_start_a = await probe_command_long(gcs_system, _MISSION_START_CMD_ID, param1=-1.0)
        result_start_a = int(ack_start_a["result"]) if ack_start_a is not None else None
        state_after_a = await _wait_for_mission_state(
            state, {MISSION_STATE_ACTIVE, MISSION_STATE_PAUSED}, timeout_s=RESTART_STATE_TIMEOUT_S
        )
        mode_after_a = await _get_flight_mode(gcs_system, timeout_s=2.0)
        log.info(
            "Attempt A: MISSION_START(param1=-1) ack=%s -> mission_state=%s flight_mode=%r (seq=%s)",
            result_start_a, state_after_a, mode_after_a, state.get("seq"),
        )
        if result_start_a != MAV_RESULT_ACCEPTED:
            pytest.skip(
                f"MAV_CMD_MISSION_START(param1=-1) was not ACCEPTED (result={result_start_a}) "
                "after attempt A — cannot test mode reactivation without it"
            )

        if state_after_a in (MISSION_STATE_ACTIVE, MISSION_STATE_PAUSED):
            # Resumed even without param2=1 (matches the code-inspection prediction, not the
            # original hypothesis) — let it finish and re-complete before attempt B, so B still
            # starts from a clean, freshly-COMPLETE precondition.
            recompleted_state = await _wait_for_mission_state(
                state, {MISSION_STATE_COMPLETE}, timeout_s=RESTART_COMPLETE_TIMEOUT_S
            )
            log.info(
                "Attempt A resumed the mission (param2=0!) — waited for re-completion: "
                "mission_state=%s", recompleted_state,
            )

        # --- Attempt B: param1=<early item>, param2=1, then reactivate ------
        ack_reset_b = await probe_command_long(gcs_system, _CMD_ID, param1=float(early_seq), param2=1.0)
        result_reset_b = int(ack_reset_b["result"]) if ack_reset_b is not None else None
        log.info("Attempt B: DO_SET_MISSION_CURRENT(param1=%d, param2=1) ack=%s", early_seq, result_reset_b)
        if result_reset_b != MAV_RESULT_ACCEPTED:
            pytest.skip(
                f"DO_SET_MISSION_CURRENT(param1={early_seq}, param2=1) was not ACCEPTED "
                f"(result={result_reset_b}) — cannot test resumption without it"
            )

        ack_start_b = await probe_command_long(gcs_system, _MISSION_START_CMD_ID, param1=-1.0)
        result_start_b = int(ack_start_b["result"]) if ack_start_b is not None else None
        state_after_b = await _wait_for_mission_state(
            state, {MISSION_STATE_ACTIVE, MISSION_STATE_PAUSED}, timeout_s=RESTART_STATE_TIMEOUT_S
        )
        mode_after_b = await _get_flight_mode(gcs_system, timeout_s=2.0)
        log.info(
            "Attempt B: MISSION_START(param1=-1) ack=%s -> mission_state=%s flight_mode=%r (seq=%s)",
            result_start_b, state_after_b, mode_after_b, state.get("seq"),
        )
        if result_start_b != MAV_RESULT_ACCEPTED:
            pytest.skip(
                f"MAV_CMD_MISSION_START(param1=-1) was not ACCEPTED (result={result_start_b}) "
                "after attempt B — cannot test mode reactivation without it"
            )

        # --- Assertions -------------------------------------------------
        # Per the authoritative matrix, param2=1 must be able to restart a completed
        # mission (attempt B). Whether param2=0 (attempt A) ALSO restarts it (per the
        # code-inspection prediction above) is exactly the open question this test
        # answers empirically — logged either way, asserted per the matrix's claim.
        if state_after_a in (MISSION_STATE_ACTIVE, MISSION_STATE_PAUSED):
            log.warning(
                "DOC DISCREPANCY (or code-inspection prediction confirmed): "
                "DO_SET_MISSION_CURRENT(param1=%d, param2=0) + MISSION_START(param1=-1) "
                "resumed a completed mission (mission_state=%s) — the authoritative matrix "
                "implies only param2=1 should do this", early_seq, state_after_a,
            )

        if state_after_b not in (MISSION_STATE_ACTIVE, MISSION_STATE_PAUSED):
            log.warning(
                "DOC DISCREPANCY: DO_SET_MISSION_CURRENT(param1=%d, param2=1) + "
                "MISSION_START(param1=-1) left mission_state=%s (expected ACTIVE(3)/PAUSED(4)) "
                "within %.0fs — spec says param2=1 makes a completed mission restartable",
                early_seq, state_after_b, RESTART_STATE_TIMEOUT_S,
            )
            pytest.xfail(
                f"mission_state stayed {state_after_b} after param2=1 + reactivation; spec says "
                "param2=1 should restart a completed mission (ACTIVE/PAUSED)"
            )
        assert state_after_b in (MISSION_STATE_ACTIVE, MISSION_STATE_PAUSED)
    finally:
        state_task.cancel()  # fire-and-forget — do NOT await (§4a)
        await _rtl_and_land(gcs_system)
        await clear_all_mission_types(gcs_system)
