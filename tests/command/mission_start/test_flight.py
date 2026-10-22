"""
MAV_CMD_MISSION_START (cmd=300) — Tier 2 execution tests.

Every flight uploads test_command.py's build_mission() — a takeoff, waypoints
A, B, C in a line along the vehicle's heading, then an approach waypoint and
NAV_LAND — and sends
MISSION_START via COMMAND_INT. Which item the mission is on is read from the
protocol's own signals, not from the vehicle's position:

- MISSION_CURRENT.seq — the item being run (requested at 10 Hz);
- MISSION_ITEM_REACHED.seq — an item has been completed.

So the layout doesn't identify anything; it only has to be flyable by any
vehicle type, which for a fixed-wing means the first waypoint must lie beyond
its climb-out (test_command.py § The mission).

Tests:

- test_mission_start_compat_starts_mission — the XML's one requirement:
  "start running a mission". Armed on the ground, send 0/0: does the mission
  start (the vehicle takes off on the first item)? Also records whether it
  moves past that item, i.e. whether param2 = 0 is read as "whole mission"
  (as every GCS assumes) or literally as "end after item 0".
- test_mission_start_compat_first_item_honoured — param1 = B, then C (two
  values, root CLAUDE.md rule 4c): NACKed, or MISSION_CURRENT goes to that
  item without passing through an earlier waypoint.
- test_mission_start_compat_last_item_honoured — param2 = A, then B: NACKed,
  or once that item is reached the mission doesn't move on to the next one
  within END_HOLD_S (rule 4b — where it settles, not what it passes through).
- test_mission_start_obs_arms_when_disarmed — characterisation: sent to a
  disarmed vehicle, does MISSION_START arm it? The XML doesn't say. PX4 does
  (Commander arms on MISSION_START); ArduPilot only switches to AUTO.

Each verdict test arms first (except the characterisation one), so the same
flow works on a stack that doesn't arm on MISSION_START.

Expected from source (test_command.py's module docstring): PX4 starts the
mission at param1 but never reads param2, so the last-item check should FAIL
(accepted, not honoured — rule 4). ArduPilot DENIES any non-zero param1/param2,
so both param checks pass as REJECTED (NACKed).

Running
-------
PX4 SIH multicopter / fixed-wing::

    pytest tests/command/mission_start/test_flight.py \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx \\
        --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO

    ... --px4-model=sihsim_airplane --vehicle-type=fixed_wing ...
"""

import asyncio
import json
import logging
import os

import pytest

from tests.command.conftest import probe_command_int
from tests.command.mission_start.test_command import SPEC, build_mission
from tests.flight_helpers import (
    AIRBORNE_THRESHOLD_M,
    _get_heading,
    _get_home_position,
    _rtl_and_land,
    _tier2_auto_record,  # noqa: F401 — autouse: records every test's outcome into the combined report
    _wait_armable,
    record_compat_command_supported,
    record_tier2_detail,
    request_message_rate,
    require_real_stack,  # noqa: F401 — registers the real-stack skip gate for this module
)
from tests.land_helpers import combine, record_param
from tests.mission.conftest import (  # noqa: F401
    clear_all_mission_types,
    ensure_mission_int_capability,
    home_item_for_mission,
    requires_home_slot,  # dependency of home_item_for_mission
)
from tests import report
from tests.mock_flight_stack import MAV_RESULT_UNSUPPORTED

log = logging.getLogger(__name__)

# Two flights per test, each: arm, takeoff, up to two legs, a hold, RTL/land (or a stack restart).
pytestmark = pytest.mark.timeout(900)

_CMD_NAME = "MISSION_START"  # read by tests/flight_helpers.py's _tier2_auto_record
_CMD_ID = 300  # MAV_CMD_MISSION_START
report.declare_params("command", _CMD_NAME, [f"{p.slot}_{p.label}" for p in SPEC.params])

REQUIRED_MESSAGES = {"MISSION_CURRENT": "which mission item is running"}  # tests/message_watcher.py

_ACCEPTED = (0, 5)  # MAV_RESULT_ACCEPTED, IN_PROGRESS

# Arm + takeoff to 30 m + up to two 200 m legs: a multicopter at PX4's 5 m/s
# mission speed needs ~90 s. The one dial to widen for a slower stack.
REACH_TIMEOUT_S = float(os.environ.get("MISSION_START_REACH_TIMEOUT_S", "180.0"))
# From the ACK to MISSION_CURRENT reporting the requested first item.
SELECT_TIMEOUT_S = float(os.environ.get("MISSION_START_SELECT_TIMEOUT_S", "10.0"))
# After the last item is reached: a stack that moves on reports the next item
# in MISSION_CURRENT at once, so a few seconds is plenty.
END_HOLD_S = float(os.environ.get("MISSION_START_END_HOLD_S", "5.0"))
ARM_OBSERVE_S = 10.0


class StartWatch:
    """MISSION_CURRENT, MISSION_ITEM_REACHED, altitude and armed state for one flight."""

    def __init__(self, system):
        self.system = system
        self.max_alt = 0.0
        self.came_down = False  # back below AIRBORNE_THRESHOLD_M after being airborne (landed or crashed)
        self.armed = False
        self.seqs: list[int] = []     # distinct MISSION_CURRENT seqs, in order
        self.reached: list[int] = []  # MISSION_ITEM_REACHED seqs, in order
        self.changed = asyncio.Event()  # set on every update
        self._tasks: list[asyncio.Task] = []

    def start(self) -> None:
        self._tasks = [asyncio.create_task(f()) for f in
                       (self._positions, self._armed, self._mission_current, self._item_reached)]

    def stop(self) -> None:
        for t in self._tasks:
            t.cancel()

    @property
    def airborne(self) -> bool:
        return self.max_alt >= AIRBORNE_THRESHOLD_M

    async def _positions(self) -> None:
        async for pos in self.system.telemetry.position():
            if self.airborne and pos.relative_altitude_m < AIRBORNE_THRESHOLD_M:
                self.came_down = True
            if pos.relative_altitude_m > self.max_alt:
                self.max_alt = pos.relative_altitude_m
                self.changed.set()

    async def _armed(self) -> None:
        async for armed in self.system.telemetry.armed():
            if armed != self.armed:
                self.armed = armed
                self.changed.set()

    async def _mission_current(self) -> None:
        async for msg in self.system.mavlink_direct.message("MISSION_CURRENT"):
            seq = int(json.loads(msg.fields_json)["seq"])
            if not self.seqs or self.seqs[-1] != seq:
                self.seqs.append(seq)
                log.info("MISSION_CURRENT: %d", seq)
                self.changed.set()

    async def _item_reached(self) -> None:
        async for msg in self.system.mavlink_direct.message("MISSION_ITEM_REACHED"):
            seq = int(json.loads(msg.fields_json)["seq"])
            if seq not in self.reached:
                self.reached.append(seq)
                log.info("MISSION_ITEM_REACHED: %d", seq)
                self.changed.set()

    async def wait_for(self, predicate, timeout_s: float) -> bool:
        """True once predicate() holds (checked on every update) within timeout_s."""
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout_s
        while not predicate():
            remaining = deadline - loop.time()
            if remaining <= 0:
                return False
            self.changed.clear()
            try:
                await asyncio.wait_for(self.changed.wait(), remaining)
            except asyncio.TimeoutError:
                return predicate()
        return True

    def brief(self) -> str:
        return (f"MISSION_CURRENT {self.seqs or 'none'}, MISSION_ITEM_REACHED {self.reached or 'none'}, "
                f"max alt {self.max_alt:.1f} m, armed={self.armed}")


async def _send(system, param1: float, param2: float) -> int | None:
    """MISSION_START via COMMAND_INT (primary type — no location, no float in params 5/6)."""
    kw = dict(SPEC.baseline)
    ack = await probe_command_int(
        system, _CMD_ID, frame=6, param1=param1, param2=param2, param3=kw["param3"], param4=kw["param4"],
        x=kw["int_x"], y=kw["int_y"], z=kw["int_z"],
    )
    return None if ack is None else int(ack["result"])


async def _fly(system, home_item, restart, *, param1: float, param2: float, arm_first: bool, observe) -> dict:
    """
    Upload the mission, (arm,) send MISSION_START(param1, param2), run
    `observe(watch)` for whatever the test measures, then bring the vehicle
    home. Returns {"result", "nacked", "watch", **observe's result}.
    """
    home = await _get_home_position(system)
    items, _ = build_mission((home.latitude_deg, home.longitude_deg), home_item, await _get_heading(system))
    await clear_all_mission_types(system)
    await ensure_mission_int_capability(system)
    await system.mission_raw.upload_mission(items)
    await request_message_rate(system, 42, 10.0)  # MISSION_CURRENT
    await _wait_armable(system)
    watch = StartWatch(system)
    watch.start()
    started = False  # armed by us, or MISSION_START accepted (which arms on PX4)
    try:
        if arm_first:
            started = True
            await system.action.arm()
        result = await _send(system, param1, param2)
        log.info("MISSION_START(param1=%s, param2=%s): result=%s", param1, param2, result)
        out = {"result": result, "nacked": result is not None and result not in _ACCEPTED, "watch": watch}
        started |= result is not None and not out["nacked"]
        if result is None:
            pytest.fail("No COMMAND_ACK for MISSION_START — cannot classify")
        if not out["nacked"]:
            out.update(await observe(watch))
        return out
    finally:
        if not watch.airborne:
            try:
                await system.action.disarm()  # armed on the ground (or already disarmed): RTL would only take off
            except Exception as exc:  # noqa: BLE001 — refused, e.g. a fixed-wing mid takeoff run; RTL below
                log.info("disarm on the ground: %r", exc)
            await watch.wait_for(lambda: not watch.armed, 3.0)
        watch.stop()
        if watch.armed:
            await _rtl_and_land(system, restart)
        elif started and not watch.airborne:
            # Started but never left the ground: the vehicle may not be fit to arm
            # again. A PX4 fixed-wing started at a waypoint from the ground banks
            # towards it on the runway and is left tipped over ("Preflight Fail:
            # Attitude failure (roll)") — see README.md § Results. The armed
            # state can't tell us: HEARTBEAT is 1 Hz, and that flight arms and
            # disarms inside a second.
            log.info("started but never airborne — restarting the flight stack for a clean vehicle")
            restart()
        await clear_all_mission_types(system)
        try:
            await system.action.hold()  # PX4 isn't armable in Mission mode once the mission is cleared
        except Exception as exc:  # noqa: BLE001 — best effort
            log.warning("could not switch to Hold after the flight: %r", exc)


def _seq(name: str, home_item) -> int:
    """The seq of `name` in build_mission() — depends only on whether there's a home slot, not on home itself."""
    return build_mission((0.0, 0.0), home_item)[1]["seq"][name]


def _value(label: str, r: dict, ok: bool | None, detail: str) -> dict:
    return {"ok": bool(ok), "nacked": r["nacked"],
            "detail": f"{label}: " + (f"NACKed (result={r['result']})" if r["nacked"] else detail)}


# ---------------------------------------------------------------------------
# Command support — "start running a mission"
# ---------------------------------------------------------------------------

async def test_mission_start_compat_starts_mission(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    param1 = 0, param2 = 0, armed on the ground — PASS if the mission starts (the vehicle takes off on the first item).

    Also records, as characterisation, whether the mission moves past the
    takeoff: taken literally, param2 = 0 means "end after item 0", but it is
    what GCSs send for the whole mission (see README.md § Spec gaps).
    """
    t = _seq("T", home_item_for_mission)

    async def observe(watch):
        await watch.wait_for(lambda: watch.airborne, REACH_TIMEOUT_S)
        continued = await watch.wait_for(lambda: any(s > t for s in watch.seqs), REACH_TIMEOUT_S)
        return {"continued": continued}

    r = await _fly(gcs_system, home_item_for_mission, restart_flight_stack,
                   param1=0.0, param2=0.0, arm_first=True, observe=observe)
    if r["result"] == MAV_RESULT_UNSUPPORTED:
        record_compat_command_supported(request, False)
        pytest.skip("NA: MISSION_START is UNSUPPORTED on this stack")
    if r["nacked"]:
        pytest.fail(f"MISSION_START 0/0 refused (result={r['result']}) with a mission loaded and the vehicle "
                    "armed — inconclusive, the baseline itself didn't start")
    w = r["watch"]
    continued = ("moved past the takeoff (param2 = 0 read as the whole mission)" if r["continued"]
                 else "did NOT move past the takeoff — mission ended after item 0, or ran out of time")
    record_tier2_detail(request, f"PASS if the vehicle takes off: {w.brief()}; {continued}")
    record_compat_command_supported(request, w.airborne,
                                    notes=None if w.airborne else "Accepted, but the mission does not start")
    if not w.airborne:
        report.compat_fail(f"MISSION_START accepted but the mission did not start: {w.brief()}")


# ---------------------------------------------------------------------------
# param1 (First Item) / param2 (Last Item) — "is it honoured" (rules 4, 4a-4c)
# ---------------------------------------------------------------------------

async def test_mission_start_compat_first_item_honoured(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    param1 = waypoint B, then C — "is First Item honoured": NACKed, or MISSION_CURRENT goes to that item without passing through an earlier waypoint.

    Before the start MISSION_CURRENT shows the takeoff (or an earlier item), so
    those are ignored; a stack that ignored param1 would show waypoint A next.
    """
    t = _seq("T", home_item_for_mission)
    values = []
    for name in ("B", "C"):
        k = _seq(name, home_item_for_mission)

        async def observe(watch, k=k):
            selected = await watch.wait_for(lambda: k in watch.seqs, SELECT_TIMEOUT_S)
            return {"selected": selected, "skipped_over": [s for s in watch.seqs if t < s < k]}

        r = await _fly(gcs_system, home_item_for_mission, restart_flight_stack,
                       param1=float(k), param2=0.0, arm_first=True, observe=observe)
        ok = not r["nacked"] and r["selected"] and not r["skipped_over"]
        detail = ("" if r["nacked"] else
                  f"param1 = {k}, MISSION_CURRENT {'reached' if r['selected'] else 'did NOT reach'} {k} within "
                  f"{SELECT_TIMEOUT_S:.0f} s" + (f" after earlier items {r['skipped_over']}" if r["skipped_over"] else "")
                  + f" ({r['watch'].brief()})")
        values.append(_value(name, r, ok, detail))
    record_param(request, "param1 (First Item)", ("1_First Item",), combine(values),
                 "PASS if each value is NACKed or MISSION_CURRENT goes straight to that item")


async def test_mission_start_compat_last_item_honoured(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    param2 = waypoint A, then B — "is Last Item honoured": NACKed, or once that item is reached the mission doesn't move on to the next.

    "After this item is run, the mission ends": MISSION_ITEM_REACHED reports
    the item (or MISSION_CURRENT moves past it, which means it was reached),
    and END_HOLD_S later MISSION_CURRENT has not gone past it.
    """
    values = []
    for name in ("A", "B"):
        j = _seq(name, home_item_for_mission)

        async def observe(watch, j=j):
            done = await watch.wait_for(lambda: j in watch.reached or any(s > j for s in watch.seqs), REACH_TIMEOUT_S)
            if not done:
                return {"done": False}
            came_down = watch.came_down
            await asyncio.sleep(END_HOLD_S)
            return {"done": True, "came_down": came_down, "moved_on": [s for s in watch.seqs if s > j]}

        r = await _fly(gcs_system, home_item_for_mission, restart_flight_stack,
                       param1=0.0, param2=float(j), arm_first=True, observe=observe)
        if not r["nacked"] and not r["done"]:
            pytest.fail(f"param2 = {j} ({name}): item {j} not reached within {REACH_TIMEOUT_S:.0f} s — inconclusive; "
                        f"likely insufficient time for this stack, widen MISSION_START_REACH_TIMEOUT_S "
                        f"({r['watch'].brief()})")
        if not r["nacked"] and r["came_down"]:
            # The mission logic may still run past the item, but a vehicle that
            # came back down isn't flying the mission — rule 10: no verdict from it.
            pytest.fail(f"param2 = {j} ({name}): the vehicle came back down after takeoff before item {j} was "
                        f"passed — crashed or landed, inconclusive ({r['watch'].brief()})")
        ok = not r["nacked"] and not r["moved_on"]
        detail = ("" if r["nacked"] else
                  f"param2 = {j}, " + (f"mission moved on to {r['moved_on']}" if r["moved_on"]
                                       else f"still on item {j} {END_HOLD_S:.0f} s after reaching it")
                  + f" ({r['watch'].brief()})")
        values.append(_value(name, r, ok, detail))
    record_param(request, "param2 (Last Item)", ("2_Last Item",), combine(values),
                 f"PASS if each value is NACKed or the mission stays on that item ({END_HOLD_S:.0f} s after reaching it)")


# ---------------------------------------------------------------------------
# Characterisation
# ---------------------------------------------------------------------------

async def test_mission_start_obs_arms_when_disarmed(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """Characterisation: MISSION_START 0/0 sent to a disarmed vehicle — does it arm (and start)? The XML doesn't say."""
    async def observe(watch):
        await watch.wait_for(lambda: watch.armed and watch.airborne, ARM_OBSERVE_S)
        return {}

    r = await _fly(gcs_system, home_item_for_mission, restart_flight_stack,
                   param1=0.0, param2=0.0, arm_first=False, observe=observe)
    if r["nacked"]:
        record_tier2_detail(request, f"Observational: refused while disarmed (result={r['result']})")
        return
    w = r["watch"]
    armed = w.armed or w.airborne
    record_tier2_detail(request, f"Observational: within {ARM_OBSERVE_S:.0f} s of MISSION_START the vehicle "
                        f"{'armed' if armed else 'stayed disarmed'}"
                        f"{' and took off' if w.airborne else ''} ({w.brief()})")
