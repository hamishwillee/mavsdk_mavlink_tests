"""
MAV_CMD_MISSION_START (cmd=300) — Tier 2 execution tests.

Every flight uploads test_command.py's build_mission() — a takeoff at home,
then waypoints N, E, S and W of home — and sends MISSION_START via COMMAND_INT.
Which waypoint the vehicle reaches (within REACH_RADIUS_M) identifies which
item the mission is on: from home or any waypoint, the path to any other
waypoint passes nowhere near a third one.

Tests:

- test_mission_start_compat_starts_mission — the XML's one requirement:
  "start running a mission". Armed on the ground, send 0/0: does the mission
  start (the vehicle takes off on item 0)? Also records whether it continues
  past item 0, i.e. whether param2 = 0 is read as "whole mission" (as every
  GCS assumes) or literally as "end after item 0".
- test_mission_start_compat_first_item_honoured — param1 = E, then S (two
  values, root CLAUDE.md rule 4c): NACKed, or the first waypoint the vehicle
  reaches is that item.
- test_mission_start_compat_last_item_honoured — param2 = N, then E: NACKed,
  or the vehicle reaches that item, stays there for LAST_ITEM_HOLD_S and
  never reaches the next waypoint (rule 4b — where it settles, not what it
  passes through).
- test_mission_start_obs_arms_when_disarmed — characterisation: sent to a
  disarmed vehicle, does MISSION_START arm it? The XML doesn't say. PX4 does
  (Commander arms on MISSION_START); ArduPilot only switches to AUTO.

Each verdict test arms first (except the characterisation one), so the same
flow works on a stack that doesn't arm on MISSION_START.

Expected from source (test_command.py's module docstring): PX4 starts the
mission at param1 but never reads param2, so the last-item check should FAIL
(accepted, not honoured — rule 4). ArduPilot DENIES any non-zero param1/param2,
so both param checks pass as REJECTED (NACKed).

Geometry suits a hovering vehicle (40 m spacing). A fixed-wing would need
MISSION_START_WAYPOINT_SPACING_M in the hundreds of metres and a landing
appended to satisfy PX4's MIS_TKO_LAND_REQ — neither is done here.

Running
-------
PX4 SIH multicopter::

    pytest tests/command/mission_start/test_flight.py \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx \\
        --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO
"""

import asyncio
import json
import logging
import os

import pytest

from tests.command.conftest import probe_command_int
from tests.command.mission_start.test_command import SPEC, WAYPOINTS, build_mission
from tests.flight_helpers import (
    AIRBORNE_THRESHOLD_M,
    _dist_m,
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

# Two flights per test, each: arm, takeoff, up to two 40 m legs, a hold, RTL/land.
pytestmark = pytest.mark.timeout(900)

_CMD_NAME = "MISSION_START"  # read by tests/flight_helpers.py's _tier2_auto_record
_CMD_ID = 300  # MAV_CMD_MISSION_START
report.declare_params("command", _CMD_NAME, [f"{p.slot}_{p.label}" for p in SPEC.params])

_ACCEPTED = (0, 5)  # MAV_RESULT_ACCEPTED, IN_PROGRESS

REACH_RADIUS_M = float(os.environ.get("MISSION_START_REACH_RADIUS_M", "8.0"))
# Arm + takeoff to 15 m + two 40 m legs. The one dial to widen for a slower stack.
REACH_TIMEOUT_S = float(os.environ.get("MISSION_START_REACH_TIMEOUT_S", "90.0"))
# After the last item is reached: long enough to fly the next 57 m leg (N→E)
# at a few m/s, so a stack that ignores param2 is seen reaching the next waypoint.
LAST_ITEM_HOLD_S = float(os.environ.get("MISSION_START_LAST_ITEM_HOLD_S", "30.0"))
ARM_OBSERVE_S = 10.0

_NEXT = {name: WAYPOINTS[(i + 1) % len(WAYPOINTS)][0] for i, (name, _, _) in enumerate(WAYPOINTS)}


class StartWatch:
    """Position, armed state and MISSION_CURRENT for one flight; which waypoints were reached, in order."""

    def __init__(self, system, layout: dict):
        self.system = system
        self.layout = layout
        self.lat = self.lon = None
        self.alt = 0.0
        self.max_alt = 0.0
        self.armed = False
        self.seqs: list[int] = []        # distinct MISSION_CURRENT seqs, in order
        self.reached: list[str] = []     # waypoint names, in the order first reached
        self.changed = asyncio.Event()   # set on every new waypoint reached
        self._tasks: list[asyncio.Task] = []

    def start(self) -> None:
        self._tasks = [asyncio.create_task(f()) for f in (self._positions, self._armed, self._mission_current)]

    def stop(self) -> None:
        for t in self._tasks:
            t.cancel()  # gRPC streams: never await after cancel (root CLAUDE.md design decision 4a)

    def dist_to(self, name: str) -> float | None:
        if self.lat is None:
            return None
        return _dist_m(self.lat, self.lon, *self.layout["pos"][name])

    async def _positions(self) -> None:
        async for pos in self.system.telemetry.position():
            self.lat, self.lon, self.alt = pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m
            self.max_alt = max(self.max_alt, self.alt)
            if self.alt < AIRBORNE_THRESHOLD_M:
                continue
            for name in self.layout["pos"]:
                if name not in self.reached and self.dist_to(name) <= REACH_RADIUS_M:
                    self.reached.append(name)
                    log.info("MISSION_START flight: reached waypoint %s (seq %d), MISSION_CURRENT seqs %s",
                             name, self.layout["seq"][name], self.seqs)
                    self.changed.set()

    async def _armed(self) -> None:
        async for armed in self.system.telemetry.armed():
            self.armed = armed

    async def _mission_current(self) -> None:
        async for msg in self.system.mavlink_direct.message("MISSION_CURRENT"):
            seq = int(json.loads(msg.fields_json)["seq"])
            if not self.seqs or self.seqs[-1] != seq:
                self.seqs.append(seq)

    async def wait_reached(self, name: str, timeout_s: float) -> bool:
        """True once `name` has been reached (or already was) within timeout_s."""
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout_s
        while name not in self.reached:
            remaining = deadline - loop.time()
            if remaining <= 0:
                return False
            self.changed.clear()
            try:
                await asyncio.wait_for(self.changed.wait(), remaining)
            except asyncio.TimeoutError:
                return False
        return True

    async def wait_first_reached(self, timeout_s: float) -> str | None:
        try:
            await asyncio.wait_for(self.changed.wait(), timeout_s)
        except asyncio.TimeoutError:
            return None
        return self.reached[0]

    def brief(self) -> str:
        return (f"max alt {self.max_alt:.1f} m, reached {self.reached or 'none'}, "
                f"MISSION_CURRENT seqs {self.seqs}, armed={self.armed}")


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
    items, layout = build_mission((home.latitude_deg, home.longitude_deg), home_item)
    await clear_all_mission_types(system)
    await ensure_mission_int_capability(system)
    await system.mission_raw.upload_mission(items)
    await request_message_rate(system, 42, 10.0)  # MISSION_CURRENT
    await _wait_armable(system)
    watch = StartWatch(system, layout)
    watch.start()
    try:
        if arm_first:
            await system.action.arm()
        result = await _send(system, param1, param2)
        log.info("MISSION_START(param1=%s, param2=%s): result=%s", param1, param2, result)
        out = {"result": result, "nacked": result is not None and result not in _ACCEPTED, "watch": watch,
               "layout": layout}
        if result is None:
            pytest.fail("No COMMAND_ACK for MISSION_START — cannot classify")
        if not out["nacked"]:
            out.update(await observe(watch))
        return out
    finally:
        watch.stop()
        if watch.max_alt < AIRBORNE_THRESHOLD_M:
            try:
                await system.action.disarm()  # armed on the ground (or already disarmed): RTL would only take off
            except Exception as exc:  # noqa: BLE001 — best effort; PX4 auto-disarms anyway
                log.info("disarm on the ground: %r", exc)
        else:
            await _rtl_and_land(system, restart)
        await clear_all_mission_types(system)
        try:
            await system.action.hold()  # PX4 isn't armable in Mission mode once the mission is cleared
        except Exception as exc:  # noqa: BLE001 — best effort
            log.warning("could not switch to Hold after the flight: %r", exc)


def _require_airborne(r: dict, what: str) -> None:
    """A verdict needs a flight that happened (rule 10): otherwise it proves nothing about `what`."""
    w = r["watch"]
    if w.max_alt < AIRBORNE_THRESHOLD_M:
        pytest.fail(f"MISSION_START accepted but the vehicle never got airborne — {what} inconclusive ({w.brief()})")


# ---------------------------------------------------------------------------
# Command support — "start running a mission"
# ---------------------------------------------------------------------------

async def test_mission_start_compat_starts_mission(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    param1 = 0, param2 = 0, armed on the ground — PASS if the mission starts (the vehicle takes off on item 0).

    Also records, as characterisation, whether the mission continues past
    item 0: taken literally, param2 = 0 means "end after item 0", but it is
    what GCSs send for the whole mission (see README.md § Spec gaps).
    """
    async def observe(watch):
        n_reached = await watch.wait_reached("N", REACH_TIMEOUT_S)
        return {"continued": n_reached}

    r = await _fly(gcs_system, home_item_for_mission, restart_flight_stack,
                   param1=0.0, param2=0.0, arm_first=True, observe=observe)
    if r["result"] == MAV_RESULT_UNSUPPORTED:
        record_compat_command_supported(request, False)
        pytest.skip("NA: MISSION_START is UNSUPPORTED on this stack")
    if r["nacked"]:
        pytest.fail(f"MISSION_START 0/0 refused (result={r['result']}) with a mission loaded and the vehicle "
                    "armed — inconclusive, the baseline itself didn't start")
    w = r["watch"]
    started = w.max_alt >= AIRBORNE_THRESHOLD_M
    continued = ("continued past item 0 (param2 = 0 read as the whole mission)" if r["continued"]
                 else "did NOT reach the first waypoint — mission ended after item 0, or ran out of time")
    record_tier2_detail(request, f"PASS if the vehicle takes off: {w.brief()}; {continued}")
    record_compat_command_supported(request, started,
                                    notes=None if started else "Accepted, but the mission does not start")
    if not started:
        report.compat_fail(f"MISSION_START accepted but the mission did not start: {w.brief()}")


# ---------------------------------------------------------------------------
# param1 (First Item) / param2 (Last Item) — "is it honoured" (rules 4, 4a-4c)
# ---------------------------------------------------------------------------

def _seq(name: str, home_item) -> int:
    """The seq of `name` in build_mission() — depends only on whether there's a home slot, not on home itself."""
    return build_mission((0.0, 0.0), home_item)[1]["seq"][name]


def _value(name: str, r: dict, ok: bool | None, detail: str) -> dict:
    return {"ok": bool(ok), "nacked": r["nacked"],
            "detail": f"{name}: " + (f"NACKed (result={r['result']})" if r["nacked"] else detail)}


async def test_mission_start_compat_first_item_honoured(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """param1 = waypoint E, then S — "is First Item honoured": NACKed, or the first waypoint reached is that item."""
    values = []
    for name in ("E", "S"):
        async def observe(watch):
            return {"first": await watch.wait_first_reached(REACH_TIMEOUT_S)}

        k = _seq(name, home_item_for_mission)
        r = await _fly(gcs_system, home_item_for_mission, restart_flight_stack,
                       param1=float(k), param2=0.0, arm_first=True, observe=observe)
        if not r["nacked"]:
            _require_airborne(r, "First Item")
            if r["first"] is None:
                pytest.fail(f"param1 = {k} ({name}): no waypoint reached within {REACH_TIMEOUT_S:.0f} s — inconclusive; "
                            f"likely insufficient time for this stack ({r['watch'].brief()})")
        values.append(_value(name, r, not r["nacked"] and r["first"] == name,
                             f"param1 = {k}, first waypoint reached {r.get('first')} ({r['watch'].brief()})"))
    record_param(request, "param1 (First Item)", ("1_First Item",), combine(values),
                 "PASS if each value is NACKed or the mission's first waypoint is that item")


async def test_mission_start_compat_last_item_honoured(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    param2 = waypoint N, then E — "is Last Item honoured": NACKed, or the vehicle settles at that item and never reaches the next.

    "After this item is run, the mission ends": the vehicle reaches the item,
    then stays — LAST_ITEM_HOLD_S later it is still within REACH_RADIUS_M of it
    and has not reached the following waypoint.
    """
    values = []
    for name in ("N", "E"):
        nxt = _NEXT[name]

        async def observe(watch, name=name, nxt=nxt):
            if not await watch.wait_reached(name, REACH_TIMEOUT_S):
                return {"reached": False}
            await asyncio.sleep(LAST_ITEM_HOLD_S)
            return {"reached": True, "moved_on": nxt in watch.reached, "settled_dist": watch.dist_to(name)}

        j = _seq(name, home_item_for_mission)
        r = await _fly(gcs_system, home_item_for_mission, restart_flight_stack,
                       param1=0.0, param2=float(j), arm_first=True, observe=observe)
        if not r["nacked"]:
            _require_airborne(r, "Last Item")
            if not r["reached"]:
                pytest.fail(f"param2 = {j} ({name}): waypoint {name} not reached within {REACH_TIMEOUT_S:.0f} s — "
                            f"inconclusive; likely insufficient time for this stack ({r['watch'].brief()})")
        ok = (not r["nacked"]) and not r["moved_on"] and r["settled_dist"] <= REACH_RADIUS_M
        detail = ("" if r["nacked"] else
                  f"param2 = {j}, {LAST_ITEM_HOLD_S:.0f} s after reaching {name}: "
                  f"{'reached the next waypoint ' + nxt if r['moved_on'] else 'did not reach ' + nxt}, "
                  f"{r['settled_dist']:.1f} m from {name} ({r['watch'].brief()})")
        values.append(_value(name, r, ok, detail))
    record_param(request, "param2 (Last Item)", ("2_Last Item",), combine(values),
                 f"PASS if each value is NACKed or the vehicle settles at that item (within {REACH_RADIUS_M:.0f} m "
                 f"after {LAST_ITEM_HOLD_S:.0f} s) without reaching the next")


# ---------------------------------------------------------------------------
# Characterisation
# ---------------------------------------------------------------------------

async def test_mission_start_obs_arms_when_disarmed(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """Characterisation: MISSION_START 0/0 sent to a disarmed vehicle — does it arm (and start)? The XML doesn't say."""
    async def observe(watch):
        loop = asyncio.get_running_loop()
        deadline = loop.time() + ARM_OBSERVE_S
        while loop.time() < deadline and not (watch.armed and watch.max_alt >= AIRBORNE_THRESHOLD_M):
            await asyncio.sleep(0.5)
        return {}

    r = await _fly(gcs_system, home_item_for_mission, restart_flight_stack,
                   param1=0.0, param2=0.0, arm_first=False, observe=observe)
    if r["nacked"]:
        record_tier2_detail(request, f"Observational: refused while disarmed (result={r['result']})")
        return
    w = r["watch"]
    armed = w.armed or w.max_alt >= AIRBORNE_THRESHOLD_M
    record_tier2_detail(request, f"Observational: within {ARM_OBSERVE_S:.0f} s of MISSION_START the vehicle "
                        f"{'armed' if armed else 'stayed disarmed'}"
                        f"{' and took off' if w.max_alt >= AIRBORNE_THRESHOLD_M else ''} ({w.brief()})")
