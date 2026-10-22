"""
MAV_CMD_MISSION_START (cmd=300) — Tier 1 ACK tests.

"start running a mission" (common.xml). hasLocation="false" isDestination="false".

Parameter layout (common.xml)::

    param1  First Item  (minValue=0, increment=1)  first_item: the first mission item to run           DEFINED
    param2  Last Item   (minValue=0, increment=1)  last_item: the last mission item to run (after this   DEFINED
                                                   item is run, the mission ends)
    param3-7  (omitted from the XML — Empty)                                                           UNDEFINED

No location and no float in params 5/6, so COMMAND_INT is the primary message
type (tests/command/CLAUDE.md § selection rules, rule 3); the mandatory common
checks send both types regardless.

This command is stateful: what a stack answers depends on whether a mission is
loaded, so the file has two classes.

- TestMissionStartNoMission — no mission uploaded. Small: confirms a start is
  refused when there is nothing to run.
- TestMissionStartCommand — a mission is uploaded once for the class
  (_mission_loaded_cls) and every other check runs against it, including the
  seven mandatory common checks inherited from Tier1CommandTestBase. Without a
  mission, PX4 DENIES every send regardless of params, which would make "a
  real value in an Empty param is rejected" pass for the wrong reason.

**On PX4, an accepted MISSION_START arms the vehicle and flies the mission**
(Commander: switch to AUTO_MISSION, then arm). Every accepted probe in
TestMissionStartCommand is therefore a real start: the vehicle takes off on the
first one and keeps flying (re-starting the mission) while the class runs. The
class fixture waits until the vehicle is armable first, so the baseline is
answered ACCEPTED rather than TEMPORARILY_REJECTED, and on teardown brings it
home with RTL (restarting the stack if it doesn't land — root CLAUDE.md Tier 2
pattern #8). ArduPilot only switches to AUTO; it does not arm.

What each stack does with the params (source, read 2026-10-22)
-------------------------------------------------------------
PX4 main (Commander.cpp, VEHICLE_CMD_MISSION_START; navigator_main.cpp):
  - no valid mission → DENIED ("Mission start denied! No valid mission").
  - param1 must be finite, >= -1 and < the mission item count, else DENIED.
    -1 is PX4's own value for "start from the current item": Navigator only
    calls set_current_mission_index() for param1 >= 0. The XML's minValue is 0.
  - param2 is never read.
  - mode change or arming refused → TEMPORARILY_REJECTED.
  - cmd 300 has no entry in mavlink_command_params.hpp, so params 3-7 are not
    validated at the MAVLink boundary.
ArduPilot master (GCS_MAVLink_{Copter,Plane,Rover}.cpp):
  - any non-zero param1 or param2 → DENIED ("first-item/last item not
    supported"). NaN is non-zero, so NaN is DENIED too.
  - otherwise switch to AUTO: Copter/Rover ACCEPTED or FAILED (mode change
    refused); Plane always ACCEPTED. Params 3-7 are not read.

Spec gaps this file surfaces (see README.md § Spec gaps)
---------------------------------------------------------
- param2 = 0 literally means "end after item 0", yet 0/0 is what GCSs send to
  run the whole mission and the only value ArduPilot accepts. The XML defines
  no "run to the end" value. The baseline here is 0/0 for that reason.
- Neither param defines a sentinel; both stacks DENY NaN in param1.

Running
-------
Paired mock (no autopilot)::

    pytest tests/command/mission_start/test_command.py -v --log-cli-level=INFO

PX4 SIH multicopter::

    pytest tests/command/mission_start/test_command.py \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx \\
        --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO

    ... --px4-model=sihsim_airplane --vehicle-type=fixed_wing ...
"""

import logging
import math
import os

import pytest
import pytest_asyncio
from mavsdk.plugins.mission_raw import MissionItem

from tests import report
from tests.command.conftest import (
    CommandSpec,
    INT32_MAX,
    ParamSpec,
    Tier1CommandTestBase,
    _check,
    _record,
    _reduce_dual,
    _ACK_WINDOW_S,
    probe_dual,
    gcs_system_cls,  # noqa: F401 — class-scoped fixtures, see do_set_global_origin/test_command.py
    mock_stack_cls,  # noqa: F401
)
from tests.flight_helpers import (
    _get_heading,
    _get_home_position,
    _offset_lat_lon,
    _rtl_and_land,
    _wait_armable,
    mission_landing_items,
)
from tests.mission.conftest import (  # noqa: F401
    clear_all_mission_types,
    ensure_mission_int_capability,
    home_item_for_mission,
    requires_home_slot,  # dependency of home_item_for_mission
)
from tests.mock_flight_stack import MAV_RESULT_UNSUPPORTED
from tests.report import _tier1_auto_record  # noqa: F401 — autouse: records the bespoke tests too

log = logging.getLogger(__name__)

_CMD = "MISSION_START"
_CMD_ID = 300  # MAV_CMD_MISSION_START
_CMD_NAME = _CMD  # read by tests/report.py's _tier1_auto_record

_ACCEPTED = (0, 5)  # MAV_RESULT_ACCEPTED, IN_PROGRESS

# ---------------------------------------------------------------------------
# The mission — shared with test_flight.py
# ---------------------------------------------------------------------------
#
# A takeoff, then waypoints A, B and C in a straight line along the
# vehicle's heading, then the shared approach waypoint + NAV_LAND. Which item
# the mission is on is read from MISSION_CURRENT / MISSION_ITEM_REACHED, not
# from where the vehicle is — the layout only has to be flyable by any vehicle
# type, which takes three things:
#   - waypoints ahead, on the line the vehicle takes off along: no turns;
#   - the first waypoint beyond a fixed-wing's climb-out. At 60 m, PX4 SIH
#     fixed-wing flew over it while still climbing, turned back at 30 m, lost
#     speed in the turn and hit the ground (2026-10-22, README.md § Results);
#   - a landing, for PX4 fixed-wing/VTOL's MIS_TKO_LAND_REQ — without it PX4
#     refuses the mission and DENIES every MISSION_START
#     (tests/mission/CLAUDE.md § "Mission shape a stack will fly").

MISSION_ALT_M = 30.0
WAYPOINT_SPACING_M = float(os.environ.get("MISSION_START_WAYPOINT_SPACING_M", "200.0"))
WAYPOINTS = ("A", "B", "C")  # 1, 2, 3 × WAYPOINT_SPACING_M from home along the bearing

# Mock mode has no telemetry home; any plausible point will do (PX4 SIH default).
_MOCK_HOME = (47.3977, 8.5456)


def build_mission(home_lat_lon: tuple[float, float], home_item: MissionItem | None,
                  bearing_deg: float = 0.0) -> tuple[list[MissionItem], dict]:
    """
    [home slot (ArduPilot)], NAV_TAKEOFF, NAV_WAYPOINTs A/B/C along
    `bearing_deg` (pass the vehicle's heading), then an approach waypoint and
    NAV_LAND further along the same line (mission_landing_items()).

    Returns (items, layout): layout["seq"][name] is each item's seq — "T" for
    the takeoff, "A"/"B"/"C", "APPROACH" and "LAND".
    """
    lat0, lon0 = home_lat_lon
    b = math.radians(bearing_deg)

    def along(d: float) -> tuple[float, float]:
        return _offset_lat_lon(lat0, lon0, d * math.cos(b), d * math.sin(b))

    # The takeoff's position is ahead on the line too: PX4 fixed-wing climbs out
    # towards it, and with it at home turned back through ~200° during the climb.
    body = [("T", 22, *along(WAYPOINT_SPACING_M / 2))]  # MAV_CMD_NAV_TAKEOFF
    for i, name in enumerate(WAYPOINTS, start=1):
        body.append((name, 16, *along(i * WAYPOINT_SPACING_M)))  # MAV_CMD_NAV_WAYPOINT
    items: list[MissionItem] = []
    if home_item is not None:
        items.append(home_item)
    layout: dict = {"seq": {}}
    for i, (name, command, lat, lon) in enumerate(body):
        seq = len(items)
        items.append(MissionItem(
            seq=seq, frame=6, command=command, current=1 if i == 0 else 0, autocontinue=1,
            param1=0.0, param2=0.0, param3=0.0, param4=float("nan"),
            x=int(lat * 1e7), y=int(lon * 1e7), z=MISSION_ALT_M, mission_type=0,
        ))
        layout["seq"][name] = seq
    last = items[-1]
    landing = mission_landing_items(items, anchor_lat_lon_int=(last.x, last.y), bearing_deg=bearing_deg)
    for name, item in zip(("APPROACH", "LAND"), landing):
        layout["seq"][name] = item.seq
        items.append(item)
    return items, layout


# ---------------------------------------------------------------------------
# Spec
# ---------------------------------------------------------------------------

_NEVER_READ = (
    "Neither stack reads param{slot}: PX4 has no mavlink_command_params.hpp entry for cmd 300 "
    "(params unchecked at the MAVLink boundary) and Commander uses only param1; ArduPilot's "
    "handlers read only param1/param2"
)

SPEC = CommandSpec(
    cmd_id=_CMD_ID,
    name=_CMD,
    # No location and no float in params 5/6 — check 7 is NA in both directions.
    baseline=dict(
        param1=0.0,   # First Item: the first item
        param2=0.0,   # Last Item: 0 — what GCSs send for "the whole mission" (see module docstring)
        param3=None, param4=None,
        long5=None, long6=None, long7=None,
        int_x=INT32_MAX, int_y=INT32_MAX, int_z=None,
    ),
    params=[
        ParamSpec(1, "First Item", defined=True),
        ParamSpec(2, "Last Item", defined=True),
        *[ParamSpec(slot, "Empty", defined=False, reject_fail_reason=_NEVER_READ.format(slot=slot))
          for slot in range(3, 8)],
    ],
)


async def probe(system, **overrides) -> int | None:
    """probe_dual() with SPEC's baseline plus overrides, reduced to one result (None = no ACK on a type)."""
    kw = dict(SPEC.baseline)
    kw.update(overrides)
    int_ack, long_ack = await probe_dual(system, _CMD_ID, window_s=_ACK_WINDOW_S, **kw)
    label = ", ".join(f"{k}={v}" for k, v in overrides.items()) or "baseline"
    return _reduce_dual(_CMD, label, int_ack, long_ack)


# ---------------------------------------------------------------------------
# Class fixtures — mission state
# ---------------------------------------------------------------------------

@pytest_asyncio.fixture(scope="class", loop_scope="class")
async def _no_mission_cls(gcs_system_cls, mock_stack_cls):
    """Clear every mission type, so the class runs with nothing loaded."""
    await clear_all_mission_types(gcs_system_cls)
    yield


@pytest_asyncio.fixture(scope="class", loop_scope="class")
async def _mission_loaded_cls(gcs_system_cls, mock_stack_cls, home_item_for_mission, request):
    """
    Upload build_mission() once for the class and, on a real stack, wait until
    the vehicle is armable — PX4 answers TEMPORARILY_REJECTED, not ACCEPTED,
    while it can't arm. Teardown: RTL and land (accepted probes have PX4 flying),
    restarting the stack if it doesn't land, then clear the mission.
    """
    system = gcs_system_cls
    real = mock_stack_cls is None
    if real:
        home = await _get_home_position(system)
        home_lat_lon = (home.latitude_deg, home.longitude_deg)
        bearing = await _get_heading(system)
    else:
        home_lat_lon, bearing = _MOCK_HOME, 0.0
    items, layout = build_mission(home_lat_lon, home_item_for_mission, bearing)
    await clear_all_mission_types(system)
    await ensure_mission_int_capability(system)  # else upload_mission() can fail INT_MESSAGES_NOT_SUPPORTED
    await system.mission_raw.upload_mission(items)
    if real:
        await _wait_armable(system)
    request.cls.MISSION = (items, layout)
    yield
    if real:
        await _rtl_and_land(system, request.getfixturevalue("restart_flight_stack"))
        try:
            await system.action.hold()  # PX4 isn't armable in Mission mode once the mission is cleared
        except Exception as exc:  # noqa: BLE001 — best effort
            log.warning("could not switch to Hold after the class: %r", exc)
    await clear_all_mission_types(system)


# ---------------------------------------------------------------------------
# No mission loaded
# ---------------------------------------------------------------------------

@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(300)
@pytest.mark.usefixtures("_no_mission_cls")
class TestMissionStartNoMission:
    """MISSION_START with no mission loaded."""

    SPEC = SPEC  # read by conftest's _record()/_write_tier1_log

    async def test_mission_start_no_mission_rejected(self, gcs_system_cls, mock_stack_cls, request):
        """
        Not accepted when no mission is loaded.

        param1 names "the first mission item to run"; with no mission there is
        no such item, so accepting would claim a start that can't happen. Which
        NACK (DENIED, FAILED, ...) is undefined — the result is recorded, not
        asserted. Expected: PX4 DENIED; ArduCopter/Rover FAILED (AUTO refused);
        ArduPlane ACCEPTED (it switches to AUTO regardless).
        """
        result = await probe(gcs_system_cls)
        if result == MAV_RESULT_UNSUPPORTED:
            pytest.skip("NA: MISSION_START is UNSUPPORTED on this stack")
        _check(type(self), request, "Not accepted when no mission is loaded", result,
               expect=lambda r: r not in _ACCEPTED,
               fail_reason="MISSION_START accepted with no mission loaded — there is no first item to run")


# ---------------------------------------------------------------------------
# Mission loaded — mandatory common checks + per-param tests
# ---------------------------------------------------------------------------

@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(300)
@pytest.mark.usefixtures("_mission_loaded_cls")
class TestMissionStartCommand(Tier1CommandTestBase):
    """
    MISSION_START with build_mission() loaded. The mandatory common checks are
    inherited from Tier1CommandTestBase; the rest are MISSION_START's own.
    """

    SPEC = SPEC
    MISSION: tuple[list[MissionItem], dict]  # set by _mission_loaded_cls

    async def _probe_after_baseline(self, system, mock_stack, **overrides) -> int | None:
        """
        Root CLAUDE.md rule 4a: send the baseline first. If it isn't accepted, a
        NACK of the value under test says nothing about the param — NA.
        """
        await self._ensure_supported(system, mock_stack)
        base = await probe(system)
        if base not in _ACCEPTED:
            pytest.skip(f"NA: the baseline (param1=0, param2=0) wasn't accepted (result={base}), "
                        "so this value's ACK says nothing about the param")
        return await probe(system, **overrides)

    def _seq(self, name: str) -> int:
        return self.MISSION[1]["seq"][name]

    # -- valid values: record the ACK, pass either way (rule 4a) -------------

    async def test_mission_start_param1_item_index_ack(self, gcs_system_cls, mock_stack_cls, request):
        """
        param1 = a valid item other than the first — ACK recorded; accepted or NACKed both pass.

        A NACK is a legitimate "not supported" (rule 4a); acceptance only says
        it isn't NACKed — whether the mission really starts there is
        test_flight.py's test_mission_start_compat_first_item_honoured.
        """
        k = self._seq("B")
        result = await self._probe_after_baseline(gcs_system_cls, mock_stack_cls, param1=float(k))
        if result is not None:
            report.record_nonsentinel_ack("command", _CMD_NAME, "1_First Item", nacked=result not in _ACCEPTED)
        _record(type(self), request, "PASS", f"param1 = {k} (a valid item) — ACK recorded", result)

    async def test_mission_start_param2_item_index_ack(self, gcs_system_cls, mock_stack_cls, request):
        """
        param2 = a valid item before the last — ACK recorded; accepted or NACKed both pass.

        Whether the mission really ends there is test_flight.py's
        test_mission_start_compat_last_item_honoured.
        """
        j = self._seq("B")
        result = await self._probe_after_baseline(gcs_system_cls, mock_stack_cls, param2=float(j))
        if result is not None:
            report.record_nonsentinel_ack("command", _CMD_NAME, "2_Last Item", nacked=result not in _ACCEPTED)
        _record(type(self), request, "PASS", f"param2 = {j} (a valid item) — ACK recorded", result)

    # -- out of range: the XML gives minValue=0, and an item must exist -------

    async def test_mission_start_param1_below_min_rejected(self, gcs_system_cls, mock_stack_cls, request):
        """
        Not accepted for param1 = -1 (below the XML's minValue=0).

        Root CLAUDE.md rule 5: an explicit XML minimum gets a real assertion.
        PX4 accepts -1 as its own "start from the current item" (Navigator
        skips set_current_mission_index() for param1 < 0) — not a clamp to 0 —
        and tests/command/do_set_mission_current/test_flight.py relies on it.
        """
        result = await self._probe_after_baseline(gcs_system_cls, mock_stack_cls, param1=-1.0)
        _check(type(self), request, "Not accepted for param1 = -1 (below minValue=0)", result,
               expect=lambda r: r not in _ACCEPTED,
               fail_reason="param1 = -1 accepted although the XML's minValue is 0 "
                           "(PX4 reads -1 as 'start from the current item')")

    async def test_mission_start_param2_below_min_rejected(self, gcs_system_cls, mock_stack_cls, request):
        """Not accepted for param2 = -1 (below the XML's minValue=0)."""
        result = await self._probe_after_baseline(gcs_system_cls, mock_stack_cls, param2=-1.0)
        _check(type(self), request, "Not accepted for param2 = -1 (below minValue=0)", result,
               expect=lambda r: r not in _ACCEPTED,
               fail_reason="param2 = -1 accepted although the XML's minValue is 0 (PX4 never reads param2)")

    async def test_mission_start_param1_out_of_range_rejected(self, gcs_system_cls, mock_stack_cls, request):
        """
        Not accepted for param1 = an item past the end of the mission.

        The XML defines param1 as a mission item to run; there is no such item.
        """
        k = len(self.MISSION[0]) + 5
        result = await self._probe_after_baseline(gcs_system_cls, mock_stack_cls, param1=float(k))
        _check(type(self), request, f"Not accepted for param1 = {k} (past the last item)", result,
               expect=lambda r: r not in _ACCEPTED,
               fail_reason=f"param1 = {k} accepted, but the mission has only {len(self.MISSION[0])} items")

    # -- characterisation: the XML says nothing (rule 3) ----------------------

    async def test_mission_start_param2_out_of_range(self, gcs_system_cls, mock_stack_cls, request):
        """
        Characterisation: param2 = an item past the end of the mission.

        Arguably harmless (the mission ends at its real end first), arguably
        invalid — the XML doesn't say. Result recorded, not asserted.
        """
        j = len(self.MISSION[0]) + 5
        result = await self._probe_after_baseline(gcs_system_cls, mock_stack_cls, param2=float(j))
        _record(type(self), request, "PASS", f"Observational: param2 = {j} (past the last item)", result)

    async def test_mission_start_param1_non_integer(self, gcs_system_cls, mock_stack_cls, request):
        """
        Characterisation: param1 = 1.5 (the XML gives increment=1; a fractional item index is meaningless).

        Result recorded, not asserted. PX4 accepts and truncates; ArduPilot DENIES (non-zero).
        """
        result = await self._probe_after_baseline(gcs_system_cls, mock_stack_cls, param1=1.5)
        _record(type(self), request, "PASS", "Observational: param1 = 1.5 (non-integer item index)", result)

    async def test_mission_start_param2_before_param1(self, gcs_system_cls, mock_stack_cls, request):
        """
        Characterisation: param2 < param1 (last item before the first) — the XML doesn't say. Result recorded.
        """
        p1, p2 = self._seq("C"), self._seq("A")
        result = await self._probe_after_baseline(gcs_system_cls, mock_stack_cls, param1=float(p1), param2=float(p2))
        _record(type(self), request, "PASS", f"Observational: param1 = {p1}, param2 = {p2} (last before first)", result)
