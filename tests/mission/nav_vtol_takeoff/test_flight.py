"""
MAV_CMD_NAV_VTOL_TAKEOFF (cmd=84) — Tier 2 execution tests (mission item).

"Takeoff from ground using VTOL mode, and transition to forward flight with
specified heading. The command should be ignored by vehicles that dont support
both VTOL and fixed-wing flight (multicopters, boats, etc.)."

Uploads a mission starting with a NAV_VTOL_TAKEOFF item, flies it, and checks
each parameter against telemetry. Requires a real flight stack
(--drone-address); every test is skipped in paired/mock mode.

Same conventions as tests/mission/nav_takeoff/test_flight.py (read that
module's docstring for the full outcome vocabulary):
- Test names are `test_vtol_takeoff_<compat|info|obs>_<what>` (root CLAUDE.md
  rule 8). `compat` tests are each param's one definitive "is it honoured"
  check and feed the report's Compatibility summary; `obs` tests characterise
  and assert nothing (rule 3).
- Every "is it honoured" check flies two values and combines them into one
  verdict (rule 4c): PASS if every value was NACKed (a legitimate way not to
  support it, rule 4a) or honoured; FAIL (compatibility error) if any value was
  accepted but not honoured (rule 4). Transition Heading is an enum, so it is
  tested per value instead.
- Values are judged where the vehicle ends up (rule 4b), at the discrete event
  that reflects the param (pattern #10): altitude when the takeoff item
  completes; position as the closest approach while the item is current;
  heading when the transition to forward flight starts.
- Nothing is gated on --vehicle-type (rule 5): whether the vehicle is a VTOL at
  all is read from its own HEARTBEAT type (MAV_TYPE_VTOL_*), not vtol_state —
  ArduCopter reports vtol_state MC.
- Flights shared by several tests are flown once and cached (rule 7).

Expected from source (blind read, 2026-10-01 — Tier 2 pattern #5)
-----------------------------------------------------------------
PX4 main (`mission.cpp`, `mission_block.cpp`):
- climbs in hover to the item altitude, turns to face the item's lat/lon
  (bearing from the current position — param4 is overwritten with NaN, so Yaw
  is ignored), transitions, then flies to the item's lat/lon as a fixed-wing
  waypoint at the item altitude. If the lat/lon is within the acceptance radius
  it transitions in place.
- so: Altitude honoured, Lat/Lon honoured, Yaw ignored. param2 isn't stored at
  all; `mavlink_command_params.hpp` (`{ 84, 0x78, 0x7C }`) may NACK any
  non-zero param2 at upload, which would make every Transition Heading value
  except VEHICLE_DEFAULT a legitimate REJECTED.
ArduPlane QuadPlane (`quadplane.cpp::do_vtol_takeoff`):
- "we always use the current location in XY for takeoff" — lat/lon zeroed on
  purpose; altitude is relative to the current height; params 1-4 aren't
  stored (`AP_Mission.cpp`). The item completes in VTOL mode at altitude; the
  transition only happens on the following fixed-wing item.
- so: Altitude honoured, Lat/Lon ignored (compatibility error), Yaw/Transition
  Heading accepted but dropped (compatibility error), and no transition as
  part of the command itself.

Running
-------
PX4 VTOL (Gazebo — root CLAUDE.md: SIH VTOL never leaves the ground)::

    pytest tests/mission/nav_vtol_takeoff/test_flight.py --drone-address=udp://:14540 \\
        --px4-sitl=~/github/px4/PX4-Autopilot --px4-model=gz_standard_vtol \\
        --vehicle-type=vtol --autopilot=px4 -v --log-cli-level=INFO

ArduPlane QuadPlane::

    pytest tests/mission/nav_vtol_takeoff/test_flight.py \\
        --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \\
        --ardupilot-sitl=~/github/ardupilot/ardupilot/build/sitl/bin/arduplane \\
        --ardupilot-model=quadplane --vehicle-type=quadplane --autopilot=ardupilot \\
        -v --log-cli-level=INFO
"""

import asyncio
import json
import logging
import math
import os

import pytest
from mavsdk.plugins.mission_raw import MissionItem, MissionRawError
from mavsdk.plugins.telemetry import VtolState

from tests import report
from ..conftest import clear_all_mission_types
from .test_protocol import SPEC as _VTOL_TAKEOFF_SPEC, TRANSITION_HEADINGS, HEADING_SPECIFIED
from tests.flight_helpers import (
    AIRBORNE_THRESHOLD_M,
    _dist_m,
    _get_heading,
    _get_home_position,
    _offset_lat_lon,
    mission_landing_items,
    request_message_rate,
    _rtl_and_land,
    vehicle_is_vtol,
    _tier2_auto_record,  # noqa: F401 — autouse: records every test's outcome for the Tier 2 log
    _wait_armable,
    record_compat_command_supported,
    record_compat_json,
    record_tier2_detail,
    record_tier2_param_verdict,
    require_real_stack,  # noqa: F401 — registers the real-stack skip gate for this module
)

log = logging.getLogger(__name__)

# A test that triggers a cached two-value probe flies two missions, each up to
# ITEM_TIMEOUT_S + TRANSITION_GRACE_S plus arm/cleanup (restart fallback).
pytestmark = pytest.mark.timeout(1200)

_CMD_NAME = "NAV_VTOL_TAKEOFF"
_CMD_ID = 84

report.declare_command("mission", _CMD_NAME, _CMD_ID)
report.declare_params("mission", _CMD_NAME, [f"{p.slot}_{p.label}" for p in _VTOL_TAKEOFF_SPEC.params])

NAN = float("nan")
INT32_MAX = 0x7FFF_FFFF
TRANSFER_TIMEOUT_S = 30.0

# Altitude (param7): two values (rule 4c), relative to home, judged at the moment
# the takeoff item completes. Kept away from PX4's VTO_LOITER_ALT default (80 m).
ALTITUDE_VALUES_M = (30.0, 50.0)
ALT_TOLERANCE_MIN_M = 5.0
ALT_TOLERANCE_FRAC = 0.2
TAKEOFF_ALT_M = 30.0  # altitude for every flight that isn't measuring altitude

# Lat/Lon (params 5/6): two targets 90° apart (rule 4c), far enough that a VTOL
# has finished transitioning before it gets there. Not yet tuned from real
# telemetry (pattern #2) — a fixed-wing accepts a waypoint at its acceptance
# radius, so the starting tolerance is generous; override per stack if needed.
POSITION_OFFSET_M = 400.0
POSITION_BEARINGS = (("N", 0.0), ("E", 90.0))
POSITION_TOLERANCE_M = float(os.environ.get("NAV_VTOL_TAKEOFF_POSITION_TOLERANCE_M", "50.0"))
_LANDING_AWAY_BEARING_DEG = 225.0  # south-west — away from both targets

# Headings: Yaw Angle (param4, with param2 = SPECIFIED) two values; Transition
# Heading NEXT_WAYPOINT in two directions; TAKEOFF once (its target is the
# vehicle's own ground heading). Judged at the start of the transition.
YAW_VALUES_DEG = (135.0, 225.0)
NEXT_WP_OFFSET_M = 500.0
NEXT_WP_RELATIVE_BEARINGS_DEG = (90.0, -90.0)  # relative to the ground heading
HEADING_TOLERANCE_DEG = float(os.environ.get("NAV_VTOL_TAKEOFF_HEADING_TOLERANCE_DEG", "25.0"))

# Per flight: how long the takeoff item may stay current, then how long to keep
# watching for a transition that a stack makes on the following item instead.
ITEM_TIMEOUT_S = float(os.environ.get("NAV_VTOL_TAKEOFF_ITEM_TIMEOUT_S", "180.0"))
TRANSITION_GRACE_S = 60.0

_FW_STATES = (VtolState.TRANSITION_TO_FW, VtolState.FW)

# How long a fresh (re)boot may take to send its first GLOBAL_POSITION_INT (heading and
# position come from it). Gazebo VTOL took over 30 s after HOME_POSITION (2026-10-01).
POSITION_READY_TIMEOUT_S = float(os.environ.get("NAV_VTOL_TAKEOFF_POSITION_READY_TIMEOUT_S", "120.0"))

# Cached flights (rule 7) — each flown at most once per session.
_altitude_result: dict | None = None
_position_result: dict | None = None
_specified_result: dict | None = None


# ---------------------------------------------------------------------------
# Mission building
# ---------------------------------------------------------------------------

def _build_mission(home_item, *probes, landing_bearing_deg: float | None = None, home_ref=None):
    """
    Items for one flight: the home slot (ArduPilot), the probes (first one current),
    then an approach waypoint + NAV_LAND so stacks that require a landing will start
    it (tests/mission/CLAUDE.md § "Mission shape a stack will fly"). The landing is
    anchored on home along `landing_bearing_deg` when given, so the way there can't
    pass over a position target by chance.
    """
    items = []
    if home_item is not None:
        items.append(MissionItem(
            seq=0, frame=home_item.frame, command=home_item.command, current=0, autocontinue=1,
            param1=home_item.param1, param2=home_item.param2, param3=home_item.param3, param4=home_item.param4,
            x=home_item.x, y=home_item.y, z=home_item.z, mission_type=0,
        ))
    for i, p in enumerate(probes):
        items.append(MissionItem(
            seq=len(items), frame=p.frame, command=p.command, current=(1 if i == 0 else 0),
            autocontinue=p.autocontinue, param1=p.param1, param2=p.param2, param3=p.param3,
            param4=p.param4, x=p.x, y=p.y, z=p.z, mission_type=p.mission_type,
        ))
    if landing_bearing_deg is not None:
        items.extend(mission_landing_items(items, anchor_lat_lon_int=home_ref, bearing_deg=landing_bearing_deg))
    else:
        items.extend(mission_landing_items(items, home_ref))
    return items


def _home_int(home) -> tuple[int, int]:
    return int(home.latitude_deg * 1e7), int(home.longitude_deg * 1e7)


def _vtol_item(home, **overrides) -> MissionItem:
    """NAV_VTOL_TAKEOFF at home, TAKEOFF_ALT_M relative, VEHICLE_DEFAULT heading, Yaw NaN."""
    x, y = _home_int(home)
    kw = dict(seq=0, frame=6, command=_CMD_ID, current=1, autocontinue=1,
              param1=0.0, param2=0.0, param3=0.0, param4=NAN,
              x=x, y=y, z=TAKEOFF_ALT_M, mission_type=0)
    kw.update(overrides)
    return MissionItem(**kw)


def _waypoint(lat_deg: float, lon_deg: float, alt_m: float) -> MissionItem:
    return MissionItem(seq=0, frame=6, command=16, current=0, autocontinue=1,
                       param1=0.0, param2=0.0, param3=0.0, param4=NAN,
                       x=int(lat_deg * 1e7), y=int(lon_deg * 1e7), z=alt_m, mission_type=0)


def _fmt_m(v) -> str:
    return "-" if v is None else f"{v:.0f} m"


def _angle_diff(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


# ---------------------------------------------------------------------------
# One flight, every measurement
# ---------------------------------------------------------------------------

async def _fly(gcs_system, home_item, restart_flight_stack, *, takeoff: MissionItem, after=(),
               target: tuple[float, float] | None = None, landing_bearing_deg: float | None = None) -> dict:
    """
    Upload [takeoff, *after, landing], fly it, and return everything a test may judge:

      nacked/reason    upload rejected, or start_mission() refused (no flight) — a refusal to
                       start is the stack declining the command, same as a NACK (rule 4a)
      max_alt          highest relative altitude seen (airborne check)
      alt_at_complete  relative altitude when MISSION_CURRENT moved past the takeoff item
      completed        whether the takeoff item completed within ITEM_TIMEOUT_S
      closest          closest approach to `target` while the takeoff item was current
      transition       {heading, alt, during_item} at the first TRANSITION_TO_FW/FW state, or None
      vtol_states      every vtol_state name seen
      ground_heading   heading on the ground before arming
      end_from_home    distance from home when the watch ended (before RTL)
    """
    home = await _get_home_position(gcs_system)
    # Long timeout: straight after a (re)start PX4 sends HOME_POSITION before any
    # GLOBAL_POSITION_INT, which heading comes from (the message watcher caught it, 2026-10-01).
    ground_heading = await _get_heading(gcs_system, timeout_s=POSITION_READY_TIMEOUT_S)
    items = _build_mission(home_item, takeoff, *after,
                           landing_bearing_deg=landing_bearing_deg, home_ref=_home_int(home))
    takeoff_seq = 1 if home_item is not None else 0
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
    except MissionRawError as exc:
        await clear_all_mission_types(gcs_system)
        reason = str(exc).split(":")[0].strip()
        log.info("VTOL takeoff flight: upload NACKed (%s)", reason)
        return {"nacked": True, "reason": reason, "ground_heading": ground_heading}

    state = {"seq": None, "heading": ground_heading, "alt": 0.0, "max_alt": 0.0,
             "closest": math.inf, "alt_at_complete": None, "transition": None, "from_home": None}
    vtol_states: set[str] = set()
    item_done = asyncio.Event()
    transitioned = asyncio.Event()
    in_fw = asyncio.Event()  # vtol_state FW seen — the transition finished

    async def _positions() -> None:
        async for pos in gcs_system.telemetry.position():
            state["alt"] = pos.relative_altitude_m
            state["max_alt"] = max(state["max_alt"], pos.relative_altitude_m)
            state["from_home"] = _dist_m(pos.latitude_deg, pos.longitude_deg, home.latitude_deg, home.longitude_deg)
            if target is not None and state["seq"] == takeoff_seq:
                state["closest"] = min(state["closest"], _dist_m(pos.latitude_deg, pos.longitude_deg, *target))

    async def _headings() -> None:
        async for hdg in gcs_system.telemetry.heading():
            state["heading"] = hdg.heading_deg

    async def _vtol() -> None:
        async for vs in gcs_system.telemetry.vtol_state():
            vtol_states.add(vs.name)
            if vs == VtolState.FW:
                in_fw.set()
            if vs in _FW_STATES and state["transition"] is None and state["max_alt"] >= AIRBORNE_THRESHOLD_M:
                state["transition"] = {"heading": state["heading"], "alt": state["alt"],
                                       "during_item": state["seq"] == takeoff_seq and not item_done.is_set()}
                transitioned.set()

    async def _mission_current() -> None:
        async for msg in gcs_system.mavlink_direct.message("MISSION_CURRENT"):
            seq = int(json.loads(msg.fields_json)["seq"])
            if state["seq"] == takeoff_seq and seq > takeoff_seq and not item_done.is_set():
                state["alt_at_complete"] = state["alt"]
                item_done.set()
            state["seq"] = seq

    tasks = [asyncio.create_task(f()) for f in (_positions, _headings, _vtol, _mission_current)]
    refused = None
    try:
        await request_message_rate(gcs_system, 42, 10.0)  # MISSION_CURRENT
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        try:
            await gcs_system.mission_raw.start_mission()
        except MissionRawError as exc:
            # e.g. PX4 multicopter: upload accepted, then "Switching to Mission is
            # currently not available" (DENIED) — it won't run a VTOL takeoff.
            refused = f"start_mission refused ({exc.result.name})"
            return {"nacked": True, "reason": refused, "ground_heading": ground_heading,
                    "vtol_states": sorted(vtol_states)}
        try:
            await asyncio.wait_for(item_done.wait(), ITEM_TIMEOUT_S)
        except asyncio.TimeoutError:
            log.warning("VTOL takeoff flight: takeoff item still current after %.0fs", ITEM_TIMEOUT_S)
        if not transitioned.is_set():
            try:
                await asyncio.wait_for(transitioned.wait(), TRANSITION_GRACE_S)
            except asyncio.TimeoutError:
                pass
        if transitioned.is_set() and not in_fw.is_set():
            # A transition that has started may finish after the item completes (PX4
            # completes an overhead takeoff item as the transition begins); watch for it
            # before anything concludes the transition was aborted.
            try:
                await asyncio.wait_for(in_fw.wait(), TRANSITION_GRACE_S)
            except asyncio.TimeoutError:
                pass
    finally:
        for t in tasks:
            t.cancel()
        end_from_home = state["from_home"]
        await _rtl_and_land(gcs_system, restart_flight_stack)
        await clear_all_mission_types(gcs_system)
        # A transition that started but never reached forward flight (e.g. a PX4
        # quad-chute) can latch a failure that keeps the vehicle unarmable even after
        # it lands ("VTOL fixed-wing system failure detected", PX4 v1.17 Gazebo,
        # 2026-10-01: every later test timed out at _wait_armable). Restart.
        if "TRANSITION_TO_FW" in vtol_states and "FW" not in vtol_states:
            log.warning("Transition aborted during this flight — restarting the flight stack")
            restart_flight_stack()

    result = {
        "nacked": False, "reason": None, "ground_heading": ground_heading,
        "max_alt": state["max_alt"], "alt_at_complete": state["alt_at_complete"],
        "completed": item_done.is_set(), "closest": state["closest"] if target is not None else None,
        "transition": state["transition"], "vtol_states": sorted(vtol_states),
        "end_from_home": end_from_home,
    }
    log.info("VTOL takeoff flight: %s", result)
    return result




def _require_airborne(r: dict, what: str) -> None:
    """A verdict needs a flight: a vehicle that never left the ground proves nothing (rule 10)."""
    if not r["nacked"] and r["max_alt"] < AIRBORNE_THRESHOLD_M:
        pytest.fail(f"Vehicle never got airborne — {what} inconclusive ({r})")
    if not r["nacked"] and "TRANSITION_TO_FW" in r.get("vtol_states", []) and "FW" not in r["vtol_states"]:
        # e.g. PX4 Gazebo v1.17: the simulated airspeed sensor dropped out mid-transition
        # and PX4 quad-chuted ("Quad-chute triggered", 2026-10-01) — the vehicle then holds
        # wherever it is, which says nothing about the param under test.
        pytest.fail(f"Transition to forward flight started but never completed (aborted — e.g. a "
                    f"quad-chute; see the message watcher's STATUSTEXT) — {what} inconclusive ({r})")


# ---------------------------------------------------------------------------
# Verdicts (rule 4a): NACKed = REJECTED (PASS), honoured = SUPPORTED (PASS),
# accepted but not honoured = compatibility error (FAIL)
# ---------------------------------------------------------------------------

def _combine(results: list[dict]) -> dict:
    """One verdict from several values of the same param (rule 4c)."""
    return {
        "ok": all(r["ok"] for r in results),
        "nacked": all(r["nacked"] for r in results),
        "detail": "; ".join(r["detail"] for r in results),
    }


def _verdict_text(v: dict) -> str:
    if v["nacked"]:
        return "REJECTED (NACKed)"
    return "SUPPORTED" if v["ok"] else "NOT SUPPORTED — COMPATIBILITY ERROR"


def _record(request, label: str, keys: tuple[str, ...], v: dict, criterion: str) -> None:
    record_tier2_detail(request, f"{criterion}: {v['detail']}")
    record_tier2_param_verdict(request, label, _verdict_text(v))
    for key in keys:
        record_compat_json(request, key, supported=v["ok"],
                           nacks_on_non_sentinel_value=None if v["ok"] else v["nacked"])
    if not (v["ok"] or v["nacked"]):
        report.compat_fail(f"{label} accepted but not honoured: {v['detail']} (root CLAUDE.md rule 4)")


def _heading_value(r: dict, expected: float, label: str) -> dict:
    """Judge one flight's transition heading against `expected`."""
    if r["nacked"]:
        return {"nacked": True, "ok": False, "detail": f"{label}: REJECTED ({r['reason']})"}
    _require_airborne(r, f"{label} heading")
    tr = r["transition"]
    if tr is None:
        return {"nacked": False, "ok": False,
                "detail": f"{label}: no transition to forward flight seen (states {r['vtol_states']})"}
    diff = _angle_diff(tr["heading"], expected)
    where = "during the takeoff item" if tr["during_item"] else "after the takeoff item"
    return {"nacked": False, "ok": diff <= HEADING_TOLERANCE_DEG,
            "detail": f"{label}: expected {expected:.0f}°, transition heading {tr['heading']:.1f}° "
                      f"(diff {diff:.1f}°, {where})"}


# ---------------------------------------------------------------------------
# Cached probes
# ---------------------------------------------------------------------------

async def _check_altitude(gcs_system, home_item, restart_flight_stack) -> list[dict]:
    """Fly every ALTITUDE_VALUES_M once (shared with the takes-off-and-transitions test)."""
    global _altitude_result
    if _altitude_result is None:
        flights = []
        for alt in ALTITUDE_VALUES_M:
            home = await _get_home_position(gcs_system)
            r = await _fly(gcs_system, home_item, restart_flight_stack, takeoff=_vtol_item(home, z=alt))
            r["target_alt"] = alt
            flights.append(r)
        _altitude_result = {"flights": flights}
    return _altitude_result["flights"]


async def _check_position(gcs_system, home_item, restart_flight_stack) -> dict:
    global _position_result
    if _position_result is None:
        values = []
        for label, bearing in POSITION_BEARINGS:
            home = await _get_home_position(gcs_system)
            n = POSITION_OFFSET_M * math.cos(math.radians(bearing))
            e = POSITION_OFFSET_M * math.sin(math.radians(bearing))
            t_lat, t_lon = _offset_lat_lon(home.latitude_deg, home.longitude_deg, n, e)
            r = await _fly(gcs_system, home_item, restart_flight_stack,
                           takeoff=_vtol_item(home, x=int(t_lat * 1e7), y=int(t_lon * 1e7)),
                           target=(t_lat, t_lon), landing_bearing_deg=_LANDING_AWAY_BEARING_DEG)
            tag = f"{POSITION_OFFSET_M:.0f} m {label}"
            if r["nacked"]:
                values.append({"nacked": True, "ok": False, "detail": f"{tag}: REJECTED ({r['reason']})"})
                continue
            _require_airborne(r, "lat/lon")
            values.append({"nacked": False, "ok": r["closest"] <= POSITION_TOLERANCE_M,
                           "detail": f"{tag}: closest approach {r['closest']:.1f} m while the item was current"
                                     f"{'' if r['completed'] else ' (item never completed)'}"})
        _position_result = _combine(values)
    return _position_result


async def _check_specified(gcs_system, home_item, restart_flight_stack) -> dict:
    """param2 = SPECIFIED with each YAW_VALUES_DEG — param4's verdict, and SPECIFIED's for param2."""
    global _specified_result
    if _specified_result is None:
        values = []
        for yaw in YAW_VALUES_DEG:
            home = await _get_home_position(gcs_system)
            r = await _fly(gcs_system, home_item, restart_flight_stack,
                           takeoff=_vtol_item(home, param2=float(HEADING_SPECIFIED), param4=yaw))
            values.append(_heading_value(r, yaw, f"SPECIFIED, yaw {yaw:.0f}°"))
        _specified_result = _combine(values)
    return _specified_result


# ---------------------------------------------------------------------------
# Command level: takes off in VTOL mode and transitions to forward flight
# ---------------------------------------------------------------------------

async def test_vtol_takeoff_compat_takes_off_and_transitions(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    The XML's one requirement: the vehicle takes off in VTOL mode and transitions to forward flight.

    Read from the altitude test's flights (shared, rule 7). PASS if the vehicle got
    airborne and its transition to forward flight began while the NAV_VTOL_TAKEOFF
    item was still current. FAIL (compatibility error) if it took off but only
    transitioned on a later item, or never — the transition is part of this
    command, not of whatever comes next. NA for a vehicle that never reports a VTOL
    state (the XML says such vehicles should ignore the command); whether it took
    off anyway is recorded in the detail.
    """
    flights = await _check_altitude(gcs_system, home_item_for_mission, restart_flight_stack)
    if all(f["nacked"] for f in flights):
        vtol = await vehicle_is_vtol(gcs_system)
        reasons = sorted({f["reason"] for f in flights})
        record_tier2_detail(
            request,
            f"NA: the NAV_VTOL_TAKEOFF mission was refused ({', '.join(reasons)})"
            + ("" if vtol else " — and the vehicle isn't a VTOL (HEARTBEAT type), so the XML says it should "
                               "ignore the command: refusing it is consistent with that"),
        )
        pytest.skip(f"NA: the NAV_VTOL_TAKEOFF mission was refused ({', '.join(reasons)})")
    flown = [f for f in flights if not f["nacked"]]
    if not await vehicle_is_vtol(gcs_system):
        took_off = any(f["max_alt"] >= AIRBORNE_THRESHOLD_M for f in flown)
        record_tier2_detail(
            request,
            f"NA: vehicle's HEARTBEAT type isn't a VTOL type — not a VTOL, so the XML says "
            f"it should ignore this command; it {'took off anyway' if took_off else 'did not take off'}",
        )
        pytest.skip("NA: not a VTOL (HEARTBEAT type isn't MAV_TYPE_VTOL_*) — the XML says such vehicles should ignore the command")
    for f in flown:
        _require_airborne(f, "takeoff/transition")
    during = [f["transition"] is not None and f["transition"]["during_item"] for f in flown]
    after = [f["transition"] is not None and not f["transition"]["during_item"] for f in flown]
    detail = "; ".join(
        f"{f['target_alt']:.0f} m: max alt {f['max_alt']:.1f} m, transition "
        + ("none" if f["transition"] is None else
           f"at {f['transition']['alt']:.1f} m {'during' if f['transition']['during_item'] else 'after'} the takeoff item")
        for f in flown
    )
    record_tier2_detail(request, f"PASS if the vehicle takes off and begins its transition during the takeoff item: {detail}")
    if all(during):
        record_compat_command_supported(request, True)
        return
    note = ("Takes off, but transitions to forward flight only on the following item, not as part of NAV_VTOL_TAKEOFF"
            if all(after) else "Takes off, but no transition to forward flight as part of NAV_VTOL_TAKEOFF")
    record_compat_command_supported(request, True, notes=note)
    report.compat_fail(f"{note}: {detail}")


# ---------------------------------------------------------------------------
# param7 (Altitude)
# ---------------------------------------------------------------------------

async def test_vtol_takeoff_compat_altitude_honoured(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    param7 = 30 m and 50 m — "is altitude honoured": NACKed, or at that altitude when the takeoff item completes.

    Two values (rule 4c), each its own flight. Judged at the item's completion event
    (pattern #10) with a tolerance of max(5 m, 20%) — a stack that ignores param7
    and climbs to a default of its own can't match both.
    """
    flights = await _check_altitude(gcs_system, home_item_for_mission, restart_flight_stack)
    values = []
    for f in flights:
        alt = f["target_alt"]
        if f["nacked"]:
            values.append({"nacked": True, "ok": False, "detail": f"{alt:.0f} m: REJECTED ({f['reason']})"})
            continue
        _require_airborne(f, "altitude")
        tol = max(ALT_TOLERANCE_MIN_M, ALT_TOLERANCE_FRAC * alt)
        if f["alt_at_complete"] is None:
            values.append({"nacked": False, "ok": False,
                           "detail": f"{alt:.0f} m: takeoff item never completed (max alt {f['max_alt']:.1f} m)"})
            continue
        values.append({"nacked": False, "ok": abs(f["alt_at_complete"] - alt) <= tol,
                       "detail": f"{alt:.0f} m: {f['alt_at_complete']:.1f} m at item completion (±{tol:.0f} m)"})
    _record(request, "param7 (Altitude)", ("7_Altitude",), _combine(values),
            "PASS if each commanded altitude is NACKed or reached when the takeoff item completes")


def _altitude_meaning(per_value: list[tuple[float, float | None, float | None]]) -> str:
    """
    Classify what param7 is used as, from (commanded, transition_alt, final_alt) per
    value — "matches" meaning within the altitude test's tolerance for every value.
    """
    def _all_match(i: int) -> bool:
        return all(v[i] is not None and abs(v[i] - v[0]) <= max(ALT_TOLERANCE_MIN_M, ALT_TOLERANCE_FRAC * v[0])
                   for v in per_value)
    transition, final = _all_match(1), _all_match(2)
    if transition and final:
        return "BOTH — transition altitude and final altitude"
    if transition:
        return "TRANSITION ALTITUDE ONLY — the vehicle then goes to an altitude of its own"
    if final:
        return "FINAL ALTITUDE ONLY — the transition happens at some other altitude"
    return "NEITHER"


async def test_vtol_takeoff_obs_altitude_meaning(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    param7 — characterisation: is Altitude the transition altitude, the altitude the vehicle ends at, or both?

    The XML says only "Altitude". For the standalone command PX4 uses it as the
    transition altitude and then climbs to its own loiter altitude (see
    tests/command/nav_vtol_takeoff); this records the same question for the
    mission item, from the altitude test's flights (cached, rule 7): the altitude
    at the start of the transition (only counted when it happens during the
    takeoff item — otherwise it belongs to whatever item comes next) and the
    altitude when the item completes. Observational (rule 3) — convergent
    behaviour across stacks is evidence for a spec clarification.
    """
    flights = [f for f in await _check_altitude(gcs_system, home_item_for_mission, restart_flight_stack)
               if not f["nacked"]]
    if not flights:
        pytest.skip("NA: every altitude upload was NACKed — nothing flew")
    if not await vehicle_is_vtol(gcs_system):
        pytest.skip("NA: not a VTOL (HEARTBEAT type isn't MAV_TYPE_VTOL_*) — there is no transition to classify")
    per_value = [(f["target_alt"],
                  f["transition"]["alt"] if f["transition"] and f["transition"]["during_item"] else None,
                  f["alt_at_complete"]) for f in flights]
    values = "; ".join(
        f"{c:.0f} m: transition {_fmt_m(t) if t is not None else 'not during the item'}, at completion {_fmt_m(e)}"
        for c, t, e in per_value)
    record_tier2_detail(request, f"Observational — param7 used as: {_altitude_meaning(per_value)} ({values})")

# ---------------------------------------------------------------------------
# params 5/6 (Latitude/Longitude)
# ---------------------------------------------------------------------------

async def test_vtol_takeoff_compat_respects_position(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    params 5/6 = points 400 m north and 400 m east — "is lat/lon honoured": NACKed, or the vehicle goes there.

    Under the XML's hasLocation/isDestination convention lat/lon is a destination
    (rule 6). Judged as the closest approach to the target while the takeoff item is
    current, so the following items can't carry the vehicle over it by chance (the
    landing is placed south-west of home).
    """
    v = await _check_position(gcs_system, home_item_for_mission, restart_flight_stack)
    _record(request, "param5/6 (Lat/Lon)", ("5_Latitude", "6_Longitude"), v,
            f"PASS if each target is NACKed or reached within {POSITION_TOLERANCE_M:.0f} m while the takeoff item is current")


async def test_vtol_takeoff_obs_from_current_position(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    params 5/6 = INT32_MAX ("use current position") — characterisation: where is the vehicle when the item completes?

    The sentinel's meaning is spec-defined, but for a takeoff that ends in forward
    flight there's no fixed point to assert against (rule 5) — so this records how
    far from home the vehicle is when the item completes, and whether it flew.
    """
    home = await _get_home_position(gcs_system)
    r = await _fly(gcs_system, home_item_for_mission, restart_flight_stack,
                   takeoff=_vtol_item(home, x=INT32_MAX, y=INT32_MAX))
    record_compat_json(request, "5_Latitude", accept_nan_or_int32max=not r["nacked"])
    record_compat_json(request, "6_Longitude", accept_nan_or_int32max=not r["nacked"])
    if r["nacked"]:
        record_tier2_detail(request, f"Observational: INT32_MAX lat/lon NACKed ({r['reason']})")
        return
    tr = r["transition"]
    record_tier2_detail(
        request,
        f"Observational (no fixed target for the sentinel): max alt {r['max_alt']:.1f} m, "
        f"item {'completed' if r['completed'] else f'still current after {ITEM_TIMEOUT_S:.0f} s'}, "
        f"{_fmt_m(r['end_from_home'])} from home at the end, transition "
        + ("none" if tr is None else f"at {tr['alt']:.1f} m heading {tr['heading']:.0f}°"),
    )


# ---------------------------------------------------------------------------
# param4 (Yaw Angle) and param2 (Transition Heading)
# ---------------------------------------------------------------------------

async def test_vtol_takeoff_compat_yaw_honoured(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    param4 = 135° and 225° with param2 = SPECIFIED — "is yaw honoured": NACKed, or the transition starts on that heading.

    The XML ties param4 to the transition heading through VTOL_TRANSITION_HEADING_
    SPECIFIED ("Use the specified heading in parameter 4"), and the command's
    description says it transitions "with specified heading" — so the transition
    heading is what's judged. A stack that rejects param2 = SPECIFIED rejects this
    combination (REJECTED). test_vtol_takeoff_obs_yaw_without_specified_heading
    records what param4 does on its own.
    """
    if await vehicle_is_vtol(gcs_system) is False:
        pytest.skip("NA: not a VTOL (HEARTBEAT type isn't MAV_TYPE_VTOL_*) — no transition, so no transition heading to judge")
    v = await _check_specified(gcs_system, home_item_for_mission, restart_flight_stack)
    _record(request, "param4 (Yaw Angle)", ("4_Yaw Angle",), v,
            f"PASS if each yaw is NACKed or the transition starts within ±{HEADING_TOLERANCE_DEG:.0f}° of it")


async def test_vtol_takeoff_compat_transition_heading(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    param2 (Transition Heading) — each enum value with a checkable heading: NACKed, or the transition starts on it.

    - SPECIFIED: from test_vtol_takeoff_compat_yaw_honoured's flights (cached).
    - NEXT_WAYPOINT: a waypoint 500 m away at 90° either side of the ground heading
      (two directions, rule 4c) — the transition must face it.
    - TAKEOFF: a waypoint 90° off the ground heading — the transition must keep the
      ground heading, not turn toward it.
    VEHICLE_DEFAULT and ANY have no fixed target ("respect the vehicle's
    configuration", "any heading"), so they're checked only at Tier 1 (stored or
    NACKed). One verdict for the param: supported only if every checked value is.
    """
    if await vehicle_is_vtol(gcs_system) is False:
        pytest.skip("NA: not a VTOL (HEARTBEAT type isn't MAV_TYPE_VTOL_*) — no transition, so no transition heading to judge")
    values = [await _check_specified(gcs_system, home_item_for_mission, restart_flight_stack)]

    async def _with_next(param2: int, rel_bearing: float, label: str, expect_ground: bool) -> dict:
        home = await _get_home_position(gcs_system)
        ground = await _get_heading(gcs_system, timeout_s=POSITION_READY_TIMEOUT_S)
        bearing = (ground + rel_bearing) % 360
        n = NEXT_WP_OFFSET_M * math.cos(math.radians(bearing))
        e = NEXT_WP_OFFSET_M * math.sin(math.radians(bearing))
        wp_lat, wp_lon = _offset_lat_lon(home.latitude_deg, home.longitude_deg, n, e)
        r = await _fly(gcs_system, home_item_for_mission, restart_flight_stack,
                       takeoff=_vtol_item(home, param2=float(param2)),
                       after=(_waypoint(wp_lat, wp_lon, TAKEOFF_ALT_M),))
        expected = r["ground_heading"] if expect_ground else bearing
        return _heading_value(r, expected, label)

    for rel in NEXT_WP_RELATIVE_BEARINGS_DEG:
        values.append(await _with_next(1, rel, f"NEXT_WAYPOINT, waypoint {rel:+.0f}° off the ground heading", False))
    values.append(await _with_next(2, NEXT_WP_RELATIVE_BEARINGS_DEG[0], "TAKEOFF, waypoint +90° off the ground heading", True))

    _record(request, "param2 (Transition Heading)", ("2_Transition Heading",), _combine(values),
            f"PASS if each value is NACKed or the transition starts within ±{HEADING_TOLERANCE_DEG:.0f}° of its heading "
            f"(VEHICLE_DEFAULT/ANY have no fixed target)")


async def test_vtol_takeoff_obs_yaw_without_specified_heading(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    param4 = 135° with param2 = VEHICLE_DEFAULT — characterisation: does the yaw affect anything on its own?

    The XML gives param4 no meaning separate from the transition heading (only
    SPECIFIED says to use it), so this records the transition heading and asserts
    nothing (rule 3).
    """
    if await vehicle_is_vtol(gcs_system) is False:
        pytest.skip("NA: not a VTOL (HEARTBEAT type isn't MAV_TYPE_VTOL_*) — no transition, so no transition heading to judge")
    home = await _get_home_position(gcs_system)
    r = await _fly(gcs_system, home_item_for_mission, restart_flight_stack, takeoff=_vtol_item(home, param4=135.0))
    if r["nacked"]:
        record_tier2_detail(request, f"Observational: param4 = 135° with VEHICLE_DEFAULT NACKed ({r['reason']})")
        return
    tr = r["transition"]
    record_tier2_detail(
        request,
        "Observational (param4 alone has no defined effect): "
        + ("no transition seen" if tr is None else
           f"transition heading {tr['heading']:.1f}° (diff from 135°: {_angle_diff(tr['heading'], 135.0):.1f}°; "
           f"ground heading {r['ground_heading']:.0f}°)"),
    )


async def test_vtol_takeoff_compat_yaw_sentinel(gcs_system, home_item_for_mission, request):
    """
    param4 = NaN — NOT TESTABLE via the mission protocol.

    NaN means "use the current system yaw heading mode", a dynamic outcome with no
    fixed target; the only way to show it's live-processed is to change the heading
    first and re-send, which a stored mission item can't do. Tier 1 checks it's
    stored as NaN.
    """
    record_tier2_param_verdict(request, "Yaw sentinel (NaN)", "NOT TESTABLE (mission protocol can't re-send mid-flight)")
    pytest.skip("NA: NaN's effect has no fixed target, and a mission item can't be re-sent mid-flight — see docstring")


# ---------------------------------------------------------------------------
# params 1/3 (Empty)
# ---------------------------------------------------------------------------

async def _probe_outcome(gcs_system, home_item, **overrides) -> bool:
    """Upload/download/clear one VTOL takeoff item without flying; True if NACKed."""
    home = await _get_home_position(gcs_system)
    items = _build_mission(home_item, _vtol_item(home, **overrides), home_ref=_home_int(home))
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
        return False
    except MissionRawError:
        return True
    finally:
        await clear_all_mission_types(gcs_system)


async def test_vtol_takeoff_compat_empty_params(gcs_system, home_item_for_mission, request):
    """
    params 1 and 3 (Empty) — nothing to support; records the sentinel facts without flying.

    Two upload probes per param (does NaN upload; does a real value NACK), so this
    module's JSON is complete on its own — Tier 1 checks the same in more depth.
    """
    for slot, key in ((1, "1_Empty"), (3, "3_Empty")):
        nacked_nan = await _probe_outcome(gcs_system, home_item_for_mission, **{f"param{slot}": NAN})
        nacked_val = await _probe_outcome(gcs_system, home_item_for_mission, **{f"param{slot}": 1.0})
        record_compat_json(request, key, supported="not-applicable",
                           accept_nan_or_int32max=not nacked_nan, nacks_on_non_sentinel_value=nacked_val)
    pytest.skip("NA: params 1 and 3 are Empty — nothing to functionally support")
