"""
MAV_CMD_NAV_VTOL_LAND (cmd=85) — Tier 2 execution tests (mission item).

"Land using VTOL mode"

Uploads [takeoff, (transit waypoint), NAV_VTOL_LAND], flies it, and checks each
parameter against telemetry. Requires a real flight stack (--drone-address);
every test is skipped in paired/mock mode. The flight routine and verdict
helpers are shared with the other three landing modules — tests/land_helpers.py
(read its docstring for the conventions).

- Test names are `test_vtol_land_<compat|obs>_<what>` (root CLAUDE.md rule 8).
- Latitude/Longitude, Yaw and Approach Altitude share two flights (rule 7): due N
  of the start with yaw 135° and approach altitude 50 m, due E with 315° and 70 m
  (rule 4c). Position and yaw are judged at touchdown (rule 4b); the approach
  altitude as the highest altitude reached once the landing item is current —
  both values are above the altitude the landing starts from, so honouring
  either shows as a climb a plain descent can't produce.
- A VTOL (HEARTBEAT type) takes off with NAV_VTOL_TAKEOFF and reaches the landing
  item in forward flight — the case "land using VTOL mode" is about. Anything
  else takes off with NAV_TAKEOFF and lands from a hover.

Expected from source (blind read, 2026-10-13 — Tier 2 pattern #5)
-----------------------------------------------------------------
PX4 v1.18.0-beta1-660 (`mission_base.cpp::handleLanding`):
- in fixed-wing flight: flies to the item's lat/lon as a waypoint at the current
  altitude, back-transitions there, then descends — Lat/Lon honoured.
- Yaw: "ignore yaw for landing items" (`_mission_item.yaw = NAN`) — stored but
  never used. Expect a compatibility error.
- Approach Altitude: allowed by the mask, not stored (Tier 1) and not used — the
  approach is at whatever altitude the vehicle is already at. Compatibility error.
- Land Options: a non-default value is NACKed at upload (mission mask 0x7C).
- on a multicopter the item is flown like NAV_LAND.

Running
-------
PX4 SIH VTOL (the px4io/px4-sitl container build — see README.md)::

    pytest tests/mission/nav_vtol_land/test_flight.py --drone-address=udp://:14540 \\
        --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_standard_vtol \\
        --vehicle-type=vtol --autopilot=px4 -v --log-cli-level=INFO

PX4 SIH multicopter: the same with `--px4-model=sihsim_quadx --vehicle-type=quadcopter`.
"""

import logging

import pytest

from tests import land_helpers as lh
from tests import report
from .test_protocol import SPEC as _SPEC, LAND_OPTIONS
from tests.flight_helpers import (
    _tier2_auto_record,  # noqa: F401 — autouse: records every test's outcome for the Tier 2 log
    record_compat_command_supported,
    record_compat_json,
    record_tier2_detail,
    record_tier2_param_verdict,
    require_real_stack,  # noqa: F401 — registers the real-stack skip gate for this module
)

log = logging.getLogger(__name__)

# The shared pair is two flights, each up to TOUCHDOWN_TIMEOUT_S plus arm/cleanup.
pytestmark = pytest.mark.timeout(1200)

_CMD_NAME = "NAV_VTOL_LAND"
_CMD_ID = 85

report.declare_command("mission", _CMD_NAME, _CMD_ID)
report.declare_params("mission", _CMD_NAME, [f"{p.slot}_{p.label}" for p in _SPEC.params])

_pair: list[dict] | None = None  # cached shared flights (rule 7)


async def _flights(gcs_system, home_item, restart) -> list[dict]:
    global _pair
    if _pair is None:
        async def _fly(**kw):
            return await lh.fly_mission(gcs_system, home_item, restart, _CMD_ID, **kw)
        _pair = await lh.fly_pair(_fly, gcs_system,
                                  extra=tuple({"approach": {"param3": a}} for a in lh.APPROACH_ALT_VALUES_M))
        for r, alt in zip(_pair, lh.APPROACH_ALT_VALUES_M):
            r["approach_alt"] = alt
    return _pair


async def test_vtol_land_compat_lands_in_vtol_mode(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    The XML's one requirement: the vehicle lands, in VTOL (hover) mode.

    PASS if every accepted NAV_VTOL_LAND mission ends in a touchdown and — for a
    landing that started in forward flight — the vehicle is back in multicopter
    mode when it touches down. A vehicle that lands from a hover (not a VTOL, or
    never transitioned) can only show the first half; the detail says which.
    """
    flights = await _flights(gcs_system, home_item_for_mission, restart_flight_stack)
    ok, detail = lh.lands_verdict(flights)
    record_tier2_detail(request, f"PASS if the vehicle touches down in VTOL (hover) mode: {detail}")
    if ok is None:
        if lh.refused_at_start(flights):
            # The item round-trips (Tier 1) but the stack won't fly a mission with it:
            # not supported on this vehicle, whatever the upload said.
            record_compat_command_supported(request, False, notes=lh.START_REFUSED_NOTE)
        pytest.skip(f"NA: every NAV_VTOL_LAND mission was refused — {detail}")
    if not ok:
        record_compat_command_supported(request, False, notes="Accepted, but the vehicle does not land")
        report.compat_fail(f"NAV_VTOL_LAND accepted but the vehicle did not touch down: {detail}")
    in_fw = [r for r in flights if not r["nacked"] and r["landed"] and r["touchdown"]["vtol_state"] in ("FW", "TRANSITION_TO_FW")]
    if in_fw:
        record_compat_command_supported(request, True, notes="Lands, but not in VTOL (hover) mode")
        report.compat_fail(f"NAV_VTOL_LAND touched down while still in fixed-wing mode: {detail}")
    record_compat_command_supported(request, True)


async def test_vtol_land_compat_position_honoured(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """params 5/6 = points due N and due E of the start — "is lat/lon honoured": NACKed, or the vehicle touches down there."""
    v = lh.position_verdict(await _flights(gcs_system, home_item_for_mission, restart_flight_stack))
    lh.record_param(request, "param5/6 (Lat/Lon)", ("5_Latitude", "6_Longitude"), v,
                    f"PASS if each target is NACKed or the touchdown is within {lh.TOUCHDOWN_TOLERANCE_M:.0f} m of it")


async def test_vtol_land_compat_yaw_honoured(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """param4 = 135° and 315° — "is yaw honoured": NACKed, or the vehicle touches down on that heading."""
    v = lh.yaw_verdict(await _flights(gcs_system, home_item_for_mission, restart_flight_stack))
    lh.record_param(request, "param4 (Yaw)", ("4_Yaw",), v,
                    f"PASS if each yaw is NACKed or the heading at touchdown is within ±{lh.YAW_TOLERANCE_DEG:.0f}° of it")


async def test_vtol_land_compat_approach_altitude_honoured(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """param3 = 50 m and 70 m — "is the approach altitude honoured": NACKed, or the vehicle climbs to it before descending."""
    v = lh.approach_altitude_verdict(await _flights(gcs_system, home_item_for_mission, restart_flight_stack))
    lh.record_param(request, "param3 (Approach Altitude)", ("3_Approach Altitude",), v,
                    "PASS if each approach altitude is NACKed or is the highest altitude reached during the landing")


async def test_vtol_land_obs_approach(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """Characterisation: how the landing is flown — where it starts, where the back-transition happens, where it touches down."""
    flights = await _flights(gcs_system, home_item_for_mission, restart_flight_stack)
    record_tier2_detail(request, "Observational — " + " | ".join(f"{r['label']}: {lh.describe_landing(r)}" for r in flights))


async def test_vtol_land_obs_ground_altitude(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    param7 = 15 m above home ("ground level") — characterisation: does the descent end there or on the ground?

    param7 is the ground level, not a destination, so on flat simulated terrain
    there is nothing to compare a touchdown against — the param is recorded as
    untestable (rule 10d) and this just records what a deliberately wrong ground
    level does.
    """
    report.record_compat_fact("mission", _CMD_NAME, "7_Ground Altitude", untestable=lh.GROUND_ALT_UNTESTABLE)
    r = await lh.fly_mission(gcs_system, home_item_for_mission, restart_flight_stack, _CMD_ID,
                             groups={"altitude": {"z": 15.0}})
    record_tier2_detail(request, f"Observational: z = 15 m — {lh.describe_landing(r)}"
                        + ("" if r["nacked"] or not r["landed"] else f"; touchdown altitude {r['touchdown']['alt']:.1f} m"))


async def test_vtol_land_compat_untestable_params(gcs_system, home_item_for_mission, request):
    """
    params 1 (Land Options), 2 (Empty) and Yaw = NaN — no execution verdict possible; records why, without flying.

    Land Options: DEFAULT is "default autopilot landing behaviour", so there is no
    fixed approach shape to tell the other two values apart from. A value NACKed
    at upload is still a verdict (rule 4a), so both non-default values are probed.
    """
    nacked = [await lh.mission_probe_nacked(gcs_system, home_item_for_mission, _CMD_ID, param1=float(v))
              for v in LAND_OPTIONS if v != 0]
    if all(nacked):
        report.record_nonsentinel_ack("mission", _CMD_NAME, "1_Land Options", nacked=True)
    else:
        report.record_compat_fact("mission", _CMD_NAME, "1_Land Options",
                                  untestable="DEFAULT is autopilot-defined — no fixed approach shape to compare against")
    record_compat_json(request, "2_Empty", supported="not-applicable",
                       accept_nan_or_int32max=not await lh.mission_probe_nacked(gcs_system, home_item_for_mission, _CMD_ID, param2=lh.NAN),
                       nacks_on_non_sentinel_value=await lh.mission_probe_nacked(gcs_system, home_item_for_mission, _CMD_ID, param2=1.0))
    record_tier2_param_verdict(request, "Yaw sentinel (NaN)", lh.YAW_SENTINEL_NOT_TESTABLE)
    pytest.skip("NA: Land Options and the Yaw sentinel can't be judged from a landing — see docstring")
