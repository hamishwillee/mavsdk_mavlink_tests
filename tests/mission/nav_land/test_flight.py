"""
MAV_CMD_NAV_LAND (cmd=21) — Tier 2 execution tests (mission item).

"Land at location."

Uploads [takeoff, (transit waypoint), NAV_LAND], flies it, and checks each
parameter against telemetry. Requires a real flight stack (--drone-address);
every test is skipped in paired/mock mode. The flight routine and verdict
helpers are shared with the other three landing modules — tests/land_helpers.py
(read its docstring for the conventions).

- Test names are `test_land_<compat|obs>_<what>` (root CLAUDE.md rule 8).
- Latitude/Longitude and Yaw share two flights (rule 7): due N of the start with yaw
  135°, due E with 315° (rule 4c). Judged at touchdown (rule 4b).
- A VTOL (HEARTBEAT type) takes off with NAV_VTOL_TAKEOFF and reaches the landing
  item in forward flight; anything else takes off with NAV_TAKEOFF.

Expected from source (blind read, 2026-10-13 — Tier 2 pattern #5)
-----------------------------------------------------------------
PX4 v1.18.0-beta1-660 (`mission_base.cpp::handleLanding`, `mission_block.cpp`):
- a multicopter not already over the landing point first flies to it at its
  current altitude ("Ignoring waypoint altitude"), then descends — so Lat/Lon is
  honoured and the approach is a dogleg, not a glide slope.
- "ignore yaw for landing items": `_mission_item.yaw = NAN` — Yaw is stored
  (Tier 1) but never used. Expect a compatibility error.
- param7 isn't used for the descent; it ends when the land detector trips.
- a VTOL in fixed-wing flight has NAV_LAND replaced by NAV_VTOL_LAND
  (`force_vtol()`, NAV_FORCE_VT): flies to the point, back-transitions, lands.

Running
-------
PX4 SIH multicopter (the px4io/px4-sitl container build — see README.md)::

    pytest tests/mission/nav_land/test_flight.py --drone-address=udp://:14540 \\
        --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx \\
        --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO

PX4 SIH VTOL: the same with `--px4-model=sihsim_standard_vtol --vehicle-type=vtol`.
"""

import logging

import pytest

from tests import land_helpers as lh
from tests import report
from .test_protocol import SPEC as _SPEC
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

_CMD_NAME = "NAV_LAND"
_CMD_ID = 21

report.declare_command("mission", _CMD_NAME, _CMD_ID)
report.declare_params("mission", _CMD_NAME, [f"{p.slot}_{p.label}" for p in _SPEC.params])

_pair: list[dict] | None = None  # cached shared flights (rule 7)


async def _flights(gcs_system, home_item, restart) -> list[dict]:
    global _pair
    if _pair is None:
        async def _fly(**kw):
            return await lh.fly_mission(gcs_system, home_item, restart, _CMD_ID, **kw)
        _pair = await lh.fly_pair(_fly, gcs_system)
    return _pair


async def test_land_compat_lands(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """The XML's one requirement: the vehicle lands. PASS if every accepted NAV_LAND mission ends in a touchdown."""
    flights = await _flights(gcs_system, home_item_for_mission, restart_flight_stack)
    ok, detail = lh.lands_verdict(flights)
    record_tier2_detail(request, f"PASS if the vehicle touches down: {detail}")
    if ok is None:
        if lh.refused_at_start(flights):
            # The item round-trips (Tier 1) but the stack won't fly a mission with it:
            # not supported on this vehicle, whatever the upload said.
            record_compat_command_supported(request, False, notes=lh.START_REFUSED_NOTE)
        pytest.skip(f"NA: every NAV_LAND mission was refused — {detail}")
    record_compat_command_supported(request, ok, notes=None if ok else "Accepted, but the vehicle does not land")
    if not ok:
        report.compat_fail(f"NAV_LAND accepted but the vehicle did not touch down: {detail}")


async def test_land_compat_position_honoured(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """params 5/6 = points due N and due E of the start — "is lat/lon honoured": NACKed, or the vehicle touches down there."""
    v = lh.position_verdict(await _flights(gcs_system, home_item_for_mission, restart_flight_stack))
    lh.record_param(request, "param5/6 (Lat/Lon)", ("5_Latitude", "6_Longitude"), v,
                    f"PASS if each target is NACKed or the touchdown is within {lh.TOUCHDOWN_TOLERANCE_M:.0f} m of it")


async def test_land_compat_yaw_honoured(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """param4 = 135° and 315° — "is yaw honoured": NACKed, or the vehicle touches down on that heading."""
    v = lh.yaw_verdict(await _flights(gcs_system, home_item_for_mission, restart_flight_stack))
    lh.record_param(request, "param4 (Yaw Angle)", ("4_Yaw Angle",), v,
                    f"PASS if each yaw is NACKed or the heading at touchdown is within ±{lh.YAW_TOLERANCE_DEG:.0f}° of it")


async def test_land_obs_approach(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """Characterisation: how the landing is flown — where it starts, any back-transition, where it is at half altitude."""
    flights = await _flights(gcs_system, home_item_for_mission, restart_flight_stack)
    record_tier2_detail(request, "Observational — " + " | ".join(f"{r['label']}: {lh.describe_landing(r)}" for r in flights))


async def test_land_obs_ground_altitude(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    param7 = 15 m above home ("ground level in current frame") — characterisation: does the descent end there or on the ground?

    The XML calls param7 the ground level, not a destination, so on flat simulated
    terrain there is nothing to compare a touchdown against — the param is recorded
    as untestable (rule 10d) and this just records what a deliberately wrong ground
    level does.
    """
    report.record_compat_fact("mission", _CMD_NAME, "7_Altitude", untestable=lh.GROUND_ALT_UNTESTABLE)
    r = await lh.fly_mission(gcs_system, home_item_for_mission, restart_flight_stack, _CMD_ID,
                             groups={"altitude": {"z": 15.0}})
    record_tier2_detail(request, f"Observational: z = 15 m — {lh.describe_landing(r)}"
                        + ("" if r["nacked"] or not r["landed"] else f"; touchdown altitude {r['touchdown']['alt']:.1f} m"))


async def test_land_compat_untestable_params(gcs_system, home_item_for_mission, request):
    """
    params 1 (Abort Alt), 2 (Land Mode), 3 (Empty) and Yaw = NaN — no execution verdict possible; records why, without flying.

    Abort Alt only matters in an aborted landing and Land Mode only with a
    precision-landing beacon — neither is simulated. A value NACKed at upload is
    still a verdict (rule 4a), so each is probed.
    """
    for key, kw, why in (("1_Abort Alt", dict(param1=25.0), "Needs an aborted landing, which the harness can't trigger"),
                         ("2_Land Mode", dict(param2=2.0), "Needs a precision-landing beacon")):
        nacked = await lh.mission_probe_nacked(gcs_system, home_item_for_mission, _CMD_ID, **kw)
        if nacked:
            report.record_nonsentinel_ack("mission", _CMD_NAME, key, nacked=True)
        else:
            report.record_compat_fact("mission", _CMD_NAME, key, untestable=why)
    record_compat_json(request, "3_Empty", supported="not-applicable",
                       accept_nan_or_int32max=not await lh.mission_probe_nacked(gcs_system, home_item_for_mission, _CMD_ID, param3=lh.NAN),
                       nacks_on_non_sentinel_value=await lh.mission_probe_nacked(gcs_system, home_item_for_mission, _CMD_ID, param3=1.0))
    record_tier2_param_verdict(request, "Yaw sentinel (NaN)", lh.YAW_SENTINEL_NOT_TESTABLE)
    pytest.skip("NA: Abort Alt, Land Mode and the Yaw sentinel can't be judged from a simulated landing — see docstring")
