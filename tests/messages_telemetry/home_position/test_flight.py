"""
HOME_POSITION in flight (README.md in this directory; the shared flight is in ../flight.py).

- The XML: home "is set automatically by the system during the takeoff". So
  through the flight home stays where the vehicle took off (±10 m of where it
  was when armed; fixed to ±0.5 m from takeoff to landing). Info: the XML also
  allows DO_SET_HOME, and doesn't say home can't move.
- x/y/z can only be shown right once home is away from the local origin. The
  flight lands away from both and arms once on the ground there: a stack that
  sets home at arming moves it, and x/y/z must then be home relative to
  GPS_GLOBAL_ORIGIN (±1 m). Whether home moves (at landing, or at that arming)
  is characterised.
"""

from __future__ import annotations

import math

import pytest

from tests import report
from tests.flight_helpers import require_real_stack  # noqa: F401 — autouse: flight tests need a real stack
from tests.messages_telemetry import flight as F
from tests.messages_telemetry.conftest import (
    finite, ned_from_origin, ned_offset_m, nearest, record_detail, record_field_fact, skip_untestable,
)

_MSG_NAME = "HOME_POSITION"
_MSG_ID = 242
_GLOBAL = ["latitude", "longitude", "altitude"]
_LOCAL = ["x", "y", "z"]
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME,
                      [*_GLOBAL, *_LOCAL, "q", "approach_x", "approach_y", "approach_z", "time_usec"])

pytestmark = F.PYTESTMARK

TAKEOFF_DISTANCE_M = 10.0
FIXED_M = 0.5
LOCAL_TOLERANCE_M = 1.0
NONTRIVIAL_M = 5.0


def _offset(a: dict, b: dict) -> tuple[float, float]:
    """(horizontal, vertical) metres between two HOME_POSITION/GLOBAL_POSITION_INT-shaped positions."""
    n, e = ned_offset_m(a["latitude"], a["longitude"], b["latitude"], b["longitude"])
    return math.hypot(n, e), abs(a["altitude"] - b["altitude"]) / 1000.0


def _as_home(gpi: dict) -> dict:
    return {"latitude": gpi["lat"], "longitude": gpi["lon"], "altitude": gpi["alt"]}


async def test_home_position_info_set_at_takeoff_in_flight(gcs_system, request, restart_flight_stack):
    """(Information) In flight home is where the vehicle took off (±10 m) and doesn't move (±0.5 m)."""
    fl = await F.get_flight(gcs_system, restart_flight_stack)
    t0, t1 = fl.need_flying(request, _GLOBAL)
    homes = fl.s(_MSG_NAME, t0, t1)
    gpi = fl.s("GLOBAL_POSITION_INT")
    if len(homes) < 3 or not gpi:
        skip_untestable(request, _GLOBAL, "too few HOME_POSITION in flight", f"{len(homes)} received")
    at_arm = _as_home(nearest(gpi, fl.t("armed")).fields)
    first = homes[0].fields
    moved = max(max(_offset(first, h.fields)) for h in homes)
    from_takeoff = _offset(at_arm, first)
    ok = moved <= FIXED_M and from_takeoff[0] <= TAKEOFF_DISTANCE_M and from_takeoff[1] <= TAKEOFF_DISTANCE_M
    for f in _GLOBAL:
        record_field_fact(request, f, supported=True if ok else False)
    detail = (f"(Information) {len(homes)} in flight: {from_takeoff[0]:.1f} m / {from_takeoff[1]:.1f} m (horiz/vert) "
              f"from the vehicle at arming, moved at most {moved:.2f} m; {fl.label}")
    record_detail(request, detail)
    assert ok, f"home isn't the takeoff position, fixed through the flight — {detail}"


async def test_home_position_info_local_after_moving_home(gcs_system, request, restart_flight_stack):
    """(Information) Once home is away from the origin (after landing elsewhere and arming), x/y/z are home relative to GPS_GLOBAL_ORIGIN (±1 m)."""
    fl = await F.get_flight(gcs_system, restart_flight_stack)
    fl.need_touchdown(request, _LOCAL) if fl.kind != "ground" else fl.need_moving(request, _LOCAL)
    # After the re-arm if there was one, else a few seconds after the landing disarm.
    t_after = fl.t("redisarmed") or ((fl.t("disarmed") or fl.t("end")) + 5.0)
    t_rearm = (fl.t("rearmed") - 1.0) if fl.t("rearmed") else None  # HEARTBEAT can lag the arming by ~1 s
    before = fl.s(_MSG_NAME, None, fl.t("airborne") or fl.t("armed"))
    after = fl.s(_MSG_NAME, t_after)
    origins = fl.s("GPS_GLOBAL_ORIGIN")
    if not before or not after or not origins:
        skip_untestable(request, _LOCAL, "too few HOME_POSITION/GPS_GLOBAL_ORIGIN around the landing",
                        f"{len(before)} before, {len(after)} after, {len(origins)} GPS_GLOBAL_ORIGIN")
    h0, h = before[-1].fields, after[-1].fields
    moved = _offset(h0, h)[0]
    landed = fl.s(_MSG_NAME, fl.t("touchdown") or fl.t("arrived"), t_rearm)
    before_rearm = bool(landed) and _offset(h0, landed[-1].fields)[0] > NONTRIVIAL_M
    when = ("moved after landing, before any re-arm" if before_rearm else
            "not re-armed" if t_rearm is None else
            "moved at re-arming" if moved > NONTRIVIAL_M else "didn't move at re-arming")
    expected = ned_from_origin(nearest(origins, after[-1].host_mono).fields, h["latitude"], h["longitude"], h["altitude"])
    local = [h[f] for f in _LOCAL]
    shown = "/".join(f"{v:+.2f}" if finite(v) else str(v) for v in local)
    detail = (f"(Information) home moved {moved:.0f} m ({when}); x/y/z {shown} m, expected from GPS_GLOBAL_ORIGIN "
              f"{expected[0]:+.2f}/{expected[1]:+.2f}/{expected[2]:+.2f}; {fl.label}")
    record_detail(request, detail)
    if max(abs(e) for e in expected[:2]) < NONTRIVIAL_M:
        for f in _LOCAL:
            record_field_fact(request, f, untestable="home stayed at the origin: not reset by landing elsewhere or arming there")
        pytest.skip(f"NA: home stayed within {NONTRIVIAL_M:.0f} m of the origin — {detail}")
    if not finite(*local):
        for f in _LOCAL:
            record_field_fact(request, f, supported=False, notes="NaN with a local origin present")
        pytest.fail(f"x/y/z not finite with a known origin — {detail}")
    ok = all(abs(a - b) <= LOCAL_TOLERANCE_M for a, b in zip(local, expected))
    for f, ex in zip(_LOCAL, expected):
        if not ok:
            record_field_fact(request, f, supported=False)
        elif abs(ex) >= NONTRIVIAL_M:
            record_field_fact(request, f, supported=True)
        else:
            record_field_fact(request, f, untestable=f"home {ex:+.1f} m from the origin on this axis: ~0 either way")
    assert ok, f"x/y/z isn't home relative to GPS_GLOBAL_ORIGIN — {detail}"
