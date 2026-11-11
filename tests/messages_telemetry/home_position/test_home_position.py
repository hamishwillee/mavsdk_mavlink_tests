"""
HOME_POSITION (message id 242) — https://mavlink.io/en/messages/common.html#HOME_POSITION

What the XML defines (README.md in this directory has the reasoning and results):

- latitude/longitude (degE7, WGS84), altitude (mm, MSL).
- x/y/z (m): the same position "in the local coordinate frame (NED)". No
  invalid value, and no origin stated (the local frame's origin is
  GPS_GLOBAL_ORIGIN).
- q[4] (`invalid="[NaN]"`): the surface orientation at takeoff — heading and
  slope — "All fields should be set to NaN if an accurate quaternion for both
  heading and surface slope cannot be supplied".
- approach_x/y/z (m): the end of the approach vector. No invalid value.
- time_usec (extension): "UNIX Epoch time or time since system boot".
- The description says home is set automatically at takeoff, or by DO_SET_HOME.

So, with the vehicle sitting where it booted (home is where it is):
- latitude/longitude/altitude match GLOBAL_POSITION_INT (±5 m), and
  GLOBAL_POSITION_INT.relative_alt — defined as "altitude above home" — equals
  its alt minus this altitude (±0.5 m). Info: a mismatch says two messages
  disagree, not which is wrong.
- x/y/z equal home's position relative to GPS_GLOBAL_ORIGIN (±1 m). As with
  LOCAL_POSITION_NED, the axes are only shown right when home is away from the
  origin; otherwise they're recorded untestable.
- q and approach_* are characterised: both stacks may legitimately not supply
  them, and approach_* has no way to say so.
"""

from __future__ import annotations

import logging
import math

import pytest

from tests import report
from tests.messages_telemetry.conftest import (
    classify_time_usec,
    distinct,
    finite,
    ned_from_origin,
    nearest,
    record_detail,
    record_field_fact,
    skip_untestable,
)
from tests.messages_telemetry.emission import Emission

log = logging.getLogger(__name__)

_MSG_NAME = "HOME_POSITION"
_MSG_ID = 242
REQUIRED_MESSAGES = {
    "HOME_POSITION": "the message under test",
    "GLOBAL_POSITION_INT": "the vehicle position home is compared against",
}
_GLOBAL = ["latitude", "longitude", "altitude"]
_LOCAL = ["x", "y", "z"]
_APPROACH = ["approach_x", "approach_y", "approach_z"]
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME,
                      [*_GLOBAL, *_LOCAL, "q", *_APPROACH, "time_usec"])

pytestmark = pytest.mark.timeout(180)

EMISSION = Emission(_MSG_NAME, _MSG_ID, window_extra=("GLOBAL_POSITION_INT", "GPS_GLOBAL_ORIGIN"))
EMISSION.define_tests(globals(), "home_position")

HOME_DISTANCE_M = 5.0        # the vehicle hasn't moved since home was set
RELATIVE_ALT_TOLERANCE_M = 0.5
LOCAL_TOLERANCE_M = 1.0
NONTRIVIAL_M = 5.0
NORM_TOLERANCE = 1e-3


async def test_home_position_info_global_is_vehicle_position(gcs_system, request):
    """(Information) latitude/longitude/altitude match the stationary vehicle's position, and relative_alt agrees."""
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        skip_untestable(request, _GLOBAL, "not enough messages to analyse", w.na)
    gpi = w.others.get("GLOBAL_POSITION_INT") or []
    if not gpi:
        skip_untestable(request, _GLOBAL, "no GLOBAL_POSITION_INT to compare against")
    h = w.samples[-1].fields
    worst_h = worst_v = worst_rel = 0.0
    for g in gpi:
        n, e, d = ned_from_origin(h, g.fields["lat"], g.fields["lon"], g.fields["alt"])
        worst_h = max(worst_h, math.hypot(n, e))
        worst_v = max(worst_v, abs(d))
        worst_rel = max(worst_rel, abs(g.fields["relative_alt"] - (g.fields["alt"] - h["altitude"])) / 1000)
    ok = worst_h <= HOME_DISTANCE_M and worst_v <= HOME_DISTANCE_M and worst_rel <= RELATIVE_ALT_TOLERANCE_M
    for f in _GLOBAL:
        record_field_fact(request, f, supported=ok)
    detail = (f"(Information) home {h['latitude'] / 1e7:.7f}, {h['longitude'] / 1e7:.7f}, "
              f"{h['altitude'] / 1000:.2f} m; vehicle worst {worst_h:.2f} m horizontal, {worst_v:.2f} m vertical "
              f"from it; relative_alt vs alt−home worst {worst_rel:.3f} m (n={len(gpi)})")
    record_detail(request, detail)
    assert ok, f"home isn't where the stationary vehicle is — {detail}"


async def test_home_position_info_local_matches_origin(gcs_system, request):
    """(Information) x/y/z equal home's position relative to GPS_GLOBAL_ORIGIN (±1 m); characterise NaN/0."""
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        skip_untestable(request, _LOCAL, "not enough messages to analyse", w.na)
    origins = w.others.get("GPS_GLOBAL_ORIGIN") or []
    h = w.samples[-1].fields
    local = [h[f] for f in _LOCAL]
    shown = "/".join(f"{v:+.2f}" if finite(v) else str(v) for v in local)
    if not origins:
        skip_untestable(request, _LOCAL, "no GPS_GLOBAL_ORIGIN to compare against", f"x/y/z {shown}")
    expected = ned_from_origin(origins[-1].fields, h["latitude"], h["longitude"], h["altitude"])
    if not finite(*local):
        for f in _LOCAL:
            record_field_fact(request, f, supported=False, notes="NaN with a local origin present")
        record_detail(request, f"(Information) x/y/z {shown} although GPS_GLOBAL_ORIGIN is known")
        pytest.fail(f"x/y/z not finite ({shown}) with a known origin")
    diffs = [abs(a - b) for a, b in zip(local, expected)]
    ok = all(d <= LOCAL_TOLERANCE_M for d in diffs)
    for f, v, ex in zip(_LOCAL, local, expected):
        if not ok:
            record_field_fact(request, f, supported=False)
        elif abs(ex) >= NONTRIVIAL_M:
            record_field_fact(request, f, supported=True)
        else:
            record_field_fact(request, f, untestable=f"home {ex:+.1f} m from the origin: ~0 either way")
    detail = (f"(Information) x/y/z {shown} m; expected from GPS_GLOBAL_ORIGIN "
              f"{expected[0]:+.2f}/{expected[1]:+.2f}/{expected[2]:+.2f}; worst {max(diffs):.2f} m")
    record_detail(request, detail)
    assert ok, f"x/y/z isn't home relative to GPS_GLOBAL_ORIGIN — {detail}"


async def test_home_position_compat_q(gcs_system, request):
    """q is all NaN (not supplied) or a unit quaternion; characterise which, and its yaw vs the vehicle's heading."""
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        skip_untestable(request, ["q"], "not enough messages to analyse", w.na)
    qs = [s.fields["q"] for s in w.samples]
    # MAVSDK's fields_json carries NaN as null, so a NaN arrives as None.
    nan = [q for q in qs if all(c is None or (isinstance(c, float) and math.isnan(c)) for c in q)]
    unit = [q for q in qs if finite(*q) and abs(math.sqrt(sum(c * c for c in q)) - 1) <= NORM_TOLERANCE]
    bad = [q for q in qs if q not in nan and q not in unit]
    if bad:
        record_field_fact(request, "q", supported=False)
        record_detail(request, f"{len(bad)} values neither all-NaN nor unit, e.g. {bad[0]}")
        report.compat_fail(f"q neither all NaN (invalid) nor a unit quaternion: {bad[0]}")
    if nan and not unit:
        record_field_fact(request, "q", supported=False, notes="Always NaN (no heading/slope supplied)")
        record_detail(request, f"always all NaN — \"cannot be supplied\" ({len(qs)} msgs)")
        return
    w_, x, y, z = unit[-1]
    roll = math.degrees(math.atan2(2 * (w_ * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w_ * y - z * x)))))
    yaw = math.degrees(math.atan2(2 * (w_ * z + x * y), 1 - 2 * (y * y + z * z))) % 360
    gpi = w.others.get("GLOBAL_POSITION_INT") or []
    hdg = (f", vehicle heading {gpi[-1].fields['hdg'] / 100:.1f}°"
           if gpi and gpi[-1].fields.get("hdg", 65535) != 65535 else "")
    record_field_fact(request, "q", supported=True,
                      notes=None if not nan else "Switches between NaN and a quaternion")
    record_detail(request, f"unit quaternion: roll {roll:+.1f}°, pitch {pitch:+.1f}°, yaw {yaw:.1f}°{hdg} "
                           f"({len(unit)} unit, {len(nan)} NaN)")


async def test_home_position_obs_approach(gcs_system, request):
    """Characterise approach_x/y/z: there's no invalid value, so all-0 can't say "not set"."""
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        skip_untestable(request, _APPROACH, "not enough messages to analyse", w.na)
    values = [tuple(s.fields[f] for f in _APPROACH) for s in w.samples]
    if all(v == (0.0, 0.0, 0.0) for v in values):
        for f in _APPROACH:
            record_field_fact(request, f, untestable="always 0: no invalid value, so 'not set' looks like the origin")
        record_detail(request, f"always [0, 0, 0] ({len(values)} msgs)")
        return
    nonfinite = [v for v in values if not finite(*v)]
    for f in _APPROACH:
        record_field_fact(request, f, supported=not nonfinite, notes="NaN sent" if nonfinite else None)
    record_detail(request, f"values {distinct(values)}")


async def test_home_position_obs_time_usec(gcs_system, request):
    """Characterise time_usec: UNIX or boot time, and whether it's the send time or when home was set."""
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        skip_untestable(request, ["time_usec"], "not enough messages to analyse", w.na)
    gpi = w.others.get("GLOBAL_POSITION_INT") or []
    kind, plausible, desc = classify_time_usec(w.samples, boot_ms_at={g.host_mono: g.fields["time_boot_ms"]
                                                                       for g in gpi})
    if kind == "zero":
        record_field_fact(request, "time_usec", supported=False, notes="Always 0")
    else:
        record_field_fact(request, "time_usec", supported=plausible, notes=desc)
    record_detail(request, f"{kind}: {desc}")
    if not plausible and kind != "zero":
        report.compat_fail(f"time_usec is neither a plausible UNIX nor boot time — {desc}")
