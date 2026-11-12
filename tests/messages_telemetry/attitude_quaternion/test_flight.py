"""
ATTITUDE_QUATERNION in flight (README.md in this directory; the shared flight is in ../flight.py).

At rest the vehicle sits near one heading and level, which hides a conjugated
(sign-flipped) quaternion near 0/180° and leaves the body rates at ~0. On the
flight it faces several headings, banks and turns:
- q as ZYX Euler angles matches ATTITUDE (±1°) at every heading; recorded
  supported once the headings flown cover ≥ 90°;
- rollspeed/pitchspeed/yawspeed equal ATTITUDE's (±0.05 rad/s or 10%) on each
  axis, compared where |rate| ≥ 0.15 rad/s;
- repr_offset_q stays [0,0,0,0] or a unit quaternion; what a VTOL sends in
  forward flight (a tailsitter's display offset) is characterised.
"""

from __future__ import annotations

import math

from tests import report
from tests.flight_helpers import require_real_stack  # noqa: F401 — autouse: flight tests need a real stack
from tests.messages_telemetry import flight as F
from tests.messages_telemetry.attitude_quaternion.test_attitude_quaternion import _euler_deg, _finite, _norm, _q
from tests.messages_telemetry.conftest import record_detail, record_field_fact, skip_untestable

_MSG_NAME = "ATTITUDE_QUATERNION"
_MSG_ID = 31
_QUAT_FIELDS = ["q1", "q2", "q3", "q4"]
_RATE_FIELDS = ["rollspeed", "pitchspeed", "yawspeed"]
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME,
                      ["time_boot_ms", *_QUAT_FIELDS, *_RATE_FIELDS, "repr_offset_q"])

pytestmark = F.PYTESTMARK

ANGLE_TOLERANCE_DEG = 1.0
RATE_TOLERANCE = 0.05
RATE_MIN = 0.15
NONTRIVIAL_SPREAD_DEG = 90.0
WITHIN = 0.95
NORM_TOLERANCE = 1e-3


async def test_attitude_quaternion_info_q_matches_attitude_in_flight(gcs_system, request, restart_flight_stack):
    """(Information) In flight the quaternion as ZYX Euler angles matches ATTITUDE (±1°) on every heading flown."""
    fl = await F.get_flight(gcs_system, restart_flight_stack)
    t0, t1 = fl.need_moving(request, _QUAT_FIELDS)
    samples = [s for s in fl.s(_MSG_NAME, t0, t1) if _finite(_q(s)) and _norm(_q(s)) > 0]
    att = fl.boot("ATTITUDE")
    flags, headings = [], []
    worst = [0.0, 0.0, 0.0]
    for s in samples:
        a = att.interp(s.fields["time_boot_ms"], ("roll", "pitch", "yaw"), angles=("roll", "pitch", "yaw"))
        if a is None:
            continue
        e = _euler_deg(_q(s))
        ref = [math.degrees(a[k]) for k in ("roll", "pitch", "yaw")]
        d = [abs(F.angle_diff(x, y)) for x, y in zip(e, ref)]
        worst = [max(w, x) for w, x in zip(worst, d)]
        flags.append(all(x <= ANGLE_TOLERANCE_DEG for x in d))
        headings.append(ref[2])
    if len(flags) < 20:
        skip_untestable(request, _QUAT_FIELDS, "too few ATTITUDE_QUATERNION/ATTITUDE pairs in flight", f"{len(flags)}")
    spread = F.heading_spread(headings)
    ok = F.fraction(flags) >= WITHIN
    for f in _QUAT_FIELDS:
        if not ok:
            record_field_fact(request, f, supported=False)
        elif spread >= NONTRIVIAL_SPREAD_DEG:
            record_field_fact(request, f, supported=True)
        else:
            record_field_fact(request, f, untestable=f"headings flown cover only {spread:.0f}°")
    detail = (f"(Information) {100 * F.fraction(flags):.0f}% within {ANGLE_TOLERANCE_DEG}° of ATTITUDE (worst r/p/y "
              f"{worst[0]:.2f}/{worst[1]:.2f}/{worst[2]:.2f}°), headings covering {spread:.0f}°, n={len(flags)}; {fl.label}")
    record_detail(request, detail)
    assert ok, f"ATTITUDE_QUATERNION doesn't describe the same attitude in flight — {detail}"


async def test_attitude_quaternion_info_rates_match_attitude_in_flight(gcs_system, request, restart_flight_stack):
    """(Information) In flight rollspeed/pitchspeed/yawspeed equal ATTITUDE's (±0.05 rad/s or 10%) where rotating ≥ 0.15 rad/s."""
    fl = await F.get_flight(gcs_system, restart_flight_stack)
    t0, t1 = fl.need_moving(request, _RATE_FIELDS)
    att = fl.boot("ATTITUDE")
    flags = [[], [], []]
    for s in fl.s(_MSG_NAME, t0, t1):
        a = att.interp(s.fields["time_boot_ms"], tuple(_RATE_FIELDS))
        if a is None:
            continue
        for i, f in enumerate(_RATE_FIELDS):
            r, v = a[f], s.fields[f]
            if _finite([r, v]) and abs(r) >= RATE_MIN:
                flags[i].append(abs(v - r) <= max(RATE_TOLERANCE, 0.1 * abs(r)))
    failed = []
    for f, x in zip(_RATE_FIELDS, flags):
        if len(x) < 5:
            record_field_fact(request, f, untestable=f"in flight only {len(x)} samples rotating ≥ {RATE_MIN} rad/s")
        elif F.fraction(x) < WITHIN:
            record_field_fact(request, f, supported=False)
            failed.append(f)
        else:
            record_field_fact(request, f, supported=True)
    detail = "(Information) " + "; ".join(f"{f} {100 * F.fraction(x):.0f}% of {len(x)}" for f, x in zip(_RATE_FIELDS, flags)) \
        + f" within max({RATE_TOLERANCE} rad/s, 10%) of ATTITUDE at ≥ {RATE_MIN} rad/s; {fl.label}"
    record_detail(request, detail)
    assert not failed, f"body rates differ from ATTITUDE's in flight — {detail}"


async def test_attitude_quaternion_compat_repr_offset_q_in_flight(gcs_system, request, restart_flight_stack):
    """In flight repr_offset_q is [0,0,0,0] or a unit quaternion; characterise what's sent in forward flight."""
    fl = await F.get_flight(gcs_system, restart_flight_stack)
    t0, t1 = fl.need_moving(request, ["repr_offset_q"])
    samples = fl.s(_MSG_NAME, t0, t1)
    values = [s.fields.get("repr_offset_q") for s in samples]
    if not values or any(v is None for v in values):
        skip_untestable(request, ["repr_offset_q"], "not decoded (extension missing), or not sent in flight")
    ess = fl.s("EXTENDED_SYS_STATE")
    zero = [v for v in values if all(c == 0 for c in v)]
    unit = [v for v in values if _finite(v) and abs(_norm(v) - 1) <= NORM_TOLERANCE]
    bad = [v for v in values if v not in zero and v not in unit]
    in_fw = [s.fields["repr_offset_q"] for s in samples
             if ess and min(ess, key=lambda e: abs(e.host_mono - s.host_mono)).fields["vtol_state"] == 4]
    shown = sorted({tuple(round(c, 3) for c in v) for v in values})[:3]
    detail = (f"{len(zero)} × [0,0,0,0], {len(unit)} unit ({shown}); in FW (vtol_state): "
              f"{'none' if not in_fw else sorted({tuple(round(c, 3) for c in v) for v in in_fw})[:2]}; {fl.label}")
    record_detail(request, detail)
    if bad:
        record_field_fact(request, "repr_offset_q", supported=False)
        report.compat_fail(f"repr_offset_q neither [0,0,0,0] nor a unit quaternion: {bad[0]}")
    if unit:
        record_field_fact(request, "repr_offset_q", supported=True,
                          notes="An offset in forward flight" if in_fw and any(v in unit for v in in_fw) else None)
