"""
ATTITUDE_QUATERNION (message id 31) — https://mavlink.io/en/messages/common.html#ATTITUDE_QUATERNION

What the XML defines (README.md in this directory has the full reasoning):

- "The attitude in the aeronautical frame (right-handed, Z-down, X-front,
  Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero
  rotation would be expressed as (1 0 0 0)." No invalid value for q1..q4.
- `rollspeed`/`pitchspeed`/`yawspeed` (rad/s): body angular rates, worded
  exactly as in ATTITUDE. No invalid value.
- `repr_offset_q` (extension, `invalid="[0]"`): a display-offset rotation,
  [w, x, y, z]; [0, 0, 0, 0] "if field not supported", [1, 0, 0, 0] for no
  offset (the XML's example: a tailsitter in fixed-wing mode sends a 90° pitch).

So, with nothing flying:
- An attitude quaternion is a rotation, so it must have unit norm — compat.
- Converted to ZYX Euler angles it should match ATTITUDE (whose XML gives the
  same frame) and its yaw the GLOBAL_POSITION_INT heading — info, since a
  mismatch says the two messages disagree, not which is wrong. ArduPlane and
  QuadPlane SITL start facing 270°, so component sign/order errors show; near a
  0° yaw (every other SITL here) only component-order errors would, and the
  detail says which.
- The rates can only be shown to be right while rotating. At rest the test
  checks they're finite and agree with ATTITUDE's, and records the fields as
  untestable rather than supported (root CLAUDE.md rule 10).
- `repr_offset_q` must be [0, 0, 0, 0] or a unit quaternion — compat. Which one
  is characterised. The tailsitter's fixed-wing-mode value needs a flight.
"""

from __future__ import annotations

import logging
import math
import statistics

import pytest

from tests import report
from tests.messages_telemetry.conftest import record_detail, record_field_fact
from tests.messages_telemetry.emission import Emission

log = logging.getLogger(__name__)

_MSG_NAME = "ATTITUDE_QUATERNION"
_MSG_ID = 31
REQUIRED_MESSAGES = {
    "ATTITUDE_QUATERNION": "the message under test",
    "ATTITUDE": "the Euler attitude it's compared against",
}
_QUAT_FIELDS = ["q1", "q2", "q3", "q4"]
_RATE_FIELDS = ["rollspeed", "pitchspeed", "yawspeed"]
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME,
                      ["time_boot_ms", *_QUAT_FIELDS, *_RATE_FIELDS, "repr_offset_q"])

pytestmark = pytest.mark.timeout(180)

EMISSION = Emission(_MSG_NAME, _MSG_ID, window_extra=("ATTITUDE", "GLOBAL_POSITION_INT"))
EMISSION.define_tests(globals(), "attitude_quaternion")  # the five emission tests

NORM_TOLERANCE = 1e-3       # |q| = 1 ± this (float32 EKF output is normalised to ~1e-6)
ANGLE_TOLERANCE_DEG = 1.0   # quaternion-derived Euler vs ATTITUDE, same estimate
HEADING_TOLERANCE_DEG = 2.0 # quaternion yaw vs GLOBAL_POSITION_INT.hdg (cdeg resolution, own sample time)
# rad/s, rates vs ATTITUDE's at rest. Each message copies the latest gyro sample when it's
# sent, so at rest they differ by gyro noise: up to 0.018 rad/s on PX4 SIH (2026-11-11).
# A different quantity (deg/s, another frame) would be off by far more.
RATE_TOLERANCE = 0.05
REST_RATE_MAX = 0.2         # rad/s: a vehicle on the ground isn't turning faster than this
NONTRIVIAL_DEG = 10.0       # an angle this far from 0/180 makes sign errors visible


def _q(s) -> list[float]:
    return [s.fields[k] for k in _QUAT_FIELDS]


def _norm(q) -> float:
    return math.sqrt(sum(c * c for c in q))


def _euler_deg(q) -> tuple[float, float, float]:
    """[w, x, y, z] -> ZYX intrinsic roll, pitch, yaw in degrees (yaw in -180..180)."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def _angle_diff(a: float, b: float) -> float:
    """a - b in degrees, wrapped to -180..180."""
    return (a - b + 180.0) % 360.0 - 180.0


def _nearest(samples, boot_ms: int):
    return min(samples, key=lambda s: abs(s.fields["time_boot_ms"] - boot_ms))


def _finite(values) -> bool:
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


# ---------------------------------------------------------------------------
# B. Fields
# ---------------------------------------------------------------------------

async def test_attitude_quaternion_compat_boot_ms_tracks_real_time(gcs_system, request):
    """time_boot_ms never goes backwards or sticks, and advances at host-clock rate (±5%)."""
    await EMISSION.boot_ms_tracks_real_time(gcs_system, request)


async def test_attitude_quaternion_compat_q_is_unit(gcs_system, request):
    """Every q1..q4 is finite and |q| = 1 (±1e-3): an attitude quaternion is a rotation."""
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        for f in _QUAT_FIELDS:
            record_field_fact(request, f, untestable="not enough messages to analyse")
        pytest.skip(f"NA: {w.na}")
    qs = [_q(s) for s in w.samples]
    nonfinite = sum(1 for q in qs if not _finite(q))
    norms = [_norm(q) for q in qs if _finite(q)]
    worst = max((abs(n - 1) for n in norms), default=float("inf"))
    negative_w = sum(1 for q in qs if _finite(q) and q[0] < 0)
    ok = nonfinite == 0 and worst <= NORM_TOLERANCE
    detail = (f"{len(qs)} msgs: worst |q|−1 {worst:.1e}, {nonfinite} non-finite, "
              f"w<0 in {negative_w} (q and −q are the same rotation)")
    if not ok:
        for f in _QUAT_FIELDS:
            record_field_fact(request, f, supported=False)
        record_detail(request, detail)
        report.compat_fail(f"not a unit quaternion — {detail}")
    record_detail(request, detail)
    # Supported is decided by the next test, which checks the rotation itself.


async def test_attitude_quaternion_info_q_matches_attitude(gcs_system, request):
    """(Information) The quaternion as ZYX Euler angles matches ATTITUDE (±1°), and its yaw the GPS heading (±2°)."""
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        pytest.skip(f"NA: {w.na}")
    att = w.others.get("ATTITUDE") or []
    if not att:
        for f in _QUAT_FIELDS:
            record_field_fact(request, f, untestable="no ATTITUDE to compare against")
        pytest.skip("NA: no ATTITUDE received to compare against")
    diffs = []  # (droll, dpitch, dyaw) in degrees
    for s in w.samples:
        q = _q(s)
        if not _finite(q) or _norm(q) == 0:
            continue
        a = _nearest(att, s.fields["time_boot_ms"])
        r, p, y = _euler_deg(q)
        diffs.append((_angle_diff(r, math.degrees(a.fields["roll"])),
                      _angle_diff(p, math.degrees(a.fields["pitch"])),
                      _angle_diff(y, math.degrees(a.fields["yaw"]))))
    if not diffs:
        pytest.skip("NA: no finite quaternion to compare")
    worst = [max(abs(d[i]) for d in diffs) for i in range(3)]
    roll, pitch, yaw = _euler_deg(_q(w.samples[-1]))
    yaw360 = yaw % 360.0
    strong = min(abs(_angle_diff(yaw, 0)), abs(_angle_diff(yaw, 180))) >= NONTRIVIAL_DEG
    strength = ("yaw far from 0/180: sign and order errors would show" if strong else
                "yaw near 0/180: only component-order errors would show")

    heading = ""
    heading_ok = True
    gpi = [g for g in (w.others.get("GLOBAL_POSITION_INT") or []) if g.fields.get("hdg") != 65535]
    if gpi:
        hdiff = max(abs(_angle_diff(_euler_deg(_q(_nearest(w.samples, g.fields["time_boot_ms"])))[2],
                                    g.fields["hdg"] / 100.0)) for g in gpi)
        heading_ok = hdiff <= HEADING_TOLERANCE_DEG
        heading = f"; vs GLOBAL_POSITION_INT.hdg worst {hdiff:.2f}° (n={len(gpi)})"
    else:
        heading = "; no GLOBAL_POSITION_INT heading to compare"

    ok = all(x <= ANGLE_TOLERANCE_DEG for x in worst) and heading_ok
    for f in _QUAT_FIELDS:
        record_field_fact(request, f, supported=ok)
    detail = (f"(Information) attitude r/p/y {roll:+.1f}/{pitch:+.1f}/{yaw360:.1f}°; vs ATTITUDE worst "
              f"{worst[0]:.2f}/{worst[1]:.2f}/{worst[2]:.2f}° (n={len(diffs)}){heading} — {strength}")
    record_detail(request, detail)
    assert ok, f"ATTITUDE_QUATERNION doesn't describe the same attitude — {detail}"


async def test_attitude_quaternion_info_rates_at_rest(gcs_system, request):
    """(Information) At rest the body rates are finite, small (<0.2 rad/s) and agree with ATTITUDE's (±0.05 rad/s)."""
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        for f in _RATE_FIELDS:
            record_field_fact(request, f, untestable="not enough messages to analyse")
        pytest.skip(f"NA: {w.na}")
    att = w.others.get("ATTITUDE") or []
    rates = [[s.fields[f] for f in _RATE_FIELDS] for s in w.samples]
    nonfinite = sum(1 for r in rates if not _finite(r))
    finite = [r for r in rates if _finite(r)]
    biggest = max((abs(v) for r in finite for v in r), default=0.0)
    all_zero = bool(finite) and all(v == 0 for r in finite for v in r)
    parts = [f"{len(rates)} msgs: max |rate| {biggest:.4f} rad/s, {nonfinite} non-finite"
             + (", all exactly 0" if all_zero else "")]
    agree = True
    if att:
        dmax = 0.0
        for s in w.samples:
            a = _nearest(att, s.fields["time_boot_ms"])
            for f in _RATE_FIELDS:
                if _finite([s.fields[f], a.fields[f]]):
                    dmax = max(dmax, abs(s.fields[f] - a.fields[f]))
        agree = dmax <= RATE_TOLERANCE
        parts.append(f"vs ATTITUDE worst {dmax:.4f} rad/s")
    else:
        parts.append("no ATTITUDE to compare")
    ok = nonfinite == 0 and biggest <= REST_RATE_MAX and agree
    for f in _RATE_FIELDS:
        record_field_fact(request, f, untestable="needs rotation; at rest only finite, small and as ATTITUDE")
    detail = "(Information) " + "; ".join(parts)
    record_detail(request, detail)
    assert ok, f"body rates implausible at rest — {detail}"


async def test_attitude_quaternion_compat_repr_offset_q(gcs_system, request):
    """repr_offset_q is [0,0,0,0] (not supported) or a unit quaternion; characterise which."""
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        record_field_fact(request, "repr_offset_q", untestable="not enough messages to analyse")
        pytest.skip(f"NA: {w.na}")
    values = [s.fields.get("repr_offset_q") for s in w.samples]
    if any(v is None for v in values):
        record_field_fact(request, "repr_offset_q", untestable="not decoded (extension missing)")
        pytest.skip("NA: repr_offset_q not in the decoded message (extension not sent or not decoded)")
    zero = [v for v in values if all(c == 0 for c in v)]
    unit = [v for v in values if _finite(v) and abs(_norm(v) - 1) <= NORM_TOLERANCE]
    bad = [v for v in values if v not in zero and v not in unit]
    distinct = sorted({tuple(round(c, 4) for c in v) for v in values})
    shown = ", ".join(str(list(d)) for d in distinct[:3])
    if bad:
        record_field_fact(request, "repr_offset_q", supported=False)
        record_detail(request, f"{len(bad)} invalid values, e.g. {bad[0]}")
        report.compat_fail(f"repr_offset_q neither [0,0,0,0] nor a unit quaternion: {bad[0]}")
    if zero and not unit:
        # Can't tell "not supported" from "supported, no offset": PX4 sends [0,0,0,0]
        # for no offset (to save bandwidth) and only fills it for a tailsitter in
        # fixed-wing mode, which needs a flight. Don't record a verdict the data
        # didn't establish (root CLAUDE.md rule 10).
        record_field_fact(request, "repr_offset_q",
                          untestable="only [0,0,0,0] seen: not supported, or no offset; needs tailsitter FW flight")
        what = "always [0, 0, 0, 0] (\"not supported\" per the XML — or no offset)"
    elif unit and not zero:
        record_field_fact(request, "repr_offset_q", supported=True)
        what = f"supported: {shown}"
    else:
        record_field_fact(request, "repr_offset_q", supported=True, notes="Switches between [0,0,0,0] and an offset")
        what = f"mixed: {len(zero)} × [0, 0, 0, 0] and {len(unit)} unit quaternions ({shown})"
    record_detail(request, f"{what} ({len(values)} msgs)")
