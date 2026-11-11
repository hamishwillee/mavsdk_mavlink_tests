"""
LOCAL_POSITION_NED (message id 32) — https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED

What the XML defines (README.md in this directory has the reasoning and results):

- "The filtered local position (e.g. fused computer vision and
  accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical
  frame, NED / north-east-down convention)."
- `x`/`y`/`z` (m), `vx`/`vy`/`vz` (m/s), `time_boot_ms`. No invalid values,
  and the frame's origin isn't stated (GPS_GLOBAL_ORIGIN "publishes the GPS
  coordinates of the vehicle local origin").

So, with nothing moving:
- The position should be GLOBAL_POSITION_INT's position expressed relative to
  GPS_GLOBAL_ORIGIN (±1 m) — info, since a mismatch says the messages
  disagree, not which one is wrong. That checks the axes and signs only when
  the vehicle is away from the origin (the mock: 40 m N, 30 m E); a SITL that
  set its origin where it stands gives ~0 whichever way round the axes are, so
  those fields are recorded untestable (root CLAUDE.md rule 10).
- Velocities: finite, ~0 and equal to GLOBAL_POSITION_INT's at rest. Their
  axes need movement, so they're recorded untestable.
"""

from __future__ import annotations

import logging
import math

import pytest

from tests import report
from tests.messages_telemetry.conftest import (
    finite,
    ned_from_origin,
    record_detail,
    record_field_fact,
    skip_untestable,
)
from tests.messages_telemetry.emission import Emission

log = logging.getLogger(__name__)

_MSG_NAME = "LOCAL_POSITION_NED"
_MSG_ID = 32
REQUIRED_MESSAGES = {
    "LOCAL_POSITION_NED": "the message under test",
    "GLOBAL_POSITION_INT": "the global position it's compared against",
}
_POS = ["x", "y", "z"]
_VEL = ["vx", "vy", "vz"]
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME, ["time_boot_ms", *_POS, *_VEL])

pytestmark = pytest.mark.timeout(180)

EMISSION = Emission(_MSG_NAME, _MSG_ID, window_extra=("GLOBAL_POSITION_INT", "GPS_GLOBAL_ORIGIN"))
EMISSION.define_tests(globals(), "local_position_ned")

POSITION_TOLERANCE_M = 1.0   # vs GLOBAL_POSITION_INT + origin: both from the same estimate
NONTRIVIAL_M = 5.0           # an axis this far from 0 makes a swapped or flipped axis show
VELOCITY_TOLERANCE = 0.3     # m/s, vs GLOBAL_POSITION_INT (cm/s) at rest
REST_SPEED_MAX = 0.5         # m/s: a vehicle on the ground isn't moving faster than this


def _nearest_boot(samples, boot_ms):
    return min(samples, key=lambda s: abs(s.fields["time_boot_ms"] - boot_ms))


async def test_local_position_ned_compat_boot_ms_tracks_real_time(gcs_system, request):
    """time_boot_ms never goes backwards or sticks, and advances at host-clock rate (±5%)."""
    await EMISSION.boot_ms_tracks_real_time(gcs_system, request)


async def test_local_position_ned_info_position_matches_global(gcs_system, request):
    """(Information) x/y/z equal GLOBAL_POSITION_INT's position relative to GPS_GLOBAL_ORIGIN (±1 m)."""
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        skip_untestable(request, _POS, "not enough messages to analyse", w.na)
    origins = w.others.get("GPS_GLOBAL_ORIGIN") or []
    gpi = w.others.get("GLOBAL_POSITION_INT") or []
    if not origins or not gpi:
        skip_untestable(request, _POS, "no GPS_GLOBAL_ORIGIN or GLOBAL_POSITION_INT to compare against",
                        f"GPS_GLOBAL_ORIGIN {len(origins)}, GLOBAL_POSITION_INT {len(gpi)} received")
    origin = origins[-1].fields
    nonfinite = sum(1 for s in w.samples if not finite(*(s.fields[f] for f in _POS)))
    worst = [0.0, 0.0, 0.0]
    for g in gpi:
        lp = _nearest_boot(w.samples, g.fields["time_boot_ms"])
        expected = ned_from_origin(origin, g.fields["lat"], g.fields["lon"], g.fields["alt"])
        for i, f in enumerate(_POS):
            if finite(lp.fields[f]):
                worst[i] = max(worst[i], abs(lp.fields[f] - expected[i]))
    last = w.samples[-1].fields
    ok = nonfinite == 0 and all(d <= POSITION_TOLERANCE_M for d in worst)
    for i, f in enumerate(_POS):
        strong = finite(last[f]) and abs(last[f]) >= NONTRIVIAL_M
        if not ok:
            record_field_fact(request, f, supported=False)
        elif strong:
            record_field_fact(request, f, supported=True)
        else:
            record_field_fact(request, f, untestable=f"at rest {last[f]:+.1f} m from the origin: ~0 either way; "
                                                     "needs movement")
    detail = (f"(Information) position {last['x']:+.2f}/{last['y']:+.2f}/{last['z']:+.2f} m; "
              f"vs GLOBAL_POSITION_INT relative to origin worst {worst[0]:.2f}/{worst[1]:.2f}/{worst[2]:.2f} m "
              f"(n={len(gpi)}), {nonfinite} non-finite")
    record_detail(request, detail)
    assert ok, f"LOCAL_POSITION_NED isn't GLOBAL_POSITION_INT relative to GPS_GLOBAL_ORIGIN — {detail}"


async def test_local_position_ned_info_velocity_at_rest(gcs_system, request):
    """(Information) At rest vx/vy/vz are finite, small (<0.5 m/s) and equal GLOBAL_POSITION_INT's (±0.3 m/s)."""
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        skip_untestable(request, _VEL, "not enough messages to analyse", w.na)
    vels = [[s.fields[f] for f in _VEL] for s in w.samples]
    nonfinite = sum(1 for v in vels if not finite(*v))
    fastest = max((math.sqrt(sum(c * c for c in v)) for v in vels if finite(*v)), default=0.0)
    parts = [f"{len(vels)} msgs: max speed {fastest:.3f} m/s, {nonfinite} non-finite"]
    agree = True
    gpi = w.others.get("GLOBAL_POSITION_INT") or []
    if gpi:
        dmax = 0.0
        for g in gpi:
            lp = _nearest_boot(w.samples, g.fields["time_boot_ms"])
            for f in _VEL:
                if finite(lp.fields[f]):
                    dmax = max(dmax, abs(lp.fields[f] - g.fields[f] / 100.0))
        agree = dmax <= VELOCITY_TOLERANCE
        parts.append(f"vs GLOBAL_POSITION_INT worst {dmax:.3f} m/s")
    else:
        parts.append("no GLOBAL_POSITION_INT to compare")
    for f in _VEL:
        record_field_fact(request, f, untestable="needs movement; at rest only finite, ~0 and as GLOBAL_POSITION_INT")
    detail = "(Information) " + "; ".join(parts)
    record_detail(request, detail)
    assert nonfinite == 0 and fastest <= REST_SPEED_MAX and agree, f"velocity implausible at rest — {detail}"
