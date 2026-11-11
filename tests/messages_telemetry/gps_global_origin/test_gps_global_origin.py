"""
GPS_GLOBAL_ORIGIN (message id 49) — https://mavlink.io/en/messages/common.html#GPS_GLOBAL_ORIGIN

What the XML defines (README.md in this directory has the reasoning and results):

- "Publishes the GPS coordinates of the vehicle local origin (0,0,0) position.
  Emitted whenever a new GPS-Local position mapping is requested or set - e.g.
  following SET_GPS_GLOBAL_ORIGIN message."
- latitude/longitude (degE7, WGS84), altitude (mm, MSL); time_usec
  (extension, "UNIX Epoch time or time since system boot"). No invalid values.

So, with nothing moving:
- The origin plus LOCAL_POSITION_NED should be GLOBAL_POSITION_INT's position
  (±1 m) — info. Unlike the local-frame checks, this tests the global fields
  absolutely: any unit, axis or reference error moves the result by far more.
- The origin doesn't change while the vehicle sits still — info.
- "Emitted whenever ... set" says nothing about streaming; a stack that only
  sends it on change shows up in the emission tests (an accepted rate it
  doesn't honour is still rule 4a's compatibility error).
"""

from __future__ import annotations

import logging
import math

import pytest

from tests import report
from tests.messages_telemetry.conftest import (
    classify_time_usec,
    finite,
    ned_from_origin,
    record_detail,
    record_field_fact,
    skip_untestable,
)
from tests.messages_telemetry.emission import Emission

log = logging.getLogger(__name__)

_MSG_NAME = "GPS_GLOBAL_ORIGIN"
_MSG_ID = 49
REQUIRED_MESSAGES = {
    "GPS_GLOBAL_ORIGIN": "the message under test",
    "GLOBAL_POSITION_INT": "the global position it's checked against",
}
_FIELDS = ["latitude", "longitude", "altitude"]
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME, [*_FIELDS, "time_usec"])

pytestmark = pytest.mark.timeout(180)

EMISSION = Emission(_MSG_NAME, _MSG_ID, window_extra=("GLOBAL_POSITION_INT", "LOCAL_POSITION_NED"))
EMISSION.define_tests(globals(), "gps_global_origin")

TOLERANCE_M = 1.0


async def test_gps_global_origin_info_origin_plus_local_is_global(gcs_system, request):
    """(Information) Origin + LOCAL_POSITION_NED equals GLOBAL_POSITION_INT's position (±1 m)."""
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        skip_untestable(request, _FIELDS, "not enough messages to analyse", w.na)
    gpi = w.others.get("GLOBAL_POSITION_INT") or []
    lpn = w.others.get("LOCAL_POSITION_NED") or []
    if not gpi or not lpn:
        skip_untestable(request, _FIELDS, "no GLOBAL_POSITION_INT or LOCAL_POSITION_NED to compare against",
                        f"GLOBAL_POSITION_INT {len(gpi)}, LOCAL_POSITION_NED {len(lpn)} received")
    o = w.samples[-1].fields
    worst = [0.0, 0.0, 0.0]
    for g in gpi:
        lp = min(lpn, key=lambda s: abs(s.fields["time_boot_ms"] - g.fields["time_boot_ms"]))
        ned = ned_from_origin(o, g.fields["lat"], g.fields["lon"], g.fields["alt"])
        for i, f in enumerate(("x", "y", "z")):
            if finite(lp.fields[f]):
                worst[i] = max(worst[i], abs(ned[i] - lp.fields[f]))
    ok = all(d <= TOLERANCE_M for d in worst)
    for f in _FIELDS:
        record_field_fact(request, f, supported=ok)
    detail = (f"(Information) origin {o['latitude'] / 1e7:.7f}, {o['longitude'] / 1e7:.7f}, "
              f"{o['altitude'] / 1000:.2f} m; origin+local vs global worst N/E/D "
              f"{worst[0]:.2f}/{worst[1]:.2f}/{worst[2]:.2f} m (n={len(gpi)})")
    record_detail(request, detail)
    assert ok, f"GPS_GLOBAL_ORIGIN + LOCAL_POSITION_NED isn't GLOBAL_POSITION_INT — {detail}"


async def test_gps_global_origin_info_constant_at_rest(gcs_system, request):
    """(Information) The origin doesn't move while the vehicle sits still."""
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        pytest.skip(f"NA: {w.na}")
    values = {tuple(s.fields[f] for f in _FIELDS) for s in w.samples}
    record_detail(request, f"(Information) {len(values)} distinct origin(s) in {len(w.samples)} msgs")
    assert len(values) == 1, f"origin changed at rest: {sorted(values)[:3]}"


async def test_gps_global_origin_obs_time_usec(gcs_system, request):
    """Characterise time_usec: UNIX or boot time, and whether it's the send time or when the origin was set."""
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
