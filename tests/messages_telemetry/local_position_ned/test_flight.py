"""
LOCAL_POSITION_NED in flight — the position and velocity checks that need
movement (README.md in this directory; the shared flight is in ../flight.py).

At rest every axis reads ~0, so a swapped or sign-flipped axis can't show. On
the shared flight the vehicle goes north, east, south and west and climbs and
descends, so each axis takes both signs:
- x/y/z equal GLOBAL_POSITION_INT relative to GPS_GLOBAL_ORIGIN (±1.5 m, plus
  the distance moved between the two messages' timestamps);
- vx/vy/vz equal GLOBAL_POSITION_INT's velocity (±0.5 m/s), and the rate of
  change of x/y/z over 0.5 s (±0.5 m/s or 10% of the speed) — the second needs
  nothing but this message.
An axis is recorded supported once it has been far from 0 (5 m, 1 m/s).
"""

from __future__ import annotations

import bisect
import math

from tests import report
from tests.flight_helpers import require_real_stack  # noqa: F401 — autouse: flight tests need a real stack
from tests.messages_telemetry import flight as F
from tests.messages_telemetry.conftest import finite, ned_from_origin, nearest, record_detail, record_field_fact, skip_untestable

_MSG_NAME = "LOCAL_POSITION_NED"
_MSG_ID = 32
_POS = ["x", "y", "z"]
_VEL = ["vx", "vy", "vz"]
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME, ["time_boot_ms", *_POS, *_VEL])

pytestmark = F.PYTESTMARK

POSITION_TOLERANCE_M = 1.5
VELOCITY_TOLERANCE = 0.5     # m/s
WITHIN = 0.95                # fraction of samples inside the tolerance
NONTRIVIAL_M = 5.0
NONTRIVIAL_MS = 1.0
DERIVATIVE_SPAN_MS = 500
DERIVATIVE_RELATIVE = 0.1   # or 10% of the speed: a fixed-wing turning at 20 m/s changes velocity within the span


def _axis_verdicts(request, fields, ok: list[bool], extremes: list[tuple[float, float]], nontrivial: float,
                   unit: str) -> None:
    for f, good, (lo, hi) in zip(fields, ok, extremes):
        if not good:
            record_field_fact(request, f, supported=False)
        elif max(abs(lo), abs(hi)) >= nontrivial:
            record_field_fact(request, f, supported=True)
        else:
            record_field_fact(request, f, untestable=f"in flight only {lo:+.1f}..{hi:+.1f} {unit}: ~0 either way")


async def test_local_position_ned_info_position_in_flight(gcs_system, request, restart_flight_stack):
    """(Information) In flight x/y/z equal GLOBAL_POSITION_INT relative to GPS_GLOBAL_ORIGIN (±1.5 m) on every axis."""
    fl = await F.get_flight(gcs_system, restart_flight_stack)
    t0, t1 = fl.need_moving(request, _POS)
    lp = fl.s(_MSG_NAME, t0, t1)
    gpi = fl.boot("GLOBAL_POSITION_INT")
    origins = fl.s("GPS_GLOBAL_ORIGIN")
    if len(lp) < 20 or not origins or not gpi.samples:
        skip_untestable(request, _POS, "not enough messages recorded in flight",
                        f"{len(lp)} LOCAL_POSITION_NED, {len(origins)} GPS_GLOBAL_ORIGIN, {len(gpi.samples)} GLOBAL_POSITION_INT")
    inside = [[], [], []]
    worst = [0.0, 0.0, 0.0]
    nonfinite = 0
    for s in lp:
        f = s.fields
        if not finite(*(f[k] for k in _POS)):
            nonfinite += 1
            continue
        g = gpi.at(f["time_boot_ms"])
        if g is None:
            continue
        dt = abs(g.fields["time_boot_ms"] - f["time_boot_ms"]) / 1000.0
        origin = nearest(origins, s.host_mono).fields
        exp = ned_from_origin(origin, g.fields["lat"], g.fields["lon"], g.fields["alt"])
        for i, k in enumerate(_POS):
            err = abs(f[k] - exp[i]) - abs(f[_VEL[i]] if finite(f[_VEL[i]]) else 0.0) * dt
            worst[i] = max(worst[i], err)
            inside[i].append(err <= POSITION_TOLERANCE_M)
    extremes = [(min(s.fields[k] for s in lp if finite(s.fields[k])), max(s.fields[k] for s in lp if finite(s.fields[k])))
                for k in _POS]
    ok = [nonfinite == 0 and F.fraction(x) >= WITHIN for x in inside]
    _axis_verdicts(request, _POS, ok, extremes, NONTRIVIAL_M, "m")
    detail = ("(Information) " + "; ".join(
        f"{k} {lo:+.0f}..{hi:+.0f} m, {100 * F.fraction(x):.0f}% within {POSITION_TOLERANCE_M} m (worst {w:.2f})"
        for k, (lo, hi), x, w in zip(_POS, extremes, inside, worst))
        + f"; n={len(inside[0])}, {nonfinite} non-finite; {fl.label}")
    record_detail(request, detail)
    assert all(ok), f"LOCAL_POSITION_NED isn't GLOBAL_POSITION_INT relative to GPS_GLOBAL_ORIGIN in flight — {detail}"


async def test_local_position_ned_info_velocity_in_flight(gcs_system, request, restart_flight_stack):
    """(Information) In flight vx/vy/vz equal GLOBAL_POSITION_INT's velocity (±0.5 m/s) and the rate of change of x/y/z."""
    fl = await F.get_flight(gcs_system, restart_flight_stack)
    t0, t1 = fl.need_moving(request, _VEL)
    lp = sorted((s for s in fl.s(_MSG_NAME, t0, t1) if finite(*(s.fields[k] for k in _POS + _VEL))),
                key=lambda s: s.fields["time_boot_ms"])
    gpi = fl.boot("GLOBAL_POSITION_INT")
    if len(lp) < 20:
        skip_untestable(request, _VEL, "not enough messages recorded in flight", f"{len(lp)} LOCAL_POSITION_NED")
    vs_gpi = [[], [], []]
    vs_deriv = [[], [], []]
    keys = [s.fields["time_boot_ms"] for s in lp]
    for idx, s in enumerate(lp):
        f = s.fields
        g = gpi.interp(f["time_boot_ms"], tuple(_VEL))
        if g is not None:
            for i, k in enumerate(_VEL):
                vs_gpi[i].append(abs(f[k] - g[k] / 100.0) <= VELOCITY_TOLERANCE)
        j = bisect.bisect_left(keys, f["time_boot_ms"] + DERIVATIVE_SPAN_MS)
        if j < len(lp) and keys[j] - f["time_boot_ms"] <= DERIVATIVE_SPAN_MS * 1.5:
            e = lp[j].fields
            dt = (e["time_boot_ms"] - f["time_boot_ms"]) / 1000.0
            for i, (p, v) in enumerate(zip(_POS, _VEL)):
                speed = math.sqrt(sum(f[k] ** 2 for k in _VEL))
                vs_deriv[i].append(abs((e[p] - f[p]) / dt - (f[v] + e[v]) / 2)
                                   <= max(VELOCITY_TOLERANCE, DERIVATIVE_RELATIVE * speed))
    if not vs_gpi[0] or not vs_deriv[0]:
        skip_untestable(request, _VEL, "no GLOBAL_POSITION_INT or consecutive samples to compare against",
                        f"{len(vs_gpi[0])} GLOBAL_POSITION_INT pairs, {len(vs_deriv[0])} derivative pairs")
    extremes = [(min(s.fields[k] for s in lp), max(s.fields[k] for s in lp)) for k in _VEL]
    ok = [F.fraction(a) >= WITHIN and F.fraction(b) >= WITHIN for a, b in zip(vs_gpi, vs_deriv)]
    _axis_verdicts(request, _VEL, ok, extremes, NONTRIVIAL_MS, "m/s")
    detail = ("(Information) " + "; ".join(
        f"{k} {lo:+.1f}..{hi:+.1f} m/s, {100 * F.fraction(a):.0f}% as GLOBAL_POSITION_INT, "
        f"{100 * F.fraction(b):.0f}% as d{k[1]}/dt"
        for k, (lo, hi), a, b in zip(_VEL, extremes, vs_gpi, vs_deriv))
        + f" (within {VELOCITY_TOLERANCE} m/s; the derivative over {DERIVATIVE_SPAN_MS} ms within max({VELOCITY_TOLERANCE} m/s, "
        f"{DERIVATIVE_RELATIVE:.0%}); n={len(lp)}); {fl.label}")
    record_detail(request, detail)
    assert all(ok), f"LOCAL_POSITION_NED velocity doesn't match its position or GLOBAL_POSITION_INT — {detail}"
