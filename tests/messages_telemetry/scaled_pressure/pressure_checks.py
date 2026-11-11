"""
Field checks shared by SCALED_PRESSURE, SCALED_PRESSURE2 and SCALED_PRESSURE3
(identical fields; README.md in this directory has the reasoning and results).

The XML: press_abs and press_diff in hPa (floats), temperature ("Absolute
pressure temperature", cdegC, no invalid value) and temperature_press_diff
(extension, cdegC, invalid 0: "0, if not available. Report values of 0 (or 1)
as 1 cdegC"). With the vehicle at rest:

- press_abs is the ISA pressure at GLOBAL_POSITION_INT's altitude (MSL), ±10 hPa
  (~80 m) — compat: Pa or kPa would be off by 100×.
- press_diff (an airspeed sensor's dynamic pressure) is ~0 with no wind
  (|·| ≤ 1 hPa, ~13 m/s) — info; recorded untestable, since showing it's right
  needs airspeed. Always exactly 0 is characterised as "no sensor".
- temperature: −40..85 °C (compat); 0 is characterised (it has no invalid value).
- temperature_press_diff: 0 = not available; otherwise −40..85 °C (compat).
"""

from __future__ import annotations

import math
import statistics

from tests import report
from tests.messages_telemetry.conftest import finite, record_detail, record_field_fact, skip_untestable

FIELDS = ["time_boot_ms", "press_abs", "press_diff", "temperature", "temperature_press_diff"]
PRESS_TOLERANCE_HPA = 10.0
PRESS_DIFF_REST_MAX = 1.0
TEMP_RANGE = (-4000, 8500)


def isa_pressure_hpa(alt_m: float) -> float:
    return 1013.25 * (1 - 2.25577e-5 * alt_m) ** 5.25588


async def press_abs_matches_altitude(emission, system, request) -> None:
    w = await emission.sample_window(system)
    if w.na:
        skip_untestable(request, ["press_abs"], "message not sent, or too few to analyse", w.na)
    values = [s.fields["press_abs"] for s in w.samples]
    if all(v == 0 for v in values):
        record_field_fact(request, "press_abs", supported=False, notes="Always 0 (no barometer on this instance)")
        record_detail(request, f"press_abs always 0 ({len(values)} msgs): no barometer")
        return
    gpi = w.others.get("GLOBAL_POSITION_INT") or []
    if not gpi:
        skip_untestable(request, ["press_abs"], "no GLOBAL_POSITION_INT altitude to compare against")
    alt = gpi[-1].fields["alt"] / 1000
    expected = isa_pressure_hpa(alt)
    mean = statistics.fmean(v for v in values if finite(v)) if any(finite(v) for v in values) else float("nan")
    ok = finite(mean) and abs(mean - expected) <= PRESS_TOLERANCE_HPA
    record_field_fact(request, "press_abs", supported=ok)
    detail = (f"press_abs mean {mean:.2f} hPa (range {min(values):.2f}..{max(values):.2f}); ISA at "
              f"{alt:.1f} m MSL {expected:.2f} hPa ({len(values)} msgs)")
    record_detail(request, detail)
    if not ok:
        report.compat_fail(f"press_abs isn't the absolute pressure in hPa — {detail}")


async def press_diff_at_rest(emission, system, request) -> None:
    w = await emission.sample_window(system)
    if w.na:
        skip_untestable(request, ["press_diff"], "message not sent, or too few to analyse", w.na)
    values = [s.fields["press_diff"] for s in w.samples]
    if all(v == 0 for v in values):
        record_field_fact(request, "press_diff", untestable="always exactly 0: no sensor, or no airflow")
        record_detail(request, f"(Information) press_diff always exactly 0 ({len(values)} msgs)")
        return
    nonfinite = sum(1 for v in values if not finite(v))
    biggest = max((abs(v) for v in values if finite(v)), default=0.0)
    record_field_fact(request, "press_diff", untestable="needs airspeed; at rest only finite and ~0")
    detail = (f"(Information) press_diff {min(values):+.4f}..{max(values):+.4f} hPa, {nonfinite} non-finite "
              f"({len(values)} msgs)")
    record_detail(request, detail)
    assert nonfinite == 0 and biggest <= PRESS_DIFF_REST_MAX, f"press_diff implausible at rest — {detail}"


async def temperatures(emission, system, request) -> None:
    w = await emission.sample_window(system)
    names = ["temperature", "temperature_press_diff"]
    if w.na:
        skip_untestable(request, names, "message not sent, or too few to analyse", w.na)
    parts, bad = [], []
    for n in names:
        values = [s.fields.get(n) for s in w.samples]
        if any(v is None for v in values):
            record_field_fact(request, n, untestable="extension not decoded")
            parts.append(f"{n} not decoded")
            continue
        if all(v == 0 for v in values):
            note = "Always 0 (not available)" if n == "temperature_press_diff" else "Always 0 (no invalid value defined)"
            record_field_fact(request, n, supported=False, notes=note)
            parts.append(f"{n} always 0")
            continue
        out = [v for v in values if v != 0 and not TEMP_RANGE[0] <= v <= TEMP_RANGE[1]]
        bad += [(n, v) for v in out]
        record_field_fact(request, n, supported=not out)
        parts.append(f"{n} {min(values) / 100:.2f}..{max(values) / 100:.2f} °C")
    record_detail(request, "; ".join(parts))
    if bad:
        report.compat_fail(f"temperature outside −40..85 °C: {bad[:3]}")


def define_field_tests(namespace: dict, prefix: str, emission) -> None:
    specs = [
        ("compat_boot_ms_tracks_real_time", lambda s, r: emission.boot_ms_tracks_real_time(s, r),
         "time_boot_ms never goes backwards or sticks, and advances at host-clock rate (±5%)."),
        ("compat_press_abs_matches_altitude", lambda s, r: press_abs_matches_altitude(emission, s, r),
         "press_abs is the ISA pressure (hPa) at GLOBAL_POSITION_INT's MSL altitude (±10 hPa)."),
        ("info_press_diff_at_rest", lambda s, r: press_diff_at_rest(emission, s, r),
         "(Information) At rest press_diff is finite and ~0 (≤1 hPa)."),
        ("compat_temperatures", lambda s, r: temperatures(emission, s, r),
         "temperature and temperature_press_diff are 0 (not available) or −40..85 °C."),
    ]
    for suffix, fn, doc in specs:
        def make(f):
            async def test(gcs_system, request):
                await f(gcs_system, request)
            return test
        t = make(fn)
        t.__name__ = t.__qualname__ = f"test_{prefix}_{suffix}"
        t.__doc__ = doc
        t.__module__ = namespace["__name__"]
        namespace[t.__name__] = t
