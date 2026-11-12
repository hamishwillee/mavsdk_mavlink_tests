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

import pytest

from tests import report
from tests.messages_telemetry.conftest import finite, nearest, record_detail, record_field_fact, skip_untestable

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


# ---------------------------------------------------------------------------
# In flight (../flight.py): two cruise altitudes and airflow
# ---------------------------------------------------------------------------

FLIGHT_DP_TOLERANCE_HPA = 0.5   # or 10% of the expected change (ISA vs the simulator's own atmosphere)
FLIGHT_WITHIN = 0.9
FLIGHT_NONTRIVIAL_CLIMB_M = 15.0
FLIGHT_MIN_AIRSPEED = 8.0       # m/s: q ≈ 0.4 hPa — a real airspeed sensor can't read 0
FLIGHT_Q_RATIO = (0.5, 2.0)     # press_diff / ½ρv²: a unit error (Pa, kPa, mbar×100) is far outside


def _gpi_alt_at(fl, boot_ms):
    g = fl.boot("GLOBAL_POSITION_INT").at(boot_ms, 100)
    return None if g is None else g.fields["alt"] / 1000.0


async def press_abs_tracks_altitude(fl, name, request) -> None:
    from tests.messages_telemetry import flight as F
    t0, t1 = fl.need_moving(request, ["press_abs"])
    before = [s for s in fl.s(name, None, t0) if s.fields["press_abs"]]
    during = [s for s in fl.s(name, t0, t1) if s.fields["press_abs"]]
    if len(before) < 5 or len(during) < 20:
        skip_untestable(request, ["press_abs"], "message not sent (no such barometer), or too few",
                        f"{len(before)} before arming, {len(during)} in flight")
    ref = [(s.fields["press_abs"], _gpi_alt_at(fl, s.fields["time_boot_ms"])) for s in before]
    ref = [(p, a) for p, a in ref if a is not None]
    if not ref:
        skip_untestable(request, ["press_abs"], "no GLOBAL_POSITION_INT altitude to compare against")
    p0, alt0 = statistics.fmean(p for p, _ in ref), statistics.fmean(a for _, a in ref)
    flags, by_alt = [], {}
    worst = 0.0
    for s in during:
        alt = _gpi_alt_at(fl, s.fields["time_boot_ms"])
        if alt is None:
            continue
        expected = isa_pressure_hpa(alt) - isa_pressure_hpa(alt0)
        measured = s.fields["press_abs"] - p0
        err = abs(measured - expected)
        worst = max(worst, err)
        flags.append(err <= max(FLIGHT_DP_TOLERANCE_HPA, 0.1 * abs(expected)))
        by_alt.setdefault(round((alt - alt0) / 10) * 10, []).append(measured)
    climb = max(by_alt) - min(by_alt) if by_alt else 0
    ok = F.fraction(flags) >= FLIGHT_WITHIN
    if not ok:
        record_field_fact(request, "press_abs", supported=False)
    elif climb >= FLIGHT_NONTRIVIAL_CLIMB_M:
        record_field_fact(request, "press_abs", supported=True)
    else:
        record_field_fact(request, "press_abs", untestable=f"altitude changed only ~{climb} m in flight")
    levels = ", ".join(f"{h:+d} m: {statistics.fmean(v):+.2f} hPa" for h, v in sorted(by_alt.items())
                       if len(v) >= 5 and h in (0, 30, 50))
    detail = (f"(Information) change from the ground ({p0:.2f} hPa at {alt0:.1f} m) vs ISA: "
              f"{100 * F.fraction(flags):.0f}% within max({FLIGHT_DP_TOLERANCE_HPA} hPa, 10%) (worst {worst:.2f}); "
              f"{levels or 'no cruise levels'}; n={len(flags)}; {fl.label}")
    record_detail(request, detail)
    assert ok, f"press_abs doesn't follow altitude as hPa — {detail}"


async def press_diff_tracks_airspeed(fl, name, request) -> None:
    t0, t1 = fl.need_moving(request, ["press_diff"])
    samples = fl.s(name, t0, t1)
    vfr = fl.s("VFR_HUD", t0, t1)
    if len(samples) < 20 or not vfr:
        skip_untestable(request, ["press_diff"], "message not sent (no such sensor), or too few",
                        f"{len(samples)} {name}, {len(vfr)} VFR_HUD in flight")
    # VFR_HUD airspeed is NaN (None here) without an airspeed estimate (PX4 multicopter): then use the
    # ground speed — neither simulator has wind by default — and say so.
    gpi = fl.boot("GLOBAL_POSITION_INT")
    fast, speeds, from_ground = [], [], 0
    for s in samples:
        v = nearest(vfr, s.host_mono)
        if abs(v.host_mono - s.host_mono) > 0.3:
            continue
        airspeed = v.fields["airspeed"]
        if not finite(airspeed):
            g = gpi.at(s.fields["time_boot_ms"], 100)
            if g is None:
                continue
            airspeed = math.hypot(g.fields["vx"], g.fields["vy"]) / 100.0
            from_ground += 1
        speeds.append(airspeed)
        if airspeed >= FLIGHT_MIN_AIRSPEED:
            alt = _gpi_alt_at(fl, s.fields["time_boot_ms"]) or 0.0
            rho = 1.225 * isa_pressure_hpa(alt) / 1013.25
            fast.append((s.fields["press_diff"], 0.5 * rho * airspeed ** 2 / 100.0, airspeed))
    top = max(speeds, default=0.0)
    source = "VFR_HUD airspeed" if not from_ground else "ground speed (VFR_HUD airspeed unknown; no simulated wind)"
    if len(fast) < 10:
        record_field_fact(request, "press_diff", untestable=f"flew at most {top:.1f} m/s ({source})")
        pytest.skip(f"NA: {source} reached only {top:.1f} m/s — need ≥ {FLIGHT_MIN_AIRSPEED:.0f} m/s")
    if all(p == 0 for p, _, _ in fast):
        # No differential-pressure sensor on this vehicle (PX4 SIH publishes an airspeed directly, with no
        # sensor behind it): that says nothing about whether the stack fills the field when one exists.
        record_field_fact(request, "press_diff",
                          untestable=f"always 0 in flight at up to {top:.0f} m/s: no airspeed sensor on this vehicle")
        record_detail(request, f"always exactly 0 at {FLIGHT_MIN_AIRSPEED:.0f}–{top:.0f} m/s ({source}) "
                               f"({len(fast)} msgs): no airspeed sensor on this vehicle; {fl.label}")
        return
    ratios = sorted(p / q for p, q, _ in fast if q > 0)
    median = ratios[len(ratios) // 2]
    ok = FLIGHT_Q_RATIO[0] <= median <= FLIGHT_Q_RATIO[1]
    record_field_fact(request, "press_diff", supported=ok)
    detail = (f"press_diff / ½ρv² (v = {source} {FLIGHT_MIN_AIRSPEED:.0f}–{top:.0f} m/s): median {median:.2f}, "
              f"{ratios[0]:.2f}..{ratios[-1]:.2f} (n={len(ratios)}); {fl.label}")
    record_detail(request, detail)
    if not ok:
        report.compat_fail(f"press_diff isn't the dynamic pressure in hPa — {detail}")


def define_flight_tests(namespace: dict, prefix: str, name: str) -> None:
    """The pressure flight tests, named test_<prefix>_... (rule 8)."""
    from tests.messages_telemetry import flight as F
    specs = [
        ("info_press_abs_tracks_altitude_in_flight", press_abs_tracks_altitude,
         "(Information) In flight press_abs changes from its ground value as ISA does with altitude (±0.5 hPa or 10%)."),
        ("compat_press_diff_tracks_airspeed_in_flight", press_diff_tracks_airspeed,
         "In flight at ≥ 8 m/s airspeed, press_diff is 0 (no sensor: untestable) or the dynamic pressure ½ρv² in hPa (×0.5–2)."),
    ]
    for suffix, fn, doc in specs:
        def make(f):
            async def test(gcs_system, request, restart_flight_stack):
                fl = await F.get_flight(gcs_system, restart_flight_stack)
                await f(fl, name, request)
            return test
        t = make(fn)
        t.__name__ = t.__qualname__ = f"test_{prefix}_{suffix}"
        t.__doc__ = doc
        t.__module__ = namespace["__name__"]
        namespace[t.__name__] = t
