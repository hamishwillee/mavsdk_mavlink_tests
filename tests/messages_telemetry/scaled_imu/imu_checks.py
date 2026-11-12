"""
Field checks shared by SCALED_IMU, SCALED_IMU2 and SCALED_IMU3 (identical
fields; README.md in this directory has the reasoning and results).

The XML gives units (mG, mrad/s, mgauss, cdegC) but no frame and no invalid
values except temperature's 0 ("IMU does not provide temperature values"; 0 °C
is sent as 1). With the vehicle at rest:

- The accelerometer measures specific force: (0, 0, -1 g) in the earth frame,
  rotated into the body frame by ATTITUDE's roll/pitch — so
  x = g·sinθ, y = −g·sinφ·cosθ, z = −g·cosφ·cosθ (±60 mG; |a| within 5%).
  Compat: a wrong unit or sign contradicts "X/Y/Z acceleration" in mG. A level
  vehicle has x/y ≈ 0 whichever way round they are, so those are recorded
  untestable unless the expected value is large enough to show.
- Gyro: small at rest; axes need rotation (untestable).
- Magnetometer: Earth's field (150–700 mgauss), rotated to the earth frame
  with ATTITUDE, points down in the northern hemisphere (inclination 30–80°)
  and roughly north (declination within ±25°) — info, as calibration and
  declination vary. Per-axis verdicts only where the component is large.
- An instance whose accel/gyro/mag are all exactly 0 in every message is "not
  provided" (the XML has no way to say so), not a wrong value.
- temperature: 0 = not provided; otherwise −40..85 °C (compat).
"""

from __future__ import annotations

import bisect
import math
import statistics

import pytest

from tests import report
from tests.messages_telemetry.conftest import finite, nearest, record_detail, record_field_fact, skip_untestable

ACC = ["xacc", "yacc", "zacc"]
GYRO = ["xgyro", "ygyro", "zgyro"]
MAG = ["xmag", "ymag", "zmag"]
FIELDS = ["time_boot_ms", *ACC, *GYRO, *MAG, "temperature"]

ACC_AXIS_TOLERANCE_MG = 60.0
ACC_NORM_TOLERANCE = 0.05
NONTRIVIAL_MG = 100.0
GYRO_REST_MAX = 50.0          # mrad/s
MAG_RANGE = (150.0, 700.0)    # mgauss
INCLINATION_RANGE = (30.0, 80.0)
DECLINATION_MAX = 25.0
NONTRIVIAL_MGAUSS = 100.0
TEMP_RANGE = (-4000, 8500)    # cdegC


def _all_zero(samples, names) -> bool:
    return all(s.fields[n] == 0 for s in samples for n in names)


def _mean(samples, name) -> float:
    return statistics.fmean(s.fields[name] for s in samples)


async def accel_is_gravity(emission, system, request) -> None:
    w = await emission.sample_window(system)
    if w.na:
        skip_untestable(request, ACC, "message not sent, or too few to analyse", w.na)
    if _all_zero(w.samples, ACC):
        for f in ACC:
            record_field_fact(request, f, supported=False, notes="Always 0 (no accelerometer on this instance)")
        record_detail(request, f"accel always exactly 0 ({len(w.samples)} msgs): not provided")
        return
    att = w.others.get("ATTITUDE") or []
    if not att:
        skip_untestable(request, ACC, "no ATTITUDE to compare against")
    a = nearest(att, w.samples[-1].host_mono).fields
    roll, pitch = a["roll"], a["pitch"]
    expected = [1000 * math.sin(pitch), -1000 * math.sin(roll) * math.cos(pitch),
                -1000 * math.cos(roll) * math.cos(pitch)]
    mean = [_mean(w.samples, f) for f in ACC]
    norm = math.sqrt(sum(m * m for m in mean))
    diffs = [abs(m - e) for m, e in zip(mean, expected)]
    ok = all(d <= ACC_AXIS_TOLERANCE_MG for d in diffs) and abs(norm / 1000 - 1) <= ACC_NORM_TOLERANCE
    for f, e in zip(ACC, expected):
        if not ok:
            record_field_fact(request, f, supported=False)
        elif abs(e) >= NONTRIVIAL_MG:
            record_field_fact(request, f, supported=True)
        else:
            record_field_fact(request, f, untestable=f"expected {e:+.0f} mG at this attitude: ~0 either way; needs tilt")
    detail = (f"mean acc {mean[0]:+.0f}/{mean[1]:+.0f}/{mean[2]:+.0f} mG (|a| {norm:.0f}); expected from ATTITUDE "
              f"r/p {math.degrees(roll):+.1f}/{math.degrees(pitch):+.1f}°: {expected[0]:+.0f}/{expected[1]:+.0f}/"
              f"{expected[2]:+.0f} ({len(w.samples)} msgs)")
    record_detail(request, detail)
    if not ok:
        report.compat_fail(f"acceleration isn't specific force in mG in the body frame — {detail}")


async def gyro_at_rest(emission, system, request) -> None:
    w = await emission.sample_window(system)
    if w.na:
        skip_untestable(request, GYRO, "message not sent, or too few to analyse", w.na)
    if _all_zero(w.samples, GYRO):
        for f in GYRO:
            record_field_fact(request, f, supported=False, notes="Always 0 (no gyro on this instance)")
        record_detail(request, f"gyro always exactly 0 ({len(w.samples)} msgs): not provided")
        return
    biggest = max(abs(s.fields[f]) for s in w.samples for f in GYRO)
    for f in GYRO:
        record_field_fact(request, f, untestable="needs rotation; at rest only small")
    detail = f"(Information) max |gyro| {biggest} mrad/s at rest ({len(w.samples)} msgs)"
    record_detail(request, detail)
    assert biggest <= GYRO_REST_MAX, f"gyro implausible at rest — {detail}"


async def mag_is_earth_field(emission, system, request) -> None:
    w = await emission.sample_window(system)
    if w.na:
        skip_untestable(request, MAG, "message not sent, or too few to analyse", w.na)
    if _all_zero(w.samples, MAG):
        for f in MAG:
            record_field_fact(request, f, supported=False, notes="Always 0 (no magnetometer on this instance)")
        record_detail(request, f"mag always exactly 0 ({len(w.samples)} msgs): not provided")
        return
    att = w.others.get("ATTITUDE") or []
    gpi = w.others.get("GLOBAL_POSITION_INT") or []
    if not att:
        skip_untestable(request, MAG, "no ATTITUDE to compare against")
    b = [_mean(w.samples, f) for f in MAG]
    a = nearest(att, w.samples[-1].host_mono).fields
    r, p, y = a["roll"], a["pitch"], a["yaw"]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    # body -> NED (R = Rz(y) Ry(p) Rx(r))
    rot = [[cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
           [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
           [-sp, sr * cp, cr * cp]]
    n, e, d = (sum(rot[i][j] * b[j] for j in range(3)) for i in range(3))
    field = math.sqrt(n * n + e * e + d * d)
    incl = math.degrees(math.atan2(d, math.hypot(n, e)))
    decl = math.degrees(math.atan2(e, n))
    north = (gpi[-1].fields["lat"] >= 0) if gpi else True
    incl_ok = INCLINATION_RANGE[0] <= (incl if north else -incl) <= INCLINATION_RANGE[1]
    ok = MAG_RANGE[0] <= field <= MAG_RANGE[1] and incl_ok and abs(decl) <= DECLINATION_MAX
    for f, v in zip(MAG, b):
        if not ok:
            record_field_fact(request, f, supported=False)
        elif abs(v) >= NONTRIVIAL_MGAUSS:
            record_field_fact(request, f, supported=True)
        else:
            record_field_fact(request, f, untestable=f"{v:+.0f} mgauss at this heading: ~0 either way")
    detail = (f"(Information) mean mag {b[0]:+.0f}/{b[1]:+.0f}/{b[2]:+.0f} mgauss; in NED |B| {field:.0f}, "
              f"inclination {incl:+.1f}°, declination {decl:+.1f}° (yaw {math.degrees(y) % 360:.1f}°)")
    record_detail(request, detail)
    assert ok, f"magnetometer isn't Earth's field in mgauss, body frame — {detail}"


async def temperature(emission, system, request) -> None:
    w = await emission.sample_window(system)
    if w.na:
        skip_untestable(request, ["temperature"], "message not sent, or too few to analyse", w.na)
    values = [s.fields.get("temperature") for s in w.samples]
    if any(v is None for v in values):
        skip_untestable(request, ["temperature"], "extension not decoded")
    if all(v == 0 for v in values):
        record_field_fact(request, "temperature", supported=False, notes="Always 0 (not provided)")
        record_detail(request, "always 0: not provided")
        return
    bad = [v for v in values if v != 0 and not TEMP_RANGE[0] <= v <= TEMP_RANGE[1]]
    record_field_fact(request, "temperature", supported=not bad)
    record_detail(request, f"{min(values) / 100:.2f}..{max(values) / 100:.2f} °C ({len(values)} msgs)")
    if bad:
        report.compat_fail(f"temperature outside −40..85 °C: {bad[:3]} cdegC")


def define_field_tests(namespace: dict, prefix: str, emission) -> None:
    """The IMU field tests, named test_<prefix>_... (rule 8)."""
    specs = [
        ("compat_boot_ms_tracks_real_time", lambda s, r: emission.boot_ms_tracks_real_time(s, r),
         "time_boot_ms never goes backwards or sticks, and advances at host-clock rate (±5%)."),
        ("compat_accel_is_gravity", lambda s, r: accel_is_gravity(emission, s, r),
         "At rest acceleration is specific force: −1 g rotated into the body frame by ATTITUDE (±60 mG)."),
        ("info_gyro_at_rest", lambda s, r: gyro_at_rest(emission, s, r),
         "(Information) At rest the gyro reads < 50 mrad/s on every axis."),
        ("info_mag_is_earth_field", lambda s, r: mag_is_earth_field(emission, s, r),
         "(Information) The magnetometer reads Earth's field: 150–700 mgauss, dipping down, roughly north."),
        ("compat_temperature", lambda s, r: temperature(emission, s, r),
         "temperature is 0 (not provided) or −40..85 °C."),
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
# In flight (../flight.py): rotation, acceleration and several headings
# ---------------------------------------------------------------------------

_EULER = ("roll", "pitch", "yaw")
FLIGHT_RATE_MIN = 0.15          # rad/s: an ATTITUDE body rate at least this big is compared
# mrad/s, or 30% of the rate: ATTITUDE's rates are filtered and sampled at other instants, which in a
# PX4 tailsitter's pitch oscillation differs by ~100 mrad/s (2026-11-12). A unit (deg/s) or sign error is far outside.
FLIGHT_GYRO_TOLERANCE = 100.0
FLIGHT_GYRO_RELATIVE = 0.3
FLIGHT_ACC_TOLERANCE_MG = 120.0
FLIGHT_ACC_WINDOW_MS = 500      # velocity change taken over this long
FLIGHT_ACC_NONTRIVIAL_MG = 150.0
FLIGHT_MAG_NONTRIVIAL = 150.0   # mgauss an axis must vary by across the flight
FLIGHT_DECL_SPREAD = 20.0       # degrees: the Earth-frame field direction across headings
FLIGHT_INCL_SPREAD = 10.0
FLIGHT_NORM_SPREAD = 0.2
FLIGHT_MIN_N = 5
FLIGHT_WITHIN = 0.9


def _flight_imu(fl, name, request, fields, t0, t1):
    samples = fl.s(name, t0, t1)
    if len(samples) < 20:
        skip_untestable(request, fields, "message not sent in flight (no such sensor), or too few",
                        f"{len(samples)} {name} in flight")
    if _all_zero(samples, fields):
        for f in fields:
            record_field_fact(request, f, supported=False, notes="Always 0 (not provided on this instance)")
        record_detail(request, f"{', '.join(fields)} always exactly 0 in flight ({len(samples)} msgs): not provided")
        pytest.skip("NA: not provided on this instance (always exactly 0)")
    return samples


async def gyro_matches_body_rates(fl, name, request) -> None:
    from tests.messages_telemetry import flight as F
    t0, t1 = fl.need_moving(request, GYRO)
    samples = _flight_imu(fl, name, request, GYRO, t0, t1)
    att = fl.boot("ATTITUDE")
    rates = ["rollspeed", "pitchspeed", "yawspeed"]
    flags = [[], [], []]
    signs = [set(), set(), set()]
    for s in samples:
        a = att.interp(s.fields["time_boot_ms"], tuple(rates))
        if a is None:
            continue
        for i, (g, r) in enumerate(zip(GYRO, rates)):
            rate = a[r]
            if finite(rate) and abs(rate) >= FLIGHT_RATE_MIN:
                expected = 1000.0 * rate
                flags[i].append(math.copysign(1, s.fields[g]) == math.copysign(1, rate)
                                and abs(s.fields[g] - expected) <= max(FLIGHT_GYRO_TOLERANCE, FLIGHT_GYRO_RELATIVE * abs(expected)))
                signs[i].add("+" if rate > 0 else "−")
    failed = []
    for i, f in enumerate(GYRO):
        if len(flags[i]) < FLIGHT_MIN_N:
            record_field_fact(request, f, untestable=f"in flight only {len(flags[i])} samples rotating ≥ "
                                                     f"{FLIGHT_RATE_MIN} rad/s about this axis")
        elif F.fraction(flags[i]) < FLIGHT_WITHIN:
            record_field_fact(request, f, supported=False)
            failed.append(f)
        else:
            record_field_fact(request, f, supported=True)
    detail = ("(Information) vs ATTITUDE body rates ≥ " f"{FLIGHT_RATE_MIN} rad/s: " + "; ".join(
        f"{g} {100 * F.fraction(x):.0f}% of {len(x)} (signs {''.join(sorted(sg)) or '-'})"
        for g, x, sg in zip(GYRO, flags, signs)) + f" within max({FLIGHT_GYRO_TOLERANCE:.0f} mrad/s, {FLIGHT_GYRO_RELATIVE:.0%}); {fl.label}")
    record_detail(request, detail)
    assert not failed, f"gyro isn't the body angular rate in mrad/s — {detail}"


async def accel_is_specific_force(fl, name, request) -> None:
    from tests.messages_telemetry import flight as F
    t0, t1 = fl.need_moving(request, ACC)
    samples = sorted(_flight_imu(fl, name, request, ACC, t0, t1), key=lambda s: s.fields["time_boot_ms"])
    gpi = sorted(fl.s("GLOBAL_POSITION_INT", t0, t1), key=lambda s: s.fields["time_boot_ms"])
    att = fl.boot("ATTITUDE")
    gidx = F.BootIndex(gpi)
    keys = [s.fields["time_boot_ms"] for s in samples]
    flags = [[], [], []]
    big = [0, 0, 0]
    worst = [0.0, 0.0, 0.0]
    for g0 in gpi[::3]:
        b0 = g0.fields["time_boot_ms"]
        g1 = gidx.at(b0 + FLIGHT_ACC_WINDOW_MS, 100)
        b1 = g1.fields["time_boot_ms"] if g1 is not None else None
        a = None if b1 is None else att.interp((b0 + b1) / 2, _EULER, angles=_EULER)
        if a is None or g1 is g0:
            continue
        dt = (b1 - b0) / 1000.0
        acc_ned = [(g1.fields[k] - g0.fields[k]) / 100.0 / dt for k in ("vx", "vy", "vz")]
        force_ned = [acc_ned[0], acc_ned[1], acc_ned[2] - F.GRAVITY]
        rot = F.body_to_ned(a["roll"], a["pitch"], a["yaw"])
        expected = [1000.0 * c / F.GRAVITY for c in F.mat_vec(rot, force_ned, transpose=True)]
        lo, hi = bisect.bisect_left(keys, b0), bisect.bisect_right(keys, b1)
        if hi - lo < 2:
            continue
        measured = [statistics.fmean(s.fields[f] for s in samples[lo:hi]) for f in ACC]
        for i in range(3):
            err = abs(measured[i] - expected[i])
            worst[i] = max(worst[i], err)
            flags[i].append(err <= FLIGHT_ACC_TOLERANCE_MG)
            big[i] += abs(expected[i]) >= FLIGHT_ACC_NONTRIVIAL_MG
    if not flags[0]:
        skip_untestable(request, ACC, "no GLOBAL_POSITION_INT/ATTITUDE to derive the motion from")
    failed = []
    for i, f in enumerate(ACC):
        if F.fraction(flags[i]) < FLIGHT_WITHIN:
            record_field_fact(request, f, supported=False)
            failed.append(f)
        elif big[i] >= 3:
            record_field_fact(request, f, supported=True)
        else:
            record_field_fact(request, f, untestable=f"in flight expected ≥ {FLIGHT_ACC_NONTRIVIAL_MG:.0f} mG "
                                                     f"on this axis only {big[i]} times")
    detail = (f"vs specific force from GLOBAL_POSITION_INT's velocity change over {FLIGHT_ACC_WINDOW_MS} ms, "
              f"rotated by ATTITUDE: " + "; ".join(
                  f"{f} {100 * F.fraction(x):.0f}% within {FLIGHT_ACC_TOLERANCE_MG:.0f} mG (worst {w:.0f}, "
                  f"{n} windows ≥ {FLIGHT_ACC_NONTRIVIAL_MG:.0f} mG)" for f, x, w, n in zip(ACC, flags, worst, big))
              + f"; {len(flags[0])} windows; {fl.label}")
    record_detail(request, detail)
    if failed:
        report.compat_fail(f"{', '.join(failed)} isn't specific force in mG in the body frame — {detail}")


async def mag_constant_in_earth_frame(fl, name, request) -> None:
    from tests.messages_telemetry import flight as F
    t0, t1 = fl.need_moving(request, MAG)
    samples = _flight_imu(fl, name, request, MAG, t0, t1)
    att = fl.boot("ATTITUDE")
    bins: dict[int, list[tuple[float, float, float]]] = {}
    for s in samples:
        a = att.interp(s.fields["time_boot_ms"], _EULER, angles=_EULER)
        if a is None:
            continue
        rot = F.body_to_ned(a["roll"], a["pitch"], a["yaw"])
        n, e, d = F.mat_vec(rot, [s.fields[f] for f in MAG])
        heading = math.degrees(a["yaw"]) % 360.0
        bins.setdefault(int(heading // 45), []).append((n, e, d))
    stats = []  # (heading bin centre, declination, inclination, |B|)
    for b, vs in sorted(bins.items()):
        if len(vs) < FLIGHT_MIN_N:
            continue
        n, e, d = (statistics.fmean(v[i] for v in vs) for i in range(3))
        stats.append((b * 45 + 22.5, math.degrees(math.atan2(e, n)), math.degrees(math.atan2(d, math.hypot(n, e))),
                      math.sqrt(n * n + e * e + d * d)))
    spread = F.heading_spread([x[0] for x in stats])
    if len(stats) < 2 or spread < 90.0:
        skip_untestable(request, MAG, "flew too few headings to compare the field across",
                        f"{len(stats)} heading bins covering {spread:.0f}°")
    decl = [x[1] for x in stats]
    decl_spread = max(abs(F.angle_diff(a, b)) for a in decl for b in decl)
    incl_spread = max(x[2] for x in stats) - min(x[2] for x in stats)
    norms = [x[3] for x in stats]
    norm_spread = (max(norms) - min(norms)) / statistics.fmean(norms)
    ok = decl_spread <= FLIGHT_DECL_SPREAD and incl_spread <= FLIGHT_INCL_SPREAD and norm_spread <= FLIGHT_NORM_SPREAD
    for f in MAG:
        values = [s.fields[f] for s in samples]
        if not ok:
            record_field_fact(request, f, supported=False)
        elif max(values) - min(values) >= FLIGHT_MAG_NONTRIVIAL:
            record_field_fact(request, f, supported=True)
        else:
            record_field_fact(request, f, untestable=f"varied only {max(values) - min(values):.0f} mgauss in flight")
    detail = (f"(Information) Earth-frame field over {len(stats)} headings covering {spread:.0f}°: declination "
              f"{min(decl):+.0f}..{max(decl):+.0f}° (spread {decl_spread:.0f}°), inclination spread {incl_spread:.1f}°, "
              f"|B| {min(norms):.0f}..{max(norms):.0f} mgauss; {fl.label}")
    record_detail(request, detail)
    assert ok, f"magnetometer doesn't rotate with the body (field not fixed in the Earth frame) — {detail}"


def define_flight_tests(namespace: dict, prefix: str, name: str) -> None:
    """The IMU flight tests, named test_<prefix>_... (rule 8)."""
    from tests.messages_telemetry import flight as F
    specs = [
        ("info_gyro_matches_body_rates_in_flight", gyro_matches_body_rates,
         "(Information) In flight the gyro equals ATTITUDE's body rates in mrad/s, with the right sign, on each axis."),
        ("compat_accel_is_specific_force_in_flight", accel_is_specific_force,
         "In flight acceleration is specific force (velocity change − gravity) in the body frame, in mG (±120 mG)."),
        ("info_mag_constant_in_earth_frame_in_flight", mag_constant_in_earth_frame,
         "(Information) In flight the magnetometer, rotated by ATTITUDE, is the same Earth field on every heading."),
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
