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
