"""
Field checks shared by GPS_RAW_INT and GPS2_RAW (README.md in this directory
has the reasoning and results).

The XML gives sentinels for most "unknown" values — eph/epv/vel/cog
UINT16_MAX, satellites_visible UINT8_MAX, yaw 0 ("doesn't provide yaw"),
UINT16_MAX ("configured but unable"), 36000 for north — and none for
alt_ellipsoid or the *_acc extensions. With the vehicle at rest:

- fix_type is a GPS_FIX_TYPE value (compat); lat/lon/alt match
  GLOBAL_POSITION_INT (±10 m) — info, a GPS isn't the estimate.
- eph/epv, vel, cog, satellites_visible and yaw stay inside their defined
  ranges or sentinels (compat); what's sent is characterised. vel/cog need
  movement to be shown right (untestable).
- The accuracy extensions and alt_ellipsoid are characterised: with no
  invalid value, 0 can't say "unknown".
"""

from __future__ import annotations

import math

from tests import report
from tests.messages_telemetry.conftest import (
    classify_time_usec,
    distinct,
    ned_offset_m,
    nearest,
    record_detail,
    record_field_fact,
    skip_untestable,
)

FIELDS = ["time_usec", "fix_type", "lat", "lon", "alt", "eph", "epv", "vel", "cog", "satellites_visible",
          "alt_ellipsoid", "h_acc", "v_acc", "vel_acc", "hdg_acc", "yaw"]
GPS2_EXTRA = ["dgps_numch", "dgps_age"]
UINT16_MAX = 65535
UINT8_MAX = 255
FIX_TYPES = {0: "NO_GPS", 1: "NO_FIX", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED",
             7: "STATIC", 8: "PPP"}
POSITION_TOLERANCE_M = 10.0
REST_VEL_MAX = 50              # cm/s
GEOID_RANGE_M = (-110.0, 90.0)  # EGM96 undulation extremes
ACC_MAX_MM = 100_000


def _na(request, w, fields):
    if w.na:
        skip_untestable(request, fields, "message not sent (no such receiver), or too few to analyse", w.na)


async def time_usec(emission, system, request) -> None:
    w = await emission.sample_window(system)
    _na(request, w, ["time_usec"])
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


async def fix_and_position(emission, system, request) -> None:
    w = await emission.sample_window(system)
    _na(request, w, ["fix_type", "lat", "lon", "alt"])
    fixes = [int(s.fields["fix_type"]) for s in w.samples]
    unknown = [f for f in fixes if f not in FIX_TYPES]
    fix_desc = distinct([FIX_TYPES.get(f, f) for f in fixes])
    if unknown:
        record_field_fact(request, "fix_type", supported=False)
        record_detail(request, f"fix_type {fix_desc}")
        report.compat_fail(f"fix_type not a GPS_FIX_TYPE value: {sorted(set(unknown))}")
    with_fix = [s for s in w.samples if s.fields["fix_type"] >= 3]
    if not with_fix:
        record_field_fact(request, "fix_type", supported=True, notes=f"Only {fix_desc} at rest")
        skip_untestable(request, ["lat", "lon", "alt"], "no 3D fix", f"fix_type {fix_desc}")
    record_field_fact(request, "fix_type", supported=True)
    gpi = w.others.get("GLOBAL_POSITION_INT") or []
    if not gpi:
        skip_untestable(request, ["lat", "lon", "alt"], "no GLOBAL_POSITION_INT to compare against")
    worst_h = worst_v = 0.0
    for s in with_fix:
        g = nearest(gpi, s.host_mono).fields
        n, e = ned_offset_m(g["lat"], g["lon"], s.fields["lat"], s.fields["lon"])
        worst_h = max(worst_h, math.hypot(n, e))
        worst_v = max(worst_v, abs(s.fields["alt"] - g["alt"]) / 1000)
    ok = worst_h <= POSITION_TOLERANCE_M and worst_v <= POSITION_TOLERANCE_M
    for f in ("lat", "lon", "alt"):
        record_field_fact(request, f, supported=ok)
    detail = (f"fix_type {fix_desc}; vs GLOBAL_POSITION_INT worst {worst_h:.2f} m horizontal, {worst_v:.2f} m "
              f"vertical ({len(with_fix)} msgs with 3D fix)")
    record_detail(request, detail)
    assert ok, f"(Information) GPS position far from the estimate — {detail}"


async def ranges_and_sentinels(emission, system, request) -> None:
    """eph/epv, vel, cog, satellites_visible, yaw: inside the defined range or sentinel."""
    names = ["eph", "epv", "vel", "cog", "satellites_visible", "yaw"]
    w = await emission.sample_window(system)
    _na(request, w, names)
    vals = {n: [int(s.fields[n]) for s in w.samples] for n in names}
    errors = []
    if any(c != UINT16_MAX and not 0 <= c <= 35999 for c in vals["cog"]):
        errors.append(f"cog {distinct(vals['cog'])} (0..35999 or UINT16_MAX)")
    if any(y not in (0, UINT16_MAX) and not 1 <= y <= 36000 for y in vals["yaw"]):
        errors.append(f"yaw {distinct(vals['yaw'])} (0, 1..36000 or UINT16_MAX)")
    for n in ("eph", "epv"):
        if any(v == 0 for v in vals[n]):
            errors.append(f"{n} 0: a DOP can't be 0 (UINT16_MAX is 'unknown')")
    parts = [f"{n} {distinct(v)}" for n, v in vals.items()]
    record_detail(request, "; ".join(parts))
    for n in ("eph", "epv", "satellites_visible"):
        sentinel = UINT8_MAX if n == "satellites_visible" else UINT16_MAX
        if all(v == sentinel for v in vals[n]):
            record_field_fact(request, n, supported=False, notes="Always unknown")
        else:
            record_field_fact(request, n, supported=not any(e.startswith(n) for e in errors))
    for n in ("vel", "cog"):
        if any(e.startswith(n) for e in errors):
            record_field_fact(request, n, supported=False)
        elif all(v == UINT16_MAX for v in vals[n]):
            record_field_fact(request, n, supported=False, notes="Always unknown")
        else:
            record_field_fact(request, n, untestable="needs movement; at rest only in range")
    if any(e.startswith("yaw") for e in errors):
        record_field_fact(request, "yaw", supported=False)
    elif all(v == 0 for v in vals["yaw"]):
        record_field_fact(request, "yaw", untestable="0: this GPS doesn't provide yaw")
    else:
        record_field_fact(request, "yaw", supported=True, notes=f"Values {distinct(vals['yaw'])}")
    if errors:
        report.compat_fail("; ".join(errors))


async def extensions(emission, system, request) -> None:
    """Characterise alt_ellipsoid and the accuracy fields (no invalid values defined)."""
    names = ["alt_ellipsoid", "h_acc", "v_acc", "vel_acc", "hdg_acc"]
    w = await emission.sample_window(system)
    _na(request, w, names)
    if any(s.fields.get("h_acc") is None for s in w.samples):
        skip_untestable(request, names, "extensions not decoded")
    parts = []
    sep = [(s.fields["alt_ellipsoid"] - s.fields["alt"]) / 1000 for s in w.samples if s.fields["alt_ellipsoid"]]
    if not sep:
        record_field_fact(request, "alt_ellipsoid", supported=False, notes="Always 0")
        parts.append("alt_ellipsoid always 0")
    else:
        ok = all(GEOID_RANGE_M[0] <= x <= GEOID_RANGE_M[1] for x in sep)
        same = all(x == 0 for x in sep)
        record_field_fact(request, "alt_ellipsoid", supported=ok,
                          notes="Equal to alt (no geoid separation)" if same else None)
        parts.append(f"alt_ellipsoid − alt {min(sep):+.2f}..{max(sep):+.2f} m")
    for n in ("h_acc", "v_acc", "vel_acc", "hdg_acc"):
        values = [int(s.fields[n]) for s in w.samples]
        if all(v == 0 for v in values):
            if n == "hdg_acc" and not any(s.fields["yaw"] for s in w.samples):
                record_field_fact(request, n, untestable="0, and this GPS doesn't provide yaw")
            else:
                record_field_fact(request, n, supported=False, notes="Always 0 (no invalid value defined)")
            parts.append(f"{n} always 0")
        else:
            record_field_fact(request, n, supported=all(0 < v <= ACC_MAX_MM for v in values if v))
            parts.append(f"{n} {distinct(values)}")
    record_detail(request, "; ".join(parts))


async def dgps(emission, system, request) -> None:
    w = await emission.sample_window(system)
    _na(request, w, GPS2_EXTRA)
    parts = []
    for n in GPS2_EXTRA:
        values = [int(s.fields[n]) for s in w.samples]
        if all(v == 0 for v in values):
            record_field_fact(request, n, untestable="0: no DGPS in use")
        else:
            record_field_fact(request, n, supported=True)
        parts.append(f"{n} {distinct(values)}")
    record_detail(request, "; ".join(parts))


def define_field_tests(namespace: dict, prefix: str, emission, gps2: bool = False) -> None:
    specs = [
        ("obs_time_usec", lambda s, r: time_usec(emission, s, r),
         "Characterise time_usec (UNIX or boot); fail only if it's neither."),
        ("info_fix_and_position", lambda s, r: fix_and_position(emission, s, r),
         "(Information) fix_type is a GPS_FIX_TYPE; with a 3D fix lat/lon/alt match GLOBAL_POSITION_INT (±10 m)."),
        ("compat_ranges_and_sentinels", lambda s, r: ranges_and_sentinels(emission, s, r),
         "eph/epv/vel/cog/satellites_visible/yaw are inside their defined ranges or sentinels."),
        ("obs_extensions", lambda s, r: extensions(emission, s, r),
         "Characterise alt_ellipsoid and the accuracy extensions (no invalid values defined)."),
    ]
    if gps2:
        specs.append(("obs_dgps", lambda s, r: dgps(emission, s, r), "Characterise dgps_numch/dgps_age."))
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
