"""
FLIGHT_INFORMATION in flight (README.md in this directory; the shared flight is in ../flight.py).

The XML: "Takeoff and landing values reset to zero on arm"; takeoff_time_utc
"Timestamp at takeoff (since system boot)"; landing_time "Timestamp at landing
(in ms since system boot)". All have invalid 0. So:
- takeoff_time_utc, once airborne: 0 (not supported) or the takeoff time —
  between arming and 5 s after the vehicle passed 2 m (compat);
- landing_time: 0 while flying, then 0 (not supported) or the landing time —
  within 15 s after touchdown, or 5 s before (compat);
- arming again on the ground (after the landing): takeoff and landing reset to
  0 and arming_time_utc set to the new arming (compat).
Times are on the vehicle's boot clock (time_boot_ms of GLOBAL_POSITION_INT at
each event).
"""

from __future__ import annotations

import pytest

from tests import report
from tests.flight_helpers import require_real_stack  # noqa: F401 — autouse: flight tests need a real stack
from tests.messages_telemetry import flight as F
from tests.messages_telemetry.conftest import record_detail, record_field_fact, skip_untestable

_MSG_NAME = "FLIGHT_INFORMATION"
_MSG_ID = 264
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME,
                      ["time_boot_ms", "arming_time_utc", "takeoff_time_utc", "flight_uuid", "landing_time"])

pytestmark = F.PYTESTMARK

SETTLE_S = 3.0
TAKEOFF_WINDOW_MS = (-1000, 5000)    # around (arming, airborne at 2 m)
LANDING_WINDOW_MS = (-5000, 15000)   # around touchdown
ARM_WINDOW_MS = (-2000, 1000)


def _s(ms: float | None) -> str:
    return "?" if ms is None else f"{ms / 1000:.1f}s"


async def test_flight_information_compat_takeoff_time_set_at_takeoff(gcs_system, request, restart_flight_stack):
    """Once airborne, takeoff_time_utc is 0 (not supported) or the takeoff time (after arming, ≤ 5 s after leaving the ground)."""
    fl = await F.get_flight(gcs_system, restart_flight_stack)
    t_air, t_down = fl.need_flying(request, ["takeoff_time_utc"])
    samples = fl.s(_MSG_NAME, t_air + SETTLE_S, t_down)
    if len(samples) < 2:
        skip_untestable(request, ["takeoff_time_utc"], "too few FLIGHT_INFORMATION in flight", f"{len(samples)} received")
    arm_ms, air_ms = fl.boot_ms_at(fl.t("armed")), fl.boot_ms_at(t_air)
    values = sorted({int(s.fields["takeoff_time_utc"]) for s in samples})
    detail = (f"takeoff_time_utc in flight {[_s(v / 1000) for v in values]}; armed {_s(arm_ms)}, "
              f"≥ 2 m at {_s(air_ms)} (boot); {fl.label}")
    record_detail(request, detail)
    if values == [0]:
        record_field_fact(request, "takeoff_time_utc", supported=False, notes="Always 0 in flight")
        return
    bad = [v for v in values if not (arm_ms + TAKEOFF_WINDOW_MS[0] <= v / 1000 <= air_ms + TAKEOFF_WINDOW_MS[1])]
    record_field_fact(request, "takeoff_time_utc", supported=not bad)
    if bad:
        report.compat_fail(f"takeoff_time_utc isn't the takeoff time — {detail}")


async def test_flight_information_compat_landing_time_set_at_landing(gcs_system, request, restart_flight_stack):
    """landing_time is 0 in flight, then 0 (not supported) or the landing time (touchdown −5 s .. +15 s)."""
    fl = await F.get_flight(gcs_system, restart_flight_stack)
    t_down = fl.need_touchdown(request, ["landing_time"])
    t_air = fl.t("airborne")
    flying = fl.s(_MSG_NAME, t_air + SETTLE_S, t_down - 5.0)
    after = fl.s(_MSG_NAME, t_down + SETTLE_S, fl.t("rearmed") or fl.t("end"))
    if any(s.fields.get("landing_time") is None for s in flying + after):
        skip_untestable(request, ["landing_time"], "extension not decoded")
    if not after:
        skip_untestable(request, ["landing_time"], "no FLIGHT_INFORMATION after landing")
    down_ms = fl.boot_ms_at(t_down)
    in_flight = sorted({int(s.fields["landing_time"]) for s in flying})
    landed = sorted({int(s.fields["landing_time"]) for s in after})
    detail = (f"landing_time in flight {[_s(v) for v in in_flight]}, after touchdown {[_s(v) for v in landed]}; "
              f"touchdown {_s(down_ms)} (boot); {fl.label}")
    record_detail(request, detail)
    errors = []
    if any(in_flight):
        errors.append("non-zero while flying (should be reset on arming)")
    nonzero = [v for v in landed if v]
    if nonzero and any(not (down_ms + LANDING_WINDOW_MS[0] <= v <= down_ms + LANDING_WINDOW_MS[1]) for v in nonzero):
        errors.append("not the landing time")
    if errors:
        record_field_fact(request, "landing_time", supported=False)
        report.compat_fail(f"landing_time {'; '.join(errors)} — {detail}")
    if not nonzero:
        record_field_fact(request, "landing_time", supported=False, notes="Always 0 after landing")
    else:
        record_field_fact(request, "landing_time", supported=True)


async def test_flight_information_compat_times_reset_on_arm(gcs_system, request, restart_flight_stack):
    """Arming again after the landing resets takeoff and landing times to 0, and sets arming_time_utc to the new arming."""
    fl = await F.get_flight(gcs_system, restart_flight_stack)
    fl.need_touchdown(request, [])
    t_rearm = fl.t("rearmed")
    if t_rearm is None:
        pytest.skip("NA: the vehicle couldn't be armed again after landing")
    # Stop short of the first disarmed HEARTBEAT: at 1 Hz it can follow the disarm by up to a second.
    samples = fl.s(_MSG_NAME, t_rearm + 0.5, (fl.t("redisarmed") or fl.t("end")) - 1.0)
    if not samples:
        pytest.skip("NA: no FLIGHT_INFORMATION while armed again after landing")
    f = samples[-1].fields
    arm_ms = fl.boot_ms_at(t_rearm)
    takeoff, landing, arming = int(f["takeoff_time_utc"]), f.get("landing_time"), int(f["arming_time_utc"]) / 1000
    errors = []
    if takeoff:
        errors.append(f"takeoff_time_utc {_s(takeoff / 1000)} not reset")
    if landing:
        errors.append(f"landing_time {_s(landing)} not reset")
    if not (arm_ms + ARM_WINDOW_MS[0] <= arming <= arm_ms + ARM_WINDOW_MS[1]):
        errors.append(f"arming_time_utc {_s(arming)} isn't the new arming ({_s(arm_ms)})")
    detail = (f"armed again at {_s(arm_ms)}: arming {_s(arming)}, takeoff {_s(takeoff / 1000)}, landing "
              f"{_s(landing) if landing is not None else 'not decoded'}; {fl.label}")
    record_detail(request, detail)
    if errors:
        report.compat_fail(f"{'; '.join(errors)} — {detail}")


async def test_flight_information_obs_flight_uuid_across_flight(gcs_system, request, restart_flight_stack):
    """Characterise flight_uuid before, during and after the flight, and after arming again."""
    fl = await F.get_flight(gcs_system, restart_flight_stack)
    fl.need_moving(request, [])
    phases = {"before": (None, fl.t("armed")), "flying": (fl.t("armed"), fl.t("touchdown") or fl.t("arrived")),
              "landed": (fl.t("touchdown") or fl.t("arrived"), fl.t("rearmed")),
              "re-armed": (fl.t("rearmed"), None)}
    shown = {k: sorted({int(s.fields["flight_uuid"]) for s in fl.s(_MSG_NAME, a, b)}) for k, (a, b) in phases.items()
             if not (k == "re-armed" and a is None)}
    record_detail(request, f"flight_uuid {shown}; {fl.label}")
