"""
EXTENDED_SYS_STATE in flight (README.md in this directory; the shared flight is in ../flight.py).

MAV_LANDED_STATE: ON_GROUND "MAV is landed (on ground)", IN_AIR "MAV is in air",
TAKEOFF / LANDING "currently taking off / landing", UNDEFINED "unknown".
MAV_VTOL_STATE: MC, FW, and the two transitions; UNDEFINED "not configured as VTOL".

- landed_state: never ON_GROUND while ≥ 5 m above home (GLOBAL_POSITION_INT),
  never IN_AIR/TAKEOFF/LANDING while disarmed on the ground before or after —
  compat. Which of TAKEOFF/LANDING a stack uses, and when, is characterised.
- vtol_state (a VTOL only): while flying fast (ground speed ≥ 9 m/s for 10 s —
  faster than either stack's default multicopter mission speed, 5 m/s, and
  neither simulator has wind by default) never MC or UNDEFINED, and FW at some
  point (a transition may still be reported at speed: ArduPilot QuadPlane
  does); on the ground MC or FW (not UNDEFINED or a transition) — compat. Which of the two it reports on the ground depends on the mode
  (ArduPilot QuadPlane: FW outside a Q mode) and is characterised, as is the
  sequence of states. Anything else: characterised (the ground test
  already checks it against the HEARTBEAT type).
"""

from __future__ import annotations

import math
from collections import Counter

import pytest

from tests import report
from tests.flight_helpers import require_real_stack  # noqa: F401 — autouse: flight tests need a real stack
from tests.messages_telemetry import flight as F
from tests.messages_telemetry.conftest import nearest, record_detail, record_field_fact, skip_untestable

_MSG_NAME = "EXTENDED_SYS_STATE"
_MSG_ID = 245
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME, ["vtol_state", "landed_state"])

pytestmark = F.PYTESTMARK

VTOL_STATES = {0: "UNDEFINED", 1: "TRANSITION_TO_FW", 2: "TRANSITION_TO_MC", 3: "MC", 4: "FW"}
LANDED_STATES = {0: "UNDEFINED", 1: "ON_GROUND", 2: "IN_AIR", 3: "TAKEOFF", 4: "LANDING"}
IN_AIR_M = 5.0
GROUND_SETTLE_S = 5.0     # after touchdown, before "on the ground" applies
FW_AIRSPEED = 9.0
FW_SUSTAINED_S = 10.0
FW_EDGE_S = 3.0           # transition allowance at either end of a fast stretch


def _sequence(samples, names, t_ref) -> str:
    out, last = [], None
    for s in samples:
        v = int(s.fields[names[0]])
        if v != last:
            out.append(f"{names[1].get(v, v)}@{s.host_mono - t_ref:+.0f}s")
            last = v
    return " → ".join(out)


async def test_extended_sys_state_compat_landed_state_in_flight(gcs_system, request, restart_flight_stack):
    """landed_state is never ON_GROUND ≥ 5 m up, nor airborne/taking off/landing while disarmed on the ground."""
    fl = await F.get_flight(gcs_system, restart_flight_stack)
    t_air, t_down = fl.need_flying(request, ["landed_state"])
    ess = fl.s(_MSG_NAME)
    gpi = fl.s("GLOBAL_POSITION_INT")
    if len(ess) < 10:
        skip_untestable(request, ["landed_state"], "too few EXTENDED_SYS_STATE recorded", f"{len(ess)} received")
    up, ground, wrong = [], [], []
    t_arm = fl.t("armed")
    t_after = (fl.t("touchdown") or t_down) + GROUND_SETTLE_S
    t_rearm = fl.t("rearmed")
    for s in ess:
        v = int(s.fields["landed_state"])
        g = nearest(gpi, s.host_mono)
        if abs(g.host_mono - s.host_mono) > 0.5:
            continue
        if g.fields["relative_alt"] / 1000.0 >= IN_AIR_M:
            up.append(v)
            if v == 1:
                wrong.append(f"ON_GROUND at {g.fields['relative_alt'] / 1000:.0f} m")
        elif s.host_mono < t_arm - 1.0 or (fl.t("disarmed") and s.host_mono > max(t_after, fl.t("disarmed"))
                                     and (t_rearm is None or s.host_mono < t_rearm)):
            ground.append(v)
            if v in (2, 3, 4):
                wrong.append(f"{LANDED_STATES[v]} disarmed on the ground")
    seq = _sequence(ess, ("landed_state", LANDED_STATES), t_air)
    detail = (f"≥ {IN_AIR_M:.0f} m: {dict(Counter(LANDED_STATES.get(v, v) for v in up))}; disarmed on the ground: "
              f"{dict(Counter(LANDED_STATES.get(v, v) for v in ground))}; sequence (from airborne) {seq}; {fl.label}")
    record_detail(request, detail)
    if wrong:
        record_field_fact(request, "landed_state", supported=False)
        report.compat_fail(f"landed_state contradicts the vehicle: {wrong[0]} ({len(wrong)} msgs) — {detail}")
    if up and all(v == 0 for v in up):
        record_field_fact(request, "landed_state", supported=False, notes="UNDEFINED in flight")
    elif any(v in (2, 3, 4) for v in up):
        record_field_fact(request, "landed_state", supported=True)


async def test_extended_sys_state_obs_landed_state_transitions(gcs_system, request, restart_flight_stack):
    """Characterise which landed states appear from arming to touchdown, and when (TAKEOFF/LANDING are optional)."""
    fl = await F.get_flight(gcs_system, restart_flight_stack)
    t_air, _ = fl.need_flying(request, [])
    ess = fl.s(_MSG_NAME, fl.t("armed"), fl.t("redisarmed") or fl.t("end"))
    ev = {k: f"{v - t_air:+.0f}s" for k, v in fl.events.items() if k in ("armed", "airborne", "touchdown", "disarmed")}
    record_detail(request, f"{_sequence(ess, ('landed_state', LANDED_STATES), t_air)}; events {ev}; {fl.label}")


async def test_extended_sys_state_compat_vtol_state_in_flight(gcs_system, request, restart_flight_stack):
    """A VTOL flying fast (≥ 9 m/s for 10 s) reports FW (or a transition), never MC; on the ground MC or FW."""
    fl = await F.get_flight(gcs_system, restart_flight_stack)
    t0, t1 = fl.need_moving(request, ["vtol_state"])
    ess = fl.s(_MSG_NAME)
    seq = _sequence(fl.s(_MSG_NAME, t0), ("vtol_state", VTOL_STATES), t0)
    if fl.kind != "vtol":
        record_detail(request, f"not a VTOL ({fl.label}, HEARTBEAT type {fl.mav_type}): {seq or 'no messages'}")
        pytest.skip(f"NA: not a VTOL (HEARTBEAT type {fl.mav_type}) — vtol_state {seq or 'not received'}")
    vfr = fl.s("GLOBAL_POSITION_INT", t0, t1)
    speed = {id(v): math.hypot(v.fields["vx"], v.fields["vy"]) / 100.0 for v in vfr}
    fast: list[tuple[float, float]] = []   # sustained fast stretches
    start = None
    for v in vfr:
        if speed[id(v)] >= FW_AIRSPEED:
            start = start or v.host_mono
        else:
            if start and v.host_mono - start >= FW_SUSTAINED_S:
                fast.append((start, v.host_mono))
            start = None
    if start and vfr and vfr[-1].host_mono - start >= FW_SUSTAINED_S:
        fast.append((start, vfr[-1].host_mono))
    if not fast:
        top = max(speed.values(), default=0.0)
        record_field_fact(request, "vtol_state", untestable=f"never in forward flight (ground speed ≤ {top:.0f} m/s)")
        pytest.fail(f"Inconclusive — the VTOL never flew forward (ground speed at most {top:.1f} m/s); {seq}")
    in_fast = [int(s.fields["vtol_state"]) for s in ess
               if any(a + FW_EDGE_S <= s.host_mono <= b - FW_EDGE_S for a, b in fast)]
    on_ground = [int(s.fields["vtol_state"]) for s in ess if s.host_mono < fl.t("armed") - 1.0
                 or (fl.t("touchdown") and s.host_mono > fl.t("touchdown") + GROUND_SETTLE_S)]
    wrong = [v for v in in_fast if v in (0, 3)] + [v for v in on_ground if v not in (3, 4)]
    if 4 not in in_fast:
        wrong.append(-1)  # never FW while flying fast
    fast_s = sum(b - a for a, b in fast)
    detail = (f"fast ({fast_s:.0f} s at ≥ {FW_AIRSPEED:.0f} m/s): {dict(Counter(VTOL_STATES.get(v, v) for v in in_fast))}; "
              f"on the ground: {dict(Counter(VTOL_STATES.get(v, v) for v in on_ground))}; sequence {seq}; {fl.label}")
    record_detail(request, detail)
    if wrong:
        record_field_fact(request, "vtol_state", supported=False)
        report.compat_fail(f"vtol_state doesn't match the flight (FW, not MC, when fast; MC/FW on the ground) — {detail}")
    record_field_fact(request, "vtol_state", supported=True)
