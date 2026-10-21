"""
MAV_CMD_NAV_VTOL_TAKEOFF (cmd=84) via COMMAND_INT — Tier 2 execution tests.

"Takeoff from ground using VTOL mode, and transition to forward flight with
specified heading. The command should be ignored by vehicles that dont support
both VTOL and fixed-wing flight (multicopters, boats, etc.)."

Arms, sends NAV_VTOL_TAKEOFF as a COMMAND_INT, and checks each parameter against
telemetry. Requires a real flight stack (--drone-address); every test is skipped
in paired/mock mode. Same conventions as tests/mission/nav_vtol_takeoff/
test_flight.py (two values per "is it honoured" check combined into one
verdict, NACK = legitimate REJECTED, accepted-but-ignored = compatibility error,
nothing gated on --vehicle-type, shared flights cached).

A command has no "item complete" event, so values are judged where the vehicle
settles (rule 4b): the mean altitude/position over the last SETTLE_WINDOW_S of
an OBSERVE_S watch — for a fixed-wing that's the centre of its loiter orbit.
Headings are judged when the transition to forward flight starts.

Expected from source (blind read, 2026-10-01 — Tier 2 pattern #5)
-----------------------------------------------------------------
PX4 main (`navigator_main.cpp`, `vtol_takeoff.cpp`):
- climbs in hover to param7 (the *transition* altitude, absolute), turns toward
  the loiter location (param5/6) — or to param4 when param2 is exactly 3
  (SPECIFIED) — transitions, then loiters at param5/6 at home + VTO_LOITER_ALT
  (default 80 m).
- so: Lat/Lon honoured; Altitude used for the transition but the vehicle
  *settles* at VTO_LOITER_ALT, not param7 (rule 4b → compatibility error; the
  detail shows the transition altitude too). param1 is stored as "loiter
  height" (`setLoiterHeight`) but never read — the loiter altitude is
  VTO_LOITER_ALT regardless.
- `mavlink_command_params.hpp` `{ 84, 0x78, 0x7C }` (command mask 0x7C =
  params 3-7) DENIES a non-zero param1 or param2 on main, so SPECIFIED — and
  with it every Yaw check — should be REJECTED. (Branch commit aad2f0f3 changes
  the mask to 0x7B; see tests/command/nav_vtol_takeoff/CLAUDE.md.)
ArduPlane: no COMMAND_INT handler for this command (QuadPlane runs it only as a
mission item) — UNSUPPORTED, every test skips.
ArduCopter: maps it onto its normal takeoff, in GUIDED only; it isn't a VTOL, so
the command-level test reports NA.

Running
-------
PX4 VTOL (Gazebo)::

    pytest tests/command/nav_vtol_takeoff/test_flight.py --drone-address=udp://:14540 \\
        --px4-sitl=~/github/px4/PX4-Autopilot --px4-model=gz_standard_vtol \\
        --vehicle-type=vtol --autopilot=px4 -v --log-cli-level=INFO
"""

import asyncio
import logging
import math
import os

import pytest
from mavsdk.plugins.telemetry import VtolState

from tests import report
from tests.command.conftest import INT32_MAX, _FMT, probe_command_int
from .test_command import SPEC as _VTOL_TAKEOFF_SPEC
from tests.flight_helpers import (
    AIRBORNE_THRESHOLD_M,
    _arm_and_send_takeoff,
    _dist_m,
    _get_heading,
    _get_home_position,
    _get_position,
    _offset_lat_lon,
    _rtl_and_land,
    vehicle_is_vtol,
    _tier2_auto_record,  # noqa: F401 — autouse: records every test's outcome for the Tier 2 log
    record_compat_command_supported,
    record_compat_json,
    record_tier2_detail,
    record_tier2_param_verdict,
    require_real_stack,  # noqa: F401 — registers the real-stack skip gate for this module
)
from tests.mock_flight_stack import MAV_RESULT_UNSUPPORTED

log = logging.getLogger(__name__)

# Two flights per cached probe, each OBSERVE_S plus arm/cleanup (restart fallback).
pytestmark = pytest.mark.timeout(900)

_CMD = "NAV_VTOL_TAKEOFF"
_CMD_ID = 84
_CMD_NAME = _CMD  # the report key — must match test_command.py's SPEC.name
assert _VTOL_TAKEOFF_SPEC.name == _CMD_NAME

HEADING_SPECIFIED = 3
HEADING_TAKEOFF = 2

# How long to watch each flight, and the tail of it that counts as "settled".
OBSERVE_S = float(os.environ.get("NAV_VTOL_TAKEOFF_OBSERVE_S", "150.0"))
SETTLE_WINDOW_S = 30.0  # ≈ one fixed-wing loiter orbit

ALTITUDE_VALUES_M = (30.0, 50.0)  # relative; away from PX4's VTO_LOITER_ALT default (80 m)
ALT_TOLERANCE_MIN_M = 5.0
ALT_TOLERANCE_FRAC = 0.2
TAKEOFF_ALT_M = 30.0

POSITION_OFFSET_M = 400.0
POSITION_BEARINGS = (("N", 0.0), ("E", 90.0))
POSITION_TOLERANCE_M = float(os.environ.get("NAV_VTOL_TAKEOFF_POSITION_TOLERANCE_M", "40.0"))

YAW_VALUES_DEG = (135.0, 225.0)
HEADING_TOLERANCE_DEG = float(os.environ.get("NAV_VTOL_TAKEOFF_HEADING_TOLERANCE_DEG", "25.0"))
LOITER_RELATIVE_BEARING_DEG = 90.0  # TAKEOFF check: loiter point 90° off the ground heading

_FW_STATES = (VtolState.TRANSITION_TO_FW, VtolState.FW)

# How long a fresh (re)boot may take to send its first GLOBAL_POSITION_INT (heading and
# position come from it). Gazebo VTOL took over 30 s after HOME_POSITION (2026-10-01).
POSITION_READY_TIMEOUT_S = float(os.environ.get("NAV_VTOL_TAKEOFF_POSITION_READY_TIMEOUT_S", "120.0"))

# Caches (rule 7).
_supported: bool | None = None
_no_takeoff: str | None = None  # set when the baseline (real-altitude) takeoffs were accepted but never left the ground
_altitude_flights: list[dict] | None = None
_position_result: dict | None = None
_specified_result: dict | None = None


# ---------------------------------------------------------------------------
# Gate and restart binding
# ---------------------------------------------------------------------------

_restart = None


@pytest.fixture(autouse=True)
def _bind_restart_flight_stack(restart_flight_stack):
    """Every cleanup restarts the stack if the vehicle doesn't land (root CLAUDE.md Tier 2 pattern #8)."""
    global _restart
    _restart = restart_flight_stack
    yield
    _restart = None


async def _ensure_supported(system) -> None:
    """
    Skip when a flight's ACK already showed the command UNSUPPORTED, or the baseline
    takeoffs showed it accepted but never flying.

    Support is learned from the first real (armed) send in _fly(), not from a
    disarmed probe: on PX4 multicopter a NAV_VTOL_TAKEOFF sent while disarmed left
    the next armed one ACCEPTED but never climbing — reproduced twice, first flight
    of the session both times (2026-10-01).
    """
    if _supported is False:
        pytest.skip(f"{_CMD} (cmd={_CMD_ID}) is UNSUPPORTED via COMMAND_INT on this stack — flight tests not run")
    if _no_takeoff is not None:
        pytest.skip(f"NA: {_no_takeoff} — see test_vtol_takeoff_compat_takes_off_and_transitions")


# ---------------------------------------------------------------------------
# One flight, every measurement
# ---------------------------------------------------------------------------

async def _ground_heading(system) -> float:
    """
    Heading before arming, with a long timeout: straight after a flight-stack
    (re)start PX4 sends no GLOBAL_POSITION_INT (which heading comes from) until its EKF
    has a global position — even once HOME_POSITION is already arriving. 2026-10-01 a
    test starting ~5 s into a fresh boot timed out here (the message watcher showed
    GLOBAL_POSITION_INT never received).
    """
    return await _get_heading(system, timeout_s=POSITION_READY_TIMEOUT_S)


def _angle_diff(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


async def _fly(system, **overrides) -> dict:
    """
    Arm, send NAV_VTOL_TAKEOFF (COMMAND_INT, frame 5, z given relative and converted
    to AMSL by _arm_and_send_takeoff), watch for OBSERVE_S, then RTL/restart. Returns:

      nacked/result   COMMAND_ACK result (no flight when NACKed)
      max_alt         highest relative altitude seen
      settled_alt     mean relative altitude over the last SETTLE_WINDOW_S
      centre          mean (lat, lon) over the last SETTLE_WINDOW_S
      transition      {heading, alt} at the first TRANSITION_TO_FW/FW state, or None
      vtol_states     every vtol_state name seen
      ground_heading  heading before arming
    """
    ground_heading = await _ground_heading(system)
    kw = dict(command=_CMD_ID, param1=0.0, param2=0.0, param3=0.0, param4=None, z=TAKEOFF_ALT_M)
    kw.update(overrides)
    state = {"heading": ground_heading, "alt": 0.0, "transition": None}
    vtol_states: set[str] = set()
    track: list[tuple[float, float, float, float]] = []  # (t, alt, lat, lon)

    async def _positions() -> None:
        loop = asyncio.get_running_loop()
        async for pos in system.telemetry.position():
            state["alt"] = pos.relative_altitude_m
            track.append((loop.time(), pos.relative_altitude_m, pos.latitude_deg, pos.longitude_deg))

    async def _headings() -> None:
        async for hdg in system.telemetry.heading():
            state["heading"] = hdg.heading_deg

    async def _vtol() -> None:
        async for vs in system.telemetry.vtol_state():
            vtol_states.add(vs.name)
            if vs in _FW_STATES and state["transition"] is None and state["alt"] >= AIRBORNE_THRESHOLD_M:
                state["transition"] = {"heading": state["heading"], "alt": state["alt"]}

    tasks = [asyncio.create_task(f()) for f in (_positions, _headings, _vtol)]
    # _arm_and_send_takeoff sends z as home AMSL + the relative value; record both
    # altitudes so a wrong absolute target (e.g. home altitude not yet converged on a
    # fresh boot) shows up in the result instead of looking like a behaviour.
    home = await _get_home_position(system)
    pos = await _get_position(system, timeout_s=POSITION_READY_TIMEOUT_S)
    amsl = {"home_amsl": home.absolute_altitude_m, "vehicle_amsl": pos.absolute_altitude_m,
            "z_sent": None if kw["z"] is None else home.absolute_altitude_m + kw["z"]}
    try:
        ack = await _arm_and_send_takeoff(system, return_ack=True, **kw)
        if ack is None:
            pytest.fail("No COMMAND_ACK for NAV_VTOL_TAKEOFF — cannot classify")
        result = int(ack["result"])
        global _supported
        if result == MAV_RESULT_UNSUPPORTED:
            _supported = False
            pytest.skip(f"{_CMD} (cmd={_CMD_ID}) is UNSUPPORTED via COMMAND_INT on this stack — flight tests not run")
        _supported = True
        if result not in (0, 5):
            return {"nacked": True, "result": result, "ground_heading": ground_heading,
                    "vtol_states": sorted(vtol_states)}
        await asyncio.sleep(OBSERVE_S)
    finally:
        for t in tasks:
            t.cancel()
        await _rtl_and_land(system, _restart)
        # A transition that started but never reached forward flight (e.g. a PX4
        # quad-chute) can latch a failure that keeps the vehicle unarmable even after
        # it lands ("VTOL fixed-wing system failure detected", PX4 v1.17 Gazebo,
        # 2026-10-01: every later test timed out at _wait_armable). Restart.
        if "TRANSITION_TO_FW" in vtol_states and "FW" not in vtol_states and _restart is not None:
            log.warning("Transition aborted during this flight — restarting the flight stack")
            _restart()

    end = track[-1][0] if track else 0.0
    tail = [p for p in track if p[0] >= end - SETTLE_WINDOW_S] or track[-1:]
    max_alt = max((p[1] for p in track), default=0.0)
    out = {
        "nacked": False, "result": result, "ground_heading": ground_heading, "max_alt": max_alt,
        "settled_alt": sum(p[1] for p in tail) / len(tail) if tail else None,
        "centre": (sum(p[2] for p in tail) / len(tail), sum(p[3] for p in tail) / len(tail)) if tail else None,
        "transition": state["transition"], "vtol_states": sorted(vtol_states), **amsl,
    }
    log.info(_FMT, _CMD, "flight", out)
    return out




def _require_airborne(r: dict, what: str) -> None:
    if not r["nacked"] and r["max_alt"] < AIRBORNE_THRESHOLD_M:
        pytest.fail(f"Vehicle never got airborne — {what} inconclusive ({r})")
    if not r["nacked"] and "TRANSITION_TO_FW" in r.get("vtol_states", []) and "FW" not in r["vtol_states"]:
        # e.g. PX4 Gazebo v1.17: the simulated airspeed sensor dropped out mid-transition
        # and PX4 quad-chuted ("Quad-chute triggered", 2026-10-01) — the vehicle then holds
        # wherever it is, which says nothing about the param under test.
        pytest.fail(f"Transition to forward flight started but never completed (aborted — e.g. a "
                    f"quad-chute; see the message watcher's STATUSTEXT) — {what} inconclusive ({r})")


# ---------------------------------------------------------------------------
# Verdicts (rule 4a)
# ---------------------------------------------------------------------------

def _combine(results: list[dict]) -> dict:
    return {"ok": all(r["ok"] for r in results), "nacked": all(r["nacked"] for r in results),
            "detail": "; ".join(r["detail"] for r in results)}


def _verdict_text(v: dict) -> str:
    if v["nacked"]:
        return "REJECTED (NACKed)"
    return "SUPPORTED" if v["ok"] else "NOT SUPPORTED — COMPATIBILITY ERROR"


def _record(request, label: str, keys: tuple[str, ...], v: dict, criterion: str) -> None:
    record_tier2_detail(request, f"{criterion}: {v['detail']}")
    record_tier2_param_verdict(request, label, _verdict_text(v))
    for key in keys:
        record_compat_json(request, key, supported=v["ok"],
                           nacks_on_non_sentinel_value=None if v["ok"] else v["nacked"])
    if not (v["ok"] or v["nacked"]):
        report.compat_fail(f"{label} accepted but not honoured: {v['detail']} (root CLAUDE.md rule 4)")


def _heading_value(r: dict, expected: float, label: str) -> dict:
    if r["nacked"]:
        return {"nacked": True, "ok": False, "detail": f"{label}: REJECTED (result={r['result']})"}
    _require_airborne(r, f"{label} heading")
    tr = r["transition"]
    if tr is None:
        return {"nacked": False, "ok": False,
                "detail": f"{label}: no transition to forward flight seen (states {r['vtol_states']})"}
    diff = _angle_diff(tr["heading"], expected)
    return {"nacked": False, "ok": diff <= HEADING_TOLERANCE_DEG,
            "detail": f"{label}: expected {expected:.0f}°, transition heading {tr['heading']:.1f}° (diff {diff:.1f}°)"}


# ---------------------------------------------------------------------------
# Cached probes
# ---------------------------------------------------------------------------

async def _check_altitude(system) -> list[dict]:
    """
    Fly every ALTITUDE_VALUES_M once (shared with the takes-off-and-transitions
    test). These are the baseline takeoffs: if every accepted one stays on the
    ground, the command doesn't execute here and every later flight test is NA.
    Only these set that — a characterisation flight that doesn't climb (e.g.
    z = NaN) says nothing about the others.
    """
    global _altitude_flights, _no_takeoff
    if _altitude_flights is None:
        flights = []
        for alt in ALTITUDE_VALUES_M:
            r = await _fly(system, z=alt)
            r["target_alt"] = alt
            flights.append(r)
        _altitude_flights = flights
        flown = [f for f in flights if not f["nacked"]]
        if flown and all(f["max_alt"] < AIRBORNE_THRESHOLD_M for f in flown):
            _no_takeoff = "NAV_VTOL_TAKEOFF was accepted with a real altitude but the vehicle never left the ground"
    return _altitude_flights


async def _check_position(system) -> dict:
    global _position_result
    if _position_result is None:
        values = []
        home = await _get_home_position(system)
        for label, bearing in POSITION_BEARINGS:
            n = POSITION_OFFSET_M * math.cos(math.radians(bearing))
            e = POSITION_OFFSET_M * math.sin(math.radians(bearing))
            t_lat, t_lon = _offset_lat_lon(home.latitude_deg, home.longitude_deg, n, e)
            r = await _fly(system, x=int(t_lat * 1e7), y=int(t_lon * 1e7))
            tag = f"{POSITION_OFFSET_M:.0f} m {label}"
            if r["nacked"]:
                values.append({"nacked": True, "ok": False, "detail": f"{tag}: REJECTED (result={r['result']})"})
                continue
            _require_airborne(r, "lat/lon")
            d = _dist_m(*r["centre"], t_lat, t_lon)
            values.append({"nacked": False, "ok": d <= POSITION_TOLERANCE_M,
                           "detail": f"{tag}: settled centre {d:.1f} m from target, "
                                     f"{_dist_m(*r['centre'], home.latitude_deg, home.longitude_deg):.1f} m from home"})
        _position_result = _combine(values)
    return _position_result


async def _check_specified(system) -> dict:
    global _specified_result
    if _specified_result is None:
        values = []
        for yaw in YAW_VALUES_DEG:
            r = await _fly(system, param2=float(HEADING_SPECIFIED), param4=yaw)
            values.append(_heading_value(r, yaw, f"SPECIFIED, yaw {yaw:.0f}°"))
        _specified_result = _combine(values)
    return _specified_result


# ---------------------------------------------------------------------------
# Command level
# ---------------------------------------------------------------------------

async def test_vtol_takeoff_compat_takes_off_and_transitions(gcs_system, request):
    """
    The XML's one requirement: the vehicle takes off in VTOL mode and transitions to forward flight.

    Read from the altitude test's flights (shared, rule 7). PASS if the vehicle got
    airborne and transitioned. FAIL (compatibility error) if a VTOL accepted the
    command and then didn't take off, or took off but never transitioned. NA for a
    vehicle that never reports a VTOL state — the XML says it should ignore the
    command; what it actually did is recorded in the detail.
    """
    await _ensure_supported(gcs_system)
    flights = await _check_altitude(gcs_system)
    if all(f["nacked"] for f in flights):
        record_tier2_detail(request, f"NA: NAV_VTOL_TAKEOFF NACKed (results {[f['result'] for f in flights]})")
        pytest.skip("NA: NAV_VTOL_TAKEOFF NACKed with a real altitude — nothing flew")
    flown = [f for f in flights if not f["nacked"]]
    took_off = [f["max_alt"] >= AIRBORNE_THRESHOLD_M for f in flown]
    if not await vehicle_is_vtol(gcs_system):
        record_tier2_detail(
            request,
            "NA: vehicle's HEARTBEAT type isn't a VTOL type — not a VTOL, so the XML says it "
            f"should ignore this command; it was ACCEPTED and {'took off' if any(took_off) else 'did not take off'}",
        )
        pytest.skip("NA: not a VTOL (HEARTBEAT type isn't MAV_TYPE_VTOL_*) — the XML says such vehicles should ignore the command")
    detail = "; ".join(
        f"{f['target_alt']:.0f} m: max alt {f['max_alt']:.1f} m, transition "
        + ("none" if f["transition"] is None else f"at {f['transition']['alt']:.1f} m")
        for f in flown
    )
    record_tier2_detail(request, f"PASS if the vehicle takes off and transitions to forward flight: {detail}")
    if not all(took_off):
        record_compat_command_supported(request, False, notes="Accepted, but the vehicle does not take off")
        report.compat_fail(f"NAV_VTOL_TAKEOFF accepted but the vehicle did not take off: {detail}")
    if not all(f["transition"] is not None for f in flown):
        record_compat_command_supported(request, True, notes="Takes off, but does not transition to forward flight")
        report.compat_fail(f"Took off but did not transition to forward flight: {detail}")
    record_compat_command_supported(request, True)


# ---------------------------------------------------------------------------
# param7 (Altitude)
# ---------------------------------------------------------------------------

async def test_vtol_takeoff_compat_altitude_honoured(gcs_system, request):
    """
    param7 = 30 m and 50 m — "is altitude honoured": NACKed, or the vehicle settles at it.

    Rule 4b: where the vehicle ends up, not what it passes through. The altitude at
    the start of the transition is in the detail too, since a stack may use param7
    as a transition altitude and then climb to a loiter altitude of its own.
    """
    await _ensure_supported(gcs_system)
    values = []
    for f in await _check_altitude(gcs_system):
        alt = f["target_alt"]
        if f["nacked"]:
            values.append({"nacked": True, "ok": False, "detail": f"{alt:.0f} m: REJECTED (result={f['result']})"})
            continue
        _require_airborne(f, "altitude")
        tol = max(ALT_TOLERANCE_MIN_M, ALT_TOLERANCE_FRAC * alt)
        tr = f["transition"]
        values.append({"nacked": False, "ok": abs(f["settled_alt"] - alt) <= tol,
                       "detail": f"{alt:.0f} m: settled {f['settled_alt']:.1f} m (±{tol:.0f} m), transition at "
                                 + ("-" if tr is None else f"{tr['alt']:.1f} m")})
    _record(request, "param7 (Altitude)", ("7_Altitude",), _combine(values),
            "PASS if each commanded altitude is NACKed or the vehicle settles at it")


def _altitude_meaning(per_value: list[tuple[float, float | None, float | None]]) -> str:
    """
    Classify what param7 is used as, from (commanded, transition_alt, final_alt) per
    value — "matches" meaning within the altitude test's tolerance for every value.
    """
    def _all_match(i: int) -> bool:
        return all(v[i] is not None and abs(v[i] - v[0]) <= max(ALT_TOLERANCE_MIN_M, ALT_TOLERANCE_FRAC * v[0])
                   for v in per_value)
    transition, final = _all_match(1), _all_match(2)
    if transition and final:
        return "BOTH — transition altitude and final altitude"
    if transition:
        return "TRANSITION ALTITUDE ONLY — the vehicle then goes to an altitude of its own"
    if final:
        return "FINAL ALTITUDE ONLY — the transition happens at some other altitude"
    return "NEITHER"


async def test_vtol_takeoff_obs_altitude_meaning(gcs_system, request):
    """
    param7 — characterisation: is Altitude the transition altitude, the altitude the vehicle settles at, or both?

    The XML says only "Altitude". PX4 uses it as the transition altitude and then
    climbs to VTO_LOITER_ALT (source: setTransitionAltitudeAbsolute(param7), then a
    loiter at home + VTO_LOITER_ALT) — which is why the rule-4b altitude test fails
    there. This records, from the altitude test's flights (cached, rule 7), the
    altitude at the start of the transition and the settled altitude, so the same
    question is answered for every stack. Observational (rule 3) — convergent
    behaviour across stacks is evidence for a spec clarification.
    """
    await _ensure_supported(gcs_system)
    flights = [f for f in await _check_altitude(gcs_system) if not f["nacked"]]
    if not flights:
        pytest.skip("NA: every altitude command was NACKed — nothing flew")
    if not await vehicle_is_vtol(gcs_system):
        pytest.skip("NA: not a VTOL (HEARTBEAT type isn't MAV_TYPE_VTOL_*) — there is no transition to classify")
    per_value = [(f["target_alt"], f["transition"]["alt"] if f["transition"] else None, f["settled_alt"])
                 for f in flights]
    values = "; ".join(
        f"{c:.0f} m: transition {'-' if t is None else f'{t:.1f} m'}, settled {'-' if e is None else f'{e:.1f} m'}"
        for c, t, e in per_value)
    record_tier2_detail(request, f"Observational — param7 used as: {_altitude_meaning(per_value)} ({values})")

async def test_vtol_takeoff_obs_altitude_nan(gcs_system, request):
    """param7 = NaN — characterisation: does it take off, and where does it settle? (no sentinel meaning in the XML)"""
    await _ensure_supported(gcs_system)
    r = await _fly(gcs_system, z=None)
    if r["nacked"]:
        record_tier2_detail(request, f"Observational: z=NaN NACKed (result={r['result']})")
        return
    record_tier2_detail(request, f"Observational: z=NaN — max alt {r['max_alt']:.1f} m, settled {r['settled_alt']:.1f} m")


# ---------------------------------------------------------------------------
# params 5/6 (Latitude/Longitude)
# ---------------------------------------------------------------------------

async def test_vtol_takeoff_compat_position_honoured(gcs_system, request):
    """
    params 5/6 = points 400 m north and 400 m east — "is lat/lon honoured": NACKed, or the vehicle settles there.

    Destination under the hasLocation/isDestination convention (rule 6). Judged by
    the centre of the last SETTLE_WINDOW_S of track, so it works for a vehicle that
    hovers at the point and one that orbits it.
    """
    await _ensure_supported(gcs_system)
    v = await _check_position(gcs_system)
    _record(request, "param5/6 (Lat/Lon)", ("5_Latitude", "6_Longitude"), v,
            f"PASS if each target is NACKed or the vehicle settles within {POSITION_TOLERANCE_M:.0f} m of it")


async def test_vtol_takeoff_obs_from_current_position(gcs_system, request):
    """params 5/6 = INT32_MAX ("use current position") — characterisation: where does the vehicle settle?"""
    await _ensure_supported(gcs_system)
    home = await _get_home_position(gcs_system)
    r = await _fly(gcs_system, x=INT32_MAX, y=INT32_MAX)
    record_compat_json(request, "5_Latitude", accept_nan_or_int32max=not r["nacked"])
    record_compat_json(request, "6_Longitude", accept_nan_or_int32max=not r["nacked"])
    if r["nacked"]:
        record_tier2_detail(request, f"Observational: INT32_MAX lat/lon NACKed (result={r['result']})")
        return
    d = _dist_m(*r["centre"], home.latitude_deg, home.longitude_deg)
    record_tier2_detail(request, f"Observational (no fixed target for the sentinel): settled centre {d:.1f} m from home, "
                                 f"max alt {r['max_alt']:.1f} m")


# ---------------------------------------------------------------------------
# param4 (Yaw Angle) and param2 (Transition Heading)
# ---------------------------------------------------------------------------

async def test_vtol_takeoff_compat_yaw_honoured(gcs_system, request):
    """
    param4 = 135° and 225° with param2 = SPECIFIED — "is yaw honoured": NACKed, or the transition starts on that heading.

    SPECIFIED ("Use the specified heading in parameter 4") is the only place the XML
    gives param4 an effect, so the transition heading is what's judged.
    """
    await _ensure_supported(gcs_system)
    if await vehicle_is_vtol(gcs_system) is False:
        pytest.skip("NA: not a VTOL (HEARTBEAT type isn't MAV_TYPE_VTOL_*) — no transition, so no transition heading to judge")
    v = await _check_specified(gcs_system)
    _record(request, "param4 (Yaw Angle)", ("4_Yaw Angle",), v,
            f"PASS if each yaw is NACKed or the transition starts within ±{HEADING_TOLERANCE_DEG:.0f}° of it")


async def test_vtol_takeoff_compat_transition_heading(gcs_system, request):
    """
    param2 (Transition Heading) — SPECIFIED (cached from the yaw test) and TAKEOFF: NACKed, or the transition starts on that heading.

    TAKEOFF ("the heading on takeoff, while sitting on the ground") is checked with
    the loiter point 90° off the ground heading, so "face the loiter point" can't
    pass it by coincidence. NEXT_WAYPOINT has no defined target for a standalone
    command (there is no next waypoint); VEHICLE_DEFAULT and ANY have no fixed
    target — those are checked only at Tier 1.
    """
    await _ensure_supported(gcs_system)
    if await vehicle_is_vtol(gcs_system) is False:
        pytest.skip("NA: not a VTOL (HEARTBEAT type isn't MAV_TYPE_VTOL_*) — no transition, so no transition heading to judge")
    values = [await _check_specified(gcs_system)]
    home = await _get_home_position(gcs_system)
    ground = await _ground_heading(gcs_system)
    bearing = (ground + LOITER_RELATIVE_BEARING_DEG) % 360
    n = POSITION_OFFSET_M * math.cos(math.radians(bearing))
    e = POSITION_OFFSET_M * math.sin(math.radians(bearing))
    t_lat, t_lon = _offset_lat_lon(home.latitude_deg, home.longitude_deg, n, e)
    r = await _fly(gcs_system, param2=float(HEADING_TAKEOFF), x=int(t_lat * 1e7), y=int(t_lon * 1e7))
    values.append(_heading_value(r, r["ground_heading"], "TAKEOFF, loiter point +90° off the ground heading"))
    _record(request, "param2 (Transition Heading)", ("2_Transition Heading",), _combine(values),
            f"PASS if each value is NACKed or the transition starts within ±{HEADING_TOLERANCE_DEG:.0f}° of its heading "
            f"(NEXT_WAYPOINT/VEHICLE_DEFAULT/ANY have no fixed target for a command)")


async def test_vtol_takeoff_compat_yaw_sentinel(gcs_system, request):
    """
    param4 = NaN — NOT TESTABLE: "use the current yaw heading mode" has no fixed
    target, and re-sending a VTOL takeoff mid-flight to see whether the heading
    changes (nav_takeoff's technique) isn't a defined use of this command.
    """
    await _ensure_supported(gcs_system)
    record_tier2_param_verdict(request, "Yaw sentinel (NaN)",
                               "NOT TESTABLE (no fixed target; a VTOL takeoff can't be re-sent mid-flight)")
    pytest.skip("NA: NaN's effect has no fixed target — see docstring")


# ---------------------------------------------------------------------------
# params 1/3 (Empty)
# ---------------------------------------------------------------------------

async def test_vtol_takeoff_obs_param1_loiter_height(gcs_system, request):
    """
    param1 = 40 (Empty in the XML; PX4 names it "loiter height") — characterisation: does it change where the vehicle settles?

    An implementation-specific use of an Empty slot isn't a compliance question
    (rule 3); recorded so the devguide and the PX4 notes can say what it does.
    """
    await _ensure_supported(gcs_system)
    r = await _fly(gcs_system, param1=40.0)
    if r["nacked"]:
        record_tier2_detail(request, f"Observational: param1 = 40 NACKed (result={r['result']})")
        return
    record_tier2_detail(request, f"Observational: param1 = 40, z = {TAKEOFF_ALT_M:.0f} m — settled at "
                                 f"{r['settled_alt']:.1f} m (max {r['max_alt']:.1f} m)")


async def test_vtol_takeoff_compat_empty_params(gcs_system, request):
    """
    params 1 and 3 (Empty) — nothing to support; records the sentinel facts from disarmed ACK probes, without flying.

    Tier 1 (test_command.py) checks the same in more depth; repeated here so this
    module's JSON is complete when run on its own.
    """
    await _ensure_supported(gcs_system)
    for slot, key in ((1, "1_Empty"), (3, "3_Empty")):
        acks = {}
        for label, value in (("nan", None), ("real", 1.0)):
            ack = await probe_command_int(gcs_system, _CMD_ID, frame=6, param4=None, x=INT32_MAX, y=INT32_MAX,
                                          z=TAKEOFF_ALT_M, **{f"param{slot}": value})
            acks[label] = None if ack is None else int(ack["result"])
        record_compat_json(request, key, supported="not-applicable",
                           accept_nan_or_int32max=acks["nan"] in (0, 5),
                           nacks_on_non_sentinel_value=acks["real"] not in (0, 5, None))
    pytest.skip("NA: params 1 and 3 are Empty — nothing to functionally support")
