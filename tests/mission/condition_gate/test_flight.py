"""
MAV_CMD_CONDITION_GATE (cmd=4501) — Tier 2 (execution) verification. PX4 only
(ArduPilot does not implement this command at all — see CLAUDE.md).

Directly tests the specific behavioural claims made by mavlink-devguide
PR #761 (https://github.com/mavlink/mavlink-devguide/pull/761), using real
flight telemetry rather than trusting the prose:

1. "marks an off-path location (not a destination)... the vehicle flies
   directly towards the next mission item that is on the path" — verified by
   sampling the flown ground track during the leg and asserting it stays
   close to the straight line between the two flanking waypoints, never
   approaching the gate's own (deliberately off-path) coordinates.
2. "The mission state machine is blocked on the gate mission item until the
   vehicle reaches the point on the path that is perpendicular to the gate
   location" — verified indirectly: a MAV_CMD_DO_CHANGE_SPEED item placed
   immediately after the gate can only execute once the mission sequence
   advances past the gate, so a mid-leg speed change (not at either
   waypoint) is direct evidence of the blocking/trigger mechanism, and its
   location on the track confirms *where* the crossing was detected.
3. "UseAltitude field ignored [by PX4]... geometry test is 2D" — has a Tier 2
   test (test_gate_obs_use_altitude_gates_on_altitude_if_supported) that
   always flies: per root CLAUDE.md's Tier 2 design pattern #1, only an
   actual upload NACK is a legitimate skip basis, so Tier 1's finding that
   param2 doesn't survive the round trip (test_params_1_2_zeroed_on_roundtrip_px4)
   is logged as context, not used to decide whether to run. The test is
   purely observational either way (no assertion on which outcome is
   "correct").

Mirrors the shape of the sibling manual-verification tool at
mavsdk_qgc_server_tests/condition_gate_tests/upload_condition_gate_mission.py
(waypoint -> gate -> DO_CHANGE_SPEED -> waypoint -> RTL, with takeoff
commanded separately beforehand — see root CLAUDE.md's Tier 2 design pattern
#11 and 2026-09-16's dated section in CLAUDE.md for why this file doesn't
carry a mission-item NAV_TAKEOFF) but automates the observation with
telemetry sampling instead of a human watching QGroundControl, and uploads
via the raw mavlink_direct transport (see tests/mission/conftest.py's
raw_upload_mission_items) since mission_raw rejects this <wip/> command
client-side.

Running
-------
    pytest tests/mission/condition_gate/test_flight.py \\
        --drone-address=udp://:14540 --vehicle-type=quadcopter --autopilot=px4 \\
        --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \\
        -v --log-cli-level=INFO
"""

import asyncio
import json
import logging
import math
import os

import pytest
from mavsdk.mavlink_direct import MavlinkMessage
from mavsdk.mission_raw import MissionItem

from tests import report
from ..conftest import (
    RawMissionError,
    clear_all_mission_types,
    raw_download_mission_items,
    raw_upload_mission_items,
)
from .test_protocol import SPEC as _GATE_SPEC
from tests.flight_helpers import (
    _get_home_position,
    _request_position_stream,
    _rtl_and_land,
    _takeoff_via_command,
    _tier2_auto_record,
    record_tier2_detail,
    record_tier2_param_verdict,
    require_real_stack,
)

log = logging.getLogger(__name__)

pytestmark = pytest.mark.timeout(360)

# Read by tests/report.py's write() to name/identify this module's report.
_CMD_NAME = "CONDITION_GATE"
_CMD_ID = 4501

# Declare identity + full param-slot list at import time — see nav_takeoff/
# test_flight.py's identical pattern for the full reasoning.
report.declare_command("mission", _CMD_NAME, _CMD_ID)
report.declare_params("mission", _CMD_NAME, [f"{p.slot}_{p.label}" for p in _GATE_SPEC.params])

NAN = float("nan")
_GATE_CMD = 4501
_WAYPOINT_CMD = 16
_DO_CHANGE_SPEED_CMD = 178
_RTL_CMD = 20
_MISSION_START_CMD = 300

_CRUISE_ALT_M = 30.0
_LEG_START_M = 50.0     # wp1: metres north of home
_LEG_LENGTH_M = 140.0   # wp1 -> wp2 distance along the leg
_GATE_FRACTION = 0.5    # gate's projection onto the leg, as a fraction of its length
_GATE_OFFSET_M = 25.0   # perpendicular (east) offset of the gate from the leg — "off-path"
_REDUCED_SPEED_MPS = 2.0
_DEFAULT_CRUISE_MPS = 5.0  # fallback if MPC_XY_CRUISE can't be read
_ON_TRACK_TOLERANCE_M = 8.0     # max acceptable cross-track deviation if the gate is NOT a destination
_GATE_APPROACH_TOLERANCE_M = 10.0  # min acceptable closest-approach to the gate's own coordinates

# TUNING NOTE (see root CLAUDE.md's "Tier 2 live-test padding" convention):
# sampling duration is derived from the *measured* cruise speed and known
# mission geometry, not a flat guess — a stack with a slower configured
# cruise speed needs proportionally longer automatically. Only
# _CLIMB_AND_TRANSIT_ALLOWANCE_S is a flat, stack-agnostic guess (there is no
# cheap way to query actual climb rate over mavlink). This command is PX4-only
# today (ArduPilot doesn't implement it — see the skip in the test below), so
# there is no second real stack to have tuned this against yet; the override
# below exists for when there is one, rather than requiring a code edit:
#
#   CONDITION_GATE_CLIMB_ALLOWANCE_S=45 pytest tests/mission/condition_gate/test_flight.py ...
#
# If that's still not enough, the test fails with a distinct
# "look like insufficient sampling time" message (see _timing_caveat() below)
# rather than misreporting it as a behavioural difference — widen the
# allowance (env var, or the default once confirmed) rather than trusting a
# plain assertion failure to mean the gate doesn't work on that stack.
# Confirmed on PX4 SIH (5 m/s cruise) across multiple runs: the trigger is
# always detected by t=~49-56s into a ~105s budget, i.e. roughly half the
# budget goes unused — see each run's "budget unused" log line. Left as-is (not shrunk
# further) because there's no second stack yet to confirm the margin is still
# enough once climb rate/cruise speed actually differ from PX4 SIH's.
_CLIMB_AND_TRANSIT_ALLOWANCE_S = float(os.environ.get("CONDITION_GATE_CLIMB_ALLOWANCE_S", "25.0"))


def _local_xy(lat_deg: float, lon_deg: float, ref_lat_deg: float, ref_lon_deg: float) -> tuple[float, float]:
    """Flat-earth (north_m, east_m) of (lat, lon) relative to a reference point."""
    north_m = (lat_deg - ref_lat_deg) * 111111.0
    east_m = (lon_deg - ref_lon_deg) * 111111.0 * math.cos(math.radians(ref_lat_deg))
    return north_m, east_m


def _latlon_at(ref_lat_deg: float, ref_lon_deg: float, north_m: float, east_m: float) -> tuple[float, float]:
    """Inverse of _local_xy: (lat, lon) offset by (north_m, east_m) from a reference point."""
    lat = ref_lat_deg + north_m / 111111.0
    lon = ref_lon_deg + east_m / (111111.0 * math.cos(math.radians(ref_lat_deg)))
    return lat, lon


def _cross_track_distance_m(north_m: float, east_m: float, leg_start_north_m: float, leg_end_north_m: float) -> float:
    """
    Perpendicular distance from (north_m, east_m) to the straight leg, which
    runs due north (constant east=0) from leg_start_north_m to
    leg_end_north_m. Simplifies to |east_m| since the leg has no east
    component in this test's geometry.
    """
    return abs(east_m)


def _build_mission(
    home_lat: float, home_lon: float, home_amsl_m: float,
    *, gate_param2: float = 0.0, gate_alt_m: float = _CRUISE_ALT_M,
) -> list[MissionItem]:
    """
    wp1 -> gate (off-path) -> DO_CHANGE_SPEED -> wp2 -> RTL.

    Deliberately carries NO takeoff item — the vehicle takes off via
    _takeoff_via_command() (commanded, not a mission-item NAV_TAKEOFF) before
    this mission is ever uploaded, per root CLAUDE.md's Tier 2 design pattern
    #11 principle applied to CONDITION_GATE 2026-09-16: this file exists to
    test CONDITION_GATE, not NAV_TAKEOFF, so its own reliability shouldn't
    depend on mission-item NAV_TAKEOFF's (see nav_takeoff/CLAUDE.md and root
    CLAUDE.md future-work #10 for why that's a real, separate risk on some
    stacks/frames — PX4 fixed-wing/VTOL never leaves the ground via a
    mission-item takeoff in this environment, an issue this file has no
    reason to inherit).

    All items use frame=6 (GLOBAL_RELATIVE_ALT_INT) except DO_CHANGE_SPEED
    (frame=2, MAV_FRAME_MISSION — its params are unscaled, per
    tests/mission/CLAUDE.md's frame-type findings) and RTL (frame=2, since
    PX4 rejects RTL as a mission item outside MAV_FRAME_MISSION — see
    do_reposition's sibling finding for the same command family).

    `gate_param2` (UseAltitude) and `gate_alt_m` are overridable for
    test_gate_obs_use_altitude_gates_on_altitude_if_supported's conditional
    3D-gate probe/flight — every other caller uses the 2D-gate defaults.
    """
    leg_end_m = _LEG_START_M + _LEG_LENGTH_M
    gate_north_m = _LEG_START_M + _GATE_FRACTION * _LEG_LENGTH_M

    wp1_lat, wp1_lon = _latlon_at(home_lat, home_lon, _LEG_START_M, 0.0)
    wp2_lat, wp2_lon = _latlon_at(home_lat, home_lon, leg_end_m, 0.0)
    gate_lat, gate_lon = _latlon_at(home_lat, home_lon, gate_north_m, _GATE_OFFSET_M)

    items = [
        MissionItem(
            seq=0, frame=6, command=_WAYPOINT_CMD, current=1, autocontinue=1,
            param1=0.0, param2=0.0, param3=0.0, param4=NAN,
            x=int(wp1_lat * 1e7), y=int(wp1_lon * 1e7), z=_CRUISE_ALT_M,
            mission_type=0,
        ),
        MissionItem(
            seq=1, frame=6, command=_GATE_CMD, current=0, autocontinue=1,
            param1=0.0, param2=gate_param2, param3=0.0, param4=0.0,
            x=int(gate_lat * 1e7), y=int(gate_lon * 1e7), z=gate_alt_m,
            mission_type=0,
        ),
        MissionItem(
            seq=2, frame=2, command=_DO_CHANGE_SPEED_CMD, current=0, autocontinue=1,
            param1=1.0, param2=_REDUCED_SPEED_MPS, param3=-1.0, param4=0.0,
            x=0, y=0, z=0.0, mission_type=0,
        ),
        MissionItem(
            seq=3, frame=6, command=_WAYPOINT_CMD, current=0, autocontinue=1,
            param1=0.0, param2=0.0, param3=0.0, param4=NAN,
            x=int(wp2_lat * 1e7), y=int(wp2_lon * 1e7), z=_CRUISE_ALT_M,
            mission_type=0,
        ),
        MissionItem(
            seq=4, frame=2, command=_RTL_CMD, current=0, autocontinue=1,
            param1=0.0, param2=0.0, param3=0.0, param4=0.0,
            x=0, y=0, z=0.0, mission_type=0,
        ),
    ]
    return items


async def _send_mission_start(system) -> None:
    await system.mavlink_direct.send_message(MavlinkMessage(
        message_name="COMMAND_LONG",
        system_id=255, component_id=1,
        target_system_id=1, target_component_id=1,
        fields_json=json.dumps({
            "target_system": 1, "target_component": 1,
            "command": _MISSION_START_CMD, "confirmation": 0,
            "param1": 0.0, "param2": 0.0, "param3": 0.0, "param4": 0.0,
            "param5": 0.0, "param6": 0.0, "param7": 0.0,
        }),
    ))


async def _get_cruise_speed_mps(system) -> float:
    """Read PX4's MPC_XY_CRUISE param; fall back to a documented default on failure."""
    try:
        async with asyncio.timeout(5.0):
            result = await system.param.get_param_float("MPC_XY_CRUISE")
            return float(result)
    except Exception as exc:
        log.warning("Could not read MPC_XY_CRUISE (%s) — assuming %.1f m/s default", exc, _DEFAULT_CRUISE_MPS)
        return _DEFAULT_CRUISE_MPS


def _timing_caveat(samples: list[tuple[float, float, float, float]], sample_duration_s: float, target_north_m: float) -> str:
    """
    Return a prefix to prepend to an assertion failure message when the
    sampling window looks like it ran out before the vehicle reached the
    mission progress the assertion actually needed — as opposed to the
    vehicle genuinely having reached that point and behaved unexpectedly.

    This distinction matters most the first time this test runs against a
    stack other than the one its timing was tuned against (see
    _CLIMB_AND_TRANSIT_ALLOWANCE_S's TUNING NOTE): a slower climb or cruise
    speed can exhaust the budget before anything interesting has had a
    chance to happen, which should be reported as "the test's own tuning was
    too tight for this stack", not "this stack fails to implement the gate".

    Heuristic: the last sample arrived within 10% of the budget's end AND the
    furthest point reached is still meaningfully short of target_north_m.
    Returns "" when nothing suggests a timing shortfall.
    """
    last_elapsed_s = samples[-1][3]
    max_north_m = max(s[0] for s in samples)
    ran_to_budget = last_elapsed_s >= sample_duration_s * 0.9
    fell_short = max_north_m < target_north_m - 5.0
    if ran_to_budget and fell_short:
        return (
            f"LIKELY INSUFFICIENT SAMPLING TIME FOR THIS STACK, not necessarily a behavioural "
            f"finding: sampling ran for {last_elapsed_s:.1f}s of a {sample_duration_s:.1f}s budget and "
            f"only reached north={max_north_m:.1f} m (needed ~{target_north_m:.1f} m). If this stack has a "
            f"slower climb or cruise speed than PX4 SIH, raise CONDITION_GATE_CLIMB_ALLOWANCE_S "
            f"(env var) and re-run before concluding anything about behaviour. "
        )
    return ""


async def _sample_track(system, home_lat: float, home_lon: float, duration_s: float) -> list[tuple[float, float, float, float]]:
    """
    Sample GLOBAL_POSITION_INT for duration_s; return a list of
    (north_m, east_m, groundspeed_mps, elapsed_s) relative to (home_lat, home_lon)
    and to the start of sampling. `elapsed_s` is diagnostic — used by
    `_timing_caveat()` and this file's own logging to distinguish "ran out of
    sampling time" from "behaved unexpectedly", and to inform future tuning of
    `_CLIMB_AND_TRANSIT_ALLOWANCE_S` — not by any assertion directly.
    """
    samples: list[tuple[float, float, float, float]] = []
    t0 = asyncio.get_event_loop().time()

    async def _collect() -> None:
        async for msg in system.mavlink_direct.message("GLOBAL_POSITION_INT"):
            fields = json.loads(msg.fields_json)
            lat = fields["lat"] / 1e7
            lon = fields["lon"] / 1e7
            vx = fields.get("vx", 0) / 100.0  # cm/s -> m/s
            vy = fields.get("vy", 0) / 100.0
            speed = math.hypot(vx, vy)
            north_m, east_m = _local_xy(lat, lon, home_lat, home_lon)
            elapsed_s = asyncio.get_event_loop().time() - t0
            samples.append((north_m, east_m, speed, elapsed_s))

    task = asyncio.create_task(_collect())
    await asyncio.sleep(duration_s)
    task.cancel()  # fire-and-forget — see CLAUDE.md §4a
    return samples


async def test_gate_compat_does_not_bend_path_and_triggers_mid_leg(gcs_system, request):
    """
    Fly wp1 -> gate (25 m off-path) -> DO_CHANGE_SPEED -> wp2 -> RTL (takeoff
    is commanded, not a mission item — see _build_mission's docstring) and
    verify, from telemetry:
      1. The flown track during the wp1->wp2 leg stays close to the direct
         line between them (the gate is not a destination — PR #761 claim 1).
      2. The track never approaches the gate's own coordinates as closely as
         it approaches the direct-line projection point (confirms the gate
         doesn't pull the route toward itself).
      3. Groundspeed drops from cruise to the commanded reduced speed at a
         point roughly matching the gate's projection onto the leg, not at
         either waypoint (the mission-blocking/trigger mechanism — PR #761
         claim 2).

    This is the definitive "is CONDITION_GATE supported at all" check for
    this file (root CLAUDE.md rule 8) — SUPPORTED if every claim above holds,
    NA (via the NACK skip below) on a stack that rejects the command outright.
    """
    home = await _get_home_position(gcs_system)
    home_lat, home_lon, home_amsl = home.latitude_deg, home.longitude_deg, home.absolute_altitude_m

    items = _build_mission(home_lat, home_lon, home_amsl)
    cruise_mps = await _get_cruise_speed_mps(gcs_system)
    log.info("CONDITION_GATE flight: cruise speed=%.2f m/s, reduced target=%.2f m/s", cruise_mps, _REDUCED_SPEED_MPS)

    leg_end_m = _LEG_START_M + _LEG_LENGTH_M
    # NOTE: this is the gate's simple projection onto the path (a naive
    # "perpendicular to the path" assumption), used below only as a rough
    # tolerance anchor. It is NOT where PX4 actually triggers: source
    # (mission_block.cpp's NAV_CMD_CONDITION_GATE branch) computes
    # dot(vehicle - gate, normalize(next_real_waypoint - gate)) >= 0 — a
    # plane through the gate perpendicular to the gate->next-waypoint vector,
    # not to the path. For this geometry that plane is crossed at
    # north~111m, ~9m before this naive 120m — see CLAUDE.md's
    # blind-source-review finding. The assertion tolerance below is wide
    # enough to not care about this, but don't mistake gate_north_m here for
    # the true trigger point if tightening it later.
    gate_north_m = _LEG_START_M + _GATE_FRACTION * _LEG_LENGTH_M

    samples: list[tuple[float, float, float, float]] = []
    try:
        try:
            await raw_upload_mission_items(gcs_system, items, mission_type=0)
        except RawMissionError as exc:
            # Tier 1 already tells us definitively whether this stack accepts
            # CONDITION_GATE at all (test_mission_item_supported) — if it doesn't
            # (e.g. ArduPilot: MAV_MISSION_UNSUPPORTED), there is no mission to
            # fly and attempting to arm/fly anyway would only ever reproduce
            # that same finding at a much higher cost. Skip, don't fail: see
            # root CLAUDE.md's "Tier 1 findings gate Tier 2 scope" convention.
            # Checked BEFORE taking off (per _build_mission's docstring, the
            # upload no longer carries a takeoff item either) so an
            # unsupported stack skips in seconds, never arming at all.
            record_tier2_param_verdict(request, "CONDITION_GATE (command)", "REJECTED (NACKed) — not implemented as a mission item on this stack")
            pytest.skip(
                f"CONDITION_GATE rejected as a mission item on this stack ({exc}) — "
                "Tier 1 (test_protocol.py::test_mission_item_supported) already establishes this; "
                "no Tier 2 flight is possible or meaningful here"
            )
        await _request_position_stream(gcs_system, rate_hz=5.0)
        await _takeoff_via_command(gcs_system, _CRUISE_ALT_M)
        await _send_mission_start(gcs_system)

        # Derived from the measured cruise speed (not assumed) so a stack
        # configured with a slower cruise speed gets proportionally more time
        # automatically — see the TUNING NOTE above _CLIMB_AND_TRANSIT_ALLOWANCE_S.
        sample_duration_s = (
            _CLIMB_AND_TRANSIT_ALLOWANCE_S
            + _LEG_START_M / cruise_mps
            + _LEG_LENGTH_M / _REDUCED_SPEED_MPS  # worst case: whole leg at the slower post-gate speed
        )
        samples = await _sample_track(gcs_system, home_lat, home_lon, sample_duration_s)
    finally:
        await _rtl_and_land(gcs_system)
        await clear_all_mission_types(gcs_system)

    assert samples, "No GLOBAL_POSITION_INT samples collected during the flight"
    log.info("CONDITION_GATE flight: %d samples, max north reached = %.1f m at t=%.1fs (budget %.1fs)",
              len(samples), max(s[0] for s in samples), samples[-1][3], sample_duration_s)

    # Restrict analysis to samples actually on the wp1->wp2 leg (between the
    # two waypoints' north coordinate, with a little slack) — takeoff/climb
    # and the final RTL climb-out are not part of the claim under test.
    leg_samples = [s for s in samples if _LEG_START_M - 5.0 <= s[0] <= leg_end_m + 5.0]
    assert leg_samples, (
        _timing_caveat(samples, sample_duration_s, _LEG_START_M) +
        f"No samples fell within the wp1->wp2 leg (north {_LEG_START_M:.0f}-{leg_end_m:.0f} m); "
        f"full sample range: north {min(s[0] for s in samples):.1f}-{max(s[0] for s in samples):.1f} m"
    )

    # --- Claim 1: track stays on the direct line, not bent toward the gate ---
    max_cross_track = max(_cross_track_distance_m(n, e, _LEG_START_M, leg_end_m) for n, e, _, _ in leg_samples)
    log.info("CONDITION_GATE flight: max cross-track deviation on leg = %.2f m (gate offset = %.1f m)",
              max_cross_track, _GATE_OFFSET_M)
    assert max_cross_track < _ON_TRACK_TOLERANCE_M, (
        f"Flown track deviated {max_cross_track:.1f} m from the direct wp1->wp2 line "
        f"(tolerance {_ON_TRACK_TOLERANCE_M:.1f} m) — the gate (offset {_GATE_OFFSET_M:.1f} m) may be "
        "bending the route, contradicting PR #761's 'not a destination' claim"
    )
    closest_to_gate = min(math.hypot(n - gate_north_m, e - _GATE_OFFSET_M) for n, e, _, _ in leg_samples)
    log.info("CONDITION_GATE flight: closest approach to gate's own coordinates = %.2f m", closest_to_gate)
    assert closest_to_gate > _GATE_APPROACH_TOLERANCE_M, (
        f"Vehicle passed within {closest_to_gate:.1f} m of the gate's own coordinates "
        f"(tolerance {_GATE_APPROACH_TOLERANCE_M:.1f} m) — suggests the gate IS being treated as a route "
        "destination, contradicting PR #761"
    )

    # --- Claim 2: speed drop occurs near the gate's leg-projection, not at a waypoint ---
    threshold = (cruise_mps + _REDUCED_SPEED_MPS) / 2.0
    drop_samples = [s for s in leg_samples if s[2] < threshold]
    assert drop_samples, (
        _timing_caveat(samples, sample_duration_s, gate_north_m) +
        f"Groundspeed never dropped below the midpoint threshold ({threshold:.2f} m/s) on the leg — "
        f"DO_CHANGE_SPEED (target {_REDUCED_SPEED_MPS:.1f} m/s) may not have executed at all; "
        f"observed speeds: {[round(s[2], 2) for s in leg_samples]}"
    )
    trigger_north_m = drop_samples[0][0]
    trigger_elapsed_s = drop_samples[0][3]
    log.info(
        "CONDITION_GATE flight: speed dropped below %.2f m/s at north=%.1f m, t=%.1fs "
        "(gate projects to %.1f m; %.1fs of the %.1fs sampling budget was unused after this — "
        "see root CLAUDE.md's Tier 2 padding convention before shrinking budgets further)",
        threshold, trigger_north_m, trigger_elapsed_s, gate_north_m,
        sample_duration_s - trigger_elapsed_s, sample_duration_s,
    )
    assert _LEG_START_M + 5.0 < trigger_north_m < leg_end_m - 5.0, (
        f"Speed drop detected at north={trigger_north_m:.1f} m, essentially at a waypoint "
        f"(leg spans {_LEG_START_M:.0f}-{leg_end_m:.0f} m) rather than mid-leg — does not look like "
        "gate-triggered blocking"
    )
    # Generous tolerance: PX4 needs some distance to decelerate to the new
    # setpoint, so the *detected* drop lags the true crossing point.
    assert abs(trigger_north_m - gate_north_m) < _LEG_LENGTH_M * 0.4, (
        f"Speed drop at north={trigger_north_m:.1f} m is far from the gate's projection "
        f"({gate_north_m:.1f} m) relative to the leg length ({_LEG_LENGTH_M:.0f} m)"
    )
    record_tier2_detail(
        request,
        f"PASS if the route stays on the direct wp1-wp2 line (max cross-track "
        f"{max_cross_track:.1f}m) and the DO_CHANGE_SPEED trigger fires mid-leg "
        f"(north={trigger_north_m:.1f}m, gate projects to {gate_north_m:.1f}m) rather than at "
        f"either waypoint — confirms mavlink-devguide PR #761's two behavioural claims",
    )
    record_tier2_param_verdict(request, "CONDITION_GATE (command)", "SUPPORTED")


# ---------------------------------------------------------------------------
# UseAltitude (param2) — conditional Tier 2 pattern
# ---------------------------------------------------------------------------
#
# Tier 1 (test_params_1_2_zeroed_on_roundtrip_px4) shows PX4 currently never
# stores UseAltitude on the round trip. Per root CLAUDE.md's Tier 2 design
# pattern #1, that observation is not a skip basis — only an actual upload
# NACK is — so this test always flies and logs the round-trip result as
# context alongside whatever the flight itself shows, rather than trusting
# "not stored" to mean "cannot possibly affect execution."

_GATE_ALT_OFFSET_M = 80.0  # deliberately far from cruise altitude — if UseAltitude
                           # gated on altitude, the vehicle (staying near cruise alt)
                           # would never satisfy an altitude-inclusive crossing test


async def _probe_use_altitude_stored(system, home_lat: float, home_lon: float) -> bool:
    """Inline Tier 1-style probe: does param2=1.0 survive a solo gate upload right now?"""
    gate_lat, gate_lon = _latlon_at(home_lat, home_lon, _LEG_START_M, _GATE_OFFSET_M)
    probe_item = MissionItem(
        seq=0, frame=5, command=_GATE_CMD, current=1, autocontinue=1,
        param1=0.0, param2=1.0, param3=NAN, param4=NAN,
        x=int(gate_lat * 1e7), y=int(gate_lon * 1e7), z=_CRUISE_ALT_M, mission_type=0,
    )
    try:
        await raw_upload_mission_items(system, [probe_item], mission_type=0)
        downloaded = await raw_download_mission_items(system, mission_type=0)
    finally:
        await clear_all_mission_types(system)
    return bool(downloaded) and abs(downloaded[0].param2 - 1.0) < 1e-6


async def test_gate_obs_use_altitude_gates_on_altitude_if_supported(gcs_system, request):
    """
    Fly a gate placed _GATE_ALT_OFFSET_M above cruise altitude with
    UseAltitude=1 (MAV_BOOL_TRUE — "include altitude") and observe whether
    DO_CHANGE_SPEED still fires. Purely observational (no assertion on which
    outcome is "correct" — nobody has seen this behaviour yet): either result
    is a real, useful finding to log and document.

    Always flies, regardless of what the inline round-trip probe below shows:
    per root CLAUDE.md's Tier 2 design pattern #1, only an actual upload NACK
    is a legitimate skip basis. Tier 1's finding that param2 doesn't survive
    the round trip is logged here as context, not treated as proof the value
    can't affect execution some other way.
    """
    home = await _get_home_position(gcs_system)
    home_lat, home_lon = home.latitude_deg, home.longitude_deg

    param2_stored = await _probe_use_altitude_stored(gcs_system, home_lat, home_lon)
    log.info(
        "Inline probe: UseAltitude (param2=1.0) %s on round trip — proceeding to fly regardless",
        "PRESERVED" if param2_stored else "NOT preserved (matches Tier 1's roundtrip finding)",
    )

    items = _build_mission(
        home_lat, home_lon, home.absolute_altitude_m,
        gate_param2=1.0, gate_alt_m=_CRUISE_ALT_M + _GATE_ALT_OFFSET_M,
    )
    cruise_mps = await _get_cruise_speed_mps(gcs_system)
    samples: list[tuple[float, float, float, float]] = []
    try:
        try:
            await raw_upload_mission_items(gcs_system, items, mission_type=0)
        except RawMissionError as exc:
            # Same NACK-skip basis as test_gate_compat_does_not_bend_path_and_
            # triggers_mid_leg — CONDITION_GATE's own command-level support is
            # already established there; this test's UseAltitude-specific
            # question is moot if the whole command isn't accepted.
            pytest.skip(
                f"NA: CONDITION_GATE rejected as a mission item on this stack ({exc}) — "
                "test_gate_compat_does_not_bend_path_and_triggers_mid_leg already establishes this"
            )
        await _request_position_stream(gcs_system, rate_hz=5.0)
        await _takeoff_via_command(gcs_system, _CRUISE_ALT_M)
        await _send_mission_start(gcs_system)
        sample_duration_s = (
            _CLIMB_AND_TRANSIT_ALLOWANCE_S + _LEG_START_M / cruise_mps + _LEG_LENGTH_M / _REDUCED_SPEED_MPS
        )
        samples = await _sample_track(gcs_system, home_lat, home_lon, sample_duration_s)
    finally:
        await _rtl_and_land(gcs_system)
        await clear_all_mission_types(gcs_system)

    threshold = (cruise_mps + _REDUCED_SPEED_MPS) / 2.0
    fired = [s for s in samples if s[2] < threshold]
    if fired:
        detail = (
            f"Observational: UseAltitude=1 with gate {_GATE_ALT_OFFSET_M:.0f}m above cruise altitude — "
            f"DO_CHANGE_SPEED fired anyway at north={fired[0][0]:.1f}m — UseAltitude has no functional "
            f"effect on the crossing test (param2_stored={param2_stored})"
        )
        log.warning(detail)
    else:
        detail = (
            f"Observational: UseAltitude=1 with gate {_GATE_ALT_OFFSET_M:.0f}m above cruise altitude — "
            f"DO_CHANGE_SPEED never fired within the budget — UseAltitude appears to gate on altitude "
            f"as mavlink-devguide PR #761 describes (param2_stored={param2_stored})"
        )
        log.warning(detail)
    record_tier2_detail(request, detail)
