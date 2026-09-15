"""
MAV_CMD_NAV_TAKEOFF (cmd=22) — Tier 2 execution tests.

Tests whether arming and starting a mission causes the vehicle to actually take off
and, for the yaw/pitch/position tests, whether the corresponding execution behaviour
is observed.  Requires a real flight stack (--drone-address).  All tests are skipped
in paired/mock mode.

Outcome vocabulary in the Tier 2 log (see tests/flight_helpers.py's _tier2_auto_record):
  PASS / FAIL — a real, direct result. "Is it honoured" tests (tracks_yaw, tracks_pitch,
    respects_position) use a genuine `assert`, not `pytest.xfail()`: even though the
    underlying pattern (an accepted, defined param silently ignored without NACKing) is
    a common, cross-stack spec violation (root CLAUDE.md rule 4), a specific test run
    against a specific stack should say so plainly — FAIL, not a softened XFAIL.
  NA — a *different* test already showed some prerequisite capability doesn't work
    (e.g. yaw isn't tracked at all), so this test's own specific question is moot.
    Showing it as FAIL would misleadingly imply it tested its own named behaviour;
    showing it as NA says plainly "nothing new was learned here". Triggered by
    `pytest.skip("NA: ...")` — see the module-level `_check_yaw_tracked`/
    `_check_pitch_tracked`/`_check_position_tracked` helpers, which fly once, cache the
    result, and are reused by every dependent test so the prerequisite (or, for
    position, the identical mission a second test also needs) is never re-flown
    redundantly.
  (Information) — a prefix on a PASS/FAIL detail marking a test that isn't measuring
    NAV_TAKEOFF's own param compliance (e.g. the implicit-takeoff convenience feature),
    so a reader doesn't mistake it for a compliance result.

Test name suffix tells you the category before you even read the detail column:
  _compat — feeds the "Compatibility summary" appended after the main table (see
    tests/flight_helpers.py's record_tier2_param_verdict) — a real PASS/FAIL/NA answer
    to "is this param supported".
  _info — PASS/FAIL, but not measuring NAV_TAKEOFF's own param compliance (e.g. the
    implicit-takeoff convenience feature) — the "(Information)" prefix in its detail
    says the same thing at the log-line level.
  _obs — characterisation only, no compatibility verdict, no assertion beyond basic
    safety — edge/out-of-range values and safety-only trajectory behaviour.

Every detail line states its own criterion explicitly ("PASS if <criterion>: <measured
values>", "NA: <why moot>", or "Observational (<why no criterion>): <measured values>")
so the log is self-explanatory without cross-referencing this file. Every test's outcome
is captured automatically (tests/flight_helpers.py's _tier2_auto_record, imported below)
and written to logs/mission_nav_takeoff_tier2_<autopilot>_<vehicle>_<version>_
<timestamp>.log after every test — same table format as the Tier 1 log.

Spec-conformance framing (root CLAUDE.md's "General testing philosophy for MAV_CMD
support" — read that section first): NAV_TAKEOFF's XML description only requires "the
vehicle takes off". Every per-param test below is either an "is it honoured" test (a
real param the command accepts — Yaw, Pitch, and, via the shared hasLocation/
isDestination convention, Lat/Lon — assert against real telemetry, FAIL if ignored) or a
characterisation test (edge/out-of-range values, sentinels, safety-only trajectory
behaviour the spec never defines — observational, no assertion beyond basic safety).
None of the "is it honoured" tests are gated on --vehicle-type: the harness can't verify
that flag against the real connected vehicle, so commanded values are chosen so the
*measured outcome itself* distinguishes supported from unsupported regardless of vehicle
type (see test_takeoff_compat_tracks_pitch).

Safety
    RTL + land are commanded in a finally block after each test.

Running
-------
Against PX4 SIH::

    pytest tests/mission/nav_takeoff/test_flight.py --drone-address=udp://:14540 -v --log-cli-level=INFO

Against ArduCopter SITL::

    pytest tests/mission/nav_takeoff/test_flight.py \\
      --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \\
      --ardupilot-sitl=~/ardu_sitl/arducopter \\
      --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 \\
      -v --log-cli-level=INFO
"""

import asyncio
import logging
import os

import pytest
from mavsdk.mission_raw import MissionItem, MissionRawError

from ..conftest import clear_all_mission_types
from tests.flight_helpers import (
    TAKEOFF_TIMEOUT_S,
    _dist_m,
    _get_heading,
    _get_home_position,
    _get_position,
    _north_of,
    _rtl_and_land,
    _tier2_auto_record,  # noqa: F401 — autouse: records every test's outcome for the Tier 2 log
    _wait_armable,
    _wait_for_altitude,
    _wait_for_altitude_with_peak_pitch,
    _wait_for_altitude_with_pitch_extremes,
    record_compat_command_supported,
    record_compat_json,
    record_tier2_detail,
    record_tier2_param_verdict,
    require_real_stack,  # noqa: F401 — registers the real-stack skip gate for this module
)

log = logging.getLogger(__name__)

# Flight tests: arming (60 s) + takeoff (90 s) + RTL/land (120 s) + margin.
pytestmark = pytest.mark.timeout(360)

# Read by tests/flight_helpers.py's flush_tier2_logs() to name/identify this module's log.
_CMD_NAME = "NAV_TAKEOFF"
_CMD_ID = 22

# Read by tests/flight_helpers.py's _render_compat_json() so every one of
# NAV_TAKEOFF's 7 mission-item params always appears in the rendered
# compatibility JSON — including a param whose Tier 2 test errored out
# (e.g. TimeoutError) before ever calling record_compat_json(), which
# otherwise renders as silently ABSENT rather than "supported": null
# ("not independently tested this run" per the schema) — see 2026-09-15
# discussion in nav_takeoff/CLAUDE.md.
_COMPAT_JSON_ALL_PARAMS = [
    "1_Pitch", "2_Empty", "3_Flags", "4_Yaw", "5_Latitude", "6_Longitude", "7_Altitude",
]

TRANSFER_TIMEOUT_S = 30.0
TAKEOFF_ALT_M      = 20.0   # metres relative to home — this file's own nominal altitude
YAW_TOLERANCE_DEG  = 20.0   # ± degrees for heading assertion
TARGET_YAW_DEG     = 137.0  # unusual value to distinguish from any default heading
# Chosen to be achievable as a genuine climb-out pitch for fixed-wing while comfortably
# exceeding what a hover-capable vehicle incidentally reaches under vertical
# acceleration alone (observed 1.1-3.4° peak |pitch| across this session's PX4 MC runs)
# — see test_takeoff_compat_tracks_pitch and root CLAUDE.md rule 5.
PITCH_TARGET_DEG = 10.0
PITCH_HONOURED_FRACTION = 0.5  # peak |pitch| must reach at least this fraction of the commanded value to count as "supported"
PITCH_OVERFLOW_RAW_DEG = 380.0  # 360° + 20° — wraps to 20° if the stack treats pitch cyclically like yaw
# Distance from home that counts as "lateral navigation has started", for the
# ascend-before-lateral-movement characterisation.
LATERAL_START_M = 5.0
# North offset for the two position/trajectory tests' explicit, non-sentinel,
# non-home NAV_TAKEOFF target.
POSITION_TARGET_OFFSET_M = 100.0
# Not yet tuned from real telemetry (test is new) — generous starting point per
# root CLAUDE.md's Tier 2 design pattern #2/#4; tighten once real runs are observed.
LOCATION_TOLERANCE_M = float(os.environ.get("NAV_TAKEOFF_LOCATION_TOLERANCE_M", "20.0"))

# Two widely-separated altitude targets for "is param7 genuinely read and used" —
# a single fixed commanded value can't distinguish "genuinely tracks param7" from
# "coincidentally near a hardcoded default/early-exit condition" (2026-09-15: a
# single-target design passed cleanly on both PX4 MC and ArduPlane FW, every real
# run, leaving that question unanswered in practice — see nav_takeoff/CLAUDE.md).
# Always fly both (no single-value test, no NA-gating) so every run produces a real
# scaling comparison. Deliberately far apart (a ~3x range straddling TAKEOFF_ALT_M)
# so genuine tracking — even allowing for a legitimate early-completion margin like
# ArduPlane's pitch level-off timeout — is clearly distinguishable from a stack that
# ignores param7 and completes at some roughly fixed altitude regardless of command.
ALTITUDE_LOW_M = 10.0
ALTITUDE_HIGH_M = 35.0
# Minimum (high - low) transition-altitude separation required to call it "tracks
# the commanded value" — well above plausible sample/climb-rate jitter between two
# runs, comfortably below the full 25m gap between ALTITUDE_LOW_M/ALTITUDE_HIGH_M
# so a stack with a real but imprecise tracking behaviour (e.g. an early-exit
# margin proportional to climb rate, not a fixed offset) still clears it.
ALTITUDE_TRACKING_MARGIN_M = 8.0

NAN = float("nan")

# Cached results of the three "is it supported at all" probes — each flies exactly once
# per session (module-level memoisation) no matter how many dependent tests consult it,
# and no matter what order/subset of tests actually runs.
_yaw_tracked_result: dict | None = None
_pitch_tracked_result: dict | None = None
_position_tracked_result: dict | None = None
# Populated by test_takeoff_obs_altitude_tracks_low; consumed by
# test_takeoff_compat_altitude_tracks_high's own scaling comparison.
_altitude_low_result: dict | None = None

# Set True by any successful explicit-NAV_TAKEOFF-item flight in this module
# (_fly_to_altitude_target or any of the three _check_*_tracked probes reaching
# altitude). test_takeoff_info_implicit_from_waypoint runs LAST in this file
# specifically so this has already been populated by the time it needs it —
# see that test's docstring for why.
_explicit_takeoff_confirmed: bool = False


# ---------------------------------------------------------------------------
# Mission builder
# ---------------------------------------------------------------------------

def _build_mission(home_item, *probes):
    """
    Build a mission item list.

    If home_item is not None (ArduCopter), prepend it at seq=0 with current=0
    and number probes from seq=1.  Otherwise number probes from seq=0.
    The first probe item gets current=1 (mission start point).
    """
    items = []
    if home_item is not None:
        items.append(MissionItem(
            seq=0, frame=home_item.frame, command=home_item.command,
            current=0, autocontinue=1,
            param1=home_item.param1, param2=home_item.param2,
            param3=home_item.param3, param4=home_item.param4,
            x=home_item.x, y=home_item.y, z=home_item.z,
            mission_type=0,
        ))
    offset = len(items)
    for i, p in enumerate(probes):
        items.append(MissionItem(
            seq=offset + i,
            frame=p.frame, command=p.command,
            current=(1 if i == 0 else 0),
            autocontinue=p.autocontinue,
            param1=p.param1, param2=p.param2,
            param3=p.param3, param4=p.param4,
            x=p.x, y=p.y, z=p.z,
            mission_type=p.mission_type,
        ))
    return items


def _takeoff_item(home, **overrides) -> MissionItem:
    """Default NAV_TAKEOFF MissionItem at home lat/lon, TAKEOFF_ALT_M relative."""
    defaults = dict(
        seq=0, frame=6, command=_CMD_ID, current=1, autocontinue=1,
        param1=15.0, param2=0.0, param3=0.0, param4=NAN,
        x=int(home.latitude_deg * 1e7), y=int(home.longitude_deg * 1e7),
        z=float(TAKEOFF_ALT_M), mission_type=0,
    )
    defaults.update(overrides)
    return MissionItem(**defaults)


# ---------------------------------------------------------------------------
# Telemetry helpers (_get_home_position, _wait_armable, _wait_for_altitude,
# _get_heading, _rtl_and_land, _wait_for_altitude_with_peak_pitch,
# _wait_for_altitude_with_pitch_extremes), require_real_stack, and the Tier 2
# auto-logging fixtures are imported from tests.flight_helpers (above) —
# generic vehicle-state scaffolding, not specific to the mission protocol.
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Inline-probe helper for conditional Tier 2 tests
# ---------------------------------------------------------------------------

async def _probe_takeoff_item(system, home_item, **overrides):
    """Upload a NAV_TAKEOFF item with given overrides, download, return the stored item.

    Performs a complete upload–download–clear cycle without flying.
    Returns (stored_item, nacked):
      stored_item — the downloaded MissionItem at the probe seq, or None if not found
      nacked      — True if the upload raised MissionRawError
    """
    defaults = dict(
        seq=0, frame=6, command=_CMD_ID, current=1, autocontinue=1,
        param1=15.0, param2=0.0, param3=0.0, param4=NAN,
        x=0, y=0, z=float(TAKEOFF_ALT_M), mission_type=0,
    )
    defaults.update(overrides)
    takeoff = MissionItem(**defaults)
    items = _build_mission(home_item, takeoff)
    probe_seq = 1 if home_item is not None else 0
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await system.mission_raw.upload_mission(items)
        downloaded = await system.mission_raw.download_mission()
        stored = next((d for d in downloaded if d.seq == probe_seq), None)
        return stored, False
    except MissionRawError:
        return None, True
    finally:
        await clear_all_mission_types(system)


# ---------------------------------------------------------------------------
# Cached "is it supported at all" probes — flown once, reused by dependent tests
# ---------------------------------------------------------------------------

def _compat_verdict(nacked: bool, ok: bool) -> str:
    """
    Correlate a Tier 1 protocol-level outcome (was the value NACKed?) with a Tier 2
    execution-level outcome (was it actually honoured?) into one compatibility-summary
    verdict. The interesting case is ACCEPTED-but-not-honoured: Tier 1 alone (a
    round-trip storage check) can look completely clean — the value may even be stored
    correctly — while Tier 2 shows it has no effect at execution. That combination is
    itself a confirmed compatibility error distinct from "not supported": the stack
    should have rejected a value it cannot act on, per root CLAUDE.md's general testing
    philosophy rule 4, rather than silently accepting it.
    """
    if nacked:
        return "REJECTED (NACKed) — correctly rejects a value it doesn't support."
    if ok:
        return "SUPPORTED"
    return "NOT SUPPORTED — COMPAT-ERR: not NACKed (see root CLAUDE.md rule 4)."


def _compat_json_fields(nacked: bool, ok: bool) -> dict:
    """
    Same (nacked, ok) correlation as _compat_verdict(), rendered as
    mavlink-compat-data schema fields instead of a human-readable string —
    for record_compat_json()'s `supported`/`nacks_on_non_sentinel_value`.
    `nacks_on_non_sentinel_value` is omitted (None) whenever `supported` is
    True, matching that schema field's own gating (see record_compat_json's
    docstring): a working param is expected to accept real values, so the
    question doesn't apply there.
    """
    if nacked:
        return {"supported": False, "nacks_on_non_sentinel_value": True}
    if ok:
        return {"supported": True, "nacks_on_non_sentinel_value": None}
    return {"supported": False, "nacks_on_non_sentinel_value": False}


async def _check_yaw_tracked(gcs_system, home_item_for_mission, restart_flight_stack) -> dict:
    """Fly once with TARGET_YAW_DEG; cache and return {'ok', 'nacked', 'heading', 'diff'}."""
    global _yaw_tracked_result
    if _yaw_tracked_result is not None:
        return _yaw_tracked_result

    home = await _get_home_position(gcs_system)
    takeoff = _takeoff_item(home, param4=TARGET_YAW_DEG)
    items = _build_mission(home_item_for_mission, takeoff)
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
    except MissionRawError:
        log.info("Yaw tracking probe: upload NACKed — correctly rejects an unsupported value")
        await clear_all_mission_types(gcs_system)
        _yaw_tracked_result = {"ok": False, "nacked": True, "heading": None, "diff": None}
        return _yaw_tracked_result

    try:
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        pos = await _wait_for_altitude(gcs_system, TAKEOFF_ALT_M * 0.85)
        global _explicit_takeoff_confirmed
        _explicit_takeoff_confirmed = True
        heading = await _get_heading(gcs_system)
        diff = abs((heading - TARGET_YAW_DEG + 180) % 360 - 180)
        result = {"ok": diff <= YAW_TOLERANCE_DEG, "nacked": False, "heading": heading, "diff": diff}
        log.info(
            "Yaw tracking probe: alt=%.1fm heading=%.1f° target=%.0f° diff=%.1f° -> %s",
            pos.relative_altitude_m, heading, TARGET_YAW_DEG, diff,
            "SUPPORTED" if result["ok"] else "NOT SUPPORTED",
        )
    finally:
        await _rtl_and_land(gcs_system, restart_flight_stack)
        await clear_all_mission_types(gcs_system)
    _yaw_tracked_result = result
    return result


async def _check_pitch_tracked(gcs_system, home_item_for_mission, restart_flight_stack) -> dict:
    """Fly once with PITCH_TARGET_DEG; cache and return {'ok', 'nacked', 'peak_pitch', 'required'}."""
    global _pitch_tracked_result
    if _pitch_tracked_result is not None:
        return _pitch_tracked_result

    home = await _get_home_position(gcs_system)
    takeoff = _takeoff_item(home, param1=PITCH_TARGET_DEG, param4=NAN)
    items = _build_mission(home_item_for_mission, takeoff)
    required = PITCH_TARGET_DEG * PITCH_HONOURED_FRACTION
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
    except MissionRawError:
        log.info("Pitch tracking probe: upload NACKed — correctly rejects an unsupported value")
        await clear_all_mission_types(gcs_system)
        _pitch_tracked_result = {"ok": False, "nacked": True, "peak_pitch": None, "required": required}
        return _pitch_tracked_result

    try:
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        pos, peak_pitch = await _wait_for_altitude_with_peak_pitch(gcs_system, TAKEOFF_ALT_M * 0.85)
        global _explicit_takeoff_confirmed
        _explicit_takeoff_confirmed = True
        result = {"ok": peak_pitch >= required, "nacked": False, "peak_pitch": peak_pitch, "required": required}
        log.info(
            "Pitch tracking probe: alt=%.1fm peak_|pitch|=%.1f° (need >= %.1f°) -> %s",
            pos.relative_altitude_m, peak_pitch, required,
            "SUPPORTED" if result["ok"] else "NOT SUPPORTED",
        )
    finally:
        await _rtl_and_land(gcs_system, restart_flight_stack)
        await clear_all_mission_types(gcs_system)
    _pitch_tracked_result = result
    return result


async def _check_position_tracked(gcs_system, home_item_for_mission, restart_flight_stack) -> dict:
    """
    Fly once with an explicit target POSITION_TARGET_OFFSET_M north of home; cache and
    return {'ok', 'nacked', 'dist_from_target', 'dist_from_home', 'samples'}.

    Shared by test_takeoff_compat_respects_position (is lat/lon honoured — final
    position) and test_takeoff_obs_ascends_before_lateral_movement (does altitude lead
    lateral movement — trajectory): both need the identical mission and differ only in
    what they read back from one flight, so flying it twice — as this file originally
    did — wasted an entire arm/climb/RTL/land cycle for no new information. Same
    caching principle as _check_yaw_tracked/_check_pitch_tracked, extended to position.
    """
    global _position_tracked_result
    if _position_tracked_result is not None:
        return _position_tracked_result

    home = await _get_home_position(gcs_system)
    target_lat_int = _north_of(home.latitude_deg, POSITION_TARGET_OFFSET_M)
    target_lon_int = int(home.longitude_deg * 1e7)
    takeoff = _takeoff_item(home, x=target_lat_int, y=target_lon_int, param4=NAN)
    items = _build_mission(home_item_for_mission, takeoff)
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
    except MissionRawError:
        log.info("Position tracking probe: upload NACKed — correctly rejects an unsupported value")
        await clear_all_mission_types(gcs_system)
        _position_tracked_result = {
            "ok": False, "nacked": True, "dist_from_target": None, "dist_from_home": None,
            "samples": [],
        }
        return _position_tracked_result

    samples: list[tuple[float, float]] = []  # (alt_rel_m, dist_from_home_m)

    async def _sample_pos() -> None:
        async for pos in gcs_system.telemetry.position():
            dist = _dist_m(pos.latitude_deg, pos.longitude_deg, home.latitude_deg, home.longitude_deg)
            samples.append((pos.relative_altitude_m, dist))

    sample_task = asyncio.create_task(_sample_pos())
    try:
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        threshold = TAKEOFF_ALT_M * 0.85
        try:
            pos = await _wait_for_altitude(gcs_system, threshold)
            global _explicit_takeoff_confirmed
            _explicit_takeoff_confirmed = True
            log.info("Altitude reached: %.1f m — sampling position", pos.relative_altitude_m)
        except TimeoutError:
            log.warning("Altitude threshold not reached — analysing samples/position collected so far")

        after = await _get_position(gcs_system)
        target_lat, target_lon = target_lat_int / 1e7, target_lon_int / 1e7
        dist_from_target = _dist_m(after.latitude_deg, after.longitude_deg, target_lat, target_lon)
        dist_from_home = _dist_m(after.latitude_deg, after.longitude_deg, home.latitude_deg, home.longitude_deg)
        result = {
            "ok": dist_from_target <= LOCATION_TOLERANCE_M, "nacked": False,
            "dist_from_target": dist_from_target, "dist_from_home": dist_from_home,
            "samples": samples,
        }
        log.info(
            "dist_from_target=%.1fm dist_from_home=%.1fm (commanded %.0fm north of home) -> %s",
            dist_from_target, dist_from_home, POSITION_TARGET_OFFSET_M,
            "SUPPORTED" if result["ok"] else "NOT SUPPORTED",
        )
    finally:
        sample_task.cancel()
        await _rtl_and_land(gcs_system, restart_flight_stack)
        await clear_all_mission_types(gcs_system)
    _position_tracked_result = result
    return result


# ---------------------------------------------------------------------------
# Altitude (param7) and Flags (param3)
# ---------------------------------------------------------------------------

async def _fly_to_altitude_target(gcs_system, home_item_for_mission, restart_flight_stack, target_alt: float) -> float:
    """
    Fly a NAV_TAKEOFF (z=target_alt) + NAV_WAYPOINT (same altitude) mission and return
    the altitude at the mission-item-transition instant — the moment
    mission_raw.mission_progress() advances past the takeoff item, the one event every
    stack fires exactly when ITS OWN "takeoff complete" criterion is satisfied,
    regardless of what happens afterward. Shared by test_takeoff_obs_altitude_tracks_
    low/test_takeoff_compat_altitude_tracks_high so both commanded values go through
    identical machinery — only target_alt differs.

    Why the transition event, not "wait for an 85% crossing, then sample again after a
    settle delay" (an earlier design): that conflates "did param7 gate takeoff
    completion" with "does whatever runs next hold that altitude" — two different
    questions, source-confirmed to actually differ (see nav_takeoff/CLAUDE.md):
    ArduPlane's verify_takeoff() marks takeoff complete on a one-way altitude-threshold
    crossing, and with nothing to fly to next in a single-item mission, ArduPlane's own
    exit_mission_callback() immediately switches to RTL — climbing toward RTL_ALTITUDE
    (a completely different, unrelated parameter) — so a fixed-delay-later sample was
    measuring RTL_ALTITUDE convergence, not param7 at all. Adding a same-altitude
    waypoint after the takeoff item removes that specific confound for every stack
    uniformly, no --vehicle-type branching needed.
    """
    home = await _get_home_position(gcs_system)
    takeoff = _takeoff_item(home, param4=NAN, z=target_alt)
    waypoint = MissionItem(
        seq=0, frame=6, command=16,  # MAV_CMD_NAV_WAYPOINT
        current=0, autocontinue=1,
        param1=0.0, param2=0.0, param3=0.0, param4=NAN,
        x=_north_of(home.latitude_deg, 50), y=int(home.longitude_deg * 1e7),
        z=target_alt, mission_type=0,
    )
    items = _build_mission(home_item_for_mission, takeoff, waypoint)
    takeoff_seq = 1 if home_item_for_mission is not None else 0
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        async with asyncio.timeout(TAKEOFF_TIMEOUT_S):
            async for progress in gcs_system.mission_raw.mission_progress():
                if progress.current > takeoff_seq:
                    break
        global _explicit_takeoff_confirmed
        _explicit_takeoff_confirmed = True
        transition_pos = await _get_position(gcs_system)
        return transition_pos.relative_altitude_m
    finally:
        await _rtl_and_land(gcs_system, restart_flight_stack)
        await clear_all_mission_types(gcs_system)


async def test_takeoff_obs_altitude_tracks_low(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    NAV_TAKEOFF param7=ALTITUDE_LOW_M — first half of the two-flight comparison that is
    this file's only test for "is altitude genuinely supported." A single commanded
    value can't tell "genuinely ignores param7" apart from "uses param7 as a target,
    but a separate, legitimate condition can end the takeoff phase before reaching
    it" — both look identical as "transition altitude below target" (see the
    2026-09-15 ArduPlane pitch-level-off finding in nav_takeoff/CLAUDE.md for a
    confirmed real example of the latter, and CLAUDE.md for why an earlier
    single-value design never actually exercised this distinction in practice — every
    real run happened to pass on its own). Commanding two widely different altitudes
    and checking whether the achieved altitude *scales* between them settles the
    question without needing any stack-specific knowledge (root CLAUDE.md rule 5's
    principle — a measured outcome that itself distinguishes support — applied to
    altitude instead of a single fixed value).

    Always flies (no NA-gating on a prior single-value result — there isn't one
    anymore). Purely the "low" data point — see test_takeoff_compat_altitude_tracks_
    high for the actual scaling comparison and its assertion; this test only asserts a
    basic safety floor (reaches *some* sane altitude, i.e. didn't simply fail to fly).
    """
    transition_alt = await _fly_to_altitude_target(gcs_system, home_item_for_mission, restart_flight_stack, ALTITUDE_LOW_M)
    global _altitude_low_result
    _altitude_low_result = {"transition_alt": transition_alt}
    log.info("Altitude tracking probe (low): commanded=%.1fm transition_alt=%.1fm", ALTITUDE_LOW_M, transition_alt)
    record_tier2_detail(
        request,
        f"Observational — one data point of a two-point scaling comparison (see "
        f"test_takeoff_compat_altitude_tracks_high for the actual assertion): "
        f"commanded {ALTITUDE_LOW_M:.0f}m, transition at {transition_alt:.1f}m",
    )
    assert transition_alt >= ALTITUDE_LOW_M * 0.3, (
        f"Vehicle barely left the ground ({transition_alt:.1f}m) with param7="
        f"{ALTITUDE_LOW_M:.0f}m commanded — basic safety floor failed, not just "
        f"imprecise tracking."
    )


async def test_takeoff_compat_altitude_tracks_high(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    NAV_TAKEOFF param7=ALTITUDE_HIGH_M — second half of the two-flight comparison (see
    test_takeoff_obs_altitude_tracks_low's docstring for the full reasoning). Always
    flies; this is the test that actually asserts the scaling claim and feeds this
    file's "param7 (Altitude)" compatibility-summary verdict.

    If test_takeoff_obs_altitude_tracks_low already ran in the same session (the
    normal case — it's defined earlier in this file, so pytest runs it first by
    default), compares this flight's transition altitude against that one's. A
    transition altitude that rises by at least ALTITUDE_TRACKING_MARGIN_M between the
    low and high commanded targets is direct, stack-agnostic evidence param7 is
    genuinely read and used — a real, decidable compatibility question (root
    CLAUDE.md rule 4's framing: an accepted param with zero observable effect on
    behaviour, at any commanded value, is a confirmed compatibility error, not just
    "imprecise"), so a FAIL here is asserted, not softened to a bare observation. If
    the low test wasn't run in this session (e.g. this test selected in isolation via
    -k), the comparison can't be made — falls back to NA for the scaling claim
    specifically, still reporting this flight's own reading.
    """
    transition_alt = await _fly_to_altitude_target(gcs_system, home_item_for_mission, restart_flight_stack, ALTITUDE_HIGH_M)
    log.info("Altitude tracking probe (high): commanded=%.1fm transition_alt=%.1fm", ALTITUDE_HIGH_M, transition_alt)

    if _altitude_low_result is None:
        record_tier2_detail(
            request,
            f"NA for the scaling comparison — test_takeoff_obs_altitude_tracks_low "
            f"didn't run in this session, so there is no low-target reading to compare "
            f"against: commanded {ALTITUDE_HIGH_M:.0f}m, transition at {transition_alt:.1f}m",
        )
        pytest.skip(
            "NA: test_takeoff_obs_altitude_tracks_low's result is unavailable (not run "
            "in this session) — can't compare, see this test's own docstring"
        )

    low_alt = _altitude_low_result["transition_alt"]
    separation = transition_alt - low_alt
    tracks = separation >= ALTITUDE_TRACKING_MARGIN_M
    verdict = (
        f"TRACKS commanded value (low={low_alt:.1f}m, high={transition_alt:.1f}m, "
        f"separation={separation:.1f}m)"
        if tracks else
        f"DOES NOT TRACK commanded value (low={low_alt:.1f}m, high={transition_alt:.1f}m, "
        f"separation={separation:.1f}m, need >= {ALTITUDE_TRACKING_MARGIN_M:.0f}m) — "
        f"genuinely ignored, not just imprecise"
    )
    log.info("Altitude tracking comparison: %s", verdict)
    record_tier2_detail(
        request,
        f"PASS if the achieved altitude scales meaningfully (>= {ALTITUDE_TRACKING_MARGIN_M:.0f}m "
        f"separation) between a low ({ALTITUDE_LOW_M:.0f}m) and high ({ALTITUDE_HIGH_M:.0f}m) "
        f"commanded target — this is the only test for whether param7 (Altitude) is "
        f"genuinely supported: {verdict}",
    )
    record_tier2_param_verdict(request, "param7 (Altitude)", "SUPPORTED" if tracks else "NOT SUPPORTED")
    record_compat_json(
        request, "7_Altitude", supported=tracks,
        notes=(
            "Tracks commanded value despite possible early completion" if tracks
            else "Ignores param7 — achieved altitude doesn't scale with commanded value"
        ),
    )
    assert tracks, (
        f"Transition altitude did not meaningfully increase between a "
        f"{ALTITUDE_LOW_M:.0f}m and a {ALTITUDE_HIGH_M:.0f}m commanded target "
        f"({low_alt:.1f}m -> {transition_alt:.1f}m, {separation:.1f}m separation, "
        f"need >= {ALTITUDE_TRACKING_MARGIN_M:.0f}m) — param7 is accepted but has no "
        f"observable effect on behaviour at any commanded value (compatibility error, "
        f"not just imprecise tracking)."
    )


async def test_takeoff_compat_flags_not_yet_covered(gcs_system, home_item_for_mission, request):
    """
    NAV_TAKEOFF param3 (Flags, NAV_TAKEOFF_FLAGS bitmask) has no Tier 2 execution test
    yet — bitmask/flag semantics were explicitly deferred when this suite was first
    designed (see nav_takeoff/README.md's parameter table). "Is it functionally
    supported" stays NA regardless — that needs real flight semantics no one has
    designed a test for yet, not just protocol round-tripping.

    Still does two lightweight, non-flying protocol probes (upload/download/clear, no
    arm/fly/RTL — negligible cost, root CLAUDE.md Tier 2 pattern #1's "no meaningful
    cost to running it anyway" exception) to record the two sentinel-value facts a
    probe alone CAN answer: does the sentinel round-trip, does a real value NACK. Same
    facts Tier 1's generic test_undefined_param_sentinel_accepted/
    test_undefined_param_nonsentinel_rejected[param2] and this file's own bespoke
    param3 tests establish in more depth in test_protocol.py's own log — duplicated
    here too (not merged across files) so the mavlink-compat-data JSON export in this
    file's own Tier 2 log is self-contained and doesn't need a separate Tier 1 run in
    the same session to be complete.
    """
    _, nacked_nan = await _probe_takeoff_item(gcs_system, home_item_for_mission, param3=NAN)
    _, nacked_val = await _probe_takeoff_item(gcs_system, home_item_for_mission, param3=1.0)
    record_compat_json(
        request, "3_Flags",
        accept_nan_or_int32max=not nacked_nan,
        nacks_on_non_sentinel_value=nacked_val,
    )
    record_tier2_param_verdict(request, "param3 (Flags)", "NOT TESTED (bitmask semantics deferred — see docstring)")
    pytest.skip("NA: param3 (Flags) execution semantics not yet designed — see docstring")


async def test_takeoff_compat_empty_sentinel(gcs_system, home_item_for_mission, request):
    """
    NAV_TAKEOFF param2 (Empty, reserved/undocumented upstream slot) — always NA for "is
    it functionally supported" (a reserved slot has no function to support), but two
    lightweight, non-flying protocol probes (see test_takeoff_compat_flags_not_yet_
    covered's docstring for why this duplicates rather than cross-references Tier 1)
    record the sentinel-value facts: does NaN round-trip, does a real value NACK.
    """
    _, nacked_nan = await _probe_takeoff_item(gcs_system, home_item_for_mission, param2=NAN)
    _, nacked_val = await _probe_takeoff_item(gcs_system, home_item_for_mission, param2=1.0)
    record_compat_json(
        request, "2_Empty",
        supported="not-applicable",
        accept_nan_or_int32max=not nacked_nan,
        nacks_on_non_sentinel_value=nacked_val,
    )
    pytest.skip("NA: param2 (Empty) is a reserved/undocumented slot — nothing to functionally support")


# ---------------------------------------------------------------------------
# Yaw (param4) — "is it supported" core test and edge-value/sentinel tests
# ---------------------------------------------------------------------------

async def test_takeoff_compat_tracks_yaw(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    NAV_TAKEOFF param4=137° — "is yaw supported" core test.

    Tests two acceptable outcomes as PASS: either the stack NACKs this value outright
    (a legitimate way to not support it — see root CLAUDE.md rule 4), or it accepts it
    and the vehicle turns to face the commanded yaw. FAIL is specifically the
    accepted-but-not-applied combination — a compatibility error, not softened to
    XFAIL: a specific run against a specific stack should report it plainly. A FAIL
    here means the edge-value yaw tests below (negative/overflow) report NA rather
    than re-discovering the same fact.
    """
    result = await _check_yaw_tracked(gcs_system, home_item_for_mission, restart_flight_stack)
    verdict = _compat_verdict(result["nacked"], result["ok"])
    if result["nacked"]:
        detail = f"PASS (NACKed) if yaw param rejects a value it can't support: {verdict}"
    else:
        detail = (
            f"PASS if yaw param is supported (heading matches commanded yaw within "
            f"±{YAW_TOLERANCE_DEG:.0f}°); FAIL implies param4 is accepted but not applied at "
            f"execution: heading={result['heading']:.1f}° target={TARGET_YAW_DEG:.0f}° "
            f"diff={result['diff']:.1f}°"
        )
    record_tier2_detail(request, detail)
    record_tier2_param_verdict(request, "param4 (Yaw)", verdict)
    record_compat_json(request, "4_Yaw", **_compat_json_fields(result["nacked"], result["ok"]))
    assert result["ok"] or result["nacked"], (
        f"Heading {result['heading']:.1f}° deviates {result['diff']:.1f}° from target "
        f"{TARGET_YAW_DEG}° (tolerance ±{YAW_TOLERANCE_DEG}°), and the upload was not "
        f"NACKed either — yaw param is accepted but not supported (compatibility error)."
    )


async def test_takeoff_compat_with_yaw_sentinel(gcs_system, home_item_for_mission, request):
    """
    NAV_TAKEOFF param4=NaN — NA: cannot be meaningfully tested via the mission protocol.

    The spec defines NaN as "use the current system yaw heading mode" — a dynamic
    outcome with no fixed target. The only way to actually confirm the sentinel is
    live-processed (rather than just recording whatever heading happens to result,
    which proves nothing on its own) is to set a distinctive fixed value first, then
    re-send with NaN, and check whether the heading changes — but a mission item
    executes once; there is no way to re-send mid-flight the way COMMAND_INT allows.
    See tests/command/nav_takeoff/test_flight.py::test_yaw_sentinel_changes_from_previous
    for that technique on the command-protocol side, where it is possible.

    NA, not a weak PASS: this is not a claim that the sentinel doesn't work, only that
    the mission protocol can't test it meaningfully — no flight is attempted.
    """
    record_tier2_param_verdict(
        request, "Yaw sentinel (NaN)",
        "NOT TESTABLE (mission protocol can't re-send mid-flight — see COMMAND_INT side)",
    )
    pytest.skip(
        "NA: NaN's effect is dynamic/context-dependent with no fixed target to check, and "
        "the mission protocol can't re-send mid-flight to test whether it's live-processed "
        "(unlike COMMAND_INT) — see docstring"
    )


async def test_takeoff_obs_with_negative_yaw(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    NAV_TAKEOFF param4=-90° — PASS if the stack either NACKs a negative yaw outright, or
    correctly tracks it to the equivalent positive heading (270°); FAIL if accepted but
    execution matches neither. NA if test_takeoff_compat_tracks_yaw already shows yaw isn't
    supported at all — edge-value handling of an untracked param would just rediscover
    that same fact.
    """
    yaw_result = await _check_yaw_tracked(gcs_system, home_item_for_mission, restart_flight_stack)
    if not yaw_result["ok"]:
        pytest.skip(
            "NA: yaw param is not supported (test_takeoff_compat_tracks_yaw) — a negative-yaw "
            "NACK-or-wrap check is moot until basic yaw tracking works"
        )

    expected_raw = -90.0
    expected_wrapped = expected_raw % 360  # 270.0

    stored, nacked = await _probe_takeoff_item(gcs_system, home_item_for_mission, param4=expected_raw)
    if nacked:
        record_tier2_detail(
            request,
            f"PASS if negative yaw is rejected (NACK) or, if accepted, execution tracks "
            f"the equivalent positive heading ({expected_wrapped:.0f}°): upload was "
            f"NACKed — a valid rejection of a negative-sign value",
        )
        return
    if stored is None:
        pytest.skip("NA: probe item not found in download — cannot determine stored value")

    home = await _get_home_position(gcs_system)
    takeoff = _takeoff_item(home, param4=expected_raw)
    items = _build_mission(home_item_for_mission, takeoff)
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        pos = await _wait_for_altitude(gcs_system, TAKEOFF_ALT_M * 0.85)
        heading = await _get_heading(gcs_system)
        diff = abs((heading - expected_wrapped + 180) % 360 - 180)
        ok = diff <= YAW_TOLERANCE_DEG
        record_tier2_detail(
            request,
            f"PASS if negative yaw is rejected (NACK) or execution tracks the equivalent "
            f"positive heading ({expected_wrapped:.0f}°) within ±{YAW_TOLERANCE_DEG:.0f}°: "
            f"accepted, heading={heading:.1f}° diff={diff:.1f}°",
        )
        assert ok, (
            f"param4=-90° was accepted (not NACKed) but heading {heading:.1f}° matches "
            f"neither the equivalent positive heading ({expected_wrapped:.0f}°) nor was "
            f"the upload rejected."
        )
    finally:
        await _rtl_and_land(gcs_system, restart_flight_stack)
        await clear_all_mission_types(gcs_system)


async def test_takeoff_obs_with_overflow_yaw(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    NAV_TAKEOFF param4=450° (360°+90°) — PASS if the stack either NACKs an out-of-range
    yaw, or correctly wraps it to the equivalent heading (90°); FAIL if accepted but
    execution matches neither. Same NA-gating as test_takeoff_obs_with_negative_yaw — see
    its docstring.
    """
    yaw_result = await _check_yaw_tracked(gcs_system, home_item_for_mission, restart_flight_stack)
    if not yaw_result["ok"]:
        pytest.skip(
            "NA: yaw param is not supported (test_takeoff_compat_tracks_yaw) — an overflow-yaw "
            "NACK-or-wrap check is moot until basic yaw tracking works"
        )

    expected_raw = 450.0
    expected_wrapped = expected_raw % 360  # 90.0

    stored, nacked = await _probe_takeoff_item(gcs_system, home_item_for_mission, param4=expected_raw)
    if nacked:
        record_tier2_detail(
            request,
            f"PASS if an over-360° yaw is rejected (NACK) or, if accepted, execution "
            f"tracks the wrapped equivalent ({expected_wrapped:.0f}°): upload was NACKed "
            f"— a valid rejection of an out-of-range value",
        )
        return
    if stored is None:
        pytest.skip("NA: probe item not found in download — cannot determine stored value")

    home = await _get_home_position(gcs_system)
    takeoff = _takeoff_item(home, param4=expected_raw)
    items = _build_mission(home_item_for_mission, takeoff)
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        pos = await _wait_for_altitude(gcs_system, TAKEOFF_ALT_M * 0.85)
        heading = await _get_heading(gcs_system)
        diff = abs((heading - expected_wrapped + 180) % 360 - 180)
        ok = diff <= YAW_TOLERANCE_DEG
        record_tier2_detail(
            request,
            f"PASS if an over-360° yaw is rejected (NACK) or execution wraps to the "
            f"equivalent heading ({expected_wrapped:.0f}°) within ±{YAW_TOLERANCE_DEG:.0f}°: "
            f"accepted, heading={heading:.1f}° diff={diff:.1f}°",
        )
        assert ok, (
            f"param4=450° was accepted (not NACKed) but heading {heading:.1f}° matches "
            f"neither the wrapped equivalent ({expected_wrapped:.0f}°) nor was the upload "
            f"rejected."
        )
    finally:
        await _rtl_and_land(gcs_system, restart_flight_stack)
        await clear_all_mission_types(gcs_system)


# ---------------------------------------------------------------------------
# Pitch (param1) — "is it supported" core test and edge-value/sentinel tests
# ---------------------------------------------------------------------------

async def test_takeoff_compat_tracks_pitch(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    NAV_TAKEOFF param1=10° — "is pitch supported" core test. Same shape as
    test_takeoff_compat_tracks_yaw — see its docstring for the FAIL-not-xfail reasoning and
    the NA-gating it enables for the edge-value pitch tests below.

    10° is chosen so the measured outcome alone distinguishes "supported" from "not
    supported" regardless of vehicle type, without gating on --vehicle-type (root
    CLAUDE.md's general philosophy rule 5 — the harness can't verify that flag against
    the real connected vehicle): it's an achievable, plausible climb-out pitch for a
    fixed-wing aircraft, yet comfortably exceeds what a hover-capable vehicle reaches
    incidentally under vertical acceleration alone (observed 1.1-3.4° peak in this
    session's PX4 MC runs).
    Tests two acceptable outcomes as PASS: either the stack NACKs this value outright,
    or it accepts it and the vehicle attempts to match it. FAIL is specifically the
    accepted-but-not-applied combination — a compatibility error (root CLAUDE.md rule 4).
    """
    result = await _check_pitch_tracked(gcs_system, home_item_for_mission, restart_flight_stack)
    verdict = _compat_verdict(result["nacked"], result["ok"])
    if result["nacked"]:
        detail = f"PASS (NACKed) if pitch param rejects a value it can't support: {verdict}"
    else:
        detail = (
            f"PASS if pitch param is supported (peak |pitch| reaches >= "
            f"{result['required']:.1f}°, {PITCH_HONOURED_FRACTION:.0%} of commanded "
            f"{PITCH_TARGET_DEG:.0f}°); FAIL implies param1 is accepted but not applied at "
            f"execution: peak_|pitch|={result['peak_pitch']:.1f}°"
        )
    record_tier2_detail(request, detail)
    record_tier2_param_verdict(request, "param1 (Pitch)", verdict)
    record_compat_json(request, "1_Pitch", **_compat_json_fields(result["nacked"], result["ok"]))
    assert result["ok"] or result["nacked"], (
        f"Peak |pitch| {result['peak_pitch']:.1f}° is below the required "
        f"{result['required']:.1f}°, and the upload was not NACKed either — pitch param "
        f"is accepted but not supported (compatibility error)."
    )


async def test_takeoff_compat_with_pitch_sentinel(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    NAV_TAKEOFF param1=NaN — characterisation only, no assertion.

    The spec gives param1 no sentinel meaning (unlike param4's NaN). Some stacks reject
    NaN here (ArduPilot's sanity_check_params — see nav_takeoff/CLAUDE.md); this test
    only runs when Tier 1 shows it accepted (an inline probe decides), and then logs
    whatever peak pitch results — purely observational.
    """
    stored, nacked = await _probe_takeoff_item(gcs_system, home_item_for_mission, param1=NAN)
    if nacked:
        record_tier2_param_verdict(request, "Pitch sentinel (NaN)", "REJECTED (NACKed — spec violation, see nav_takeoff/CLAUDE.md)")
        record_compat_json(request, "1_Pitch", accept_nan_or_int32max=False)
        pytest.skip("param1=NaN NACKed — protocol-level result captured by Tier 1 test_protocol.py")
    if stored is None:
        pytest.skip("Probe item not found in download")

    home = await _get_home_position(gcs_system)
    takeoff = _takeoff_item(home, param1=NAN, param4=NAN)
    items = _build_mission(home_item_for_mission, takeoff)
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        pos, peak_pitch = await _wait_for_altitude_with_peak_pitch(gcs_system, TAKEOFF_ALT_M * 0.85)
        detail = (
            f"Observational (param1 has no defined sentinel meaning, unlike param4's NaN): "
            f"alt={pos.relative_altitude_m:.1f}m peak_|pitch|={peak_pitch:.1f}°"
        )
        log.info("param1=NaN sentinel: %s", detail)
        record_tier2_detail(request, detail)
        record_tier2_param_verdict(request, "Pitch sentinel (NaN)", "ACCEPTED (no defined meaning to verify against)")
        record_compat_json(request, "1_Pitch", accept_nan_or_int32max=True)
    finally:
        await _rtl_and_land(gcs_system, restart_flight_stack)
        await clear_all_mission_types(gcs_system)


async def test_takeoff_obs_with_large_pitch(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    NAV_TAKEOFF param1=89° — characterises whether an extreme commanded pitch is
    clamped to a lower value or tracked close to raw. NA if pitch isn't supported at
    all (test_takeoff_compat_tracks_pitch). Observational otherwise (the spec defines no range
    to assert a "correct" clamp value against), but still asserts the basic safety
    property that the vehicle reaches altitude despite the extreme value.
    """
    pitch_result = await _check_pitch_tracked(gcs_system, home_item_for_mission, restart_flight_stack)
    if not pitch_result["ok"]:
        pytest.skip(
            "NA: pitch param is not supported (test_takeoff_compat_tracks_pitch) — clamping "
            "behaviour is moot until basic pitch tracking works"
        )

    expected_raw = 89.0
    home = await _get_home_position(gcs_system)
    takeoff = _takeoff_item(home, param1=expected_raw, param4=NAN)
    items = _build_mission(home_item_for_mission, takeoff)
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        threshold = TAKEOFF_ALT_M * 0.85
        pos, min_pitch, max_pitch = await _wait_for_altitude_with_pitch_extremes(gcs_system, threshold)
        if max_pitch >= expected_raw * 0.85:
            classification = f"TRACKS RAW (peak {max_pitch:.1f}° close to commanded {expected_raw:.0f}°)"
        else:
            classification = f"CLAMPED (peak {max_pitch:.1f}°, well below commanded {expected_raw:.0f}°)"
        log.info("alt=%.1fm %s", pos.relative_altitude_m, classification)
        record_tier2_detail(
            request,
            f"Observational (spec defines no range for param1, so no 'correct' clamp "
            f"value to assert against) plus a safety check that altitude is still "
            f"reached: {classification}, alt={pos.relative_altitude_m:.1f}m",
        )
        assert pos.relative_altitude_m >= threshold, (
            f"Vehicle did not reach {threshold:.1f} m with param1=89°. "
            "Stack may be misinterpreting a large pitch as an invalid command."
        )
    finally:
        await _rtl_and_land(gcs_system, restart_flight_stack)
        await clear_all_mission_types(gcs_system)


async def test_takeoff_obs_with_negative_pitch(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    NAV_TAKEOFF param1=-10° — PASS if the vehicle actually pitches nose-down (signed
    peak pitch is negative and reaches at least PITCH_HONOURED_FRACTION of the
    commanded magnitude), not just "reaches some pitch magnitude of either sign". NA if
    pitch isn't supported at all (test_takeoff_compat_tracks_pitch).
    """
    pitch_result = await _check_pitch_tracked(gcs_system, home_item_for_mission, restart_flight_stack)
    if not pitch_result["ok"]:
        pytest.skip(
            "NA: pitch param is not supported (test_takeoff_compat_tracks_pitch) — direction "
            "is moot until basic pitch tracking works"
        )

    expected_raw = -10.0
    required = abs(expected_raw) * PITCH_HONOURED_FRACTION
    home = await _get_home_position(gcs_system)
    takeoff = _takeoff_item(home, param1=expected_raw, param4=NAN)
    items = _build_mission(home_item_for_mission, takeoff)
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        threshold = TAKEOFF_ALT_M * 0.85
        pos, min_pitch, max_pitch = await _wait_for_altitude_with_pitch_extremes(gcs_system, threshold)
        ok = min_pitch <= -required
        log.info(
            "alt=%.1fm min_pitch=%.1f° max_pitch=%.1f° (need min <= -%.1f°)",
            pos.relative_altitude_m, min_pitch, max_pitch, required,
        )
        record_tier2_detail(
            request,
            f"PASS if a negative (nose-down) pitch is commanded and the vehicle actually "
            f"pitches nose-down (signed min pitch <= -{required:.1f}°): "
            f"min_pitch={min_pitch:.1f}° max_pitch={max_pitch:.1f}°",
        )
        if not ok:
            # A durable, terse "name the exception" fact per mavlink-compat-data's own
            # notes convention — pitch is otherwise confirmed supported (we only reach
            # this point when test_takeoff_compat_tracks_pitch's own probe was ok), so
            # this specifically means positive-only, not "not supported" outright.
            record_compat_json(request, "1_Pitch", notes="Positive pitch only, doesn't invert for negative")
        assert ok, (
            f"Commanded param1=-10° (nose-down) but the most negative pitch observed was "
            f"{min_pitch:.1f}°, short of the required -{required:.1f}° — negative pitch is "
            f"accepted but not applied in the correct direction."
        )
    finally:
        await _rtl_and_land(gcs_system, restart_flight_stack)
        await clear_all_mission_types(gcs_system)


async def test_takeoff_obs_with_pitch_overflow(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    NAV_TAKEOFF param1=380° (360°+20°) — characterises whether an out-of-physical-range
    pitch value wraps to its 360°-modulo equivalent (20°), the same question
    test_takeoff_obs_with_overflow_yaw asks for yaw. NA if pitch isn't supported at all.
    Observational: unlike yaw's heading, there is no spec basis for pitch to be a
    cyclic 0-360° quantity, so no assertion beyond the basic safety check that altitude
    is still reached.
    """
    pitch_result = await _check_pitch_tracked(gcs_system, home_item_for_mission, restart_flight_stack)
    if not pitch_result["ok"]:
        pytest.skip(
            "NA: pitch param is not supported (test_takeoff_compat_tracks_pitch) — wrap "
            "behaviour is moot until basic pitch tracking works"
        )

    expected_raw = PITCH_OVERFLOW_RAW_DEG
    expected_wrapped = expected_raw % 360  # 20.0
    home = await _get_home_position(gcs_system)
    takeoff = _takeoff_item(home, param1=expected_raw, param4=NAN)
    items = _build_mission(home_item_for_mission, takeoff)
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        threshold = TAKEOFF_ALT_M * 0.85
        pos, min_pitch, max_pitch = await _wait_for_altitude_with_pitch_extremes(gcs_system, threshold)
        if abs(max_pitch - expected_wrapped) <= 3.0:
            classification = f"WRAPPED (peak {max_pitch:.1f}° close to the 360°-modulo equivalent {expected_wrapped:.0f}°)"
        elif max_pitch > expected_wrapped:
            classification = f"NOT WRAPPED (peak {max_pitch:.1f}°, tracks toward raw {expected_raw:.0f}° or is clamped above {expected_wrapped:.0f}°)"
        else:
            classification = f"ALTERED (peak {max_pitch:.1f}°, matches neither raw nor wrapped)"
        log.info("alt=%.1fm %s", pos.relative_altitude_m, classification)
        record_tier2_detail(
            request,
            f"Observational (pitch is not inherently a cyclic 0-360° quantity, so "
            f"wrapping is not spec-mandated the way yaw's heading is) plus a safety "
            f"check that altitude is still reached: {classification}",
        )
        assert pos.relative_altitude_m >= threshold, f"Vehicle did not reach {threshold:.1f} m with param1=380°."
    finally:
        await _rtl_and_land(gcs_system, restart_flight_stack)
        await clear_all_mission_types(gcs_system)


# ---------------------------------------------------------------------------
# Location (param5/6 = x/y) — sentinel, explicit-target, and trajectory tests
# ---------------------------------------------------------------------------

async def test_takeoff_compat_from_current_position(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    NAV_TAKEOFF with lat/lon = INT32_MAX — PASS if lat/lon set to the sentinel value
    are within LOCATION_TOLERANCE_M of the vehicle's position at takeoff.

    INT32_MAX is the sentinel meaning "use current vehicle position". This test does
    not assume the sentinel survives storage: it reads back whatever x/y the round trip
    reports and derives an expected touchdown location from *that* observation —
    sentinel still INT32_MAX → expect "current position" (home); anything else →
    expect the concrete lat/lon the probe actually saw. Only an upload NACK, or the
    probe item being absent from the download, is a legitimate reason to skip.

    Note: this assumes the takeoff point is fixed at the moment NAV_TAKEOFF executes —
    on a moving base (ship/vehicle launch) the vehicle would still be expected to track
    the position captured at that instant, not continue following the (now-moving)
    origin. Not tested here — no scenario in this suite simulates a moving launch point.

    Distinct from test_takeoff_info_implicit_from_waypoint (no NAV_TAKEOFF item at all, never
    checks position) and test_takeoff_compat_respects_position (an explicit, real — not
    sentinel — target position).
    """
    INT32_MAX = 0x7FFF_FFFF

    stored, nacked = await _probe_takeoff_item(
        gcs_system, home_item_for_mission, x=INT32_MAX, y=INT32_MAX
    )
    if nacked:
        record_compat_json(request, "5_Latitude", accept_nan_or_int32max=False)
        record_compat_json(request, "6_Longitude", accept_nan_or_int32max=False)
        pytest.skip("INT32_MAX lat/lon NACKed — result captured by Tier 1 test_protocol_location_current_position")
    if stored is None:
        pytest.skip("Probe item not found in download")
    record_compat_json(request, "5_Latitude", accept_nan_or_int32max=True)
    record_compat_json(request, "6_Longitude", accept_nan_or_int32max=True)

    home = await _get_home_position(gcs_system)
    if stored.x == INT32_MAX and stored.y == INT32_MAX:
        expected_lat, expected_lon = home.latitude_deg, home.longitude_deg
        log.info(
            "INT32_MAX sentinel preserved — proceeding to execution check "
            "(expect takeoff near home %.7f, %.7f)", expected_lat, expected_lon,
        )
    else:
        expected_lat, expected_lon = stored.x / 1e7, stored.y / 1e7
        log.info(
            "Sentinel altered on storage (stored x=%d, y=%d) — proceeding to execution "
            "check against the stored coordinate (%.7f, %.7f), not the sentinel's meaning",
            stored.x, stored.y, expected_lat, expected_lon,
        )

    takeoff = _takeoff_item(home, x=INT32_MAX, y=INT32_MAX, param4=NAN)
    items = _build_mission(home_item_for_mission, takeoff)
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        threshold = TAKEOFF_ALT_M * 0.85
        pos = await _wait_for_altitude(gcs_system, threshold)
        log.info("Altitude reached: %.1f m", pos.relative_altitude_m)
        assert pos.relative_altitude_m >= threshold, f"Vehicle did not reach {threshold:.1f} m."

        after = await _get_position(gcs_system)
        dist = _dist_m(after.latitude_deg, after.longitude_deg, expected_lat, expected_lon)
        log.info(
            "Horizontal offset from expected target: %.1f m (tolerance %.1f m)",
            dist, LOCATION_TOLERANCE_M,
        )
        record_tier2_detail(
            request,
            f"PASS if lat/lon set to the sentinel value are within {LOCATION_TOLERANCE_M:.0f}m "
            f"of the vehicle's position at takeoff: alt={pos.relative_altitude_m:.1f}m "
            f"offset={dist:.1f}m",
        )
        record_tier2_param_verdict(
            request, "Location sentinel (INT32_MAX)",
            "SUPPORTED" if dist <= LOCATION_TOLERANCE_M else "NOT SUPPORTED",
        )
        assert dist <= LOCATION_TOLERANCE_M, (
            f"Vehicle is {dist:.1f} m from the expected target ({expected_lat:.7f}, "
            f"{expected_lon:.7f}), exceeding tolerance {LOCATION_TOLERANCE_M:.1f} m. "
            f"Execution does not reflect the location Tier 1 observed stored."
        )
    finally:
        await _rtl_and_land(gcs_system, restart_flight_stack)
        await clear_all_mission_types(gcs_system)


async def test_takeoff_compat_respects_position(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    NAV_TAKEOFF with an explicit, real (non-sentinel) lat/lon POSITION_TARGET_OFFSET_M
    north of home — PASS if lat/lon params are supported: the vehicle travels to the
    commanded point within LOCATION_TOLERANCE_M.

    NAV_TAKEOFF's own prose is silent on what lat/lon mean, but the MAVLink XML schema
    tags params 5/6 with the same hasLocation/isDestination convention shared by every
    location-bearing command including NAV_WAYPOINT — read under that shared
    convention, "go here" is the natural meaning, the same as a waypoint's own
    destination semantics.

    Tests two acceptable outcomes as PASS: either the stack NACKs this value outright
    (a legitimate way to not support it — see root CLAUDE.md rule 4), or it accepts it
    and the vehicle travels to the commanded point. FAIL is specifically the
    accepted-but-not-honoured combination — a compatibility error, not left as an
    unassertable characterisation — the same shape as the yaw/pitch "is it honoured"
    tests above.

    Shares its flight with test_takeoff_obs_ascends_before_lateral_movement via
    _check_position_tracked (same mission, so flying it twice would just burn an extra
    arm/climb/RTL/land cycle for no new information — see that helper's docstring).
    """
    result = await _check_position_tracked(gcs_system, home_item_for_mission, restart_flight_stack)
    verdict = _compat_verdict(result["nacked"], result["ok"])
    if result["nacked"]:
        detail = f"PASS (NACKed) if lat/lon params reject a value they can't support: {verdict}"
    else:
        detail = (
            f"PASS if lat/lon params are supported — vehicle travels to the commanded "
            f"point within {LOCATION_TOLERANCE_M:.0f}m (treated as a destination under "
            f"the shared hasLocation/isDestination convention, same as a waypoint); FAIL "
            f"implies param5/6 is accepted but not applied at execution: "
            f"dist_from_target={result['dist_from_target']:.1f}m "
            f"dist_from_home={result['dist_from_home']:.1f}m "
            f"(commanded {POSITION_TARGET_OFFSET_M:.0f}m north of home)"
        )
    record_tier2_detail(request, detail)
    record_tier2_param_verdict(request, "param5/6 (Lat/Lon)", verdict)
    fields = _compat_json_fields(result["nacked"], result["ok"])
    record_compat_json(request, "5_Latitude", **fields)
    record_compat_json(request, "6_Longitude", **fields)
    assert result["ok"] or result["nacked"], (
        f"Vehicle is {result['dist_from_target']:.1f} m from the commanded target "
        f"({result['dist_from_home']:.1f} m from home instead), exceeding tolerance "
        f"{LOCATION_TOLERANCE_M:.1f} m, and the upload was not NACKed either — lat/lon "
        f"accepted but not honoured as a destination (compatibility error)."
    )


async def test_takeoff_obs_ascends_before_lateral_movement(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    NAV_TAKEOFF with an explicit target POSITION_TARGET_OFFSET_M north of home — PASS
    if the vehicle ascends to some altitude before beginning lateral movement toward
    the target. Observational safety characterisation, not a spec requirement (root
    CLAUDE.md's General testing philosophy rule 6) — any outcome is protocol-valid, so
    no assertion is made regardless of what's observed.

    Not gated on --vehicle-type: a vehicle physically incapable of isolating vertical
    motion (most fixed-wing) will simply show lateral movement starting at or near
    ground level, which is itself the informative, correctly-classified result.

    Shares its flight with test_takeoff_compat_respects_position via
    _check_position_tracked — same mission, so this reads the trajectory samples from
    whichever test flew it first rather than flying its own duplicate.
    """
    result = await _check_position_tracked(gcs_system, home_item_for_mission, restart_flight_stack)
    if result["nacked"]:
        pytest.skip(
            "NA: the target-position upload was NACKed (see "
            "test_takeoff_compat_respects_position) — no takeoff was attempted, so "
            "there is no ascent-order trajectory to characterise"
        )

    samples = result["samples"]
    min_ascent_m = None
    for alt, dist in samples:
        if dist > LATERAL_START_M:
            min_ascent_m = alt
            break

    if min_ascent_m is not None:
        classification = (
            f"ascended to {min_ascent_m:.2f}m before lateral movement "
            f"(>{LATERAL_START_M:.0f}m from home) was first detected"
        )
    else:
        classification = f"no lateral movement (>{LATERAL_START_M:.0f}m from home) detected in {len(samples)} samples"

    detail = (
        f"Observational safety characterisation, not a spec requirement — any outcome "
        f"is protocol-valid: {classification} (target {POSITION_TARGET_OFFSET_M:.0f}m "
        f"north of home)"
    )
    log.info(detail)
    record_tier2_detail(request, detail)


# ---------------------------------------------------------------------------
# Baseline takeoff — deliberately last, not first: see docstring below.
# ---------------------------------------------------------------------------

async def test_takeoff_info_implicit_from_waypoint(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """
    (Information) A waypoint-only mission triggers an implicit takeoff.

    Uploads a single NAV_WAYPOINT 50 m north of the vehicle's home position at
    TAKEOFF_ALT_M relative altitude — no explicit NAV_TAKEOFF item is present at all.
    Not a NAV_TAKEOFF param-compliance test (hence the "(Information)" tag) — it
    characterises a convenience feature some stacks provide, not the command itself.

    Deliberately runs LAST in this file (not first, despite being the conceptual
    "baseline") so that by the time it runs, every explicit-NAV_TAKEOFF-item test
    above has already had its chance to fly. If this test fails to reach altitude but
    `_explicit_takeoff_confirmed` is True (some other test in this file DID reach
    altitude using an explicit NAV_TAKEOFF item), that is direct, positive evidence
    this frame *requires* an explicit NAV_TAKEOFF item to launch — stated explicitly
    in the detail line rather than left as an ambiguous timeout indistinguishable from
    "this frame doesn't take off at all, for some other reason". If nothing else in
    the file got airborne either, a more cautious "inconclusive" framing is used
    instead, since this test's own failure doesn't isolate the cause on its own then.

    PASS criterion: the vehicle still climbs to at least 85% of TAKEOFF_ALT_M within
    TAKEOFF_TIMEOUT_S seconds. This test only checks altitude — it does NOT verify the
    horizontal trajectory taken to get there; see test_takeoff_obs_ascends_before_
    lateral_movement and test_takeoff_compat_respects_position for that, using an
    explicit NAV_TAKEOFF item instead.
    """
    home = await _get_home_position(gcs_system)

    waypoint = MissionItem(
        seq=0,
        frame=6,        # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        command=16,     # MAV_CMD_NAV_WAYPOINT
        current=1,
        autocontinue=1,
        param1=0.0, param2=0.0, param3=0.0, param4=NAN,
        x=_north_of(home.latitude_deg, 50),
        y=int(home.longitude_deg * 1e7),
        z=float(TAKEOFF_ALT_M),
        mission_type=0,
    )
    items = _build_mission(home_item_for_mission, waypoint)

    threshold = TAKEOFF_ALT_M * 0.85
    reached: float | None = None
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
        log.info("Mission uploaded (%d items); waiting for armable", len(items))

        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        log.info("Armed and mission started — waiting for altitude %.1f m", threshold)

        try:
            pos = await _wait_for_altitude(gcs_system, threshold)
            reached = pos.relative_altitude_m
            log.info("Altitude reached: %.1f m relative", reached)
        except TimeoutError:
            reached = None
    finally:
        await _rtl_and_land(gcs_system, restart_flight_stack)
        await clear_all_mission_types(gcs_system)

    ok = reached is not None and reached >= threshold
    command_notes = None
    if ok:
        detail = (
            f"(Information) PASS if mission uses the WAYPOINT's altitude for takeoff "
            f"when NAV_TAKEOFF is not present: alt={reached:.1f}m (threshold {threshold:.1f}m)"
        )
    elif _explicit_takeoff_confirmed:
        detail = (
            f"(Information) FAIL: the waypoint-only mission never reached {threshold:.1f}m, "
            f"but at least one other test in this file DID reach altitude using an "
            f"explicit NAV_TAKEOFF item — TAKEOFF item is required for this frame; a "
            f"waypoint-only mission does not trigger an implicit takeoff the way it "
            f"does on some other stacks"
        )
        # Durable, frame-wide qualitative fact — exactly what mavlink-compat-data's
        # own "notes" convention wants (terse, names the exception): this frame
        # needs an explicit NAV_TAKEOFF item, unlike a stack with the implicit-
        # takeoff-from-first-waypoint convenience feature (e.g. PX4 MC).
        command_notes = "Requires explicit NAV_TAKEOFF item; no implicit takeoff from waypoint"
    else:
        detail = (
            f"(Information) FAIL: the waypoint-only mission never reached {threshold:.1f}m"
            f"{f' (last reading {reached:.1f}m)' if reached is not None else ''}, and no "
            f"other test in this file reached altitude either — inconclusive whether "
            f"this frame specifically requires an explicit NAV_TAKEOFF item, or has a "
            f"broader takeoff problem"
        )
    if ok or _explicit_takeoff_confirmed:
        # Command-level "does it take off at all" fact (root CLAUDE.md rule 1)
        # — only ever recorded True: a single test's own failure doesn't
        # prove the command never works, so False is never asserted here.
        record_compat_command_supported(request, True, notes=command_notes)
    record_tier2_detail(request, detail)
    assert ok, detail
