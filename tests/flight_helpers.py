"""
Generic Tier 2 (flight/execution) test helpers — vehicle-state scaffolding
shared across both the command-protocol and mission-protocol test trees.

Deliberately protocol-neutral (lives at tests/, not tests/command/ or
tests/mission/): both trees need "get a real vehicle into a flyable state and
observe telemetry" as a precondition for testing their own, genuinely
command/mission-specific execution semantics, and keeping that split by
protocol would make one tree depend on the other for no reason.

Consolidates what used to be three independently-drifted copies of this same
scaffolding: tests/command/nav_takeoff/test_flight.py (source of most of
what's here), tests/command/do_reposition/test_flight.py, and
tests/mission/nav_takeoff/test_flight.py — the latter two had each
reimplemented arm/wait/observe logic from scratch rather than importing it,
and had drifted (most notably two incompatible _dist_m() formulas; see its
docstring below). Callers needing NAV_TAKEOFF only as a way to reach a
flyable state (not testing NAV_TAKEOFF itself) should use
_arm_and_send_takeoff(); a command's own execution-semantics tests (touchdown
analysis, DO_JUMP tallying, WIND_COV comparison, etc.) stay in that command's
own test_flight.py — nothing command-specific belongs here.
"""

import asyncio
import json
import logging
import math
import re
import time as _time_m
from pathlib import Path

import pytest
from mavsdk.mavlink_direct import MavlinkMessage
from mavsdk.telemetry import LandedState

from tests.command.conftest import probe_command_long, send_command_int
from tests.conftest import _format_autopilot_header

log = logging.getLogger(__name__)

ARMABLE_TIMEOUT_S = 60.0
TAKEOFF_ALT_M = 30.0        # metres relative — nominal altitude for _arm_and_send_takeoff()
TAKEOFF_TIMEOUT_S = 90.0    # default timeout for _wait_for_altitude()
RTL_LAND_TIMEOUT_S = 120.0
# Shorter wait used by _rtl_and_land() whenever a restart_flight_stack fallback
# is available: if the vehicle is actually going to reach ON_GROUND, it does
# so quickly (observed: PX4 MC's own successful RTL+land, this session, well
# under 30s) -- waiting out the full RTL_LAND_TIMEOUT_S before falling back
# to a restart only makes sense when there's no fallback to fall back TO.
# Tuned from a real, measured negative: 2026-09-15's ArduPlane FW run showed
# 0 of ~9 flights ever reached ON_GROUND within 120s (no RTL_AUTOLAND/
# DO_LAND_START — see root CLAUDE.md Tier 2 pattern #8), so every one of
# those paid the full 120s before restarting; this constant reclaims that
# time for any stack/config in the same situation, going forward.
RTL_LAND_TIMEOUT_WITH_RESTART_S = 30.0
AIRBORNE_THRESHOLD_M = 2.0  # metres — confirms the vehicle left the ground

_NAV_TAKEOFF_CMD_ID = 22  # MAV_CMD_NAV_TAKEOFF

# SITL self-recovery cooldown state (module-level: the concern is "has any
# caller rebooted the SITL recently", not per-command).
_REBOOT_COOLDOWN_S: float = 300.0
_last_sitl_reboot_ts: float = 0.0


@pytest.fixture(autouse=True)
def require_real_stack(request):
    """Skip every test in a module using this fixture when no --drone-address is given."""
    if request.config.getoption("--drone-address") is None:
        pytest.skip("Execution tests require a real flight stack (--drone-address not set)")


# ---------------------------------------------------------------------------
# Telemetry request helpers
# ---------------------------------------------------------------------------


async def _request_home_position(system) -> None:
    """
    Request HOME_POSITION (msg id=242) from the autopilot once.

    ArduCopter does not re-broadcast HOME_POSITION automatically after the
    initial connect. MAVSDK's telemetry.home() uses this message. After many
    function-scoped System reconnects within the same session, the stream may
    have gone stale. MAV_CMD_REQUEST_MESSAGE (512) fetches one fresh copy.
    """
    await system.mavlink_direct.send_message(MavlinkMessage(
        message_name="COMMAND_LONG",
        system_id=255, component_id=1,
        target_system_id=1, target_component_id=0,
        fields_json=json.dumps({
            "target_system": 1, "target_component": 0,
            "command": 512,      # MAV_CMD_REQUEST_MESSAGE
            "param1": 242.0,    # MAVLINK_MSG_ID_HOME_POSITION
            "param2": 0.0, "param3": 0.0, "param4": 0.0,
            "param5": 0.0, "param6": 0.0, "param7": 0.0,
            "confirmation": 0,
        }),
    ))
    await asyncio.sleep(0.3)


async def _request_position_stream(system, rate_hz: float = 5.0) -> None:
    """
    Request GLOBAL_POSITION_INT (msg id=33) from the autopilot.

    MAVSDK mavsdk_server does not automatically request position streaming
    from ArduCopter (unlike PX4, which sends it by default). ArduCopter
    requires an explicit MAV_CMD_SET_MESSAGE_INTERVAL before it begins
    streaming GLOBAL_POSITION_INT — without this, telemetry.position() and
    mavlink_direct.message("GLOBAL_POSITION_INT") both time out indefinitely.
    """
    interval_us = int(1_000_000 / rate_hz)
    await system.mavlink_direct.send_message(MavlinkMessage(
        message_name="COMMAND_LONG",
        system_id=255, component_id=1,
        target_system_id=1, target_component_id=0,
        fields_json=json.dumps({
            "target_system": 1, "target_component": 0,
            "command": 511,     # MAV_CMD_SET_MESSAGE_INTERVAL
            "param1": 33.0,    # MAVLINK_MSG_ID_GLOBAL_POSITION_INT
            "param2": float(interval_us),
            "param3": 0.0, "param4": 0.0, "param5": 0.0,
            "param6": 0.0, "param7": 0.0,
            "confirmation": 0,
        }),
    ))
    await asyncio.sleep(0.2)  # brief settle for stream to start


async def _get_home_position(system, timeout_s: float = 30.0):
    """Return the vehicle's home Position from telemetry."""
    async with asyncio.timeout(timeout_s):
        async for home in system.telemetry.home():
            return home
    raise TimeoutError("Home position not received within timeout")


async def _get_position(system, timeout_s: float = 5.0):
    """Return the current Position from telemetry."""
    async with asyncio.timeout(timeout_s):
        async for pos in system.telemetry.position():
            return pos
    raise TimeoutError("Position not received")


async def _get_heading(system, timeout_s: float = 5.0) -> float:
    """Return current vehicle heading in degrees (0-360)."""
    async with asyncio.timeout(timeout_s):
        async for hdg in system.telemetry.heading():
            return hdg.heading_deg
    raise TimeoutError("Heading not received")


async def _get_flight_mode(system, timeout_s: float = 5.0) -> str:
    """Return current flight mode name as a string."""
    async with asyncio.timeout(timeout_s):
        async for fm in system.telemetry.flight_mode():
            return str(fm)
    raise TimeoutError("Flight mode not received")


# ---------------------------------------------------------------------------
# Wait helpers
# ---------------------------------------------------------------------------


async def _wait_armable(system, timeout_s: float = ARMABLE_TIMEOUT_S):
    """
    Block until the vehicle reports is_armable=True.

    Uses fire-and-forget task + asyncio.Event to avoid leaving a dangling gRPC
    health stream after timeout cancellation (CLAUDE.md §4a pattern) — the
    gRPC stream does not respond to asyncio cancellation reliably; wrapping it
    in a background task and only awaiting a plain Event ensures clean timeout
    handling.
    """
    armable_event = asyncio.Event()

    async def _watch() -> None:
        async for health in system.telemetry.health():
            if health.is_armable:
                armable_event.set()
                return

    task = asyncio.create_task(_watch())
    try:
        await asyncio.wait_for(armable_event.wait(), timeout=timeout_s)
    finally:
        task.cancel()  # fire-and-forget — do NOT await (§4a)


async def _wait_for_altitude(system, threshold_m: float, timeout_s: float = TAKEOFF_TIMEOUT_S):
    """Block until relative_altitude_m >= threshold_m; return the Position."""
    async with asyncio.timeout(timeout_s):
        async for pos in system.telemetry.position():
            if pos.relative_altitude_m >= threshold_m:
                return pos
    raise TimeoutError(
        f"Relative altitude {threshold_m:.1f} m not reached within {timeout_s:.0f} s"
    )


async def _wait_for_altitude_with_peak_pitch(
    system, threshold_m: float, timeout_s: float = TAKEOFF_TIMEOUT_S
):
    """Block until threshold_m reached; also return max |pitch| sampled during the wait."""
    peak_pitch: float = 0.0

    async def _sample_pitch() -> None:
        nonlocal peak_pitch
        async for att in system.telemetry.attitude_euler():
            mag = abs(att.pitch_deg)
            if mag > peak_pitch:
                peak_pitch = mag

    pitch_task = asyncio.create_task(_sample_pitch())
    try:
        pos = await _wait_for_altitude(system, threshold_m, timeout_s)
    finally:
        pitch_task.cancel()
        # Do not await the task: the attitude gRPC stream may not yield promptly
        # after cancellation (same issue as CLAUDE.md §4a for connection_state()).

    return pos, peak_pitch


async def _wait_for_altitude_with_pitch_extremes(
    system, threshold_m: float, timeout_s: float = TAKEOFF_TIMEOUT_S
):
    """
    Block until threshold_m reached; also return the signed min and max pitch sampled
    during the wait (unlike _wait_for_altitude_with_peak_pitch's unsigned magnitude) —
    needed to confirm *direction* (e.g. a commanded negative/nose-down pitch actually
    produces a negative attitude, not just "some" pitch magnitude of either sign).
    """
    min_pitch: float = 0.0
    max_pitch: float = 0.0

    async def _sample_pitch() -> None:
        nonlocal min_pitch, max_pitch
        async for att in system.telemetry.attitude_euler():
            if att.pitch_deg < min_pitch:
                min_pitch = att.pitch_deg
            if att.pitch_deg > max_pitch:
                max_pitch = att.pitch_deg

    pitch_task = asyncio.create_task(_sample_pitch())
    try:
        pos = await _wait_for_altitude(system, threshold_m, timeout_s)
    finally:
        pitch_task.cancel()

    return pos, min_pitch, max_pitch


async def _wait_for_horizontal_position(
    system, target_lat: float, target_lon: float, threshold_m: float, timeout_s: float = 60.0,
):
    """Block until the vehicle is within threshold_m of target lat/lon; return the Position."""
    async with asyncio.timeout(timeout_s):
        async for pos in system.telemetry.position():
            if _dist_m(pos.latitude_deg, pos.longitude_deg, target_lat, target_lon) <= threshold_m:
                return pos
    raise TimeoutError(f"Did not reach within {threshold_m:.1f} m of target within {timeout_s:.0f} s")


# ---------------------------------------------------------------------------
# Mode / action helpers
# ---------------------------------------------------------------------------


async def _set_guided_mode_ardupilot(system, timeout_s: float = 15.0, custom_mode: int = 4) -> bool:
    """
    Set ArduPilot GUIDED mode and verify via telemetry.

    Sends DO_SET_MODE (176) with the given ``custom_mode`` and polls
    ``telemetry.flight_mode()`` until the mode is confirmed.

    MAVSDK mode string mapping (observed):
    - ArduCopter GUIDED (custom_mode=4)  -> "OFFBOARD"
    - ArduPlane GUIDED  (custom_mode=15) -> "UNKNOWN" (MAVSDK has no mapping
      for ArduPlane custom modes beyond a small set)

    Acceptance rules:
    - "OFFBOARD" or "GUIDED" in the mode string: definitive confirmation.
    - "UNKNOWN": MAVSDK has no mode mapping but the DO_SET_MODE ACK was
      accepted; treat as confirmed since we cannot verify further.
    - Any other definite mode (e.g. "MANUAL", "STABILIZED"): retry.

    Returns True when confirmed (including UNKNOWN-after-ACK), False on timeout.
    """
    guided_ack_accepted = False
    async with asyncio.timeout(timeout_s):
        while True:
            ack = await probe_command_long(system, 176, param1=1.0, param2=float(custom_mode))
            if ack and int(ack["result"]) == 0:
                guided_ack_accepted = True
            await asyncio.sleep(0.5)
            try:
                mode = await _get_flight_mode(system, timeout_s=1.0)
                if "OFFBOARD" in mode or "GUIDED" in mode:
                    log.info("GUIDED mode confirmed: telemetry reports %r", mode)
                    return True
                if "UNKNOWN" in mode and guided_ack_accepted:
                    log.info(
                        "GUIDED mode confirmed (ACK only): telemetry reports %r — "
                        "no MAVSDK mapping for custom_mode=%d", mode, custom_mode,
                    )
                    return True
                log.info("GUIDED mode not yet active: current mode %r — retrying", mode)
            except TimeoutError:
                pass
            await asyncio.sleep(0.5)
    return False


async def _rtl_and_land(system, restart_flight_stack=None, timeout_s: float | None = None) -> None:
    """
    Command RTL and wait for landed state; then disarm. Best-effort — never raises.

    If `restart_flight_stack` is given (the callable from tests/conftest.py's
    fixture of the same name) and the vehicle does not reach ON_GROUND within
    timeout_s, restarts the flight stack from scratch instead of blindly
    disarming. Why: RTL alone doesn't land a fixed-wing aircraft without
    RTL_AUTOLAND/DO_LAND_START (ArduPlane defaults RTL_AUTOLAND to disabled),
    so on a stack/config where RTL just loiters, the old unconditional
    disarm() either crashes the vehicle (a stack that honours an in-air
    disarm) or silently fails (ArduPilot's AP_Arming_Plane::disarm()
    correctly refuses a MAVLink disarm while is_flying()) — either way
    leaving the vehicle in a broken state that every subsequent test then
    inherits. See root CLAUDE.md's 2026-09-15 ArduPlane finding and
    restart_flight_stack's own docstring for the full story. Callers that
    don't pass restart_flight_stack keep the old best-effort-disarm
    behaviour unchanged (e.g. commands/missions where RTL+land is already
    known to work reliably, such as PX4 MC).

    `timeout_s` defaults to `RTL_LAND_TIMEOUT_WITH_RESTART_S` (30s) when
    `restart_flight_stack` is given, or the full `RTL_LAND_TIMEOUT_S` (120s)
    otherwise — a real landing happens quickly when it's going to happen at
    all, so waiting out the full timeout before falling back to a restart
    that's already available just wastes wall-clock time (see
    `RTL_LAND_TIMEOUT_WITH_RESTART_S`'s own comment for the measured cost).
    Pass an explicit value to override either default.
    """
    if timeout_s is None:
        timeout_s = RTL_LAND_TIMEOUT_WITH_RESTART_S if restart_flight_stack is not None else RTL_LAND_TIMEOUT_S
    landed = False
    try:
        await system.action.return_to_launch()
        async with asyncio.timeout(timeout_s):
            async for state in system.telemetry.landed_state():
                if state == LandedState.ON_GROUND:
                    landed = True
                    break
    except Exception as exc:
        log.warning("RTL/land wait failed: %s", exc)

    if landed:
        await asyncio.sleep(2.0)
        try:
            await system.action.disarm()
        except Exception:
            pass
    elif restart_flight_stack is not None:
        log.warning(
            "Vehicle did not reach ON_GROUND within %.0fs — restarting the flight "
            "stack instead of forcing a disarm (see root CLAUDE.md's 2026-09-15 "
            "ArduPlane finding)", timeout_s,
        )
        restart_flight_stack()
    else:
        try:
            await system.action.disarm()
        except Exception:
            pass


async def _reboot_sitl_if_degraded(system, label: str = "SITL") -> None:
    """
    Reboot the flight stack via MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (cmd=246).

    Intended to be called when ``_wait_armable()`` times out, indicating the
    SITL has degraded after many consecutive arm-takeoff-RTL cycles (PX4 SIH
    accumulates EKF drift and physics state over many cycles until
    ``is_armable`` never returns True again).

    After sending the reboot command:
    - PX4 reboots its firmware and the SIH physics module (~15 s).
    - The session-scoped ``mavsdk_server`` reconnects automatically on the
      same UDP port when PX4 resumes broadcasting heartbeats.
    - Position and home streams are re-requested so the caller's next
      ``_wait_armable`` has fresh telemetry.

    A 5-minute cooldown prevents reboot loops if recovery fails. Caller must
    call ``_wait_armable(system, timeout_s=120.0)`` after this returns.
    `label` is used only for logging (e.g. the calling command's name).
    """
    global _last_sitl_reboot_ts
    now = _time_m.monotonic()
    if now - _last_sitl_reboot_ts < _REBOOT_COOLDOWN_S:
        log.warning(
            "%s SITL reboot: within cooldown (%.0f s) — not rebooting; "
            "caller's _wait_armable will raise TimeoutError", label, _REBOOT_COOLDOWN_S,
        )
        return
    _last_sitl_reboot_ts = now
    log.info(
        "%s SITL reboot: is_armable=False after timeout — sending "
        "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (SITL degraded after many flight cycles)", label,
    )
    try:
        # param1=1.0: reboot autopilot (not companion). PX4 may disconnect
        # before the ACK arrives, so exceptions are suppressed.
        await probe_command_long(system, 246, param1=1.0)
    except Exception:
        pass
    # Wait for PX4 SIH to boot and for EKF to begin converging before
    # re-requesting streams (streams silently timeout if PX4 isn't ready yet).
    await asyncio.sleep(15.0)
    await _request_position_stream(system)
    await _request_home_position(system)
    log.info("%s SITL reboot: done — caller should now _wait_armable(120 s)", label)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def _dist_m(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float) -> float:
    """
    Flat-earth distance in metres between two WGS84 points (accurate to
    < 0.1% for distances < 10 km).

    Canonical version, adopted from do_reposition/test_flight.py — replaces a
    cruder `abs(lat2/90 + 0.001)` longitude-scale approximation that had
    independently drifted into tests/command/nav_takeoff/test_flight.py.
    """
    mid_lat = math.radians((lat1_deg + lat2_deg) / 2)
    dlat = (lat2_deg - lat1_deg) * 111111.0
    dlon = (lon2_deg - lon1_deg) * 111111.0 * math.cos(mid_lat)
    return math.sqrt(dlat**2 + dlon**2)


def _offset_lat_lon(lat_deg: float, lon_deg: float, north_m: float, east_m: float) -> tuple[float, float]:
    """Return (lat, lon) displaced by north_m and east_m from the given point."""
    lat = lat_deg + north_m / 111111.0
    lon = lon_deg + east_m / (111111.0 * math.cos(math.radians(lat_deg)))
    return lat, lon


def _north_of(lat_deg: float, metres: float) -> int:
    """Return latitude as int×1e7 for a position metres north of lat_deg."""
    return int((lat_deg + metres / 111111.0) * 1e7)


# ---------------------------------------------------------------------------
# "Get airborne via NAV_TAKEOFF" utility
# ---------------------------------------------------------------------------
#
# Used by other commands' Tier 2 tests purely as a way to reach a flyable
# state, not to test NAV_TAKEOFF itself — NAV_TAKEOFF's own execution
# semantics (param behaviour, per-vehicle-type support, etc.) are tested in
# tests/command/nav_takeoff/test_flight.py, not here.


def _takeoff_cmd(**overrides) -> dict:
    """Return default COMMAND_INT kwargs for MAV_CMD_NAV_TAKEOFF."""
    defaults = dict(
        command=_NAV_TAKEOFF_CMD_ID,
        frame=6,       # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        param1=0.0,    # Pitch: 0 deg (use default)
        param2=0.0,    # Unused
        param3=0.0,    # Flags: none
        param4=0.0,    # Yaw: 0.0 = north (None encodes as NaN — "use current heading")
        x=0,           # lat: 0 -> most stacks treat as "use current position"
        y=0,           # lon: 0
        z=float(TAKEOFF_ALT_M),
    )
    defaults.update(overrides)
    return defaults


async def _arm_and_send_takeoff(system, **overrides) -> None:
    """
    Wait for armable -> arm -> send NAV_TAKEOFF COMMAND_INT.

    PX4 treats the z field in COMMAND_INT as absolute AMSL altitude (it
    ignores the frame field). The caller passes z as a RELATIVE altitude
    (above home); this function adds the home AMSL altitude to produce the
    correct absolute z.

    Special cases:
    - z=None: passed as-is (NaN on wire -> stack uses its default altitude).
    - z=0.0: treated literally (0m AMSL) for a zero-altitude observation test.

    Home lat/lon is used as x/y unless overrides specify different values.
    Frame is forced to 5 (GLOBAL_INT, absolute AMSL) to match the absolute z.
    """
    home = await _get_home_position(system)
    home_lat_int = int(home.latitude_deg * 1e7)
    home_lon_int = int(home.longitude_deg * 1e7)
    home_amsl_m = home.absolute_altitude_m

    # Convert relative z to absolute; preserve None (NaN) and 0.0 as-is.
    relative_z = overrides.pop("z", float(TAKEOFF_ALT_M))
    if relative_z is not None and relative_z != 0.0:
        absolute_z = home_amsl_m + relative_z
    else:
        absolute_z = relative_z  # None (NaN) or 0.0 absolute for observation tests

    merged = {
        "x": home_lat_int,
        "y": home_lon_int,
        "z": absolute_z,
        "frame": 5,  # GLOBAL_INT: absolute AMSL — matches PX4 COMMAND_INT behavior
    }
    merged.update(overrides)
    await _request_position_stream(system)  # ArduCopter doesn't stream by default
    await _wait_armable(system)
    await system.action.arm()
    await asyncio.sleep(0.5)  # brief settle after arm before command
    kw = _takeoff_cmd(**merged)
    await send_command_int(system, **kw)


# ---------------------------------------------------------------------------
# Tier 2 results logging — zero-configuration, common to every Tier 2 module
# ---------------------------------------------------------------------------
#
# Mirrors tests/command/conftest.py's / tests/mission/conftest.py's
# _write_tier1_log — same table shape, same logs/ naming convention, same
# "always write, regardless of pass/fail or --log-cli-level" guarantee.
# Before this, Tier 2 test outcomes existed only as live log.info() calls,
# visible with --log-cli-level=INFO and otherwise not persisted anywhere.
#
# Deliberately auto-capturing rather than requiring an explicit record() call
# per test (Tier 1's approach, workable there because every check funnels
# through a handful of shared base-class methods): Tier 2 tests are
# free-form async functions with no shared base to hook into, and this repo's
# "common framework" convention (root CLAUDE.md) means the goal is a rollout
# that costs each new/existing Tier 2 module two constants and one import, not
# a per-test-function edit repeated across every command/mission folder. The
# outcome and a one-line detail (the test's docstring first line by default,
# or the tail of its failure/skip reason) are pulled from the pytest report
# stashed by tests/conftest.py's pytest_runtest_makereport hook; use
# record_tier2_detail() from inside a test only when that default isn't
# informative enough (e.g. to report specific measured values).
#
# Adopt in a new Tier 2 module: import `_tier2_auto_record` alongside
# `require_real_stack` (both autouse — importing them by name is what
# activates them, same mechanism `require_real_stack` already relies on) and
# declare two module-level constants, `_CMD_NAME` and `_CMD_ID`.
#
# Writes incrementally (the log file is rewritten after every single test, not
# batched to session end): a real-hardware Tier 2 run that hangs or gets
# force-killed (a live concern — see e.g. root CLAUDE.md's documented
# ArduCopter SITL boot issue and general connection-reliability pitfalls)
# still leaves on disk whatever ran before the kill, rather than nothing.

_TIER2_FMT = "%-46s | %-9s | %s"
_TIER2_MODULE_RESULTS: dict[str, list[tuple[str, str, str]]] = {}
_TIER2_MODULE_TIMESTAMP: dict[str, str] = {}  # module -> timestamp fixed at its first result
_TIER2_DETAILS: dict[str, str] = {}  # nodeid -> explicit detail override
_TIER2_PARAM_VERDICTS: dict[str, list[tuple[str, str]]] = {}  # module -> [(param_label, verdict), ...]


def _safe_log_token(s: str) -> str:
    return s.replace("/", "_").replace(" ", "_").replace("\\", "_")


def record_tier2_detail(request, detail: str) -> None:
    """
    Attach a specific one-line detail (e.g. measured values) to the calling
    test's auto-captured Tier 2 log entry, in place of the default (its
    docstring's first line, or its failure/skip reason). Optional — most
    tests need no call here at all.
    """
    _TIER2_DETAILS[request.node.nodeid] = detail


def record_tier2_param_verdict(request, param_label: str, verdict: str) -> None:
    """
    Tag this test's result as the definitive answer to "is `param_label` supported",
    for the "Compatibility summary" section appended after the main results table in
    this module's Tier 2 log — a compact, scannable per-param rollup (e.g. "param1
    (Pitch): NOT SUPPORTED") distilled from the detailed table above it, so a reader
    doesn't have to reconstruct which of a dozen test rows answers which param's
    question. Call once, from whichever single test is the definitive "is it honoured"
    check for that param (e.g. test_takeoff_compat_tracks_pitch for "param1 (Pitch)") — not
    from every test that happens to touch the param. `verdict` is a short human string:
    "SUPPORTED", "NOT SUPPORTED", "NOT TESTABLE (<why>)", "NOT TESTED (<why>)", etc.
    """
    module_name = request.node.module.__name__
    _TIER2_PARAM_VERDICTS.setdefault(module_name, []).append((param_label, verdict))


# ---------------------------------------------------------------------------
# mavlink-compat-data schema-shaped JSON export
# ---------------------------------------------------------------------------
# Structured (not just human-readable-string) per-param compatibility facts,
# shaped to match the sibling data repo github.com/hamishwillee/
# mavlink-compat-data's command compatibility schema (schema/
# compatibility-entry.schema.json's paramStatement/paramSupported defs, per
# PR #9 — https://github.com/hamishwillee/mavlink-compat-data/pull/9). Lets a
# Tier 2 run emit a block ready to paste into that repo's
# data/dialects/<dialect>/mav_cmd/<context>/<NAME>.json under
# compatibility.<stack>.frames.<frame>, instead of hand-transcribing prose
# into that shape after the fact. Kept alongside, not instead of,
# record_tier2_param_verdict() — that's the human-readable summary line;
# this is the machine-shaped one, and the two are recorded independently
# (nothing parses one from the other).
_COMPAT_JSON_PARAMS: dict[str, dict[str, dict]] = {}  # module -> {param_key: fields}
_COMPAT_JSON_COMMAND: dict[str, dict] = {}  # module -> {"supported": bool, "basis": str}
_UNSET = object()  # record_compat_json's default for `supported` — distinct from a real None ("not tested")

# --vehicle-type (as this harness's CLI already uses it) -> mavlink-compat-data
# vocab.json frame name. "fixed_wing" is stack-dependent (px4: fixedwing,
# ardupilot: plane) so it's resolved separately in _compat_json_frame().
_COMPAT_FRAME_MAP = {
    "quadcopter": "multicopter",
    "vtol": "vtol",
    "quadplane": "standard_quadplane",
    "copter": "copter",
    "rover": "rover",
}


def _compat_json_frame(autopilot: str, vehicle_type: str) -> str | None:
    """Map this harness's (--autopilot, --vehicle-type) to a mavlink-compat-data vocab.json frame name."""
    if vehicle_type == "fixed_wing":
        return "fixedwing" if autopilot == "px4" else "plane"
    return _COMPAT_FRAME_MAP.get(vehicle_type)


def record_compat_command_supported(
    request, supported: bool, basis: str = "verified", notes: str | list[str] | None = None,
) -> None:
    """
    Record whether MAV_CMD_<X> works AT ALL for this (stack, frame) — the
    frame-level `supported`/`notes` facts in mavlink-compat-data's schema
    (root CLAUDE.md rule 1: "the vehicle takes off" is the one thing a
    takeoff command's XML actually requires — this is that fact, not a
    per-param one). `notes` here is a durable, frame-wide qualitative fact
    (e.g. "requires an explicit NAV_TAKEOFF item to launch"), distinct from
    any single param's own notes. Merges into whatever's already recorded
    (like record_compat_json) — safe to call more than once, e.g. once for
    `supported` and again later once a `notes` fact is established.
    """
    module_name = request.node.module.__name__
    facts = _COMPAT_JSON_COMMAND.setdefault(module_name, {})
    facts["supported"] = supported
    facts["basis"] = basis
    if notes is not None:
        facts["notes"] = notes


def record_compat_json(
    request,
    param_key: str,
    *,
    supported=_UNSET,  # True | False | None | "not-applicable"
    accept_nan_or_int32max: bool | None = None,
    nacks_on_non_sentinel_value: bool | None = None,
    notes: str | list[str] | None = None,
) -> None:
    """
    Record one param's compatibility fact in mavlink-compat-data's own schema
    shape — see this section's module-level comment for the target schema.

    **Merges into any facts already recorded for this param_key** (does not
    overwrite) — different tests in the same module typically establish
    different facets of the same param (e.g. one test finds whether it's
    functionally `supported`, a separate sentinel-specific test finds
    `accept_nan_or_int32max`), and calling this from each should accumulate,
    not clobber. Only pass the field(s) *this* call actually has evidence
    for; omitted fields (default) leave whatever's already recorded alone.

    `param_key`: mavlink-compat-data's `"<index>_<name>"` form, e.g.
    `"1_Pitch"` — matches the shared `ParamSpec`/XML param label, not this
    harness's own `"param1 (Pitch)"` label used by record_tier2_param_verdict.
    `supported`: `True` (confirmed working — rendered as the
    `{"added_version": true}` object form), `False` (confirmed not working),
    `None` (not independently tested this run), or `"not-applicable"` (a
    reserved/"Empty" param slot — see schema note on that convention).
    `accept_nan_or_int32max`: does this param's sentinel value get accepted
    (only meaningful if you actually tested the sentinel specifically).
    `nacks_on_non_sentinel_value`: does the stack correctly reject a real,
    non-sentinel value on this param — only meaningful (and only rendered)
    when `supported` is not `True`, per the schema's own gating (a working
    param is expected to accept real values, so the question is moot there).
    `notes`: extremely terse per mavlink-compat-data's own convention — a
    fragment, ~10 words, no period, describing observable behaviour only
    (never an internal-implementation claim like "not stored" — black-box
    testing can't evidence that, only the effect).
    """
    module_name = request.node.module.__name__
    facts = _COMPAT_JSON_PARAMS.setdefault(module_name, {}).setdefault(param_key, {})
    if supported is not _UNSET:
        facts["supported"] = supported
    if accept_nan_or_int32max is not None:
        facts["accept_nan_or_int32max"] = accept_nan_or_int32max
    if nacks_on_non_sentinel_value is not None:
        facts["nacks_on_non_sentinel_value"] = nacks_on_non_sentinel_value
    if notes is not None:
        facts["notes"] = notes


def _render_compat_json_param(fact: dict) -> dict:
    supported = fact.get("supported")  # a param may accumulate only sentinel facts, never a supported call
    out: dict = {}
    if supported is True:
        sup: dict = {"added_version": True}
        if fact.get("accept_nan_or_int32max") is not None:
            sup["accept_nan_or_int32max"] = fact["accept_nan_or_int32max"]
        out["supported"] = sup
    else:
        out["supported"] = supported  # False / None / "not-applicable"
        if fact.get("accept_nan_or_int32max") is not None:
            out["accept_nan_or_int32max"] = fact["accept_nan_or_int32max"]
        if fact.get("nacks_on_non_sentinel_value") is not None:
            out["nacks_on_non_sentinel_value"] = fact["nacks_on_non_sentinel_value"]
    if fact.get("notes"):
        out["notes"] = fact["notes"]
    return out


def _render_compat_json(module_name: str, config) -> str | None:
    """
    Render this module's recorded record_compat_json()/
    record_compat_command_supported() facts as a mavlink-compat-data-shaped
    JSON fragment for one (stack, frame), ready to paste into that repo's
    compatibility.<stack>.frames.<frame>. Returns None if neither was ever
    called this run (most modules — this is opt-in per Tier 2 file, not
    automatic, since it needs per-param calls a generic writer can't infer).
    """
    params = _COMPAT_JSON_PARAMS.get(module_name)
    command = _COMPAT_JSON_COMMAND.get(module_name)

    # A module may declare its full param set (_COMPAT_JSON_ALL_PARAMS, e.g.
    # NAV_TAKEOFF's 7 slots) so every one of them always appears in the
    # rendered JSON — even a param whose Tier 2 test errored out (uncaught
    # exception) before ever calling record_compat_json(), and so has no
    # recorded facts at all this run. Per the schema, "supported": null
    # (an empty {} fact dict renders to exactly that — see
    # _render_compat_json_param) is the correct "not independently tested
    # this run" signal — distinct from a param the module doesn't cover at
    # all (silently absent, the pre-2026-09-15 behaviour), which would be
    # indistinguishable from "we forgot to test it".
    import sys
    module = sys.modules.get(module_name)
    all_param_keys = getattr(module, "_COMPAT_JSON_ALL_PARAMS", None)
    if all_param_keys:
        params = {k: (params or {}).get(k, {}) for k in all_param_keys}

    if not params and not command:
        return None

    info = getattr(config, "_autopilot_info", {}) or {}
    autopilot = (info.get("autopilot") or "").lower()
    vehicle_type = config.getoption("--vehicle-type") or ""
    frame = _compat_json_frame(autopilot, vehicle_type)
    if autopilot not in ("px4", "ardupilot") or frame is None:
        return (
            f"# Could not map autopilot={autopilot!r}/vehicle_type={vehicle_type!r} to a "
            f"mavlink-compat-data (stack, frame) pair (schema/vocab.json) — pass "
            f"--autopilot/--vehicle-type matching that vocab, or fill this in by hand."
        )

    firmware_version = info.get("firmware_version", "") or ""
    # mavlink-compat-data's own convention (its CLAUDE.md): never record
    # supported/notes/version facts purely from a dev/pre-release snapshot —
    # a specific git commit isn't a stable, re-comparable identity. Still
    # render the JSON (useful to have ready), but flag it and omit the
    # version-durability fields rather than silently overclaiming them.
    is_dev_build = bool(re.search(r"(?i)-dev\b|-beta\b|-rc\d|\bmain\b", firmware_version))
    bare_version = firmware_version.split("-")[0] if firmware_version else None

    frame_obj: dict = {}
    if command is not None:
        frame_obj["supported"] = {"added_version": True} if command["supported"] else False
        frame_obj["basis"] = "testing" if is_dev_build else command.get("basis", "verified")
        if bare_version and not is_dev_build:
            frame_obj["earliest_checked_version"] = bare_version
        if command.get("notes"):
            frame_obj["notes"] = command["notes"]
    if params:
        frame_obj["params"] = {k: _render_compat_json_param(v) for k, v in sorted(params.items())}

    doc = {autopilot: {"frames": {frame: frame_obj}}}
    rendered = json.dumps(doc, indent=2)
    if is_dev_build:
        rendered = (
            f"# NOTE: {firmware_version} is a dev/pre-release build — per mavlink-compat-data's\n"
            f"# own convention, do not merge this upstream as-is; retest against a tagged\n"
            f"# release first (this is why basis is \"testing\" not \"verified\", and\n"
            f"# earliest_checked_version is omitted).\n"
            + rendered
        )
    return rendered


@pytest.fixture(autouse=True)
def _tier2_auto_record(request):
    """
    Record this test's outcome (PASS/FAIL/XFAIL/XPASS/SKIP/NA) and a one-line
    detail, then (re)write the whole module's Tier 2 log immediately — with
    no per-test code required. Writing after every test, not just at session
    end, means a run that hangs or is force-killed still leaves a real,
    if partial, log on disk.

    NA is distinct from SKIP: SKIP means the test's own precondition for running
    at all is absent (no real stack connected — require_real_stack). NA means the
    test ran far enough to discover that a *different* test already showed some
    prerequisite capability doesn't work (e.g. yaw isn't tracked at all), so this
    test's own specific question is moot — showing it as FAIL would be misleading
    (it isn't testing what its name says), and showing it as a bare SKIP loses
    that distinction. Trigger it by prefixing a `pytest.skip("NA: ...")` reason
    with the literal "NA: " marker; the prefix is stripped from the logged detail.
    """
    yield
    node = request.node
    rep = getattr(node, "_report_call", None) or getattr(node, "_report_setup", None)
    if rep is None:
        return  # e.g. collection error before any phase ran

    if getattr(rep, "skipped", False):
        outcome = "XFAIL" if hasattr(rep, "wasxfail") else "SKIP"
    elif getattr(rep, "passed", False):
        outcome = "XPASS" if hasattr(rep, "wasxfail") else "PASS"
    else:
        outcome = "FAIL"

    detail = _TIER2_DETAILS.pop(node.nodeid, None)
    if detail is None:
        if outcome == "SKIP" and isinstance(rep.longrepr, tuple) and len(rep.longrepr) == 3:
            # Skip reason reprs as (path, lineno, "Skipped: <reason>") — take just the reason.
            reason = str(rep.longrepr[2]).removeprefix("Skipped: ")
            if reason.startswith("NA: "):
                outcome = "NA"
                reason = reason.removeprefix("NA: ")
            detail = reason[:160]
        elif outcome in ("FAIL", "XFAIL") and rep.longrepr:
            detail = str(rep.longrepr).strip().splitlines()[-1][:160]
        else:
            doc = (node.function.__doc__ or "").strip()
            detail = doc.splitlines()[0].strip() if doc else ""

    module_name = node.module.__name__
    _TIER2_MODULE_RESULTS.setdefault(module_name, []).append((node.name, outcome, detail))
    _TIER2_MODULE_TIMESTAMP.setdefault(module_name, _time_m.strftime("%Y%m%d_%H%M%S"))
    _write_tier2_module_log(request.config, module_name)


def _write_tier2_module_log(config, module_name: str) -> None:
    """
    (Re)write one module's accumulated Tier 2 results table to logs/,
    unconditionally, overwriting the same path each time — called after every
    test via _tier2_auto_record, not just once at session end.

    The module is identified by its own `_CMD_NAME`/`_CMD_ID` constants;
    protocol ("mission" vs "command", matching Tier 1's log filename prefix)
    is inferred from the module's own package path. A module missing either
    constant, or outside both trees, is skipped rather than guessed at.
    """
    import sys

    results = _TIER2_MODULE_RESULTS.get(module_name)
    if not results:
        return
    module = sys.modules.get(module_name)
    name = getattr(module, "_CMD_NAME", None)
    cmd_id = getattr(module, "_CMD_ID", None)
    if module is None or name is None or cmd_id is None:
        return
    if ".mission." in module_name:
        protocol = "mission"
    elif ".command." in module_name:
        protocol = "command"
    else:
        return

    info = getattr(config, "_autopilot_info", {})
    drone_address = config.getoption("--drone-address")
    header = _format_autopilot_header(info, drone_address)

    counts: dict[str, int] = {}
    lines = [
        f"Tier 2 results: MAV_CMD_{name} (cmd={cmd_id})",
        "=" * 100,
        f"{'Test':<46} {'Outcome':<9} Detail",
        "-" * 100,
    ]
    for test_name, outcome, detail in results:
        counts[outcome] = counts.get(outcome, 0) + 1
        lines.append(f"{test_name:<46} {outcome:<9} {detail}")
    lines.append("-" * 100)
    lines.append(
        f"Total: {len(results)}  " + "  ".join(f"{k}={v}" for k, v in sorted(counts.items()))
    )

    verdicts = _TIER2_PARAM_VERDICTS.get(module_name)
    if verdicts:
        lines.append("")
        lines.append("Compatibility summary")
        lines.append("=" * 100)
        width = max(len(p) for p, _ in verdicts)
        for param_label, verdict in verdicts:
            lines.append(f"{param_label:<{width}}  {verdict}")

    compat_json = _render_compat_json(module_name, config)
    if compat_json:
        lines.append("")
        lines.append("Compatibility JSON (mavlink-compat-data schema — github.com/hamishwillee/mavlink-compat-data)")
        lines.append("=" * 100)
        lines.append(compat_json)

    table = "\n".join(lines)
    log.info("\n%s", table)

    ap = _safe_log_token(info.get("autopilot", "unknown").lower().replace("ardupilotmega", "ardupilot"))
    vt = _safe_log_token(info.get("vehicle_type", "unknown").lower())
    ver_raw = info.get("firmware_version", "")
    ver = f"_{_safe_log_token(ver_raw)}" if ver_raw and ver_raw != "N/A" else ""
    timestamp = _TIER2_MODULE_TIMESTAMP[module_name]

    logs_dir = Path("logs")
    logs_dir.mkdir(exist_ok=True)
    safe_name = _safe_log_token(name.lower())
    log_path = logs_dir / f"{protocol}_{safe_name}_tier2_{ap}_{vt}{ver}_{timestamp}.log"
    log_path.write_text(header + "\n\n" + table + "\n")
    log.info(_TIER2_FMT, name, "LOG", f"Tier 2 results log updated: {log_path}")
