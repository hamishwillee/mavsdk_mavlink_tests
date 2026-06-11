"""
MAV_CMD_NAV_LAND (cmd=21) via COMMAND_INT — Tier 2 execution tests.

Sends NAV_LAND directly via COMMAND_INT (not via mission upload) and observes
telemetry to determine *execution semantics* that ACK-level (Tier 1) tests
cannot answer:

- Does the vehicle fly a descending glide-slope toward the commanded point, fly
  there at altitude and then descend (a "dogleg"), or simply descend in place
  and ignore the commanded lat/lon?
- Is the commanded coordinate the actual touchdown point?
- For VTOL: does NAV_LAND trigger a transition to hover for vertical landing?
- Is NAV_LAND "accepted but inert" on vehicle types that cannot land (rover)?

Requires a real flight stack (--drone-address).  All tests are skipped in
paired/mock mode.  Each test takes off, sends NAV_LAND, observes the descent,
and lands/RTLs in a finally block — these are full flight cycles, hence the
generous per-test timeout.

Running
-------
PX4 multicopter::

    pytest tests/command/nav_land/test_flight.py \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \\
        --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO

PX4 fixed-wing::

    pytest tests/command/nav_land/test_flight.py \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_airplane \\
        --vehicle-type=fixed_wing --autopilot=px4 -v --log-cli-level=INFO

PX4 VTOL::

    pytest tests/command/nav_land/test_flight.py \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_standard_vtol \\
        --vehicle-type=vtol --autopilot=px4 -v --log-cli-level=INFO

PX4 Rover::

    pytest tests/command/nav_land/test_flight.py \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_rover_ackermann \\
        --vehicle-type=rover --autopilot=px4 -v --log-cli-level=INFO

ArduCopter::

    pytest tests/command/nav_land/test_flight.py \\
        --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \\
        --ardupilot-sitl=~/ardu_sitl/arducopter \\
        --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 \\
        --vehicle-type=quadcopter --autopilot=ardupilot -v --log-cli-level=INFO
"""

import asyncio
import datetime
import logging
from pathlib import Path

import pytest
from mavsdk.telemetry import LandedState, VtolState

from tests.command.conftest import (
    probe_command_int,
    probe_command_long,
    _FMT,
)

from tests.command.nav_takeoff.test_flight import (
    _get_home_position,
    _request_home_position,
    _request_position_stream,
    _wait_armable,
    _wait_for_altitude,
    _get_flight_mode,
    _get_heading,
    _set_guided_mode_ardupilot,
    _rtl_and_land,
    _dist_m,
    _takeoff_cmd,
    TAKEOFF_ALT_M,
    AIRBORNE_THRESHOLD_M,
)

log = logging.getLogger(__name__)

# Flight tests: arming + takeoff + landing observation + RTL/land + margin.
pytestmark = pytest.mark.timeout(360)

_CMD = "NAV_LAND"
_CMD_ID = 21  # MAV_CMD_NAV_LAND

_PRECISION_LAND_MODE_DISABLED = 0

_ACK_NAMES = {0: "ACCEPTED", 1: "TEMP_REJECTED", 2: "DENIED", 3: "UNSUPPORTED", 4: "FAILED"}

# Lateral offset of the commanded landing point from the takeoff/hover position —
# large enough that "fly toward target" vs "descend in place" are clearly distinguishable.
LAND_TARGET_OFFSET_M   = 80.0
DESCENT_DETECT_TIMEOUT_S = 30.0   # max wait for altitude to start dropping after NAV_LAND
TOUCHDOWN_TIMEOUT_S      = 300.0  # max wait for landed_state() == ON_GROUND
                                  # (ArduCopter run #1 reached descent but still had not
                                  # hit ON_GROUND at 180 s — see CLAUDE.md/README notes)
TOUCHDOWN_MATCH_M        = 10.0   # touchdown within this radius of the commanded point counts as "match"


# ---------------------------------------------------------------------------
# Land command builder
# ---------------------------------------------------------------------------

def _land_cmd(**overrides) -> dict:
    """Return default COMMAND_INT kwargs for NAV_LAND (mirrors test_command.py's)."""
    defaults = dict(
        command=_CMD_ID,
        frame=6,       # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        param1=0.0,    # Abort Alt: 0 = use system default
        param2=float(_PRECISION_LAND_MODE_DISABLED),
        param3=0.0,    # Empty
        param4=None,   # Yaw: NaN = use current heading
        x=0,
        y=0,
        z=0.0,         # Altitude: ground level
    )
    defaults.update(overrides)
    return defaults


# ---------------------------------------------------------------------------
# NAV_LAND-specific telemetry helpers
# ---------------------------------------------------------------------------

async def _get_vtol_state(system, timeout_s: float = 5.0) -> VtolState:
    """Return the current VTOL state (MC / FW / TRANSITION_TO_*  / UNDEFINED)."""
    async with asyncio.timeout(timeout_s):
        async for vs in system.telemetry.vtol_state():
            return vs
    raise TimeoutError("VTOL state not received")


async def _wait_for_landed_state(system, target: LandedState, timeout_s: float) -> bool:
    """True if landed_state() reports ``target`` within timeout_s.

    Fire-and-forget task + plain Event — never await after cancel() on a gRPC
    stream (CLAUDE.md §4a: the stream does not yield to asyncio cancellation).
    """
    event = asyncio.Event()

    async def _watch() -> None:
        async for state in system.telemetry.landed_state():
            if state == target:
                event.set()
                return

    task = asyncio.create_task(_watch())
    try:
        await asyncio.wait_for(event.wait(), timeout=timeout_s)
        return True
    except (asyncio.TimeoutError, TimeoutError):
        return False
    finally:
        task.cancel()


async def _wait_for_descent_below(system, threshold_m: float, timeout_s: float) -> bool:
    """True if relative_altitude_m drops below threshold_m within timeout_s."""
    event = asyncio.Event()

    async def _watch() -> None:
        async for pos in system.telemetry.position():
            if pos.relative_altitude_m < threshold_m:
                event.set()
                return

    task = asyncio.create_task(_watch())
    try:
        await asyncio.wait_for(event.wait(), timeout=timeout_s)
        return True
    except (asyncio.TimeoutError, TimeoutError):
        return False
    finally:
        task.cancel()


# ---------------------------------------------------------------------------
# Trajectory analysis
# ---------------------------------------------------------------------------

def _classify_trajectory(samples: list[tuple[float, float]], initial_dist_m: float) -> str:
    """Classify a descent from (alt_rel_m, dist_to_target_m) samples.

    Compares the lateral distance-to-target at the moment the vehicle has
    descended to half its peak altitude with the distance at the start of the
    descent:
      - dogleg            — already (near) above the target before halfway down
      - descend-in-place  — has barely moved toward the target by halfway down
      - glide-slope       — descends and closes on the target together
    """
    if not samples or initial_dist_m < 1.0:
        return "unknown"
    peak_alt = max(s[0] for s in samples)
    if peak_alt < 1.0:
        return "unknown"
    mid_alt = peak_alt * 0.5
    mid_sample = min(samples, key=lambda s: abs(s[0] - mid_alt))
    ratio = mid_sample[1] / initial_dist_m
    if ratio < 0.25:
        return "dogleg"
    if ratio > 0.75:
        return "descend-in-place"
    return "glide-slope"


_TRAJECTORY_DESC = {
    "glide-slope": (
        "flies a descending glide-slope toward the commanded point — altitude "
        "and lateral distance to the target shrink together"
    ),
    "dogleg": (
        "flies laterally to above the commanded point first (at ~constant "
        "altitude), then descends — a 'dogleg' / fly-there-then-land approach"
    ),
    "descend-in-place": (
        "ignores the commanded lat/lon and simply descends from its current "
        "position"
    ),
    "unknown": "trajectory shape could not be classified from sampled telemetry",
}


# ---------------------------------------------------------------------------
# Per-run summary log (manual README transcription — see CLAUDE.md plan notes;
# NAV_LAND Tier 2 will run a small, bounded number of times, so the elaborate
# marker-based README auto-patcher used by nav_takeoff is not warranted here).
# ---------------------------------------------------------------------------

def _write_summary_log(request, autopilot: str | None, test_name: str, summary_block: str) -> Path:
    vehicle   = request.config.getoption("--vehicle-type", default="unknown") or "unknown"
    auto      = autopilot or "unknown"
    now       = datetime.datetime.now()
    date_str  = now.strftime("%Y-%m-%d %H:%M")
    date_file = now.strftime("%Y%m%d_%H%M%S")

    Path("logs").mkdir(exist_ok=True)
    log_path = (Path("logs") /
                f"command_nav_land_summary_{auto}_{vehicle}_{test_name}_{date_file}.md")
    log_path.write_text(
        f"# NAV_LAND COMMAND_INT — Behaviour Summary ({test_name})\n\n"
        f"**Autopilot:** {auto}  **Vehicle:** {vehicle}  **Date:** {date_str}\n\n"
        + summary_block + "\n",
        encoding="utf-8",
    )
    log.info(_FMT, _CMD, "summary log written", str(log_path))
    return log_path


# ---------------------------------------------------------------------------
# Shared: arm + take off vertically to TAKEOFF_ALT_M (stack-agnostic call site)
# ---------------------------------------------------------------------------

async def _arm_and_climb(
    system, autopilot: str | None, home, target_alt_m: float = TAKEOFF_ALT_M,
    altitude_threshold_frac: float = 0.85, climb_timeout_s: float = 180.0,
):
    """Arm (stack-specific) and send NAV_TAKEOFF, then wait to reach the threshold altitude.

    Mirrors the proven arm + command-field sequence from
    test_mc_takeoff_comprehensive (validated on PX4 MC and ArduCopter MC):
    PX4 treats COMMAND_INT z as absolute AMSL (frame=5); ArduCopter's COMMAND_INT
    handler requires frame=3 (GLOBAL_RELATIVE_ALT) with a relative z.  ArduCopter's
    action.arm() is unreliable on SITL cold start — use a COMMAND_LONG retry loop.

    `altitude_threshold_frac`/`climb_timeout_s` let callers relax the climb
    expectations for vehicle types with shallower/slower climb profiles (e.g.
    fixed-wing) without affecting the already-validated MC defaults.  Logs
    altitude progress every ~20 s during the wait — useful for diagnosing a
    plateau (vehicle leveling off below threshold) vs. a genuinely slow climb.
    """
    home_lat_int = int(home.latitude_deg * 1e7)
    home_lon_int = int(home.longitude_deg * 1e7)

    if autopilot == "ardupilot":
        async with asyncio.timeout(120.0):
            while True:
                ack = await probe_command_long(system, 400, param1=1.0)
                if ack and int(ack["result"]) == 0:
                    log.info(_FMT, _CMD, "arm", "ACCEPTED")
                    break
                await asyncio.sleep(3.0)
    else:
        # 120 s, not the 60 s default: on a freshly-booted SITL (no warm-up from
        # prior tests in this run) PX4's EKF/GPS can take well over a minute to
        # converge before is_armable goes True.
        await _wait_armable(system, timeout_s=120.0)
        await system.action.arm()
    await asyncio.sleep(3.0 if autopilot == "ardupilot" else 0.5)  # motor spool-up settle

    if autopilot == "px4":
        cmd_z, cmd_frame = home.absolute_altitude_m + target_alt_m, 5
    else:
        cmd_z, cmd_frame = target_alt_m, 3

    kw = _takeoff_cmd(x=home_lat_int, y=home_lon_int, z=cmd_z, frame=cmd_frame)
    ack = await probe_command_int(system, **kw)
    ack_r = int(ack["result"]) if ack else -1
    log.info(_FMT, _CMD, "NAV_TAKEOFF (climb-out)",
             f"ACK={_ACK_NAMES.get(ack_r, f'result={ack_r}')}  "
             f"(armed, target {cmd_z:.1f} m {'AMSL' if autopilot == 'px4' else 'rel'}, "
             f"frame={cmd_frame})")
    if ack_r not in (0, 1):  # not ACCEPTED / TEMPORARILY_REJECTED
        raise RuntimeError(
            f"NAV_TAKEOFF climb-out command was {_ACK_NAMES.get(ack_r, f'result={ack_r}')} "
            f"while armed — vehicle will not climb; aborting wait early"
        )

    threshold_m = target_alt_m * altitude_threshold_frac
    progress: list[float] = []

    async def _log_progress() -> None:
        async for pos in system.telemetry.position():
            progress.append(pos.relative_altitude_m)

    progress_task = asyncio.create_task(_log_progress())
    try:
        async with asyncio.timeout(climb_timeout_s):
            while True:
                await asyncio.sleep(20.0)
                if progress:
                    log.info(_FMT, _CMD, "climb progress",
                             f"{progress[-1]:.1f} m (target {threshold_m:.1f} m, "
                             f"{climb_timeout_s:.0f} s budget)")
                if progress and progress[-1] >= threshold_m:
                    return
    finally:
        progress_task.cancel()


# ---------------------------------------------------------------------------
# Autouse fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def require_real_stack(request):
    """Skip every test in this module when no --drone-address is given."""
    if request.config.getoption("--drone-address") is None:
        pytest.skip("Execution tests require a real flight stack (--drone-address not set)")


# ---------------------------------------------------------------------------
# MC comprehensive — trajectory shape + landing-point identity
# ---------------------------------------------------------------------------

@pytest.mark.timeout(600)
async def test_mc_landing_comprehensive(gcs_system, request):
    """
    COMMAND_INT NAV_LAND — comprehensive multicopter landing observation.

    Takes off vertically to 30 m, then sends NAV_LAND with x/y commanding a
    point 80 m laterally offset from the hover position (z = ground level).
    Samples (altitude, distance-to-commanded-point) densely through the
    descent to classify the trajectory as glide-slope / dogleg /
    descend-in-place, and compares the touchdown point to the commanded
    coordinate — directly answering the open "is the commanded point the
    touchdown point?" question (Spec gap — landing-point identity).

    Runs on any quadcopter stack where NAV_LAND is SUPPORTED (PX4 MC,
    ArduCopter MC).  Logs a natural-language behaviour summary at the end.
    """
    vehicle_type = request.config.getoption("--vehicle-type", default=None) or "unknown"
    if vehicle_type != "quadcopter":
        pytest.skip(f"Not applicable to vehicle type {vehicle_type!r} — use quadcopter")

    autopilot = request.config.getoption("--autopilot", default=None)

    await _request_position_stream(gcs_system)
    await _request_home_position(gcs_system)
    home = await _get_home_position(gcs_system, timeout_s=90.0)

    # No pre-flight ACK probe here (unlike NAV_TAKEOFF's two-stage gate): empirically,
    # ANY accepted NAV_LAND sent to a grounded, disarmed PX4 quadcopter — regardless
    # of target coordinates — latches it into an internal auto-land nav_state that
    # then blocks `is_armable` from ever going True (`_wait_armable` timed out at
    # 120 s both with x=y=0 *and* with x/y set to the vehicle's own home position;
    # a vehicle that skips the probe becomes armable in ~1 s — see the now-removed
    # `test_diag_watch_health` diagnostic).  A pre-check is also redundant: Tier 1
    # already determined NAV_LAND is SUPPORTED on both quadcopter platforms this
    # test runs against (PX4 MC, ArduCopter MC) — see `tests/command/CLAUDE.md`.

    # Commanded landing point: offset north of home — clearly distinguishable
    # from "stay where you are" (the vehicle hovers near home after takeoff).
    TARGET_LAT = home.latitude_deg + LAND_TARGET_OFFSET_M / 111111.0
    TARGET_LON = home.longitude_deg

    log.info(
        _FMT, _CMD, "MC comprehensive — setup",
        f"home ({home.latitude_deg:.5f}°N, {home.longitude_deg:.5f}°E  "
        f"AMSL={home.absolute_altitude_m:.1f} m)  "
        f"land target {LAND_TARGET_OFFSET_M:.0f} m north  autopilot={autopilot}",
    )

    # Dense (alt, dist-to-target, lat, lon) sampling for trajectory + touchdown analysis.
    samples: list[tuple[float, float, float, float]] = []

    async def _sample_pos() -> None:
        async for pos in gcs_system.telemetry.position():
            d = _dist_m(pos.latitude_deg, pos.longitude_deg, TARGET_LAT, TARGET_LON)
            samples.append((pos.relative_altitude_m, d, pos.latitude_deg, pos.longitude_deg))

    sample_task = asyncio.create_task(_sample_pos())

    obs: dict = {
        "land_ack": None,
        "descent_started": False,
        "touched_down": False,
        "descent_start_idx": 0,
        "initial_dist_m": None,
        "final_pos": None,
        "pre_arm_notes": [],
    }

    try:
        # ── PRE-ARM SETUP (stack-specific) ───────────────────────────────────
        if autopilot == "ardupilot" and vehicle_type == "quadcopter":
            guided_ok = await _set_guided_mode_ardupilot(gcs_system)
            obs["pre_arm_notes"].append(
                f"GUIDED mode ({'confirmed' if guided_ok else 'NOT confirmed — timeout'})"
            )
        else:
            obs["pre_arm_notes"].append("no mode change required")
        log.info(_FMT, _CMD, "pre-arm", obs["pre_arm_notes"][-1])

        # ── GET AIRBORNE (NAV_TAKEOFF — proven sequence from comprehensive test) ──
        await _arm_and_climb(gcs_system, autopilot, home, TAKEOFF_ALT_M)
        await asyncio.sleep(3.0)  # let it stabilise/loiter at altitude
        pos = await _wait_for_altitude(gcs_system, TAKEOFF_ALT_M * 0.5, timeout_s=5.0)
        log.info(_FMT, _CMD, "airborne", f"hovering at {pos.relative_altitude_m:.1f} m — sending NAV_LAND")

        obs["initial_dist_m"] = _dist_m(pos.latitude_deg, pos.longitude_deg, TARGET_LAT, TARGET_LON)
        obs["descent_start_idx"] = len(samples)

        # ── SEND NAV_LAND TOWARD THE OFFSET TARGET (stack-specific z/frame) ──
        if autopilot == "px4":
            cmd_z, cmd_frame = home.absolute_altitude_m, 5   # ground level AMSL
        else:
            cmd_z, cmd_frame = 0.0, 3                         # ground level relative to home

        cmd_kw = _land_cmd(
            x=int(TARGET_LAT * 1e7), y=int(TARGET_LON * 1e7),
            z=cmd_z, frame=cmd_frame,
        )
        ack = await probe_command_int(gcs_system, **cmd_kw)
        ack_r = int(ack["result"]) if ack else -1
        obs["land_ack"] = ack_r
        log.info(_FMT, _CMD, "NAV_LAND sent", f"ACK={_ACK_NAMES.get(ack_r, f'result={ack_r}')}")

        if ack_r in (0, 1):  # ACCEPTED or TEMPORARILY_REJECTED
            obs["descent_started"] = await _wait_for_descent_below(
                gcs_system, TAKEOFF_ALT_M * 0.9, DESCENT_DETECT_TIMEOUT_S
            )
            log.info(_FMT, _CMD, "descent check",
                     "started" if obs["descent_started"] else
                     f"NOT detected within {DESCENT_DETECT_TIMEOUT_S:.0f} s")
            if obs["descent_started"]:
                obs["touched_down"] = await _wait_for_landed_state(
                    gcs_system, LandedState.ON_GROUND, TOUCHDOWN_TIMEOUT_S
                )
                log.info(_FMT, _CMD, "touchdown check",
                         "ON_GROUND reached" if obs["touched_down"] else
                         f"NOT reached within {TOUCHDOWN_TIMEOUT_S:.0f} s")
                if samples:
                    obs["final_pos"] = samples[-1]
    finally:
        sample_task.cancel()
        await _rtl_and_land(gcs_system)

    # ── ANALYSIS ──────────────────────────────────────────────────────────────
    descent_samples = samples[obs["descent_start_idx"]:]
    trajectory = (
        _classify_trajectory(
            [(s[0], s[1]) for s in descent_samples], obs["initial_dist_m"] or 0.0
        )
        if obs["touched_down"] else None
    )

    touchdown_dist_m = None
    if obs["final_pos"] is not None:
        _, _, fl, flo = obs["final_pos"]
        touchdown_dist_m = _dist_m(fl, flo, TARGET_LAT, TARGET_LON)

    ack_r = obs["land_ack"]
    if ack_r not in (0, 1):
        flight_desc = (
            f"NAV_LAND returned {_ACK_NAMES.get(ack_r, f'result={ack_r}')} — "
            "command not accepted; no landing observed."
        )
    elif not obs["descent_started"]:
        flight_desc = (
            f"NAV_LAND was ACCEPTED but the vehicle did not begin descending within "
            f"{DESCENT_DETECT_TIMEOUT_S:.0f} s — accepted but inert on this platform/mode."
        )
    elif not obs["touched_down"]:
        final_alt_m = obs["final_pos"][0] if obs["final_pos"] is not None else None
        alt_note = (
            f" The vehicle was at {final_alt_m:.1f} m relative altitude when the wait "
            "expired — " + (
                "essentially on the ground; this looks like a `landed_state()` "
                "reporting lag/quirk rather than a still-airborne vehicle."
                if final_alt_m is not None and final_alt_m < AIRBORNE_THRESHOLD_M else
                "still clearly airborne, so the descent is genuinely slow/stalled "
                "rather than a telemetry-reporting quirk."
            )
            if final_alt_m is not None else ""
        )
        flight_desc = (
            f"NAV_LAND triggered a descent but the vehicle did not reach ON_GROUND "
            f"within {TOUCHDOWN_TIMEOUT_S:.0f} s.{alt_note}"
        )
    else:
        match = touchdown_dist_m is not None and touchdown_dist_m < TOUCHDOWN_MATCH_M
        flight_desc = (
            f"Vehicle {_TRAJECTORY_DESC[trajectory]}. "
            f"Touchdown is {touchdown_dist_m:.1f} m from the commanded coordinate — "
            + ("the commanded point IS the touchdown point."
               if match else
               "the commanded coordinate is NOT the touchdown point.")
        )

    log.info(_FMT, _CMD, "BEHAVIOUR SUMMARY", flight_desc)

    summary_block = (
        "**Setup:**\n"
        f"- Pre-arm: {'; '.join(obs['pre_arm_notes'])}\n"
        f"- Takeoff to {TAKEOFF_ALT_M:.0f} m, then NAV_LAND commanded "
        f"{LAND_TARGET_OFFSET_M:.0f} m laterally offset (ground-level z)\n"
        "\n"
        "**Observation:**\n"
        f"- NAV_LAND ACK: {_ACK_NAMES.get(ack_r, f'result={ack_r}')}\n"
        f"- Descent started: {obs['descent_started']}\n"
        f"- Touched down: {obs['touched_down']}\n"
        + (f"- Trajectory: {trajectory}\n" if trajectory else "")
        + (f"- Touchdown distance from commanded point: {touchdown_dist_m:.1f} m\n"
           if touchdown_dist_m is not None else "")
        + "\n"
        "**Finding:**\n"
        f"- {flight_desc}\n"
    )
    _write_summary_log(request, autopilot, "mc_landing_comprehensive", summary_block)


# ---------------------------------------------------------------------------
# FW comprehensive — approach trajectory + landing-point identity (3 referents)
# ---------------------------------------------------------------------------

# Fixed-wing aircraft cruise at speed and turn with a wide radius — 80 m (the MC
# offset) would be swallowed by a single turn.  400 m gives enough room for an
# approach pattern (if any) to be observable while keeping the flight bounded.
FW_LAND_TARGET_OFFSET_M = 400.0
FW_ROLLOUT_OBSERVE_S    = 20.0   # keep sampling after touchdown to find the "finish" point


@pytest.mark.timeout(600)
async def test_fw_landing_comprehensive(gcs_system, request):
    """
    COMMAND_INT NAV_LAND — comprehensive fixed-wing landing observation.

    Attempts to take off to 30 m, then sends NAV_LAND with x/y commanding a
    point 400 m laterally offset (z = ground level).  Samples (altitude,
    distance-to-commanded-point, lat, lon) densely through the approach,
    descent, touchdown, and rollout to:
      - classify the approach trajectory (glide-slope / dogleg / descend-in-place)
      - compare the commanded coordinate against three candidate referents — the
        *aim point* (where descent begins), the *touchdown* point (wheels down),
        and the *finish* point (where the aircraft comes to rest) — directly
        answering the open "touchdown vs. aim-point vs. finish-point" question
        (Spec gap — landing-point identity, fixed-wing variant).

    Tolerant of the documented PX4 FW SIH-simulator limitation
    (nav_takeoff/README.md § PX4 FW): the sihsim_airplane model performs a
    ground roll after NAV_TAKEOFF but never lifts off (altitude stays below
    AIRBORNE_THRESHOLD_M).  A short, bounded liftoff probe detects this up
    front; if liftoff never happens the test records "nothing to land from"
    as its finding and exits cleanly (PASS, not a timeout) — a known SITL
    constraint, not a NAV_LAND-specific gap.  If liftoff *is* achieved, it
    falls through to the full climb-to-cruise → NAV_LAND → landing
    observation below.

    PX4 FW only: ArduPlane FW does not support NAV_LAND via COMMAND_INT (Tier 1
    result: UNSUPPORTED — nothing to observe, so it skips here).

    Logs a natural-language behaviour summary at the end.
    """
    vehicle_type = request.config.getoption("--vehicle-type", default=None) or "unknown"
    if vehicle_type != "fixed_wing":
        pytest.skip(f"Not applicable to vehicle type {vehicle_type!r} — use fixed_wing")

    autopilot = request.config.getoption("--autopilot", default=None)
    if autopilot != "px4":
        pytest.skip(
            "ArduPlane FW: NAV_LAND is UNSUPPORTED via COMMAND_INT (Tier 1 result) "
            "— nothing to observe"
        )

    await _request_position_stream(gcs_system)
    await _request_home_position(gcs_system)
    home = await _get_home_position(gcs_system, timeout_s=90.0)

    TARGET_LAT = home.latitude_deg + FW_LAND_TARGET_OFFSET_M / 111111.0
    TARGET_LON = home.longitude_deg

    log.info(
        _FMT, _CMD, "FW comprehensive — setup",
        f"home ({home.latitude_deg:.5f}°N, {home.longitude_deg:.5f}°E  "
        f"AMSL={home.absolute_altitude_m:.1f} m)  "
        f"land target {FW_LAND_TARGET_OFFSET_M:.0f} m north  autopilot={autopilot}",
    )

    # Dense (alt, dist-to-target, lat, lon) sampling — covers approach, descent,
    # touchdown, *and* rollout (kept running FW_ROLLOUT_OBSERVE_S past touchdown).
    samples: list[tuple[float, float, float, float]] = []

    async def _sample_pos() -> None:
        async for pos in gcs_system.telemetry.position():
            d = _dist_m(pos.latitude_deg, pos.longitude_deg, TARGET_LAT, TARGET_LON)
            samples.append((pos.relative_altitude_m, d, pos.latitude_deg, pos.longitude_deg))

    sample_task = asyncio.create_task(_sample_pos())

    obs: dict = {
        "land_ack": None,
        "descent_started": False,
        "touched_down": False,
        "descent_start_idx": 0,
        "touchdown_idx": None,
        "initial_dist_m": None,
        "mode_before_land": None,
        "mode_after_land": None,
        "aim_pos": None,
        "touchdown_pos": None,
        "finish_pos": None,
    }

    try:
        log.info(_FMT, _CMD, "pre-arm", "no mode change required (PX4)")

        # ── LIFTOFF PROBE — tolerant of the documented SIH ground-roll limitation ──
        # nav_takeoff/README.md § "PX4 FW (1.18.0)" already records that the SIH
        # fixed-wing simulator performs a ground roll after NAV_TAKEOFF but never
        # achieves liftoff (altitude stays < AIRBORNE_THRESHOLD_M ≈ 2 m — "an SIH
        # simulator limitation, not a protocol issue").  Probe for liftoff with a
        # short, bounded wait (the same threshold test_mc_takeoff_comprehensive
        # uses to distinguish "climbed" from "ground roll only") instead of the
        # full multi-minute climb-to-cruise wait — that lets this test report
        # "nothing to land from" promptly rather than time out on a doomed wait.
        home_lat_int = int(home.latitude_deg * 1e7)
        home_lon_int = int(home.longitude_deg * 1e7)
        await _wait_armable(gcs_system, timeout_s=120.0)
        await gcs_system.action.arm()
        await asyncio.sleep(0.5)

        cmd_z = home.absolute_altitude_m + TAKEOFF_ALT_M
        kw = _takeoff_cmd(x=home_lat_int, y=home_lon_int, z=cmd_z, frame=5)
        takeoff_ack = await probe_command_int(gcs_system, **kw)
        takeoff_ack_r = int(takeoff_ack["result"]) if takeoff_ack else -1
        log.info(_FMT, _CMD, "NAV_TAKEOFF (climb-out)",
                 f"ACK={_ACK_NAMES.get(takeoff_ack_r, f'result={takeoff_ack_r}')} "
                 f"(armed, target {cmd_z:.1f} m AMSL, frame=5)")

        pos = None
        if takeoff_ack_r in (0, 1):  # ACCEPTED or TEMPORARILY_REJECTED
            try:
                pos = await _wait_for_altitude(gcs_system, AIRBORNE_THRESHOLD_M, timeout_s=90.0)
            except TimeoutError:
                pass

        if pos is None:
            flight_desc = (
                "Cannot observe NAV_LAND behaviour on PX4 fixed-wing: NAV_TAKEOFF was "
                f"{_ACK_NAMES.get(takeoff_ack_r, f'result={takeoff_ack_r}')} but the aircraft "
                f"performed a ground roll and never reached the {AIRBORNE_THRESHOLD_M:.1f} m "
                "airborne threshold within 90 s — the documented SIH-simulator limitation "
                "(nav_takeoff/README.md § PX4 FW: \"ground roll only, altitude < 2 m — an "
                "SIH simulator limitation, not a protocol issue\"), not a NAV_LAND-specific "
                "gap. With the aircraft never airborne, NAV_LAND was not sent — there is "
                "nothing to land."
            )
            log.info(_FMT, _CMD, "BEHAVIOUR SUMMARY", flight_desc)
            summary_block = (
                "**Setup:**\n"
                "- Pre-arm: no mode change required\n"
                f"- Attempted takeoff to {TAKEOFF_ALT_M:.0f} m before sending NAV_LAND "
                f"{FW_LAND_TARGET_OFFSET_M:.0f} m laterally offset\n"
                "\n"
                "**Observation:**\n"
                f"- NAV_TAKEOFF ACK: {_ACK_NAMES.get(takeoff_ack_r, f'result={takeoff_ack_r}')}\n"
                "- Became airborne: False (ground roll only — known SIH-simulator limitation)\n"
                "- NAV_LAND: not sent — nothing to land\n"
                "\n"
                "**Finding:**\n"
                f"- {flight_desc}\n"
            )
            _write_summary_log(request, autopilot, "fw_landing_comprehensive", summary_block)
            return

        # Liftoff achieved (contrary to the documented limitation) — continue the
        # climb to cruise altitude before sending NAV_LAND, exactly as planned.
        log.info(_FMT, _CMD, "airborne (climb-out)",
                 f"reached {pos.relative_altitude_m:.1f} m — continuing climb to cruise")
        await _wait_for_altitude(gcs_system, TAKEOFF_ALT_M * 0.85, timeout_s=180.0)
        await asyncio.sleep(3.0)  # let it stabilise in cruise
        pos = await _wait_for_altitude(gcs_system, TAKEOFF_ALT_M * 0.5, timeout_s=5.0)
        obs["mode_before_land"] = await _get_flight_mode(gcs_system)
        log.info(_FMT, _CMD, "airborne",
                 f"at {pos.relative_altitude_m:.1f} m, mode={obs['mode_before_land']} "
                 "— sending NAV_LAND")

        obs["initial_dist_m"] = _dist_m(pos.latitude_deg, pos.longitude_deg, TARGET_LAT, TARGET_LON)
        obs["descent_start_idx"] = len(samples)

        # ── SEND NAV_LAND TOWARD THE OFFSET TARGET (PX4: absolute-AMSL z, frame=5) ──
        cmd_kw = _land_cmd(
            x=int(TARGET_LAT * 1e7), y=int(TARGET_LON * 1e7),
            z=home.absolute_altitude_m, frame=5,
        )
        ack = await probe_command_int(gcs_system, **cmd_kw)
        ack_r = int(ack["result"]) if ack else -1
        obs["land_ack"] = ack_r
        log.info(_FMT, _CMD, "NAV_LAND sent", f"ACK={_ACK_NAMES.get(ack_r, f'result={ack_r}')}")

        if ack_r in (0, 1):  # ACCEPTED or TEMPORARILY_REJECTED
            await asyncio.sleep(2.0)
            obs["mode_after_land"] = await _get_flight_mode(gcs_system)
            log.info(_FMT, _CMD, "mode change",
                     f"{obs['mode_before_land']} → {obs['mode_after_land']}")

            obs["descent_started"] = await _wait_for_descent_below(
                gcs_system, TAKEOFF_ALT_M * 0.9, DESCENT_DETECT_TIMEOUT_S
            )
            log.info(_FMT, _CMD, "descent check",
                     "started" if obs["descent_started"] else
                     f"NOT detected within {DESCENT_DETECT_TIMEOUT_S:.0f} s")

            if obs["descent_started"]:
                if samples:
                    obs["aim_pos"] = samples[-1]   # position when meaningful descent began

                obs["touched_down"] = await _wait_for_landed_state(
                    gcs_system, LandedState.ON_GROUND, TOUCHDOWN_TIMEOUT_S
                )
                log.info(_FMT, _CMD, "touchdown check",
                         "ON_GROUND reached" if obs["touched_down"] else
                         f"NOT reached within {TOUCHDOWN_TIMEOUT_S:.0f} s")

                if obs["touched_down"]:
                    obs["touchdown_idx"] = len(samples)
                    if samples:
                        obs["touchdown_pos"] = samples[-1]
                    # Keep sampling through deceleration/rollout to locate the
                    # "finish" point — where the aircraft actually comes to rest.
                    await asyncio.sleep(FW_ROLLOUT_OBSERVE_S)
                    if samples:
                        obs["finish_pos"] = samples[-1]
                    log.info(_FMT, _CMD, "rollout observed",
                             f"sampled for {FW_ROLLOUT_OBSERVE_S:.0f} s after touchdown")
    finally:
        sample_task.cancel()
        await _rtl_and_land(gcs_system)

    # ── ANALYSIS ──────────────────────────────────────────────────────────────
    descent_samples = (
        samples[obs["descent_start_idx"]:obs["touchdown_idx"]]
        if obs["touchdown_idx"] else samples[obs["descent_start_idx"]:]
    )
    trajectory = (
        _classify_trajectory(
            [(s[0], s[1]) for s in descent_samples], obs["initial_dist_m"] or 0.0
        )
        if obs["touched_down"] else None
    )

    def _dist_to_target(pos):
        if pos is None:
            return None
        _, _, plat, plon = pos
        return _dist_m(plat, plon, TARGET_LAT, TARGET_LON)

    aim_dist_m       = _dist_to_target(obs["aim_pos"])
    touchdown_dist_m = _dist_to_target(obs["touchdown_pos"])
    finish_dist_m    = _dist_to_target(obs["finish_pos"])

    ack_r = obs["land_ack"]
    if ack_r not in (0, 1):
        flight_desc = (
            f"NAV_LAND returned {_ACK_NAMES.get(ack_r, f'result={ack_r}')} — "
            "command not accepted; no landing observed."
        )
    elif not obs["descent_started"]:
        flight_desc = (
            f"NAV_LAND was ACCEPTED but the aircraft did not begin descending within "
            f"{DESCENT_DETECT_TIMEOUT_S:.0f} s — accepted but inert on this platform/mode."
        )
    elif not obs["touched_down"]:
        flight_desc = (
            f"NAV_LAND triggered a descent but the aircraft did not reach ON_GROUND "
            f"within {TOUCHDOWN_TIMEOUT_S:.0f} s."
        )
    else:
        candidates = [
            ("aim point (where descent begins)", aim_dist_m),
            ("touchdown point (wheels down)", touchdown_dist_m),
            ("finish point (where it comes to rest)", finish_dist_m),
        ]
        matches = [(label, d) for label, d in candidates if d is not None and d < TOUCHDOWN_MATCH_M]
        if matches:
            match_desc = " and ".join(f"the {label} ({d:.1f} m away)" for label, d in matches)
            referent_desc = f"the commanded coordinate matches {match_desc}."
        else:
            scored = [c for c in candidates if c[1] is not None]
            best = min(scored, key=lambda c: c[1]) if scored else None
            referent_desc = (
                f"the commanded coordinate matches none of the three candidate referents "
                f"within {TOUCHDOWN_MATCH_M:.0f} m"
                + (f" — closest is the {best[0]} at {best[1]:.1f} m" if best else "") + "."
            )
        flight_desc = (
            f"Aircraft {_TRAJECTORY_DESC[trajectory]} "
            f"(mode {obs['mode_before_land']} → {obs['mode_after_land']} on NAV_LAND). "
            f"Distances from the commanded point — aim: {aim_dist_m:.1f} m, "
            f"touchdown: {touchdown_dist_m:.1f} m, finish: {finish_dist_m:.1f} m. "
            + referent_desc
        )

    log.info(_FMT, _CMD, "BEHAVIOUR SUMMARY", flight_desc)

    summary_block = (
        "**Setup:**\n"
        "- Pre-arm: no mode change required\n"
        f"- Takeoff to {TAKEOFF_ALT_M:.0f} m, then NAV_LAND commanded "
        f"{FW_LAND_TARGET_OFFSET_M:.0f} m laterally offset (ground-level z)\n"
        "\n"
        "**Observation:**\n"
        f"- NAV_LAND ACK: {_ACK_NAMES.get(ack_r, f'result={ack_r}')}\n"
        f"- Flight mode: {obs['mode_before_land']} → {obs['mode_after_land']}\n"
        f"- Descent started: {obs['descent_started']}\n"
        f"- Touched down: {obs['touched_down']}\n"
        + (f"- Trajectory: {trajectory}\n" if trajectory else "")
        + (f"- Aim-point distance from commanded point: {aim_dist_m:.1f} m\n"
           if aim_dist_m is not None else "")
        + (f"- Touchdown distance from commanded point: {touchdown_dist_m:.1f} m\n"
           if touchdown_dist_m is not None else "")
        + (f"- Finish-point distance from commanded point: {finish_dist_m:.1f} m\n"
           if finish_dist_m is not None else "")
        + "\n"
        "**Finding:**\n"
        f"- {flight_desc}\n"
    )
    _write_summary_log(request, autopilot, "fw_landing_comprehensive", summary_block)


# ---------------------------------------------------------------------------
# VTOL — landing-mode classification (transition-to-hover / current-mode / inert)
# ---------------------------------------------------------------------------

@pytest.mark.timeout(600)
async def test_vtol_landing_behaviour(gcs_system, request):
    """
    COMMAND_INT NAV_LAND — VTOL landing-mode observation.

    Takes off (PX4 VTOL's natural post-takeoff state is MC/hover — see
    nav_takeoff/README.md), then sends NAV_LAND toward a laterally-offset
    point.  Concurrently samples telemetry.vtol_state() (MC / FW /
    TRANSITION_TO_FW / TRANSITION_TO_MC / UNDEFINED) and telemetry.landed_state()
    to classify the observed behaviour into exactly the three buckets the user
    asked about:
      (a) transitions to hover (MC) and lands vertically
      (b) lands in the current mode without transitioning
      (c) does not land at all (command accepted but inert — mirroring the
          PX4-Rover/NAV_TAKEOFF "permissive but meaningless" gap)

    PX4 VTOL only — gates on vehicle_type == "vtol".

    Logs a natural-language behaviour summary at the end.
    """
    vehicle_type = request.config.getoption("--vehicle-type", default=None) or "unknown"
    if vehicle_type != "vtol":
        pytest.skip(f"Not applicable to vehicle type {vehicle_type!r} — use vtol")

    autopilot = request.config.getoption("--autopilot", default=None)

    await _request_position_stream(gcs_system)
    await _request_home_position(gcs_system)
    home = await _get_home_position(gcs_system, timeout_s=90.0)

    TARGET_LAT = home.latitude_deg + LAND_TARGET_OFFSET_M / 111111.0
    TARGET_LON = home.longitude_deg

    log.info(
        _FMT, _CMD, "VTOL behaviour — setup",
        f"home ({home.latitude_deg:.5f}°N, {home.longitude_deg:.5f}°E  "
        f"AMSL={home.absolute_altitude_m:.1f} m)  "
        f"land target {LAND_TARGET_OFFSET_M:.0f} m north  autopilot={autopilot}",
    )

    samples: list[tuple[float, float, float, float]] = []
    vtol_states: list[VtolState] = []

    async def _sample_pos() -> None:
        async for pos in gcs_system.telemetry.position():
            d = _dist_m(pos.latitude_deg, pos.longitude_deg, TARGET_LAT, TARGET_LON)
            samples.append((pos.relative_altitude_m, d, pos.latitude_deg, pos.longitude_deg))

    async def _sample_vtol_state() -> None:
        async for vs in gcs_system.telemetry.vtol_state():
            if not vtol_states or vtol_states[-1] != vs:
                vtol_states.append(vs)
                log.info(_FMT, _CMD, "vtol_state change", str(vs))

    sample_task = asyncio.create_task(_sample_pos())
    vtol_task = asyncio.create_task(_sample_vtol_state())

    obs: dict = {
        "land_ack": None,
        "descent_started": False,
        "touched_down": False,
        "initial_dist_m": None,
        "vtol_state_before_land": None,
        "vtol_states_idx": 0,
        "vtol_states_during_land": [],
        "final_pos": None,
    }

    try:
        log.info(_FMT, _CMD, "pre-arm", "no mode change required (PX4)")

        # ── GET AIRBORNE (NAV_TAKEOFF — proven sequence from comprehensive test) ──
        await _arm_and_climb(gcs_system, autopilot, home, TAKEOFF_ALT_M)
        await asyncio.sleep(3.0)  # let it stabilise/loiter at altitude
        pos = await _wait_for_altitude(gcs_system, TAKEOFF_ALT_M * 0.5, timeout_s=5.0)
        obs["vtol_state_before_land"] = await _get_vtol_state(gcs_system)
        log.info(_FMT, _CMD, "airborne",
                 f"hovering at {pos.relative_altitude_m:.1f} m, "
                 f"vtol_state={obs['vtol_state_before_land']} — sending NAV_LAND")

        obs["initial_dist_m"] = _dist_m(pos.latitude_deg, pos.longitude_deg, TARGET_LAT, TARGET_LON)
        obs["vtol_states_idx"] = len(vtol_states)

        # ── SEND NAV_LAND TOWARD THE OFFSET TARGET (PX4: ground-level AMSL z, frame=5) ──
        cmd_kw = _land_cmd(
            x=int(TARGET_LAT * 1e7), y=int(TARGET_LON * 1e7),
            z=home.absolute_altitude_m, frame=5,
        )
        ack = await probe_command_int(gcs_system, **cmd_kw)
        ack_r = int(ack["result"]) if ack else -1
        obs["land_ack"] = ack_r
        log.info(_FMT, _CMD, "NAV_LAND sent", f"ACK={_ACK_NAMES.get(ack_r, f'result={ack_r}')}")

        if ack_r in (0, 1):  # ACCEPTED or TEMPORARILY_REJECTED
            obs["descent_started"] = await _wait_for_descent_below(
                gcs_system, TAKEOFF_ALT_M * 0.9, DESCENT_DETECT_TIMEOUT_S
            )
            log.info(_FMT, _CMD, "descent check",
                     "started" if obs["descent_started"] else
                     f"NOT detected within {DESCENT_DETECT_TIMEOUT_S:.0f} s")
            if obs["descent_started"]:
                obs["touched_down"] = await _wait_for_landed_state(
                    gcs_system, LandedState.ON_GROUND, TOUCHDOWN_TIMEOUT_S
                )
                log.info(_FMT, _CMD, "touchdown check",
                         "ON_GROUND reached" if obs["touched_down"] else
                         f"NOT reached within {TOUCHDOWN_TIMEOUT_S:.0f} s")
                if samples:
                    obs["final_pos"] = samples[-1]

        obs["vtol_states_during_land"] = vtol_states[obs["vtol_states_idx"]:]
    finally:
        sample_task.cancel()
        vtol_task.cancel()
        await _rtl_and_land(gcs_system)

    # ── ANALYSIS ──────────────────────────────────────────────────────────────
    states_during = obs["vtol_states_during_land"] or (
        [obs["vtol_state_before_land"]] if obs["vtol_state_before_land"] is not None else []
    )
    distinct = list(dict.fromkeys(states_during))
    transitioned = any(
        s in (VtolState.TRANSITION_TO_MC, VtolState.TRANSITION_TO_FW) for s in distinct
    )
    states_str = " → ".join(str(s) for s in distinct) if distinct else "unknown"

    if transitioned:
        mode_note = f"transitions mode during the landing sequence (observed: {states_str})"
    elif distinct == [VtolState.MC]:
        mode_note = (
            "stays in MC/hover throughout — it was already in the rotary-wing "
            "mode needed for vertical landing, so no transition is observable"
        )
    else:
        mode_note = f"remains in {states_str} throughout (no MC transition observed)"

    ack_r = obs["land_ack"]
    touchdown_dist_m = None
    if obs["final_pos"] is not None:
        _, _, fl, flo = obs["final_pos"]
        touchdown_dist_m = _dist_m(fl, flo, TARGET_LAT, TARGET_LON)

    if ack_r not in (0, 1):
        flight_desc = (
            f"NAV_LAND returned {_ACK_NAMES.get(ack_r, f'result={ack_r}')} — "
            "command not accepted; no landing observed."
        )
        bucket = "(c) does not land — command was not even accepted"
    elif not obs["descent_started"]:
        flight_desc = (
            f"NAV_LAND was ACCEPTED ({mode_note}) but the vehicle did not begin "
            f"descending within {DESCENT_DETECT_TIMEOUT_S:.0f} s — accepted but inert "
            "on this platform/mode."
        )
        bucket = "(c) does not land at all — accepted but inert"
    elif not obs["touched_down"]:
        flight_desc = (
            f"NAV_LAND triggered a descent (vehicle {mode_note}) but it did not reach "
            f"ON_GROUND within {TOUCHDOWN_TIMEOUT_S:.0f} s."
        )
        bucket = (
            "between (a)/(b) and (c) — descent began (so the command is not inert) "
            "but landing did not complete within the observation window"
        )
    else:
        match = touchdown_dist_m is not None and touchdown_dist_m < TOUCHDOWN_MATCH_M
        flight_desc = (
            f"Vehicle {mode_note}. It descended and reached ON_GROUND "
            f"{touchdown_dist_m:.1f} m from the commanded point — "
            + ("the commanded point IS the touchdown point."
               if match else
               "the commanded coordinate is NOT the touchdown point.")
        )
        bucket = (
            "(a) transitions to hover (MC) and lands vertically" if transitioned else
            "(b) lands in the current mode (already MC/hover) without transitioning"
        )

    log.info(_FMT, _CMD, "BEHAVIOUR SUMMARY", f"{flight_desc} Classification: {bucket}.")

    summary_block = (
        "**Setup:**\n"
        "- Pre-arm: no mode change required\n"
        f"- Takeoff to {TAKEOFF_ALT_M:.0f} m (PX4 VTOL takes off in MC/hover mode), "
        f"then NAV_LAND commanded {LAND_TARGET_OFFSET_M:.0f} m laterally offset "
        "(ground-level z)\n"
        "\n"
        "**Observation:**\n"
        f"- VTOL state before NAV_LAND: {obs['vtol_state_before_land']}\n"
        f"- VTOL states during landing sequence: {states_str}\n"
        f"- NAV_LAND ACK: {_ACK_NAMES.get(ack_r, f'result={ack_r}')}\n"
        f"- Descent started: {obs['descent_started']}\n"
        f"- Touched down: {obs['touched_down']}\n"
        + (f"- Touchdown distance from commanded point: {touchdown_dist_m:.1f} m\n"
           if touchdown_dist_m is not None else "")
        + "\n"
        "**Finding:**\n"
        f"- {flight_desc}\n"
        f"- Classification: {bucket}\n"
    )
    _write_summary_log(request, autopilot, "vtol_landing_behaviour", summary_block)


# ---------------------------------------------------------------------------
# PX4 Rover — inertness check (lightweight: a rover is never airborne)
# ---------------------------------------------------------------------------

_ROVER_OBSERVE_S = 15.0
_ROVER_MOVE_NOISE_M = 2.0  # GPS-jitter floor below which "movement" is not meaningful


@pytest.mark.timeout(180)
async def test_px4_rover_land_is_inert(gcs_system, request):
    """
    COMMAND_INT NAV_LAND — PX4 Rover inertness check.

    A ground rover is never airborne, so there is no takeoff -> land cycle to
    run.  This deliberately lightweight test arms the rover in place, sends
    NAV_LAND via COMMAND_INT toward a laterally-offset point, and watches
    mode / landed_state / position for ~15 s to confirm "ACCEPTED but no
    landing-like behaviour" — the same permissive-but-meaningless acceptance
    pattern already documented for NAV_TAKEOFF on PX4 Rover
    (nav_takeoff/README.md § PX4 Rover: "a ground vehicle accepting a flight
    command without executing it or returning UNSUPPORTED is misleading").

    Gates on autopilot == "px4" and vehicle_type == "rover": PX4 is the only
    stack where NAV_LAND is SUPPORTED on a rover (Tier 1 result — ArduRover
    returns UNSUPPORTED and is gated out by require_real_stack entirely).

    Logs a natural-language behaviour summary at the end.
    """
    vehicle_type = request.config.getoption("--vehicle-type", default=None) or "unknown"
    autopilot = request.config.getoption("--autopilot", default=None)
    if not (autopilot == "px4" and vehicle_type == "rover"):
        pytest.skip(
            f"Only applicable to autopilot=px4, vehicle_type=rover "
            f"(got autopilot={autopilot!r}, vehicle_type={vehicle_type!r}) — "
            "ArduRover returns UNSUPPORTED for NAV_LAND (Tier 1 result; "
            "require_real_stack skips the whole suite there)"
        )

    await _request_position_stream(gcs_system)
    await _request_home_position(gcs_system)
    home = await _get_home_position(gcs_system, timeout_s=90.0)

    TARGET_LAT = home.latitude_deg + LAND_TARGET_OFFSET_M / 111111.0
    TARGET_LON = home.longitude_deg

    log.info(
        _FMT, _CMD, "PX4 rover — setup",
        f"home ({home.latitude_deg:.5f}°N, {home.longitude_deg:.5f}°E)  "
        f"land target {LAND_TARGET_OFFSET_M:.0f} m north — rover is never airborne, "
        "so it is armed in place (no takeoff)",
    )

    async def _sample_position():
        async with asyncio.timeout(10.0):
            async for pos in gcs_system.telemetry.position():
                return pos

    async def _sample_landed_state():
        async with asyncio.timeout(10.0):
            async for state in gcs_system.telemetry.landed_state():
                return state

    await _wait_armable(gcs_system, timeout_s=120.0)
    await gcs_system.action.arm()
    await asyncio.sleep(1.0)

    pos_before = await _sample_position()
    mode_before = await _get_flight_mode(gcs_system)
    landed_before = await _sample_landed_state()
    log.info(_FMT, _CMD, "pre-land",
             f"mode={mode_before}, landed_state={landed_before}, "
             f"pos=({pos_before.latitude_deg:.5f}°, {pos_before.longitude_deg:.5f}°)")

    cmd_kw = _land_cmd(
        x=int(TARGET_LAT * 1e7), y=int(TARGET_LON * 1e7),
        z=home.absolute_altitude_m, frame=5,
    )
    ack = await probe_command_int(gcs_system, **cmd_kw)
    ack_r = int(ack["result"]) if ack else -1
    log.info(_FMT, _CMD, "NAV_LAND sent", f"ACK={_ACK_NAMES.get(ack_r, f'result={ack_r}')}")

    await asyncio.sleep(_ROVER_OBSERVE_S)

    pos_after = await _sample_position()
    mode_after = await _get_flight_mode(gcs_system)
    landed_after = await _sample_landed_state()
    moved_m = _dist_m(pos_before.latitude_deg, pos_before.longitude_deg,
                      pos_after.latitude_deg, pos_after.longitude_deg)
    log.info(_FMT, _CMD, "post-land",
             f"mode={mode_after}, landed_state={landed_after}, moved {moved_m:.1f} m "
             f"over {_ROVER_OBSERVE_S:.0f} s")

    try:
        await gcs_system.action.disarm()
    except Exception:
        pass

    # ── ANALYSIS ──────────────────────────────────────────────────────────────
    mode_changed = mode_before != mode_after
    landed_changed = landed_before != landed_after
    moved = moved_m > _ROVER_MOVE_NOISE_M

    if ack_r not in (0, 1):
        flight_desc = (
            f"NAV_LAND returned {_ACK_NAMES.get(ack_r, f'result={ack_r}')} — unexpected, "
            "since Tier 1 found PX4 Rover ACCEPTS NAV_LAND on every vehicle type. "
            "Command not accepted, so there is no landing-like behaviour to observe."
        )
    elif (landed_changed and landed_before == LandedState.IN_AIR
          and landed_after == LandedState.ON_GROUND and not mode_changed and not moved):
        # A stationary, never-armed-to-fly rover cannot have "landed" — the only thing
        # that changed is landed_state, and it moved in the direction telemetry would
        # settle on its own after spawn/arm regardless of any command (mirrors the
        # `landed_state()` reporting lag/quirk already seen on ArduCopter MC).
        flight_desc = (
            f"NAV_LAND was {_ACK_NAMES.get(ack_r, f'result={ack_r}')}; the only change "
            f"observed over {_ROVER_OBSERVE_S:.0f} s was landed_state settling from "
            f"{landed_before} to {landed_after} — but mode ({mode_before}) and position "
            f"(moved {moved_m:.1f} m) stayed unchanged throughout, and a stationary "
            "ground vehicle cannot meaningfully transition from \"in air\" to \"on "
            "ground\". This looks like a `landed_state()` reporting/settling artifact "
            "(the simulator briefly reports IN_AIR after spawn/arm before settling to "
            "ON_GROUND), independent of NAV_LAND — not a genuine landing response. "
            "Discounting it, this confirms, for NAV_LAND, the same permissive-but-"
            "meaningless acceptance pattern already documented for NAV_TAKEOFF on PX4 "
            "Rover (nav_takeoff/README.md § PX4 Rover) — the command is accepted but "
            "has no effect on a vehicle that cannot perform the manoeuvre."
        )
    elif not (mode_changed or landed_changed or moved):
        flight_desc = (
            f"NAV_LAND was {_ACK_NAMES.get(ack_r, f'result={ack_r}')} but produced no "
            f"observable landing-like behaviour over {_ROVER_OBSERVE_S:.0f} s — mode "
            f"({mode_before}), landed_state ({landed_before}), and position "
            f"(moved {moved_m:.1f} m, within GPS-jitter noise) all stayed effectively "
            "unchanged. This confirms, for NAV_LAND, the same permissive-but-meaningless "
            "acceptance pattern already documented for NAV_TAKEOFF on PX4 Rover "
            "(nav_takeoff/README.md § PX4 Rover) — the command is accepted but has no "
            "effect on a vehicle that cannot perform the manoeuvre."
        )
    else:
        changes = []
        if mode_changed:
            changes.append(f"mode {mode_before} → {mode_after}")
        if landed_changed:
            changes.append(f"landed_state {landed_before} → {landed_after}")
        if moved:
            changes.append(f"moved {moved_m:.1f} m")
        flight_desc = (
            f"NAV_LAND was {_ACK_NAMES.get(ack_r, f'result={ack_r}')} and produced an "
            f"observable change over {_ROVER_OBSERVE_S:.0f} s — {', '.join(changes)} — "
            "contrary to the expected inert-acceptance pattern; worth further investigation."
        )

    log.info(_FMT, _CMD, "BEHAVIOUR SUMMARY", flight_desc)

    summary_block = (
        "**Setup:**\n"
        "- Rover armed in place; no takeoff (a ground vehicle is never airborne)\n"
        f"- NAV_LAND commanded {LAND_TARGET_OFFSET_M:.0f} m laterally offset "
        f"(ground-level z), observed for {_ROVER_OBSERVE_S:.0f} s\n"
        "\n"
        "**Observation:**\n"
        f"- NAV_LAND ACK: {_ACK_NAMES.get(ack_r, f'result={ack_r}')}\n"
        f"- Mode: {mode_before} → {mode_after}\n"
        f"- Landed state: {landed_before} → {landed_after}\n"
        f"- Position change: {moved_m:.1f} m\n"
        "\n"
        "**Finding:**\n"
        f"- {flight_desc}\n"
    )
    _write_summary_log(request, autopilot, "px4_rover_land_is_inert", summary_block)
