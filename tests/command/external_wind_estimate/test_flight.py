"""
MAV_CMD_EXTERNAL_WIND_ESTIMATE (cmd=43004) — Tier 2 execution tests: ground vs air.

Tier 1 (test_command.py) can only observe the COMMAND_ACK result, which is
unconditionally ACCEPTED by PX4 regardless of vehicle state (source-confirmed,
see below).  It CANNOT show whether the command actually changed the EKF wind
estimate — and, critically, whether that differs between "on the ground" and
"in the air".  This module answers that execution-semantics question by
observing the WIND_COV (msg id=231) MAVLink stream, which mirrors the EKF's
internal `wind` uORB topic (wind_x = windspeed_north, wind_y = windspeed_east;
src/modules/mavlink/streams/WIND_COV.hpp), before and after sending the command
in each state.

Headline finding (source-traced AND empirically confirmed below; see
README.md § "Ground vs air" for the full writeup and observed values):

    PX4's Ekf::resetWindToExternalObservation() (src/modules/ekf2/EKF/wind.cpp)
    is gated by the vehicle's current landed state (originally
    `!_control_status.flags.in_air`; re-sourced from the `vehicle_land_detected`
    uORB topic by commit b4a5854c62, same effect) — the wind-state reset is
    applied ONLY while the vehicle is on the ground, and is rejected while
    airborne.

As of commit 793d308c53 (branch fix_external_wind_estimate_mavlink,
2026-09-09), the COMMAND_ACK handler in EKF2.cpp DOES reflect this: it returns
TEMPORARILY_REJECTED (not ACCEPTED) when the reset was not applied — so a GCS
CAN now tell from the ACK alone whether the estimate was actually applied.
(Pre-fix, the handler returned ACCEPTED unconditionally in both states, which
is what this module originally caught.)

What remains a DOC DISCREPANCY worth flagging: the command's own description
in development.xml explicitly describes an *in-flight* use case ("...extending
the time when operating without GPS before position drift builds to an unsafe
level... the command might reasonably be sent every few minutes when
operating at altitude") — yet the current PX4 implementation still only
honours the command while landed; it now correctly REJECTS the in-flight case
instead of silently discarding it, but still doesn't implement the in-flight
behaviour the spec describes. See CLAUDE.md § MAV_CMD_EXTERNAL_WIND_ESTIMATE
and README.md for the write-up; this looks like a PX4 implementation gap
rather than a MAVLink spec problem (the spec text is unambiguous; PX4 just
doesn't implement the in-air half of it).

Test 1 (ground) sends the command while disarmed/landed and expects the WIND_COV
values to move to (approximately) the commanded speed/direction, with ACK
ACCEPTED.
Test 2 (air) arms, takes off, and sends the command with a distinguishable
speed/direction — and expects the WIND_COV values to stay close to whatever
they already were (i.e. NOT move toward the newly-commanded values), confirming
the landed-state gate empirically rather than by source inspection alone. Both
ACCEPTED and TEMPORARILY_REJECTED are tolerated for the air ACK (this module
runs against both pre- and post-793d308c53 PX4 checkouts); only UNSUPPORTED
would indicate the command itself isn't recognised.

Running
-------
PX4 SIH multicopter (head revision)::

    pytest tests/command/external_wind_estimate/test_flight.py \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \\
        --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO

Skipped entirely in paired/mock mode (no --drone-address) — MockFlightStack
has no EKF and does not publish WIND_COV.
"""

import asyncio
import json
import logging
import math

import pytest

from tests.command.conftest import probe_command_long
from tests.command.external_wind_estimate.test_command import _probe, _reduce
from tests.flight_helpers import (
    AIRBORNE_THRESHOLD_M,
    _arm_and_send_takeoff,
    _request_position_stream,
    _rtl_and_land,
    _wait_for_altitude,
)
from tests.mock_flight_stack import MAV_RESULT_UNSUPPORTED

log = logging.getLogger(__name__)

pytestmark = pytest.mark.timeout(300)

_CMD = "EXTERNAL_WIND_ESTIMATE"

_WIND_COV_MSG_ID = 231       # MAVLINK_MSG_ID_WIND_COV
_SET_MESSAGE_INTERVAL = 511  # MAV_CMD_SET_MESSAGE_INTERVAL
_WIND_COV_RATE_HZ = 5.0

_BASELINE_SETTLE_S = 1.5   # window to check for (normally absent) WIND_COV before the command
_AFTER_SETTLE_S = 3.0      # window to see whether/when WIND_COV starts publishing after the command
_WIND_TOLERANCE_M_S = 3.0  # tolerance for "close to commanded value"

# Ground test: wind blowing FROM east (90 deg) at 8 m/s -> blows TO west
#   -> expected windspeed_north ~= 0, windspeed_east ~= -8
GROUND_WIND_SPEED = 8.0
GROUND_WIND_DIR = 90.0

# Air test: a deliberately distinguishable value -- wind blowing FROM south (180 deg)
# at 15 m/s -> blows TO north -> expected windspeed_north ~= +15, windspeed_east ~= 0
# IF applied. The whole point of this test is to show it is NOT applied in-air.
AIR_WIND_SPEED = 15.0
AIR_WIND_DIR = 180.0

TAKEOFF_ALT_M = 20.0


def _expected_wind_ne(speed: float, direction_from_deg: float) -> tuple[float, float]:
    """Expected (north, east) wind components once the reset has been applied.

    Mirrors PX4's own math exactly (EKF2.cpp + wind.cpp +
    ComputeWindInitAndCovFromWindSpeedAndDirection): the "from" azimuth is
    converted to a "to" bearing by adding 180 deg, then north = speed*cos,
    east = speed*sin (angle measured clockwise from true north).
    """
    direction_to_rad = math.radians(direction_from_deg) + math.pi
    return speed * math.cos(direction_to_rad), speed * math.sin(direction_to_rad)


@pytest.fixture(autouse=True)
def require_real_stack(request):
    """Skip every test in this module when no --drone-address is given."""
    if request.config.getoption("--drone-address") is None:
        pytest.skip("Ground/air execution tests require a real flight stack (WIND_COV/EKF) "
                    "-- --drone-address not set")


async def _request_wind_cov_stream(system, rate_hz: float = _WIND_COV_RATE_HZ) -> None:
    """Request WIND_COV (msg id=231) via MAV_CMD_SET_MESSAGE_INTERVAL (511)."""
    interval_us = int(1_000_000 / rate_hz)
    await probe_command_long(
        system, _SET_MESSAGE_INTERVAL,
        param1=float(_WIND_COV_MSG_ID), param2=float(interval_us),
        param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0,
    )
    await asyncio.sleep(0.2)


async def _subscribe_wind_cov(system):
    """
    Start a background WIND_COV subscription.

    Returns (task, queue). Caller must cancel the task (fire-and-forget) when
    done -- do NOT await after cancel (gRPC stream caveat, CLAUDE.md §4a).
    """
    queue: asyncio.Queue = asyncio.Queue()

    async def _collect() -> None:
        async for msg in system.mavlink_direct.message("WIND_COV"):
            await queue.put(json.loads(msg.fields_json))

    task = asyncio.create_task(_collect())
    await asyncio.sleep(0.05)
    return task, queue


async def _wait_for_sample(queue: asyncio.Queue, wait_s: float) -> dict | None:
    """
    Wait up to wait_s for WIND_COV samples; return the freshest one seen, or
    None if none arrived in the whole window.

    PX4 only calls wind_s::publish() -- and therefore only starts sending
    WIND_COV at all -- once EKF2's `get_wind_status()` is true (source:
    estimator_interface.h: `_control_status.flags.wind || _external_wind_init`).
    Neither is true by default, so it is normal and expected for NO sample to
    arrive at all before the command is sent, and (per this module's core
    finding) for none to ever arrive while airborne even after the command is
    sent. A blocking "get the next one" wait would hang forever in exactly the
    case this test exists to demonstrate -- hence a bounded window that
    tolerates "nothing arrived" as a valid, meaningful outcome.
    """
    deadline = asyncio.get_event_loop().time() + wait_s
    latest = None
    while True:
        remaining = deadline - asyncio.get_event_loop().time()
        if remaining <= 0:
            break
        try:
            latest = await asyncio.wait_for(queue.get(), timeout=remaining)
        except asyncio.TimeoutError:
            break
    return latest


def _fmt_wind(sample: dict | None) -> str:
    if sample is None:
        return "none received"
    return f"north={sample.get('wind_x'):.2f} east={sample.get('wind_y'):.2f} m/s"


def _matches_commanded(sample: dict | None, expected_north: float, expected_east: float) -> bool:
    if sample is None:
        return False
    return (
        abs(sample["wind_x"] - expected_north) < _WIND_TOLERANCE_M_S
        and abs(sample["wind_y"] - expected_east) < _WIND_TOLERANCE_M_S
    )


# ---------------------------------------------------------------------------
# Test 1 -- ground (disarmed / landed)
# ---------------------------------------------------------------------------

async def test_ground_wind_estimate_applied(gcs_system, request):
    """
    On the ground (disarmed), the commanded wind speed/direction must be
    applied to the EKF wind state (observed via WIND_COV) -- this is the
    documented, spec-aligned use case: `!_control_status.flags.in_air` is true
    while landed, so Ekf::resetWindToExternalObservation() executes and sets
    `_external_wind_init = true`, which is what makes PX4 start publishing
    WIND_COV at all (see _wait_for_sample() docstring). No baseline sample is
    expected BEFORE the command -- that absence is itself confirmation that
    the wind estimate was not previously valid.
    """
    await _request_wind_cov_stream(gcs_system)
    wind_task, wind_queue = await _subscribe_wind_cov(gcs_system)

    try:
        baseline = await _wait_for_sample(wind_queue, _BASELINE_SETTLE_S)
        log.info("%-24s | %-30s | %s", _CMD, "ground baseline WIND_COV (before command)", _fmt_wind(baseline))

        # Uses _probe()/_reduce() (not a raw probe_command_long) -- sends both
        # COMMAND_INT and COMMAND_LONG and merges to one result, subject to
        # the same confirmed Commander/EKF2 dual-ACK race as Tier 1
        # (test_command.py) -- see its module docstring.
        int_ack, long_ack = await _probe(gcs_system, param1=GROUND_WIND_SPEED, param3=GROUND_WIND_DIR)
        result = _reduce("ground ACK", int_ack, long_ack)
        assert result is not None, "No COMMAND_ACK received for EXTERNAL_WIND_ESTIMATE (ground)"
        log.info("%-24s | %-30s | result=%d", _CMD, "ground ACK", result)
        assert result != MAV_RESULT_UNSUPPORTED

        after = await _wait_for_sample(wind_queue, _AFTER_SETTLE_S)
        log.info("%-24s | %-30s | %s", _CMD, "ground after-command WIND_COV", _fmt_wind(after))
    finally:
        wind_task.cancel()

    expected_north, expected_east = _expected_wind_ne(GROUND_WIND_SPEED, GROUND_WIND_DIR)
    log.info(
        "%-24s | %-30s | expected north=%.2f east=%.2f",
        _CMD, "ground expected wind (from PX4's own math)", expected_north, expected_east,
    )

    assert after is not None, (
        "No WIND_COV was published at all after the ground EXTERNAL_WIND_ESTIMATE reset -- "
        "expected the topic to start streaming once _external_wind_init is set"
    )
    assert _matches_commanded(after, expected_north, expected_east), (
        f"Ground WIND_COV did not move to the commanded wind vector: "
        f"got north={after['wind_x']:.2f} east={after['wind_y']:.2f}, "
        f"expected north={expected_north:.2f} east={expected_east:.2f} "
        f"(tolerance {_WIND_TOLERANCE_M_S} m/s)"
    )

    _write_ground_result(request, baseline, after, expected_north, expected_east)


# ---------------------------------------------------------------------------
# Test 2 -- air (armed, airborne)
# ---------------------------------------------------------------------------

async def test_air_wind_estimate_ignored(gcs_system, request):
    """
    While airborne, the commanded wind speed/direction is NOT applied to the
    EKF wind state -- the vehicle is not landed, so
    Ekf::resetWindToExternalObservation() returns false without doing
    anything, and `_external_wind_init` is never set. As of commit
    793d308c53, COMMAND_ACK correctly reflects this as TEMPORARILY_REJECTED
    rather than ACCEPTED -- see the module docstring for the pre-fix history
    of the "ACK says yes, nothing happened" gap this test originally caught.

    get_wind_status() (estimator_interface.h) is
    `_control_status.flags.wind || _external_wind_init`. `_external_wind_init`
    is sticky for the life of the PX4 boot once any EXTERNAL_WIND_ESTIMATE has
    been applied on the ground -- so if test_ground_wind_estimate_applied ran
    earlier in the same SITL session, WIND_COV may already be streaming here,
    and "baseline" below may show that EARLIER ground value rather than
    nothing. Either way, the assertion that matters is the same: the value
    observed AFTER sending this test's (distinguishable) air command must NOT
    match what was just commanded -- whether because no new sample arrives at
    all, or because samples keep arriving with the stale prior value.
    """
    await _request_position_stream(gcs_system)
    try:
        await _arm_and_send_takeoff(gcs_system, z=TAKEOFF_ALT_M)
        await _wait_for_altitude(gcs_system, AIRBORNE_THRESHOLD_M, timeout_s=30.0)

        await _request_wind_cov_stream(gcs_system)
        wind_task, wind_queue = await _subscribe_wind_cov(gcs_system)

        try:
            baseline = await _wait_for_sample(wind_queue, _BASELINE_SETTLE_S)
            log.info("%-24s | %-30s | %s", _CMD, "air baseline WIND_COV (before command)", _fmt_wind(baseline))

            # _probe()/_reduce() -- see the equivalent comment in test_ground_wind_estimate_applied.
            int_ack, long_ack = await _probe(gcs_system, param1=AIR_WIND_SPEED, param3=AIR_WIND_DIR)
            result = _reduce("air ACK", int_ack, long_ack)
            assert result is not None, "No COMMAND_ACK received for EXTERNAL_WIND_ESTIMATE (air)"
            log.info("%-24s | %-30s | result=%d", _CMD, "air ACK", result)
            assert result != MAV_RESULT_UNSUPPORTED

            after = await _wait_for_sample(wind_queue, _AFTER_SETTLE_S)
            log.info("%-24s | %-30s | %s", _CMD, "air after-command WIND_COV", _fmt_wind(after))
        finally:
            wind_task.cancel()
    finally:
        await _rtl_and_land(gcs_system)

    expected_north, expected_east = _expected_wind_ne(AIR_WIND_SPEED, AIR_WIND_DIR)
    log.info(
        "%-24s | %-30s | commanded-would-be north=%.2f east=%.2f",
        _CMD, "air commanded value (if it HAD been applied)", expected_north, expected_east,
    )

    log.warning(
        "DOC DISCREPANCY: PX4's resetWindToExternalObservation() (wind.cpp) only applies "
        "the estimate -- and only makes get_wind_status() true, which gates whether WIND_COV "
        "is published at all (EKF2.cpp PublishWindEstimate / estimator_interface.h) -- while "
        "landed. As of commit 793d308c53 the ACK correctly reports TEMPORARILY_REJECTED "
        "(not ACCEPTED) when this happens, so this is no longer a silent ACK/behaviour "
        "mismatch -- but the spec's own description explicitly describes an in-flight "
        "('operating at altitude') use case that PX4 still does not implement at all. "
        f"ack_result={result} baseline={_fmt_wind(baseline)} after={_fmt_wind(after)} "
        f"commanded-if-applied=(north={expected_north:.2f}, east={expected_east:.2f})"
    )

    assert not _matches_commanded(after, expected_north, expected_east), (
        "Airborne WIND_COV matched the commanded wind vector -- expected either no WIND_COV "
        "at all, or values not matching what was just commanded (PX4's in_air gate should "
        f"have made this a no-op); baseline={_fmt_wind(baseline)} after={_fmt_wind(after)}"
    )
    log.info(
        "%-24s | %-30s | %s",
        _CMD, "air result",
        "no WIND_COV published at all (in_air gate confirmed)" if after is None
        else "WIND_COV published but did not match commanded vector",
    )

    _write_air_result(request, baseline, after, expected_north, expected_east)


# ---------------------------------------------------------------------------
# Result logging -- per-run summary file (README.md is updated manually from these)
# ---------------------------------------------------------------------------

def _write_ground_result(request, baseline, after, expected_north, expected_east) -> None:
    _append_result(request, "GROUND", baseline, after, expected_north, expected_east,
                    "Applied -- WIND_COV moved to the commanded vector")


def _write_air_result(request, baseline, after, expected_north, expected_east) -> None:
    _append_result(request, "AIR", baseline, after, expected_north, expected_east,
                    "Ignored -- WIND_COV stayed near baseline (in_air gate)")


def _append_result(request, label, baseline, after, expected_north, expected_east, verdict) -> None:
    import datetime
    from pathlib import Path

    Path("logs").mkdir(exist_ok=True)
    vehicle = request.config.getoption("--vehicle-type", default="unknown") or "unknown"
    autopilot = request.config.getoption("--autopilot", default="unknown") or "unknown"
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = Path("logs") / f"command_external_wind_estimate_{label.lower()}_{autopilot}_{vehicle}_{ts}.log"
    path.write_text(
        f"EXTERNAL_WIND_ESTIMATE Tier 2 -- {label}\n"
        f"autopilot={autopilot} vehicle={vehicle} timestamp={ts}\n"
        f"baseline: {_fmt_wind(baseline)}\n"
        f"after:    {_fmt_wind(after)}\n"
        f"commanded-equivalent: north={expected_north:.3f} east={expected_east:.3f}\n"
        f"verdict: {verdict}\n",
        encoding="utf-8",
    )
    log.info("%-24s | %-30s | %s", _CMD, f"{label} result log written", str(path))
