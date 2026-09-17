"""
MAV_CMD_DO_SET_ACTUATOR (cmd=187) via COMMAND_INT/COMMAND_LONG — Tier 2 execution tests.

This is the decisive verification for PX4-Autopilot PR #28723 (see
test_command.py's module docstring for the full bug description). Tier 1's
ACK-only tests cannot distinguish correct from incorrect x/y scaling —
Commander.cpp answers DO_SET_ACTUATOR with an unconditional ACCEPTED
regardless of value — so this file observes the *actual decoded value*
externally, via a real PWM output.

Observability mechanism (confirmed empirically against this session's PX4
checkout before writing these tests, not assumed from source alone)
---------------------------------------------------------------------
`FunctionActuatorSet` (src/lib/mixer_module/functions/FunctionActuatorSet.hpp)
copies vehicle_command_s.param1-6 into up to 6 "Peripheral_via_Actuator_SetN"
output-function slots whenever `command == DO_SET_ACTUATOR && index (param7) == 0`.
Assigning a spare PWM output channel's `PWM_MAIN_FUNC<n>` parameter to the
matching function enum value (301 + (actuator_number - 1), e.g. 305 for
Actuator 5) makes that actuator's decoded value observable as a real PWM
microsecond value via `ACTUATOR_OUTPUT_STATUS` (msg id 375), scaled
1000-2000us for -1..1 (PWM = 1000 + (value + 1) * 500). `sihsim_quadx` only
assigns PWM_MAIN_FUNC1-4 to the four motors; channels 5-8 are free and
confirmed (empirically, against the running SITL instance, not assumed)
to actually activate in `ACTUATOR_OUTPUT_STATUS` — channels 9+ do NOT,
despite the `PWM_MAIN_FUNC9`/`10` parameters existing and accepting the
assignment without error (pwm_out_sim's simulated output count appears
fixed at boot from the airframe's own FUNC1-4 config, not dynamically
extended by a later `param set`). Channels 5/6 map to Actuator 5/6 (the
params PR #28723 actually fixes); a separate test reuses the same physical
channels 5-8 for Actuator 1-4 (see
`test_actuator_compat_actuators1to4_reach_output` — these were never at
risk from the scaling bug, they're plain floats in every message type, but
still deserve genuine Tier 2 confirmation rather than staying
`"supported": null` forever). Reuse is safe since each test reconfigures
`PWM_MAIN_FUNC<n>` fresh — there's no cross-test ordering dependency.

Portability: `PWM_MAIN_FUNC<n>` and the `Peripheral_via_Actuator_SetN`
enum are PX4-specific — a different stack (or a PX4 build lacking this
mixer function) won't accept the param or won't stream
`ACTUATOR_OUTPUT_STATUS` for it. `_ensure_actuator_output_observable()`
checks this once per session and skips every test in this file with a
specific reason if the mechanism itself isn't available, rather than
letting every test fail with a misleading `pwm=None` that looks like a
scaling bug but is actually "can't observe this stack at all."

Empirically confirmed: the output only reflects the commanded value once
ARMED (a disarmed vehicle clamps outputs to a fixed disarmed PWM regardless
of the internal actuator_controls value) — no takeoff/flight is needed,
since DO_SET_ACTUATOR is an instant command with no navigation component.

IMPORTANT — what this can and cannot show
-------------------------------------------
`ACTUATOR_OUTPUT_STATUS` reports PX4's own *commanded* output value — the
number it computed internally and would drive to a physical PWM pin — not
sensed/measured physical position; there is no closed loop here. This is
still the correct, sufficient point to observe PR #28723's fix, because the
bug is a pure software decoding issue (the MAVLink wire int32 -> internal
float scaling) that happens entirely upstream of any physical actuator —
reading the commanded value directly exercises the exact buggy code path
(`decode_scaled_int32_field()`), all the way through PX4's own mixer/output
stage. What this does NOT show: whether a real, physically-attached
actuator on the far end of that PWM signal actually moves correctly — SITL
has no physical model for a generic "Peripheral_via_Actuator_Set" output
(unlike the four simulated motors), and MAVLink has no generic feedback
message for one either (only specific typed actuators, e.g. a gimbal's
`MOUNT_STATUS`, report confirmed physical state). "Supported" in this
file's compat-data records means "the commanded value was verified
correct," not "confirmed to physically move an actuator" — see the
`notes` field on each `record_compat_json`/`record_compat_command_supported`
call.

Running
-------
PX4 SIH multicopter::

    pytest tests/command/do_set_actuator/test_flight.py \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \\
        --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO
"""

import asyncio
import json
import logging

import pytest

from tests.command.conftest import probe_command_int, probe_command_long, INT32_MAX
from tests.flight_helpers import (
    _tier2_auto_record,  # noqa: F401 — autouse: records every test's outcome into the combined report
    record_compat_command_supported,
    record_compat_json,
    require_real_stack,  # noqa: F401 — registers the real-stack skip gate for this module
)

log = logging.getLogger(__name__)

pytestmark = pytest.mark.timeout(120)

_CMD = "DO_SET_ACTUATOR"
_CMD_ID = 187
_CMD_NAME = _CMD  # read by tests/flight_helpers.py's _tier2_auto_record

# Peripheral_via_Actuator_SetN function enum values (src/lib/mixer_module/output_functions.yaml),
# one per actuator slot (1-6). sihsim_quadx assigns PWM_MAIN_FUNC1-4 to the
# four motors, leaving 5+ nominally free — but empirically (checked directly
# against the running SITL instance before writing this, not assumed) only
# channels up to 8 actually come alive in ACTUATOR_OUTPUT_STATUS: channels
# 9/10 never set their `active` bit or update their value, even though the
# PWM_MAIN_FUNC9/10 *parameters* themselves exist (1-16) and accept the
# assignment without error — pwm_out_sim's simulated output count appears
# fixed at boot (from the airframe's own PWM_MAIN_FUNC1-4 config), not
# dynamically extended by a later `param set`. So only channels 5-8 are
# usable here, reused per-test (each test reconfigures PWM_MAIN_FUNC fresh,
# so there's no permanent conflict as long as one test's own channel set
# doesn't overlap with itself): Actuator 5/6 (the params PR #28723 actually
# fixes) use channels 5/6; a separate test for Actuator 1-4 (added for full
# param coverage — these were never at risk from the scaling bug, they're
# plain floats in every message type, but still deserve genuine Tier 2
# confirmation) reuses channels 5-8 for actuators 1-4 respectively.
_FUNC_ACTUATOR_SET_BASE = 300  # Peripheral_via_Actuator_Set<n> == 300+n
_CHANNEL_FOR_ACTUATOR5 = 5  # PWM_MAIN_FUNC5 -> actuator[4]
_CHANNEL_FOR_ACTUATOR6 = 6  # PWM_MAIN_FUNC6 -> actuator[5]
_CHANNEL_FOR_ACTUATOR = {5: 5, 6: 6, 1: 5, 2: 6, 3: 7, 4: 8}  # actuator number -> PWM_MAIN channel

_PWM_MIN = 1000.0
_PWM_MAX = 2000.0
_PWM_TOLERANCE = 5.0  # allow for lroundf rounding

# Cached once per session (these tests all run against the same SITL process,
# and the capability doesn't change mid-session) — see _ensure_actuator_output_observable.
_observability_checked = False
_observability_reason: str | None = None


def _expected_pwm(value: float) -> float:
    """The PWM (us) a -1..1 actuator value maps to, per the mixer's linear scaling."""
    return _PWM_MIN + (value + 1.0) * (_PWM_MAX - _PWM_MIN) / 2.0


async def _configure_actuator_outputs(system, *actuator_numbers: int) -> None:
    """Assign PWM_MAIN_FUNC<channel> to Peripheral_via_Actuator_Set<n> for each requested actuator number."""
    for n in actuator_numbers:
        await system.param.set_param_int(f"PWM_MAIN_FUNC{_CHANNEL_FOR_ACTUATOR[n]}", _FUNC_ACTUATOR_SET_BASE + n)
    await asyncio.sleep(0.5)
    # MAV_CMD_SET_MESSAGE_INTERVAL(511): stream ACTUATOR_OUTPUT_STATUS(375) at 20Hz.
    await probe_command_int(system, command=511, param1=375.0, param2=50000.0)
    await asyncio.sleep(0.3)


async def _ensure_actuator_output_observable(system) -> None:
    """
    Verify this stack supports the PWM-output observability mechanism this
    whole file depends on (a) accepting the PWM_MAIN_FUNC<n> parameter and
    (b) actually streaming ACTUATOR_OUTPUT_STATUS — before that's confirmed,
    a "PWM != expected" result is meaningless: it could mean the scaling is
    wrong, OR it could just mean this stack doesn't support (or doesn't
    implement identically to PX4) either half of the observability mechanism
    itself, e.g. ArduPilot uses entirely different SERVOn_FUNCTION param
    names/enum values, not PWM_MAIN_FUNC<n>/Peripheral_via_Actuator_SetN.
    Checked once per session (module-level cache) since every test in this
    file runs against the same SITL/vehicle connection.

    Skips (not fails) every test in this file with a clear, specific reason
    when the mechanism itself isn't available — this is a real, useful
    distinction: "cannot verify on this stack" is a different finding from
    "verified and the scaling is wrong," and conflating them (a bare PWM
    assertion failure with pwm=None) would misreport the former as the latter.
    """
    global _observability_checked, _observability_reason
    if _observability_checked:
        if _observability_reason is not None:
            pytest.skip(_observability_reason)
        return
    _observability_checked = True
    try:
        await _configure_actuator_outputs(system, 5)
    except Exception as exc:
        _observability_reason = (
            f"Stack does not accept PWM_MAIN_FUNC5={_FUNC_ACTUATOR_SET_BASE + 5} "
            f"(PX4-specific parameter/enum — likely a different stack): {exc}"
        )
        pytest.skip(_observability_reason)
        return
    readings = await _read_actuator_channels(system, _CHANNEL_FOR_ACTUATOR5, settle_s=2.0)
    if _CHANNEL_FOR_ACTUATOR5 not in readings:
        _observability_reason = (
            "No ACTUATOR_OUTPUT_STATUS(375) received after configuring PWM_MAIN_FUNC5 and "
            "requesting the stream via MAV_CMD_SET_MESSAGE_INTERVAL — stack may not stream "
            "this message, or may not honour the interval request"
        )
        pytest.skip(_observability_reason)


async def _read_actuator_channels(system, *channel_1based: int, settle_s: float = 1.5) -> dict[int, float]:
    """
    Sample ACTUATOR_OUTPUT_STATUS for settle_s and return the last value seen
    for each requested 1-based PWM_MAIN channel (as its 0-indexed array slot).
    """
    queue: asyncio.Queue = asyncio.Queue()

    async def _collect() -> None:
        async for msg in system.mavlink_direct.message("ACTUATOR_OUTPUT_STATUS"):
            await queue.put(json.loads(msg.fields_json)["actuator"])

    task = asyncio.create_task(_collect())
    await asyncio.sleep(0.1)

    latest: dict[int, float] = {}
    deadline = asyncio.get_event_loop().time() + settle_s
    while asyncio.get_event_loop().time() < deadline:
        try:
            actuator = await asyncio.wait_for(queue.get(), timeout=0.3)
            for ch in channel_1based:
                latest[ch] = actuator[ch - 1]
        except asyncio.TimeoutError:
            continue

    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    return latest


async def test_actuator_compat_command_int_scales_by_1e7(gcs_system, request):
    """
    COMMAND_INT: x=5,000,000 (Actuator 5 = 0.5, scaled 1e7) produces the
    correct PWM output (1750us) once armed.

    PASS if the observed PWM is within tolerance of 1750us (0.5 scaled
    correctly). CAVEAT, found via a real negative-control run against
    genuinely unpatched PX4 mainline (commit 651b8687b8, before either fix
    commit — see CLAUDE.md's dated section): THIS specific test — frame=6,
    no sentinel edge case — passes even on that unpatched build (frame=6
    apparently already routes through a pre-existing generic "global frame"
    1e7 path by coincidence, not fully root-caused). This test alone does
    NOT discriminate the fix; `test_actuator_compat_sentinel_independent_per_field`
    and `test_actuator_compat_local_frame_scales_same` below are the ones
    that actually do (both genuinely fail pre-fix, confirmed the same run).
    Kept as the primary/first test for its documentation value (the
    simplest possible demonstration of the mechanism), not for its
    discriminating power.
    """
    await _ensure_actuator_output_observable(gcs_system)
    await _configure_actuator_outputs(gcs_system, 5)
    await gcs_system.action.arm()
    await asyncio.sleep(1.0)
    try:
        ack = await probe_command_int(gcs_system, command=_CMD_ID, frame=6, x=5_000_000, y=INT32_MAX, z=0.0)
        assert ack is not None and int(ack["result"]) == 0, f"DO_SET_ACTUATOR not ACCEPTED: {ack}"

        readings = await _read_actuator_channels(gcs_system, _CHANNEL_FOR_ACTUATOR5)
        pwm = readings.get(_CHANNEL_FOR_ACTUATOR5)
        expected = _expected_pwm(0.5)
        ok = pwm is not None and abs(pwm - expected) <= _PWM_TOLERANCE
        log.info("PASS if PWM within %.0f of %.1f: measured=%s", _PWM_TOLERANCE, expected, pwm)
        record_compat_command_supported(
            request, ok, notes="Verified via commanded PWM output, not physical actuator feedback",
        )
        record_compat_json(
            request, "5_Actuator 5", supported=ok,
            notes="Commanded output value verified" if ok else f"measured PWM={pwm}, expected {expected}",
        )
        assert ok, f"Actuator 5 PWM={pwm}, expected {expected}±{_PWM_TOLERANCE} for x=5,000,000 (0.5 scaled by 1e7)"
    finally:
        await gcs_system.action.disarm()


async def test_actuator_compat_sentinel_independent_per_field(gcs_system, request):
    """
    COMMAND_INT: x=INT32_MAX (Actuator 5 ignored) paired with y=5,000,000
    (Actuator 6 = 0.5) leaves Actuator 5's output unchanged from its own
    pre-command baseline while Actuator 6 moves to 1750us — the fix's other
    half: the ignore sentinel must apply per-field, not jointly.

    PASS if Actuator 6 reaches 1750us AND Actuator 5 stays within tolerance
    of its own baseline (read before the command, not a hardcoded constant).
    """
    await _ensure_actuator_output_observable(gcs_system)
    await _configure_actuator_outputs(gcs_system, 5, 6)
    await gcs_system.action.arm()
    await asyncio.sleep(1.0)
    try:
        baseline = await _read_actuator_channels(gcs_system, _CHANNEL_FOR_ACTUATOR5, settle_s=0.5)
        baseline5 = baseline.get(_CHANNEL_FOR_ACTUATOR5)

        ack = await probe_command_int(gcs_system, command=_CMD_ID, frame=6, x=INT32_MAX, y=5_000_000, z=0.0)
        assert ack is not None and int(ack["result"]) == 0, f"DO_SET_ACTUATOR not ACCEPTED: {ack}"

        readings = await _read_actuator_channels(gcs_system, _CHANNEL_FOR_ACTUATOR5, _CHANNEL_FOR_ACTUATOR6)
        pwm5, pwm6 = readings.get(_CHANNEL_FOR_ACTUATOR5), readings.get(_CHANNEL_FOR_ACTUATOR6)
        expected6 = _expected_pwm(0.5)
        ignored_ok = pwm5 is not None and baseline5 is not None and abs(pwm5 - baseline5) <= _PWM_TOLERANCE
        used_ok = pwm6 is not None and abs(pwm6 - expected6) <= _PWM_TOLERANCE
        log.info(
            "PASS if Actuator5 unchanged (baseline=%s, measured=%s) AND Actuator6~=%0.1f (measured=%s)",
            baseline5, pwm5, expected6, pwm6,
        )
        ok = ignored_ok and used_ok
        record_compat_json(request, "5_Actuator 5", nacks_on_non_sentinel_value=None, notes=None,
                            supported=ignored_ok)
        record_compat_json(request, "6_Actuator 6", supported=used_ok)
        assert used_ok, f"Actuator 6 PWM={pwm6}, expected {expected6}±{_PWM_TOLERANCE}"
        assert ignored_ok, f"Actuator 5 PWM changed from baseline {baseline5} to {pwm5} despite x=INT32_MAX (should be ignored independently of y)"
    finally:
        await gcs_system.action.disarm()


async def test_actuator_compat_local_frame_scales_same(gcs_system, request):
    """
    COMMAND_INT under MAV_FRAME_LOCAL_NED (frame=1) — the exact frame whose
    shared location-scaling branch (1e4 divisor) caused the pre-fix bug
    (commit 0f029991b2's own message: "shared location path uses 1e4 for
    local and body frames") — still produces the correct 1e7-scaled PWM.

    PASS if PWM matches the same 1750us target as the default-frame test,
    proving the decode does not depend on frame for this command.
    """
    await _ensure_actuator_output_observable(gcs_system)
    await _configure_actuator_outputs(gcs_system, 5)
    await gcs_system.action.arm()
    await asyncio.sleep(1.0)
    try:
        ack = await probe_command_int(gcs_system, command=_CMD_ID, frame=1, x=5_000_000, y=INT32_MAX, z=0.0)
        assert ack is not None and int(ack["result"]) == 0, f"DO_SET_ACTUATOR not ACCEPTED: {ack}"

        readings = await _read_actuator_channels(gcs_system, _CHANNEL_FOR_ACTUATOR5)
        pwm = readings.get(_CHANNEL_FOR_ACTUATOR5)
        expected = _expected_pwm(0.5)
        ok = pwm is not None and abs(pwm - expected) <= _PWM_TOLERANCE
        log.info("PASS if PWM (frame=1) within %.0f of %.1f: measured=%s", _PWM_TOLERANCE, expected, pwm)
        assert ok, (
            f"Actuator 5 PWM={pwm} under frame=1 (LOCAL_NED), expected {expected}±{_PWM_TOLERANCE} — "
            "if this differs from the default-frame result, decoding is still frame-dependent (the exact pre-fix bug)"
        )
    finally:
        await gcs_system.action.disarm()


async def test_actuator_info_command_long_matches_command_int(gcs_system, request):
    """
    COMMAND_LONG: param5=0.5 (a genuine float, no int32 scaling involved)
    produces the same PWM as COMMAND_INT's scaled x=5,000,000 — regression/
    equivalence coverage between message types. COMMAND_LONG was never
    affected by PR #28723 (no int32 fields to mis-scale), so this is
    informational, not part of the fix verification itself.
    """
    await _ensure_actuator_output_observable(gcs_system)
    await _configure_actuator_outputs(gcs_system, 5)
    await gcs_system.action.arm()
    await asyncio.sleep(1.0)
    try:
        ack = await probe_command_long(gcs_system, command=_CMD_ID, param5=0.5, param6=None, param7=0.0)
        assert ack is not None and int(ack["result"]) == 0, f"DO_SET_ACTUATOR not ACCEPTED: {ack}"

        readings = await _read_actuator_channels(gcs_system, _CHANNEL_FOR_ACTUATOR5)
        pwm = readings.get(_CHANNEL_FOR_ACTUATOR5)
        expected = _expected_pwm(0.5)
        ok = pwm is not None and abs(pwm - expected) <= _PWM_TOLERANCE
        log.info("(Information) PWM via COMMAND_LONG param5=0.5: expected=%.1f measured=%s", expected, pwm)
        assert ok, f"Actuator 5 PWM={pwm} via COMMAND_LONG, expected {expected}±{_PWM_TOLERANCE}"
    finally:
        await gcs_system.action.disarm()


async def test_actuator_compat_actuators1to4_reach_output(gcs_system, request):
    """
    Actuator 1-4 (params 1-4, plain floats in every message type — never at
    risk from PR #28723's scaling bug, but still deserving genuine Tier 2
    confirmation rather than staying "supported": null forever) each reach
    their own correct, independently-verified PWM output, sent together in
    one command with a distinct value per actuator so a channel mixup (e.g.
    Actuator 2's value landing on Actuator 3's output) would be caught, not
    just "did every channel move at all."

    Reuses channels 5-8 (Actuator 5/6's own dedicated tests use these same
    physical channels 5/6 for a different purpose — safe, since each test
    reconfigures PWM_MAIN_FUNC fresh via _configure_actuator_outputs and
    there's no cross-test ordering dependency). Channels 9/10 are
    deliberately NOT used for Actuator 3/4 — confirmed empirically (direct
    probe against the running SITL instance, not assumed) that PWM_MAIN
    channels beyond 8 never come alive in ACTUATOR_OUTPUT_STATUS on this
    airframe, even though the PWM_MAIN_FUNC9/10 parameters themselves exist
    and accept the assignment without error — pwm_out_sim's simulated
    output count appears fixed at boot time from the airframe's own
    PWM_MAIN_FUNC1-4 config, not dynamically extended by a later `param set`.

    PASS if every one of the 4 channels matches its own expected PWM.
    """
    await _ensure_actuator_output_observable(gcs_system)
    await _configure_actuator_outputs(gcs_system, 1, 2, 3, 4)
    await gcs_system.action.arm()
    await asyncio.sleep(1.5)  # extra settle for channels 7/8's first-ever activation this session
    try:
        # Distinct, non-round values per actuator: 1->0.1, 2->0.2, 3->0.3, 4->0.4.
        values = {1: 0.1, 2: 0.2, 3: 0.3, 4: 0.4}
        ack = await probe_command_int(
            gcs_system, command=_CMD_ID, frame=6,
            param1=values[1], param2=values[2], param3=values[3], param4=values[4],
            x=INT32_MAX, y=INT32_MAX, z=0.0,
        )
        assert ack is not None and int(ack["result"]) == 0, f"DO_SET_ACTUATOR not ACCEPTED: {ack}"

        channels = {n: _CHANNEL_FOR_ACTUATOR[n] for n in values}
        # Longer settle window than the other tests in this file (default 1.5s):
        # channels 7/8 are activated here for the first time in the session
        # (unlike 5/6, already warmed up by earlier tests in this file), and
        # empirically that first activation needs more time to stop reporting
        # a transient stale value than an already-active channel does — see
        # this test's own docstring and this file's CLAUDE.md for the full
        # investigation (confirmed via an isolated single-test run: the same
        # assertion passes immediately when this is the *only* test in the
        # session, so this is a one-time warm-up cost, not a real bug).
        readings = await _read_actuator_channels(gcs_system, *channels.values(), settle_s=3.5)
        results: dict[int, tuple[float | None, float, bool]] = {}
        for n, value in values.items():
            pwm = readings.get(channels[n])
            expected = _expected_pwm(value)
            ok = pwm is not None and abs(pwm - expected) <= _PWM_TOLERANCE
            results[n] = (pwm, expected, ok)
            log.info("PASS if Actuator %d (channel %d) ~= %.1f: measured=%s", n, channels[n], expected, pwm)
            record_compat_json(
                request, f"{n}_Actuator {n}", supported=ok,
                notes="Commanded output value verified" if ok else f"measured PWM={pwm}, expected {expected}",
            )

        failures = [n for n, (pwm, expected, ok) in results.items() if not ok]
        assert not failures, (
            "Actuator(s) " + ", ".join(str(n) for n in failures) + " did not reach expected PWM: "
            + "; ".join(f"Actuator {n}: measured={results[n][0]}, expected={results[n][1]}" for n in failures)
        )
    finally:
        await gcs_system.action.disarm()
