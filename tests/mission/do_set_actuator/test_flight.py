"""
MAV_CMD_DO_SET_ACTUATOR (cmd=187) as a mission item — Tier 2 execution test.

Confirms PR #28723's MISSION_ITEM_INT actuator-scaling fix (commit
ea5734b460) through the FULL pipeline: upload -> mission execution ->
vehicle_command dispatch -> actuator output, not just protocol-level
storage (that's test_protocol.py's job). See tests/command/do_set_actuator/
test_flight.py's module docstring for the PWM-observability mechanism this
reuses (PWM_MAIN_FUNC5 -> Peripheral_via_Actuator_Set5, ACTUATOR_OUTPUT_STATUS).

DO_SET_ACTUATOR is an "instant action" mission item (src/modules/navigator/
mission_block.cpp lists it alongside DO_LAND_START, DO_TRIGGER_CONTROL,
etc. as completing without a timeout) — no takeoff or navigation is needed;
a single-item mission is sufficient once armed.

IMPORTANT — what this can and cannot show: see tests/command/do_set_actuator/
test_flight.py's module docstring, same caveat verbatim — `ACTUATOR_OUTPUT_STATUS`
is PX4's own commanded output value, not sensed physical position; there is
no closed loop and no physical actuator model for this generic output
function in SITL. Sufficient for this PR (a pure decoding bug upstream of
any physical actuator), not evidence of confirmed physical movement.

Running
-------
PX4 SIH multicopter::

    pytest tests/mission/do_set_actuator/test_flight.py \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \\
        --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO
"""

import asyncio
import json
import logging

import pytest
from mavsdk.mission_raw import MissionItem

from tests import report
from tests.command.conftest import probe_command_int
from tests.flight_helpers import (
    _tier2_auto_record,  # noqa: F401 — autouse: records every test's outcome into the combined report
    record_compat_command_supported,
    record_compat_json,
    require_real_stack,  # noqa: F401 — registers the real-stack skip gate for this module
)
from tests.mission.conftest import clear_all_mission_types, home_item_for_mission  # noqa: F401
from .test_protocol import SPEC as _DO_SET_ACTUATOR_SPEC

log = logging.getLogger(__name__)

pytestmark = pytest.mark.timeout(120)

_CMD_NAME = "DO_SET_ACTUATOR"  # read by tests/report.py's key_from_module (_tier1_auto_record/_tier2_auto_record)
_CMD_ID = 187

# Declare identity + full param-slot list at import time (deriving the list from
# the sibling test_protocol.py's own SPEC — single source of truth), so every
# param always appears in the rendered JSON even if this file runs without
# test_protocol.py in the same session. See nav_takeoff/test_flight.py for the
# identical pattern this mirrors.
report.declare_command("mission", _CMD_NAME, _CMD_ID)
report.declare_params(
    "mission", _CMD_NAME, [f"{p.slot}_{p.label}" for p in _DO_SET_ACTUATOR_SPEC.params],
)

_FUNC_ACTUATOR_SET5 = 305  # Peripheral_via_Actuator_Set5
_CHANNEL_FOR_ACTUATOR5 = 5  # PWM_MAIN_FUNC5 -> actuator[4]

_PWM_MIN = 1000.0
_PWM_MAX = 2000.0
_PWM_TOLERANCE = 5.0


def _expected_pwm(value: float) -> float:
    return _PWM_MIN + (value + 1.0) * (_PWM_MAX - _PWM_MIN) / 2.0


async def _configure_actuator_output(system) -> None:
    await system.param.set_param_int("PWM_MAIN_FUNC5", _FUNC_ACTUATOR_SET5)
    await asyncio.sleep(0.5)
    await probe_command_int(system, command=511, param1=375.0, param2=50000.0)  # ACTUATOR_OUTPUT_STATUS @ 20Hz
    await asyncio.sleep(0.3)


# Portability check, mirroring tests/command/do_set_actuator/test_flight.py's
# _ensure_actuator_output_observable — PWM_MAIN_FUNC<n>/Peripheral_via_Actuator_SetN
# are PX4-specific; skip with a clear reason rather than a misleading "PWM
# wrong" result if this stack doesn't support the observability mechanism.
_observability_checked = False
_observability_reason: str | None = None


async def _ensure_actuator_output_observable(system) -> None:
    global _observability_checked, _observability_reason
    if _observability_checked:
        if _observability_reason is not None:
            pytest.skip(_observability_reason)
        return
    _observability_checked = True
    try:
        await _configure_actuator_output(system)
    except Exception as exc:
        _observability_reason = (
            f"Stack does not accept PWM_MAIN_FUNC5={_FUNC_ACTUATOR_SET5} "
            f"(PX4-specific parameter/enum — likely a different stack): {exc}"
        )
        pytest.skip(_observability_reason)
        return
    readings = await _read_actuator5_pwm(system, settle_s=2.0)
    if readings is None:
        _observability_reason = (
            "No ACTUATOR_OUTPUT_STATUS(375) received after configuring PWM_MAIN_FUNC5 and "
            "requesting the stream — stack may not stream this message, or may not honour "
            "the interval request"
        )
        pytest.skip(_observability_reason)


async def _read_actuator5_pwm(system, settle_s: float = 2.0) -> float | None:
    queue: asyncio.Queue = asyncio.Queue()

    async def _collect() -> None:
        async for msg in system.mavlink_direct.message("ACTUATOR_OUTPUT_STATUS"):
            await queue.put(json.loads(msg.fields_json)["actuator"][_CHANNEL_FOR_ACTUATOR5 - 1])

    task = asyncio.create_task(_collect())
    await asyncio.sleep(0.1)

    latest = None
    deadline = asyncio.get_event_loop().time() + settle_s
    while asyncio.get_event_loop().time() < deadline:
        try:
            latest = await asyncio.wait_for(queue.get(), timeout=0.3)
        except asyncio.TimeoutError:
            continue

    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    return latest


def _build_actuator_mission(home_item) -> list[MissionItem]:
    """Home-slot prepend (ArduCopter) + a single DO_SET_ACTUATOR item: x=5,000,000 (Actuator 5 = 0.5)."""
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
    seq = len(items)
    items.append(MissionItem(
        seq=seq, frame=2, command=187, current=(0 if home_item is not None else 1),  # MAV_FRAME_MISSION
        autocontinue=1,
        param1=float("nan"), param2=float("nan"), param3=float("nan"), param4=float("nan"),
        x=5_000_000, y=0x7FFF_FFFF, z=0.0,
        mission_type=0,
    ))
    return items


async def test_actuator_compat_mission_item_scales_by_1e7(gcs_system, home_item_for_mission, request):
    """
    A mission with a single DO_SET_ACTUATOR item (Actuator 5 = 0.5 via
    x=5,000,000) reaches the vehicle's actuator output correctly once
    executed — the full upload -> mission-execution -> dispatch pipeline,
    not just protocol-level storage.

    PASS if the observed PWM is within tolerance of 1750us.
    """
    await _ensure_actuator_output_observable(gcs_system)
    items = _build_actuator_mission(home_item_for_mission)
    try:
        await gcs_system.mission_raw.upload_mission(items)
        await gcs_system.action.arm()
        await asyncio.sleep(1.0)
        await gcs_system.mission_raw.start_mission()

        pwm = await _read_actuator5_pwm(gcs_system, settle_s=3.0)
        expected = _expected_pwm(0.5)
        ok = pwm is not None and abs(pwm - expected) <= _PWM_TOLERANCE
        log.info("PASS if PWM within %.0f of %.1f after mission execution: measured=%s", _PWM_TOLERANCE, expected, pwm)
        # This is the definitive Tier 2 "is it honoured" check for the mission-item
        # path — a real flown mission with a quantitative PWM confirmation, not just
        # an upload/download round-trip — so it's what promotes the command-level
        # fact from Tier 1's "null" (not independently tested past acceptance) to a
        # genuine True/False. Without this call the JSON stays "supported": null
        # forever, even on a passing run, since nothing else in this file records it.
        record_compat_command_supported(
            request, ok,
            notes="Verified via commanded PWM output, not physical actuator feedback" if ok
            else f"measured PWM={pwm}, expected {expected}",
        )
        record_compat_json(
            request, "5_Actuator 5", supported=ok,
            notes="Commanded output value verified" if ok else None,
        )
        assert ok, f"Actuator 5 PWM={pwm} after mission-item DO_SET_ACTUATOR execution, expected {expected}±{_PWM_TOLERANCE}"
    finally:
        try:
            await gcs_system.action.disarm()
        except Exception as exc:
            log.warning("disarm failed (may already be disarmed): %s", exc)
        await clear_all_mission_types(gcs_system)
