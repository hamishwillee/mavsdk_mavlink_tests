"""
FLIGHT_INFORMATION (message id 264) — https://mavlink.io/en/messages/common.html#FLIGHT_INFORMATION

What the XML defines (README.md in this directory has the reasoning and results):

- "time since boot for arm, takeoff, and land, and a flight number. Takeoff and
  landing values reset to zero on arm." Requested with MAV_CMD_REQUEST_MESSAGE.
- arming_time_utc (µs since boot, invalid 0): "Set to 0 on boot. Set value on arming."
- takeoff_time_utc (µs since boot, invalid 0): "Set to 0 at boot and on arming."
- flight_uuid (invalid 0): "Flight number."
- landing_time (extension, ms since boot, invalid 0): "Set to 0 at boot and on arming."
- time_boot_ms.

So:
- Disarmed on the ground, every time is 0 or in the past (≤ time_boot_ms) —
  compat; flight_uuid is characterised.
- Arming (no takeoff) sets arming_time_utc to the arming time (±2 s) — compat,
  and the only "is it honoured" check here. takeoff/landing times need a
  flight, so they're recorded untestable. The test disarms straight away and
  characterises what disarming does (the XML doesn't say), including to
  flight_uuid.
"""

from __future__ import annotations

import asyncio
import logging
import time

import pytest

from tests import report
from tests.flight_helpers import _wait_armable
from tests.messages_telemetry.conftest import (
    MAV_RESULT_ACCEPTED,
    MessageSampler,
    record_detail,
    record_field_fact,
    request_message,
    result_name,
    skip_untestable,
    vehicle_target,
)
from tests.messages_telemetry.emission import Emission, autopilot_only

log = logging.getLogger(__name__)

_MSG_NAME = "FLIGHT_INFORMATION"
_MSG_ID = 264
REQUIRED_MESSAGES = {"FLIGHT_INFORMATION": "the message under test"}
_TIMES = ["arming_time_utc", "takeoff_time_utc", "landing_time"]
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME, ["time_boot_ms", *_TIMES[:2], "flight_uuid", "landing_time"])

pytestmark = pytest.mark.timeout(180)

EMISSION = Emission(_MSG_NAME, _MSG_ID)
EMISSION.define_tests(globals(), "flight_information")

ARM_TIME_TOLERANCE_MS = 2000
ARMABLE_TIMEOUT_S = 90.0


def _ms(fields: dict, name: str) -> float:
    """A time field in ms since boot (landing_time is already ms; the others are µs)."""
    return fields[name] if name == "landing_time" else fields[name] / 1000.0


async def _request_one(system) -> tuple[int | None, dict | None]:
    target = await vehicle_target(system)
    async with MessageSampler(system, _MSG_NAME) as s:
        result = await request_message(system, _MSG_ID)
        for _ in range(30):
            mine = autopilot_only(s.samples(_MSG_NAME), target)
            if mine:
                return result, mine[-1].fields
            await asyncio.sleep(0.1)
    return result, None


async def test_flight_information_compat_boot_ms_tracks_real_time(gcs_system, request):
    """time_boot_ms never goes backwards or sticks, and advances at host-clock rate (±5%)."""
    await EMISSION.boot_ms_tracks_real_time(gcs_system, request)


async def test_flight_information_compat_times_in_the_past(gcs_system, request):
    """Disarmed: every time field is 0 (unset) or not after time_boot_ms."""
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        skip_untestable(request, _TIMES, "not enough messages to analyse", w.na)
    f = w.samples[-1].fields
    future = [n for n in _TIMES if _ms(f, n) > f["time_boot_ms"] + 1000]
    shown = ", ".join(f"{n} {_ms(f, n) / 1000:.1f}s" if f[n] else f"{n} 0" for n in _TIMES)
    record_detail(request, f"time_boot_ms {f['time_boot_ms'] / 1000:.1f}s: {shown}")
    if future:
        for n in future:
            record_field_fact(request, n, supported=False)
        report.compat_fail(f"{', '.join(future)} later than time_boot_ms — not a time since boot ({shown})")


async def test_flight_information_obs_flight_uuid(gcs_system, request):
    """Characterise flight_uuid while disarmed (0 is the invalid value)."""
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        skip_untestable(request, ["flight_uuid"], "not enough messages to analyse", w.na)
    values = sorted({int(s.fields["flight_uuid"]) for s in w.samples})
    record_detail(request, f"flight_uuid {values}")
    # The verdict comes from the arming test, which sees whether it changes.


async def test_flight_information_compat_arming_time_set_on_arm(gcs_system, request):
    """Arming (no takeoff) sets arming_time_utc to the arming time (±2 s); then disarm and characterise."""
    if report.MOCK_MODE:
        skip_untestable(request, ["arming_time_utc", "takeoff_time_utc", "landing_time", "flight_uuid"],
                        "mock flight stack: can't arm")
    result, before = await _request_one(gcs_system)
    if before is None:
        skip_untestable(request, ["arming_time_utc", "takeoff_time_utc", "landing_time", "flight_uuid"],
                        "message not sent on request", f"REQUEST_MESSAGE {result_name(result)}, no reply")
    try:
        await _wait_armable(gcs_system, ARMABLE_TIMEOUT_S)
    except TimeoutError as exc:
        skip_untestable(request, ["arming_time_utc"], "couldn't arm", str(exc))
    _, pre = await _request_one(gcs_system)
    t_pre = pre["time_boot_ms"] if pre else before["time_boot_ms"]
    try:
        await gcs_system.action.arm()
    except Exception as exc:  # noqa: BLE001 — any arming refusal makes this NA
        skip_untestable(request, ["arming_time_utc"], "arming refused", repr(exc))
    try:
        await asyncio.sleep(1.0)
        _, armed = await _request_one(gcs_system)
    finally:
        try:
            await gcs_system.action.disarm()
        except Exception as exc:  # noqa: BLE001
            log.warning("disarm failed: %r", exc)
    await asyncio.sleep(1.5)
    _, after = await _request_one(gcs_system)
    if armed is None:
        pytest.fail("no FLIGHT_INFORMATION while armed")
    arm_ms = armed["arming_time_utc"] / 1000.0
    ok = t_pre - ARM_TIME_TOLERANCE_MS <= arm_ms <= armed["time_boot_ms"]
    record_field_fact(request, "arming_time_utc", supported=ok)
    for n in ("takeoff_time_utc", "landing_time"):
        record_field_fact(request, n, untestable="needs a flight")
    uuids = [before["flight_uuid"], armed["flight_uuid"], after["flight_uuid"] if after else None]
    if any(uuids):
        changed = len({u for u in uuids if u is not None}) > 1
        record_field_fact(request, "flight_uuid", supported=True,
                          notes="Changed across arm/disarm" if changed else None)
    else:
        record_field_fact(request, "flight_uuid", supported=False, notes="Always 0")
    after_s = ("no reply after disarm" if after is None else
               f"after disarm arming {after['arming_time_utc'] / 1e6:.1f}s, takeoff "
               f"{after['takeoff_time_utc'] / 1e6:.1f}s")
    detail = (f"armed between boot {t_pre / 1000:.1f}s and {armed['time_boot_ms'] / 1000:.1f}s: arming_time "
              f"{arm_ms / 1000:.1f}s, takeoff {armed['takeoff_time_utc'] / 1e6:.1f}s while armed; {after_s}; "
              f"flight_uuid before/armed/after {uuids}")
    record_detail(request, detail)
    if not ok:
        report.compat_fail(f"arming_time_utc isn't the arming time — {detail}")
