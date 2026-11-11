"""
MESSAGE_INTERVAL (message id 244) — https://mavlink.io/en/messages/common.html#MESSAGE_INTERVAL

What the XML defines (README.md in this directory has the reasoning and results):

- "The interval between messages for a particular MAVLink message ID. This
  message is sent in response to the MAV_CMD_REQUEST_MESSAGE command with
  param1=244 (this message) and param2=message_id".
- `message_id`: the ID of the requested message.
- `interval_us`: "A value of -1 indicates this stream is disabled, 0 indicates
  it is not available, > 0 indicates the interval at which it is sent."

It's a reply, not a stream, so the generic emission tests don't apply. What
is tested:
- After SET_MESSAGE_INTERVAL at two rates (rule 4c), -1 and 0, the reply's
  message_id is the one asked about and interval_us reports what was set —
  compat (a NACKed SET or REQUEST is a legitimate PASS, rule 4a).
- For a set of messages, interval_us agrees with what actually arrives: > 0
  while the message streams (±30% of the measured rate), and not > 0 when
  nothing arrives — compat ("> 0 indicates the interval at which it is sent").
- A message id no dialect defines: NACKed, or 0 ("not available") — compat.
- Whether a reply is also sent unsolicited (after a SET) — characterised.
"""

from __future__ import annotations

import asyncio
import logging

import pytest

from tests import report
from tests.messages_telemetry.conftest import (
    MAV_RESULT_ACCEPTED,
    MSG_IDS,
    MessageSampler,
    compat_verdict,
    get_message_interval,
    harness_requested_hz,
    rate_hz,
    record_detail,
    record_field_fact,
    record_message_fact,
    record_verdict,
    result_name,
    set_message_interval,
    vehicle_target,
)
from tests.messages_telemetry.emission import autopilot_only

log = logging.getLogger(__name__)

_MSG_NAME = "MESSAGE_INTERVAL"
_MSG_ID = 244
REQUIRED_MESSAGES = {}
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME, ["message_id", "interval_us"])

pytestmark = pytest.mark.timeout(180)

TARGET = "SYSTEM_TIME"          # the message whose interval is set and read back
RATES_HZ = (2.0, 5.0)
INTERVAL_TOLERANCE = 0.02       # reported vs set (ArduPilot rounds to ms)
UNKNOWN_MSG_ID = 65000          # in no dialect (2026-11-12)
# Messages compared against what actually arrives. Includes some a SITL may
# not be able to send (second/third sensors, second GPS).
CANDIDATES = ["SYSTEM_TIME", "ATTITUDE", "ATTITUDE_QUATERNION", "GLOBAL_POSITION_INT", "LOCAL_POSITION_NED",
              "GPS_RAW_INT", "GPS2_RAW", "SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3", "SCALED_PRESSURE",
              "SCALED_PRESSURE2", "SCALED_PRESSURE3", "EXTENDED_SYS_STATE", "HOME_POSITION"]
LISTEN_S = 10.0
MATCH_TOLERANCE = 0.3
SLOWEST_US = 5_000_000          # a stream slower than this may not show in LISTEN_S

_replied = False


def _note_reply(interval_us) -> None:
    global _replied
    if interval_us is not None:
        _replied = True


async def test_message_interval_compat_reports_set_rate(gcs_system, request):
    """After SET_MESSAGE_INTERVAL at 2 Hz and 5 Hz, interval_us reports each (±2%) for the right message_id."""
    msg_id = MSG_IDS[TARGET]
    rows = []  # (hz, set_result, get_result, interval)
    try:
        for hz in RATES_HZ:
            set_result = await set_message_interval(gcs_system, msg_id, 1e6 / hz)
            get_result, interval = await get_message_interval(gcs_system, msg_id)
            _note_reply(interval)
            rows.append((hz, set_result, get_result, interval))
    finally:
        await set_message_interval(gcs_system, msg_id, 0)
    nacked = all(r[1] != MAV_RESULT_ACCEPTED or r[2] != MAV_RESULT_ACCEPTED for r in rows)
    honoured = [r for r in rows if r[1] == MAV_RESULT_ACCEPTED and r[2] == MAV_RESULT_ACCEPTED]
    ok = all(r[3] is not None and abs(r[3] - 1e6 / r[0]) <= INTERVAL_TOLERANCE * 1e6 / r[0] for r in honoured)
    verdict = compat_verdict(nacked=nacked, ok=ok)
    record_verdict(request, "interval_us after SET (rate)", verdict)
    detail = "; ".join(f"{hz:.0f} Hz: SET {result_name(s)}, GET {result_name(g)} → "
                       f"{'no reply' if i is None else f'{i} us'}" for hz, s, g, i in rows)
    record_detail(request, f"{verdict} — {TARGET}: {detail}")
    if honoured:
        record_field_fact(request, "interval_us", supported=ok)
        record_field_fact(request, "message_id", supported=all(r[3] is not None for r in honoured))
    if _replied:
        record_message_fact(request, supported=True)
    if not nacked and not ok:
        report.compat_fail(f"interval_us doesn't report the interval set — {detail}")


async def test_message_interval_compat_reports_disabled(gcs_system, request):
    """After SET_MESSAGE_INTERVAL -1, interval_us is -1 (disabled)."""
    msg_id = MSG_IDS[TARGET]
    try:
        set_result = await set_message_interval(gcs_system, msg_id, -1)
        get_result, interval = await get_message_interval(gcs_system, msg_id)
    finally:
        await set_message_interval(gcs_system, msg_id, 0)
    _note_reply(interval)
    nacked = set_result != MAV_RESULT_ACCEPTED or get_result != MAV_RESULT_ACCEPTED
    ok = interval == -1
    verdict = compat_verdict(nacked=nacked, ok=ok)
    record_verdict(request, "interval_us after SET -1", verdict)
    record_detail(request, f"{verdict} — SET -1 {result_name(set_result)}, GET {result_name(get_result)} → "
                           f"{interval}")
    if not nacked and not ok:
        report.compat_fail(f"interval_us {interval} after SET_MESSAGE_INTERVAL -1 (expected -1, disabled)")


async def test_message_interval_obs_reports_default(gcs_system, request):
    """Characterise interval_us after SET_MESSAGE_INTERVAL 0 (default), against the measured rate."""
    msg_id = MSG_IDS[TARGET]
    target = await vehicle_target(gcs_system)
    set_result = await set_message_interval(gcs_system, msg_id, 0)
    async with MessageSampler(gcs_system, TARGET) as s:
        await asyncio.sleep(LISTEN_S)
    get_result, interval = await get_message_interval(gcs_system, msg_id)
    _note_reply(interval)
    measured = rate_hz(autopilot_only(s.samples(TARGET), target), LISTEN_S)
    record_detail(request, f"SET 0 {result_name(set_result)}; {TARGET} {measured:.2f} Hz; GET "
                           f"{result_name(get_result)} → {interval}")


async def test_message_interval_compat_matches_what_arrives(gcs_system, request):
    """interval_us is > 0 (±30%) for each message that streams, and not > 0 for one that never arrives."""
    target = await vehicle_target(gcs_system)
    async with MessageSampler(gcs_system, *CANDIDATES) as s:
        await asyncio.sleep(LISTEN_S)
    rows, wrong = [], []
    for name in CANDIDATES:
        measured = rate_hz(autopilot_only(s.samples(name), target), LISTEN_S)
        result, interval = await get_message_interval(gcs_system, MSG_IDS[name])
        _note_reply(interval)
        if result != MAV_RESULT_ACCEPTED or interval is None:
            rows.append(f"{name} {measured:.1f} Hz: GET {result_name(result)}")
            if result == MAV_RESULT_ACCEPTED:
                wrong.append(f"{name}: GET ACCEPTED but no reply")
            continue
        harness = " (harness-requested)" if await harness_requested_hz(gcs_system, MSG_IDS[name]) else ""
        rows.append(f"{name} {measured:.1f} Hz{harness}: {interval}")
        if measured > 0 and interval <= 0:
            wrong.append(f"{name} arrives at {measured:.1f} Hz but interval_us {interval}")
        elif measured > 0 and abs(1e6 / interval - measured) > MATCH_TOLERANCE * measured:
            wrong.append(f"{name} arrives at {measured:.1f} Hz but interval_us {interval} ({1e6 / interval:.1f} Hz)")
        elif measured == 0 and 0 < interval <= SLOWEST_US:
            wrong.append(f"{name} never arrives but interval_us {interval}")
    detail = "; ".join(rows)
    record_detail(request, (f"{len(wrong)} disagree: {' | '.join(wrong)} — " if wrong else "all agree — ") + detail)
    record_verdict(request, "interval_us vs what arrives", "AGREES" if not wrong else
                   f"DISAGREES for {len(wrong)} of {len(CANDIDATES)}")
    if _replied:
        record_message_fact(request, supported=True)
    if wrong:
        report.compat_fail(f"interval_us doesn't describe what's sent: {' | '.join(wrong)}")


async def test_message_interval_compat_unknown_message(gcs_system, request):
    """For a message id no dialect defines: NACKed, or interval_us 0 ("not available")."""
    result, interval = await get_message_interval(gcs_system, UNKNOWN_MSG_ID)
    _note_reply(interval)
    nacked = result != MAV_RESULT_ACCEPTED
    ok = interval == 0
    verdict = compat_verdict(nacked=nacked, ok=ok)
    record_verdict(request, "unknown message id", verdict)
    record_detail(request, f"{verdict} — GET({UNKNOWN_MSG_ID}) {result_name(result)} → "
                           f"{'no reply' if interval is None else interval}")
    if not nacked and not ok:
        report.compat_fail(f"message {UNKNOWN_MSG_ID} (in no dialect): GET ACCEPTED, interval_us "
                           f"{'none' if interval is None else interval} (expected 0, not available)")


async def test_message_interval_obs_unsolicited(gcs_system, request):
    """Characterise: is MESSAGE_INTERVAL sent without a request (e.g. after a SET_MESSAGE_INTERVAL)?"""
    target = await vehicle_target(gcs_system)
    msg_id = MSG_IDS[TARGET]
    async with MessageSampler(gcs_system, _MSG_NAME) as s:
        await asyncio.sleep(3.0)
        quiet = len(autopilot_only(s.samples(_MSG_NAME), target))
        await set_message_interval(gcs_system, msg_id, 1e6 / RATES_HZ[0])
        await asyncio.sleep(2.0)
        after_set = len(autopilot_only(s.samples(_MSG_NAME), target)) - quiet
    await set_message_interval(gcs_system, msg_id, 0)
    record_detail(request, f"{quiet} unsolicited in 3s; {after_set} after SET_MESSAGE_INTERVAL")
    if _replied:
        record_message_fact(request, supported=True)
