"""
SYSTEM_TIME (message id 2) — https://mavlink.io/en/messages/common.html#SYSTEM_TIME

What the XML defines (and doesn't), and so what each test can assert —
README.md in this directory has the full reasoning:

- "The system time is the time of the sender's master clock." Two fields:
  `time_unix_usec` (us, "UNIX epoch time") and `time_boot_ms` (ms, "time since
  system boot"). Neither has an invalid value, min, max or increment.
- Nothing says whether it's streamed by default, at what rate, or what to send
  before the clock is known — those are characterised (`obs`), never failed.
- The generic SET_MESSAGE_INTERVAL / REQUEST_MESSAGE apply (tests/messages_
  telemetry/emission.py): an accepted request that isn't honoured is a
  compatibility error (root CLAUDE.md rule 4a); a NACK is a legitimate PASS.
- A non-zero `time_unix_usec` that isn't a plausible epoch time, or a
  `time_boot_ms` that doesn't advance like a clock, contradicts the XML (FAIL,
  compatibility error). `time_unix_usec` = 0 is recorded as "unknown" — there's
  no defined invalid value, so it can't be a FAIL either way.

Nothing flies, so every vehicle type runs the same file with no branching.
Tests run in file order: the default rate is measured before anything changes
it, and the two reboot tests are last.
"""

from __future__ import annotations

import asyncio
import logging
import os
import statistics
import time
from dataclasses import dataclass, field

import pytest

from tests import report
from tests.message_watcher import ensure_harness_streams
from tests.messages_telemetry.conftest import (
    MessageSampler,
    Sample,
    record_detail,
    record_field_fact,
    result_name,
    set_message_interval,
    vehicle_target,
)
from tests.messages_telemetry.emission import Emission, autopilot_only

log = logging.getLogger(__name__)

_MSG_NAME = "SYSTEM_TIME"
_MSG_ID = 2
REQUIRED_MESSAGES = {"SYSTEM_TIME": "the message under test"}
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME, ["time_unix_usec", "time_boot_ms"])

pytestmark = pytest.mark.timeout(180)

OTHER_TIME_MESSAGES = ("ATTITUDE", "GLOBAL_POSITION_INT")
EMISSION = Emission(_MSG_NAME, _MSG_ID, window_extra=OTHER_TIME_MESSAGES)

UNIX_FLOOR_US = 1_577_836_800 * 1_000_000   # 2020-01-01: anything earlier isn't today's date
UNIX_TOLERANCE_S = float(os.environ.get("SYSTEM_TIME_UNIX_TOLERANCE_S", "5.0"))  # vs host clock (SITL shares it)
OFFSET_SPREAD_MS = 50.0         # unix/1000 - boot_ms may wander this much over the window
OTHER_BOOT_TOLERANCE_MS = 500.0 # another message's time_boot_ms vs SYSTEM_TIME's clock
REBOOT_WATCH_S = float(os.environ.get("SYSTEM_TIME_REBOOT_WATCH_S", "60.0"))
REBOOT_MARGIN_MS = 2000.0


# ---------------------------------------------------------------------------
# A. Emission (generic: tests/messages_telemetry/emission.py)
# ---------------------------------------------------------------------------

async def test_system_time_obs_default_rate(gcs_system, request):
    """Characterise: unrequested rate and senders, and whether MESSAGE_INTERVAL agrees."""
    await EMISSION.default_rate(gcs_system, request)


async def test_system_time_compat_set_interval_honoured(gcs_system, request):
    """SET_MESSAGE_INTERVAL at 2 Hz and 10 Hz: NACKed, or the measured rate matches each."""
    await EMISSION.set_interval_honoured(gcs_system, request)


async def test_system_time_compat_set_interval_disable(gcs_system, request):
    """SET_MESSAGE_INTERVAL -1 (disable): NACKed, or no SYSTEM_TIME for 3 s."""
    await EMISSION.set_interval_disable(gcs_system, request)


async def test_system_time_compat_set_interval_default(gcs_system, request):
    """SET_MESSAGE_INTERVAL 0 (default) after a change: NACKed, or the rate returns to the unrequested one."""
    await EMISSION.set_interval_default(gcs_system, request)


async def test_system_time_compat_request_message(gcs_system, request):
    """REQUEST_MESSAGE(SYSTEM_TIME) with the stream off: NACKed, or exactly one message."""
    await EMISSION.request_message(gcs_system, request)


# ---------------------------------------------------------------------------
# B. time_boot_ms
# ---------------------------------------------------------------------------

async def test_system_time_compat_boot_ms_tracks_real_time(gcs_system, request):
    """time_boot_ms never goes backwards or sticks, and advances at host-clock rate (±5%)."""
    await EMISSION.boot_ms_tracks_real_time(gcs_system, request)


async def test_system_time_info_boot_ms_matches_other_messages(gcs_system, request):
    """(Information) ATTITUDE/GLOBAL_POSITION_INT time_boot_ms agree with SYSTEM_TIME's clock (±500 ms)."""
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        pytest.skip(f"NA: {w.na}")
    st = w.samples
    parts, worst = [], 0.0
    for name, others in w.others.items():
        if not others:
            parts.append(f"{name}: none received")
            continue
        resid = []
        for o in others:
            near = min(st, key=lambda s: abs(s.host_mono - o.host_mono))
            expected = near.fields["time_boot_ms"] + (o.host_mono - near.host_mono) * 1000
            resid.append(o.fields["time_boot_ms"] - expected)
        m = max(resid, key=abs)
        worst = max(worst, abs(m))
        parts.append(f"{name}: median {statistics.median(resid):+.0f} ms, worst {m:+.0f} ms (n={len(resid)})")
    if all(not v for v in w.others.values()):
        pytest.skip(f"NA: none of {', '.join(OTHER_TIME_MESSAGES)} received to compare against")
    detail = "; ".join(parts)
    record_detail(request, f"(Information) {detail}")
    assert worst <= OTHER_BOOT_TOLERANCE_MS, f"time_boot_ms disagrees between messages — {detail}"


# ---------------------------------------------------------------------------
# C. time_unix_usec
# ---------------------------------------------------------------------------

async def test_system_time_compat_unix_is_epoch(gcs_system, request):
    """Every non-zero time_unix_usec is a plausible UNIX time: after 2020 and within 5 s of the host clock."""
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        record_field_fact(request, "time_unix_usec", untestable="not enough messages to analyse")
        pytest.skip(f"NA: {w.na}")
    values = [(s, s.fields["time_unix_usec"]) for s in w.samples]
    zeros = sum(1 for _, v in values if v == 0)
    nonzero = [(s, v) for s, v in values if v != 0]
    if not nonzero:
        record_field_fact(request, "time_unix_usec", untestable="only 0 sent (clock not set)", notes="0 when time unknown")
        pytest.skip(f"NA: every time_unix_usec was 0 ({len(values)} msgs) — clock not set, nothing to check")
    offsets = [v / 1e6 - s.host_wall for s, v in nonzero]  # vehicle minus host, seconds
    bad = [(s, v) for (s, v), off in zip(nonzero, offsets) if v < UNIX_FLOOR_US or abs(off) > UNIX_TOLERANCE_S]
    ok = not bad
    notes = "0 when time unknown" if zeros else None
    record_field_fact(request, "time_unix_usec", supported=ok, notes=notes)
    detail = (f"{len(nonzero)} non-zero, {zeros} zero; vehicle−host median {statistics.median(offsets) * 1000:+.0f} ms, "
              f"worst {max(offsets, key=abs) * 1000:+.0f} ms (tolerance ±{UNIX_TOLERANCE_S:.0f}s)")
    record_detail(request, detail)
    if bad:
        s, v = bad[0]
        report.compat_fail(f"time_unix_usec {v} isn't UNIX epoch time (host {s.host_wall:.3f}s) — {detail}")


async def test_system_time_obs_unix_vs_boot_ms_drift(gcs_system, request):
    """
    Characterise: does time_unix_usec/1000 − time_boot_ms stay constant, i.e.
    are both fields one clock? The XML doesn't require it, so this never fails.
    A consumer that converts a time_boot_ms timestamp from another message to
    UNIX time using one SYSTEM_TIME's offset depends on it, though. (PX4 SITL:
    time_boot_ms is lockstep simulation time and time_unix_usec the host's
    CLOCK_REALTIME, so they drift apart at the simulation's real-time error,
    ~20 ms/s on SIH — a SITL property. ArduPilot: one clock, constant offset.)
    """
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        pytest.skip(f"NA: {w.na}")
    pairs = [(s.fields["time_unix_usec"], s.fields["time_boot_ms"]) for s in w.samples if s.fields["time_unix_usec"]]
    if len(pairs) < 3:
        pytest.skip("NA: fewer than 3 non-zero time_unix_usec values")
    offs = [u / 1000 - b for u, b in pairs]
    spread = max(offs) - min(offs)
    jumps = sum(1 for a, b in zip(offs, offs[1:]) if abs(b - a) > OFFSET_SPREAD_MS)
    span_s = (pairs[-1][1] - pairs[0][1]) / 1000
    drift = (offs[-1] - offs[0]) / span_s if span_s else 0.0
    one_clock = spread <= OFFSET_SPREAD_MS
    if not one_clock:
        log.warning("SYSTEM_TIME: time_unix_usec and time_boot_ms drift apart (%+.2f ms/s)", drift)
    record_detail(request, (f"{'one clock' if one_clock else 'TWO CLOCKS'}: offset spread {spread:.1f} ms over "
                            f"{span_s:.1f}s boot time, drift {drift:+.2f} ms/s, {jumps} jumps"))


# ---------------------------------------------------------------------------
# D. After a reboot (last: each restarts the flight stack once, shared)
# ---------------------------------------------------------------------------

@dataclass
class _Reboot:
    na: str | None = None
    kill_mono: float = 0.0
    request: str = ""
    samples: list[Sample] = field(default_factory=list)


_reboot: _Reboot | None = None


async def _after_reboot(system, restart_flight_stack) -> _Reboot:
    """Restart the flight stack once and watch SYSTEM_TIME (at 5 Hz) from as soon as it's back."""
    global _reboot
    if _reboot is not None:
        return _reboot
    if report.MOCK_MODE:
        _reboot = _Reboot(na="mock flight stack: nothing to reboot")
        return _reboot
    kill = time.monotonic()
    restart_flight_stack()  # synchronous, like flight_helpers._rtl_and_land's use of it
    await ensure_harness_streams(system)
    target = await vehicle_target(system)
    back = time.monotonic()
    async with MessageSampler(system, _MSG_NAME) as s:
        result = await set_message_interval(system, _MSG_ID, 1e6 / 5.0)
        deadline = back + REBOOT_WATCH_S
        while time.monotonic() < deadline:
            mine = autopilot_only(s.samples(_MSG_NAME), target)
            valid = [x for x in mine if x.fields["time_unix_usec"]]
            if len(valid) >= 3:
                break
            await asyncio.sleep(0.2)
    await EMISSION.restore_default(system)
    mine = autopilot_only(s.samples(_MSG_NAME), target)
    _reboot = _Reboot(None, kill, f"5 Hz requested {(back - kill):.0f}s after the kill ({result_name(result)})", mine)
    if not mine:
        _reboot.na = f"no SYSTEM_TIME within {REBOOT_WATCH_S:.0f}s of the restart ({_reboot.request})"
    log.info("SYSTEM_TIME after reboot: %d msgs, first boot_ms=%s; %s", len(mine),
             mine[0].fields["time_boot_ms"] if mine else None, _reboot.request)
    return _reboot


@pytest.mark.timeout(300)
async def test_system_time_info_boot_ms_resets_on_reboot(gcs_system, request, restart_flight_stack):
    """(Information) After a restart, time_boot_ms is no more than the time since the old process was killed."""
    r = await _after_reboot(gcs_system, restart_flight_stack)
    if r.na:
        pytest.skip(f"NA: {r.na}")
    first = r.samples[0]
    since_kill_ms = (first.host_mono - r.kill_mono) * 1000
    boot_ms = first.fields["time_boot_ms"]
    detail = (f"(Information) first message: time_boot_ms {boot_ms} ms, {since_kill_ms:.0f} ms after the old "
              f"process was killed ({r.request})")
    record_detail(request, detail)
    assert boot_ms <= since_kill_ms + REBOOT_MARGIN_MS, f"time_boot_ms didn't restart from boot — {detail}"


@pytest.mark.timeout(300)
async def test_system_time_obs_unix_after_boot(gcs_system, request, restart_flight_stack):
    """Characterise: what time_unix_usec is sent between boot and the clock becoming known."""
    r = await _after_reboot(gcs_system, restart_flight_stack)
    if r.na:
        pytest.skip(f"NA: {r.na}")
    first = r.samples[0]
    zeros = [x for x in r.samples if x.fields["time_unix_usec"] == 0]
    valid = [x for x in r.samples if x.fields["time_unix_usec"]]
    zero_after_valid = bool(valid) and any(x.host_mono > valid[0].host_mono for x in zeros)
    if not zeros:
        what = f"valid from the first message (time_boot_ms {first.fields['time_boot_ms']})"
    elif valid:
        what = (f"0 in {len(zeros)} messages (time_boot_ms {zeros[0].fields['time_boot_ms']}.."
                f"{zeros[-1].fields['time_boot_ms']}), valid from time_boot_ms {valid[0].fields['time_boot_ms']}")
        record_field_fact(request, "time_unix_usec", notes="0 until clock set")
    else:
        what = f"0 in all {len(zeros)} messages for {REBOOT_WATCH_S:.0f}s"
        record_field_fact(request, "time_unix_usec", notes="0 until clock set")
    if zero_after_valid:
        what += "; went back to 0 after being valid"
    record_detail(request, f"{what} ({r.request})")
