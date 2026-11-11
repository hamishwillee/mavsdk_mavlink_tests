"""
Generic emission tests and shared field checks for any streamed message.

Every message in tests/messages_telemetry/ answers the same emission
questions, and most carry a `time_boot_ms`, so the test bodies live here once:

    EMISSION = Emission("ATTITUDE_QUATERNION", 31)

    async def test_attitude_quaternion_obs_default_rate(gcs_system, request):
        await EMISSION.default_rate(gcs_system, request)

Test *names* stay in each message's module (`test_<msg>_<category>_<rest>`,
root CLAUDE.md rule 8), so a report reads per message. What each check
asserts, and why, is in this directory's CLAUDE.md:

- default_rate (obs): unrequested rate and senders; does MESSAGE_INTERVAL agree.
- set_interval_honoured (compat): two rates, far apart (rule 4c). A NACK is a
  legitimate PASS; accepted but not honoured is the compatibility error (4a).
- set_interval_disable (compat): -1 stops it (after first making it stream).
- set_interval_default (compat): 0 after a change restores the unrequested rate.
- request_message (compat): with the stream off, exactly one message.
- sample_window(): one shared high-rate window for the field tests (rule 7).
- boot_ms_tracks_real_time(): a `time_boot_ms` field that never goes
  backwards or sticks, and advances at host-clock rate.

The unrequested rate is measured once per session, before anything changes it,
so every rate test restores interval 0 when it's done.
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, field

import pytest

from tests import report
from tests.messages_telemetry.conftest import (
    MAV_RESULT_ACCEPTED,
    RATE_SETTLE_S,
    RATE_TOLERANCE,
    MessageSampler,
    Sample,
    compat_verdict,
    get_message_interval,
    interval_stats,
    measure_rate,
    rate_hz,
    rate_matches,
    record_detail,
    record_field_fact,
    record_message_fact,
    record_verdict,
    request_message,
    result_name,
    senders,
    set_message_interval,
    vehicle_target,
)

log = logging.getLogger(__name__)

DEFAULT_WINDOW_S = 10.0     # unrequested listen: long enough for a 0.2 Hz stream to show twice
RATE_WINDOW_S = 5.0         # per requested rate
RATES_HZ = (2.0, 10.0)      # rule 4c: two values, far apart
SILENCE_WINDOW_S = 3.0      # "disabled" means nothing for this long
SAMPLE_RATE_HZ = 10.0       # the shared field-analysis window
SAMPLE_WINDOW_S = 10.0
BOOT_RATE_TOLERANCE = 0.05  # time_boot_ms advance vs host time


def autopilot_only(samples: list[Sample], target: tuple[int, int]) -> list[Sample]:
    return [s for s in samples if (s.sysid, s.compid) == target]


@dataclass
class Baseline:
    rate_hz: float
    samples: list[Sample]           # autopilot only
    all_senders: list[tuple[int, int]]


@dataclass
class Window:
    na: str | None = None                          # why the window couldn't be analysed
    how: str = ""
    samples: list[Sample] = field(default_factory=list)   # the message, autopilot only
    others: dict[str, list[Sample]] = field(default_factory=dict)


class Emission:
    """The emission tests and shared windows for one message, with its cached measurements."""

    def __init__(self, name: str, msg_id: int, window_extra: tuple[str, ...] = ()):
        self.name = name
        self.msg_id = msg_id
        self.window_extra = window_extra  # sampled alongside in sample_window(), for cross-checks
        self._baseline: Baseline | None = None
        self._window: Window | None = None
        self.ever_received = False

    # -- shared state ------------------------------------------------------

    def _note(self, samples: list[Sample]) -> None:
        if samples:
            self.ever_received = True

    async def restore_default(self, system) -> None:
        await set_message_interval(system, self.msg_id, 0)

    async def baseline(self, system) -> Baseline:
        """The unrequested rate — measured once, before any test changes the interval."""
        if self._baseline is None:
            target = await vehicle_target(system)
            _, samples = await measure_rate(system, self.name, DEFAULT_WINDOW_S)
            mine = autopilot_only(samples, target)
            self._baseline = Baseline(rate_hz(mine, DEFAULT_WINDOW_S), mine, senders(samples))
            self._note(mine)
            log.info("%s default: %.2f Hz (%d msgs in %.0fs, intervals median/min/max %s), senders %s",
                     self.name, self._baseline.rate_hz, len(mine), DEFAULT_WINDOW_S, interval_stats(mine),
                     self._baseline.all_senders)
        return self._baseline

    async def sample_window(self, system) -> Window:
        """
        One SAMPLE_WINDOW_S window of the message at SAMPLE_RATE_HZ (or its
        default rate if the request is refused), with `window_extra` sampled
        alongside. Read by every field test, so it's sampled once.
        """
        if self._window is not None:
            return self._window
        baseline = await self.baseline(system)
        target = await vehicle_target(system)
        result = await set_message_interval(system, self.msg_id, 1e6 / SAMPLE_RATE_HZ)
        try:
            await asyncio.sleep(RATE_SETTLE_S)
            async with MessageSampler(system, self.name, *self.window_extra) as s:
                await asyncio.sleep(SAMPLE_WINDOW_S)
        finally:
            await self.restore_default(system)
        mine = autopilot_only(s.samples(self.name), target)
        self._note(mine)
        how = (f"{SAMPLE_RATE_HZ:.0f} Hz requested, {len(mine)} msgs in {SAMPLE_WINDOW_S:.0f}s"
               if result == MAV_RESULT_ACCEPTED else
               f"rate request {result_name(result)}, default stream: {len(mine)} msgs in {SAMPLE_WINDOW_S:.0f}s")
        na = None
        if len(mine) < 3:
            na = f"too few {self.name} messages to analyse ({how}; default {baseline.rate_hz:.2f} Hz)"
        others = {n: autopilot_only(s.samples(n), target) for n in self.window_extra}
        self._window = Window(na, how, mine, others)
        log.info("%s sample window: %s; intervals median/min/max %s; others %s",
                 self.name, how, interval_stats(mine), {n: len(v) for n, v in others.items()})
        return self._window

    # -- the five emission tests --------------------------------------------

    async def default_rate(self, system, request) -> None:
        """obs: unrequested rate and senders, and whether MESSAGE_INTERVAL agrees."""
        b = await self.baseline(system)
        ack, interval_us = await get_message_interval(system, self.msg_id)
        if interval_us is None:
            mi, agrees = f"no MESSAGE_INTERVAL reply (REQUEST_MESSAGE {result_name(ack)})", None
        elif interval_us > 0:
            mi_hz = 1e6 / interval_us
            agrees = rate_matches(b.rate_hz, mi_hz)
            mi = f"MESSAGE_INTERVAL {interval_us} us ({mi_hz:.2f} Hz)"
        else:
            agrees = b.rate_hz == 0
            mi = f"MESSAGE_INTERVAL {interval_us} ({'disabled' if interval_us == -1 else 'not available'})"
        if agrees is False:
            log.warning("%s: measured default %.2f Hz but %s", self.name, b.rate_hz, mi)
        record_detail(request, (
            f"{b.rate_hz:.2f} Hz unrequested (median gap {interval_stats(b.samples)}), senders {b.all_senders}; "
            f"{mi}{'' if agrees is None else (' — agrees' if agrees else ' — DISAGREES with measured rate')}"
        ))
        if b.samples:
            record_message_fact(request, supported=True, notes="Streamed by default")
        else:
            record_message_fact(request, notes="Not streamed by default")

    async def set_interval_honoured(self, system, request) -> None:
        """compat: SET_MESSAGE_INTERVAL at each of RATES_HZ is NACKed, or honoured."""
        await self.baseline(system)
        target = await vehicle_target(system)
        rows = []  # (hz, result, measured_hz or None)
        try:
            for hz in RATES_HZ:
                result = await set_message_interval(system, self.msg_id, 1e6 / hz)
                if result is None:
                    pytest.fail(f"no COMMAND_ACK for SET_MESSAGE_INTERVAL({self.msg_id}, {hz} Hz)")
                if result != MAV_RESULT_ACCEPTED:
                    rows.append((hz, result, None))
                    continue
                _, samples = await measure_rate(system, self.name, RATE_WINDOW_S, settle_s=RATE_SETTLE_S)
                mine = autopilot_only(samples, target)
                self._note(mine)
                rows.append((hz, result, rate_hz(mine, RATE_WINDOW_S)))
                log.info("%s at %.0f Hz requested: measured %.2f Hz (gaps median/min/max %s)",
                         self.name, hz, rows[-1][2], interval_stats(mine))
        finally:
            await self.restore_default(system)

        nacked = [r for r in rows if r[2] is None]
        ignored = [r for r in rows if r[2] is not None and not rate_matches(r[2], r[0])]
        verdict = compat_verdict(nacked=len(nacked) == len(rows), ok=not ignored)
        if nacked and not ignored and len(nacked) < len(rows):
            verdict += f" ({', '.join(f'{r[0]:.0f} Hz {result_name(r[1])}' for r in nacked)})"
        record_verdict(request, "SET_MESSAGE_INTERVAL rate", verdict)
        detail = "; ".join(
            f"{hz:.0f} Hz: {result_name(res)}" + ("" if m is None else f", measured {m:.2f} Hz")
            for hz, res, m in rows
        )
        record_detail(request, f"{verdict} — {detail} (±{RATE_TOLERANCE:.0%})")
        if self.ever_received:
            record_message_fact(request, supported=True)
        if ignored:
            report.compat_fail(f"SET_MESSAGE_INTERVAL accepted but rate not honoured — {detail}")

    async def set_interval_disable(self, system, request) -> None:
        """compat: SET_MESSAGE_INTERVAL -1 is NACKed, or nothing arrives for SILENCE_WINDOW_S."""
        b = await self.baseline(system)
        target = await vehicle_target(system)
        try:
            # Make sure it's streaming first, or "disabled" is indistinguishable from "not streamed".
            enable = await set_message_interval(system, self.msg_id, 1e6 / 5.0)
            _, samples = await measure_rate(system, self.name, 2.0, settle_s=RATE_SETTLE_S)
            before = rate_hz(autopilot_only(samples, target), 2.0)
            if before == 0:
                pytest.skip(f"NA: can't get {self.name} streaming to disable it (5 Hz request "
                            f"{result_name(enable)}, default {b.rate_hz:.2f} Hz)")
            result = await set_message_interval(system, self.msg_id, -1)
            if result is None:
                pytest.fail("no COMMAND_ACK for SET_MESSAGE_INTERVAL(-1)")
            if result != MAV_RESULT_ACCEPTED:
                verdict = compat_verdict(nacked=True, ok=False)
                record_verdict(request, "SET_MESSAGE_INTERVAL -1 (disable)", verdict)
                record_detail(request, f"{verdict}: {result_name(result)}")
                return
            _, samples = await measure_rate(system, self.name, SILENCE_WINDOW_S, settle_s=RATE_SETTLE_S)
            n = len(autopilot_only(samples, target))
        finally:
            await self.restore_default(system)
        verdict = compat_verdict(nacked=False, ok=n == 0)
        record_verdict(request, "SET_MESSAGE_INTERVAL -1 (disable)", verdict)
        record_detail(request, f"{verdict}: {before:.1f} Hz before, {n} msgs in {SILENCE_WINDOW_S:.0f}s after -1")
        if n:
            report.compat_fail(f"SET_MESSAGE_INTERVAL -1 accepted but {n} {self.name} still arrived "
                               f"in {SILENCE_WINDOW_S:.0f}s")

    async def set_interval_default(self, system, request) -> None:
        """compat: SET_MESSAGE_INTERVAL 0 after a change is NACKed, or restores the unrequested rate."""
        b = await self.baseline(system)
        target = await vehicle_target(system)
        change_hz = 10.0 if b.rate_hz < 5.0 else 1.0  # well away from the default either way
        try:
            result = await set_message_interval(system, self.msg_id, 1e6 / change_hz)
            _, samples = await measure_rate(system, self.name, 2.0, settle_s=RATE_SETTLE_S)
            changed = rate_hz(autopilot_only(samples, target), 2.0)
            if result != MAV_RESULT_ACCEPTED or rate_matches(changed, b.rate_hz):
                pytest.skip(f"NA: couldn't move the rate away from the default to test restoring it "
                            f"({change_hz:.0f} Hz request {result_name(result)}, measured {changed:.2f} Hz)")
            result = await set_message_interval(system, self.msg_id, 0)
            if result is None:
                pytest.fail("no COMMAND_ACK for SET_MESSAGE_INTERVAL(0)")
            if result != MAV_RESULT_ACCEPTED:
                verdict = compat_verdict(nacked=True, ok=False)
                record_verdict(request, "SET_MESSAGE_INTERVAL 0 (default)", verdict)
                record_detail(request, f"{verdict}: {result_name(result)}")
                return
            _, samples = await measure_rate(system, self.name, RATE_WINDOW_S, settle_s=RATE_SETTLE_S)
            restored = rate_hz(autopilot_only(samples, target), RATE_WINDOW_S)
        finally:
            await self.restore_default(system)
        ok = rate_matches(restored, b.rate_hz) or (b.rate_hz < 0.5 and restored < 0.5)
        verdict = compat_verdict(nacked=False, ok=ok)
        record_verdict(request, "SET_MESSAGE_INTERVAL 0 (default)", verdict)
        record_detail(request, f"{verdict}: default {b.rate_hz:.2f} Hz, changed to {changed:.2f} Hz, "
                               f"after 0 {restored:.2f} Hz")
        if not ok:
            report.compat_fail(f"SET_MESSAGE_INTERVAL 0 accepted but rate {restored:.2f} Hz, "
                               f"default was {b.rate_hz:.2f} Hz")

    async def request_message(self, system, request) -> None:
        """compat: REQUEST_MESSAGE with the stream off is NACKed, or gives exactly one message."""
        b = await self.baseline(system)
        target = await vehicle_target(system)
        try:
            if b.rate_hz > 0:
                off = await set_message_interval(system, self.msg_id, -1)
                _, samples = await measure_rate(system, self.name, 2.0, settle_s=RATE_SETTLE_S)
                if autopilot_only(samples, target):
                    pytest.skip(f"NA: can't stop the stream to isolate a one-shot reply (-1 {result_name(off)})")
            async with MessageSampler(system, self.name) as s:
                sent = time.monotonic()
                result = await request_message(system, self.msg_id)
                await asyncio.sleep(2.0)
            replies = autopilot_only(s.samples(self.name), target)
        finally:
            await self.restore_default(system)
        self._note(replies)
        if result is None:
            pytest.fail(f"no COMMAND_ACK for REQUEST_MESSAGE({self.msg_id}) ({len(replies)} {self.name} arrived)")
        nacked = result != MAV_RESULT_ACCEPTED
        ok = len(replies) == 1
        verdict = compat_verdict(nacked=nacked, ok=ok)
        latency = f", {(replies[0].host_mono - sent) * 1000:.0f} ms after the request" if replies else ""
        record_verdict(request, "REQUEST_MESSAGE", verdict)
        record_detail(request, f"{verdict}: {result_name(result)}, {len(replies)} message(s) in 2s{latency}")
        if self.ever_received:
            record_message_fact(request, supported=True)
        elif nacked or not replies:
            record_message_fact(request, supported=False, notes="Never sent, even on request")
        if not nacked and not ok:
            report.compat_fail(f"REQUEST_MESSAGE accepted but {len(replies)} {self.name} arrived (expected exactly one)")

    # -- shared field check --------------------------------------------------

    async def boot_ms_tracks_real_time(self, system, request, field_name: str = "time_boot_ms") -> None:
        """compat: `field_name` never goes backwards or sticks, and advances at host-clock rate (±5%)."""
        w = await self.sample_window(system)
        if w.na:
            record_field_fact(request, field_name, untestable="not enough messages to analyse")
            pytest.skip(f"NA: {w.na}")
        boot = [s.fields[field_name] for s in w.samples]
        backwards = sum(1 for a, b in zip(boot, boot[1:]) if b < a)
        repeats = sum(1 for a, b in zip(boot, boot[1:]) if b == a)
        host_span_ms = (w.samples[-1].host_mono - w.samples[0].host_mono) * 1000
        ratio = (boot[-1] - boot[0]) / host_span_ms if host_span_ms else 0.0
        ok = backwards == 0 and boot[-1] > boot[0] and abs(ratio - 1) <= BOOT_RATE_TOLERANCE
        record_field_fact(request, field_name, supported=ok)
        detail = (f"{boot[0]}..{boot[-1]} ms over {host_span_ms / 1000:.1f}s host: rate ×{ratio:.4f}, "
                  f"{backwards} backwards, {repeats} repeated ({w.how})")
        record_detail(request, detail)
        if not ok:
            report.compat_fail(f"{field_name} doesn't behave as time since boot — {detail}")
