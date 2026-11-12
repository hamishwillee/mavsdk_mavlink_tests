"""
Shared helpers for tests/messages_telemetry/ — tests of messages a flight
stack (or other component) emits to provide information: telemetry, and
replies to requests for information or state. See this directory's CLAUDE.md.

Nothing here is specific to one message, so every message test in this tree
uses the same building blocks:

- `MessageSampler` — records each arrival of one or more messages with the
  host's receive time (monotonic and wall clock) and the sender's ids.
- `set_message_interval()` / `request_message()` / `get_message_interval()` —
  MAV_CMD_SET_MESSAGE_INTERVAL, MAV_CMD_REQUEST_MESSAGE, and
  REQUEST_MESSAGE(MESSAGE_INTERVAL), each returning what the vehicle replied.
- `compat_verdict()` — the rule 4a verdict for an "is the request honoured"
  check (NACKed / honoured / accepted-but-ignored).
- `_message_auto_record` (autouse) — every test's outcome goes into the
  message's report (tests/report.py, protocol "message"), keyed on the test
  module's `_MSG_NAME`/`_MSG_ID`. `record_detail()`, `record_field_fact()`,
  `record_message_fact()` and `record_verdict()` add to it.

Commands are addressed to the vehicle's own system id (read from its
HEARTBEAT), not a hardcoded 1 — see root CLAUDE.md's multi-instance caveat.
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import statistics
import time
from dataclasses import dataclass

import pytest

from tests import report
from tests.command.conftest import probe_command_long
from tests.message_watcher import ARDUPILOT_REQUESTED_RATES_HZ, MAV_AUTOPILOT_ARDUPILOTMEGA

log = logging.getLogger(__name__)

MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_REQUEST_MESSAGE = 512
MSG_ID_MESSAGE_INTERVAL = 244

# Ids of the messages tests here request or cross-check against.
MSG_IDS = {
    "SYSTEM_TIME": 2, "GPS_RAW_INT": 24, "SCALED_IMU": 26, "SCALED_PRESSURE": 29, "ATTITUDE": 30,
    "ATTITUDE_QUATERNION": 31, "LOCAL_POSITION_NED": 32, "GLOBAL_POSITION_INT": 33, "GPS_GLOBAL_ORIGIN": 49,
    "SCALED_IMU2": 116, "GPS2_RAW": 124, "SCALED_IMU3": 129, "SCALED_PRESSURE2": 137, "SCALED_PRESSURE3": 143,
    "HOME_POSITION": 242, "MESSAGE_INTERVAL": 244, "EXTENDED_SYS_STATE": 245, "FLIGHT_INFORMATION": 264,
    "MISSION_CURRENT": 42, "VFR_HUD": 74,
}

MAV_RESULT_ACCEPTED = 0
MAV_RESULT_NAMES = {
    0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED", 3: "UNSUPPORTED", 4: "FAILED",
    5: "IN_PROGRESS", 6: "CANCELLED", 7: "COMMAND_LONG_ONLY", 8: "COMMAND_INT_ONLY",
    9: "COMMAND_UNSUPPORTED_MAV_FRAME", 10: "NOT_IN_CONTROL",
}

_MAV_AUTOPILOT_INVALID = 8

# Relative tolerance on a measured message rate vs the requested one. A
# starting point (Tier 2 pattern #2) — tighten from logged evidence.
RATE_TOLERANCE = float(os.environ.get("MESSAGE_RATE_TOLERANCE", "0.2"))

# After a rate change is ACKed, messages already queued at the old rate can
# still arrive: don't count this long after the ACK.
RATE_SETTLE_S = 0.5


def result_name(result: int | None) -> str:
    return "NO ACK" if result is None else MAV_RESULT_NAMES.get(result, str(result))


# ---------------------------------------------------------------------------
# Sampling
# ---------------------------------------------------------------------------

@dataclass
class Sample:
    host_mono: float  # time.monotonic() at receipt
    host_wall: float  # time.time() at receipt
    sysid: int
    compid: int
    fields: dict


class MessageSampler:
    """
    Record every arrival of the named messages while the context is open:

        async with MessageSampler(system, "SYSTEM_TIME", "ATTITUDE") as s:
            await asyncio.sleep(5)
        s.samples("SYSTEM_TIME")

    Passive — subscribes only, never changes what the vehicle sends.
    """

    def __init__(self, system, *names: str):
        self._system = system
        self._names = names
        self._samples: dict[str, list[Sample]] = {n: [] for n in names}
        self._tasks: list[asyncio.Task] = []
        self.started_mono = 0.0

    async def _watch(self, name: str) -> None:
        async for msg in self._system.mavlink_direct.message(name):
            self._samples[name].append(Sample(
                time.monotonic(), time.time(), msg.system_id, msg.component_id, json.loads(msg.fields_json),
            ))

    async def __aenter__(self) -> "MessageSampler":
        self._tasks = [asyncio.create_task(self._watch(n)) for n in self._names]
        await asyncio.sleep(0.05)  # let the subscriptions register before anything is sent
        self.started_mono = time.monotonic()
        return self

    async def __aexit__(self, *exc) -> None:
        for t in self._tasks:
            t.cancel()  # fire-and-forget (root CLAUDE.md design decision 4a)

    def samples(self, name: str, since: float | None = None) -> list[Sample]:
        out = self._samples[name]
        return out if since is None else [s for s in out if s.host_mono >= since]


def rate_hz(samples: list[Sample], window_s: float) -> float:
    """
    Messages per second. With 3+ samples, (n-1) over the first-to-last span —
    counting n in a fixed window overcounts by up to one message (11 in 5 s
    reads 2.2 Hz for a 2 Hz stream). Fewer than that: n / window_s.
    """
    if len(samples) >= 3:
        span = samples[-1].host_mono - samples[0].host_mono
        if span > 0:
            return (len(samples) - 1) / span
    return len(samples) / window_s if window_s > 0 else 0.0


def interval_stats(samples: list[Sample]) -> str:
    """'median/min/max' inter-arrival time in ms — logged as tuning evidence."""
    gaps = [(b.host_mono - a.host_mono) * 1000 for a, b in zip(samples, samples[1:])]
    if not gaps:
        return "n/a"
    return f"{statistics.median(gaps):.0f}/{min(gaps):.0f}/{max(gaps):.0f} ms"


def senders(samples: list[Sample]) -> list[tuple[int, int]]:
    return sorted({(s.sysid, s.compid) for s in samples})


async def measure_rate(system, name: str, window_s: float, settle_s: float = 0.0) -> tuple[float, list[Sample]]:
    """Wait `settle_s`, then count `name` for `window_s`. Returns (rate_hz, samples)."""
    if settle_s:
        await asyncio.sleep(settle_s)
    async with MessageSampler(system, name) as s:
        await asyncio.sleep(window_s)
    samples = s.samples(name)
    return rate_hz(samples, window_s), samples


def rate_matches(measured_hz: float, expected_hz: float, tolerance: float = RATE_TOLERANCE) -> bool:
    if expected_hz == 0:
        return measured_hz == 0
    return abs(measured_hz - expected_hz) <= tolerance * expected_hz


# ---------------------------------------------------------------------------
# Requests
# ---------------------------------------------------------------------------

async def vehicle_heartbeat(system, timeout_s: float = 5.0) -> dict:
    """
    The autopilot's HEARTBEAT (autopilot field not MAV_AUTOPILOT_INVALID), plus
    its "sysid"/"compid". Cached per connection generation. Falls back to
    sysid/compid (1, 1) — the mock drone and every single-instance SITL — with
    no other fields, if none arrives.
    """
    generation = getattr(system, "generation", 0)
    cached = getattr(system, "_msg_test_heartbeat", None)
    if cached is not None and cached[0] == generation:
        return cached[1]
    found: asyncio.Future = asyncio.get_running_loop().create_future()

    async def _hb() -> None:
        async for msg in system.mavlink_direct.message("HEARTBEAT"):
            fields = json.loads(msg.fields_json)
            if fields.get("autopilot") != _MAV_AUTOPILOT_INVALID and not found.done():
                found.set_result({**fields, "sysid": msg.system_id, "compid": msg.component_id})
                return

    task = asyncio.create_task(_hb())
    try:
        hb = await asyncio.wait_for(found, timeout_s)
    except asyncio.TimeoutError:
        log.warning("No autopilot HEARTBEAT within %.0fs — addressing commands to (1, 1)", timeout_s)
        hb = {"sysid": 1, "compid": 1}
    finally:
        task.cancel()
    system._msg_test_heartbeat = (generation, hb)
    return hb


async def vehicle_target(system, timeout_s: float = 5.0) -> tuple[int, int]:
    """(sysid, compid) of the autopilot, from its HEARTBEAT (see vehicle_heartbeat)."""
    hb = await vehicle_heartbeat(system, timeout_s)
    return hb["sysid"], hb["compid"]


async def harness_requested_hz(system, msg_id: int) -> float:
    """
    The rate the harness itself requests msg_id at, at the start of every test
    (ArduPilot only: message_watcher.request_ardupilot_streams), else 0. A
    "default rate" measured without undoing that would be the harness's rate.
    """
    hb = await vehicle_heartbeat(system)
    if hb.get("autopilot") == MAV_AUTOPILOT_ARDUPILOTMEGA:
        return ARDUPILOT_REQUESTED_RATES_HZ.get(msg_id, 0.0)
    return 0.0


async def _command(system, command: int, **params) -> int | None:
    sysid, compid = await vehicle_target(system)
    ack = await probe_command_long(system, command, target_system=sysid, target_component=compid, **params)
    return None if ack is None else int(ack.get("result", -1))


async def set_message_interval(system, msg_id: int, interval_us: float) -> int | None:
    """MAV_CMD_SET_MESSAGE_INTERVAL. Returns the MAV_RESULT, or None if no ACK arrived."""
    result = await _command(system, MAV_CMD_SET_MESSAGE_INTERVAL, param1=float(msg_id), param2=float(interval_us))
    log.info("SET_MESSAGE_INTERVAL(msg=%d, interval=%d us) -> %s", msg_id, interval_us, result_name(result))
    return result


async def request_message(system, msg_id: int, param2: float = 0.0) -> int | None:
    """MAV_CMD_REQUEST_MESSAGE. Returns the MAV_RESULT, or None if no ACK arrived."""
    result = await _command(system, MAV_CMD_REQUEST_MESSAGE, param1=float(msg_id), param2=float(param2))
    log.info("REQUEST_MESSAGE(msg=%d, param2=%s) -> %s", msg_id, param2, result_name(result))
    return result


async def get_message_interval(system, msg_id: int, timeout_s: float = 2.0) -> tuple[int | None, int | None]:
    """
    REQUEST_MESSAGE(MESSAGE_INTERVAL, param2=msg_id) — the current way to ask
    for a message's interval (GET_MESSAGE_INTERVAL is superseded). Returns
    (MAV_RESULT or None, MESSAGE_INTERVAL.interval_us or None if no reply).
    """
    async with MessageSampler(system, "MESSAGE_INTERVAL") as s:
        result = await request_message(system, MSG_ID_MESSAGE_INTERVAL, param2=msg_id)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            replies = [x for x in s.samples("MESSAGE_INTERVAL") if int(x.fields.get("message_id", -1)) == msg_id]
            if replies:
                return result, int(replies[0].fields["interval_us"])
            await asyncio.sleep(0.05)
    return result, None


# ---------------------------------------------------------------------------
# Field-check helpers
# ---------------------------------------------------------------------------

EARTH_RADIUS_M = 6_378_137.0


def ned_offset_m(lat0_e7: int, lon0_e7: int, lat_e7: int, lon_e7: int) -> tuple[float, float]:
    """(north, east) metres from point 0 to point 1, flat-earth (fine for < a few km)."""
    north = math.radians((lat_e7 - lat0_e7) / 1e7) * EARTH_RADIUS_M
    east = math.radians((lon_e7 - lon0_e7) / 1e7) * EARTH_RADIUS_M * math.cos(math.radians(lat0_e7 / 1e7))
    return north, east


def ned_from_origin(origin: dict, lat_e7: int, lon_e7: int, alt_mm: int) -> tuple[float, float, float]:
    """
    (north, east, down) metres of a global position relative to a
    GPS_GLOBAL_ORIGIN-shaped `origin` (latitude/longitude degE7, altitude mm).
    """
    n, e = ned_offset_m(origin["latitude"], origin["longitude"], lat_e7, lon_e7)
    return n, e, -(alt_mm - origin["altitude"]) / 1000.0


UNIX_2020_US = 1_577_836_800 * 1_000_000  # anything above this is a UNIX time


def classify_time_usec(samples: list[Sample], boot_ms_field: str | None = None,
                       boot_ms_at: dict[float, int] | None = None) -> tuple[str, bool, str]:
    """
    Characterise a "Timestamp (UNIX Epoch time or time since system boot)"
    field named time_usec across `samples`: (kind, plausible, description).

    kind: "zero", "unix", "boot" or "mixed". Plausible: a UNIX time no later
    than the host clock (+5 s), or a boot time no later than the sender's
    uptime — taken from `boot_ms_at` (host_mono -> sender time_boot_ms from
    another message), when given. Also says whether it advances with host time
    (send time) or stays put (the time something was set).
    """
    values = [int(s.fields.get("time_usec", 0)) for s in samples]
    if all(v == 0 for v in values):
        return "zero", False, "always 0"
    kinds = {"unix" if v >= UNIX_2020_US else "boot" for v in values if v}
    kind = kinds.pop() if len(kinds) == 1 else "mixed"
    first, last = samples[0], samples[-1]
    host_span = last.host_mono - first.host_mono
    advance = (values[-1] - values[0]) / 1e6
    if host_span > 1 and abs(advance - host_span) <= 0.2 * host_span:
        moving = "advances with host time (send time)"
    elif advance == 0:
        moving = "constant (the time it was set)"
    else:
        moving = f"advanced {advance:.1f}s over {host_span:.1f}s host"
    plausible = kind in ("unix", "boot")
    if kind == "unix":
        ahead = max(values) / 1e6 - last.host_wall
        plausible = ahead <= 5
        where = f"UNIX {ahead:+.1f}s vs host"
    elif kind == "boot":
        where = f"boot {max(values) / 1e6:.1f}s"
        if boot_ms_at:
            uptime_s = max(boot_ms_at.values()) / 1000
            plausible = max(values) / 1e6 <= uptime_s + 5
            where += f" (sender uptime {uptime_s:.1f}s)"
    else:
        where = "mixed UNIX and boot values"
    return kind, plausible, f"{where}, {moving}"


def finite(*values) -> bool:
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def nearest(samples: list[Sample], host_mono: float) -> Sample:
    """The sample received closest in host time (for messages without a shared timestamp)."""
    return min(samples, key=lambda s: abs(s.host_mono - host_mono))


def skip_untestable(request, fields, reason: str, detail: str | None = None) -> None:
    """Record every field in `fields` as untestable for `reason`, and skip as NA."""
    for f in fields:
        record_field_fact(request, f, untestable=reason)
    pytest.skip(f"NA: {detail or reason}")


def distinct(values, limit: int = 4) -> str:
    """A short display of the distinct values seen ("0, 65535" / "17..23 (9 distinct)")."""
    uniq = sorted(set(values), key=lambda v: (isinstance(v, str), v))
    if len(uniq) <= limit:
        return ", ".join(str(v) for v in uniq)
    return f"{uniq[0]}..{uniq[-1]} ({len(uniq)} distinct)"


# ---------------------------------------------------------------------------
# Verdicts (root CLAUDE.md rule 4a)
# ---------------------------------------------------------------------------

def compat_verdict(nacked: bool, ok: bool) -> str:
    """REJECTED (a legitimate PASS), SUPPORTED, or the accepted-but-not-honoured compatibility error."""
    if nacked:
        return "REJECTED (NACKed)"
    if ok:
        return "SUPPORTED"
    return "NOT SUPPORTED — COMPATIBILITY ERROR"


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

_DETAILS: dict[str, str] = {}  # nodeid -> explicit detail for the report row


def _key(request) -> report.Key | None:
    return report.key_from_module(request.node.module)


def record_detail(request, detail: str) -> None:
    """One-line detail for this test's report row, in place of its docstring/failure reason."""
    _DETAILS[request.node.nodeid] = detail


def record_field_fact(request, field: str, *, supported=report._UNSET, notes=None, untestable: str | None = None) -> None:
    """A field's verdict (supported True/False) or untestable reason, plus terse notes."""
    key = _key(request)
    if key is not None:
        report.record_compat_fact(*key, field, supported=supported, notes=notes, untestable=untestable)


def record_message_fact(request, *, supported=report._UNSET, notes=None) -> None:
    """Message-level fact: is it emitted at all (by default or on request), plus terse notes."""
    key = _key(request)
    if key is not None:
        report.record_command_fact(*key, supported=supported, notes=notes)


def record_verdict(request, label: str, verdict: str) -> None:
    """A line in the report's Compatibility summary (e.g. "SET_MESSAGE_INTERVAL rate": "SUPPORTED")."""
    key = _key(request)
    if key is not None:
        report.record_param_verdict(*key, label, verdict)


def is_flight_test(node) -> bool:
    """A flight test (flight.py; marked `flight`), reported in the Tier 2 table like a command's flight tests."""
    return node.get_closest_marker("flight") is not None


@pytest.fixture(autouse=True)
def _message_auto_record(request):
    """
    Record every test's outcome and detail in its message's report and rewrite it (see tests/report.py).
    Ground tests go in the report's first table; flight tests (marked `flight`) in the Tier 2 table,
    as tests/flight_helpers.py's `_tier2_auto_record` does for a command's test_flight.py.
    """
    yield
    node = request.node
    rep = getattr(node, "_report_call", None) or getattr(node, "_report_setup", None)
    key = _key(request)
    if rep is None or key is None:
        return
    doc = (node.function.__doc__ or "").strip()
    outcome, compat, detail = report.classify_report(rep, doc.splitlines()[0].strip() if doc else "")
    explicit = _DETAILS.pop(node.nodeid, None)
    if explicit is not None and outcome == "PASS":
        detail = explicit
    elif explicit is not None:
        detail = f"{detail} [{explicit}]"[:300]
    if is_flight_test(node):
        report.record_tier2_result(*key, node.name, outcome, detail, compat)
    else:
        report.record_tier1_result(*key, node.name, outcome, detail, None, compat)
    msg_id = getattr(node.module, "_MSG_ID", None)
    if msg_id is not None:
        report.write(*key, msg_id, request.config)
