"""
Watch the MAVLink messages a test's data comes from — a debugging aid.

Much of what a test waits on is derived by MAVSDK from one specific message:
`telemetry.health().is_armable` is only ever the PREARM_CHECK bit of
SYS_STATUS, `landed_state()` only EXTENDED_SYS_STATE, and so on. If the
vehicle never sends that message, MAVSDK keeps its default (False, UNKNOWN,
...) forever, and the test times out looking exactly like a vehicle that
isn't ready. Found 2026-10-01: ArduCopter streams almost nothing unrequested
(its SRn_* stream rates default to 0), so it never sent SYS_STATUS and was
"never armable" — a harness gap that was recorded for months as a SITL boot
problem (root CLAUDE.md future-work #7).

`MessageWatcher` counts every message a test depends on — the harness's own
set (`HARNESS_MESSAGES`) plus whatever the test module declares in a
module-level `REQUIRED_MESSAGES = {"WIND_COV": "what reads it"}` — and keeps
the vehicle's recent STATUSTEXT. It is passive: it never changes what the
vehicle sends. `tests/conftest.py`'s `gcs_system` fixture runs one per test in
standalone mode, attaches it as `system.watcher`, logs a warning at teardown
for any dependency that never arrived, and appends its report to a failing
test's output. A wait that can time out on missing data (e.g.
`flight_helpers._wait_armable`) should put `watcher.explain(...)` in its
failure message.

`request_ardupilot_streams()` is the one active part: ArduPilot is asked for
the harness's messages explicitly, since it doesn't send them by default.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from collections import deque

from mavsdk.plugins.mavlink_direct import MavlinkMessage

from tests.mavsdk_compat import GCS_COMPID, GCS_SYSID

log = logging.getLogger(__name__)

# Messages the shared harness reads data from, whatever the test, and what
# depends on each (MAVSDK's telemetry_impl.cpp: each of these values is set
# from exactly this message).
HARNESS_MESSAGES: dict[str, str] = {
    "HEARTBEAT": "connection, telemetry.armed(), flight_mode()",
    "SYS_STATUS": "telemetry.health() — is_armable and every calibration/position flag",
    "GLOBAL_POSITION_INT": "telemetry.position(), heading()",
    "ATTITUDE": "telemetry.attitude_euler()",
    "EXTENDED_SYS_STATE": "telemetry.landed_state(), vtol_state()",
    "HOME_POSITION": "telemetry.home()",
    "MISSION_CURRENT": "mission_raw.mission_progress()",
}

# ArduPilot-only: what flight_helpers._wait_armable waits on before arming
# (the EKF position checks ArduPilot's own CI waits for — see there).
ARDUPILOT_MESSAGES: dict[str, str] = {
    "EKF_STATUS_REPORT": "flight_helpers._wait_armable — EKF has a usable absolute position",
}

# ArduPilot sends none of HARNESS_MESSAGES but HEARTBEAT unrequested on Copter
# (every SRn_* stream rate defaults to 0 — GCS_MAVLink_Parameters.cpp), and
# only at 1 Hz on Plane/Rover. HOME_POSITION is only sent on request or when
# home changes; MAVSDK requests it once per connection, and telemetry.home()
# only yields on a *new* message — so from the second test of a session on,
# _get_home_position() waited forever (2026-10-01; the watcher reported
# "HOME_POSITION: NEVER RECEIVED"). Streaming it fixes that. Rates are what the
# harness needs, not what a test may raise them to.
ARDUPILOT_REQUESTED_RATES_HZ: dict[int, float] = {
    1: 2.0,     # SYS_STATUS
    245: 2.0,   # EXTENDED_SYS_STATE
    30: 10.0,   # ATTITUDE
    33: 5.0,    # GLOBAL_POSITION_INT
    42: 2.0,    # MISSION_CURRENT
    242: 1.0,   # HOME_POSITION
    193: 2.0,   # EKF_STATUS_REPORT (ARDUPILOT_MESSAGES)
}

# A test shorter than this may legitimately end before a 1 Hz message arrives.
MIN_WATCH_S_FOR_MISSING = 5.0

MAV_AUTOPILOT_ARDUPILOTMEGA = 3
_MAV_AUTOPILOT_INVALID = 8
_STATUSTEXT_KEEP = 15


async def request_message_rate(system, msg_id: int, rate_hz: float, target_component: int = 1) -> None:
    """MAV_CMD_SET_MESSAGE_INTERVAL for msg_id at rate_hz (e.g. MISSION_CURRENT=42 at 10 Hz). Fire-and-forget."""
    await system.mavlink_direct.send_message(MavlinkMessage(
        message_name="COMMAND_LONG",
        system_id=GCS_SYSID, component_id=GCS_COMPID,
        target_system_id=1, target_component_id=target_component,
        fields_json=json.dumps({
            "target_system": 1, "target_component": target_component, "command": 511, "confirmation": 0,
            "param1": float(msg_id), "param2": float(int(1_000_000 / rate_hz)),
            "param3": 0.0, "param4": 0.0, "param5": 0.0, "param6": 0.0, "param7": 0.0,
        }),
    ))


async def request_ardupilot_streams(system, timeout_s: float = 2.0) -> None:
    """
    Ask ArduPilot for every message the harness depends on
    (ARDUPILOT_REQUESTED_RATES_HZ), and wait for the ACKs so none arrives
    during the test itself (a test counting COMMAND_ACKs would see it).
    Needed again after every reboot/restart — ArduPilot doesn't persist
    message intervals.
    """
    results: list[int] = []  # the ACK doesn't say which message it's for, so just count them
    done = asyncio.Event()

    async def _acks() -> None:
        async for msg in system.mavlink_direct.message("COMMAND_ACK"):
            f = json.loads(msg.fields_json)
            if f.get("command") == 511:
                results.append(f.get("result"))
                if len(results) == len(ARDUPILOT_REQUESTED_RATES_HZ):
                    done.set()
                    return

    task = asyncio.create_task(_acks())
    await asyncio.sleep(0)  # let the subscription start before the first ACK can arrive
    try:
        for msg_id, rate_hz in ARDUPILOT_REQUESTED_RATES_HZ.items():
            await request_message_rate(system, msg_id, rate_hz)
        await asyncio.wait_for(done.wait(), timeout_s)
    except asyncio.TimeoutError:
        log.warning("ArduPilot stream request: only %d/%d SET_MESSAGE_INTERVAL ACKs within %.1fs",
                    len(results), len(ARDUPILOT_REQUESTED_RATES_HZ), timeout_s)
    finally:
        task.cancel()
    refused = [r for r in results if r != 0]
    if refused:
        log.warning("ArduPilot stream request: %d SET_MESSAGE_INTERVAL refused (results %s)", len(refused), refused)


async def ensure_harness_streams(system, force: bool = False) -> None:
    """
    (Re)establish what the harness needs from the vehicle after a connection
    change: re-subscribe the watcher, and on ArduPilot re-request its streams
    (a restart or reboot forgets message intervals). Cheap no-op unless the
    shim's `generation` changed (Endpoint.reopen) or `force` is set — e.g. a
    reboot command that didn't reopen the connection, or a new test.
    """
    generation = getattr(system, "generation", 0)
    if not force and getattr(system, "_streams_generation", None) == generation:
        return
    watcher = getattr(system, "watcher", None)
    if watcher is not None and watcher.generation != generation:
        await watcher.resubscribe()
    if getattr(system, "autopilot_type", None) == MAV_AUTOPILOT_ARDUPILOTMEGA:
        await request_ardupilot_streams(system)
    system._streams_generation = generation


class MessageWatcher:
    """Counts arrivals of the messages a test depends on; see the module docstring."""

    def __init__(self, system, required: dict[str, str]):
        self.system = system
        self.required = dict(required)
        self.counts: dict[str, int] = {name: 0 for name in self.required}
        self.first: dict[str, float] = {}
        self.last: dict[str, float] = {}
        self.statustext: deque[tuple[float, str]] = deque(maxlen=_STATUSTEXT_KEEP)
        self.autopilot: int | None = None
        self._heartbeat = asyncio.Event()
        self._tasks: list[asyncio.Task] = []
        self._t0 = time.monotonic()
        self.generation = getattr(system, "generation", 0)
        self.resubscribes = 0

    async def start(self) -> None:
        for name in sorted(set(self.required) | {"HEARTBEAT", "STATUSTEXT"}):
            self._tasks.append(asyncio.create_task(self._watch(name)))
        await asyncio.sleep(0)  # let the subscriptions start

    async def resubscribe(self) -> None:
        """Subscribe again on the system's current connection (after Endpoint.reopen); counts carry on."""
        await self.stop()
        self.generation = getattr(self.system, "generation", 0)
        self.resubscribes += 1
        await self.start()

    def add(self, required: dict[str, str]) -> None:
        """Watch more messages (e.g. autopilot-specific ones, once the autopilot is known)."""
        for name, why in required.items():
            if name not in self.required:
                self.required[name] = why
                self.counts.setdefault(name, 0)
                self._tasks.append(asyncio.create_task(self._watch(name)))

    async def wait_for_heartbeat(self, timeout_s: float = 3.0) -> bool:
        """Wait for an autopilot HEARTBEAT (sets `autopilot`). False on timeout."""
        try:
            await asyncio.wait_for(self._heartbeat.wait(), timeout_s)
            return True
        except asyncio.TimeoutError:
            return False

    async def stop(self) -> None:
        for task in self._tasks:
            task.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        self._tasks.clear()

    async def _watch(self, name: str) -> None:
        async for msg in self.system.mavlink_direct.message(name):
            now = time.monotonic()
            self.counts[name] = self.counts.get(name, 0) + 1
            self.first.setdefault(name, now)
            self.last[name] = now
            if name == "HEARTBEAT" and self.autopilot is None:
                autopilot = json.loads(msg.fields_json).get("autopilot")
                if autopilot is not None and autopilot != _MAV_AUTOPILOT_INVALID:
                    self.autopilot = autopilot
                    self._heartbeat.set()
            elif name == "STATUSTEXT":
                text = json.loads(msg.fields_json).get("text", "")
                self.statustext.append((now - self._t0, text))

    # -- reporting ----------------------------------------------------------

    def missing(self) -> list[str]:
        """
        Streamed dependencies not received at all since the watcher started —
        empty if it ran for less than MIN_WATCH_S_FOR_MISSING (too short to tell).
        """
        if time.monotonic() - self._t0 < MIN_WATCH_S_FOR_MISSING:
            return []
        return [n for n in self.required if not self.counts.get(n)]

    def _line(self, name: str) -> str:
        n = self.counts.get(name, 0)
        why = self.required.get(name, "")
        if not n:
            return f"{name}: NEVER RECEIVED — {why}"
        now = time.monotonic()
        span = self.last[name] - self.first[name]
        rate = f"{(n - 1) / span:.1f} Hz" if n > 1 and span > 0 else "once"
        return f"{name}: {n} ({rate}, last {now - self.last[name]:.1f}s ago) — {why}"

    def explain(self, *names: str) -> str:
        """
        One line on the given dependencies, for a timeout message:
        e.g. explain("SYS_STATUS") -> "SYS_STATUS: NEVER RECEIVED — telemetry.health() ...".
        """
        lines = [self._line(n) for n in names]
        if self.statustext:
            lines.append("recent STATUSTEXT: " + " | ".join(t for _, t in list(self.statustext)[-5:]))
        return "; ".join(lines)

    def report(self) -> str:
        """Multi-line report: every required message, then the recent STATUSTEXT."""
        restarts = f", resubscribed after {self.resubscribes} connection restart(s)" if self.resubscribes else ""
        lines = [f"Message watcher ({time.monotonic() - self._t0:.0f}s{restarts}):"]
        lines += [f"  {self._line(name)}" for name in self.required]
        if self.statustext:
            lines.append("  recent STATUSTEXT:")
            lines += [f"    +{t:.1f}s {text}" for t, text in self.statustext]
        return "\n".join(lines)
