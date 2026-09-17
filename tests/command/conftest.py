"""
Command protocol helpers and fixtures for tests/command/.

Provides raw send/receive helpers for COMMAND_INT and COMMAND_LONG, the
protocol retry loops, XML command loading, and class-scoped fixtures that
mirror the pattern used in tests/mission/nav_takeoff/test_protocol.py.

No-ACK policy
-------------
A missing COMMAND_ACK is treated as result UNKNOWN, never UNSUPPORTED.  Not
receiving an ACK is a spec violation by the flight stack, but it is more likely
to mean the command is executing without acknowledgement than that the command
is unsupported.  Log at WARNING level when no ACK is received.
"""

import asyncio
import json
import logging
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import pytest
import pytest_asyncio
from mavsdk import System
from mavsdk.mavlink_direct import MavlinkMessage

from tests import report
from tests.conftest import DRONE_GRPC_PORT, _wait_for_connection
from tests.mock_flight_stack import (
    MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME,
    MAV_RESULT_DENIED,
    MAV_RESULT_UNSUPPORTED,
    MockFlightStack,
)
from tests.param_spec import INT32_MAX, MAV_FRAME_CATALOGUE, ParamSpec

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

ACK_TIMEOUT_S = 5.0    # per-attempt timeout waiting for COMMAND_ACK
RETRY_TIMEOUT_S = 30.0  # total timeout for the retry loop

# Window for collecting ALL acks per send in Tier1CommandTestBase, not just
# the first — needed to catch a delayed duplicate/racing ACK (see
# probe_command_int_all_acks() / effective_ack()). Matches the window used by
# test_ack_uniqueness.py for the same reason.
_ACK_WINDOW_S = 1.5

_GCS_SYSID = 255
_GCS_COMPID = 1
_DRONE_SYSID = 1
_DRONE_COMPID = 1

_FMT = "%-14s | %-44s | %s"

# ---------------------------------------------------------------------------
# XML command loading
# ---------------------------------------------------------------------------


def _load_commands(definitions_dir: Path) -> dict[int, str]:
    """
    Parse common.xml (recursively following <include> tags) and return a
    mapping of {cmd_value: cmd_name} for all MAV_CMD entries.

    common.xml is the full superset — it includes standard.xml which includes
    minimal.xml.  Parsing common.xml with recursive resolution gives all 168+
    standard commands.
    """
    seen: set[str] = set()
    commands: dict[int, str] = {}

    def _parse(filename: str) -> None:
        if filename in seen:
            return
        seen.add(filename)
        filepath = definitions_dir / filename
        if not filepath.exists():
            log.warning("MAVLink XML file not found: %s", filepath)
            return
        tree = ET.parse(filepath)
        for inc in tree.findall(".//include"):
            if inc.text:
                _parse(inc.text.strip())
        for enum in tree.findall('.//enum[@name="MAV_CMD"]'):
            for entry in enum.findall("entry"):
                val = entry.get("value")
                name = entry.get("name")
                if val is not None and name is not None:
                    commands[int(val)] = name

    _parse("common.xml")
    return commands


# ---------------------------------------------------------------------------
# Raw send helpers
# ---------------------------------------------------------------------------


async def send_command_int(
    system: System,
    command: int,
    frame: int = 6,  # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    param1: float = 0.0,
    param2: float = 0.0,
    param3: float = 0.0,
    param4: float = 0.0,
    x: int = 0,
    y: int = 0,
    z: float = 0.0,
    target_system: int = _DRONE_SYSID,
    target_component: int = _DRONE_COMPID,
) -> None:
    """Send a raw COMMAND_INT message via mavlink_direct."""
    await system.mavlink_direct.send_message(MavlinkMessage(
        message_name="COMMAND_INT",
        system_id=_GCS_SYSID,
        component_id=_GCS_COMPID,
        target_system_id=target_system,
        target_component_id=target_component,
        fields_json=json.dumps({
            "target_system": target_system,
            "target_component": target_component,
            "frame": frame,
            "command": command,
            "current": 0,
            "autocontinue": 0,
            "param1": param1,
            "param2": param2,
            "param3": param3,
            "param4": param4,
            "x": x,
            "y": y,
            "z": z,
        }),
    ))


async def send_command_long(
    system: System,
    command: int,
    param1: float = 0.0,
    param2: float = 0.0,
    param3: float = 0.0,
    param4: float = 0.0,
    param5: float = 0.0,
    param6: float = 0.0,
    param7: float = 0.0,
    confirmation: int = 0,
    target_system: int = _DRONE_SYSID,
    target_component: int = _DRONE_COMPID,
) -> None:
    """Send a raw COMMAND_LONG message via mavlink_direct."""
    await system.mavlink_direct.send_message(MavlinkMessage(
        message_name="COMMAND_LONG",
        system_id=_GCS_SYSID,
        component_id=_GCS_COMPID,
        target_system_id=target_system,
        target_component_id=target_component,
        fields_json=json.dumps({
            "target_system": target_system,
            "target_component": target_component,
            "command": command,
            "confirmation": confirmation,
            "param1": param1,
            "param2": param2,
            "param3": param3,
            "param4": param4,
            "param5": param5,
            "param6": param6,
            "param7": param7,
        }),
    ))


# ---------------------------------------------------------------------------
# ACK receive helpers
# ---------------------------------------------------------------------------

# Delay between starting the COMMAND_ACK subscription and sending the command,
# to ensure the gRPC stream is registered on the server before the ACK can arrive.
_SUBSCRIPTION_SETTLE_S = 0.05


async def probe_command_int(
    system: System,
    command: int,
    frame: int = 6,
    timeout_s: float = ACK_TIMEOUT_S,
    **send_kwargs,
) -> dict | None:
    """
    Subscribe to COMMAND_ACK, then send COMMAND_INT, then collect the ACK.

    Establishes the gRPC subscription before sending to avoid a race condition
    where the mock (or a fast real stack) sends the ACK before the subscription
    is registered on the server.

    Returns the COMMAND_ACK fields dict, or None on timeout (UNKNOWN).
    """
    return await _probe_with_send(
        system, command, timeout_s,
        lambda: send_command_int(system, command, frame, **send_kwargs),
    )


async def probe_command_long(
    system: System,
    command: int,
    timeout_s: float = ACK_TIMEOUT_S,
    confirmation: int = 0,
    **send_kwargs,
) -> dict | None:
    """
    Subscribe to COMMAND_ACK, then send COMMAND_LONG, then collect the ACK.
    """
    return await _probe_with_send(
        system, command, timeout_s,
        lambda: send_command_long(system, command, confirmation=confirmation, **send_kwargs),
    )


async def _probe_with_send(system, command_id, timeout_s, send_fn) -> dict | None:
    """
    Core pattern: subscribe → settle → send → collect ACK.

    The subscription is started as a background task before the send, ensuring
    the server has registered the subscriber before any ACK can arrive.
    """
    received: asyncio.Queue = asyncio.Queue()

    async def _collect() -> None:
        async for msg in system.mavlink_direct.message("COMMAND_ACK"):
            fields = json.loads(msg.fields_json)
            if int(fields.get("command", -1)) == command_id:
                await received.put(fields)
                return

    task = asyncio.create_task(_collect())
    await asyncio.sleep(_SUBSCRIPTION_SETTLE_S)  # let gRPC stream register on server

    await send_fn()

    try:
        return await asyncio.wait_for(received.get(), timeout=timeout_s)
    except asyncio.TimeoutError:
        log.warning(
            "No COMMAND_ACK received within %.1fs for cmd=%d — "
            "command may be executing without acknowledgement (spec violation) "
            "or truly unsupported; further testing required.",
            timeout_s, command_id,
        )
        return None
    finally:
        # Fire-and-forget cancel: do NOT await the task.  The _collect() coroutine
        # iterates a gRPC mavlink_direct stream which does not respond to asyncio
        # cancellation at its next yield — awaiting it would block indefinitely,
        # leaving the gRPC channel in a corrupted state for subsequent tests
        # (CLAUDE.md §4a, same pattern as _wait_for_connection).
        task.cancel()




async def probe_command_int_all_acks(
    system: System,
    command: int,
    frame: int = 6,
    window_s: float = 1.5,
    **send_kwargs,
) -> list[dict]:
    """
    Subscribe to COMMAND_ACK, send COMMAND_INT once, and collect every matching
    COMMAND_ACK received within window_s — not just the first.

    Unlike probe_command_int() (which returns as soon as the first matching ACK
    arrives, then cancels the subscription), this keeps listening for the full
    window so a duplicate ACK sent shortly after the first one is not missed.
    Used to verify the "exactly one terminal ACK per command" protocol
    invariant.  IN_PROGRESS ACKs are not exempt from the returned list — the
    caller decides how to classify them (a command may legitimately emit any
    number of IN_PROGRESS ACKs before its one terminal result).

    Returns the list of ACK field dicts in receipt order (may be empty).
    """
    received: list[dict] = []

    async def _collect() -> None:
        async for msg in system.mavlink_direct.message("COMMAND_ACK"):
            fields = json.loads(msg.fields_json)
            if int(fields.get("command", -1)) == command:
                received.append(fields)

    task = asyncio.create_task(_collect())
    await asyncio.sleep(_SUBSCRIPTION_SETTLE_S)  # let gRPC stream register on server

    await send_command_int(system, command, frame, **send_kwargs)

    await asyncio.sleep(window_s)
    # Fire-and-forget cancel: see CLAUDE.md §4a — do NOT await the task.
    task.cancel()
    return received


async def probe_command_long_all_acks(
    system: System,
    command: int,
    window_s: float = 1.5,
    confirmation: int = 0,
    **send_kwargs,
) -> list[dict]:
    """
    COMMAND_LONG equivalent of probe_command_int_all_acks() — see its
    docstring.  Collects every matching COMMAND_ACK received within window_s,
    not just the first.
    """
    received: list[dict] = []

    async def _collect() -> None:
        async for msg in system.mavlink_direct.message("COMMAND_ACK"):
            fields = json.loads(msg.fields_json)
            if int(fields.get("command", -1)) == command:
                received.append(fields)

    task = asyncio.create_task(_collect())
    await asyncio.sleep(_SUBSCRIPTION_SETTLE_S)  # let gRPC stream register on server

    await send_command_long(system, command, confirmation=confirmation, **send_kwargs)

    await asyncio.sleep(window_s)
    # Fire-and-forget cancel: see CLAUDE.md §4a — do NOT await the task.
    task.cancel()
    return received


def effective_ack(acks: list[dict]) -> dict | None:
    """
    Reduce a window of collected ACKs (from probe_command_int_all_acks() /
    probe_command_long_all_acks()) to the single "effective" one, preferring
    a non-UNSUPPORTED result when present.

    Some stacks have more than one internal handler racing to answer the same
    command (confirmed on PX4 for MAV_CMD_EXTERNAL_WIND_ESTIMATE — see
    external_wind_estimate/CLAUDE.md notes: a catch-all module answers
    UNSUPPORTED while the command's real handler answers ACCEPTED, and which
    one a naive "first ACK wins" listener sees is non-deterministic). Using
    the non-UNSUPPORTED result when one exists tests the command's real,
    intended behaviour rather than which racing module happened to answer
    first. Returns None if the list is empty (no ACK at all — UNKNOWN).
    """
    if not acks:
        return None
    from tests.mock_flight_stack import MAV_RESULT_UNSUPPORTED
    non_unsupported = [a for a in acks if int(a["result"]) != MAV_RESULT_UNSUPPORTED]
    return non_unsupported[0] if non_unsupported else acks[0]


async def probe_dual(
    system: System,
    command: int,
    *,
    param1: float = 0.0,
    param2: float = 0.0,
    param3: float = 0.0,
    param4: float = 0.0,
    long5: float = 0.0,
    long6: float = 0.0,
    long7: float = 0.0,
    int_x: int = 0,
    int_y: int = 0,
    int_z: float = 0.0,
    frame: int = 6,
    window_s: float = 1.5,
) -> tuple[dict | None, dict | None]:
    """
    Send the same logical command via COMMAND_INT, then COMMAND_LONG, each
    using the full-window multi-ACK collection (probe_command_*_all_acks())
    reduced to one effective_ack() — see its docstring for why that matters.

    param1-4 are identical fields in both message types and are passed
    through verbatim. MAVLink's COMMAND_INT has no param5/6/7 — it has x, y
    (int32) and z (float) instead, which correspond 1:1 to COMMAND_LONG's
    param5/6/7 for any command with hasLocation="true" (x/y carry lat/lon
    ×1e7, z carries altitude). For a command WITHOUT location, x/y/z still
    exist on the wire and must be given *some* value; long5/long6/long7 and
    int_x/int_y/int_z let the caller populate the "same" logical slot
    correctly-typed for each message: e.g. to test an undefined float slot
    with a non-NaN value, pass long5=1.0 for the COMMAND_LONG send and
    int_x=<some real-looking int> for the COMMAND_INT send (a real int is the
    x/y-appropriate equivalent of "a non-NaN float" — see
    tests/command/CLAUDE.md § Mandatory common tests for the full rationale
    and the NaN/INT32_MAX sentinel convention this mirrors).

    Returns (int_result, long_result) — each an effective_ack() dict, or None
    if that message type got no ACK at all (UNKNOWN).
    """
    int_acks = await probe_command_int_all_acks(
        system, command, frame=frame, window_s=window_s,
        param1=param1, param2=param2, param3=param3, param4=param4,
        x=int_x, y=int_y, z=int_z,
    )
    long_acks = await probe_command_long_all_acks(
        system, command, window_s=window_s,
        param1=param1, param2=param2, param3=param3, param4=param4,
        param5=long5, param6=long6, param7=long7,
    )
    return effective_ack(int_acks), effective_ack(long_acks)


async def await_command_ack(
    system: System,
    command_id: int,
    timeout_s: float = ACK_TIMEOUT_S,
) -> dict | None:
    """
    Wait for a COMMAND_ACK matching command_id on an already-established stream.

    NOTE: Only use this when the subscription is started BEFORE the command is
    sent (i.e. the caller already has an active message stream).  For new
    send+receive pairs, use probe_command_int() or probe_command_long() instead.

    Returns the fields dict on success, or None on timeout.
    """
    async def _inner() -> dict:
        async for msg in system.mavlink_direct.message("COMMAND_ACK"):
            fields = json.loads(msg.fields_json)
            if int(fields.get("command", -1)) == command_id:
                return fields

    try:
        return await asyncio.wait_for(_inner(), timeout=timeout_s)
    except asyncio.TimeoutError:
        log.warning(
            "No COMMAND_ACK received within %.1fs for cmd=%d — "
            "command may be executing without acknowledgement (spec violation) "
            "or truly unsupported; further testing required.",
            timeout_s, command_id,
        )
        return None


# ---------------------------------------------------------------------------
# Tier 1 command spec model
# ---------------------------------------------------------------------------
#
# Formalises what every tests/command/*/test_command.py file previously encoded
# by hand as prose in its module docstring plus scattered literal values: which
# of a MAV_CMD's 7 parameter slots are DEFINED (have a MAVLink meaning) vs
# UNDEFINED ("Empty" in the XML), and what a valid baseline send looks like.
# Drives Tier1CommandTestBase (below) and its pytest_generate_tests hook.
# ParamSpec itself lives in tests/param_spec.py — shared with
# tests/mission/conftest.py's Tier1MissionTestBase, since COMMAND_INT/LONG and
# MISSION_ITEM_INT share the same param1-4/x/y/z wire layout slot for slot.


@dataclass
class CommandSpec:
    """Everything Tier1CommandTestBase needs to test one MAV_CMD."""

    cmd_id: int
    name: str  # e.g. "EXTERNAL_WIND_ESTIMATE" — used in the log filename/labels
    baseline: dict  # probe_dual() kwargs for a valid baseline send
    params: list[ParamSpec]  # all 7 slots

    @property
    def undefined_params(self) -> list["ParamSpec"]:
        return [p for p in self.params if not p.defined]

    @property
    def defined_params(self) -> list["ParamSpec"]:
        return [p for p in self.params if p.defined]


def pytest_generate_tests(metafunc):
    """
    Dynamically parametrize the count-varying Tier 1 tests (undefined/defined
    param sentinel checks) from the test class's SPEC. A no-op for any test
    class without a SPEC (i.e. everything except Tier1CommandTestBase
    subclasses) — guarded so this hook cannot affect unrelated collection.
    """
    spec = getattr(metafunc.cls, "SPEC", None) if metafunc.cls is not None else None
    if spec is None:
        return
    if "undefined_param" in metafunc.fixturenames:
        params = spec.undefined_params
        metafunc.parametrize("undefined_param", params, ids=[f"param{p.slot}" for p in params])
    if "defined_param" in metafunc.fixturenames:
        params = spec.defined_params
        metafunc.parametrize("defined_param", params, ids=[f"param{p.slot}" for p in params])


# ---------------------------------------------------------------------------
# Tier 1 results — recorded by _check()/_record() into the shared report
# (tests/report.py), keyed by ("command", cmd_name) so results from
# different commands never mix, even though every migrated Test*Command
# class shares this code.
# ---------------------------------------------------------------------------


def _record(cls, request, outcome: str, description: str, result: int | None) -> None:
    """
    Record this test's outcome into the shared report (tests/report.py).
    Public-ish (some command test files import it directly, e.g.
    external_wind_estimate/test_command.py) — keep this signature stable.
    """
    report.record_tier1_result("command", cls.SPEC.name, request.node.name, outcome, description, result)


def _record_detail(cls, text: str) -> None:
    """Attach a supplementary multi-line block to the report."""
    report.record_tier1_detail("command", cls.SPEC.name, text)


def _check(cls, request, description: str, result: int | None, *, expect, xfail_reason: str | None = None) -> None:
    """
    Record this test's outcome into the shared report, then perform the
    actual pytest assertion/xfail.

    `expect`: predicate(result:int) -> bool, only called when result is not
    None. `xfail_reason`: if given and the predicate fails, xfail with this
    reason (a known, documented stack gap) instead of hard-failing.
    """
    if result is None:
        _record(cls, request, "UNKNOWN", description, None)
        return  # ambiguous no-ACK on at least one message type; already logged by _reduce_dual()
    if expect(result):
        _record(cls, request, "PASS", description, result)
        return
    if xfail_reason:
        _record(cls, request, "XFAIL", description, result)
        pytest.xfail(xfail_reason)
    _record(cls, request, "FAIL", description, result)
    pytest.fail(description)


def _reduce_dual(cmd_name: str, label: str, int_ack: dict | None, long_ack: dict | None) -> int | None:
    """
    Merge a probe_dual() result pair into a single reported outcome.

    Logs once if COMMAND_INT and COMMAND_LONG agree; logs both explicitly
    (WARNING) if they disagree — a real protocol inconsistency, not just
    noise, per CLAUDE.md § Mandatory common tests. Returns None if EITHER
    message type got no ACK at all (a partial UNKNOWN is itself worth
    surfacing rather than silently resolved to "whichever answered").
    """
    int_result = int(int_ack["result"]) if int_ack is not None else None
    long_result = int(long_ack["result"]) if long_ack is not None else None
    if int_result is None or long_result is None:
        log.warning(_FMT, cmd_name, label,
                    f"UNKNOWN on at least one message type: COMMAND_INT={int_result} COMMAND_LONG={long_result}")
        return None
    if int_result == long_result:
        log.info(_FMT, cmd_name, label, f"result={long_result} (COMMAND_INT and COMMAND_LONG agree)")
        return long_result
    log.warning(_FMT, cmd_name, label, f"INCONSISTENT: COMMAND_INT={int_result} COMMAND_LONG={long_result}")
    return long_result


@pytest.fixture(scope="class", autouse=True)
def _write_tier1_log(request):
    """
    Write this command's combined report (Tier 1, plus Tier 2 if it ran in
    the same pytest session — see tests/report.py) once, after every test in
    the class has run. A no-op for any test class that isn't a
    Tier1CommandTestBase subclass, or one where nothing actually ran (e.g.
    every test filtered out by -k).
    """
    yield
    cls = request.cls
    if cls is None or not hasattr(cls, "SPEC"):
        return
    spec = cls.SPEC
    if not report.has_tier1_results("command", spec.name):
        return
    report.write("command", spec.name, spec.cmd_id, request.config)


class Tier1CommandTestBase:
    """
    Shared Tier 1 (ACK-level) mandatory common tests — see CLAUDE.md
    § Mandatory common tests. Subclass and set `SPEC = CommandSpec(...)`; add
    the command's own bespoke per-parameter tests as ordinary methods
    alongside these. Every test sends via both COMMAND_INT and COMMAND_LONG
    (probe_dual()) and reports one merged result, per CLAUDE.md's dual-send
    rule for the mandatory checks.
    """

    SPEC: "CommandSpec"  # set by subclass

    def __init_subclass__(cls, **kwargs) -> None:
        super().__init_subclass__(**kwargs)
        # Fresh per-subclass state — NOT shared via the base class, since
        # multiple Test*Command classes (different commands) run in one
        # pytest session and must not mix "supported" caches.
        cls._supported = None
        # Register this command's identity + full param-slot list with the
        # shared report (tests/report.py) once per subclass — see
        # tests/mission/conftest.py's Tier1MissionTestBase for the mission-
        # protocol analogue of this same registration.
        spec = cls.SPEC
        report.declare_command("command", spec.name, spec.cmd_id)
        report.declare_params("command", spec.name, [f"{p.slot}_{p.label}" for p in spec.params])
        for p in spec.undefined_params:
            report.record_compat_fact("command", spec.name, f"{p.slot}_{p.label}", supported="not-applicable")

    async def _probe(self, system, **overrides) -> tuple[dict | None, dict | None]:
        """probe_dual() with this command's baseline defaults applied."""
        kw = dict(self.SPEC.baseline)
        kw.update(overrides)
        return await probe_dual(system, self.SPEC.cmd_id, window_s=_ACK_WINDOW_S, **kw)

    def _reduce(self, label: str, int_ack: dict | None, long_ack: dict | None) -> int | None:
        return _reduce_dual(self.SPEC.name, label, int_ack, long_ack)

    async def _ensure_supported(self, system, mock_stack) -> None:
        """Probe once per class; skip all subsequent tests if UNSUPPORTED."""
        cls = type(self)
        if cls._supported is None:
            int_ack, long_ack = await self._probe(system)
            result = self._reduce("support probe", int_ack, long_ack)
            cls._supported = (result != MAV_RESULT_UNSUPPORTED)
            # Unlike a mission item (stored for later, possibly-never-executed
            # use — see tests/mission/conftest.py's test_command_supported,
            # which deliberately does NOT record supported=True from upload
            # acceptance alone), a COMMAND_INT/LONG ACK is the stack acting on
            # the command now — MAV_RESULT_UNSUPPORTED(3) vs. anything else is
            # real, immediate evidence either way. No `notes` on the False
            # branch — the only reason it's reached is MAV_RESULT_UNSUPPORTED,
            # which `supported: false` already says; a note would just restate it.
            report.record_command_fact("command", self.SPEC.name, supported=cls._supported)
        if not cls._supported:
            pytest.skip(f"{self.SPEC.name} (cmd={self.SPEC.cmd_id}) is UNSUPPORTED on this platform — test not run")

    # -------------------------------------------------------------------
    # Group A — mandatory common tests 1/2/3/6 (CLAUDE.md § Mandatory common tests)
    # -------------------------------------------------------------------

    async def test_command_ack_received(self, gcs_system_cls, mock_stack_cls, request):
        """ACKs (via both COMMAND_INT and COMMAND_LONG) for a baseline, valid send."""
        int_ack, long_ack = await self._probe(gcs_system_cls)
        description = "ACKs (via both COMMAND_INT and COMMAND_LONG) for a baseline, valid send"
        if int_ack is None or long_ack is None:
            _record(type(self), request, "FAIL", description, None)
            assert int_ack is not None, (
                f"No COMMAND_ACK received via COMMAND_INT within {ACK_TIMEOUT_S:.1f}s — "
                "every command must be acknowledged per spec"
            )
            assert long_ack is not None, (
                f"No COMMAND_ACK received via COMMAND_LONG within {ACK_TIMEOUT_S:.1f}s — "
                "every command must be acknowledged per spec"
            )
        _record(type(self), request, "PASS", description, int(long_ack["result"]))
        log.info(_FMT, self.SPEC.name, "ACK received",
                 f"COMMAND_INT={int(int_ack['result'])} COMMAND_LONG={int(long_ack['result'])}")

    async def test_command_supported(self, gcs_system_cls, mock_stack_cls, request):
        """Not UNSUPPORTED for a baseline, valid send."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await self._probe(gcs_system_cls)
        result = self._reduce("baseline", int_ack, long_ack)
        _check(type(self), request, "Not UNSUPPORTED for a baseline, valid send", result,
               expect=lambda r: r != MAV_RESULT_UNSUPPORTED)

    async def test_exactly_one_ack(self, gcs_system_cls, mock_stack_cls, request):
        """Exactly one terminal COMMAND_ACK per send, via each message type."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        description = "Exactly one terminal COMMAND_ACK per send, via each message type"
        spec = self.SPEC
        kw = spec.baseline
        int_acks = await probe_command_int_all_acks(
            gcs_system_cls, spec.cmd_id, window_s=_ACK_WINDOW_S,
            frame=kw.get("frame", 6),
            param1=kw.get("param1", 0.0), param2=kw.get("param2", 0.0),
            param3=kw.get("param3", 0.0), param4=kw.get("param4", 0.0),
            x=kw.get("int_x", 0), y=kw.get("int_y", 0), z=kw.get("int_z", 0.0),
        )
        long_acks = await probe_command_long_all_acks(
            gcs_system_cls, spec.cmd_id, window_s=_ACK_WINDOW_S,
            param1=kw.get("param1", 0.0), param2=kw.get("param2", 0.0),
            param3=kw.get("param3", 0.0), param4=kw.get("param4", 0.0),
            param5=kw.get("long5", 0.0), param6=kw.get("long6", 0.0), param7=kw.get("long7", 0.0),
        )

        # Gather both counts BEFORE deciding pass/fail/xfail — pytest.xfail()
        # raises immediately, so deciding inside a loop over message types
        # would skip checking whichever type comes second.
        counts: dict[str, tuple[int, list[int]]] = {}
        for label, acks in (("COMMAND_INT", int_acks), ("COMMAND_LONG", long_acks)):
            assert len(acks) >= 1, f"No COMMAND_ACK received via {label}"
            results = [int(a["result"]) for a in acks]
            log.info(_FMT, spec.name, f"ACK count ({label})", f"n={len(acks)} results={results}")
            counts[label] = (len(acks), results)

        offenders = {label: rs for label, (n, rs) in counts.items() if n > 1}
        if offenders:
            if mock_stack_cls is not None:
                _record(type(self), request, "FAIL", description, None)
                pytest.fail(f"Got more than one ACK per send in mock mode: {offenders}")
            _record(type(self), request, "XFAIL", description, None)
            pytest.xfail(
                f"Got more than one COMMAND_ACK for a single send ({offenders}) — see this "
                "command's own module docstring / CLAUDE.md for any documented stack-specific cause"
            )
        for n, _ in counts.values():
            assert n == 1
        _record(type(self), request, "PASS", description, None)

    async def test_frame_validation_survey(self, gcs_system_cls, mock_stack_cls, request):
        """
        Frame validation survey: does the stack ever return
        MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME(9) for any MAV_FRAME value?

        COMMAND_INT only — COMMAND_LONG has no `frame` field. Observational:
        seeing UNSUPPORTED_MAV_FRAME(9) for at least one frame is positive
        evidence of frame validation (PASS); seeing it for none is
        INCONCLUSIVE, never a failure (see CLAUDE.md item 6).
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        description = "Frame validation survey: any MAV_FRAME value returns MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME(9)"
        spec = self.SPEC
        kw = spec.baseline
        per_frame: list[tuple[int, str, int | None]] = []
        for frame_id, frame_name in MAV_FRAME_CATALOGUE:
            acks = await probe_command_int_all_acks(
                gcs_system_cls, spec.cmd_id, frame=frame_id, window_s=_ACK_WINDOW_S,
                param1=kw.get("param1", 0.0), param2=kw.get("param2", 0.0),
                param3=kw.get("param3", 0.0), param4=kw.get("param4", 0.0),
                x=kw.get("int_x", 0), y=kw.get("int_y", 0), z=kw.get("int_z", 0.0),
            )
            ack = effective_ack(acks)
            result = int(ack["result"]) if ack is not None else None
            per_frame.append((frame_id, frame_name, result))

        all_acked = all(result is not None for _, _, result in per_frame)
        hits = [(fid, fname) for fid, fname, result in per_frame
                if result == MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME]

        if not all_acked:
            table_lines = [
                f"  frame={fid:>2} ({fname}): {'UNKNOWN (no ACK)' if result is None else result}"
                for fid, fname, result in per_frame
            ]
            block = (
                f"Frame validation survey ({spec.name}, cmd={spec.cmd_id}) — full breakdown "
                f"(not all frames ACKed):\n" + "\n".join(table_lines)
            )
        else:
            block = f"Frame validation survey ({spec.name}, cmd={spec.cmd_id}): all {len(per_frame)} frames ACKed."
        if hits:
            hit_desc = ", ".join(f"frame={fid} ({fname})" for fid, fname in hits)
            block += f"\nUNSUPPORTED_MAV_FRAME(9) returned for: {hit_desc} — frame validation confirmed."
        else:
            block += (
                "\nNo frame returned UNSUPPORTED_MAV_FRAME(9) — inconclusive; this does NOT mean "
                "frame is unvalidated, only that none of the tested frames triggered a rejection."
            )
        _record_detail(type(self), block)
        log.info(_FMT, spec.name, "frame validation survey", block)

        if hits:
            _record(type(self), request, "PASS", description, MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME)
        else:
            _record(type(self), request, "INCONCLUSIVE", description, None)

    # -------------------------------------------------------------------
    # Group B — undefined params: sentinel accepted, non-sentinel rejected
    # (mandatory common test 4). Parametrized per-command via SPEC.undefined_params
    # (see pytest_generate_tests above).
    # -------------------------------------------------------------------

    async def test_undefined_param_sentinel_accepted(self, gcs_system_cls, mock_stack_cls, request, undefined_param):
        """Accepted when an undefined param is sent as its own sentinel."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        p = undefined_param
        int_ack, long_ack = await self._probe(gcs_system_cls, **p.sentinel_kwargs)
        result = self._reduce(f"param{p.slot} ({p.label}) = sentinel", int_ack, long_ack)
        description = f"Accepted when param{p.slot} ({p.label}) is sent as its own sentinel (undefined param)"
        accepted = result is not None and result not in (MAV_RESULT_UNSUPPORTED, MAV_RESULT_DENIED)
        report.record_compat_fact("command", self.SPEC.name, f"{p.slot}_{p.label}", accept_nan_or_int32max=accepted)
        _check(type(self), request, description, result,
               expect=lambda r: r not in (MAV_RESULT_UNSUPPORTED, MAV_RESULT_DENIED))

    async def test_undefined_param_nonsentinel_rejected(self, gcs_system_cls, mock_stack_cls, request, undefined_param):
        """Rejected when an undefined param is sent a real (non-sentinel) value."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        p = undefined_param
        int_ack, long_ack = await self._probe(gcs_system_cls, **p.nonsentinel_kwargs)
        result = self._reduce(f"param{p.slot} ({p.label}) = non-sentinel", int_ack, long_ack)
        description = f"Rejected when param{p.slot} ({p.label}) is sent a real (non-sentinel) value (undefined param)"
        report.record_compat_fact(
            "command", self.SPEC.name, f"{p.slot}_{p.label}",
            nacks_on_non_sentinel_value=(result == MAV_RESULT_DENIED),
        )
        xfail_reason = p.reject_xfail_reason or (
            f"Stack returned {result} for undefined param{p.slot}; expected DENIED — no known "
            "stack validates parameters with no MAVLink definition (spec gap)"
        )
        _check(type(self), request, description, result, expect=lambda r: r == MAV_RESULT_DENIED,
               xfail_reason=xfail_reason)

    # -------------------------------------------------------------------
    # Group C — defined (used) params tolerate their sentinel (mandatory
    # common test 5), except a "deny_required" param (mandatory field with
    # no sentinel fallback — e.g. DO_SET_GLOBAL_ORIGIN's lat/lon/altitude),
    # where the expectation flips: DENIED is the correct, passing result.
    # -------------------------------------------------------------------

    async def test_defined_param_sentinel_tolerated(self, gcs_system_cls, mock_stack_cls, request, defined_param):
        """Not denied (or denied, for a documented mandatory-field exemption) when a defined param is sent its sentinel."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        p = defined_param
        int_ack, long_ack = await self._probe(gcs_system_cls, **p.sentinel_kwargs)
        result = self._reduce(f"param{p.slot} ({p.label}, defined) = sentinel", int_ack, long_ack)
        report.record_compat_fact(
            "command", self.SPEC.name, f"{p.slot}_{p.label}",
            accept_nan_or_int32max=(result is not None and result != MAV_RESULT_DENIED),
        )
        if p.sentinel_policy == "deny_required":
            description = (
                f"Denied when param{p.slot} ({p.label}) is sent its sentinel "
                "(mandatory field, no sentinel fallback)"
            )
            _check(type(self), request, description, result, expect=lambda r: r == MAV_RESULT_DENIED)
        else:
            description = f"Not denied when param{p.slot} ({p.label}, defined) is sent its sentinel"
            _check(type(self), request, description, result, expect=lambda r: r != MAV_RESULT_DENIED)


# ---------------------------------------------------------------------------
# Retry helpers
# ---------------------------------------------------------------------------


async def send_command_int_with_retry(
    system: System,
    command: int,
    frame: int = 6,
    *,
    max_retries: int = 5,
    ack_timeout_s: float = 1.5,
    **kwargs,
) -> dict | None:
    """
    Send COMMAND_INT with the MAVLink protocol retry loop.

    Sends the command, waits up to ack_timeout_s for a COMMAND_ACK.  If no
    ACK arrives, retransmits (COMMAND_INT has no confirmation field — retries
    are identical).  Repeats up to max_retries times.

    Returns the COMMAND_ACK fields dict on success, or None if no ACK after
    all retries.
    """
    deadline = asyncio.get_event_loop().time() + RETRY_TIMEOUT_S
    for attempt in range(max_retries + 1):
        if asyncio.get_event_loop().time() > deadline:
            break
        ack = await probe_command_int(
            system, command, frame, timeout_s=ack_timeout_s, **kwargs
        )
        if ack is not None:
            return ack
        log.debug("COMMAND_INT cmd=%d attempt %d/%d: no ACK, retrying", command, attempt + 1, max_retries + 1)
    log.warning("COMMAND_INT cmd=%d: no ACK after %d attempts", command, max_retries + 1)
    return None


async def send_command_long_with_retry(
    system: System,
    command: int,
    *,
    max_retries: int = 5,
    ack_timeout_s: float = 1.5,
    **kwargs,
) -> dict | None:
    """
    Send COMMAND_LONG with the MAVLink protocol retry loop.

    The confirmation field increments on each retry (0, 1, 2, ...) per spec.
    Returns the COMMAND_ACK fields dict on success, or None after all retries.
    """
    deadline = asyncio.get_event_loop().time() + RETRY_TIMEOUT_S
    for attempt in range(max_retries + 1):
        if asyncio.get_event_loop().time() > deadline:
            break
        ack = await probe_command_long(
            system, command, timeout_s=ack_timeout_s, confirmation=attempt, **kwargs
        )
        if ack is not None:
            return ack
        log.debug("COMMAND_LONG cmd=%d attempt %d/%d (confirmation=%d): no ACK, retrying",
                  command, attempt + 1, max_retries + 1, attempt)
    log.warning("COMMAND_LONG cmd=%d: no ACK after %d attempts", command, max_retries + 1)
    return None


# ---------------------------------------------------------------------------
# Class-scoped fixtures
# ---------------------------------------------------------------------------


@pytest_asyncio.fixture(scope="class", loop_scope="class")
async def mock_stack_cls(request) -> MockFlightStack | None:
    """Class-scoped MockFlightStack with COMMAND_INT support.  No-op in standalone."""
    drone_address = request.config.getoption("--drone-address")
    if drone_address is not None:
        yield None
        return

    system = System(mavsdk_server_address="localhost", port=DRONE_GRPC_PORT)
    await system.connect()

    stack = MockFlightStack()
    task = asyncio.create_task(stack.run(system))
    await asyncio.sleep(0.5)
    yield stack
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass


@pytest_asyncio.fixture(scope="class", loop_scope="class")
async def gcs_system_cls(gcs_mavsdk_server, mock_stack_cls, request) -> System:
    """Class-scoped GCS System for command protocol tests."""
    timeout_s = int(request.config.getoption("--connection-timeout"))
    system = System(mavsdk_server_address="localhost", port=gcs_mavsdk_server)
    await system.connect()
    await _wait_for_connection(system, timeout_s)
    if request.config.getoption("--drone-address") is not None:
        await asyncio.sleep(3.0)
    yield system
