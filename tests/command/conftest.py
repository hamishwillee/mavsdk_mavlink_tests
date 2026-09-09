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
from pathlib import Path

import pytest
import pytest_asyncio
from mavsdk import System
from mavsdk.mavlink_direct import MavlinkMessage

from tests.conftest import DRONE_GRPC_PORT, _wait_for_connection
from tests.mock_flight_stack import MockFlightStack

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

ACK_TIMEOUT_S = 5.0    # per-attempt timeout waiting for COMMAND_ACK
RETRY_TIMEOUT_S = 30.0  # total timeout for the retry loop

_GCS_SYSID = 255
_GCS_COMPID = 1
_DRONE_SYSID = 1
_DRONE_COMPID = 1

_FMT = "%-14s | %-44s | %s"

# Sentinel: "use current position" for lat/lon in COMMAND_INT
INT32_MAX = 0x7FFF_FFFF

# Canonical MAV_FRAME catalogue (0-21) for frame-validation surveys — see
# CLAUDE.md § Mandatory common tests item 6. Mirrors
# tests/mission/test_frame_types.py's frame list (kept as an independent copy
# since the two live in different subpackages and are used for different
# protocols — mission items vs COMMAND_INT).
MAV_FRAME_CATALOGUE: list[tuple[int, str]] = [
    (0, "MAV_FRAME_GLOBAL"),
    (1, "MAV_FRAME_LOCAL_NED"),
    (2, "MAV_FRAME_MISSION"),
    (3, "MAV_FRAME_GLOBAL_RELATIVE_ALT"),
    (4, "MAV_FRAME_LOCAL_ENU"),
    (5, "MAV_FRAME_GLOBAL_INT"),
    (6, "MAV_FRAME_GLOBAL_RELATIVE_ALT_INT"),
    (7, "MAV_FRAME_LOCAL_OFFSET_NED"),
    (8, "MAV_FRAME_BODY_NED"),
    (9, "MAV_FRAME_BODY_OFFSET_NED"),
    (10, "MAV_FRAME_GLOBAL_TERRAIN_ALT"),
    (11, "MAV_FRAME_GLOBAL_TERRAIN_ALT_INT"),
    (12, "MAV_FRAME_BODY_FRD"),
    (13, "MAV_FRAME_RESERVED_13"),
    (14, "MAV_FRAME_RESERVED_14"),
    (15, "MAV_FRAME_RESERVED_15"),
    (16, "MAV_FRAME_RESERVED_16"),
    (17, "MAV_FRAME_RESERVED_17"),
    (18, "MAV_FRAME_RESERVED_18"),
    (19, "MAV_FRAME_RESERVED_19"),
    (20, "MAV_FRAME_LOCAL_FRD"),
    (21, "MAV_FRAME_LOCAL_FLU"),
]

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
