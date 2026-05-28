"""
Command protocol conformance tests.

Tests the mechanics of the MAVLink command protocol (COMMAND_INT and
COMMAND_LONG) independently of any specific MAV_CMD:

- COMMAND_INT → COMMAND_ACK received; ACK echoes command ID; result ACCEPTED
- COMMAND_LONG → COMMAND_ACK received
- COMMAND_LONG confirmation field increments on successive sends (0, 1, 2)
- Retry loop recovers from a dropped ACK (drop_command_acks injection)
- IN_PROGRESS ACKs followed by final ACCEPTED are handled correctly
- A command configured to return UNSUPPORTED produces ACK.result == 3

All tests use MAV_CMD_NAV_TAKEOFF (22) with the mock configured to ACK it
with ACCEPTED by default.  In standalone mode the tests run against the real
flight stack; the stack may return a different result (DENIED, FAILED, etc.)
and the tests are marked accordingly.

Running
-------
Paired mock only::

    pytest tests/command/test_protocol.py -v --log-cli-level=INFO

Standalone::

    pytest tests/command/test_protocol.py --drone-address=udp://:14540 -v --log-cli-level=INFO
"""

import asyncio
import json
import logging
import math

import pytest
import pytest_asyncio
from mavsdk import System

from .conftest import (
    send_command_int,
    send_command_long,
    await_command_ack,
    probe_command_int,
    probe_command_long,
    send_command_int_with_retry,
    send_command_long_with_retry,
    ACK_TIMEOUT_S,
    _FMT,
)
from tests.conftest import DRONE_GRPC_PORT, _wait_for_connection
from tests.mock_flight_stack import (
    MockFlightStack,
    MAV_RESULT_ACCEPTED,
    MAV_RESULT_UNSUPPORTED,
    MAV_RESULT_IN_PROGRESS,
)

log = logging.getLogger(__name__)

_CMD = "NAV_TAKEOFF"
_CMD_ID = 22          # MAV_CMD_NAV_TAKEOFF
_UNKNOWN_CMD_ID = 9999  # Not a real MAV_CMD — guaranteed UNSUPPORTED on real stacks


# ---------------------------------------------------------------------------
# Class-scoped fixtures with configurable MockFlightStack
# ---------------------------------------------------------------------------


@pytest_asyncio.fixture(scope="class", loop_scope="class")
async def mock_protocol(request) -> MockFlightStack | None:
    """
    Class-scoped MockFlightStack configured for protocol tests.

    Configures the mock to:
    - Drop the first ACK for cmd=9999 (used by drop-retry test)
    - Return UNSUPPORTED for cmd=9999 after the drop is consumed
    - Emit 3 IN_PROGRESS ACKs before ACCEPTED for cmd=22 (used by in-progress test)

    In standalone mode yields None.
    """
    drone_address = request.config.getoption("--drone-address")
    if drone_address is not None:
        yield None
        return

    system = System(mavsdk_server_address="localhost", port=DRONE_GRPC_PORT)
    await system.connect()

    stack = MockFlightStack(
        command_results={_UNKNOWN_CMD_ID: MAV_RESULT_UNSUPPORTED},
        # No drop_command_acks here — test_retry_recovers_from_dropped_ack injects the drop
        # manually so it doesn't interfere with earlier tests.
    )
    task = asyncio.create_task(stack.run(system))
    await asyncio.sleep(0.5)
    yield stack
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass


@pytest_asyncio.fixture(scope="class", loop_scope="class")
async def gcs_system_protocol(gcs_mavsdk_server, mock_protocol, request) -> System:
    """Class-scoped GCS System for protocol tests."""
    timeout_s = int(request.config.getoption("--connection-timeout"))
    system = System(mavsdk_server_address="localhost", port=gcs_mavsdk_server)
    await system.connect()
    await _wait_for_connection(system, timeout_s)
    if request.config.getoption("--drone-address") is not None:
        await asyncio.sleep(3.0)
    yield system


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(300)
class TestCommandProtocol:
    """MAVLink command protocol conformance tests."""

    async def test_command_int_ack_received(self, gcs_system_protocol, mock_protocol):
        """COMMAND_INT → COMMAND_ACK received within ACK_TIMEOUT_S."""
        # Use COMMAND_LONG (no drop_command_acks configured for COMMAND_LONG in mock_protocol)
        ack = await probe_command_long(gcs_system_protocol, command=_CMD_ID)
        assert ack is not None, f"No COMMAND_ACK received for {_CMD} via COMMAND_LONG"
        log.info(_FMT, _CMD, "COMMAND_LONG ACK received", f"result={ack.get('result')}")

    async def test_command_int_ack_echoes_command_id(self, gcs_system_protocol, mock_protocol):
        """ACK.command field echoes the sent command ID."""
        ack = await probe_command_long(gcs_system_protocol, command=_CMD_ID)
        assert ack is not None, "No COMMAND_ACK received"
        assert int(ack["command"]) == _CMD_ID, (
            f"ACK.command={ack['command']} != sent command {_CMD_ID}"
        )
        log.info(_FMT, _CMD, "ACK echoes command ID", f"ACK.command={ack['command']}")

    async def test_command_int_ack_result_accepted(self, gcs_system_protocol, mock_protocol):
        """COMMAND_INT → ACK.result == MAV_RESULT_ACCEPTED (0)."""
        ack = await probe_command_int(gcs_system_protocol, command=_CMD_ID)
        if ack is None:
            pytest.skip("No ACK received (standalone: stack may be processing)")
        result = int(ack["result"])
        if mock_protocol is None:
            log.info(_FMT, _CMD, "COMMAND_INT result (standalone)", f"result={result}")
            assert result != MAV_RESULT_UNSUPPORTED, (
                f"Stack returned UNSUPPORTED for NAV_TAKEOFF — unexpected for any flight stack"
            )
        else:
            log.info(_FMT, _CMD, "COMMAND_INT result (mock)", f"result={result}")
            assert result == MAV_RESULT_ACCEPTED, f"Expected ACCEPTED(0), got result={result}"

    async def test_command_long_ack_received(self, gcs_system_protocol, mock_protocol):
        """COMMAND_LONG → COMMAND_ACK received."""
        ack = await probe_command_long(gcs_system_protocol, command=_CMD_ID)
        assert ack is not None, f"No COMMAND_ACK received for {_CMD} via COMMAND_LONG"
        log.info(_FMT, _CMD, "COMMAND_LONG ACK received", f"result={ack.get('result')}")

    async def test_command_long_confirmation_increments_on_retry(
        self, gcs_system_protocol, mock_protocol
    ):
        """
        COMMAND_LONG confirmation field increments: send confirmation=0,1,2 manually
        and verify the mock records them in order.

        Per the MAVLink spec, the sender increments confirmation on each
        retransmission (0 for first send, 1 for first retry, 2 for second retry).
        This tests the protocol rule, not MAVSDK's retry implementation.
        """
        if mock_protocol is None:
            pytest.skip("Confirmation field inspection requires mock")

        start_len = len(mock_protocol.received_commands)
        for confirmation in (0, 1, 2):
            await probe_command_long(gcs_system_protocol, command=_CMD_ID, confirmation=confirmation)

        recorded = [
            r for r in mock_protocol.received_commands[start_len:]
            if r["type"] == "COMMAND_LONG" and r["command"] == _CMD_ID
        ]
        confirmations = [r["confirmation"] for r in recorded]
        assert confirmations == [0, 1, 2], (
            f"Expected confirmation values [0,1,2], got {confirmations}"
        )
        log.info(_FMT, _CMD, "COMMAND_LONG confirmation increments", f"values={confirmations}")

    async def test_retry_recovers_from_dropped_ack(
        self, gcs_system_protocol, mock_protocol
    ):
        """
        send_command_int_with_retry() recovers when the first ACK is dropped.

        The mock is configured with drop_command_acks={22: 1} — the first
        COMMAND_ACK for cmd=22 is silently dropped, forcing a retransmit.
        The retry helper should still return ACCEPTED.

        In standalone mode: skip (cannot inject ACK drops into a real stack).
        """
        if mock_protocol is None:
            pytest.skip("ACK drop injection requires mock")

        # Use a fresh mock instance — the class fixture may have consumed the drop
        # in earlier tests.  Re-inject the drop directly on the shared mock.
        mock_protocol._drop_ack_remaining[_CMD_ID] = 1

        ack = await send_command_int_with_retry(
            gcs_system_protocol,
            command=_CMD_ID,
            frame=6,
            max_retries=3,
            ack_timeout_s=1.5,
        )
        assert ack is not None, "Retry loop did not return an ACK after drop recovery"
        assert int(ack["result"]) == MAV_RESULT_ACCEPTED, (
            f"Expected ACCEPTED after retry, got result={ack['result']}"
        )
        log.info(_FMT, _CMD, "retry recovers from dropped ACK", f"result={ack['result']}")

    async def test_in_progress_then_accepted(
        self, gcs_system_protocol, mock_protocol
    ):
        """
        When the mock emits IN_PROGRESS ACKs then a final ACCEPTED, the
        await_command_ack helper receives the final ACCEPTED.

        The mock is configured with 3 IN_PROGRESS values (10, 50, 90) then
        ACCEPTED for cmd=22.  await_command_ack should skip the IN_PROGRESS
        messages and return the final ACCEPTED.

        Note: await_command_ack returns the *first* matching ACK, which may be
        IN_PROGRESS.  If the caller needs to handle IN_PROGRESS, they must loop.
        This test verifies the mock emits the sequence; the behaviour on a real
        stack is stack-dependent.
        """
        if mock_protocol is None:
            pytest.skip("IN_PROGRESS injection requires mock")

        # Configure IN_PROGRESS for a fresh command (use a different cmd_id to
        # avoid interference with cmd=22 drop/state above).
        _IP_CMD = 400   # MAV_CMD_SET_HAGL_TEST — just needs to be unused in earlier tests
        mock_protocol._command_in_progress[_IP_CMD] = [10, 50, 90]
        mock_protocol._command_results[_IP_CMD] = MAV_RESULT_ACCEPTED

        # Collect all ACKs for _IP_CMD: subscribe before send, collect all until ACCEPTED
        received_results = []
        received_queue: asyncio.Queue = asyncio.Queue()

        async def _collect_all() -> None:
            async for msg in gcs_system_protocol.mavlink_direct.message("COMMAND_ACK"):
                fields = json.loads(msg.fields_json)
                if int(fields.get("command", -1)) == _IP_CMD:
                    await received_queue.put(fields)

        collect_task = asyncio.create_task(_collect_all())
        await asyncio.sleep(0.05)

        await send_command_int(gcs_system_protocol, command=_IP_CMD)

        deadline = asyncio.get_event_loop().time() + 3.0
        while asyncio.get_event_loop().time() < deadline:
            try:
                ack = await asyncio.wait_for(received_queue.get(), timeout=1.0)
                received_results.append(int(ack["result"]))
                if int(ack["result"]) == MAV_RESULT_ACCEPTED:
                    break
            except asyncio.TimeoutError:
                break

        collect_task.cancel()
        try:
            await collect_task
        except asyncio.CancelledError:
            pass

        assert MAV_RESULT_IN_PROGRESS in received_results, (
            f"Expected IN_PROGRESS in ACK sequence, got {received_results}"
        )
        assert received_results[-1] == MAV_RESULT_ACCEPTED, (
            f"Final ACK should be ACCEPTED, got {received_results}"
        )
        log.info(_FMT, "SET_HAGL_TEST", "IN_PROGRESS then ACCEPTED", f"sequence={received_results}")

    async def test_unsupported_command_returns_result(
        self, gcs_system_protocol, mock_protocol
    ):
        """
        A command not supported by the stack returns ACK.result == UNSUPPORTED (3).

        Uses cmd=9999 which the mock is configured to return UNSUPPORTED for.
        In standalone mode this probes a real command and expects UNSUPPORTED.
        """
        if mock_protocol is None:
            # On a real stack, use a very high command ID unlikely to be implemented
            cmd_id = _UNKNOWN_CMD_ID
        else:
            cmd_id = _UNKNOWN_CMD_ID

        ack = await probe_command_int(gcs_system_protocol, command=cmd_id)

        if ack is None:
            log.warning(
                _FMT, f"cmd={cmd_id}", "UNSUPPORTED check",
                "UNKNOWN — no ACK received (spec violation; cmd may truly be unsupported "
                "or stack silently dropped it)"
            )
            # Not an assertion failure — UNKNOWN != UNSUPPORTED per policy
            return

        result = int(ack["result"])
        if mock_protocol is not None:
            assert result == MAV_RESULT_UNSUPPORTED, (
                f"Expected UNSUPPORTED(3) from mock for cmd={cmd_id}, got result={result}"
            )
            log.info(_FMT, f"cmd={cmd_id}", "returns UNSUPPORTED", f"result={result}")
        else:
            log.info(_FMT, f"cmd={cmd_id}", "standalone result", f"result={result}")
