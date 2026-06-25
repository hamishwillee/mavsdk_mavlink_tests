"""
Fixtures for DO_SET_GLOBAL_ORIGIN tests.

Uses fresh fixture names (mock_stack_origin_cls / gcs_system_origin_cls) to
avoid overriding the parent conftest's mock_stack_cls / gcs_system_cls — overriding
class-scoped async fixtures causes event-loop conflicts in pytest-asyncio.

Two fixture pairs:
  origin: MockFlightStack with require_valid_location_cmds + GPS_GLOBAL_ORIGIN emit
  nack:   same but with command_results={611: DENIED} to verify no-emit-on-NACK
"""

import asyncio

import pytest_asyncio
from mavsdk import System

from tests.conftest import DRONE_GRPC_PORT, _wait_for_connection
from tests.mock_flight_stack import MAV_RESULT_DENIED, MockFlightStack

_CMD_ID = 611  # MAV_CMD_DO_SET_GLOBAL_ORIGIN

# ---------------------------------------------------------------------------
# Origin fixtures (used by TestDoSetGlobalOriginCommand)
# ---------------------------------------------------------------------------


@pytest_asyncio.fixture(scope="class", loop_scope="class")
async def mock_stack_origin_cls(request) -> MockFlightStack | None:
    """MockFlightStack with require_valid_location_cmds and GPS_GLOBAL_ORIGIN emission."""
    if request.config.getoption("--drone-address") is not None:
        yield None
        return

    system = System(mavsdk_server_address="localhost", port=DRONE_GRPC_PORT)
    await system.connect()
    stack = MockFlightStack(
        require_valid_location_cmds={_CMD_ID},
        emit_gps_global_origin=True,
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
async def gcs_system_origin_cls(gcs_mavsdk_server, mock_stack_origin_cls, request) -> System:
    """Class-scoped GCS System paired with mock_stack_origin_cls."""
    timeout_s = int(request.config.getoption("--connection-timeout"))
    system = System(mavsdk_server_address="localhost", port=gcs_mavsdk_server)
    await system.connect()
    await _wait_for_connection(system, timeout_s)
    if request.config.getoption("--drone-address") is not None:
        await asyncio.sleep(3.0)
    yield system


# ---------------------------------------------------------------------------
# Nack fixtures (used by TestDoSetGlobalOriginNackBehaviour)
# ---------------------------------------------------------------------------


@pytest_asyncio.fixture(scope="class", loop_scope="class")
async def mock_stack_nack_cls(request) -> MockFlightStack | None:
    """MockFlightStack configured to DENY all DO_SET_GLOBAL_ORIGIN commands."""
    if request.config.getoption("--drone-address") is not None:
        yield None
        return

    system = System(mavsdk_server_address="localhost", port=DRONE_GRPC_PORT)
    await system.connect()
    stack = MockFlightStack(
        require_valid_location_cmds={_CMD_ID},
        emit_gps_global_origin=True,
        command_results={_CMD_ID: MAV_RESULT_DENIED},
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
async def gcs_system_nack_cls(gcs_mavsdk_server, mock_stack_nack_cls, request) -> System:
    """Class-scoped GCS System paired with mock_stack_nack_cls."""
    timeout_s = int(request.config.getoption("--connection-timeout"))
    system = System(mavsdk_server_address="localhost", port=gcs_mavsdk_server)
    await system.connect()
    await _wait_for_connection(system, timeout_s)
    if request.config.getoption("--drone-address") is not None:
        await asyncio.sleep(3.0)
    yield system
