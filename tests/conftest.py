"""
Shared fixtures for all tests.

Event-loop strategy
-------------------
pytest-asyncio gives each test function its own asyncio event loop.  gRPC
channels (used by MAVSDK) are tied to the event loop that created them, so a
session-scoped *async* fixture that creates gRPC channels cannot be safely
shared across test functions.

Solution: start ``mavsdk_server`` once as a *synchronous* session fixture
(no event-loop ownership), then create a fresh ``System`` object per test.
Each System connects to the already-running server over gRPC, establishing
channels in the test's own event loop.  The MAVLink connection to the drone
is maintained by mavsdk_server for the whole session.

Connection strategy
-------------------
* ``--drone-address`` supplied → standalone mode: GCS mavsdk_server connects
  to that address (e.g. a PX4 SITL).  All client tests run against the real
  flight stack.
* ``--drone-address`` omitted  → paired mode: client tests run against
  MockFlightStack over loopback.  Server tests also run (they always use the
  paired loopback independently of this flag).

Fixture sets
------------
``gcs_mavsdk_server`` / ``gcs_system`` / ``mock_stack``
    Mode-aware GCS for client tests.
    Standalone: ``gcs_mavsdk_server`` starts a dedicated server pointing to
    ``--drone-address``; ``mock_stack`` is a no-op.
    Paired: ``gcs_mavsdk_server`` reuses ``paired_gcs_server``; ``mock_stack``
    starts MockFlightStack against the paired drone mavsdk_server.

``paired_gcs_server`` / ``paired_drone_server``
``paired_gcs_system`` / ``paired_drone_system``
    Loopback-only pair — always started, regardless of ``--drone-address``.
    Used by server tests and ``TestDeprecatedMessageHandling``.  Port 14560
    is used deliberately (not 14540) to avoid interference from PX4 SITL.

Ports
-----
  GCS_GRPC_PORT         = 50051  (gRPC — standalone GCS mavsdk_server)
  PAIRED_GCS_GRPC_PORT  = 50053  (gRPC — paired GCS mavsdk_server)
  DRONE_GRPC_PORT       = 50052  (gRPC — paired drone mavsdk_server)
  GCS_MAVLINK_PORT      = 14560  (MAVLink UDP — paired loopback;
                                  deliberately NOT 14540 to avoid interference
                                  from a concurrently running PX4 SITL)

Identity
--------
  GCS:   sysid=255, compid=1 — GCS must have compid=1 (autopilot-class) so the
         drone's mavsdk_server fires "System discovered" and starts its gRPC
         server.  If the GCS uses compid=190 (ground station), the drone's gRPC
         never starts.
  Drone: sysid=1,   compid=1
"""

import subprocess
import logging
import asyncio
import time
from pathlib import Path

import pytest
from mavsdk import System

from tests.mock_flight_stack import MockFlightStack

log = logging.getLogger(__name__)

GCS_GRPC_PORT = 50051
PAIRED_GCS_GRPC_PORT = 50053
DRONE_GRPC_PORT = 50052
GCS_MAVLINK_PORT = 14560  # not 14540 — avoids PX4 SITL interference


@pytest.fixture(scope="session", autouse=True)
def _clear_px4_if_paired(request):
    """
    Kill any running PX4 SITL before starting paired-mode tests.

    PX4 uses sysid=1/compid=1 — the same identity as the mock drone mavsdk_server.
    If PX4 is running it will send heartbeats that reach the GCS loopback listener
    on port 14560, causing _wait_for_connection to see two sysid=1 peers and hang.

    In standalone mode (``--drone-address`` supplied) this fixture is a no-op;
    PX4 must be running to serve as the drone under test.
    """
    if request.config.getoption("--drone-address") is not None:
        return

    result = subprocess.run(["pgrep", "-x", "px4"], capture_output=True, text=True)
    if result.returncode != 0:
        return  # no PX4 running

    pids = result.stdout.split()
    log.warning(
        "PX4 SITL detected (PID %s) while starting paired-mode tests. "
        "PX4 uses sysid=1 which conflicts with the mock drone — killing it now.",
        ", ".join(pids),
    )
    for pid in pids:
        subprocess.run(["kill", pid], check=False)
    time.sleep(1.5)


def _find_mavsdk_server() -> Path:
    """Return the path to the mavsdk_server binary bundled with MAVSDK-Python."""
    import mavsdk as _m
    candidate = Path(_m.__file__).parent / "bin" / "mavsdk_server"
    if candidate.exists():
        return candidate
    raise FileNotFoundError(f"mavsdk_server not found at {candidate}")


def _start_mavsdk_server(
    grpc_port: int,
    mavlink_url: str,
    sysid: int = 245,
    compid: int = 190,
) -> subprocess.Popen:
    binary = _find_mavsdk_server()
    cmd = [
        str(binary),
        "-p", str(grpc_port),
        "--sysid", str(sysid),
        "--compid", str(compid),
        mavlink_url,
    ]
    log.info("Starting mavsdk_server: %s", " ".join(cmd))
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    # Give the server a moment to bind its ports.
    time.sleep(1.5)
    if proc.poll() is not None:
        out = proc.stdout.read().decode(errors="replace")
        raise RuntimeError(f"mavsdk_server exited immediately:\n{out}")
    return proc


async def _wait_for_connection(system: System, timeout_s: int) -> None:
    """Raise TimeoutError if the drone does not connect within *timeout_s*."""
    async def _inner():
        async for state in system.core.connection_state():
            if state.is_connected:
                return
    await asyncio.wait_for(_inner(), timeout=timeout_s)


# ---------------------------------------------------------------------------
# Mode-aware GCS (standalone against real drone, or paired against mock)
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def gcs_mavsdk_server(request, paired_gcs_server):
    """
    GCS mavsdk_server for client tests.

    Standalone mode (``--drone-address`` given): starts a dedicated
    mavsdk_server on port 50051 connected to the real drone.

    Paired mode (no ``--drone-address``): reuses the already-started
    ``paired_gcs_server`` (port 50053).  No skip — MockFlightStack provides
    the drone-side protocol handlers.
    """
    drone_address = request.config.getoption("--drone-address")
    if drone_address is None:
        yield PAIRED_GCS_GRPC_PORT
        return

    proc = _start_mavsdk_server(
        grpc_port=GCS_GRPC_PORT,
        mavlink_url=drone_address,
        sysid=255,
        compid=1,
    )
    yield GCS_GRPC_PORT
    proc.kill()
    proc.wait()
    log.info("Standalone GCS mavsdk_server stopped")


@pytest.fixture
async def mock_stack(request, paired_drone_server):
    """
    Start MockFlightStack on the paired drone in paired mode.

    In standalone mode (``--drone-address`` given) this is a no-op; the real
    flight stack provides protocol handling.  In paired mode a fresh
    MockFlightStack is started for each test and cancelled on teardown.

    Tests that need to inspect or reconfigure the mock can request this
    fixture directly; ``gcs_system`` depends on it automatically.
    """
    drone_address = request.config.getoption("--drone-address")
    if drone_address is not None:
        yield None
        return

    system = System(mavsdk_server_address="localhost", port=paired_drone_server)
    await system.connect()

    stack = MockFlightStack()
    task = asyncio.create_task(stack.run(system))
    # Give the gRPC subscriptions a moment to establish before the test starts.
    # _wait_for_connection is intentionally skipped on the drone System: the
    # MAVLink session-level servers may already be connected (connection_state()
    # only fires on *changes*), so waiting for it would hang.
    await asyncio.sleep(0.5)
    yield stack
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass


@pytest.fixture
async def gcs_system(gcs_mavsdk_server, mock_stack, request):
    """
    A fresh MAVSDK System (GCS side) for each client test.

    Works in both standalone mode (connects to real drone via
    ``--drone-address``) and paired mode (connects to MockFlightStack over
    loopback).
    """
    timeout_s = int(request.config.getoption("--connection-timeout"))
    system = System(mavsdk_server_address="localhost", port=gcs_mavsdk_server)
    await system.connect()
    await _wait_for_connection(system, timeout_s)
    yield system


# ---------------------------------------------------------------------------
# Paired loopback (always available, independent of --drone-address)
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def paired_drone_server():
    """
    Drone-side mavsdk_server for paired-mode tests.

    Always started on a fixed loopback port, regardless of whether
    ``--drone-address`` is supplied.
    """
    proc = _start_mavsdk_server(
        grpc_port=DRONE_GRPC_PORT,
        mavlink_url=f"udpout://127.0.0.1:{GCS_MAVLINK_PORT}",
        sysid=1,
        compid=1,
    )
    yield DRONE_GRPC_PORT
    proc.kill()
    proc.wait()
    log.info("Drone mavsdk_server stopped")


@pytest.fixture(scope="session")
def paired_gcs_server(paired_drone_server):
    """
    GCS-side mavsdk_server for paired-mode tests.

    Started after the drone server so the peer is already sending heartbeats
    when the GCS begins listening.
    """
    proc = _start_mavsdk_server(
        grpc_port=PAIRED_GCS_GRPC_PORT,
        mavlink_url=f"udpin://0.0.0.0:{GCS_MAVLINK_PORT}",
        sysid=255,
        compid=1,
    )
    yield PAIRED_GCS_GRPC_PORT
    proc.kill()
    proc.wait()
    log.info("Paired GCS mavsdk_server stopped")


@pytest.fixture
async def paired_gcs_system(paired_gcs_server, request):
    """
    A fresh MAVSDK System (GCS side) for each paired-mode test.
    """
    timeout_s = int(request.config.getoption("--connection-timeout"))
    system = System(mavsdk_server_address="localhost", port=paired_gcs_server)
    await system.connect()
    await _wait_for_connection(system, timeout_s)
    yield system


@pytest.fixture
async def paired_drone_system(paired_drone_server, request):
    """
    A fresh MAVSDK System (drone side) for each paired-mode test.
    """
    timeout_s = int(request.config.getoption("--connection-timeout"))
    system = System(mavsdk_server_address="localhost", port=paired_drone_server)
    await system.connect()
    await _wait_for_connection(system, timeout_s)
    yield system
