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
  to that address (e.g. a PX4 SITL).  Standalone client tests run; server
  tests use their own paired loopback independently.
* ``--drone-address`` omitted  → server/paired tests still run (loopback);
  standalone client tests are skipped.

Two independent fixture sets
----------------------------
``gcs_mavsdk_server`` / ``gcs_system``
    Standalone GCS — connects to ``--drone-address``.  Used by client tests.
    Skips if ``--drone-address`` is not provided.

``paired_gcs_server`` / ``paired_drone_server``
``paired_gcs_system`` / ``paired_drone_system``
    Loopback-only pair — always started, regardless of ``--drone-address``.
    Used by server tests and ``TestDeprecatedMessageHandling``.  Running a
    flight stack simultaneously is safe because the paired session uses port
    14560 (not the PX4 SITL default of 14540).

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

log = logging.getLogger(__name__)

GCS_GRPC_PORT = 50051
PAIRED_GCS_GRPC_PORT = 50053
DRONE_GRPC_PORT = 50052
GCS_MAVLINK_PORT = 14560  # not 14540 — avoids PX4 SITL interference


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
# Standalone GCS (requires --drone-address)
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def gcs_mavsdk_server(request):
    """
    Start (and stop) the GCS mavsdk_server for standalone client tests.

    Requires ``--drone-address``; skips if not provided (run server tests
    in paired mode independently via ``paired_gcs_server``).
    """
    drone_address = request.config.getoption("--drone-address")
    if drone_address is None:
        pytest.skip("--drone-address not provided; standalone client tests require a real autopilot")

    proc = _start_mavsdk_server(
        grpc_port=GCS_GRPC_PORT,
        mavlink_url=drone_address,
        sysid=255,
        compid=1,
    )
    yield GCS_GRPC_PORT
    proc.kill()
    proc.wait()
    log.info("GCS mavsdk_server stopped")


@pytest.fixture
async def gcs_system(gcs_mavsdk_server, request):
    """
    A fresh MAVSDK System (GCS side) for each standalone client test.
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
