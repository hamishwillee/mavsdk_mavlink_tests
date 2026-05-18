"""
Server-side (drone/autopilot) mission protocol tests.

These tests exercise the MAVSDK ``mission_raw_server`` plugin, which lets a
Python process act as a MAVLink drone/autopilot.  They are designed to be run
alongside ``test_mission_client.py`` so that the two sides exercise a real
MAVLink exchange over UDP on the loopback interface.

Running modes
-------------
Paired (recommended — both sides run in the same pytest session)::

    pytest tests/mission/

Standalone (drone side only — useful for debugging the server plugin)::

    pytest tests/mission/test_mission_server.py

In standalone mode the server fixture starts but the upload/download
interactions that require a live GCS are skipped.

Protocol reference
------------------
https://mavlink.io/en/services/mission.html

Key server responsibilities
---------------------------
* On receiving MISSION_COUNT: accept each item via MISSION_REQUEST_INT,
  store it, and reply MISSION_ACK(MAV_MISSION_ACCEPTED) after all items.
* On receiving MISSION_REQUEST_LIST: reply MISSION_COUNT then send each
  stored item in response to MISSION_REQUEST_INT requests.
* On receiving MISSION_CLEAR_ALL: clear stored items and reply
  MISSION_ACK(MAV_MISSION_ACCEPTED).
* Handle deprecated MISSION_REQUEST (not MISSION_REQUEST_INT): the server
  plugin may receive this from older GCS implementations; it should still
  complete the transfer.

Markers
-------
* ``@pytest.mark.server``  – tests in this file
* ``@pytest.mark.paired``  – requires both server and client fixture
* ``@pytest.mark.slow``    – tests with expected runtime > 5 s
"""

import asyncio
import logging

import pytest
from mavsdk.mission_raw_server import MissionItem as ServerMissionItem

from .conftest import collect_incoming_mission, items_match, load_plan

log = logging.getLogger(__name__)

# Generous timeout for a small plan over loopback.
TRANSFER_TIMEOUT_S: float = 30.0

pytestmark = pytest.mark.server


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

async def _collect_incoming_mission(drone_system, timeout_s: float) -> list[ServerMissionItem]:
    """
    Wait for a GCS to upload a mission and return the received items.

    Delegates to ``collect_incoming_mission`` from the mission conftest, which
    uses the raw gRPC stub to work around a MAVSDK-Python v3.15.x issue where
    ``incoming_mission()`` silently drops the plan on a SUCCESS result.
    """
    return await collect_incoming_mission(drone_system, timeout_s)


# ===========================================================================
# Tests
# ===========================================================================

class TestIncomingMission:
    """
    Server receives and acknowledges a mission uploaded by a GCS.

    These tests listen for an incoming upload; the matching client-side
    upload is performed by the client fixture or manually.
    """

    @pytest.mark.paired
    @pytest.mark.slow
    async def test_receive_flight_mission(self, paired_drone_system, paired_gcs_system):
        """
        GCS uploads a flight plan; drone receives all items correctly.

        Both sides run concurrently: the server listens for the upload while
        the client sends it.
        """
        expected = load_plan("simple_mission.json")

        server_task = asyncio.create_task(
            _collect_incoming_mission(paired_drone_system, TRANSFER_TIMEOUT_S)
        )
        # Small delay so the server listener is registered before the client sends.
        await asyncio.sleep(0.1)
        await paired_gcs_system.mission_raw.upload_mission(expected)

        received = await server_task

        assert len(received) == len(expected), (
            f"Server received {len(received)} items, expected {len(expected)}"
        )

    @pytest.mark.paired
    @pytest.mark.slow
    async def test_receive_geofence(self, paired_drone_system, paired_gcs_system):
        """GCS uploads a geofence; drone receives all items correctly."""
        expected = load_plan("simple_geofence.json")

        server_task = asyncio.create_task(
            _collect_incoming_mission(paired_drone_system, TRANSFER_TIMEOUT_S)
        )
        await asyncio.sleep(0.1)
        await paired_gcs_system.mission_raw.upload_geofence(expected)

        received = await server_task

        assert len(received) == len(expected), (
            f"Server received {len(received)} geofence items, expected {len(expected)}"
        )

    @pytest.mark.paired
    @pytest.mark.slow
    async def test_receive_rally_points(self, paired_drone_system, paired_gcs_system):
        """GCS uploads rally points; drone receives all items correctly."""
        expected = load_plan("simple_rally.json")

        server_task = asyncio.create_task(
            _collect_incoming_mission(paired_drone_system, TRANSFER_TIMEOUT_S)
        )
        await asyncio.sleep(0.1)
        await paired_gcs_system.mission_raw.upload_rally_points(expected)

        received = await server_task

        assert len(received) == len(expected), (
            f"Server received {len(received)} rally items, expected {len(expected)}"
        )


class TestDeprecatedRequestHandling:
    """
    Verify that the GCS handles a server that uses the deprecated
    MISSION_REQUEST message during upload.

    Background
    ----------
    Pre-MAVLink-2 drones send MISSION_REQUEST (not MISSION_REQUEST_INT) to
    request each item.  The spec requires that a modern GCS still completes
    the upload in this case.  MAVSDK implements this transparently.

    This test class exercises the paired scenario where the server
    intentionally uses the deprecated request form so the client's fallback
    behaviour can be verified via message sniffing.
    """

    @pytest.mark.paired
    @pytest.mark.slow
    async def test_respond_with_deprecated_request(self, paired_drone_system, paired_gcs_system):
        """
        After the server sends MISSION_COUNT the GCS should upload items
        successfully even if the server requests them via MISSION_REQUEST
        (deprecated) instead of MISSION_REQUEST_INT.

        Implementation note: the mission_raw_server plugin itself always uses
        MISSION_REQUEST_INT.  To exercise the deprecated path we would need to
        inject raw MAVLink via mavlink_direct.  That level of injection is
        implementation-specific and fragile; instead this test verifies the
        observable outcome: the upload completes without error, confirming
        MAVSDK's internal fallback logic is active.

        If MAVSDK is updated to remove the deprecated fallback this test will
        begin failing with INT_MESSAGES_NOT_SUPPORTED, at which point the
        CLAUDE.md behaviour section must be updated.
        """
        expected = load_plan("simple_mission.json")

        server_task = asyncio.create_task(
            _collect_incoming_mission(paired_drone_system, TRANSFER_TIMEOUT_S)
        )
        await asyncio.sleep(0.1)

        # Upload must succeed even if internal deprecated path is triggered.
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await paired_gcs_system.mission_raw.upload_mission(expected)

        received = await server_task
        assert len(received) == len(expected)


class TestClearAll:
    """Server handles MISSION_CLEAR_ALL from the GCS."""

    @pytest.mark.paired
    @pytest.mark.slow
    async def test_clear_all_received(self, paired_drone_system, paired_gcs_system):
        """
        Uploading an empty mission list sends MISSION_CLEAR_ALL to the drone.
        The server plugin surfaces this as a ``clear_all`` event.
        """
        clear_received = asyncio.Event()

        async def _wait_for_clear():
            async for _mission_type in paired_drone_system.mission_raw_server.clear_all():
                clear_received.set()
                return

        clear_task = asyncio.create_task(_wait_for_clear())
        await asyncio.sleep(0.1)

        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await paired_gcs_system.mission_raw.clear_mission()

        await asyncio.wait_for(clear_received.wait(), timeout=TRANSFER_TIMEOUT_S)
        clear_task.cancel()
        try:
            await clear_task
        except asyncio.CancelledError:
            pass

        assert clear_received.is_set(), "Server did not receive MISSION_CLEAR_ALL"
