"""
MAVLink mission protocol conformance tests.

These tests assert that a flight stack complies with the MAVLink mission
protocol specification.  A FAIL here means the stack has a genuine protocol
non-conformance — not merely a limitation or an unsupported feature.

Unlike test_frame_types.py (which is descriptive/stack-agnostic) and
test_mission_client.py (which tests round-trip fidelity), tests here are
*normative*: they enforce what the spec requires and are expected to FAIL for
non-conformant stacks.

Current conformance tests
--------------------------
TestMissionSlotSemantics
  test_seq0_item_accepted_without_home
    The MAVLink mission protocol does not require or mention a home waypoint
    at seq=0.  A GCS must be able to upload a flight mission starting at seq=0
    without any special home item.  Stacks that reject this violate the spec.

Running
-------
Against a real flight stack::

    pytest tests/mission/test_protocol_conformance.py \\
        --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 -v -s

Against the mock (should always pass)::

    pytest tests/mission/test_protocol_conformance.py -v
"""

import asyncio
import logging

import pytest
from mavsdk.mission_raw import MissionItem, MissionRawError

from .conftest import clear_all_mission_types

log = logging.getLogger(__name__)

TRANSFER_TIMEOUT_S = 30

# SIH simulator home (Zurich) — valid global coordinates for the probe item.
_LAT_INT = 473977000   # 47.3977° × 1e7
_LON_INT = 85456000    # 8.5456° × 1e7


class TestMissionSlotSemantics:
    """
    Assert that the flight stack conforms to the MAVLink mission slot semantics.

    The MAVLink mission protocol defines no special role for seq=0 in a GCS
    upload.  Seq numbers are simply indices; the protocol does not require a
    home waypoint to occupy seq=0.
    """

    async def test_seq0_item_accepted_without_home(self, gcs_system):
        """
        Upload a single NAV_WAYPOINT at seq=0 with no home item prepended.

        Expected (conformant): upload accepted — the flight stack stores the
        item and returns it on download.

        Fail (non-conformant): upload rejected.  The MAVLink mission protocol
        does not require a home waypoint at seq=0.  Rejection is a spec
        violation.

        Note: ArduCopter V4.8.0-dev fails this test.  It rejects the upload
        with TOO_MANY_MISSION_ITEMS because it reserves seq=0 for the home
        position and cannot store a user item there.  This is a known
        ArduCopter non-conformance.
        """
        item = MissionItem(
            seq=0, frame=5, command=16,   # MAV_FRAME_GLOBAL_INT, NAV_WAYPOINT
            current=1, autocontinue=1,
            param1=0.0, param2=0.0, param3=0.0, param4=0.0,
            x=_LAT_INT, y=_LON_INT, z=10.0,
            mission_type=0,
        )

        try:
            try:
                async with asyncio.timeout(TRANSFER_TIMEOUT_S):
                    await gcs_system.mission_raw.upload_mission([item])
            except MissionRawError as exc:
                reason = str(exc).split(":")[0].strip()
                pytest.fail(
                    f"Upload of a single flight mission item (seq=0, no home prepended) "
                    f"was rejected with {reason}.  "
                    "The MAVLink mission protocol does not require a home waypoint at "
                    "seq=0.  This is a protocol non-conformance."
                )

            log.info(
                "test_seq0_item_accepted_without_home: PASS — "
                "single-item upload at seq=0 accepted (no home required)"
            )
        finally:
            await clear_all_mission_types(gcs_system)
