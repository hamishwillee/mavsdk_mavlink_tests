"""Mission-specific fixtures: plan loading and item comparison helpers."""

import asyncio
import json
from pathlib import Path

import pytest
from mavsdk.mission_raw import MissionItem
from mavsdk.mission_raw_server import MissionItem as ServerMissionItem
from mavsdk.mavlink_direct import MavlinkMessage
import mavsdk.mission_raw_server_pb2 as _mrs_pb2

PLANS_DIR = Path(__file__).parent / "plans"


# ---------------------------------------------------------------------------
# Plan loading
# ---------------------------------------------------------------------------

def load_plan(filename: str) -> list[MissionItem]:
    """
    Load mission items from a JSON plan file in the plans/ directory.

    Each JSON file contains a ``mission_type`` integer and an ``items`` list.
    Each item maps directly to the MISSION_ITEM_INT MAVLink message fields.
    Fields with leading underscores (``_comment``, ``_description``, etc.) are
    documentation-only and are ignored.
    """
    path = PLANS_DIR / filename
    with open(path) as fh:
        plan = json.load(fh)

    mission_type = plan["mission_type"]
    items = []
    for raw in plan["items"]:
        items.append(
            MissionItem(
                seq=raw["seq"],
                frame=raw["frame"],
                command=raw["command"],
                current=raw["current"],
                autocontinue=raw["autocontinue"],
                param1=float(raw.get("param1", 0.0)),
                param2=float(raw.get("param2", 0.0)),
                param3=float(raw.get("param3", 0.0)),
                param4=float(raw.get("param4", 0.0)),
                x=int(raw["x"]),
                y=int(raw["y"]),
                z=float(raw.get("z", 0.0)),
                mission_type=mission_type,
            )
        )
    return items


@pytest.fixture(scope="session")
def flight_plan_items():
    return load_plan("simple_mission.json")


@pytest.fixture(scope="session")
def geofence_items():
    return load_plan("simple_geofence.json")


@pytest.fixture(scope="session")
def rally_items():
    return load_plan("simple_rally.json")


# ---------------------------------------------------------------------------
# Comparison helpers
# ---------------------------------------------------------------------------

def items_match(uploaded: list[MissionItem], downloaded: list[MissionItem]) -> bool:
    """
    Compare two mission item lists for protocol-level equality.

    Floating-point parameters are compared with a tolerance of 1e-4 to account
    for autopilot-side rounding (e.g. ArduPilot stores floats with limited
    precision).  Integer coordinate fields (x, y) are compared exactly.
    """
    if len(uploaded) != len(downloaded):
        return False
    tol = 1e-4
    for u, d in zip(uploaded, downloaded):
        if u.seq != d.seq:
            return False
        if u.frame != d.frame:
            return False
        if u.command != d.command:
            return False
        if u.mission_type != d.mission_type:
            return False
        if u.x != d.x or u.y != d.y:
            return False
        if abs(u.z - d.z) > tol:
            return False
        for attr in ("param1", "param2", "param3", "param4"):
            if abs(getattr(u, attr) - getattr(d, attr)) > tol:
                return False
    return True


_CLEANUP_TIMEOUT_S: float = 10.0


async def clear_all_mission_types(system) -> None:
    """
    Clear all MAVLink mission types from the autopilot.

    MAVSDK's ``clear_mission()`` sends MISSION_CLEAR_ALL with mission_type=0
    (flight missions only).  Geofence (type=1) and rally (type=2) require
    separate raw MISSION_CLEAR_ALL messages sent via mavlink_direct.

    All three operations are best-effort: failures are silently ignored so
    that a cleanup failure never masks an actual test failure.
    """
    try:
        await asyncio.wait_for(system.mission_raw.clear_mission(), timeout=_CLEANUP_TIMEOUT_S)
    except Exception:
        pass
    for mission_type in (1, 2):
        await _send_clear_all(system, mission_type)


async def _send_clear_all(system, mission_type: int) -> None:
    """Send MISSION_CLEAR_ALL for *mission_type* and wait up to 10 s for ACK."""
    ack_received = asyncio.Event()

    async def _watch():
        async for msg in system.mavlink_direct.message("MISSION_ACK"):
            if json.loads(msg.fields_json).get("mission_type") == mission_type:
                ack_received.set()
                return

    watch_task = asyncio.create_task(_watch())
    await asyncio.sleep(0.1)  # let gRPC stream establish before sending

    try:
        await system.mavlink_direct.send_message(MavlinkMessage(
            message_name="MISSION_CLEAR_ALL",
            system_id=255,
            component_id=1,
            target_system_id=1,
            target_component_id=1,
            fields_json=json.dumps({
                "target_system": 1,
                "target_component": 1,
                "mission_type": mission_type,
            }),
        ))
        await asyncio.wait_for(ack_received.wait(), timeout=_CLEANUP_TIMEOUT_S)
    except Exception:
        pass
    finally:
        watch_task.cancel()
        try:
            await watch_task
        except asyncio.CancelledError:
            pass


async def collect_incoming_mission(
    drone_system, timeout_s: float
) -> list[ServerMissionItem]:
    """
    Wait for a GCS upload and return the received mission items.

    Uses the raw gRPC stub instead of ``mission_raw_server.incoming_mission()``
    because MAVSDK-Python v3.15.x sends result=SUCCESS (not NEXT) when
    delivering the mission plan.  The high-level helper discards the plan on
    SUCCESS and returns an empty generator, so we bypass it here.
    """
    from mavsdk.mission_raw_server import MissionPlan

    stub = drone_system.mission_raw_server._stub

    async def _inner():
        req = _mrs_pb2.SubscribeIncomingMissionRequest()
        stream = stub.SubscribeIncomingMission(req)
        try:
            async for response in stream:
                plan = MissionPlan.translate_from_rpc(response.mission_plan)
                stream.cancel()
                return list(plan.mission_items)
            return []
        finally:
            stream.cancel()

    return await asyncio.wait_for(_inner(), timeout=timeout_s)
