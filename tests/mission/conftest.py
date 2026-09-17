"""Mission-specific fixtures: plan loading and item comparison helpers."""

import asyncio
import json
import logging
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import pytest
import pytest_asyncio
from mavsdk import System
from mavsdk.mission_raw import MissionItem, MissionRawError
from mavsdk.mission_raw_server import MissionItem as ServerMissionItem
from mavsdk.mavlink_direct import MavlinkMessage
import mavsdk.mission_raw_server_pb2 as _mrs_pb2

from tests import report
from tests.conftest import DRONE_GRPC_PORT, _wait_for_connection
from tests.mock_flight_stack import MockFlightStack
from tests.param_spec import MAV_FRAME_CATALOGUE, ParamSpec

log = logging.getLogger(__name__)

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
# Home-slot detection
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def requires_home_slot(request) -> bool:
    """
    True if the connected flight stack expects a real home item as the true
    first item (seq=0) of every mission upload.

    ArduPilot always does, for every vehicle type — confirmed 2026-09-13 via
    a from-source ArduCopter build after a NACK-based probe (uploading a bare
    single item and checking for rejection) gave a false negative: ArduPilot
    can silently *accept* a homeless upload (no NACK) while its
    `AP_Mission::add_cmd()` still auto-inserts its own AHRS-derived home at
    storage index 0 on the very first command after a clear, shifting every
    real item one slot — the GCS is never told this happened, so its own
    seq=0 read-back returns the auto-inserted home instead of what it
    uploaded. The home item's own *content* is irrelevant (ArduPilot
    substitutes its real home regardless of what's uploaded) — only its
    *presence* as the first item matters, to keep the GCS's own seq numbering
    aligned with ArduPilot's storage growth. See tests/mission/CLAUDE.md.

    PX4 and the mock have no equivalent behavior (confirmed extensively this
    session) and don't need this — home is never prepended for them.
    """
    return (request.config.getoption("--autopilot") or "").lower() == "ardupilot"


@pytest.fixture(scope="session")
def home_item_for_mission(requires_home_slot, request):
    """
    Return a home MissionItem (seq=0) to prepend to flight missions, or None.

    None when the stack does not require a home slot (PX4, mock).
    MissionItem when required (ArduCopter).  Coordinates come from
    ``--home-lat`` / ``--home-lon`` / ``--home-alt`` CLI options.
    """
    if not requires_home_slot:
        return None
    home_lat = request.config.getoption("--home-lat")
    home_lon = request.config.getoption("--home-lon")
    home_alt = request.config.getoption("--home-alt")
    return MissionItem(
        seq=0, frame=5, command=16,
        current=0, autocontinue=1,
        param1=0.0, param2=0.0, param3=0.0, param4=0.0,
        x=int(home_lat * 1e7), y=int(home_lon * 1e7), z=float(home_alt),
        mission_type=0,
    )


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


# ---------------------------------------------------------------------------
# Mission-protocol capability probe (hoisted — was duplicated verbatim in
# nav_takeoff/test_protocol.py and do_reposition/test_protocol.py)
# ---------------------------------------------------------------------------

_MAV_PROTOCOL_CAPABILITY_MISSION_INT = 4


async def ensure_mission_int_capability(system: System, max_attempts: int = 10) -> None:
    """
    Probe AUTOPILOT_VERSION until MISSION_INT capability is confirmed.

    Needed because a burst of upload/download churn (e.g. a 65-test frame
    survey, or a Tier 1 param sweep) can leave the flight stack slow to
    respond to the very next capability probe; retrying avoids a false
    INT_MESSAGES_NOT_SUPPORTED failure caused by test ordering rather than a
    real capability gap.
    """
    for attempt in range(max_attempts):
        combined: int = 0
        first_seen = asyncio.Event()

        async def _listen() -> None:
            nonlocal combined
            try:
                async for msg in system.mavlink_direct.message("AUTOPILOT_VERSION"):
                    combined |= int(json.loads(msg.fields_json).get("capabilities", 0))
                    first_seen.set()
            except asyncio.CancelledError:
                pass

        listen_task = asyncio.create_task(_listen())
        await asyncio.sleep(0.2)  # let the gRPC stream register on the server

        await system.mavlink_direct.send_message(MavlinkMessage(
            message_name="COMMAND_LONG",
            system_id=0, component_id=0,
            target_system_id=1, target_component_id=1,
            fields_json=json.dumps({
                "target_system": 1, "target_component": 1,
                "command": 512, "confirmation": 0,
                "param1": 148.0, "param2": 0.0, "param3": 0.0,
                "param4": 0.0, "param5": 0.0, "param6": 0.0, "param7": 0.0,
            }),
        ))

        try:
            await asyncio.wait_for(first_seen.wait(), timeout=2.0)
            await asyncio.sleep(0.1)  # collect any concurrent responses
        except asyncio.TimeoutError:
            pass
        finally:
            listen_task.cancel()
            try:
                await listen_task
            except asyncio.CancelledError:
                pass

        if combined & _MAV_PROTOCOL_CAPABILITY_MISSION_INT:
            if attempt > 0:
                log.debug("MISSION_INT capability confirmed after %d attempts", attempt + 1)
            return

        log.debug(
            "Capability probe attempt %d/%d: bits=0x%x — retrying",
            attempt + 1, max_attempts, combined,
        )
        await asyncio.sleep(0.5)

    log.warning("MISSION_INT capability not confirmed after %d attempts", max_attempts)


# ---------------------------------------------------------------------------
# Class-scoped fixtures (hoisted — the pattern used locally by
# test_frame_types.py, nav_takeoff/test_protocol.py and
# do_reposition/test_protocol.py: one System/mock per test class rather than
# per test, to avoid exhausting mavsdk_server's gRPC resources over a large
# param sweep). Any mission test file can use these directly; a file that
# defines its own same-named fixture (as the three above still do) overrides
# these without conflict — fixture resolution prefers the closer definition.
# ---------------------------------------------------------------------------


@pytest_asyncio.fixture(scope="class", loop_scope="class")
async def mock_stack_cls(request):
    """Class-scoped MockFlightStack. No-op in standalone (--drone-address) mode."""
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
async def gcs_system_cls(gcs_mavsdk_server, mock_stack_cls, request):
    """Class-scoped GCS System for mission protocol tests."""
    timeout_s = int(request.config.getoption("--connection-timeout"))
    system = System(mavsdk_server_address="localhost", port=gcs_mavsdk_server)
    await system.connect()
    await _wait_for_connection(system, timeout_s)
    if request.config.getoption("--drone-address") is not None:
        await asyncio.sleep(3.0)
    else:
        await ensure_mission_int_capability(system)
    yield system


# ---------------------------------------------------------------------------
# Raw mavlink_direct mission transport
# ---------------------------------------------------------------------------
#
# MAVSDK's mission_raw plugin (mavsdk_server) rejects some commands
# client-side — before anything reaches the wire — with INVALID_ARGUMENT.
# Confirmed for any MAV_CMD tagged <wip/> in common.xml, such as
# MAV_CMD_CONDITION_GATE (4501): mavsdk_server's internal command table
# simply doesn't recognise it. These helpers implement the
# MISSION_COUNT / MISSION_REQUEST_INT / MISSION_ITEM_INT / MISSION_ACK
# handshake directly via mavlink_direct for any such command — see
# tests/mission/condition_gate/CLAUDE.md for how this was discovered.
#
# NaN encoding: mission_raw.MissionItem takes real Python floats (including
# float('nan')) directly; MAVSDK serialises them internally. mavlink_direct's
# fields_json bridge has no such support (float('nan') is not valid JSON) —
# by the convention already used in tests/command/conftest.py's
# send_command_int()/send_command_long(), a NaN float is represented on the
# wire as JSON `null` and decoded back to NaN on receipt. These helpers do
# that conversion at the boundary so callers keep working with plain
# mavsdk.mission_raw.MissionItem objects carrying real float('nan') fields.

_RAW_GCS_SYSID = 255
_RAW_GCS_COMPID = 1
_RAW_DRONE_SYSID = 1
_RAW_DRONE_COMPID = 1
_RAW_TRANSFER_TIMEOUT_S = 30.0

# MAV_MISSION_RESULT names (common.xml).
_MAV_MISSION_RESULT_NAMES: dict[int, str] = {
    0: "ACCEPTED", 1: "ERROR", 2: "UNSUPPORTED_FRAME", 3: "UNSUPPORTED",
    4: "NO_SPACE", 5: "INVALID", 6: "INVALID_PARAM1", 7: "INVALID_PARAM2",
    8: "INVALID_PARAM3", 9: "INVALID_PARAM4", 10: "INVALID_PARAM5_X",
    11: "INVALID_PARAM6_Y", 12: "INVALID_PARAM7", 13: "INVALID_SEQUENCE",
    14: "DENIED", 15: "OPERATION_CANCELLED",
}


class RawMissionError(Exception):
    """
    Raised when a raw-transport mission upload is NACKed.

    Formatted as "REASON: description", matching mavsdk.mission_raw.MissionRawError's
    string shape, so existing code that extracts the reason via
    ``str(exc).split(":")[0].strip()`` works unchanged regardless of which
    transport (mission_raw vs. raw mavlink_direct) was used.
    """

    def __init__(self, result: int):
        name = _MAV_MISSION_RESULT_NAMES.get(result, f"RESULT_{result}")
        super().__init__(f"{name}: MISSION_ACK result={result} (raw mavlink_direct transport)")
        self.result = result


def _json_safe(value):
    """NaN floats -> None (JSON null, the wire encoding for NaN); everything else passed through."""
    if isinstance(value, float) and math.isnan(value):
        return None
    return value


def _json_nan(value) -> float:
    """JSON null (Python None) -> NaN; otherwise coerce to float."""
    return float("nan") if value is None else float(value)


def _mission_item_fields(item: MissionItem, seq: int, mission_type: int) -> dict:
    return {
        "target_system": _RAW_DRONE_SYSID,
        "target_component": _RAW_DRONE_COMPID,
        "seq": seq,
        "frame": item.frame,
        "command": item.command,
        "current": item.current,
        "autocontinue": item.autocontinue,
        "param1": _json_safe(item.param1),
        "param2": _json_safe(item.param2),
        "param3": _json_safe(item.param3),
        "param4": _json_safe(item.param4),
        "x": item.x,
        "y": item.y,
        "z": _json_safe(item.z),
        "mission_type": mission_type,
    }


async def raw_upload_mission_items(gcs_system, items: list, mission_type: int = 0) -> None:
    """
    Upload mission items via raw mavlink_direct, bypassing mission_raw's
    client-side per-command allow-list. See module docstring above.

    Raises RawMissionError if the upload is NACKed (MISSION_ACK.type != 0).
    """
    ack_result: dict = {}
    ack_event = asyncio.Event()
    by_seq = {item.seq: item for item in items}

    async def _watch_ack() -> None:
        async for msg in gcs_system.mavlink_direct.message("MISSION_ACK"):
            fields = json.loads(msg.fields_json)
            if int(fields.get("mission_type", -1)) == mission_type:
                ack_result["type"] = int(fields.get("type", 1))
                ack_event.set()
                return

    async def _serve_requests(msg_type: str) -> None:
        # Handles both MISSION_REQUEST_INT and the deprecated MISSION_REQUEST
        # (no int32 x/y — ArduCopter sends this one; the spec requires a
        # modern GCS to still respond with MISSION_ITEM_INT, same as
        # mission_raw does transparently — see tests/mission/CLAUDE.md
        # "Deprecated message handling"; confirmed empirically here since a
        # GCS that only understands MISSION_REQUEST_INT never gets asked for
        # any item and the upload times out with OPERATION_CANCELLED).
        async for msg in gcs_system.mavlink_direct.message(msg_type):
            fields = json.loads(msg.fields_json)
            if int(fields.get("mission_type", -1)) != mission_type:
                continue
            seq = int(fields["seq"])
            item = by_seq.get(seq)
            if item is None:
                continue
            await gcs_system.mavlink_direct.send_message(MavlinkMessage(
                message_name="MISSION_ITEM_INT",
                system_id=_RAW_GCS_SYSID, component_id=_RAW_GCS_COMPID,
                target_system_id=_RAW_DRONE_SYSID, target_component_id=_RAW_DRONE_COMPID,
                fields_json=json.dumps(_mission_item_fields(item, seq, mission_type)),
            ))

    ack_task = asyncio.create_task(_watch_ack())
    serve_tasks = [
        asyncio.create_task(_serve_requests("MISSION_REQUEST_INT")),
        asyncio.create_task(_serve_requests("MISSION_REQUEST")),
    ]
    await asyncio.sleep(0.1)  # let gRPC streams register before sending

    try:
        await gcs_system.mavlink_direct.send_message(MavlinkMessage(
            message_name="MISSION_COUNT",
            system_id=_RAW_GCS_SYSID, component_id=_RAW_GCS_COMPID,
            target_system_id=_RAW_DRONE_SYSID, target_component_id=_RAW_DRONE_COMPID,
            fields_json=json.dumps({
                "target_system": _RAW_DRONE_SYSID, "target_component": _RAW_DRONE_COMPID,
                "count": len(items), "mission_type": mission_type,
            }),
        ))
        try:
            await asyncio.wait_for(ack_event.wait(), timeout=_RAW_TRANSFER_TIMEOUT_S)
        except asyncio.TimeoutError:
            raise RuntimeError(
                f"No MISSION_ACK received within {_RAW_TRANSFER_TIMEOUT_S:.0f}s "
                f"for raw mission upload (mission_type={mission_type})"
            )
    finally:
        # Fire-and-forget cancel — see CLAUDE.md §4a: do NOT await these tasks.
        ack_task.cancel()
        for t in serve_tasks:
            t.cancel()

    if ack_result.get("type", 1) != 0:
        raise RawMissionError(ack_result["type"])


async def raw_download_mission_items(gcs_system, mission_type: int = 0) -> list[MissionItem]:
    """
    Download mission items via raw mavlink_direct MISSION_REQUEST_LIST /
    MISSION_COUNT / MISSION_REQUEST_INT-loop / MISSION_ITEM_INT, bypassing
    mission_raw's client-side per-command allow-list.

    Returns a list of mavsdk.mission_raw.MissionItem, one per seq, built from
    the raw wire fields (JSON null floats decoded back to NaN).
    """
    count_seen = asyncio.Event()
    count_holder: dict = {}
    items: dict[int, MissionItem] = {}
    all_received = asyncio.Event()

    async def _watch_count() -> None:
        async for msg in gcs_system.mavlink_direct.message("MISSION_COUNT"):
            fields = json.loads(msg.fields_json)
            if int(fields.get("mission_type", -1)) == mission_type:
                count_holder["count"] = int(fields["count"])
                count_seen.set()
                return

    async def _watch_items() -> None:
        async for msg in gcs_system.mavlink_direct.message("MISSION_ITEM_INT"):
            fields = json.loads(msg.fields_json)
            if int(fields.get("mission_type", -1)) != mission_type:
                continue
            seq = int(fields["seq"])
            items[seq] = MissionItem(
                seq=seq,
                frame=int(fields.get("frame", 0)),
                command=int(fields.get("command", 0)),
                current=int(fields.get("current", 0)),
                autocontinue=int(fields.get("autocontinue", 1)),
                param1=_json_nan(fields.get("param1")),
                param2=_json_nan(fields.get("param2")),
                param3=_json_nan(fields.get("param3")),
                param4=_json_nan(fields.get("param4")),
                x=int(fields.get("x", 0)),
                y=int(fields.get("y", 0)),
                z=_json_nan(fields.get("z")),
                mission_type=mission_type,
            )
            count = count_holder.get("count")
            if count is not None and len(items) >= count:
                all_received.set()

    count_task = asyncio.create_task(_watch_count())
    items_task = asyncio.create_task(_watch_items())
    await asyncio.sleep(0.1)

    try:
        await gcs_system.mavlink_direct.send_message(MavlinkMessage(
            message_name="MISSION_REQUEST_LIST",
            system_id=_RAW_GCS_SYSID, component_id=_RAW_GCS_COMPID,
            target_system_id=_RAW_DRONE_SYSID, target_component_id=_RAW_DRONE_COMPID,
            fields_json=json.dumps({
                "target_system": _RAW_DRONE_SYSID, "target_component": _RAW_DRONE_COMPID,
                "mission_type": mission_type,
            }),
        ))
        try:
            await asyncio.wait_for(count_seen.wait(), timeout=_RAW_TRANSFER_TIMEOUT_S)
        except asyncio.TimeoutError:
            raise RuntimeError(
                f"No MISSION_COUNT received within {_RAW_TRANSFER_TIMEOUT_S:.0f}s "
                f"for raw mission download (mission_type={mission_type})"
            )
        count = count_holder["count"]
        if count == 0:
            return []

        for seq in range(count):
            await gcs_system.mavlink_direct.send_message(MavlinkMessage(
                message_name="MISSION_REQUEST_INT",
                system_id=_RAW_GCS_SYSID, component_id=_RAW_GCS_COMPID,
                target_system_id=_RAW_DRONE_SYSID, target_component_id=_RAW_DRONE_COMPID,
                fields_json=json.dumps({
                    "target_system": _RAW_DRONE_SYSID, "target_component": _RAW_DRONE_COMPID,
                    "seq": seq, "mission_type": mission_type,
                }),
            ))
        try:
            await asyncio.wait_for(all_received.wait(), timeout=_RAW_TRANSFER_TIMEOUT_S)
        except asyncio.TimeoutError:
            raise RuntimeError(
                f"Only {len(items)}/{count} MISSION_ITEM_INT received within "
                f"{_RAW_TRANSFER_TIMEOUT_S:.0f}s for raw mission download (mission_type={mission_type})"
            )
    finally:
        count_task.cancel()
        items_task.cancel()

    return [items[seq] for seq in sorted(items)]


# ---------------------------------------------------------------------------
# Tier 1 mission spec model — mirrors tests/command/conftest.py's
# CommandSpec/Tier1CommandTestBase, adapted for a MAV_CMD used as a mission
# item rather than sent directly as COMMAND_INT/LONG.
# ---------------------------------------------------------------------------


@dataclass
class MissionItemSpec:
    """Everything Tier1MissionTestBase needs to test one MAV_CMD as a mission item."""

    cmd_id: int
    name: str  # e.g. "CONDITION_GATE" — used in log filename/labels
    baseline: dict  # MissionItem kwargs for a valid baseline item (x/y/z + any fixed params)
    params: list[ParamSpec]  # all 7 slots
    mission_type: int = 0
    transport: Literal["mission_raw", "raw"] = "mission_raw"
    frame: int = 5  # MAV_FRAME_GLOBAL_INT — default upload frame

    @property
    def undefined_params(self) -> list["ParamSpec"]:
        return [p for p in self.params if not p.defined]

    @property
    def defined_params(self) -> list["ParamSpec"]:
        return [p for p in self.params if p.defined]


def pytest_generate_tests(metafunc):
    """
    Dynamically parametrize the count-varying Tier 1 tests (undefined/defined
    param sentinel checks) from the test class's SPEC — mission-protocol
    analogue of tests/command/conftest.py's hook of the same name. A no-op
    for any test class without a SPEC (i.e. everything except
    Tier1MissionTestBase subclasses).
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


def _with_home_prepend(home_item: MissionItem | None, probe: MissionItem) -> tuple[list, int]:
    """
    Return (item_list, probe_seq), prepending a home item for ArduCopter if
    required (hoisted — was duplicated as `_items()` in nav_takeoff and
    do_reposition's test_protocol.py).
    """
    if home_item is not None:
        home = MissionItem(
            seq=home_item.seq, frame=home_item.frame, command=home_item.command,
            current=1, autocontinue=home_item.autocontinue,
            param1=home_item.param1, param2=home_item.param2,
            param3=home_item.param3, param4=home_item.param4,
            x=home_item.x, y=home_item.y, z=home_item.z,
            mission_type=home_item.mission_type,
        )
        adjusted = MissionItem(
            seq=1, frame=probe.frame, command=probe.command,
            current=0, autocontinue=probe.autocontinue,
            param1=probe.param1, param2=probe.param2,
            param3=probe.param3, param4=probe.param4,
            x=probe.x, y=probe.y, z=probe.z,
            mission_type=probe.mission_type,
        )
        return [home, adjusted], 1
    return [probe], 0


_FMT = "%-14s | %-44s | %s"
TRANSFER_TIMEOUT_S = 30.0


def _check(cls, request, description: str, outcome: str, *, expect, xfail_reason: str | None = None) -> None:
    """
    Record this test's outcome into the shared report (tests/report.py), then
    perform the actual pytest assertion/xfail.

    `outcome`: "ACCEPTED" (upload succeeded) or a MAV_MISSION_RESULT reason
    name extracted from a caught NACK (e.g. "UNSUPPORTED", "DENIED").
    `expect`: predicate(outcome:str) -> bool. `xfail_reason`: if given and
    the predicate fails, xfail with this reason instead of hard-failing.
    """
    name = cls.SPEC.name
    if expect(outcome):
        report.record_tier1_result("mission", name, request.node.name, "PASS", description, outcome)
        return
    if xfail_reason:
        report.record_tier1_result("mission", name, request.node.name, "XFAIL", description, outcome)
        pytest.xfail(xfail_reason)
    report.record_tier1_result("mission", name, request.node.name, "FAIL", description, outcome)
    pytest.fail(f"{description} (got: {outcome})")


@pytest.fixture(scope="class", autouse=True)
def _write_tier1_log(request):
    """
    Write this command's combined report (Tier 1, plus Tier 2 if it ran in
    the same pytest session — see tests/report.py) once, after every test in
    the class has run. A no-op for any test class that isn't a
    Tier1MissionTestBase subclass, or one where nothing actually ran (e.g.
    every test filtered out by -k).
    """
    yield
    cls = request.cls
    if cls is None or not hasattr(cls, "SPEC"):
        return
    spec = cls.SPEC
    if not report.has_tier1_results("mission", spec.name):
        return
    report.write("mission", spec.name, spec.cmd_id, request.config)


class Tier1MissionTestBase:
    """
    Shared Tier 1 (mission-protocol) mandatory common tests for a MAV_CMD used
    as a mission item. Subclass and set `SPEC = MissionItemSpec(...)`; add the
    command's own bespoke per-parameter tests as ordinary methods alongside
    these (enum/bitmask/range/location semantics stay bespoke per command,
    exactly as they did before this base class existed — see
    tests/mission/CLAUDE.md § MAV_CMD support testing).
    """

    SPEC: "MissionItemSpec"  # set by subclass

    def __init_subclass__(cls, **kwargs) -> None:
        super().__init_subclass__(**kwargs)
        # Register this command's identity + full param-slot list with the
        # shared report (tests/report.py) once per subclass, so every param
        # key always appears in the rendered JSON (see declare_params'
        # docstring), and so a Tier-2-only run of the same command can find
        # this declaration even without Tier 1 running in the same session
        # (Tier 2 files import SPEC from their sibling test_protocol.py).
        spec = cls.SPEC
        report.declare_command("mission", spec.name, spec.cmd_id)
        report.declare_params("mission", spec.name, [f"{p.slot}_{p.label}" for p in spec.params])
        # Undefined ("Empty") params have nothing to functionally support —
        # set this immediately (not waiting for their sentinel test to run)
        # so it's present even if that specific parametrized test is
        # filtered out via -k.
        for p in spec.undefined_params:
            report.record_compat_fact("mission", spec.name, f"{p.slot}_{p.label}", supported="not-applicable")

    async def _upload_probe(self, system, home_item, **overrides) -> MissionItem:
        """
        Build an item from SPEC.baseline + overrides, apply home-slot
        prepend, upload (via SPEC.transport) and download, and return the
        stored probe item.

        Raises MissionRawError (mission_raw transport) or RawMissionError
        (raw transport) on NACK; raises AssertionError if the probe item is
        absent from the download.
        """
        spec = self.SPEC
        kw = dict(
            seq=0, current=1, frame=spec.frame, command=spec.cmd_id,
            autocontinue=1, mission_type=spec.mission_type,
        )
        kw.update(spec.baseline)
        kw.update(overrides)
        probe = MissionItem(**kw)
        items, probe_seq = _with_home_prepend(home_item, probe)

        if spec.transport == "raw":
            await raw_upload_mission_items(system, items, mission_type=spec.mission_type)
            downloaded = await raw_download_mission_items(system, mission_type=spec.mission_type)
        else:
            async with asyncio.timeout(TRANSFER_TIMEOUT_S):
                await system.mission_raw.upload_mission(items)
            async with asyncio.timeout(TRANSFER_TIMEOUT_S):
                downloaded = await system.mission_raw.download_mission()

        dl = next((d for d in downloaded if d.seq == probe_seq), None)
        assert dl is not None, (
            f"probe item seq={probe_seq} not found in download "
            f"(seqs present: {[d.seq for d in downloaded]})"
        )
        return dl

    async def _probe_outcome(self, system, home_item, **overrides) -> str:
        """_upload_probe(), reduced to "ACCEPTED" or a NACK reason name. Always cleans up."""
        try:
            await self._upload_probe(system, home_item, **overrides)
            return "ACCEPTED"
        except (MissionRawError, RawMissionError) as exc:
            return str(exc).split(":")[0].strip()
        finally:
            await clear_all_mission_types(system)

    # -------------------------------------------------------------------
    # Baseline: is this command accepted as a mission item at all?
    # Cached per class so every param-level test below can be skipped with
    # one clear reason once the whole item is rejected (do_reposition's
    # original pattern, generalised — see tests/mission/do_reposition/CLAUDE.md).
    # -------------------------------------------------------------------

    @pytest_asyncio.fixture(scope="class", loop_scope="class")
    async def _mission_support(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """Probe once whether SPEC.cmd_id is accepted as a mission item at all.

        Returns (supported: bool, reason: str | None).
        """
        outcome = await self._probe_outcome(gcs_system_cls, home_item_for_mission)
        return (True, None) if outcome == "ACCEPTED" else (False, outcome)

    @pytest.fixture(autouse=True)
    def _skip_if_unsupported(self, request, _mission_support):
        """Skip every param-level test once the baseline is rejected outright.

        `test_mission_item_supported` itself is exempt — it's the one test that
        records the baseline finding.
        """
        if request.node.name == "test_mission_item_supported":
            return
        supported, reason = _mission_support
        if not supported:
            pytest.skip(
                f"{self.SPEC.name} (cmd={self.SPEC.cmd_id}) rejected outright as a mission "
                f"item ({reason}); param-level probing is moot — see test_mission_item_supported"
            )

    async def test_mission_item_supported(self, _mission_support, request):
        """
        Baseline: is this command accepted as a mission item at all?

        Purely observational — many MAV_CMDs are legitimately rejected as
        mission items by design (guided-only commands, unimplemented
        commands, <wip/>-tagged commands); see the command's own CLAUDE.md
        for the spec-alignment discussion. When rejected, every remaining
        param-level test in this class is skipped (see
        `_skip_if_unsupported`) — testing individual parameter values is
        moot once the whole item is unsupported.
        """
        supported, reason = _mission_support
        description = "Baseline: accepted as a mission item at all (observational)"
        outcome = "ACCEPTED" if supported else reason
        report.record_tier1_result("mission", self.SPEC.name, request.node.name, "PASS", description, outcome)
        if not supported:
            # A protocol-level rejection is as reliable a "not supported"
            # signal as this harness can ever have — record it at the
            # command level so the JSON export has something even for a
            # command with no Tier 2 test at all (e.g. DO_REPOSITION). Only
            # attach `notes` when the reason says something `supported:
            # false` doesn't already — the generic "UNSUPPORTED" NACK reason
            # is exactly what `supported: false` already means, so a note
            # restating it would be pure redundancy.
            note = outcome if outcome != "UNSUPPORTED" else None
            report.record_command_fact("mission", self.SPEC.name, supported=False, notes=note)
        log.info(_FMT, self.SPEC.name, "command", outcome)

    # -------------------------------------------------------------------
    # Undefined params: sentinel accepted, non-sentinel rejected.
    # Parametrized per-command via SPEC.undefined_params.
    # -------------------------------------------------------------------

    async def test_undefined_param_sentinel_accepted(self, gcs_system_cls, mock_stack_cls, home_item_for_mission, request, undefined_param):
        """Accepted when an undefined param is sent as its own sentinel."""
        p = undefined_param
        outcome = await self._probe_outcome(gcs_system_cls, home_item_for_mission, **p.mission_sentinel_kwargs)
        description = f"Accepted when param{p.slot} ({p.label}) is sent as its own sentinel (undefined param)"
        report.record_compat_fact(
            "mission", self.SPEC.name, f"{p.slot}_{p.label}", accept_nan_or_int32max=(outcome == "ACCEPTED"),
        )
        _check(type(self), request, description, outcome, expect=lambda o: o == "ACCEPTED")

    async def test_undefined_param_nonsentinel_rejected(self, gcs_system_cls, mock_stack_cls, home_item_for_mission, request, undefined_param):
        """Rejected when an undefined param is sent a real (non-sentinel) value."""
        p = undefined_param
        outcome = await self._probe_outcome(gcs_system_cls, home_item_for_mission, **p.mission_nonsentinel_kwargs)
        description = f"Rejected when param{p.slot} ({p.label}) is sent a real (non-sentinel) value (undefined param)"
        report.record_compat_fact(
            "mission", self.SPEC.name, f"{p.slot}_{p.label}", nacks_on_non_sentinel_value=(outcome != "ACCEPTED"),
        )
        xfail_reason = p.reject_xfail_reason or (
            f"Stack returned {outcome!r} for undefined param{p.slot}; expected a NACK — no "
            "known stack validates parameters with no MAVLink definition (spec gap)"
        )
        _check(type(self), request, description, outcome, expect=lambda o: o != "ACCEPTED",
               xfail_reason=xfail_reason)

    # -------------------------------------------------------------------
    # Defined (used) params tolerate their sentinel, except a
    # "deny_required" param (mandatory field with no sentinel fallback),
    # where the expectation flips: rejection is the correct, passing result.
    # -------------------------------------------------------------------

    async def test_defined_param_sentinel_tolerated(self, gcs_system_cls, mock_stack_cls, home_item_for_mission, request, defined_param):
        """Not rejected (or rejected, for a documented mandatory-field exemption) when a defined param is sent its sentinel."""
        p = defined_param
        outcome = await self._probe_outcome(gcs_system_cls, home_item_for_mission, **p.mission_sentinel_kwargs)
        report.record_compat_fact(
            "mission", self.SPEC.name, f"{p.slot}_{p.label}", accept_nan_or_int32max=(outcome == "ACCEPTED"),
        )
        if p.sentinel_policy == "deny_required":
            description = (
                f"Rejected when param{p.slot} ({p.label}) is sent its sentinel "
                "(mandatory field, no sentinel fallback)"
            )
            _check(type(self), request, description, outcome, expect=lambda o: o != "ACCEPTED")
        else:
            description = f"Not rejected when param{p.slot} ({p.label}, defined) is sent its sentinel"
            _check(type(self), request, description, outcome, expect=lambda o: o == "ACCEPTED",
                   xfail_reason=p.sentinel_xfail_reason)

    # -------------------------------------------------------------------
    # Frame validation survey — mirrors Tier1CommandTestBase's own
    # test_frame_validation_survey (tests/command/conftest.py), sharing the
    # same MAV_FRAME_CATALOGUE (tests/param_spec.py). Command-side and
    # mission-side outcomes are NOT symmetric: COMMAND_INT's raw ACK result
    # code distinguishes MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME(9) (frame
    # itself invalid) from MAV_RESULT_UNSUPPORTED(3) (command not
    # recognised at all) — but MAVSDK's mission_raw plugin collapses BOTH
    # MAV_MISSION_UNSUPPORTED_FRAME and MAV_MISSION_UNSUPPORTED into one
    # client-side Result.UNSUPPORTED (confirmed by enumerating
    # mavsdk.mission_raw.MissionRawResult.Result — no frame-specific member
    # exists), so there is no equivalent PASS ("frame validation confirmed")
    # / INCONCLUSIVE split here. Purely observational: which frames a
    # command accepts as a mission item is itself command-specific (a
    # location command like NAV_TAKEOFF accepts several; a non-location
    # DO_* command may accept only MAV_FRAME_MISSION, or none at all if
    # rejected outright already) — there's no universal right answer to
    # assert against generically, so this always PASSes and records the
    # full per-frame breakdown as supplementary detail for a human/README
    # to read, exactly like an "any outcome is protocol-valid" observational
    # test (root CLAUDE.md rule 3).
    # -------------------------------------------------------------------

    async def test_frame_validation_survey(self, gcs_system_cls, mock_stack_cls, home_item_for_mission, request):
        """
        Frame validation survey: which MAV_FRAME values is this command
        accepted as a mission item under? Observational — see class docstring
        note above for why this never fails/xfails regardless of the result.
        """
        description = "Frame validation survey: which MAV_FRAME values accept this command as a mission item"
        per_frame: list[tuple[int, str, str]] = []
        for frame_id, frame_name in MAV_FRAME_CATALOGUE:
            outcome = await self._probe_outcome(gcs_system_cls, home_item_for_mission, frame=frame_id)
            per_frame.append((frame_id, frame_name, outcome))

        accepted = [(fid, fname) for fid, fname, outcome in per_frame if outcome == "ACCEPTED"]
        table_lines = [f"  frame={fid:>2} ({fname}): {outcome}" for fid, fname, outcome in per_frame]
        block = (
            f"Frame validation survey ({self.SPEC.name}, cmd={self.SPEC.cmd_id}) — "
            f"full breakdown:\n" + "\n".join(table_lines)
        )
        if accepted:
            acc_desc = ", ".join(f"frame={fid} ({fname})" for fid, fname in accepted)
            block += f"\nAccepted under: {acc_desc}"
        else:
            block += "\nAccepted under no frame in this catalogue (rejected outright — see test_mission_item_supported)."
        report.record_tier1_detail("mission", self.SPEC.name, block)
        log.info(_FMT, self.SPEC.name, "frame validation survey", block)
        report.record_tier1_result("mission", self.SPEC.name, request.node.name, "PASS", description, None)
