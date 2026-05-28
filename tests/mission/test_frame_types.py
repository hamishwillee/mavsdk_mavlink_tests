"""
MAVLink frame-type support matrix tests.

Probes every MAV_FRAME value (0–21) for all three mission types to produce a
frame-support matrix for the connected flight stack.  The tests are intentionally
stack-agnostic: no frame is pre-judged as "must be accepted" or "must be rejected".
A test PASSES for any internally-consistent outcome.

Pass conditions
---------------
- Upload rejected with MissionRawError  → PASS (rejection is valid for any frame)
- Upload accepted, frame preserved on download  → PASS
- Upload accepted, frame changed only by INT-encoding within the same altitude
  category  → PASS (acceptable, e.g. GLOBAL→GLOBAL_INT or
  GLOBAL_RELATIVE_ALT→GLOBAL_RELATIVE_ALT_INT when using MISSION_ITEM_INT)

Fail conditions
---------------
- Upload accepted but download raises MissionRawError  → FAIL (inconsistent)
- Upload accepted but download returns wrong item count  → FAIL (data loss)
- Upload raises a non-MissionRawError exception  → propagates as an error
- Upload accepted but altitude reference category changed on download  → FAIL
  (e.g. GLOBAL_RELATIVE_ALT stored as GLOBAL_INT changes what altitude the
  autopilot flies — the mission will not behave as specified).
  The only acceptable frame changes are within the same altitude category:
    GLOBAL (0) ↔ GLOBAL_INT (5)
    GLOBAL_RELATIVE_ALT (3) ↔ GLOBAL_RELATIVE_ALT_INT (6)
    GLOBAL_TERRAIN_ALT (10) ↔ GLOBAL_TERRAIN_ALT_INT (11)

Deprecated frames (LOCAL_ENU=4, BODY_NED=8, BODY_OFFSET_NED=9)
----------------------------------------------------------------
PASS whether rejected (preferred) or accepted.  If accepted, a WARNING is
logged; the test does not fail.  Flight stacks may choose to support deprecated
frames as long as they do so consistently.

MAV_FRAME_MISSION (frame=2)
---------------------------
Excluded from the matrix — it is covered by TestMissionFrame.
MAV_FRAME_MISSION is for DO_* commands where parameter values are passed
unscaled (not position-encoded).  Using it with location-bearing commands
(NAV/fence/rally) is a misuse, tested separately.

Coordinate convention
---------------------
Global frames (0, 3, 5, 6, 10, 11):  x = lat × 1e7, y = lon × 1e7
Local/body frames (all others):       x = 100, y = 200 (metres from home)

Running
-------
Against a real flight stack::

    pytest tests/mission/test_frame_types.py --drone-address=udp://:14540 -v

Against the mock (no autopilot required)::

    pytest tests/mission/test_frame_types.py -v

The mock accepts all frames and stores/returns them unchanged, so all tests
pass with "ACCEPTED, round-trip frame preserved".  Against a real flight stack
(e.g. PX4) some frames are REJECTED and some are transformed on storage.

The INFO-level log lines report each frame's ACCEPTED / REJECTED outcome and
(if accepted) whether the downloaded frame value differs from what was uploaded.
This output constitutes a live support matrix for the connected flight stack.
"""

import asyncio
import logging
import math

import pytest
import pytest_asyncio
from mavsdk import System
from mavsdk.mission_raw import MissionItem, MissionRawError

from .conftest import clear_all_mission_types
from tests.conftest import DRONE_GRPC_PORT, _wait_for_connection
from tests.mock_flight_stack import MockFlightStack

log = logging.getLogger(__name__)

TRANSFER_TIMEOUT_S = 30.0

# ---------------------------------------------------------------------------
# Frame catalogue
# ---------------------------------------------------------------------------

# (frame_value, frame_name, status)
# status: "active" | "deprecated" | "reserved"
# Frame 2 (MAV_FRAME_MISSION) is intentionally absent — see TestMissionFrame.
FRAME_CASES = [
    pytest.param(0,  "MAV_FRAME_GLOBAL",                   "active",     id="0-global"),
    pytest.param(1,  "MAV_FRAME_LOCAL_NED",                "active",     id="1-local-ned"),
    pytest.param(3,  "MAV_FRAME_GLOBAL_RELATIVE_ALT",      "active",     id="3-global-rel-alt"),
    pytest.param(4,  "MAV_FRAME_LOCAL_ENU",                "deprecated", id="4-local-enu"),
    pytest.param(5,  "MAV_FRAME_GLOBAL_INT",               "active",     id="5-global-int"),
    pytest.param(6,  "MAV_FRAME_GLOBAL_RELATIVE_ALT_INT",  "active",     id="6-global-rel-alt-int"),
    pytest.param(7,  "MAV_FRAME_LOCAL_OFFSET_NED",         "active",     id="7-local-offset-ned"),
    pytest.param(8,  "MAV_FRAME_BODY_NED",                 "deprecated", id="8-body-ned"),
    pytest.param(9,  "MAV_FRAME_BODY_OFFSET_NED",          "deprecated", id="9-body-offset-ned"),
    pytest.param(10, "MAV_FRAME_GLOBAL_TERRAIN_ALT",       "active",     id="10-terrain-alt"),
    pytest.param(11, "MAV_FRAME_GLOBAL_TERRAIN_ALT_INT",   "active",     id="11-terrain-alt-int"),
    pytest.param(12, "MAV_FRAME_BODY_FRD",                 "active",     id="12-body-frd"),
    pytest.param(13, "MAV_FRAME_RESERVED_13",              "reserved",   id="13-reserved"),
    pytest.param(14, "MAV_FRAME_RESERVED_14",              "reserved",   id="14-reserved"),
    pytest.param(15, "MAV_FRAME_RESERVED_15",              "reserved",   id="15-reserved"),
    pytest.param(16, "MAV_FRAME_RESERVED_16",              "reserved",   id="16-reserved"),
    pytest.param(17, "MAV_FRAME_RESERVED_17",              "reserved",   id="17-reserved"),
    pytest.param(18, "MAV_FRAME_RESERVED_18",              "reserved",   id="18-reserved"),
    pytest.param(19, "MAV_FRAME_RESERVED_19",              "reserved",   id="19-reserved"),
    pytest.param(20, "MAV_FRAME_LOCAL_FRD",                "active",     id="20-local-frd"),
    pytest.param(21, "MAV_FRAME_LOCAL_FLU",                "active",     id="21-local-flu"),
]

# Lookup from frame value → name (covers all values 0–21 including frame=2)
_FRAME_NAME: dict[int, str] = {p.values[0]: p.values[1] for p in FRAME_CASES}
_FRAME_NAME[2] = "MAV_FRAME_MISSION"

# ---------------------------------------------------------------------------
# Item builder
# ---------------------------------------------------------------------------

# SIH simulator home (used for global frames so coordinates are physically valid)
_LAT_INT = 473977000   # 47.3977° × 1e7
_LON_INT = 85456000    # 8.5456° × 1e7

# Small integers used for local/body frames (represent metres from home origin)
_LOCAL_X = 100
_LOCAL_Y = 200

# Frames where x/y encode lat/lon × 1e7
_GLOBAL_FRAMES = {0, 3, 5, 6, 10, 11}

# Acceptable same-meaning frame changes when using MISSION_ITEM_INT:
# only INT-encoding variants within the same altitude category are allowed.
# Any change that crosses a category boundary alters mission behaviour and is a FAIL.
_ALT_INT_EQUIV: tuple[frozenset, ...] = (
    frozenset({0, 5}),    # GLOBAL ↔ GLOBAL_INT            (absolute AMSL)
    frozenset({3, 6}),    # GLOBAL_RELATIVE_ALT ↔ …_INT    (relative to home)
    frozenset({10, 11}),  # GLOBAL_TERRAIN_ALT ↔ …_INT     (relative to terrain)
)


def _make_item(frame: int, mission_type: int, seq: int = 0, current: int = 1) -> MissionItem:
    x = _LAT_INT if frame in _GLOBAL_FRAMES else _LOCAL_X
    y = _LON_INT if frame in _GLOBAL_FRAMES else _LOCAL_Y
    if mission_type == 0:
        cmd = 16    # MAV_CMD_NAV_WAYPOINT
    elif mission_type == 1:
        cmd = 5000  # MAV_CMD_NAV_FENCE_RETURN_POINT (PX4/pymavlink numbering)
    else:
        cmd = 5100  # MAV_CMD_NAV_RALLY_POINT
    return MissionItem(
        seq=seq, frame=frame, command=cmd,
        current=current, autocontinue=1,
        param1=0.0, param2=0.0, param3=0.0, param4=0.0,
        x=x, y=y, z=10.0, mission_type=mission_type,
    )


# ---------------------------------------------------------------------------
# Shared probe helper
# ---------------------------------------------------------------------------

def _exc_reason(exc: MissionRawError) -> str:
    """Return just the result-enum name from a MissionRawError, e.g. 'UNSUPPORTED'."""
    return str(exc).split(':')[0].strip()


def _label(name: str, status: str) -> str:
    """Frame name with status annotation appended only when non-active."""
    if status == "deprecated":
        return f"{name} (DEPRECATED)"
    if status == "reserved":
        return f"{name} (RESERVED)"
    return name


async def _probe_frame(
    gcs_system,
    items: list,
    upload_fn,
    download_fn,
    frame: int,
    name: str,
    probe_seq: int = 0,
) -> str:
    """
    Probe one frame value and return a one-line human-readable result summary.

    Returns a string describing the outcome.  Raises pytest.fail() if the
    outcome is internally inconsistent (upload accepted but download broken,
    or altitude reference changed without z conversion).
    """
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await upload_fn(items)
    except MissionRawError as exc:
        return f"REJECTED: {_exc_reason(exc)}"

    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            downloaded = await download_fn()
    except MissionRawError as exc:
        pytest.fail(
            f"frame={frame} ({name}): upload succeeded but download failed — "
            f"{_exc_reason(exc)}. "
            "Flight stacks MUST return accepted missions on download."
        )

    assert downloaded, (
        f"frame={frame} ({name}): upload succeeded but download returned empty list."
    )
    assert len(downloaded) >= probe_seq + 1, (
        f"frame={frame} ({name}): uploaded {len(items)} item(s) "
        f"but downloaded {len(downloaded)} (expected at least {probe_seq + 1})."
    )
    assert all(math.isfinite(d.z) for d in downloaded), (
        f"frame={frame} ({name}): downloaded z values contain non-finite values."
    )

    dl = next((d for d in downloaded if d.seq == probe_seq), None)
    assert dl is not None, (
        f"frame={frame} ({name}): probe item with seq={probe_seq} not found "
        f"in download (seqs present: {[d.seq for d in downloaded]})."
    )
    dl_frame = dl.frame
    if dl_frame == frame:
        return f"ACCEPTED, frame preserved (z={dl.z:.3f})"

    dl_name = _FRAME_NAME.get(dl_frame, str(dl_frame))
    same_category = any(frame in pair and dl_frame in pair for pair in _ALT_INT_EQUIV)
    if not same_category:
        pytest.fail(
            f"frame={frame} ({name}): altitude reference category changed on storage "
            f"({name} → {dl_name}). "
            "Only INT-encoding changes within the same altitude category are acceptable "
            "(GLOBAL↔GLOBAL_INT, GLOBAL_RELATIVE_ALT↔GLOBAL_RELATIVE_ALT_INT, "
            "GLOBAL_TERRAIN_ALT↔GLOBAL_TERRAIN_ALT_INT). "
            "Changing altitude reference changes mission behaviour."
        )
    return f"ACCEPTED, INT-encoded as {dl_name} (z={dl.z:.3f})"


# ---------------------------------------------------------------------------
# Class-scoped GCS system fixture
#
# All tests in each class share one System object (and one gRPC channel) so
# that 65 consecutive tests don't exhaust mavsdk_server's gRPC resources.
# @pytest.mark.asyncio(loop_scope="class") ensures all tests in the class run
# in the same asyncio event loop, which is required for gRPC channels to be
# shared safely.  Teardown is done inside each test's try/finally block (not
# in a fixture) so that cleanup runs in the class event loop rather than the
# function event loop.
# ---------------------------------------------------------------------------

@pytest_asyncio.fixture(scope="class", loop_scope="class")
async def mock_stack_cls(request):
    """
    Class-scoped MockFlightStack for frame-type tests.

    Paired mode: one MockFlightStack instance covers all 21 parametrised tests
    in the class (avoids gRPC resource exhaustion from per-test System churn).
    Standalone mode (``--drone-address`` given): no-op.
    """
    drone_address = request.config.getoption("--drone-address")
    if drone_address is not None:
        yield None
        return

    system = System(mavsdk_server_address="localhost", port=DRONE_GRPC_PORT)
    await system.connect()

    stack = MockFlightStack()
    task = asyncio.create_task(stack.run(system))
    await asyncio.sleep(0.5)  # let gRPC subscriptions establish
    yield stack
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass


@pytest_asyncio.fixture(scope="class", loop_scope="class")
async def gcs_system_cls(gcs_mavsdk_server, mock_stack_cls, request):
    timeout_s = int(request.config.getoption("--connection-timeout"))
    system = System(mavsdk_server_address="localhost", port=gcs_mavsdk_server)
    await system.connect()
    await _wait_for_connection(system, timeout_s)
    yield system


# ---------------------------------------------------------------------------
# Matrix test classes
# ---------------------------------------------------------------------------

_FMT = "PASS | %-8s | frame=%2d %-40s | %s"


@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.frames
class TestFlightMissionFrames:
    """Probe all MAV_FRAME values for flight missions (mission_type=0)."""

    @pytest.mark.parametrize("frame,name,status", FRAME_CASES)
    async def test_frame(self, gcs_system_cls, home_item_for_mission, frame, name, status):
        if home_item_for_mission is not None:
            items = [home_item_for_mission, _make_item(frame, mission_type=0, seq=1, current=0)]
            probe_seq = 1
        else:
            items = [_make_item(frame, mission_type=0)]
            probe_seq = 0
        try:
            summary = await _probe_frame(
                gcs_system_cls, items,
                gcs_system_cls.mission_raw.upload_mission,
                gcs_system_cls.mission_raw.download_mission,
                frame, name,
                probe_seq=probe_seq,
            )
            log.info(_FMT, "MISSION", frame, _label(name, status), summary)
        finally:
            await clear_all_mission_types(gcs_system_cls)


@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.frames
class TestGeofenceFrames:
    """Probe all MAV_FRAME values for geofence (mission_type=1)."""

    @pytest.mark.parametrize("frame,name,status", FRAME_CASES)
    async def test_frame(self, gcs_system_cls, frame, name, status):
        try:
            summary = await _probe_frame(
                gcs_system_cls, [_make_item(frame, mission_type=1)],
                gcs_system_cls.mission_raw.upload_geofence,
                gcs_system_cls.mission_raw.download_geofence,
                frame, name,
            )
            log.info(_FMT, "GEOFENCE", frame, _label(name, status), summary)
        finally:
            await clear_all_mission_types(gcs_system_cls)


@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.frames
class TestRallyPointFrames:
    """Probe all MAV_FRAME values for rally points (mission_type=2)."""

    @pytest.mark.parametrize("frame,name,status", FRAME_CASES)
    async def test_frame(self, gcs_system_cls, frame, name, status):
        try:
            summary = await _probe_frame(
                gcs_system_cls, [_make_item(frame, mission_type=2)],
                gcs_system_cls.mission_raw.upload_rally_points,
                gcs_system_cls.mission_raw.download_rallypoints,
                frame, name,
            )
            log.info(_FMT, "RALLY", frame, _label(name, status), summary)
        finally:
            await clear_all_mission_types(gcs_system_cls)


# ---------------------------------------------------------------------------
# MAV_FRAME_MISSION (frame=2) — separate tests
# ---------------------------------------------------------------------------

@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.frames
class TestMissionFrame:
    """
    Tests for MAV_FRAME_MISSION (frame=2).

    MAV_FRAME_MISSION passes command parameters unscaled — the flight stack
    stores and returns them as-sent.  It is intended for DO_* commands that
    carry no position data.

    Using MAV_FRAME_MISSION with location-bearing commands (NAV/fence/rally,
    whose isLocation flag means param5/6/7 encode lat/lon/alt) is a misuse:
    the flight stack has no way to interpret x/y correctly.
    """

    async def test_mission_frame_with_do_command(self, gcs_system_cls):
        """
        MAV_FRAME_MISSION with DO_CHANGE_SPEED (cmd=178) — a non-location command.

        PASS if rejected (flight stack may not support MISSION frame for any command).
        PASS if accepted and param1 is preserved within tolerance.
        FAIL if accepted but param1 is corrupted (MISSION frame values must be unscaled).
        """
        item = MissionItem(
            seq=0, frame=2, command=178,  # MAV_CMD_DO_CHANGE_SPEED
            current=1, autocontinue=1,
            param1=5.0,  # speed in m/s — verified on download
            param2=0.0, param3=0.0, param4=0.0,
            x=0, y=0, z=0.0, mission_type=0,
        )
        try:
            try:
                async with asyncio.timeout(TRANSFER_TIMEOUT_S):
                    await gcs_system_cls.mission_raw.upload_mission([item])
            except MissionRawError as exc:
                log.info(_FMT, "MISSION", 2, "MAV_FRAME_MISSION",
                         f"REJECTED: {_exc_reason(exc)} (DO_CHANGE_SPEED)")
                return  # PASS

            downloaded = await gcs_system_cls.mission_raw.download_mission()
            assert downloaded, "Upload succeeded but download returned empty"
            assert abs(downloaded[0].param1 - 5.0) < 1e-4, (
                f"param1 corrupted: expected 5.0, got {downloaded[0].param1}. "
                "MAV_FRAME_MISSION values must be passed unscaled."
            )
            log.info(_FMT, "MISSION", 2, "MAV_FRAME_MISSION",
                     f"ACCEPTED, DO_CHANGE_SPEED param1={downloaded[0].param1:.4f} preserved")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_mission_frame_misuse_with_location_command(self, gcs_system_cls):
        """
        MAV_FRAME_MISSION with NAV_WAYPOINT (cmd=16) — a location command (misuse).

        x/y are set to small integers (100, 200), NOT lat/lon × 1e7.  If a flight
        stack accepts this, it would store a physically incorrect position.

        Always PASSES — the test documents behavior, not enforces a policy.
        PASS if rejected (safest behavior — preferred).
        PASS if accepted — a WARNING is logged describing what was stored.
        """
        item = MissionItem(
            seq=0, frame=2, command=16,  # MAV_CMD_NAV_WAYPOINT
            current=1, autocontinue=1,
            param1=0.0, param2=0.0, param3=0.0, param4=0.0,
            x=100, y=200, z=10.0,        # NOT lat/lon × 1e7
            mission_type=0,
        )
        try:
            try:
                async with asyncio.timeout(TRANSFER_TIMEOUT_S):
                    await gcs_system_cls.mission_raw.upload_mission([item])
            except MissionRawError as exc:
                log.info(_FMT, "MISSION", 2, "MAV_FRAME_MISSION",
                         f"REJECTED: {_exc_reason(exc)} (NAV_WAYPOINT misuse, safest outcome)")
                return  # PASS

            downloaded = await gcs_system_cls.mission_raw.download_mission()
            if downloaded:
                dl = downloaded[0]
                dl_frame_name = _FRAME_NAME.get(dl.frame, str(dl.frame))
                log.info(_FMT, "MISSION", 2, "MAV_FRAME_MISSION",
                         f"ACCEPTED, NAV_WAYPOINT misuse ← stored as {dl_frame_name} "
                         f"x={dl.x} y={dl.y} z={dl.z:.1f} (coordinates not lat/lon)")
            else:
                log.info(_FMT, "MISSION", 2, "MAV_FRAME_MISSION",
                         "ACCEPTED then download empty (NAV_WAYPOINT misuse)")
        finally:
            await clear_all_mission_types(gcs_system_cls)
