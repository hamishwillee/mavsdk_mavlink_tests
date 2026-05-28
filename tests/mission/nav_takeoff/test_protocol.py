"""
MAV_CMD_NAV_TAKEOFF (cmd=22) — Tier 1 protocol-acceptance tests.

Tests whether the mission protocol accepts the command and stores each parameter
faithfully.  Does NOT verify execution (the autopilot may accept the item and
ignore it at flight time — that requires Tier 2 telemetry-based testing).

Round-trip evidence is asymmetric
----------------------------------
- A param whose value is NOT preserved → definitely not stored; stack should have
  NACKed rather than silently altering it.
- A param whose value IS preserved → stored correctly for this specific test value;
  does not confirm the autopilot acts on the param during execution.

MAV_CMD_NAV_TAKEOFF parameter table (common.xml)
------------------------------------------------
  param1  Pitch (deg)            Defined
  param2  —                      Unused (empty) — spec: NaN; baseline uses 0.0 (ArduCopter
                                 rejects NaN for param2 — spec violation; see test_param2_unused)
  param3  Flags (NAV_TAKEOFF_FLAGS bitmask)  Defined
  param4  Yaw (deg); NaN = use current heading  Defined
  param5  Latitude  (x field, int × 1e7)   Location
  param6  Longitude (y field, int × 1e7)   Location
  param7  Altitude  (z field, float m)     Location

NAV_TAKEOFF_FLAGS:
  bit 0 (value 1) = NAV_TAKEOFF_FLAGS_HORIZONTAL_POSITION_NOT_REQUIRED

Results are per (autopilot, vehicle type) — a multicopter result does not imply
the same behaviour on fixed-wing or VTOL.

Running
-------
Against the mock (no autopilot required)::

    pytest tests/mission/nav_takeoff/test_protocol.py -v --log-cli-level=INFO

Against a real flight stack::

    pytest tests/mission/nav_takeoff/test_protocol.py --drone-address=udp://:14540 -v --log-cli-level=INFO
"""

import asyncio
import json
import logging
import math

import pytest
import pytest_asyncio
from mavsdk import System
from mavsdk.mavlink_direct import MavlinkMessage
from mavsdk.mission_raw import MissionItem, MissionRawError

from ..conftest import clear_all_mission_types
from tests.conftest import DRONE_GRPC_PORT, _wait_for_connection
from tests.mock_flight_stack import MockFlightStack

log = logging.getLogger(__name__)

TRANSFER_TIMEOUT_S = 30.0
_CMD = "NAV_TAKEOFF"
_FMT = "%-14s | %-44s | %s"

NAN = float("nan")
INT32_MAX = 0x7FFF_FFFF
_MAV_PROTOCOL_CAPABILITY_MISSION_INT = 4


async def _ensure_mission_int_capability(system: System, max_attempts: int = 10) -> None:
    """
    Probe AUTOPILOT_VERSION until MISSION_INT capability is confirmed.

    After high-churn command test classes (TestCommandSurvey runs 168 gRPC
    subscribe/cancel cycles on the GCS mavsdk_server), the server needs time
    to drain pending work before new subscriptions register reliably.  A fixed
    sleep is not sufficient; this probe retries until the capability is seen
    or the attempt limit is reached.

    Receiving AUTOPILOT_VERSION also primes MAVSDK's internal capability cache,
    so the first upload_mission() call does not need to re-query.
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

# SIH simulator home coordinates (47.3977°N, 8.5456°E)
_LAT_INT = 473977000
_LON_INT = 85456000

# ---------------------------------------------------------------------------
# Item builder
# ---------------------------------------------------------------------------


def _takeoff_item(seq: int = 0, current: int = 1, **overrides) -> MissionItem:
    """Return a valid NAV_TAKEOFF MissionItem.  Keyword overrides replace defaults."""
    defaults = dict(
        seq=seq,
        frame=5,        # MAV_FRAME_GLOBAL_INT
        command=22,     # MAV_CMD_NAV_TAKEOFF
        current=current,
        autocontinue=1,
        param1=15.0,    # Pitch: 15 deg
        param2=0.0,     # unused — 0.0 (ArduCopter rejects NaN for this param despite it being unused)
        param3=0.0,     # Flags: none
        param4=NAN,     # Yaw: NaN = use current heading
        x=_LAT_INT,
        y=_LON_INT,
        z=50.0,         # Altitude: 50 m AMSL
        mission_type=0,
    )
    defaults.update(overrides)
    return MissionItem(**defaults)


def _items(home_item, probe: MissionItem):
    """Return (item_list, probe_seq), prepending a home item for ArduCopter if required."""
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


# ---------------------------------------------------------------------------
# Upload / download helper
# ---------------------------------------------------------------------------


async def _upload_probe(system, items: list, probe_seq: int) -> MissionItem:
    """Upload items, download, and return the probe item at probe_seq.

    Raises MissionRawError if the upload is NACKed.
    Raises AssertionError if the probe item is absent from the download.
    """
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


# ---------------------------------------------------------------------------
# Class-scoped fixtures
# ---------------------------------------------------------------------------


@pytest_asyncio.fixture(scope="class", loop_scope="class")
async def mock_stack_cls(request):
    """Class-scoped MockFlightStack.  No-op in standalone (--drone-address) mode."""
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
    timeout_s = int(request.config.getoption("--connection-timeout"))
    system = System(mavsdk_server_address="localhost", port=gcs_mavsdk_server)
    await system.connect()
    await _wait_for_connection(system, timeout_s)
    # ArduCopter initialises its mission storage asynchronously after heartbeat.
    # Without this pause the very first upload can fail with NO_SPACE (max_items=0).
    if request.config.getoption("--drone-address") is not None:
        await asyncio.sleep(3.0)
    else:
        # Explicitly verify MISSION_INT capability before tests.  After high-churn
        # command test classes (TestCommandSurvey does 168 gRPC subscribe/cancel
        # cycles), the GCS mavsdk_server needs time to drain before new subscriptions
        # register reliably.  _ensure_mission_int_capability retries the probe until
        # the MISSION_INT bit is confirmed, also priming MAVSDK's capability cache.
        await _ensure_mission_int_capability(system)
    yield system


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


@pytest.mark.asyncio(loop_scope="class")
class TestNavTakeoff:
    """Protocol-acceptance tests for MAV_CMD_NAV_TAKEOFF (cmd=22)."""

    async def test_protocol_command_accepted(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """Baseline: is NAV_TAKEOFF accepted at all?"""
        probe = _takeoff_item()
        items, probe_seq = _items(home_item_for_mission, probe)
        try:
            async with asyncio.timeout(TRANSFER_TIMEOUT_S):
                await gcs_system_cls.mission_raw.upload_mission(items)
            log.info(_FMT, _CMD, "command", "ACCEPTED")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            pytest.fail(
                f"NAV_TAKEOFF upload rejected: {reason}. "
                "Command is not accepted by this flight stack."
            )
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param1_pitch_preserved(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param1 (Pitch): specific value round-trips correctly."""
        probe = _takeoff_item(param1=15.0)
        items, probe_seq = _items(home_item_for_mission, probe)
        try:
            dl = await _upload_probe(gcs_system_cls, items, probe_seq)
            assert abs(dl.param1 - 15.0) < 1e-4, (
                f"param1 (Pitch) not preserved: uploaded 15.0, downloaded {dl.param1}. "
                "Stack should have NACKed if this value is unsupported."
            )
            log.info(_FMT, _CMD, "param1 (Pitch)", f"PRESERVED ({dl.param1:.4f})")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param1 (Pitch)", f"NACKed: {reason}")
            pytest.fail(f"param1 (Pitch) upload NACKed unexpectedly: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param2_unused(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param2 (unused/empty): NaN is the spec-correct value for unused float params.

        If NaN is rejected, that is a spec violation; a 0.0 retry shows whether the command
        is otherwise accepted.  A non-zero non-NaN probe (1.0) checks NACKing of invalid values.
        """
        nan_rejected = False

        # --- NaN: spec-correct for an unused param ---
        probe = _takeoff_item(param2=NAN)
        items, probe_seq = _items(home_item_for_mission, probe)
        try:
            dl = await _upload_probe(gcs_system_cls, items, probe_seq)
            log.info(_FMT, _CMD, "param2 (unused) NaN",
                     "ACCEPTED — NaN preserved as spec requires")
        except MissionRawError as exc:
            nan_rejected = True
            reason = str(exc).split(":")[0].strip()
            log.warning(
                _FMT, _CMD, "param2 (unused) NaN",
                f"FAIL: NaN rejected ({reason}) — spec violation: unused params must accept NaN",
            )
        finally:
            await clear_all_mission_types(gcs_system_cls)

        # --- 0.0 retry: diagnostic when NaN is rejected ---
        if nan_rejected:
            probe = _takeoff_item(param2=0.0)
            items, probe_seq = _items(home_item_for_mission, probe)
            try:
                dl = await _upload_probe(gcs_system_cls, items, probe_seq)
                log.info(_FMT, _CMD, "param2 (unused) 0.0 retry",
                         f"ACCEPTED with 0.0 (workaround); stored as {dl.param2}")
            except MissionRawError as exc:
                reason = str(exc).split(":")[0].strip()
                log.warning(_FMT, _CMD, "param2 (unused) 0.0 retry",
                            f"REJECTED: {reason}")
            finally:
                await clear_all_mission_types(gcs_system_cls)

        # --- 1.0: non-zero non-NaN — should ideally be NACKed ---
        probe = _takeoff_item(param2=1.0)
        items, probe_seq = _items(home_item_for_mission, probe)
        try:
            dl = await _upload_probe(gcs_system_cls, items, probe_seq)
            if abs(dl.param2 - 1.0) < 1e-4:
                log.warning(_FMT, _CMD, "param2 (unused) 1.0",
                            "NOTE: non-NaN value silently accepted and preserved — should be NACKed")
            else:
                log.warning(_FMT, _CMD, "param2 (unused) 1.0",
                            f"NOTE: non-NaN value silently altered to {dl.param2} — should be NACKed")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param2 (unused) 1.0", f"correctly NACKed: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

        if nan_rejected:
            pytest.fail(
                "param2 (unused) NaN rejected — spec violation. "
                "Unused params must accept NaN per the MAVLink MISSION_ITEM_INT spec."
            )

    async def test_protocol_param3_flags_preserved(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param3 (Flags / NAV_TAKEOFF_FLAGS): bit 0 (HORIZONTAL_POSITION_NOT_REQUIRED) round-trips."""
        probe = _takeoff_item(param3=1.0)  # NAV_TAKEOFF_FLAGS_HORIZONTAL_POSITION_NOT_REQUIRED
        items, probe_seq = _items(home_item_for_mission, probe)
        try:
            dl = await _upload_probe(gcs_system_cls, items, probe_seq)
            assert abs(dl.param3 - 1.0) < 1e-4, (
                f"param3 (Flags) not preserved: uploaded 1.0, downloaded {dl.param3}. "
                "Stack should have NACKed if this flag value is unsupported."
            )
            log.info(_FMT, _CMD, "param3 (Flags)", f"PRESERVED ({dl.param3:.4f})")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param3 (Flags)", f"NACKed: {reason}")
            pytest.fail(f"param3 (Flags) upload NACKed unexpectedly: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param4_yaw_specific(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw): specific degree value round-trips correctly."""
        probe = _takeoff_item(param4=90.0)
        items, probe_seq = _items(home_item_for_mission, probe)
        try:
            dl = await _upload_probe(gcs_system_cls, items, probe_seq)
            assert abs(dl.param4 - 90.0) < 1e-4, (
                f"param4 (Yaw) not preserved: uploaded 90.0, downloaded {dl.param4}. "
                "Stack should have NACKed if this value is unsupported."
            )
            log.info(_FMT, _CMD, "param4 (Yaw) specific", f"PRESERVED ({dl.param4:.4f})")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param4 (Yaw) specific", f"NACKed: {reason}")
            pytest.fail(f"param4 (Yaw=90.0) upload NACKed unexpectedly: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param4_yaw_nan(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw): NaN (use current heading) is accepted and returned as NaN."""
        probe = _takeoff_item(param4=NAN)
        items, probe_seq = _items(home_item_for_mission, probe)
        try:
            dl = await _upload_probe(gcs_system_cls, items, probe_seq)
            assert math.isnan(dl.param4), (
                f"param4 (Yaw) NaN not preserved: downloaded {dl.param4}. "
                "Spec documents NaN as 'use current heading'; it should be stored as NaN."
            )
            log.info(_FMT, _CMD, "param4 (Yaw) NaN", "PRESERVED (NaN)")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param4 (Yaw) NaN", f"NACKed: {reason}")
            pytest.fail(f"param4 (Yaw=NaN) upload NACKed unexpectedly: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_location_preserved(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """params 5/6/7 (Latitude/Longitude/Altitude): location fields round-trip."""
        # Use coordinates offset slightly from home so they are distinct
        lat_int = _LAT_INT + 10000   # ~0.001 deg north
        lon_int = _LON_INT + 10000   # ~0.001 deg east
        alt = 75.0
        probe = _takeoff_item(x=lat_int, y=lon_int, z=alt)
        items, probe_seq = _items(home_item_for_mission, probe)
        try:
            dl = await _upload_probe(gcs_system_cls, items, probe_seq)
            assert dl.x == lat_int, f"Latitude (x) not preserved: {dl.x} != {lat_int}"
            assert dl.y == lon_int, f"Longitude (y) not preserved: {dl.y} != {lon_int}"
            assert abs(dl.z - alt) < 1e-4, f"Altitude (z) not preserved: {dl.z} != {alt}"
            log.info(_FMT, _CMD, "params 5/6/7 (Lat/Lon/Alt)", f"PRESERVED (x={dl.x}, y={dl.y}, z={dl.z:.1f})")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "params 5/6/7 (Lat/Lon/Alt)", f"NACKed: {reason}")
            pytest.fail(f"Location params upload NACKed unexpectedly: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    # ------------------------------------------------------------------
    # Location sentinel values (hasLocation + isDestination)
    # ------------------------------------------------------------------

    async def test_protocol_location_current_position(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """params 5/6 (lat/lon) = INT32_MAX: 'take off from current position' sentinel.

        INT32_MAX (0x7FFF_FFFF) is the MISSION_ITEM_INT sentinel meaning "use current
        position" for integer lat/lon fields.  This is the canonical way to say "take
        off from wherever the vehicle currently is" without specifying coordinates.
        """
        probe = _takeoff_item(x=INT32_MAX, y=INT32_MAX, z=50.0)
        items, probe_seq = _items(home_item_for_mission, probe)
        try:
            dl = await _upload_probe(gcs_system_cls, items, probe_seq)
            x_ok = dl.x == INT32_MAX
            y_ok = dl.y == INT32_MAX
            if x_ok and y_ok:
                log.info(_FMT, _CMD, "params 5/6 INT32_MAX (current pos)",
                         "PRESERVED — 'use current position' accepted")
            else:
                log.warning(_FMT, _CMD, "params 5/6 INT32_MAX (current pos)",
                            f"ALTERED: x={dl.x}, y={dl.y} — sentinel not preserved")
            assert x_ok and y_ok, (
                f"INT32_MAX lat/lon sentinel not preserved: got x={dl.x}, y={dl.y}. "
                "Stack should accept INT32_MAX as 'use current position'."
            )
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.warning(_FMT, _CMD, "params 5/6 INT32_MAX (current pos)", f"NACKed: {reason}")
            pytest.fail(f"INT32_MAX lat/lon NACKed — 'use current position' not supported: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_location_nan_altitude(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param 7 (altitude) = NaN: characterise 'use default altitude' sentinel.

        NaN is the float sentinel for "use default / unspecified" per MISSION_ITEM_INT.
        For a takeoff altitude this is unusual (no explicit target height), so stacks
        may legitimately NACK it.  Outcome is observed and logged; no hard assertion.
        """
        probe = _takeoff_item(z=NAN)
        items, probe_seq = _items(home_item_for_mission, probe)
        try:
            dl = await _upload_probe(gcs_system_cls, items, probe_seq)
            if math.isnan(dl.z):
                log.info(_FMT, _CMD, "param7 (Alt) NaN",
                         "PRESERVED — NaN altitude accepted ('use default')")
            else:
                log.warning(_FMT, _CMD, "param7 (Alt) NaN",
                            f"ALTERED to {dl.z:.4f} — stack normalised NaN altitude")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param7 (Alt) NaN",
                     f"NACKed ({reason}) — NaN altitude not accepted (may be intentional)")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    # ------------------------------------------------------------------
    # param3 (Flags bitmask) additional values
    # ------------------------------------------------------------------

    async def test_protocol_param3_flags_zero(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param3 (Flags) = 0.0: no flags set — most common real-world case.

        value=0 means no special flags are requested.  This must always be accepted
        and must round-trip as 0.0.
        """
        probe = _takeoff_item(param3=0.0)
        items, probe_seq = _items(home_item_for_mission, probe)
        try:
            dl = await _upload_probe(gcs_system_cls, items, probe_seq)
            assert abs(dl.param3) < 1e-4, (
                f"param3=0.0 (no flags) not preserved: got {dl.param3}. "
                "value=0 must always round-trip as 0."
            )
            log.info(_FMT, _CMD, "param3 (Flags) zero", "PRESERVED (0.0)")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            pytest.fail(f"param3=0.0 (no flags) NACKed unexpectedly: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param3_flags_undefined_bits(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param3 (Flags) = 2.0: bit 1 — not defined in NAV_TAKEOFF_FLAGS spec.

        The spec currently defines only bit 0 (value=1).  An undefined bit value
        should ideally be NACKed; silently accepting it is a minor spec violation.
        Outcome is observed and logged; no hard assertion.
        """
        probe = _takeoff_item(param3=2.0)
        items, probe_seq = _items(home_item_for_mission, probe)
        try:
            dl = await _upload_probe(gcs_system_cls, items, probe_seq)
            if abs(dl.param3 - 2.0) < 1e-4:
                log.warning(_FMT, _CMD, "param3 (Flags) undefined bit (2)",
                            "NOTE: undefined bit silently accepted and preserved — NACK preferred")
            else:
                log.warning(_FMT, _CMD, "param3 (Flags) undefined bit (2)",
                            f"NOTE: undefined bit silently altered to {dl.param3} — NACK preferred")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param3 (Flags) undefined bit (2)",
                     f"correctly NACKed: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    # ------------------------------------------------------------------
    # param1 (Pitch) additional values
    # ------------------------------------------------------------------

    async def test_protocol_param1_nan(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param1 (Pitch) = NaN: 'no minimum pitch constraint'.

        NaN for a defined param means "use default / no constraint".  For param1 this
        would mean "autopilot chooses the takeoff pitch".  Some stacks (ArduPilot)
        reject NaN in any defined param via sanity_check_params.  Outcome is observed
        and logged; no hard assertion (NaN for a defined param is ambiguous in the spec).
        """
        probe = _takeoff_item(param1=NAN)
        items, probe_seq = _items(home_item_for_mission, probe)
        try:
            dl = await _upload_probe(gcs_system_cls, items, probe_seq)
            if math.isnan(dl.param1):
                log.info(_FMT, _CMD, "param1 (Pitch) NaN",
                         "PRESERVED — NaN accepted as 'no minimum pitch'")
            else:
                log.info(_FMT, _CMD, "param1 (Pitch) NaN",
                         f"ALTERED to {dl.param1:.4f} — stack normalised NaN pitch")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.warning(_FMT, _CMD, "param1 (Pitch) NaN",
                        f"NaN rejected ({reason}) — stack requires an explicit pitch value")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param1_pitch_very_large(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param1 (Pitch) = 180°: well above the implicit [0°, 90°] maximum.

        180° is a reversal angle — not meaningful as a takeoff pitch.  Possible outcomes:
          PRESERVED — stored as 180.0 (stack accepts any positive float, defers to execution)
          CLAMPED   — stored as some value ≤ 90.0 (stack enforced an upper bound)
          NACKed    — upload rejected as out-of-range

        No assertion: any outcome is valid at the protocol level.  The goal is to
        characterise whether the stack enforces a pitch ceiling.
        """
        probe = _takeoff_item(param1=180.0)
        items, probe_seq = _items(home_item_for_mission, probe)
        try:
            dl = await _upload_probe(gcs_system_cls, items, probe_seq)
            if abs(dl.param1 - 180.0) < 1e-3:
                outcome = "PRESERVED raw (180.0°) — no upper-bound enforcement"
            elif abs(dl.param1) < 1e-3:
                outcome = "ZEROED — param1 not stored by this stack (same as all param1 values)"
            elif dl.param1 < 90.0 + 1e-3:
                outcome = f"CLAMPED to {dl.param1:.4f}° (upper bound enforced)"
            else:
                outcome = f"MODIFIED: stored as {dl.param1:.4f}°"
            log.info(_FMT, _CMD, "param1 (Pitch) 180°", outcome)
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param1 (Pitch) 180°", f"NACKed: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    # ------------------------------------------------------------------
    # param4 (Yaw) edge-case values
    # ------------------------------------------------------------------

    async def test_protocol_param4_yaw_negative(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw) = -90°: characterise storage — raw, normalised, or altered.

        A negative yaw is not canonically valid (yaw is [0, 360) by convention) but
        is a float the GCS could plausibly send.  Possible outcomes:
          PRESERVED  — stored as -90.0  (raw float, execution semantics unknown)
          NORMALISED — stored as 270.0  ([0, 360) canonical form)
          ALTERED    — stored as some other value (e.g. 0.0 — ArduCopter drops param4)
          NACKed     — upload rejected

        No assertion: any outcome is valid at the protocol level.  The result feeds
        into test_flight.py::test_takeoff_with_negative_yaw which applies the conditional
        Tier 2 pattern — flying the vehicle only when the raw value was stored, to verify
        whether execution correctly normalises it.
        """
        probe = _takeoff_item(param4=-90.0)
        items, probe_seq = _items(home_item_for_mission, probe)
        try:
            dl = await _upload_probe(gcs_system_cls, items, probe_seq)
            normalised = -90.0 % 360  # 270.0
            if abs(dl.param4 - (-90.0)) < 1e-3:
                outcome = "PRESERVED raw (-90.0°) — execution normalisation unknown; Tier 2 needed"
            elif abs(dl.param4 - normalised) < 1e-3:
                outcome = f"NORMALISED to {dl.param4:.1f}° on storage — execution unambiguous"
            elif math.isnan(dl.param4):
                outcome = "ALIASED to NaN — stack treats -90.0 as 'use current heading'"
            else:
                outcome = f"ALTERED to {dl.param4:.4f}° (neither raw nor normalised)"
            log.info(_FMT, _CMD, "param4 (Yaw) -90°", outcome)
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param4 (Yaw) -90°", f"NACKed: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param4_yaw_overflow(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw) = 450°: characterise storage — raw, wrapped, or altered.

        450° = 360° + 90°; the canonical normalised form is 90°.  Possible outcomes:
          PRESERVED  — stored as 450.0  (raw float, execution semantics unknown)
          NORMALISED — stored as 90.0   (wrapped to [0, 360))
          ALTERED    — stored as some other value
          NACKed     — upload rejected

        No assertion: either outcome is valid at the protocol level.  See
        test_flight.py::test_takeoff_with_overflow_yaw for the conditional Tier 2 test.
        """
        probe = _takeoff_item(param4=450.0)
        items, probe_seq = _items(home_item_for_mission, probe)
        try:
            dl = await _upload_probe(gcs_system_cls, items, probe_seq)
            normalised = 450.0 % 360  # 90.0
            if abs(dl.param4 - 450.0) < 1e-3:
                outcome = "PRESERVED raw (450.0°) — execution normalisation unknown; Tier 2 needed"
            elif abs(dl.param4 - normalised) < 1e-3:
                outcome = f"NORMALISED to {dl.param4:.1f}° on storage — execution unambiguous"
            elif math.isnan(dl.param4):
                outcome = "ALIASED to NaN — stack treats 450.0 as 'use current heading'"
            else:
                outcome = f"ALTERED to {dl.param4:.4f}° (neither raw nor normalised)"
            log.info(_FMT, _CMD, "param4 (Yaw) 450°", outcome)
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param4 (Yaw) 450°", f"NACKed: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param4_yaw_zero(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw) = 0.0 (due north): must be distinct from NaN.

        0.0 is a valid specific heading (due north) and must not be aliased to NaN
        ("use current heading").  Some autopilots treat the C default value 0 as
        'unset'; that is a spec violation here.

        Spec: param4 = NaN means 'use current heading mode'; param4 = 0.0 means
        'face north explicitly'.  A stack that aliases 0.0 → NaN confuses a specific
        command with the sentinel.

        Note: a stack that does NOT store param4 at all (e.g. ArduPilot) will also
        return 0.0, causing this test to PASS.  That PASS is vacuous — the stack did
        not explicitly store 0.0°; it just happens to return the zero-initialised
        value.  The test only catches the specific spec violation of aliasing 0.0 → NaN.
        """
        probe = _takeoff_item(param4=0.0)
        items, probe_seq = _items(home_item_for_mission, probe)
        try:
            dl = await _upload_probe(gcs_system_cls, items, probe_seq)
            if math.isnan(dl.param4):
                log.warning(_FMT, _CMD, "param4 (Yaw) 0°",
                            "ALIASED to NaN — spec violation: 0° (north) must be distinct from NaN (auto-heading)")
                pytest.fail(
                    "param4=0.0 (due north) aliased to NaN on storage — spec violation. "
                    "Zero is a valid explicit heading; NaN means 'use current heading mode'."
                )
            assert abs(dl.param4) < 1e-4, (
                f"param4=0.0 not preserved: stored as {dl.param4:.4f}. "
                "Zero is a valid specific heading (due north) and must round-trip faithfully."
            )
            log.info(_FMT, _CMD, "param4 (Yaw) 0°", f"PRESERVED ({dl.param4:.4f})")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param4 (Yaw) 0°", f"NACKed: {reason}")
            pytest.fail(f"param4=0.0 (due north) NACKed: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    # ------------------------------------------------------------------
    # param1 (Pitch) edge-case values
    # ------------------------------------------------------------------

    async def test_protocol_param1_pitch_large(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param1 (Pitch) = 89°: characterise storage of a near-maximum value.

        For multicopters, pitch at takeoff typically controls climb angle or speed;
        limits are autopilot-specific.  Possible outcomes:
          PRESERVED — stored as 89.0  (stack accepts without clamping)
          CLAMPED   — stored as some value < 89.0  (stack applied an internal limit)
          NACKed    — upload rejected as out-of-range

        No assertion: clamping without notification is common and not strictly a spec
        violation at the mission protocol level.  The conditional Tier 2 test
        test_flight.py::test_takeoff_with_large_pitch verifies execution succeeds when
        the raw value is stored.
        """
        probe = _takeoff_item(param1=89.0)
        items, probe_seq = _items(home_item_for_mission, probe)
        try:
            dl = await _upload_probe(gcs_system_cls, items, probe_seq)
            if abs(dl.param1 - 89.0) < 1e-3:
                outcome = "PRESERVED raw (89.0°)"
            elif abs(dl.param1) < 1e-3:
                outcome = "ZEROED — param1 not stored by this stack (same as test_protocol_param1_pitch_preserved), or value clamped to 0"
            else:
                outcome = f"MODIFIED: stored as {dl.param1:.4f}° (clamped or normalised)"
            log.info(_FMT, _CMD, "param1 (Pitch) 89°", outcome)
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param1 (Pitch) 89°", f"NACKed: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param1_pitch_negative(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param1 (Pitch) = -10°: characterise storage of a negative pitch angle.

        Negative pitch is meaningful on fixed-wing (nose-down) but typically invalid
        for multicopter takeoff.  Possible outcomes:
          PRESERVED — stored as -10.0  (raw float; execution semantics unknown)
          ABS       — stored as 10.0   (absolute value taken)
          ZEROED    — stored as 0.0    (treated as invalid, silently reset)
          NACKed    — upload rejected

        No assertion: any outcome is valid at the protocol level.  The conditional
        Tier 2 test test_flight.py::test_takeoff_with_negative_pitch runs execution only
        when the raw value (-10.0) is stored, to verify whether the vehicle still
        achieves a successful takeoff.
        """
        probe = _takeoff_item(param1=-10.0)
        items, probe_seq = _items(home_item_for_mission, probe)
        try:
            dl = await _upload_probe(gcs_system_cls, items, probe_seq)
            if abs(dl.param1 - (-10.0)) < 1e-3:
                outcome = "PRESERVED raw (-10.0°) — execution semantics unknown; Tier 2 needed"
            elif abs(dl.param1 - 10.0) < 1e-3:
                outcome = "ABS-NORMALISED to 10.0° on storage"
            elif abs(dl.param1) < 1e-3:
                outcome = "ZEROED — param1 not stored by this stack (same as test_protocol_param1_pitch_preserved), or negative value treated as invalid"
            else:
                outcome = f"ALTERED to {dl.param1:.4f}° — possible integer storage bug (e.g. uint16 underflow for negative float)"
            log.info(_FMT, _CMD, "param1 (Pitch) -10°", outcome)
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param1 (Pitch) -10°", f"NACKed: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)
