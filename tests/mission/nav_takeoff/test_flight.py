"""
MAV_CMD_NAV_TAKEOFF (cmd=22) — Tier 2 execution tests.

Tests whether arming and starting a mission causes the vehicle to actually take off
and, for the yaw test, face a specific heading.  Requires a real flight stack
(--drone-address).  All tests are skipped in paired/mock mode.

Test 1 (test_implicit_takeoff_from_waypoint)
    Upload a NAV_WAYPOINT at TAKEOFF_ALT_M relative altitude with no explicit
    NAV_TAKEOFF item.  Most flight stacks perform an implicit takeoff to reach
    the first waypoint.  PASS if the vehicle climbs to >= 85% of TAKEOFF_ALT_M
    within TAKEOFF_TIMEOUT_S seconds.

Test 2 (test_takeoff_with_yaw)
    Upload a NAV_TAKEOFF item with param4 = TARGET_YAW_DEG (137 deg).  After the
    vehicle climbs above 85% of TAKEOFF_ALT_M, sample the heading and assert it
    is within YAW_TOLERANCE_DEG of TARGET_YAW_DEG.

    Expected behaviour by stack:
      PX4 (all vehicle types)  — PASS:  stores param4 (Yaw).
      ArduCopter / ArduPlane   — FAIL:  AP_Mission discards param4 on storage;
                                         confirmed by Tier 1 test_protocol_param4_yaw_specific.

Safety
    RTL + land are commanded in a finally block after each test.

Running
-------
Against PX4 SIH::

    pytest tests/mission/test_flight.py --drone-address=udp://:14540 -v --log-cli-level=INFO

Against ArduCopter SITL::

    pytest tests/mission/test_flight.py \\
      --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \\
      --ardupilot-sitl=~/ardu_sitl/arducopter \\
      --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 \\
      -v --log-cli-level=INFO
"""

import asyncio
import logging

import pytest
from mavsdk.mission_raw import MissionItem, MissionRawError
from mavsdk.telemetry import LandedState

from ..conftest import clear_all_mission_types

log = logging.getLogger(__name__)

# Flight tests: arming (60 s) + takeoff (90 s) + RTL/land (120 s) + margin.
pytestmark = pytest.mark.timeout(360)

TRANSFER_TIMEOUT_S = 30.0
ARMABLE_TIMEOUT_S  = 60.0
TAKEOFF_ALT_M      = 20.0   # metres relative to home
TAKEOFF_TIMEOUT_S  = 90.0
YAW_TOLERANCE_DEG  = 20.0   # ± degrees for heading assertion
TARGET_YAW_DEG     = 137.0  # unusual value to distinguish from default heading
RTL_LAND_TIMEOUT_S = 120.0

NAN = float("nan")


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

def _north_of(lat_deg: float, metres: float) -> int:
    """Return latitude as int×1e7 for a position metres north of lat_deg."""
    return int((lat_deg + metres / 111111.0) * 1e7)


# ---------------------------------------------------------------------------
# Mission builder
# ---------------------------------------------------------------------------

def _build_mission(home_item, *probes):
    """
    Build a mission item list.

    If home_item is not None (ArduCopter), prepend it at seq=0 with current=0
    and number probes from seq=1.  Otherwise number probes from seq=0.
    The first probe item gets current=1 (mission start point).
    """
    items = []
    if home_item is not None:
        items.append(MissionItem(
            seq=0, frame=home_item.frame, command=home_item.command,
            current=0, autocontinue=1,
            param1=home_item.param1, param2=home_item.param2,
            param3=home_item.param3, param4=home_item.param4,
            x=home_item.x, y=home_item.y, z=home_item.z,
            mission_type=0,
        ))
    offset = len(items)
    for i, p in enumerate(probes):
        items.append(MissionItem(
            seq=offset + i,
            frame=p.frame, command=p.command,
            current=(1 if i == 0 else 0),
            autocontinue=p.autocontinue,
            param1=p.param1, param2=p.param2,
            param3=p.param3, param4=p.param4,
            x=p.x, y=p.y, z=p.z,
            mission_type=p.mission_type,
        ))
    return items


# ---------------------------------------------------------------------------
# Telemetry helpers
# ---------------------------------------------------------------------------

async def _get_home_position(system, timeout_s: float = 30.0):
    """Return the vehicle's home Position from telemetry."""
    async with asyncio.timeout(timeout_s):
        async for home in system.telemetry.home():
            return home
    raise TimeoutError("Home position not received within timeout")


async def _wait_armable(system, timeout_s: float = ARMABLE_TIMEOUT_S):
    """Block until the vehicle reports is_armable=True."""
    async with asyncio.timeout(timeout_s):
        async for health in system.telemetry.health():
            if health.is_armable:
                return


async def _wait_for_altitude(system, threshold_m: float, timeout_s: float = TAKEOFF_TIMEOUT_S):
    """Block until relative_altitude_m >= threshold_m; return the Position."""
    async with asyncio.timeout(timeout_s):
        async for pos in system.telemetry.position():
            if pos.relative_altitude_m >= threshold_m:
                return pos
    raise TimeoutError(
        f"Relative altitude {threshold_m:.1f} m not reached within {timeout_s:.0f} s"
    )


async def _get_heading(system, timeout_s: float = 5.0) -> float:
    """Return current vehicle heading in degrees (0–360)."""
    async with asyncio.timeout(timeout_s):
        async for hdg in system.telemetry.heading():
            return hdg.heading_deg
    raise TimeoutError("Heading not received")


async def _rtl_and_land(system, timeout_s: float = RTL_LAND_TIMEOUT_S):
    """Command RTL and wait for landed state; then disarm.  Best-effort — never raises."""
    try:
        await system.action.return_to_launch()
        async with asyncio.timeout(timeout_s):
            async for state in system.telemetry.landed_state():
                if state == LandedState.ON_GROUND:
                    break
    except Exception as exc:
        log.warning("RTL/land wait failed: %s", exc)
    await asyncio.sleep(2.0)
    try:
        await system.action.disarm()
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def require_real_stack(request):
    """Skip every test in this module when no --drone-address is given."""
    if request.config.getoption("--drone-address") is None:
        pytest.skip("Execution tests require a real flight stack (--drone-address not set)")


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

async def test_takeoff_implicit_from_waypoint(gcs_system, home_item_for_mission):
    """
    A waypoint-only mission triggers an implicit takeoff.

    Uploads a single NAV_WAYPOINT 50 m north of the vehicle's home position at
    TAKEOFF_ALT_M relative altitude.  After arming and starting the mission, the
    test asserts that the vehicle climbs to at least 85% of TAKEOFF_ALT_M within
    TAKEOFF_TIMEOUT_S seconds.  No explicit NAV_TAKEOFF item is present.
    """
    home = await _get_home_position(gcs_system)

    waypoint = MissionItem(
        seq=0,
        frame=6,        # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        command=16,     # MAV_CMD_NAV_WAYPOINT
        current=1,
        autocontinue=1,
        param1=0.0, param2=0.0, param3=0.0, param4=NAN,
        x=_north_of(home.latitude_deg, 50),
        y=int(home.longitude_deg * 1e7),
        z=float(TAKEOFF_ALT_M),
        mission_type=0,
    )
    items = _build_mission(home_item_for_mission, waypoint)

    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
        log.info("Mission uploaded (%d items); waiting for armable", len(items))

        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        log.info("Armed and mission started — waiting for altitude %.1f m", TAKEOFF_ALT_M * 0.85)

        threshold = TAKEOFF_ALT_M * 0.85
        pos = await _wait_for_altitude(gcs_system, threshold)
        log.info("Altitude reached: %.1f m relative", pos.relative_altitude_m)

        assert pos.relative_altitude_m >= threshold, (
            f"Vehicle did not reach {threshold:.1f} m "
            f"(last reading: {pos.relative_altitude_m:.1f} m)"
        )
    finally:
        await _rtl_and_land(gcs_system)
        await clear_all_mission_types(gcs_system)


async def test_takeoff_with_yaw(gcs_system, home_item_for_mission):
    """
    NAV_TAKEOFF with param4=137 deg — vehicle should face that heading post-takeoff.

    Uploads a single NAV_TAKEOFF item at the vehicle's home position with:
      param4 = 137.0 deg (TARGET_YAW_DEG — chosen to be distinct from north)
      altitude = TAKEOFF_ALT_M m relative

    After the vehicle climbs above 85% of TAKEOFF_ALT_M, samples the current heading
    and asserts it is within YAW_TOLERANCE_DEG of TARGET_YAW_DEG.

    PX4 stores param4 (Yaw) for NAV_TAKEOFF — this test should PASS on PX4.
    ArduCopter/ArduPlane discard param4 on storage (AP_Mission only stores param1);
    the vehicle will face whatever its default heading is (typically near-north or
    current) — this test is expected to FAIL on those stacks.  The Tier 1 test
    test_protocol_param4_yaw_specific in test_protocol.py confirms the storage
    behaviour that predicts this outcome.
    """
    home = await _get_home_position(gcs_system)

    takeoff = MissionItem(
        seq=0,
        frame=6,        # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        command=22,     # MAV_CMD_NAV_TAKEOFF
        current=1,
        autocontinue=1,
        param1=15.0,            # Pitch: 15 deg
        param2=0.0,             # unused (0.0 — ArduCopter rejects NaN for param2)
        param3=0.0,             # Flags: none
        param4=TARGET_YAW_DEG,  # Yaw: 137 deg
        x=int(home.latitude_deg  * 1e7),
        y=int(home.longitude_deg * 1e7),
        z=float(TAKEOFF_ALT_M),
        mission_type=0,
    )
    items = _build_mission(home_item_for_mission, takeoff)

    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
        log.info(
            "Mission uploaded (%d items); NAV_TAKEOFF with yaw=%.0f deg",
            len(items), TARGET_YAW_DEG,
        )

        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        log.info("Armed and mission started — waiting for altitude %.1f m", TAKEOFF_ALT_M * 0.85)

        threshold = TAKEOFF_ALT_M * 0.85
        pos = await _wait_for_altitude(gcs_system, threshold)
        log.info("Altitude reached: %.1f m relative", pos.relative_altitude_m)

        heading = await _get_heading(gcs_system)
        diff = abs((heading - TARGET_YAW_DEG + 180) % 360 - 180)
        log.info(
            "Heading: %.1f deg (target %.0f deg, diff %.1f deg, tolerance ±%.0f deg)",
            heading, TARGET_YAW_DEG, diff, YAW_TOLERANCE_DEG,
        )

        assert diff <= YAW_TOLERANCE_DEG, (
            f"Heading {heading:.1f}° deviates {diff:.1f}° from target {TARGET_YAW_DEG}° "
            f"(tolerance ±{YAW_TOLERANCE_DEG}°). "
            f"ArduPilot stacks are expected to fail here: AP_Mission discards NAV_TAKEOFF "
            f"param4 (Yaw) on storage — confirmed by Tier 1 test_protocol.py::test_protocol_param4_yaw_specific."
        )
    finally:
        await _rtl_and_land(gcs_system)
        await clear_all_mission_types(gcs_system)


# ---------------------------------------------------------------------------
# Inline-probe helper for conditional Tier 2 tests
# ---------------------------------------------------------------------------

async def _probe_takeoff_item(system, home_item, **overrides):
    """Upload a NAV_TAKEOFF item with given overrides, download, return the stored item.

    Performs a complete upload–download–clear cycle without flying.
    Returns (stored_item, nacked):
      stored_item — the downloaded MissionItem at the probe seq, or None if not found
      nacked      — True if the upload raised MissionRawError
    """
    defaults = dict(
        seq=0, frame=6, command=22, current=1, autocontinue=1,
        param1=15.0, param2=0.0, param3=0.0, param4=NAN,
        x=0, y=0, z=float(TAKEOFF_ALT_M), mission_type=0,
    )
    defaults.update(overrides)
    takeoff = MissionItem(**defaults)
    items = _build_mission(home_item, takeoff)
    probe_seq = 1 if home_item is not None else 0
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await system.mission_raw.upload_mission(items)
        downloaded = await system.mission_raw.download_mission()
        stored = next((d for d in downloaded if d.seq == probe_seq), None)
        return stored, False
    except MissionRawError:
        return None, True
    finally:
        await clear_all_mission_types(system)


# ---------------------------------------------------------------------------
# Conditional Tier 2 tests — yaw edge cases
# ---------------------------------------------------------------------------

async def test_takeoff_with_negative_yaw(gcs_system, home_item_for_mission):
    """
    Conditional Tier 2: NAV_TAKEOFF param4 = -90° — verify execution normalises correctly.

    Inline probe
    ------------
    Upload param4=-90° and check the stored value:
      NORMALISED (270°) → stack committed to a canonical angle; skip (execution unambiguous).
      RAW (-90°)        → stack deferred normalisation to execution time; proceed.
      OTHER             → stack already altered the value (documented by Tier 1); skip.
      NACKed            → skip (upload failure already a protocol-level result).

    Execution check (raw path only)
    --------------------------------
    Arm → start mission → wait for altitude → assert heading ≈ 270° ± YAW_TOLERANCE_DEG.
    If the stack stores -90° raw but correctly normalises at execution, the vehicle
    should face west (270°).  A heading near north (0°/360°) would indicate the stack
    ignored the value.
    """
    expected_raw = -90.0
    expected_normalised = expected_raw % 360  # 270.0

    stored, nacked = await _probe_takeoff_item(gcs_system, home_item_for_mission, param4=expected_raw)
    if nacked:
        pytest.skip("Upload NACKed — protocol-level result captured by Tier 1 test_protocol.py")
    if stored is None:
        pytest.skip("Probe item not found in download — cannot determine stored value")

    if abs((stored.param4 - expected_normalised + 180) % 360 - 180) < 1e-3:
        pytest.skip(
            f"Stack normalised -90° → {stored.param4:.1f}° on storage — "
            "execution is unambiguous; no Tier 2 needed"
        )
    if abs((stored.param4 - expected_raw + 180) % 360 - 180) >= 1e-3:
        pytest.skip(
            f"Stack stored {stored.param4:.4f}° (neither raw {expected_raw}° nor normalised "
            f"{expected_normalised}°) — behavior already captured by Tier 1 tests"
        )

    log.info("Raw -90° stored — proceeding to execution check (expect heading ≈ 270°)")
    home = await _get_home_position(gcs_system)
    takeoff = MissionItem(
        seq=0, frame=6, command=22, current=1, autocontinue=1,
        param1=15.0, param2=0.0, param3=0.0, param4=expected_raw,
        x=int(home.latitude_deg * 1e7), y=int(home.longitude_deg * 1e7),
        z=float(TAKEOFF_ALT_M), mission_type=0,
    )
    items = _build_mission(home_item_for_mission, takeoff)
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        pos = await _wait_for_altitude(gcs_system, TAKEOFF_ALT_M * 0.85)
        log.info("Altitude reached: %.1f m", pos.relative_altitude_m)
        heading = await _get_heading(gcs_system)
        diff = abs((heading - expected_normalised + 180) % 360 - 180)
        log.info("Heading: %.1f° (target %.0f°, diff %.1f°)", heading, expected_normalised, diff)
        assert diff <= YAW_TOLERANCE_DEG, (
            f"Heading {heading:.1f}° deviates {diff:.1f}° from {expected_normalised}° "
            f"(normalised form of {expected_raw}°). Stack stored raw value but did not "
            f"normalise at execution time."
        )
    finally:
        await _rtl_and_land(gcs_system)
        await clear_all_mission_types(gcs_system)


async def test_takeoff_with_overflow_yaw(gcs_system, home_item_for_mission):
    """
    Conditional Tier 2: NAV_TAKEOFF param4 = 450° — verify execution wraps correctly.

    450° = 360° + 90°; the canonical form is 90° (east).

    Inline probe
    ------------
    Upload param4=450° and check the stored value:
      NORMALISED (90°)  → skip (execution unambiguous).
      RAW (450°)        → proceed to execution check.
      OTHER             → skip (already documented by Tier 1).
      NACKed            → skip.

    Execution check (raw path only)
    --------------------------------
    Assert heading ≈ 90° ± YAW_TOLERANCE_DEG after reaching altitude.
    """
    expected_raw = 450.0
    expected_normalised = expected_raw % 360  # 90.0

    stored, nacked = await _probe_takeoff_item(gcs_system, home_item_for_mission, param4=expected_raw)
    if nacked:
        pytest.skip("Upload NACKed — protocol-level result captured by Tier 1 test_protocol.py")
    if stored is None:
        pytest.skip("Probe item not found in download")

    if abs(stored.param4 - expected_normalised) < 1e-3:
        pytest.skip(
            f"Stack normalised 450° → {stored.param4:.1f}° on storage — "
            "execution is unambiguous; no Tier 2 needed"
        )
    if abs(stored.param4 - expected_raw) >= 1e-3:
        pytest.skip(
            f"Stack stored {stored.param4:.4f}° (neither raw {expected_raw}° nor normalised "
            f"{expected_normalised}°) — behavior already captured by Tier 1 tests"
        )

    log.info("Raw 450° stored — proceeding to execution check (expect heading ≈ 90°)")
    home = await _get_home_position(gcs_system)
    takeoff = MissionItem(
        seq=0, frame=6, command=22, current=1, autocontinue=1,
        param1=15.0, param2=0.0, param3=0.0, param4=expected_raw,
        x=int(home.latitude_deg * 1e7), y=int(home.longitude_deg * 1e7),
        z=float(TAKEOFF_ALT_M), mission_type=0,
    )
    items = _build_mission(home_item_for_mission, takeoff)
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        pos = await _wait_for_altitude(gcs_system, TAKEOFF_ALT_M * 0.85)
        log.info("Altitude reached: %.1f m", pos.relative_altitude_m)
        heading = await _get_heading(gcs_system)
        diff = abs((heading - expected_normalised + 180) % 360 - 180)
        log.info("Heading: %.1f° (target %.0f°, diff %.1f°)", heading, expected_normalised, diff)
        assert diff <= YAW_TOLERANCE_DEG, (
            f"Heading {heading:.1f}° deviates {diff:.1f}° from {expected_normalised}° "
            f"(normalised form of {expected_raw}°). Stack stored raw value but did not "
            f"wrap at execution time."
        )
    finally:
        await _rtl_and_land(gcs_system)
        await clear_all_mission_types(gcs_system)


# ---------------------------------------------------------------------------
# Conditional Tier 2 tests — pitch edge cases
# ---------------------------------------------------------------------------

async def test_takeoff_with_large_pitch(gcs_system, home_item_for_mission):
    """
    Conditional Tier 2: NAV_TAKEOFF param1 = 89° — verify takeoff succeeds.

    Inline probe
    ------------
    Upload param1=89° and check the stored value:
      PRESERVED (89°) → stack stored the raw value; proceed to execution check.
      MODIFIED        → stack clamped or rejected the value; skip (modified value is
                        safe and already the subject of another test).
      NACKed          → skip.

    Execution check (raw path only)
    --------------------------------
    A large pitch angle is unusual but the vehicle should still take off.  The test
    asserts altitude is reached — if the stack misinterprets 89° (e.g. commands a
    near-horizontal takeoff on a multicopter), the altitude check will fail.

    Note: this test does NOT verify that the vehicle pitched to exactly 89° (that
    would require attitude telemetry monitoring during the climb phase, which is
    outside the scope of this test).  It only confirms the vehicle reached the
    target altitude despite the edge-case pitch command.
    """
    expected_raw = 89.0

    stored, nacked = await _probe_takeoff_item(gcs_system, home_item_for_mission, param1=expected_raw)
    if nacked:
        pytest.skip("Upload NACKed — protocol-level result captured by Tier 1 test_protocol.py")
    if stored is None:
        pytest.skip("Probe item not found in download")
    if abs(stored.param1 - expected_raw) >= 1e-3:
        pytest.skip(
            f"Stack modified pitch 89° → {stored.param1:.4f}° on storage — "
            "execution uses the modified value; no Tier 2 needed for the raw value"
        )

    log.info("Raw 89° pitch stored — proceeding to execution check")
    home = await _get_home_position(gcs_system)
    takeoff = MissionItem(
        seq=0, frame=6, command=22, current=1, autocontinue=1,
        param1=expected_raw, param2=0.0, param3=0.0, param4=NAN,
        x=int(home.latitude_deg * 1e7), y=int(home.longitude_deg * 1e7),
        z=float(TAKEOFF_ALT_M), mission_type=0,
    )
    items = _build_mission(home_item_for_mission, takeoff)
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        threshold = TAKEOFF_ALT_M * 0.85
        pos = await _wait_for_altitude(gcs_system, threshold)
        log.info("Altitude reached: %.1f m — large pitch (89°) did not prevent takeoff",
                 pos.relative_altitude_m)
        assert pos.relative_altitude_m >= threshold, (
            f"Vehicle did not reach {threshold:.1f} m with param1=89°. "
            "Stack may be misinterpreting a large pitch as an invalid command."
        )
    finally:
        await _rtl_and_land(gcs_system)
        await clear_all_mission_types(gcs_system)


async def test_takeoff_with_negative_pitch(gcs_system, home_item_for_mission):
    """
    Conditional Tier 2: NAV_TAKEOFF param1 = -10° — verify takeoff succeeds.

    Inline probe
    ------------
    Upload param1=-10° and check the stored value:
      PRESERVED (-10°) → stack stored the raw value; proceed to execution check.
      MODIFIED         → stack normalised the value; skip.
      NACKed           → skip.

    Execution check (raw path only)
    --------------------------------
    Negative pitch is undefined for multicopter takeoff.  If the stack interprets it
    literally (nose-down on takeoff), the vehicle may fail to climb.  The test asserts
    altitude is reached — a failure indicates the stack acted on the negative pitch
    value in a way that prevented takeoff.

    Note: on fixed-wing, negative pitch during takeoff (nose-down climb) would be
    a genuine command; the test still asserts altitude is reached since even fixed-wing
    should climb during a TAKEOFF mission item.
    """
    expected_raw = -10.0

    stored, nacked = await _probe_takeoff_item(gcs_system, home_item_for_mission, param1=expected_raw)
    if nacked:
        pytest.skip("Upload NACKed — protocol-level result captured by Tier 1 test_protocol.py")
    if stored is None:
        pytest.skip("Probe item not found in download")
    if abs(stored.param1 - expected_raw) >= 1e-3:
        pytest.skip(
            f"Stack modified pitch -10° → {stored.param1:.4f}° on storage — "
            "execution uses the modified value; no Tier 2 needed for the raw value"
        )

    log.info("Raw -10° pitch stored — proceeding to execution check")
    home = await _get_home_position(gcs_system)
    takeoff = MissionItem(
        seq=0, frame=6, command=22, current=1, autocontinue=1,
        param1=expected_raw, param2=0.0, param3=0.0, param4=NAN,
        x=int(home.latitude_deg * 1e7), y=int(home.longitude_deg * 1e7),
        z=float(TAKEOFF_ALT_M), mission_type=0,
    )
    items = _build_mission(home_item_for_mission, takeoff)
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        threshold = TAKEOFF_ALT_M * 0.85
        pos = await _wait_for_altitude(gcs_system, threshold)
        log.info("Altitude reached: %.1f m — negative pitch (-10°) did not prevent takeoff",
                 pos.relative_altitude_m)
        assert pos.relative_altitude_m >= threshold, (
            f"Vehicle did not reach {threshold:.1f} m with param1=-10°. "
            "Stack may be acting on the negative pitch value literally (nose-down)."
        )
    finally:
        await _rtl_and_land(gcs_system)
        await clear_all_mission_types(gcs_system)


# ---------------------------------------------------------------------------
# Conditional Tier 2 tests — location sentinel values
# ---------------------------------------------------------------------------

async def test_takeoff_from_current_position(gcs_system, home_item_for_mission):
    """
    Conditional Tier 2: NAV_TAKEOFF with lat/lon = INT32_MAX — take off from current position.

    INT32_MAX is the sentinel meaning "use current vehicle position".  This test verifies
    that when the autopilot stores INT32_MAX faithfully (confirmed by Tier 1
    test_protocol_location_current_position), arming and starting the mission causes the
    vehicle to actually take off from where it is, not fly to some garbage coordinate.

    Inline probe
    ------------
    Upload x=INT32_MAX, y=INT32_MAX and check the stored value:
      PRESERVED (INT32_MAX for both) → stack will interpret the sentinel at execution;
                                       proceed to execution check.
      ALTERED or NACKed              → skip (already documented by Tier 1).

    Execution check (preserved path only)
    --------------------------------------
    Assert the vehicle climbs to ≥ 85% of TAKEOFF_ALT_M within TAKEOFF_TIMEOUT_S.
    A failure (no altitude reached or early abort) would indicate the stack tried to
    navigate to an invalid location rather than taking off from its current position.
    """
    INT32_MAX = 0x7FFF_FFFF

    stored, nacked = await _probe_takeoff_item(
        gcs_system, home_item_for_mission, x=INT32_MAX, y=INT32_MAX
    )
    if nacked:
        pytest.skip("INT32_MAX lat/lon NACKed — result captured by Tier 1 test_protocol_location_current_position")
    if stored is None:
        pytest.skip("Probe item not found in download")
    if stored.x != INT32_MAX or stored.y != INT32_MAX:
        pytest.skip(
            f"Stack altered INT32_MAX sentinel (stored x={stored.x}, y={stored.y}) — "
            "execution uses the altered values; no Tier 2 needed for the sentinel"
        )

    log.info("INT32_MAX lat/lon preserved — proceeding to execution check (expect takeoff from current pos)")
    takeoff = MissionItem(
        seq=0, frame=6, command=22, current=1, autocontinue=1,
        param1=15.0, param2=0.0, param3=0.0, param4=NAN,
        x=INT32_MAX, y=INT32_MAX,
        z=float(TAKEOFF_ALT_M),
        mission_type=0,
    )
    items = _build_mission(home_item_for_mission, takeoff)
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await gcs_system.mission_raw.upload_mission(items)
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        threshold = TAKEOFF_ALT_M * 0.85
        pos = await _wait_for_altitude(gcs_system, threshold)
        log.info(
            "Altitude reached: %.1f m — INT32_MAX sentinel correctly interpreted as 'take off here'",
            pos.relative_altitude_m,
        )
        assert pos.relative_altitude_m >= threshold, (
            f"Vehicle did not reach {threshold:.1f} m with INT32_MAX lat/lon. "
            "Stack may have navigated to an invalid coordinate instead of taking off."
        )
    finally:
        await _rtl_and_land(gcs_system)
        await clear_all_mission_types(gcs_system)
