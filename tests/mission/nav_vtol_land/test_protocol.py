"""
MAV_CMD_NAV_VTOL_LAND (cmd=85) — Tier 1 protocol-acceptance tests (mission item).

"Land using VTOL mode"

Tests whether the mission protocol accepts the command and stores each parameter
faithfully. Does NOT verify execution — see test_flight.py (Tier 2).

Built on tests/mission/conftest.py's Tier1MissionTestBase/MissionItemSpec: the
shared mandatory tests (baseline accepted, undefined-param sentinel pair,
defined-param sentinel tolerated, frame validation survey) are inherited; this
file adds NAV_VTOL_LAND's own per-parameter tests.

Round-trip evidence is asymmetric: a value NOT preserved means the stack didn't
store it and should have NACKed it (rule 4 — a compatibility error); a value
preserved says nothing about whether execution uses it (that's Tier 2).

MAV_CMD_NAV_VTOL_LAND parameter table (common.xml)
--------------------------------------------------
  param1  Land Options             enum NAV_VTOL_LAND_OPTIONS (0 DEFAULT, 1 FW_SPIRAL_APPROACH, 2 FW_APPROACH)
  param2  Empty
  param3  Approach Altitude (m)    same reference as the Altitude field; NaN if unspecified
  param4  Yaw (deg)                NaN = use the current system yaw heading mode
  param5  Latitude  (x, int × 1e7)
  param6  Longitude (y, int × 1e7)
  param7  Ground Altitude (z, m)   NaN = use system default landing altitude (ignore value)

Expected from source (blind read, 2026-10-13 — root CLAUDE.md Tier 2 pattern #5)
--------------------------------------------------------------------------------
PX4 v1.18.0-beta1-660: `mavlink_command_params.hpp` lists `{ 85, 0x7C, 0x7F }`
— the mission mask 0x7C allows params 3-7 only, so a non-default Land Options
(param1) or a real param2 should be NACKed at upload. `mavlink_mission.cpp`
stores only the yaw (wrapped to [0, 360)) and the location for this command —
param3 (Approach Altitude) is allowed by the mask ("QGC sets it; unused by
mission_block") but not stored, so it should come back altered: accepted but not
stored, a compatibility error.

Running
-------
Against the mock (no autopilot required)::

    pytest tests/mission/nav_vtol_land/test_protocol.py -v --log-cli-level=INFO

PX4 SIH VTOL (the px4io/px4-sitl container build — see README.md)::

    pytest tests/mission/nav_vtol_land/test_protocol.py --drone-address=udp://:14540 \\
        --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_standard_vtol \\
        --vehicle-type=vtol --autopilot=px4 -v --log-cli-level=INFO
"""

import logging
import math

import pytest

from tests import report
from tests.report import _tier1_auto_record  # noqa: F401 — autouse: bespoke tests show in the report too
from mavsdk.plugins.mission_raw import MissionRawError

from ..conftest import MissionItemSpec, Tier1MissionTestBase, clear_all_mission_types
from tests.param_spec import ParamSpec

log = logging.getLogger(__name__)

_CMD = "NAV_VTOL_LAND"
_CMD_NAME = _CMD  # read by tests/report.py's key_from_module()
_CMD_ID = 85  # MAV_CMD_NAV_VTOL_LAND
_FMT = "%-16s | %-44s | %s"

NAN = float("nan")
INT32_MAX = 0x7FFF_FFFF

# SIH simulator home (47.3977°N, 8.5456°E) — same convention as nav_takeoff.
_LAT_INT = 473977000
_LON_INT = 85456000

# NAV_VTOL_LAND_OPTIONS (common.xml)
LAND_OPTIONS = {0: "DEFAULT", 1: "FW_SPIRAL_APPROACH", 2: "FW_APPROACH"}

SPEC = MissionItemSpec(
    cmd_id=_CMD_ID,
    name=_CMD,
    mission_type=0,
    frame=6,  # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    baseline=dict(
        param1=0.0,   # Land Options: DEFAULT
        param2=0.0,   # Empty
        param3=NAN,   # Approach Altitude: unspecified (the XML's own value for that)
        param4=NAN,   # Yaw: use the current heading mode
        x=_LAT_INT,
        y=_LON_INT,
        z=0.0,        # ground level, relative to home
    ),
    params=[
        ParamSpec(1, "Land Options", defined=True),
        ParamSpec(2, "Empty", defined=False),
        ParamSpec(3, "Approach Altitude", defined=True),  # NaN sentinel IS spec-mandated
        ParamSpec(4, "Yaw", defined=True),                # NaN sentinel IS spec-mandated
        ParamSpec(5, "Latitude", defined=True),
        ParamSpec(6, "Longitude", defined=True),
        ParamSpec(7, "Ground Altitude", defined=True),    # NaN sentinel IS spec-mandated
    ],
)


def _nack_reason(exc: Exception) -> str:
    return str(exc).split(":")[0].strip()


@pytest.mark.asyncio(loop_scope="class")
class TestNavVtolLand(Tier1MissionTestBase):
    """Protocol-acceptance tests for MAV_CMD_NAV_VTOL_LAND (cmd=85) as a mission item."""

    SPEC = SPEC

    async def _stored(self, system, home_item, field: str, **kw):
        """Upload one variant; (stored value of `field`, None) or (None, NACK reason). Always clears."""
        try:
            dl = await self._upload_probe(system, home_item, **kw)
            return getattr(dl, field), None
        except MissionRawError as exc:
            return None, _nack_reason(exc)
        finally:
            await clear_all_mission_types(system)

    # ------------------------------------------------------------------
    # param1 (Land Options) — an enum: every defined value (rule 4c exemption)
    # ------------------------------------------------------------------

    async def test_nav_vtol_land_param1_land_options_values(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param1 (Land Options): every NAV_VTOL_LAND_OPTIONS value is preserved or NACKed, never silently altered.

        DEFAULT (0) can't tell "stored" from "not stored" (a stack that drops param1
        returns 0 anyway); the other two can.
        """
        outcomes = {}
        for value, name in LAND_OPTIONS.items():
            stored, nack = await self._stored(gcs_system_cls, home_item_for_mission, "param1", param1=float(value))
            outcomes[value] = f"NACKed ({nack})" if nack else ("PRESERVED" if abs(stored - value) < 1e-4 else f"ALTERED to {stored}")
            log.info(_FMT, _CMD, f"param1 = {value} ({name})", outcomes[value])
        summary = "; ".join(f"{v} {LAND_OPTIONS[v]}: {o}" for v, o in outcomes.items())
        report.record_tier1_detail("mission", _CMD, f"param1 (Land Options) per value — {summary}")
        report.record_nonsentinel_ack("mission", _CMD, "1_Land Options",
                                      nacked=all(o.startswith("NACKed") for v, o in outcomes.items() if v != 0))
        altered = [LAND_OPTIONS[v] for v, o in outcomes.items() if o.startswith("ALTERED")]
        if altered:
            report.compat_fail(f"param1 (Land Options) accepted but not stored for {', '.join(altered)} — "
                               f"should have been NACKed: {summary}")

    async def test_nav_vtol_land_param1_land_options_out_of_range(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param1 (Land Options) = 3, one past the last enum value: characterisation, no assertion (rule 3)."""
        stored, nack = await self._stored(gcs_system_cls, home_item_for_mission, "param1", param1=3.0)
        outcome = f"NACKed ({nack})" if nack else ("PRESERVED (3.0) — undefined enum value stored" if abs(stored - 3.0) < 1e-4 else f"ALTERED to {stored}")
        log.info(_FMT, _CMD, "param1 = 3 (out of range)", outcome)
        report.record_tier1_detail("mission", _CMD, f"param1 (Land Options) = 3 (out of range): {outcome}")

    # ------------------------------------------------------------------
    # param3 (Approach Altitude)
    # ------------------------------------------------------------------

    async def test_nav_vtol_land_param3_approach_altitude_preserved(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param3 (Approach Altitude) = 40 m: preserved, or NACKed."""
        stored, nack = await self._stored(gcs_system_cls, home_item_for_mission, "param3", param3=40.0)
        outcome = f"NACKed ({nack})" if nack else ("PRESERVED" if abs(stored - 40.0) < 1e-3 else f"NOT preserved ({stored})")
        log.info(_FMT, _CMD, "param3 (Approach Altitude) = 40 m", outcome)
        report.record_tier1_detail("mission", _CMD, f"param3 (Approach Altitude) = 40 m: {outcome}")
        report.record_nonsentinel_ack("mission", _CMD, "3_Approach Altitude", nacked=nack is not None)
        if nack is None and abs(stored - 40.0) >= 1e-3:
            report.compat_fail(f"param3 (Approach Altitude) not preserved: uploaded 40.0, downloaded {stored} — "
                               "accepted but not stored, should have been NACKed")

    async def test_nav_vtol_land_param3_approach_altitude_nan(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param3 (Approach Altitude) = NaN ("unspecified") is accepted and not turned into a real altitude."""
        stored, nack = await self._stored(gcs_system_cls, home_item_for_mission, "param3", param3=NAN)
        if nack:
            pytest.fail(f"param3 = NaN (spec-defined 'unspecified') NACKed: {nack}")
        log.info(_FMT, _CMD, "param3 = NaN", "PRESERVED (NaN)" if math.isnan(stored) else f"ALTERED to {stored}")
        if not math.isnan(stored):
            report.compat_fail(f"param3 = NaN (approach altitude unspecified) stored as {stored} — "
                               "'unspecified' has become a specific altitude")

    # ------------------------------------------------------------------
    # param4 (Yaw)
    # ------------------------------------------------------------------

    async def test_nav_vtol_land_param4_yaw_preserved(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw) = 90°: preserved, or NACKed."""
        stored, nack = await self._stored(gcs_system_cls, home_item_for_mission, "param4", param4=90.0)
        outcome = f"NACKed ({nack})" if nack else ("PRESERVED" if abs(stored - 90.0) < 1e-3 else f"NOT preserved ({stored})")
        log.info(_FMT, _CMD, "param4 (Yaw) = 90°", outcome)
        report.record_nonsentinel_ack("mission", _CMD, "4_Yaw", nacked=nack is not None)
        if nack is None and abs(stored - 90.0) >= 1e-3:
            report.compat_fail(f"param4 (Yaw) not preserved: uploaded 90.0, downloaded {stored} — "
                               "accepted but not stored, should have been NACKed")

    async def test_nav_vtol_land_param4_yaw_nan(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw) = NaN ("use current yaw heading mode") is accepted and stored as NaN."""
        stored, nack = await self._stored(gcs_system_cls, home_item_for_mission, "param4", param4=NAN)
        if nack:
            pytest.fail(f"param4 = NaN (spec-defined sentinel) NACKed: {nack}")
        log.info(_FMT, _CMD, "param4 = NaN", "PRESERVED (NaN)" if math.isnan(stored) else f"ALTERED to {stored}")
        if not math.isnan(stored):
            report.compat_fail(f"param4 = NaN (use current heading mode) stored as {stored} — the sentinel's meaning is lost")

    async def test_nav_vtol_land_param4_yaw_edge_values(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw) = -90° and 450°: characterisation — raw, wrapped, or altered (the XML gives no range)."""
        seen = []
        for raw in (-90.0, 450.0):
            stored, nack = await self._stored(gcs_system_cls, home_item_for_mission, "param4", param4=raw)
            if nack:
                outcome = f"NACKed ({nack})"
            elif abs(stored - raw) < 1e-3:
                outcome = f"PRESERVED raw ({raw:.0f}°)"
            elif abs(stored - raw % 360) < 1e-3:
                outcome = f"WRAPPED to {raw % 360:.0f}°"
            else:
                outcome = f"ALTERED to {stored}"
            log.info(_FMT, _CMD, f"param4 = {raw:.0f}°", outcome)
            seen.append(f"{raw:.0f}°: {outcome}")
        report.record_tier1_detail("mission", _CMD, f"param4 (Yaw) edge values — {'; '.join(seen)}")

    # ------------------------------------------------------------------
    # Location (params 5/6/7)
    # ------------------------------------------------------------------

    async def test_nav_vtol_land_location_preserved(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """params 5/6/7 (Latitude/Longitude/Ground Altitude): an explicit location round-trips."""
        lat_int, lon_int, alt = _LAT_INT + 10000, _LON_INT + 10000, 12.0
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, x=lat_int, y=lon_int, z=alt)
        except MissionRawError as exc:
            pytest.fail(f"Explicit location NACKed: {_nack_reason(exc)}")
        finally:
            await clear_all_mission_types(gcs_system_cls)
        log.info(_FMT, _CMD, "params 5/6/7 (Lat/Lon/Alt)", f"stored x={dl.x} y={dl.y} z={dl.z:.1f}")
        for key in ("5_Latitude", "6_Longitude", "7_Ground Altitude"):
            report.record_nonsentinel_ack("mission", _CMD, key, nacked=False)
        if (dl.x, dl.y) != (lat_int, lon_int) or abs(dl.z - alt) > 1e-3:
            report.compat_fail(f"Location not preserved: uploaded ({lat_int}, {lon_int}, {alt}), "
                               f"downloaded ({dl.x}, {dl.y}, {dl.z}) — accepted but altered")

    async def test_nav_vtol_land_ground_altitude_nan(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param7 (Ground Altitude) = NaN ("use system default landing altitude") is accepted and stored as NaN."""
        stored, nack = await self._stored(gcs_system_cls, home_item_for_mission, "z", z=NAN)
        if nack:
            pytest.fail(f"param7 = NaN (spec-defined sentinel) NACKed: {nack}")
        log.info(_FMT, _CMD, "param7 = NaN", "PRESERVED (NaN)" if math.isnan(stored) else f"ALTERED to {stored}")
        if not math.isnan(stored):
            report.compat_fail(f"param7 = NaN (use system default landing altitude) stored as {stored} — "
                               "'ignore value' has become a specific altitude")
