"""
MAV_CMD_NAV_LAND (cmd=21) — Tier 1 protocol-acceptance tests (mission item).

"Land at location."

Tests whether the mission protocol accepts the command and stores each parameter
faithfully. Does NOT verify execution — see test_flight.py (Tier 2).

Built on tests/mission/conftest.py's Tier1MissionTestBase/MissionItemSpec: the
shared mandatory tests (baseline accepted, undefined-param sentinel pair,
defined-param sentinel tolerated, frame validation survey) are inherited; this
file adds NAV_LAND's own per-parameter tests.

Round-trip evidence is asymmetric: a value NOT preserved means the stack didn't
store it and should have NACKed it (rule 4 — a compatibility error); a value
preserved says nothing about whether execution uses it (that's Tier 2).

MAV_CMD_NAV_LAND parameter table (common.xml)
---------------------------------------------
  param1  Abort Alt (m)        minimum target altitude if landing is aborted (0 = undefined/use system default)
  param2  Land Mode            enum PRECISION_LAND_MODE (0 DISABLED, 1 OPPORTUNISTIC, 2 REQUIRED)
  param3  Empty
  param4  Yaw Angle (deg)      NaN = use the current system yaw heading mode
  param5  Latitude  (x, int × 1e7)
  param6  Longitude (y, int × 1e7)
  param7  Altitude  (z, float m)  landing altitude (ground level in current frame)

Expected from source (blind read, 2026-10-13 — root CLAUDE.md Tier 2 pattern #5)
--------------------------------------------------------------------------------
PX4 v1.18.0-beta1-660 (`mavlink_mission.cpp`, `mavlink_command_params.hpp`
`{ 21, 0x7B, 0x7B }`): params 1, 2, 4-7 allowed, a real param3 NACKed. Stores
land_precision (param2), yaw (param4, wrapped) and the location; abort altitude
(param1) is read into the item for fixed-wing use.

Running
-------
Against the mock (no autopilot required)::

    pytest tests/mission/nav_land/test_protocol.py -v --log-cli-level=INFO

PX4 SIH multicopter::

    pytest tests/mission/nav_land/test_protocol.py --drone-address=udp://:14540 \\
        --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx \\
        --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO
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

_CMD = "NAV_LAND"
_CMD_NAME = _CMD  # read by tests/report.py's key_from_module()
_CMD_ID = 21  # MAV_CMD_NAV_LAND
_FMT = "%-16s | %-44s | %s"

NAN = float("nan")
INT32_MAX = 0x7FFF_FFFF

# SIH simulator home (47.3977°N, 8.5456°E) — same convention as nav_takeoff.
_LAT_INT = 473977000
_LON_INT = 85456000

PRECISION_LAND_MODES = {0: "DISABLED", 1: "OPPORTUNISTIC", 2: "REQUIRED"}

SPEC = MissionItemSpec(
    cmd_id=_CMD_ID,
    name=_CMD,
    mission_type=0,
    frame=6,  # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    baseline=dict(
        param1=0.0,   # Abort Alt: 0 = use system default
        param2=0.0,   # Land Mode: DISABLED
        param3=0.0,   # Empty
        param4=NAN,   # Yaw: use the current heading mode
        x=_LAT_INT,
        y=_LON_INT,
        z=0.0,        # ground level, relative to home
    ),
    params=[
        ParamSpec(1, "Abort Alt", defined=True),
        ParamSpec(2, "Land Mode", defined=True),
        ParamSpec(3, "Empty", defined=False),
        ParamSpec(4, "Yaw Angle", defined=True),   # NaN sentinel IS spec-mandated here
        ParamSpec(5, "Latitude", defined=True),
        ParamSpec(6, "Longitude", defined=True),
        ParamSpec(7, "Altitude", defined=True),
    ],
)


def _nack_reason(exc: Exception) -> str:
    return str(exc).split(":")[0].strip()


@pytest.mark.asyncio(loop_scope="class")
class TestNavLand(Tier1MissionTestBase):
    """Protocol-acceptance tests for MAV_CMD_NAV_LAND (cmd=21) as a mission item."""

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
    # param1 (Abort Alt)
    # ------------------------------------------------------------------

    async def test_nav_land_param1_abort_alt_preserved(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param1 (Abort Alt) = 25 m: preserved, or NACKed."""
        stored, nack = await self._stored(gcs_system_cls, home_item_for_mission, "param1", param1=25.0)
        outcome = f"NACKed ({nack})" if nack else ("PRESERVED" if abs(stored - 25.0) < 1e-3 else f"NOT preserved ({stored})")
        log.info(_FMT, _CMD, "param1 (Abort Alt) = 25 m", outcome)
        report.record_tier1_detail("mission", _CMD, f"param1 (Abort Alt) = 25 m: {outcome}")
        report.record_nonsentinel_ack("mission", _CMD, "1_Abort Alt", nacked=nack is not None)
        if nack is None and abs(stored - 25.0) >= 1e-3:
            report.compat_fail(f"param1 (Abort Alt) not preserved: uploaded 25.0, downloaded {stored} — "
                               "accepted but not stored, should have been NACKed")

    # ------------------------------------------------------------------
    # param2 (Land Mode) — an enum: every defined value (rule 4c exemption)
    # ------------------------------------------------------------------

    async def test_nav_land_param2_land_mode_values(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param2 (Land Mode): every PRECISION_LAND_MODE value is preserved or NACKed, never silently altered.

        DISABLED (0) can't tell "stored" from "not stored" (a stack that drops param2
        returns 0 anyway); the other two can.
        """
        outcomes = {}
        for value, name in PRECISION_LAND_MODES.items():
            stored, nack = await self._stored(gcs_system_cls, home_item_for_mission, "param2", param2=float(value))
            outcomes[value] = f"NACKed ({nack})" if nack else ("PRESERVED" if abs(stored - value) < 1e-4 else f"ALTERED to {stored}")
            log.info(_FMT, _CMD, f"param2 = {value} ({name})", outcomes[value])
        summary = "; ".join(f"{v} {PRECISION_LAND_MODES[v]}: {o}" for v, o in outcomes.items())
        report.record_tier1_detail("mission", _CMD, f"param2 (Land Mode) per value — {summary}")
        report.record_nonsentinel_ack("mission", _CMD, "2_Land Mode",
                                      nacked=all(o.startswith("NACKed") for v, o in outcomes.items() if v != 0))
        altered = [PRECISION_LAND_MODES[v] for v, o in outcomes.items() if o.startswith("ALTERED")]
        if altered:
            report.compat_fail(f"param2 (Land Mode) accepted but not stored for {', '.join(altered)} — "
                               f"should have been NACKed: {summary}")

    async def test_nav_land_param2_land_mode_out_of_range(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param2 (Land Mode) = 5, outside the enum: characterisation, no assertion (rule 3)."""
        stored, nack = await self._stored(gcs_system_cls, home_item_for_mission, "param2", param2=5.0)
        outcome = f"NACKed ({nack})" if nack else ("PRESERVED (5.0) — undefined enum value stored" if abs(stored - 5.0) < 1e-4 else f"ALTERED to {stored}")
        log.info(_FMT, _CMD, "param2 = 5 (out of range)", outcome)
        report.record_tier1_detail("mission", _CMD, f"param2 (Land Mode) = 5 (out of range): {outcome}")

    # ------------------------------------------------------------------
    # param4 (Yaw Angle)
    # ------------------------------------------------------------------

    async def test_nav_land_param4_yaw_preserved(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw Angle) = 90°: preserved, or NACKed."""
        stored, nack = await self._stored(gcs_system_cls, home_item_for_mission, "param4", param4=90.0)
        outcome = f"NACKed ({nack})" if nack else ("PRESERVED" if abs(stored - 90.0) < 1e-3 else f"NOT preserved ({stored})")
        log.info(_FMT, _CMD, "param4 (Yaw Angle) = 90°", outcome)
        report.record_nonsentinel_ack("mission", _CMD, "4_Yaw Angle", nacked=nack is not None)
        if nack is None and abs(stored - 90.0) >= 1e-3:
            report.compat_fail(f"param4 (Yaw Angle) not preserved: uploaded 90.0, downloaded {stored} — "
                               "accepted but not stored, should have been NACKed")

    async def test_nav_land_param4_yaw_nan(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw Angle) = NaN ("use current yaw heading mode") is accepted and stored as NaN."""
        stored, nack = await self._stored(gcs_system_cls, home_item_for_mission, "param4", param4=NAN)
        if nack:
            pytest.fail(f"param4 = NaN (spec-defined sentinel) NACKed: {nack}")
        log.info(_FMT, _CMD, "param4 = NaN", "PRESERVED (NaN)" if math.isnan(stored) else f"ALTERED to {stored}")
        if not math.isnan(stored):
            report.compat_fail(f"param4 = NaN (use current heading mode) stored as {stored} — the sentinel's meaning is lost")

    async def test_nav_land_param4_yaw_edge_values(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw Angle) = -90° and 450°: characterisation — raw, wrapped, or altered (the XML gives no range)."""
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
        report.record_tier1_detail("mission", _CMD, f"param4 (Yaw Angle) edge values — {'; '.join(seen)}")

    # ------------------------------------------------------------------
    # Location (params 5/6/7)
    # ------------------------------------------------------------------

    async def test_nav_land_location_preserved(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """params 5/6/7 (Latitude/Longitude/Altitude): an explicit location round-trips."""
        lat_int, lon_int, alt = _LAT_INT + 10000, _LON_INT + 10000, 12.0
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, x=lat_int, y=lon_int, z=alt)
        except MissionRawError as exc:
            pytest.fail(f"Explicit location NACKed: {_nack_reason(exc)}")
        finally:
            await clear_all_mission_types(gcs_system_cls)
        log.info(_FMT, _CMD, "params 5/6/7 (Lat/Lon/Alt)", f"stored x={dl.x} y={dl.y} z={dl.z:.1f}")
        for key in ("5_Latitude", "6_Longitude", "7_Altitude"):
            report.record_nonsentinel_ack("mission", _CMD, key, nacked=False)
        if (dl.x, dl.y) != (lat_int, lon_int) or abs(dl.z - alt) > 1e-3:
            report.compat_fail(f"Location not preserved: uploaded ({lat_int}, {lon_int}, {alt}), "
                               f"downloaded ({dl.x}, {dl.y}, {dl.z}) — accepted but altered")

    async def test_nav_land_location_nan_altitude(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param7 (Altitude) = NaN: characterisation — the XML gives NAV_LAND's Altitude no sentinel meaning."""
        stored, nack = await self._stored(gcs_system_cls, home_item_for_mission, "z", z=NAN)
        outcome = f"NACKed ({nack})" if nack else ("PRESERVED (NaN)" if math.isnan(stored) else f"ALTERED to {stored}")
        log.info(_FMT, _CMD, "param7 = NaN", outcome)
        report.record_tier1_detail("mission", _CMD, f"param7 (Altitude) = NaN: {outcome}")
