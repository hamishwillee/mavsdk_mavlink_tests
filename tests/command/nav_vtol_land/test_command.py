"""
MAV_CMD_NAV_VTOL_LAND (cmd=85) via COMMAND_INT — direct command protocol tests.

"Land using VTOL mode"

The VTOL-specific sibling of NAV_LAND (cmd=21, tests/command/nav_land/) — same
param 4-7 layout (Yaw, Latitude, Longitude, ground Altitude), with Land Options
and Approach Altitude in place of NAV_LAND's Abort Alt and Land Mode.

Parameter layout (common.xml)
------------------------------
  param1  Land Options       (enum NAV_VTOL_LAND_OPTIONS: 0 DEFAULT, 1 FW_SPIRAL_APPROACH, 2 FW_APPROACH)
  param2  Empty
  param3  Approach Altitude  (m, same reference as the Altitude field; NaN if unspecified)
  param4  Yaw                (deg; NaN = use the current system yaw heading mode)
  param5  Latitude           (COMMAND_INT x, int32 ×1e7)
  param6  Longitude          (COMMAND_INT y, int32 ×1e7)
  param7  Ground Altitude    (m; COMMAND_INT z; NaN = use system default landing altitude)

hasLocation="true", isDestination="true" → COMMAND_INT is the correct message
type (tests/command/CLAUDE.md's selection rule).

Expected from source (blind read, 2026-10-13 — root CLAUDE.md Tier 2 pattern #5)
--------------------------------------------------------------------------------
PX4 v1.18.0-beta1-660: there is no handler for this command outside the mission
executor. `Commander::handle_command()` has no VEHICLE_CMD_NAV_VTOL_LAND case
and it isn't in the "handled elsewhere" list, so it falls to the default —
UNSUPPORTED — on every vehicle type; `mavlink_receiver.cpp`'s
`command_has_location()` lists it under "Not supported by PX4 as COMMAND_INT
(mission items or unimplemented)". (`mavlink_command_params.hpp` does carry a
command mask for it, `{ 85, 0x7C, 0x7F }` — every param allowed — which nothing
reaches.) So on PX4 every test below should skip after the baseline; the tests
exist for a stack, or a later PX4, that does implement it (Tier 2 pattern #1).

Running
-------
Paired mock (no autopilot)::

    pytest tests/command/nav_vtol_land/test_command.py -v --log-cli-level=INFO

PX4 SIH VTOL (the px4io/px4-sitl container build — see README.md)::

    pytest tests/command/nav_vtol_land/test_command.py --drone-address=udp://:14540 \\
        --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_standard_vtol \\
        --vehicle-type=vtol --autopilot=px4 -v --log-cli-level=INFO
"""

import logging

import pytest

from tests import report
from tests.report import _tier1_auto_record  # noqa: F401 — autouse: bespoke tests show in the report too
from tests.command.conftest import (
    CommandSpec,
    INT32_MAX,
    ParamSpec,
    Tier1CommandTestBase,
    _FMT,
    gcs_system_cls,  # noqa: F401 — see do_set_global_origin/test_command.py comment
    mock_stack_cls,  # noqa: F401
    probe_command_int,
)
from tests.mock_flight_stack import MAV_RESULT_UNSUPPORTED

log = logging.getLogger(__name__)

_CMD = "NAV_VTOL_LAND"
_CMD_ID = 85  # MAV_CMD_NAV_VTOL_LAND
_CMD_NAME = _CMD  # read by tests/report.py's _tier1_auto_record

# SIH simulator home (47.3977°N, 8.5456°E) — same convention as nav_takeoff.
_LAT_INT = 473977000
_LON_INT = 85456000

# NAV_VTOL_LAND_OPTIONS (common.xml)
LAND_OPTIONS = {0: "DEFAULT", 1: "FW_SPIRAL_APPROACH", 2: "FW_APPROACH"}

_ACCEPTED = (0, 5)

SPEC = CommandSpec(
    cmd_id=_CMD_ID,
    name=_CMD,
    has_location=True,  # common.xml: hasLocation="true" — drives test_hasLocation_rejects_command_long
    baseline=dict(
        param1=0.0,   # Land Options: DEFAULT
        param3=None,  # Approach Altitude: NaN = unspecified
        param4=None,  # Yaw: NaN = use the current heading mode
        long5=float(_LAT_INT), long6=float(_LON_INT), long7=0.0,
        int_x=_LAT_INT, int_y=_LON_INT, int_z=0.0,
    ),
    params=[
        ParamSpec(1, "Land Options", defined=True),
        ParamSpec(2, "Empty", defined=False),
        ParamSpec(3, "Approach Altitude", defined=True),
        ParamSpec(4, "Yaw", defined=True),
        ParamSpec(5, "Latitude", defined=True),
        ParamSpec(6, "Longitude", defined=True),
        ParamSpec(7, "Ground Altitude", defined=True),
    ],
)


async def _probe(system, **kwargs) -> int | None:
    """COMMAND_INT with the baseline plus overrides; the COMMAND_ACK result, or None (no ACK)."""
    fields = dict(command=_CMD_ID, frame=6, param1=0.0, param2=0.0, param3=None, param4=None,
                  x=_LAT_INT, y=_LON_INT, z=0.0)
    fields.update(kwargs)
    ack = await probe_command_int(system, **fields)
    return None if ack is None else int(ack["result"])


@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(300)
class TestNavVtolLandCommand(Tier1CommandTestBase):
    """
    NAV_VTOL_LAND — ACK result tests. The seven mandatory common checks are
    inherited from Tier1CommandTestBase (tests/command/conftest.py); the tests
    below are this command's own per-parameter tests.

    All probes are sent to a disarmed vehicle on the ground, so an ACK says only
    whether the value is refused outright — never whether it would be honoured
    (that's test_flight.py).
    """

    SPEC = SPEC

    async def _ack(self, system, mock_stack, label: str, key: str | None = None, **kw) -> int | None:
        """Probe one variant, log it, and (for a real value of a defined param) record the ACK verdict (rule 10a)."""
        await self._ensure_supported(system, mock_stack)
        result = await _probe(system, **kw)
        if result is None:
            log.warning(_FMT, _CMD, label, "UNKNOWN — no ACK")
            return None
        log.info(_FMT, _CMD, label, f"result={result}")
        if key is not None:
            report.record_nonsentinel_ack("command", _CMD, key, nacked=result not in _ACCEPTED)
        assert result != MAV_RESULT_UNSUPPORTED, f"{label} should not make the whole command UNSUPPORTED"
        return result

    async def test_nav_vtol_land_param1_land_options_values(self, gcs_system_cls, mock_stack_cls):
        """param1 (Land Options): every NAV_VTOL_LAND_OPTIONS value — characterisation; none may make the command UNSUPPORTED."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        results = {}
        for value, name in LAND_OPTIONS.items():
            results[value] = await _probe(gcs_system_cls, param1=float(value))
            log.info(_FMT, _CMD, f"param1 (Land Options) = {value} ({name})", f"result={results[value]}")
            assert results[value] != MAV_RESULT_UNSUPPORTED, f"Land Options {name} should not cause UNSUPPORTED"
        report.record_tier1_detail("command", _CMD, "param1 (Land Options) per value — "
                                   + "; ".join(f"{v} {LAND_OPTIONS[v]}: result={r}" for v, r in results.items()))
        real = [r for v, r in results.items() if v != 0 and r is not None]
        if real:
            report.record_nonsentinel_ack("command", _CMD, "1_Land Options", nacked=all(r not in _ACCEPTED for r in real))

    async def test_nav_vtol_land_param1_land_options_out_of_range(self, gcs_system_cls, mock_stack_cls):
        """param1 (Land Options) = 3, one past the last enum value — characterisation, no assertion (rule 3)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        result = await _probe(gcs_system_cls, param1=3.0)
        log.info(_FMT, _CMD, "param1 (Land Options) = 3 (out of range)", f"result={result}")

    async def test_nav_vtol_land_param3_approach_altitude_ack(self, gcs_system_cls, mock_stack_cls):
        """param3 (Approach Altitude) = 40 m — must not make the command UNSUPPORTED; a NACK is a legitimate "not supported"."""
        await self._ack(gcs_system_cls, mock_stack_cls, "param3 (Approach Altitude) = 40 m", "3_Approach Altitude", param3=40.0)

    async def test_nav_vtol_land_param4_yaw_ack(self, gcs_system_cls, mock_stack_cls):
        """param4 (Yaw) = 90° — must not make the command UNSUPPORTED; a NACK is a legitimate "not supported"."""
        await self._ack(gcs_system_cls, mock_stack_cls, "param4 (Yaw) = 90°", "4_Yaw", param4=90.0)

    async def test_nav_vtol_land_location_specific_ack(self, gcs_system_cls, mock_stack_cls):
        """params 5/6 (Lat/Lon) — a real location 100 m from home, ground altitude 0."""
        result = await self._ack(gcs_system_cls, mock_stack_cls, "params 5/6 (Lat/Lon) specific",
                                 x=_LAT_INT + 9000, y=_LON_INT)
        if result is not None:
            for key in ("5_Latitude", "6_Longitude"):
                report.record_nonsentinel_ack("command", _CMD, key, nacked=result not in _ACCEPTED)

    async def test_nav_vtol_land_location_int32max_ack(self, gcs_system_cls, mock_stack_cls):
        """params 5/6 = INT32_MAX together ("use current position") — characterisation."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        result = await _probe(gcs_system_cls, x=INT32_MAX, y=INT32_MAX)
        log.info(_FMT, _CMD, "params 5/6 (Lat/Lon) INT32_MAX", f"result={result}")

    async def test_nav_vtol_land_location_out_of_range_latlon_ack(self, gcs_system_cls, mock_stack_cls):
        """x = 120°N, y = 200°E — impossible coordinates. Characterisation: DENIED is the tidy answer, but the XML mandates no range."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        result = await _probe(gcs_system_cls, x=1_200_000_000, y=2_000_000_000)
        log.info(_FMT, _CMD, "params 5/6 out-of-range lat/lon", f"result={result}")
        if result in _ACCEPTED:
            log.info(_FMT, _CMD, "observation (spec gap, not asserted)",
                     "Stack accepted geometrically impossible lat/lon — coordinate range not mandated")

    async def test_nav_vtol_land_ground_altitude_ack(self, gcs_system_cls, mock_stack_cls):
        """param7 (Ground Altitude) = 5 m — must not make the command UNSUPPORTED; a NACK is a legitimate "not supported"."""
        await self._ack(gcs_system_cls, mock_stack_cls, "param7 (Ground Altitude) = 5 m", "7_Ground Altitude", z=5.0)

    async def test_nav_vtol_land_wrong_frame_ack(self, gcs_system_cls, mock_stack_cls):
        """frame = MAV_FRAME_LOCAL_NED (1) with global coordinates — characterisation."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        result = await _probe(gcs_system_cls, frame=1)
        log.info(_FMT, _CMD, "frame=LOCAL_NED(1)", f"result={result}")
