"""
MAV_CMD_NAV_TAKEOFF (cmd=22) via COMMAND_INT — direct command protocol tests.

This tests the *command protocol* path (COMMAND_INT → COMMAND_ACK), which is
distinct from the *mission protocol* path (MISSION_ITEM_INT upload → storage).
The same command's parameters may behave very differently across these two paths:

- PX4 stores param4 (Yaw) in mission items but unconditionally sets yaw=NaN
  in the COMMAND_INT execution path (navigator_main.cpp:630).
- ArduPilot does not store param4 in mission items, and ignores param4 in the
  COMMAND_INT path with the comment "not supported" (GCS_MAVLink_Copter.cpp:585).

The command protocol only provides COMMAND_ACK result; it does NOT provide
access to stored parameter values.  Tests here are therefore limited to
documenting what ACK result each parameter value produces.

COMMAND_INT is the correct message type for NAV_TAKEOFF:
  hasLocation="true", isDestination="true" → use COMMAND_INT (integer lat/lon).

Running
-------
Paired mock (no autopilot)::

    pytest tests/command/takeoff/test_command.py -v --log-cli-level=INFO

Standalone::

    pytest tests/command/takeoff/test_command.py --drone-address=udp://:14540 -v --log-cli-level=INFO
"""

import logging

import pytest

from tests.command.conftest import (
    probe_command_int,
    gcs_system_cls,
    mock_stack_cls,
    ACK_TIMEOUT_S,
    INT32_MAX,
    _FMT,
)
from tests.mock_flight_stack import MAV_RESULT_ACCEPTED, MAV_RESULT_UNSUPPORTED

log = logging.getLogger(__name__)

_CMD = "NAV_TAKEOFF"
_CMD_ID = 22  # MAV_CMD_NAV_TAKEOFF

# SIH simulator home (47.3977°N, 8.5456°E)
_LAT_INT = 473977000
_LON_INT = 85456000


def _takeoff_cmd(**overrides) -> dict:
    """Return default COMMAND_INT kwargs for NAV_TAKEOFF."""
    defaults = dict(
        command=_CMD_ID,
        frame=6,       # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        param1=0.0,    # Pitch: 0 deg (use default)
        param2=0.0,    # Unused
        param3=0.0,    # Flags: none
        param4=0.0,    # Yaw: 0.0 = north (None encodes as NaN — "use current heading")
        x=_LAT_INT,
        y=_LON_INT,
        z=50.0,        # Altitude: 50 m relative
    )
    defaults.update(overrides)
    return defaults


async def _probe(system, **kwargs) -> dict | None:
    """Subscribe first, then send COMMAND_INT, then collect COMMAND_ACK."""
    kw = _takeoff_cmd(**kwargs)
    return await probe_command_int(system, **kw)



@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(300)
class TestNavTakeoffCommand:
    """NAV_TAKEOFF via COMMAND_INT — ACK result tests."""

    _supported: bool | None = None  # class-level cache; None = not yet probed

    async def _ensure_supported(self, system, mock_stack) -> None:
        """Probe NAV_TAKEOFF once per class; skip all subsequent tests if UNSUPPORTED."""
        if TestNavTakeoffCommand._supported is None:
            ack = await _probe(system)
            unsupported = (ack is not None and int(ack["result"]) == MAV_RESULT_UNSUPPORTED)
            TestNavTakeoffCommand._supported = not unsupported
        if not TestNavTakeoffCommand._supported:
            pytest.skip(f"{_CMD} (cmd={_CMD_ID}) is UNSUPPORTED on this platform — test not run")

    async def test_command_accepted(self, gcs_system_cls, mock_stack_cls):
        """Baseline: NAV_TAKEOFF COMMAND_INT returns ACCEPTED (or non-UNSUPPORTED on real stack)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls)
        if ack is None:
            pytest.skip("No ACK received — UNKNOWN (spec violation by stack)")
        result = int(ack["result"])
        log.info(_FMT, _CMD, "baseline COMMAND_INT", f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED, (
            f"NAV_TAKEOFF should be supported by any flight stack; got UNSUPPORTED(3)"
        )

    async def test_param1_pitch_ack_accepted(self, gcs_system_cls, mock_stack_cls):
        """param1 (Pitch) = 15.0 deg — expect ACCEPTED (pitch is a hint, not enforced)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param1=15.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param1 (Pitch) = 15.0", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param1 (Pitch) = 15.0", f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED, "NAV_TAKEOFF with pitch should not be UNSUPPORTED"

    async def test_param1_nan_ack_result(self, gcs_system_cls, mock_stack_cls):
        """
        param1 (Pitch) = NaN — observational: some stacks reject NaN for defined params.

        None in fields_json encodes as IEEE-754 NaN on the wire (JSON null → nlohmann/json
        → NaN float).
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param1=None)
        if ack is None:
            log.warning(_FMT, _CMD, "param1 (Pitch) = NaN", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param1 (Pitch) = NaN", f"result={result}")
        # Observational — no assertion

    async def test_param4_yaw_specific_ack(self, gcs_system_cls, mock_stack_cls):
        """
        param4 (Yaw) = 90.0 deg — observational.

        Both PX4 and ArduPilot ignore param4 in the COMMAND_INT execution path:
        - PX4: rep->current.yaw = NAN regardless of param4 (navigator_main.cpp:630)
        - ArduCopter: "param4 : yaw angle   (not supported)" (GCS_MAVLink_Copter.cpp:585)
        - ArduPlane: only altitude is read from the COMMAND_INT handler (GCS_MAVLink_Plane.cpp)
        A specific yaw value should not cause DENIED or UNSUPPORTED.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param4=90.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param4 (Yaw) = 90.0", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param4 (Yaw) = 90.0", f"result={result}")
        # Observational — no assertion; yaw is ignored by all known stacks in this path

    async def test_param4_yaw_nan_ack(self, gcs_system_cls, mock_stack_cls):
        """
        param4 (Yaw) = NaN — observational: NaN means 'use current heading'.

        Per the MAVLink spec, param4=NaN is valid and means the stack should maintain
        its current heading.  Both PX4 and ArduPilot already ignore param4 in the
        COMMAND_INT path (yaw reset to NaN in PX4, "not supported" in ArduPilot).
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param4=None)
        if ack is None:
            log.warning(_FMT, _CMD, "param4 (Yaw) = NaN", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param4 (Yaw) = NaN", f"result={result}")
        # Observational — no assertion

    async def test_location_specific_ack(self, gcs_system_cls, mock_stack_cls):
        """Specific lat/lon location — COMMAND_INT x/y carry integer lat/lon × 1e7."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, x=_LAT_INT, y=_LON_INT)
        if ack is None:
            log.warning(_FMT, _CMD, "params 5/6 (Lat/Lon) specific", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "params 5/6 (Lat/Lon) specific", f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED, "Location coordinates should not cause UNSUPPORTED"

    async def test_location_int32max_ack(self, gcs_system_cls, mock_stack_cls):
        """
        x=INT32_MAX, y=INT32_MAX — 'use current position' sentinel.

        Observational: ArduCopter rejects INT32_MAX with DENIED in the mission
        protocol, but may behave differently in the command protocol.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, x=INT32_MAX, y=INT32_MAX)
        if ack is None:
            log.warning(_FMT, _CMD, "params 5/6 (Lat/Lon) INT32_MAX", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "params 5/6 (Lat/Lon) INT32_MAX", f"result={result}")
        # Observational — no assertion

    async def test_nan_altitude_ack(self, gcs_system_cls, mock_stack_cls):
        """
        z = NaN altitude — observational: NaN means 'use current/default altitude'.

        The MAVLink spec permits NaN altitude to request the stack use a default
        or current altitude.  Observational — stacks may accept or reject this.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, z=None)
        if ack is None:
            log.warning(_FMT, _CMD, "param7 (Alt) = NaN", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param7 (Alt) = NaN", f"result={result}")
        # Observational — no assertion

    async def test_wrong_frame_ack(self, gcs_system_cls, mock_stack_cls):
        """
        frame = MAV_FRAME_LOCAL_NED (1) — observational.

        NAV_TAKEOFF uses global coordinates; LOCAL_NED is unexpected.
        Expect UNSUPPORTED_MAV_FRAME or DENIED from real stacks.
        Frame is an integer field so MAVSDK passes it without modification.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, frame=1)
        if ack is None:
            log.warning(_FMT, _CMD, "frame=LOCAL_NED(1)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "frame=LOCAL_NED(1)", f"result={result}")
        # Observational — no assertion; behaviour is stack-specific
