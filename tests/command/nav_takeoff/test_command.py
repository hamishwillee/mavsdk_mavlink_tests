"""
MAV_CMD_NAV_TAKEOFF (cmd=22) via COMMAND_INT — direct command protocol tests.

This tests the *command protocol* path (COMMAND_INT → COMMAND_ACK), which is
distinct from the *mission protocol* path (MISSION_ITEM_INT upload → storage).
The same command's parameters may behave very differently across these two paths:

e.g. a stack may store param4 (Yaw) in a mission item yet not act on it when the
command is sent directly, or the reverse.

The command protocol only provides COMMAND_ACK result; it does NOT provide
access to stored parameter values.  Tests here are therefore limited to
documenting what ACK result each parameter value produces.  In particular they
never assume a defined param is ignored, or expect a particular result for a
valid value of one: a NACK is a legitimate "not supported" (root CLAUDE.md rule
4a), and an ACCEPTED value is checked for being honoured by test_flight.py.

COMMAND_INT is the correct message type for NAV_TAKEOFF:
  hasLocation="true", isDestination="true" → use COMMAND_INT (integer lat/lon).

Running
-------
Paired mock (no autopilot)::

    pytest tests/command/nav_takeoff/test_command.py -v --log-cli-level=INFO

Standalone::

    pytest tests/command/nav_takeoff/test_command.py --drone-address=udp://:14540 -v --log-cli-level=INFO
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
    probe_command_long,
)
from tests.mock_flight_stack import MAV_RESULT_ACCEPTED, MAV_RESULT_DENIED, MAV_RESULT_UNSUPPORTED

log = logging.getLogger(__name__)

_CMD = "NAV_TAKEOFF"
_CMD_ID = 22  # MAV_CMD_NAV_TAKEOFF
_CMD_NAME = _CMD  # read by tests/report.py's key_from_module()

# SIH simulator home (47.3977°N, 8.5456°E)
_LAT_INT = 473977000
_LON_INT = 85456000

# Parameter layout (common.xml): param1=Pitch, param2=Empty, param3=Flags
# (bitmask, defined despite having no dedicated MAVSDK-visible label text),
# param4=Yaw, param5/6=Latitude/Longitude (COMMAND_INT x/y), param7=Altitude.
SPEC = CommandSpec(
    cmd_id=_CMD_ID,
    name=_CMD,
    has_location=True,  # common.xml: hasLocation="true" — drives test_hasLocation_rejects_command_long
    baseline=dict(
        param1=0.0,     # Pitch: 0 deg (use default)
        param3=0.0,     # Flags: none
        param4=None,    # Yaw: NaN = "use current heading"
        long5=float(_LAT_INT), long6=float(_LON_INT), long7=50.0,
        int_x=_LAT_INT, int_y=_LON_INT, int_z=50.0,
    ),
    params=[
        ParamSpec(1, "Pitch", defined=True),
        ParamSpec(2, "Empty", defined=False),
        ParamSpec(3, "Flags", defined=True),
        ParamSpec(4, "Yaw", defined=True),
        ParamSpec(5, "Latitude", defined=True),
        ParamSpec(6, "Longitude", defined=True),
        ParamSpec(7, "Altitude", defined=True),
    ],
)


async def _probe(system, **kwargs) -> dict | None:
    """Subscribe first, then send COMMAND_INT, then collect COMMAND_ACK (this file's own bespoke tests only)."""
    defaults = dict(
        command=_CMD_ID, frame=6,
        param1=0.0, param2=0.0, param3=0.0, param4=None,
        x=_LAT_INT, y=_LON_INT, z=50.0,
    )
    defaults.update(kwargs)
    return await probe_command_int(system, **defaults)


async def _probe_real_value(system, label: str, param_key: str, **kwargs) -> bool | None:
    """
    Send a valid, non-sentinel value for one defined param and record its ACK.
    Returns True if it was NACKed, False if not, None if there was no ACK.

    Either outcome passes: DENIED is a legitimate "not supported" verdict, and
    any other result only says the value wasn't NACKed — whether it is honoured
    is test_flight.py's question (root CLAUDE.md rules 4a/10). The one thing a
    valid param value must not do is make a supported command UNSUPPORTED.

    Expecting a particular result (e.g. DENIED) would only be right if the
    command's XML said the param is not supported on this vehicle type.
    NAV_TAKEOFF's says no such thing for any param.

    The same send without the value goes first: if that baseline isn't accepted
    (e.g. ArduPilot DENIES every COMMAND_INT NAV_TAKEOFF in this frame while
    disarmed), a NACK says nothing about the param, so the test reports NA and
    records no verdict (rule 10: never record one the test didn't establish).
    """
    base = await _probe(system)
    base_result = None if base is None else int(base["result"])
    if base_result not in (MAV_RESULT_ACCEPTED, 5):  # 5 = IN_PROGRESS
        log.info(_FMT, _CMD, label, f"NA — the same send without this value isn't accepted (result={base_result})")
        pytest.skip(
            f"NA: the same send without this value isn't accepted (result={base_result}), "
            "so its ACK says nothing about the param"
        )
    ack = await _probe(system, **kwargs)
    if ack is None:
        log.warning(_FMT, _CMD, label, "UNKNOWN — no ACK")
        return None
    result = int(ack["result"])
    nacked = result == MAV_RESULT_DENIED
    log.info(
        _FMT, _CMD, label,
        f"result={result} — " + ("NACKed (not supported)" if nacked else "not NACKed (honoured? see test_flight.py)"),
    )
    report.record_nonsentinel_ack("command", _CMD, param_key, nacked=nacked)
    assert result != MAV_RESULT_UNSUPPORTED, f"{label} should not make a supported command UNSUPPORTED"
    return nacked


@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(300)
class TestNavTakeoffCommand(Tier1CommandTestBase):
    """
    NAV_TAKEOFF — ACK result tests. Groups A/B/C (the six mandatory common
    checks) are inherited from Tier1CommandTestBase (tests/command/conftest.py).
    The tests below are this command's own bespoke per-parameter tests,
    documenting COMMAND_INT execution-path behaviour that differs from the
    mission protocol's storage behaviour (see module docstring) — mostly
    single-message-type/observational, predating the dual-probe convention.
    """

    SPEC = SPEC

    async def test_param1_pitch_ack(self, gcs_system_cls, mock_stack_cls):
        """param1 (Pitch) = 15.0 deg — records whether a real value is NACKed; passes either way."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        await _probe_real_value(gcs_system_cls, "param1 (Pitch) = 15.0", "1_Pitch", param1=15.0)

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

    async def test_param4_yaw_ack(self, gcs_system_cls, mock_stack_cls):
        """param4 (Yaw) = 90.0 deg — records whether a real value is NACKed; passes either way."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        await _probe_real_value(gcs_system_cls, "param4 (Yaw) = 90.0", "4_Yaw", param4=90.0)

    async def test_param3_flags_ack(self, gcs_system_cls, mock_stack_cls):
        """
        param3 (Flags) = 1 (NAV_TAKEOFF_FLAGS_HORIZONTAL_POSITION_NOT_REQUIRED) — observational.

        The only defined bit: "accept the command even if the autopilot does not
        have control over its horizontal position". NACKed = a legitimate "not
        supported" verdict. Accepted = inconclusive at the ACK level, and the flag
        only changes behaviour when horizontal position control is absent, which
        this harness doesn't simulate — so it's recorded as untestable (param
        coverage requirement: every param ends a run with an explicit verdict).
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        nacked = await _probe_real_value(gcs_system_cls, "param3 (Flags) = 1", "3_Flags", param3=1.0)
        if nacked is False:
            report.record_compat_fact(
                "command", _CMD, "3_Flags",
                untestable="Flag only matters without horizontal position control",
            )

    async def test_param4_yaw_nan_ack(self, gcs_system_cls, mock_stack_cls):
        """
        param4 (Yaw) = NaN — observational: NaN means 'use current heading'.

        Per the MAVLink spec, param4=NaN is valid and means the stack should use
        its current yaw heading mode.
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

    async def test_location_out_of_range_latlon_ack(self, gcs_system_cls, mock_stack_cls):
        """
        x=1_200_000_000 (120°N), y=2_000_000_000 (200°E) — out-of-range lat/lon.

        Valid lat/lon ×1e7 ranges are −900_000_000..900_000_000 (lat) and
        −1_800_000_000..1_800_000_000 (lon).  These values are geometrically impossible
        but below INT32_MAX (the "use current position" sentinel at 2_147_483_647).

        The spec does not explicitly require rejection, but it should be inferred: a stack
        that accepts an impossible coordinate may navigate toward the wrong location or
        exhibit undefined behaviour.  Expected result: MAV_RESULT_DENIED.

        Known result (FAIL): PX4 does not validate lat/lon range and returns ACCEPTED (spec gap).
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        _OUT_LAT = 1_200_000_000   # 120°N — impossible latitude
        _OUT_LON = 2_000_000_000   # 200°E — impossible longitude
        ack = await _probe(gcs_system_cls, x=_OUT_LAT, y=_OUT_LON)
        if ack is None:
            log.warning(_FMT, _CMD, "params 5/6 out-of-range lat/lon", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "params 5/6 out-of-range lat/lon", f"result={result}")
        if result != MAV_RESULT_DENIED:
            # Spec gap — characterisation only (root CLAUDE.md rule 3), so
            # recorded, not asserted.
            log.info(
                _FMT, _CMD, "observation (spec gap, not asserted)",
                f"Stack accepted geometrically impossible lat/lon (result={result}); "
                "should return MAV_RESULT_DENIED — spec gap (coordinate range not mandated)",
            )

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

    async def test_latlon_nan_command_long_ack(self, gcs_system_cls, mock_stack_cls):
        """
        COMMAND_LONG param5=NaN, param6=NaN — "use current position" sentinel.

        NaN in COMMAND_LONG param5/6 is the float-field equivalent of INT32_MAX in
        COMMAND_INT x/y: it signals "use current vehicle position" rather than a
        specific coordinate.

        PX4 source (mavlink_receiver.cpp:513): COMMAND_LONG param5/6 are passed
        through to navigator_main.cpp as float.  The navigator's takeoff handler
        checks ``PX4_ISFINITE(cmd.param5) && PX4_ISFINITE(cmd.param6)``; when
        either is NaN (non-finite), it falls back to the current global position.

        Skips in paired/mock mode (mock does not model NaN lat/lon semantics for
        COMMAND_LONG; NaN values in x/y of COMMAND_INT are not representable since
        those fields are int32_t, so this test uses COMMAND_LONG specifically).

        Observational — logs the ACK result; no assertion beyond not UNSUPPORTED.
        """
        if mock_stack_cls is not None:
            pytest.skip("NaN lat/lon COMMAND_LONG test requires a real stack")
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        # param5=None, param6=None → encoded as JSON null → NaN on the wire
        ack = await probe_command_long(
            gcs_system_cls, _CMD_ID,
            param5=None, param6=None,   # NaN lat/lon → "use current position"
            param7=5.0,                  # relative altitude (m)
        )
        if ack is None:
            log.warning(_FMT, _CMD, "COMMAND_LONG param5/6=NaN (use current pos)",
                        "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "COMMAND_LONG param5/6=NaN (use current pos)",
                 f"result={result}  "
                 "(PX4: navigator falls back to current position when lat/lon is NaN; "
                 "ArduCopter: ACCEPTED, ignores lat/lon)")
        assert result != MAV_RESULT_UNSUPPORTED, (
            "NaN lat/lon in COMMAND_LONG should not cause UNSUPPORTED — "
            "the command is valid; lat/lon=NaN means 'use current position'"
        )

    async def test_latlon_int32max_command_long(self, gcs_system_cls, mock_stack_cls):
        """
        COMMAND_LONG param5=INT32_MAX (as float), param6=INT32_MAX — 'use current position'.

        INT32_MAX (2_147_483_647) is the MAVLink 'use current position' sentinel for
        lat/lon fields.  It applies to both COMMAND_INT (int32_t x/y) and COMMAND_LONG
        (float param5/6).  A float(INT32_MAX) in param5/6 means "use current position",
        not a protocol error.

        Expected result: ACCEPTED — the sentinel is valid and means "use current position".

        Known result (FAIL): PX4 explicitly rejects float(INT32_MAX) in param5/6 as a protocol error
        (mavlink_receiver.cpp:499–505), treating it as a miscoded COMMAND_INT.  This is
        a PX4 spec violation — the correct behaviour is to treat it as "use current position".
        """
        if mock_stack_cls is not None:
            pytest.skip("INT32_MAX-as-float COMMAND_LONG test requires a real stack")
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await probe_command_long(
            gcs_system_cls, _CMD_ID,
            param5=float(INT32_MAX),
            param6=float(INT32_MAX),
            param7=5.0,
        )
        if ack is None:
            log.warning(_FMT, _CMD, "COMMAND_LONG param5/6=INT32_MAX (use current pos)",
                        "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "COMMAND_LONG param5/6=INT32_MAX (use current pos)",
                 f"result={result}  "
                 "(INT32_MAX is 'use current position' sentinel; DENIED is a PX4 spec violation)")
        if result != MAV_RESULT_ACCEPTED:
            # Spec gap — characterisation only (root CLAUDE.md rule 3), so
            # recorded, not asserted.
            log.info(
                _FMT, _CMD, "observation (spec gap, not asserted)",
                f"Stack returned {result} for INT32_MAX lat/lon in COMMAND_LONG; "
                "expected ACCEPTED — INT32_MAX is the 'use current position' sentinel "
                "(PX4 incorrectly rejects it as a protocol error)",
            )
