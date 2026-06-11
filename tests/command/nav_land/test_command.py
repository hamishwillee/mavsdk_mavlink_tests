"""
MAV_CMD_NAV_LAND (cmd=21) via COMMAND_INT — direct command protocol tests.

This tests the *command protocol* path (COMMAND_INT → COMMAND_ACK).  There is
no mission-protocol counterpart test directory for NAV_LAND in this suite
(unlike NAV_TAKEOFF) — these are the only NAV_LAND tests.

NAV_LAND has hasLocation="true" isDestination="true" → COMMAND_INT is the
correct message type (integer lat/lon preserves coordinate precision).

Parameter layout (common.xml) is structurally similar to NAV_TAKEOFF:

    param1  Abort Alt    (m, 0 = undefined/use system default)
    param2  Land Mode    (enum PRECISION_LAND_MODE: 0/1/2)
    param3  Empty
    param4  Yaw Angle    (deg, NaN = use current system yaw heading mode)
    param5  Latitude
    param6  Longitude
    param7  Altitude     (m, "Landing altitude (ground level in current frame)")

IMPORTANT — what these tests can and cannot show
-------------------------------------------------
ACK-level tests can only observe whether a value was ACCEPTED / DENIED /
UNSUPPORTED.  They CANNOT determine *how* a parameter is used during
execution.  In particular:

- param7 (Altitude) is documented as "ground level in current frame" — a
  touchdown/ground reference, NOT a "fly to this altitude, then land" target
  (contrast with NAV_TAKEOFF, where z is the climb destination).  Whether a
  stack actually treats z as a ground-level reference for touchdown
  detection — and whether the commanded x/y/z represents the touchdown point
  or some other reference (e.g. an approach/finish point for fixed-wing) —
  is an execution-semantics question that requires flight observation
  (Tier 2, tests/command/nav_land/test_flight.py, not yet implemented).
- Whether the vehicle flies directly toward the landing point (diagonal
  descent) or flies there at current altitude and then descends (dogleg) is
  likewise a Tier 2 question.

Tier 1 tests here are therefore careful to assert only "was it accepted",
never "was it interpreted correctly" — see individual docstrings.

Running
-------
Paired mock (no autopilot)::

    pytest tests/command/nav_land/test_command.py -v --log-cli-level=INFO

Standalone::

    pytest tests/command/nav_land/test_command.py --drone-address=udp://:14540 -v --log-cli-level=INFO
"""

import logging

import pytest

from tests.command.conftest import (
    probe_command_int,
    probe_command_long,
    gcs_system_cls,
    mock_stack_cls,
    ACK_TIMEOUT_S,
    INT32_MAX,
    _FMT,
)
from tests.mock_flight_stack import MAV_RESULT_ACCEPTED, MAV_RESULT_DENIED, MAV_RESULT_UNSUPPORTED

log = logging.getLogger(__name__)

_CMD = "NAV_LAND"
_CMD_ID = 21  # MAV_CMD_NAV_LAND

# SIH simulator home (47.3977°N, 8.5456°E) — same as takeoff/do_reposition tests
_LAT_INT = 473977000
_LON_INT = 85456000

# PRECISION_LAND_MODE enum values (common.xml)
_PRECISION_LAND_MODE_DISABLED = 0
_PRECISION_LAND_MODE_OPPORTUNISTIC = 1
_PRECISION_LAND_MODE_REQUIRED = 2


def _land_cmd(**overrides) -> dict:
    """Return default COMMAND_INT kwargs for NAV_LAND."""
    defaults = dict(
        command=_CMD_ID,
        frame=6,       # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        param1=0.0,    # Abort Alt: 0 = use system default
        param2=float(_PRECISION_LAND_MODE_DISABLED),  # Land Mode: normal landing
        param3=0.0,    # Empty
        param4=None,   # Yaw: NaN = "use current heading mode" (non-NaN asserts yaw intent)
        x=_LAT_INT,
        y=_LON_INT,
        z=0.0,         # Altitude: ground level (relative-alt frame ⇒ 0 = home level)
    )
    defaults.update(overrides)
    return defaults


async def _probe(system, **kwargs) -> dict | None:
    """Subscribe first, then send COMMAND_INT, then collect COMMAND_ACK."""
    kw = _land_cmd(**kwargs)
    return await probe_command_int(system, **kw)


@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(300)
class TestNavLandCommand:
    """NAV_LAND via COMMAND_INT — ACK result tests."""

    _supported: bool | None = None  # class-level cache; None = not yet probed

    async def _ensure_supported(self, system, mock_stack) -> None:
        """Probe NAV_LAND once per class; skip all subsequent tests if UNSUPPORTED."""
        if TestNavLandCommand._supported is None:
            ack = await _probe(system)
            unsupported = (ack is not None and int(ack["result"]) == MAV_RESULT_UNSUPPORTED)
            TestNavLandCommand._supported = not unsupported
        if not TestNavLandCommand._supported:
            pytest.skip(f"{_CMD} (cmd={_CMD_ID}) is UNSUPPORTED on this platform — test not run")

    # -----------------------------------------------------------------------
    # Group A — Baseline
    # -----------------------------------------------------------------------

    async def test_command_accepted(self, gcs_system_cls, mock_stack_cls):
        """Baseline: NAV_LAND COMMAND_INT returns ACCEPTED (or non-UNSUPPORTED on real stack)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls)
        if ack is None:
            pytest.skip("No ACK received — UNKNOWN (spec violation by stack)")
        result = int(ack["result"])
        log.info(_FMT, _CMD, "baseline COMMAND_INT", f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED, (
            f"NAV_LAND should be supported by any aerial flight stack; got UNSUPPORTED(3)"
        )

    # -----------------------------------------------------------------------
    # Group B — param1 (Abort Altitude; 0 = undefined/use system default)
    # -----------------------------------------------------------------------

    async def test_param1_abort_alt_zero(self, gcs_system_cls, mock_stack_cls):
        """param1=0.0 — spec-defined 'undefined/use system default' sentinel. Must not be UNSUPPORTED."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param1=0.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param1 (Abort Alt) = 0.0 (use default)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param1 (Abort Alt) = 0.0 (use default)", f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED

    async def test_param1_abort_alt_specific(self, gcs_system_cls, mock_stack_cls):
        """
        param1=10.0 m — a specific minimum abort altitude.

        Observational: whether the stack actually uses this value during a
        go-around is an execution question (Tier 2); here we only check
        whether stating an explicit preference is accepted.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param1=10.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param1 (Abort Alt) = 10.0 m", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param1 (Abort Alt) = 10.0 m", f"result={result}")
        # Observational — no assertion

    async def test_param1_abort_alt_negative(self, gcs_system_cls, mock_stack_cls):
        """
        param1=-5.0 m — negative abort altitude; not a meaningful value.

        The spec defines 0 as the only sentinel; negative altitudes are
        physically meaningless for "minimum altitude if landing is aborted".
        Observational — stacks may reject, clamp, or ignore.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param1=-5.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param1 (Abort Alt) = -5.0 m (invalid)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param1 (Abort Alt) = -5.0 m (invalid)", f"result={result}")
        # Observational — no assertion

    async def test_param1_abort_alt_nan(self, gcs_system_cls, mock_stack_cls):
        """
        param1=NaN — observational; the spec does NOT define NaN for this field.

        Unlike param4 (Yaw), where NaN is explicitly documented as a valid
        "use current heading" sentinel, param1's only documented sentinel is
        0 ("undefined/use system default"). A stack returning DENIED for NaN
        here would NOT be a spec violation — but nor would ACCEPTED (treating
        NaN as equivalent to 0, per the general "NaN = no preference"
        convention for optional float params).  This is genuinely ambiguous;
        see README "Spec gaps" — the spec should clarify whether NaN is
        equivalent to 0 for "0 = use default" numeric params generally.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param1=None)
        if ack is None:
            log.warning(_FMT, _CMD, "param1 (Abort Alt) = NaN", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param1 (Abort Alt) = NaN", f"result={result}  "
                 "(NaN sentinel undefined for this field — neither ACCEPTED nor DENIED "
                 "would be a spec violation)")
        # Observational — no assertion; feeds the spec-gaps writeup, not an xfail

    # -----------------------------------------------------------------------
    # Group C — param2 (Land Mode; PRECISION_LAND_MODE enum)
    # -----------------------------------------------------------------------

    async def test_param2_land_mode_disabled(self, gcs_system_cls, mock_stack_cls):
        """param2=PRECISION_LAND_MODE_DISABLED (0) — normal (non-precision) landing. Baseline value."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param2=float(_PRECISION_LAND_MODE_DISABLED))
        if ack is None:
            log.warning(_FMT, _CMD, "param2 (Land Mode) = DISABLED (0)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param2 (Land Mode) = DISABLED (0)", f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED

    async def test_param2_land_mode_opportunistic(self, gcs_system_cls, mock_stack_cls):
        """
        param2=PRECISION_LAND_MODE_OPPORTUNISTIC (1) — use precision landing if a
        beacon is detected, otherwise land normally.

        Observational: whether the stack actually attempts precision landing
        cannot be determined from the ACK alone — it requires a simulated
        beacon/IRLOCK target (not currently configured; see README "Future
        work"). Here we only check that stating this preference is accepted.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param2=float(_PRECISION_LAND_MODE_OPPORTUNISTIC))
        if ack is None:
            log.warning(_FMT, _CMD, "param2 (Land Mode) = OPPORTUNISTIC (1)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param2 (Land Mode) = OPPORTUNISTIC (1)", f"result={result}")
        # Observational — no assertion

    async def test_param2_land_mode_required(self, gcs_system_cls, mock_stack_cls):
        """
        param2=PRECISION_LAND_MODE_REQUIRED (2) — require precision landing,
        searching for a beacon (land normally if none found).

        Observational, for the same reason as OPPORTUNISTIC above: behavioural
        verification needs a simulated beacon and is out of scope for ACK-level
        tests.  A stack without precision-landing hardware support could
        reasonably DENY this — that would itself be a useful, documentable
        finding (not asserted here, since it's airframe/config dependent).
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param2=float(_PRECISION_LAND_MODE_REQUIRED))
        if ack is None:
            log.warning(_FMT, _CMD, "param2 (Land Mode) = REQUIRED (2)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param2 (Land Mode) = REQUIRED (2)", f"result={result}")
        # Observational — no assertion

    async def test_param2_land_mode_undefined(self, gcs_system_cls, mock_stack_cls):
        """
        param2=5.0 — value outside the PRECISION_LAND_MODE enum (0/1/2 defined).

        The spec does not state what a stack must do with an undefined enum
        value.  Observational — mirrors do_reposition's test_param2_undefined_bits;
        a stack might DENY (correct, intent cannot be honoured), clamp to a
        nearby defined value, or silently ignore (ACCEPTED).
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param2=5.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param2 (Land Mode) = 5 (undefined enum value)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param2 (Land Mode) = 5 (undefined enum value)", f"result={result}")
        # Observational — no assertion

    # -----------------------------------------------------------------------
    # Group D — param4 (Yaw Angle; NaN = use current system yaw heading mode)
    # -----------------------------------------------------------------------

    async def test_param4_yaw_specific_ack(self, gcs_system_cls, mock_stack_cls):
        """
        param4=90.0 deg — a specific desired landing heading.

        Unlike NAV_TAKEOFF (where prior survey runs showed every stack ignores
        param4), we have no prior evidence here — facing a specific direction
        on touchdown (e.g. into wind, or to present a sensor/cargo bay) is
        plausibly something a stack *would* honour.  Observational for now;
        if testing shows the value is silently ignored, this should be
        converted to an assertion (MAV_RESULT_DENIED, xfail) per the
        "unsupported params must NACK" convention documented in
        tests/command/nav_takeoff/README.md.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param4=90.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param4 (Yaw) = 90.0", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param4 (Yaw) = 90.0", f"result={result}")
        # Observational — no assertion (see docstring: revisit once behaviour is known)

    async def test_param4_yaw_nan_ack(self, gcs_system_cls, mock_stack_cls):
        """
        param4=NaN — observational: the documented "use current system yaw
        heading mode" sentinel (e.g. yaw towards next waypoint, yaw to home).
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param4=None)
        if ack is None:
            log.warning(_FMT, _CMD, "param4 (Yaw) = NaN", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param4 (Yaw) = NaN", f"result={result}")
        # Observational — no assertion

    # -----------------------------------------------------------------------
    # Group E — Location (params 5/6/7: Lat/Lon/Altitude)
    # -----------------------------------------------------------------------

    async def test_location_specific_ack(self, gcs_system_cls, mock_stack_cls):
        """Specific lat/lon at SIH home — valid coordinates. Must not be UNSUPPORTED."""
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

        Observational: for a landing command this most plausibly means "land
        where you currently are" (descend in place) rather than "land at an
        unspecified destination".  Whether the stack actually treats it that
        way is an execution question (Tier 2); here we only check the ACK.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, x=INT32_MAX, y=INT32_MAX)
        if ack is None:
            log.warning(_FMT, _CMD, "params 5/6 (Lat/Lon) INT32_MAX", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "params 5/6 (Lat/Lon) INT32_MAX", f"result={result}")
        # Observational — no assertion

    async def test_location_out_of_range_latlon_ack(self, gcs_system_cls, mock_stack_cls):
        """
        x=1_200_000_000 (120°N), y=2_000_000_000 (200°E) — out-of-range lat/lon.

        Same reasoning as tests/command/nav_takeoff/test_command.py
        test_location_out_of_range_latlon_ack: these are geometrically
        impossible coordinates below the INT32_MAX sentinel.  Expected:
        MAV_RESULT_DENIED.

        xfail if a stack accepts them — same PX4 spec gap already tracked for
        NAV_TAKEOFF (PX4 does not validate lat/lon range).
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
            pytest.xfail(
                f"Stack accepted geometrically impossible lat/lon (result={result}); "
                "should return MAV_RESULT_DENIED — spec gap (coordinate range not mandated), "
                "same as tracked for NAV_TAKEOFF"
            )
        assert result == MAV_RESULT_DENIED

    async def test_altitude_specific_ack(self, gcs_system_cls, mock_stack_cls):
        """
        z=5.0 m — a specific non-zero landing/ground-level altitude.

        IMPORTANT: this only checks that the value is ACCEPTED at the ACK
        level.  param7 here is documented as "ground level in current frame"
        — a touchdown reference, not a "climb to this altitude" destination
        like NAV_TAKEOFF's z.  Whether the stack actually uses it as a
        ground-level reference for touchdown detection (e.g. landing on a
        platform/hill where ground level differs from home elevation) is an
        execution-semantics question that requires flight observation —
        see Tier 2 proposal in README ("Landing point identity").
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, z=5.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param7 (Alt/ground-level) = 5.0 m", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param7 (Alt/ground-level) = 5.0 m", f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED, "A ground-level altitude should not cause UNSUPPORTED"

    async def test_altitude_nan_ack(self, gcs_system_cls, mock_stack_cls):
        """
        z=NaN — observational.

        Unlike NAV_TAKEOFF (where the spec permits NaN altitude to mean "use
        default altitude"), the NAV_LAND spec text for param7 ("Landing
        altitude (ground level in current frame)") does NOT document a NaN
        meaning at all.  Whether NaN here means "use current/measured ground
        level", "use system default", or is simply invalid is undefined —
        see README "Spec gaps".  Purely observational; no assertion.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, z=None)
        if ack is None:
            log.warning(_FMT, _CMD, "param7 (Alt/ground-level) = NaN", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param7 (Alt/ground-level) = NaN", f"result={result}  "
                 "(NaN meaning undefined for this field — see Spec gaps)")
        # Observational — no assertion

    async def test_wrong_frame_ack(self, gcs_system_cls, mock_stack_cls):
        """
        frame = MAV_FRAME_LOCAL_NED (1) — observational.

        NAV_LAND uses global coordinates; LOCAL_NED is unexpected.
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

    # -----------------------------------------------------------------------
    # Group F — COMMAND_LONG sentinel tests (require real stack)
    # -----------------------------------------------------------------------

    async def test_latlon_nan_command_long_ack(self, gcs_system_cls, mock_stack_cls):
        """
        COMMAND_LONG param5=NaN, param6=NaN — "use current position" sentinel.

        Mirrors tests/command/nav_takeoff/test_command.py
        test_latlon_nan_command_long_ack.  NaN in COMMAND_LONG param5/6 is the
        float-field equivalent of INT32_MAX in COMMAND_INT x/y.

        Skips in paired/mock mode (mock does not model NaN lat/lon semantics
        for COMMAND_LONG; NaN values are not representable in COMMAND_INT's
        int32_t x/y fields, so this test specifically uses COMMAND_LONG).

        Observational — logs the ACK result; no assertion beyond not UNSUPPORTED.
        """
        if mock_stack_cls is not None:
            pytest.skip("NaN lat/lon COMMAND_LONG test requires a real stack")
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await probe_command_long(
            gcs_system_cls, _CMD_ID,
            param5=None, param6=None,   # NaN lat/lon → "use current position"
            param7=0.0,                  # ground-level altitude
        )
        if ack is None:
            log.warning(_FMT, _CMD, "COMMAND_LONG param5/6=NaN (use current pos)",
                        "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "COMMAND_LONG param5/6=NaN (use current pos)",
                 f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED, (
            "NaN lat/lon in COMMAND_LONG should not cause UNSUPPORTED — "
            "the command is valid; lat/lon=NaN means 'use current position'"
        )

    async def test_latlon_int32max_command_long(self, gcs_system_cls, mock_stack_cls):
        """
        COMMAND_LONG param5=INT32_MAX (as float), param6=INT32_MAX — 'use current position'.

        Mirrors tests/command/nav_takeoff/test_command.py
        test_latlon_int32max_command_long_denied.  Expected result: ACCEPTED
        — the sentinel is valid and means "use current position".

        xfail: PX4 explicitly rejects float(INT32_MAX) in param5/6 as a
        protocol error (mavlink_receiver.cpp:499–505), treating it as a
        miscoded COMMAND_INT — same bug already tracked for NAV_TAKEOFF.
        """
        if mock_stack_cls is not None:
            pytest.skip("INT32_MAX-as-float COMMAND_LONG test requires a real stack")
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await probe_command_long(
            gcs_system_cls, _CMD_ID,
            param5=float(INT32_MAX),
            param6=float(INT32_MAX),
            param7=0.0,
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
            pytest.xfail(
                f"Stack returned {result} for INT32_MAX lat/lon in COMMAND_LONG; "
                "expected ACCEPTED — INT32_MAX is the 'use current position' sentinel "
                "(PX4 incorrectly rejects it as a protocol error, same as NAV_TAKEOFF)"
            )
        assert result == MAV_RESULT_ACCEPTED
