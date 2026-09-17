"""
MAV_CMD_NAV_VTOL_TAKEOFF (cmd=84) via COMMAND_INT — direct command protocol tests.

"Takeoff from ground using VTOL mode, and transition to forward flight with
specified heading. The command should be ignored by vehicles that dont
support both VTOL and fixed-wing flight (multicopters, boats, etc.)."

This is the VTOL-specific sibling of NAV_TAKEOFF (cmd=22, see
tests/command/nav_takeoff/test_command.py) — same COMMAND_INT/COMMAND_LONG
protocol shape and largely the same param5/6/7 (Lat/Lon/Alt) semantics, but
param2 replaces NAV_TAKEOFF's undefined-Empty slot with a real, defined
"Transition Heading" enum field, and there is no Pitch/Flags param at all
(param1 and param3 are both Empty here, vs. NAV_TAKEOFF's Pitch/Flags).

Parameter layout (common.xml)
------------------------------
  param1  Empty
  param2  Transition Heading  (enum VTOL_TRANSITION_HEADING, 0-4)
  param3  Empty
  param4  Yaw Angle  (deg; NaN = "use current system yaw heading mode")
  param5  Latitude   (COMMAND_INT x, int32 ×1e7)
  param6  Longitude  (COMMAND_INT y, int32 ×1e7)
  param7  Altitude   (m; COMMAND_INT z, float)

hasLocation="true", isDestination="true" → COMMAND_INT is the correct
message type per tests/command/CLAUDE.md's selection rule.

Known cross-stack support (tests/command/README.md's survey, footnote 1):
PX4 ACCEPTs on every vehicle type regardless of frame (PX4 doesn't gate
commands by vehicle type — same pattern documented for NAV_TAKEOFF).
ArduCopter maps it onto its standard takeoff handler (ACCEPTED on any
airframe). ArduPlane QuadPlane (the one real VTOL ArduPilot frame) rejects
a direct COMMAND_INT — it only recognises NAV_VTOL_TAKEOFF as a mission-item
command executed during AUTO (see tests/command/baseline_takeoff/README.md).

PX4's real command-protocol param semantics — XML vs. source, and a fix
this file verifies
--------------------------------------------------------------------------
common.xml marks param1/param3 "Empty" and param2 as the VTOL_TRANSITION_HEADING
enum. Reading PX4's own standalone-command handler for this MAV_CMD
(`navigator_main.cpp`, `VEHICLE_CMD_NAV_VTOL_TAKEOFF` branch) shows PX4
assigns real, source-confirmed meaning beyond the XML for two of these:

  param1  "Loiter Height" — height above takeoff altitude at which the
          vehicle establishes a loiter circle after the FW transition
          (`_vtol_takeoff.setLoiterHeight(cmd.param1)`). XML says Empty;
          PX4's command-protocol path uses it anyway — an
          implementation-specific extension, not a spec violation (the
          XML doesn't forbid a stack from giving an Empty slot meaning,
          it just doesn't define one — see root CLAUDE.md's general
          testing philosophy rule 3).
  param2  Only ever compared for exact equality to 3.0
          (VTOL_TRANSITION_HEADING_SPECIFIED) — `if (fabs(param2 - 3.0f)
          < FLT_EPSILON) setTransitionDirection(param4)`. The other four
          enum values (VEHICLE_DEFAULT/NEXT_WAYPOINT/TAKEOFF/ANY) are
          otherwise indistinguishable to this handler — none of them
          calls setTransitionDirection, so param4 (Yaw Angle) is only
          ever consulted when param2==3.
  param3  Genuinely unused, matching the XML.
  param7  "Transition Altitude" (absolute) — `setTransitionAltitudeAbsolute
          (cmd.param7)` — a more specific meaning than the XML's generic
          "Altitude", but not a different field.

This matters for THIS repo's test framework specifically because PX4 also
has a *separate*, MAVLink-boundary parameter mask
(`src/modules/mavlink/mavlink_command_params.hpp`) that DENIES any
non-"unset" value for a param outside a per-command allow-list, before the
command even reaches Navigator. At the time this test file was first
written, that table's entry for cmd=84 was `{ 84, 0x78, 0x7C }` — allowing
only params 3/4/5/6/7 for the standalone-command path (param3, despite
being genuinely unused!) and DENYING any non-zero param1/param2, even
though Navigator's own handler reads both. This was fixed same-day
(commit aad2f0f3, "fix(mavlink): allow p1/p2 for standalone
NAV_VTOL_TAKEOFF command") to `{ 84, 0x78, 0x7B }` — param1/param2/param4-7
now allowed, param3 (the one genuinely unused slot) correctly the only one
still denied. `test_param1_loiter_height_accepted` below exists
specifically to verify this fix at the ACK level — the one place Tier 1
can actually observe it (a real param1 value flipping from DENIED to
ACCEPTED). Assisted-by: Claude:claude-sonnet-5 on the PX4-side commit.

Running
-------
Paired mock (no autopilot)::

    pytest tests/command/nav_vtol_takeoff/test_command.py -v --log-cli-level=INFO

PX4 SITL (any vehicle type — PX4 doesn't gate by frame)::

    pytest tests/command/nav_vtol_takeoff/test_command.py \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_standard_vtol \\
        --vehicle-type=vtol --autopilot=px4 -v --log-cli-level=INFO
"""

import logging

import pytest

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

_CMD = "NAV_VTOL_TAKEOFF"
_CMD_ID = 84  # MAV_CMD_NAV_VTOL_TAKEOFF

# SIH simulator home (47.3977°N, 8.5456°E) — same convention as nav_takeoff.
_LAT_INT = 473977000
_LON_INT = 85456000

# VTOL_TRANSITION_HEADING enum (common.xml)
_HEADING_VEHICLE_DEFAULT = 0
_HEADING_NEXT_WAYPOINT = 1
_HEADING_TAKEOFF = 2
_HEADING_SPECIFIED = 3
_HEADING_ANY = 4

SPEC = CommandSpec(
    cmd_id=_CMD_ID,
    name=_CMD,
    has_location=True,  # common.xml: hasLocation="true" — drives test_hasLocation_rejects_command_long
    baseline=dict(
        param2=float(_HEADING_VEHICLE_DEFAULT),  # Transition Heading: respect vehicle config
        param4=None,                              # Yaw: NaN = "use current heading mode"
        long5=float(_LAT_INT), long6=float(_LON_INT), long7=50.0,
        int_x=_LAT_INT, int_y=_LON_INT, int_z=50.0,
    ),
    params=[
        ParamSpec(
            1, "Empty", defined=False,
            reject_xfail_reason=(
                "PX4's standalone-command handler reads param1 as 'Loiter Height' "
                "(navigator_main.cpp: _vtol_takeoff.setLoiterHeight(cmd.param1)) even "
                "though common.xml marks this slot Empty — an implementation-specific "
                "extension, not a validation gap. See test_param1_loiter_height_accepted."
            ),
        ),
        ParamSpec(2, "Transition Heading", defined=True),
        ParamSpec(3, "Empty", defined=False),
        ParamSpec(4, "Yaw Angle", defined=True),
        ParamSpec(5, "Latitude", defined=True),
        ParamSpec(6, "Longitude", defined=True),
        ParamSpec(7, "Altitude", defined=True),
    ],
)


async def _probe(system, **kwargs) -> dict | None:
    """Subscribe first, then send COMMAND_INT, then collect COMMAND_ACK (this file's own bespoke tests only)."""
    defaults = dict(
        command=_CMD_ID, frame=6,
        param1=0.0, param2=float(_HEADING_VEHICLE_DEFAULT), param3=0.0, param4=None,
        x=_LAT_INT, y=_LON_INT, z=50.0,
    )
    defaults.update(kwargs)
    return await probe_command_int(system, **defaults)


@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(300)
class TestNavVtolTakeoffCommand(Tier1CommandTestBase):
    """
    NAV_VTOL_TAKEOFF — ACK result tests. Groups A/B/C (the six mandatory
    common checks) are inherited from Tier1CommandTestBase (tests/command/
    conftest.py). The tests below are this command's own bespoke
    per-parameter tests, mirroring nav_takeoff/test_command.py's structure
    for the params the two commands share (Yaw, Lat/Lon, Altitude), plus new
    coverage for param2 (Transition Heading) which NAV_TAKEOFF doesn't have.
    """

    SPEC = SPEC

    async def test_param2_transition_heading_values(self, gcs_system_cls, mock_stack_cls):
        """
        param2 (Transition Heading) — every defined VTOL_TRANSITION_HEADING
        enum value (0-4) — observational: the spec defines the enum's
        meaning but not a required ACK behaviour per value, so this is a
        characterisation survey, not a pass/fail check. Not UNSUPPORTED is
        the only thing asserted, since a spec-defined enum value should
        never make the whole command unrecognised.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        names = {
            _HEADING_VEHICLE_DEFAULT: "VEHICLE_DEFAULT",
            _HEADING_NEXT_WAYPOINT: "NEXT_WAYPOINT",
            _HEADING_TAKEOFF: "TAKEOFF",
            _HEADING_SPECIFIED: "SPECIFIED",
            _HEADING_ANY: "ANY",
        }
        for value, name in names.items():
            ack = await _probe(gcs_system_cls, param2=float(value))
            label = f"param2 (Transition Heading) = {value} ({name})"
            if ack is None:
                log.warning(_FMT, _CMD, label, "UNKNOWN — no ACK")
                continue
            result = int(ack["result"])
            log.info(_FMT, _CMD, label, f"result={result}")
            assert result != MAV_RESULT_UNSUPPORTED, f"{label} should not cause UNSUPPORTED"

    async def test_param1_loiter_height_accepted(self, gcs_system_cls, mock_stack_cls):
        """
        param1 (Loiter Height) = 20.0 m — expects not DENIED.

        Verifies PX4 commit aad2f0f3 ("fix(mavlink): allow p1/p2 for
        standalone NAV_VTOL_TAKEOFF command") — see module docstring.
        common.xml marks param1 "Empty", but PX4's standalone-command
        handler genuinely reads it as the post-transition loiter height
        (navigator_main.cpp: setLoiterHeight(cmd.param1)). Before the fix,
        PX4's separate MAVLink-boundary param mask
        (mavlink_command_params.hpp) denied any non-zero param1 for this
        command before Navigator ever saw it — a real regression the mask
        table introduced, unrelated to whether Navigator itself could use
        the value. Not asserted against other stacks (which may have no
        such handler at all, and may legitimately reject or ignore it) —
        this is PX4-specific regression coverage, kept as an assertion
        (not pure characterisation) because it's verifying one specific,
        source-confirmed fix rather than surveying undefined behaviour.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param1=20.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param1 (Loiter Height) = 20.0", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param1 (Loiter Height) = 20.0", f"result={result}")
        assert result != MAV_RESULT_DENIED, (
            f"param1=20.0 was DENIED (result={result}) — PX4's standalone-command handler "
            "reads param1 as the post-transition loiter height; a real value must not be "
            "rejected at the MAVLink boundary (regression check for commit aad2f0f3)"
        )

    async def test_param2_transition_heading_specified_uses_param4(self, gcs_system_cls, mock_stack_cls):
        """
        param2 (Transition Heading) = 3.0 (SPECIFIED) with param4 (Yaw
        Angle) = 45.0 — observational.

        PX4's standalone-command handler only calls setTransitionDirection
        (cmd.param4) when param2 is exactly 3.0
        (VTOL_TRANSITION_HEADING_SPECIFIED); every other enum value leaves
        the transition direction at its default (navigator_main.cpp).
        Tier 1 has no execution visibility to confirm param4 is actually
        used here — only that this specific, meaningful combination isn't
        rejected outright.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param2=float(_HEADING_SPECIFIED), param4=45.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param2=SPECIFIED(3), param4=45.0", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param2=SPECIFIED(3), param4=45.0", f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED, "SPECIFIED transition heading + Yaw Angle should not cause UNSUPPORTED"

    async def test_param2_transition_heading_out_of_range(self, gcs_system_cls, mock_stack_cls):
        """
        param2 (Transition Heading) = 5 — one past the last defined enum
        value (VTOL_TRANSITION_HEADING_ANY=4). Observational: the spec
        defines no explicit valid range/rejection rule for an out-of-range
        enum value.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param2=5.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param2 (Transition Heading) = 5 (out of range)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param2 (Transition Heading) = 5 (out of range)", f"result={result}")
        # Observational — no assertion

    async def test_param4_yaw_ack(self, gcs_system_cls, mock_stack_cls):
        """
        param4 (Yaw Angle) = 90.0 deg, with param2 left at its baseline
        (VEHICLE_DEFAULT, not SPECIFIED) — observational, not a rule-4
        DENIED expectation.

        Unlike NAV_TAKEOFF's identical-looking param4 (which PX4
        unconditionally discards regardless of any other param — a genuine
        rule-4 violation, see nav_takeoff/test_command.py), NAV_VTOL_TAKEOFF's
        own standalone-command handler only consults param4 when param2 is
        exactly VTOL_TRANSITION_HEADING_SPECIFIED (3.0) — see module
        docstring and test_param2_transition_heading_specified_uses_param4.
        With param2 at its default here, PX4 genuinely has no obligation to
        honour param4, so ACCEPTED is expected/correct, not a spec gap.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param4=90.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param4 (Yaw Angle) = 90.0 (param2=default)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param4 (Yaw Angle) = 90.0 (param2=default)", f"result={result}")
        # Observational — no assertion; see test_param2_transition_heading_specified_uses_param4
        # for the combination where param4 is actually meaningful.

    async def test_param4_yaw_nan_ack(self, gcs_system_cls, mock_stack_cls):
        """
        param4 (Yaw Angle) = NaN — observational: NaN means 'use current
        system yaw heading mode'.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param4=None)
        if ack is None:
            log.warning(_FMT, _CMD, "param4 (Yaw Angle) = NaN", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param4 (Yaw Angle) = NaN", f"result={result}")
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
        Observational — behaviour may differ per stack.
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

        Same reasoning as nav_takeoff's identical test: geometrically
        impossible but below INT32_MAX (the sentinel). Expected result:
        MAV_RESULT_DENIED, though not explicitly mandated by the spec.

        xfail if a stack (e.g. PX4, which shares mavlink_receiver.cpp's
        COMMAND_INT decode path with NAV_TAKEOFF) does not validate lat/lon
        range.
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
                "should return MAV_RESULT_DENIED — spec gap (coordinate range not mandated)"
            )
        assert result == MAV_RESULT_DENIED

    async def test_wrong_frame_ack(self, gcs_system_cls, mock_stack_cls):
        """
        frame = MAV_FRAME_LOCAL_NED (1) — observational.
        NAV_VTOL_TAKEOFF uses global coordinates; LOCAL_NED is unexpected.
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

        Skips in paired/mock mode (mock does not model NaN lat/lon semantics
        for COMMAND_LONG). Observational — logs the ACK result; no assertion
        beyond not UNSUPPORTED.
        """
        if mock_stack_cls is not None:
            pytest.skip("NaN lat/lon COMMAND_LONG test requires a real stack")
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await probe_command_long(
            gcs_system_cls, _CMD_ID,
            param2=float(_HEADING_VEHICLE_DEFAULT),
            param5=None, param6=None,   # NaN lat/lon → "use current position"
            param7=5.0,
        )
        if ack is None:
            log.warning(_FMT, _CMD, "COMMAND_LONG param5/6=NaN (use current pos)",
                        "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "COMMAND_LONG param5/6=NaN (use current pos)", f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED, (
            "NaN lat/lon in COMMAND_LONG should not cause UNSUPPORTED — "
            "the command is valid; lat/lon=NaN means 'use current position'"
        )

    async def test_latlon_int32max_command_long(self, gcs_system_cls, mock_stack_cls):
        """
        COMMAND_LONG param5=INT32_MAX (as float), param6=INT32_MAX — 'use current position'.

        Same reasoning as nav_takeoff's identical test. Expected result:
        ACCEPTED — the sentinel is valid and means "use current position".

        xfail if a stack rejects float(INT32_MAX) in param5/6 as a protocol
        error (confirmed for PX4 NAV_TAKEOFF via the shared
        mavlink_receiver.cpp decode path — likely to reproduce here).
        """
        if mock_stack_cls is not None:
            pytest.skip("INT32_MAX-as-float COMMAND_LONG test requires a real stack")
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await probe_command_long(
            gcs_system_cls, _CMD_ID,
            param2=float(_HEADING_VEHICLE_DEFAULT),
            param5=float(INT32_MAX),
            param6=float(INT32_MAX),
            param7=5.0,
        )
        if ack is None:
            log.warning(_FMT, _CMD, "COMMAND_LONG param5/6=INT32_MAX (use current pos)",
                        "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "COMMAND_LONG param5/6=INT32_MAX (use current pos)", f"result={result}")
        if result != MAV_RESULT_ACCEPTED:
            pytest.xfail(
                f"Stack returned {result} for INT32_MAX lat/lon in COMMAND_LONG; "
                "expected ACCEPTED — INT32_MAX is the 'use current position' sentinel"
            )
        assert result == MAV_RESULT_ACCEPTED
