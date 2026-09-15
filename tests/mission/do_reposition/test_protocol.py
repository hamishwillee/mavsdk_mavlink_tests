"""
MAV_CMD_DO_REPOSITION (cmd=192) — Tier 1 mission-protocol acceptance tests.

DO_REPOSITION is documented as a *guided*-command: "This command is intended
for guided commands (for missions use MAV_CMD_NAV_WAYPOINT instead)."  It is
nonetheless `hasLocation="true" isDestination="true"`, which makes it
syntactically valid as a MISSION_ITEM_INT.  Whether a flight stack's mission
storage code accepts it — and whether it preserves each parameter — is not
defined by the spec and must be probed empirically.

Built on tests/mission/conftest.py's Tier1MissionTestBase/MissionItemSpec —
the shared mandatory Tier 1 tests (baseline-accepted, undefined-param
sentinel pair, defined-param sentinel-tolerated) are inherited; this file
only adds DO_REPOSITION's own bespoke per-parameter tests (enum/bitmask/
range/location semantics are not auto-generated — see
tests/mission/CLAUDE.md § MAV_CMD support testing).

Why this tier emphasises "expected unsupported" + sentinel probes
------------------------------------------------------------------
Uploading a value that is *expected to be valid* and seeing it round-trip is
ambiguous evidence (the stack may simply store-and-forget without validating).
Uploading a value that the spec marks invalid (out of range, wrong sign, an
undefined bitmask bit, etc.) is a much sharper probe:
  - NACKed   → the stack actively validates this parameter — strong signal.
  - Accepted → either the stack does not validate at storage time (the value
               may still be rejected/clamped at execution), or this stack's
               interpretation of the spec differs.  Ambiguous, but documents
               the as-observed behaviour for the Tier 2 mission to build on.
Sentinel values (NaN "use default/current", -1 "use default speed", INT32_MAX
"use current position") are the spec-defined edge of the valid range and are
tested for the same reason — they are the values a real GCS would send to
invoke "default" behaviour, and the spec requires them to be honoured.

MAV_CMD_DO_REPOSITION parameter table (common.xml)
--------------------------------------------------
  param1  Speed   (m/s; minValue=-1; <0 (-1) = use default)        Defined
  param2  Bitmask (MAV_DO_REPOSITION_FLAGS)                        Defined (bitmask)
  param3  Radius  (m; planes only; positive only; 0/NaN = ignored) Defined
  param4  Yaw     (RADIANS; NaN = use current heading mode;        Defined
                   for planes: loiter direction 0=CW, 1=CCW — see
                   DOC DISCREPANCY note in test_protocol_param4_yaw_specific)
  param5  Latitude  (x field, int x 1e7)                           Location
  param6  Longitude (y field, int x 1e7)                           Location
  param7  Altitude  (z field, float m)                             Location

MAV_DO_REPOSITION_FLAGS:
  bit 0 (value 1) = MAV_DO_REPOSITION_FLAGS_CHANGE_MODE   — switch to guided/hold immediately
  bit 1 (value 2) = MAV_DO_REPOSITION_FLAGS_RELATIVE_YAW  — yaw relative to current heading

NOTE on units: unlike MAV_CMD_NAV_TAKEOFF (param4 in DEGREES), DO_REPOSITION's
param4 (Yaw) is documented in RADIANS.  Test values below use radians.

Results are per (autopilot, vehicle type) — a multicopter result does not imply
the same behaviour on fixed-wing or VTOL (param3/Radius and the plane-specific
meaning of param4 are explicitly plane-only).

Running
-------
Against the mock (no autopilot required)::

    pytest tests/mission/do_reposition/test_protocol.py -v --log-cli-level=INFO

Against a real flight stack::

    pytest tests/mission/do_reposition/test_protocol.py --drone-address=udp://:14540 -v --log-cli-level=INFO
"""

import logging
import math

import pytest
from mavsdk.mission_raw import MissionRawError

from ..conftest import MissionItemSpec, Tier1MissionTestBase, clear_all_mission_types
from tests.param_spec import ParamSpec

log = logging.getLogger(__name__)

_CMD = "DO_REPOSITION"
_FMT = "%-14s | %-44s | %s"

NAN = float("nan")
INT32_MAX = 0x7FFF_FFFF

# MAV_DO_REPOSITION_FLAGS bitmask values (from common.xml)
_FLAG_CHANGE_MODE  = 1   # bit 0
_FLAG_RELATIVE_YAW = 2   # bit 1

# SIH simulator home coordinates (47.3977°N, 8.5456°E) — same as takeoff tests
_LAT_INT = 473977000
_LON_INT = 85456000

# ---------------------------------------------------------------------------
# Tier 1 spec
# ---------------------------------------------------------------------------
#
# param4 default is **0.0, not the spec-correct NaN** ("use current
# heading"): ArduPilot's `AP_Mission::sanity_check_params()` only permits NaN
# in the params of commands it explicitly recognises (NAV_WAYPOINT,
# NAV_TAKEOFF, ...).  DO_REPOSITION is not in that list, so its blanket
# `nan_mask = 0xff` rejects NaN in *any* of params 1-4 with
# `MAV_MISSION_INVALID_PARAM4` — before the command-id switch that would
# otherwise report `MAV_MISSION_UNSUPPORTED`.  Using 0.0 here keeps the
# baseline probe focused on "is DO_REPOSITION accepted at all"; the NaN
# behaviour itself is characterised separately in test_protocol_param4_yaw_nan
# (same workaround pattern as nav_takeoff's param2 — see
# tests/mission/nav_takeoff/test_protocol.py::_takeoff_item).

SPEC = MissionItemSpec(
    cmd_id=192,
    name=_CMD,
    mission_type=0,
    frame=6,  # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    baseline=dict(
        param1=-1.0,   # Speed: -1 = use default
        param2=0.0,    # Bitmask: no flags
        param3=0.0,    # Radius: 0 = ignored (per spec)
        param4=0.0,    # Yaw: 0.0 — NOT the spec-correct NaN; see note above
        x=_LAT_INT + 20000,   # ~0.002 deg north of SIH home — distinct from 0/home
        y=_LON_INT + 20000,
        z=30.0,
    ),
    params=[
        ParamSpec(1, "Speed", defined=True),
        ParamSpec(2, "Bitmask", defined=True),
        ParamSpec(3, "Radius", defined=True),
        ParamSpec(4, "Yaw", defined=True),
        ParamSpec(5, "Latitude", defined=True),
        ParamSpec(6, "Longitude", defined=True),
        ParamSpec(7, "Altitude", defined=True),
    ],
)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


@pytest.mark.asyncio(loop_scope="class")
class TestDoReposition(Tier1MissionTestBase):
    """Protocol-acceptance tests for MAV_CMD_DO_REPOSITION (cmd=192) as a mission item."""

    SPEC = SPEC

    # ------------------------------------------------------------------
    # param1 (Speed; minValue=-1; <0/-1 = "use default")
    # ------------------------------------------------------------------

    async def test_protocol_param1_speed_preserved(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param1 (Speed) = 8.0 m/s: a valid in-range value round-trips correctly."""
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param1=8.0)
            if abs(dl.param1 - 8.0) < 1e-4:
                log.info(_FMT, _CMD, "param1 (Speed) 8.0", f"PRESERVED ({dl.param1:.4f}) — probably supported")
            else:
                log.warning(_FMT, _CMD, "param1 (Speed) 8.0",
                            f"ALTERED to {dl.param1:.4f} — not stored faithfully")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param1 (Speed) 8.0", f"NACKed: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param1_speed_default_sentinel(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param1 (Speed) = -1.0: spec-documented "use default" sentinel; must be accepted."""
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param1=-1.0)
            assert abs(dl.param1 - (-1.0)) < 1e-4, (
                f"param1=-1.0 ('use default speed') not preserved: got {dl.param1}. "
                "This is a spec-defined sentinel and must round-trip faithfully."
            )
            log.info(_FMT, _CMD, "param1 (Speed) -1 sentinel", "PRESERVED (-1.0) — 'use default'")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            pytest.fail(f"param1=-1.0 ('use default') upload NACKed unexpectedly: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param1_speed_nan(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param1 (Speed) = NaN: characterise whether NaN aliases to the -1 'use default' sentinel.

        The spec defines -1 (not NaN) as "use default" for this param.  Whether NaN is
        also accepted (and how it is stored) is observational; no hard assertion.
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param1=NAN)
            if math.isnan(dl.param1):
                outcome = "PRESERVED — NaN accepted and stored as NaN"
            elif abs(dl.param1 - (-1.0)) < 1e-4:
                outcome = "ALIASED to -1.0 — NaN normalised to the spec 'use default' sentinel"
            elif abs(dl.param1) < 1e-4:
                outcome = "ZEROED — NaN treated as 0 m/s (ambiguous: explicit stop or not stored)"
            else:
                outcome = f"ALTERED to {dl.param1:.4f}"
            log.info(_FMT, _CMD, "param1 (Speed) NaN", outcome)
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param1 (Speed) NaN", f"NACKed: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param1_speed_below_min(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param1 (Speed) = -5.0: below the documented minValue=-1 — expected to be unsupported.

        Only -1 (or any value <0, per the description "less than 0 (-1) for default")
        is spec-valid as a negative speed; -5 is not a documented sentinel.  A NACK
        here is the sharp signal that the stack validates param1 against minValue;
        silent acceptance is ambiguous (the value may be clamped or aliased at
        execution time — Tier 2 cannot probe this for a DO_ command without a
        running mission, so the outcome here is the only protocol-level evidence).
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param1=-5.0)
            if abs(dl.param1 - (-5.0)) < 1e-3:
                log.warning(_FMT, _CMD, "param1 (Speed) -5 (< minValue=-1)",
                            "NOTE: out-of-range value silently accepted and preserved raw — should be NACKed/clamped")
            elif abs(dl.param1 - (-1.0)) < 1e-3:
                log.info(_FMT, _CMD, "param1 (Speed) -5 (< minValue=-1)",
                         "CLAMPED to -1.0 — stack enforced minValue on storage")
            else:
                log.warning(_FMT, _CMD, "param1 (Speed) -5 (< minValue=-1)",
                            f"ALTERED to {dl.param1:.4f}")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param1 (Speed) -5 (< minValue=-1)",
                     f"correctly NACKed: {reason} — stack validates minValue")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    # ------------------------------------------------------------------
    # param2 (Bitmask: MAV_DO_REPOSITION_FLAGS)
    # ------------------------------------------------------------------

    async def test_protocol_param2_bitmask_zero(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param2 (Bitmask) = 0: no flags set — must always be accepted and round-trip as 0."""
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param2=0.0)
            assert abs(dl.param2) < 1e-4, (
                f"param2=0 (no flags) not preserved: got {dl.param2}. "
                "value=0 must always round-trip as 0."
            )
            log.info(_FMT, _CMD, "param2 (Bitmask) 0", "PRESERVED (0.0)")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            pytest.fail(f"param2=0 (no flags) NACKed unexpectedly: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param2_bitmask_change_mode(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param2 (Bitmask) = 1: bit 0 (CHANGE_MODE) round-trips.

        CHANGE_MODE means "switch vehicle to guided/hold mode immediately" — a
        guided-command semantic that is hard to reconcile with mission execution
        (the vehicle is, by definition, already in AUTO/mission mode when this
        item runs).  Whether the bit is preserved on storage is observational;
        Tier 2 cannot meaningfully probe its *execution* semantics in a mission.
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param2=float(_FLAG_CHANGE_MODE))
            if abs(dl.param2 - _FLAG_CHANGE_MODE) < 1e-4:
                log.info(_FMT, _CMD, "param2 (Bitmask) bit0 CHANGE_MODE", "PRESERVED (1.0)")
            else:
                log.warning(_FMT, _CMD, "param2 (Bitmask) bit0 CHANGE_MODE",
                            f"ALTERED to {dl.param2:.4f}")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param2 (Bitmask) bit0 CHANGE_MODE", f"NACKed: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param2_bitmask_relative_yaw(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param2 (Bitmask) = 2: bit 1 (RELATIVE_YAW) round-trips."""
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param2=float(_FLAG_RELATIVE_YAW))
            if abs(dl.param2 - _FLAG_RELATIVE_YAW) < 1e-4:
                log.info(_FMT, _CMD, "param2 (Bitmask) bit1 RELATIVE_YAW", "PRESERVED (2.0)")
            else:
                log.warning(_FMT, _CMD, "param2 (Bitmask) bit1 RELATIVE_YAW",
                            f"ALTERED to {dl.param2:.4f}")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param2 (Bitmask) bit1 RELATIVE_YAW", f"NACKed: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param2_bitmask_all_flags(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param2 (Bitmask) = 3: both defined bits combined round-trip."""
        combined = float(_FLAG_CHANGE_MODE | _FLAG_RELATIVE_YAW)
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param2=combined)
            if abs(dl.param2 - combined) < 1e-4:
                log.info(_FMT, _CMD, "param2 (Bitmask) bits 0+1 (3)", "PRESERVED (3.0)")
            else:
                log.warning(_FMT, _CMD, "param2 (Bitmask) bits 0+1 (3)",
                            f"ALTERED to {dl.param2:.4f}")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param2 (Bitmask) bits 0+1 (3)", f"NACKed: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param2_bitmask_undefined_bits(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param2 (Bitmask) = 252 (bits 2-7): undefined bits — expected to be unsupported.

        Only bits 0/1 are defined in MAV_DO_REPOSITION_FLAGS.  252 = 0b11111100
        sets every other bit in the byte while leaving the defined bits clear.
        A NACK is the sharp signal that the stack validates the bitmask; silent
        acceptance/alteration is logged as a NOTE (minor spec violation, common).
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param2=252.0)
            if abs(dl.param2 - 252.0) < 1e-4:
                log.warning(_FMT, _CMD, "param2 (Bitmask) undefined bits (252)",
                            "NOTE: undefined bits silently accepted and preserved — NACK preferred")
            else:
                log.warning(_FMT, _CMD, "param2 (Bitmask) undefined bits (252)",
                            f"NOTE: undefined bits silently altered to {dl.param2:.4f} — NACK preferred")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param2 (Bitmask) undefined bits (252)",
                     f"correctly NACKed: {reason} — stack validates bitmask")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    # ------------------------------------------------------------------
    # param3 (Radius; planes only; positive only; 0/NaN = ignored)
    # ------------------------------------------------------------------

    async def test_protocol_param3_radius_zero(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param3 (Radius) = 0.0: spec sentinel "ignored"; must be accepted and round-trip as 0."""
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param3=0.0)
            assert abs(dl.param3) < 1e-4, (
                f"param3=0 ('ignored') not preserved: got {dl.param3}. "
                "value=0 must always round-trip as 0."
            )
            log.info(_FMT, _CMD, "param3 (Radius) 0 sentinel", "PRESERVED (0.0) — 'ignored'")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            pytest.fail(f"param3=0 ('ignored') NACKed unexpectedly: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param3_radius_nan(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param3 (Radius) = NaN: spec sentinel "ignored" (alternative to 0); should be accepted.

        The spec explicitly states "A value of zero or NaN is ignored" — NaN is
        therefore a documented valid value here (unlike most "unused" params
        where NaN is merely spec-permitted).  Accepted-as-NaN and
        accepted-aliased-to-0 are both spec-compliant; only a NACK is a violation.
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param3=NAN)
            if math.isnan(dl.param3):
                log.info(_FMT, _CMD, "param3 (Radius) NaN sentinel",
                         "PRESERVED (NaN) — 'ignored', spec-compliant")
            elif abs(dl.param3) < 1e-4:
                log.info(_FMT, _CMD, "param3 (Radius) NaN sentinel",
                         "ALIASED to 0.0 — both are spec 'ignored' sentinels; spec-compliant")
            else:
                log.warning(_FMT, _CMD, "param3 (Radius) NaN sentinel",
                            f"ALTERED to {dl.param3:.4f} — neither NaN nor 0; spec violation")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.warning(_FMT, _CMD, "param3 (Radius) NaN sentinel",
                        f"FAIL: NaN rejected ({reason}) — spec violation: 'zero or NaN' must be accepted")
            pytest.fail(
                "param3=NaN ('ignored' sentinel) rejected — spec violation. "
                "The spec explicitly states 'A value of zero or NaN is ignored'."
            )
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param3_radius_positive(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param3 (Radius) = 80.0 m: a valid in-range (positive) value round-trips."""
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param3=80.0)
            if abs(dl.param3 - 80.0) < 1e-4:
                log.info(_FMT, _CMD, "param3 (Radius) 80.0 (positive)",
                         f"PRESERVED ({dl.param3:.4f}) — probably supported")
            else:
                log.warning(_FMT, _CMD, "param3 (Radius) 80.0 (positive)",
                            f"ALTERED to {dl.param3:.4f}")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param3 (Radius) 80.0 (positive)", f"NACKed: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param3_radius_negative(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param3 (Radius) = -80.0 m: spec says "Positive values only" — expected to be unsupported.

        A negative radius is explicitly out-of-spec (direction is controlled by
        the Yaw param, not the sign of Radius).  A NACK is the sharp signal that
        the stack validates the sign; silent acceptance/aliasing is logged.
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param3=-80.0)
            if abs(dl.param3 - (-80.0)) < 1e-3:
                log.warning(_FMT, _CMD, "param3 (Radius) -80 (positive only)",
                            "NOTE: negative radius silently accepted and preserved raw — spec says 'positive values only'")
            elif abs(dl.param3 - 80.0) < 1e-3:
                log.info(_FMT, _CMD, "param3 (Radius) -80 (positive only)",
                         "ABS-NORMALISED to 80.0 — sign discarded on storage")
            elif abs(dl.param3) < 1e-3:
                log.info(_FMT, _CMD, "param3 (Radius) -80 (positive only)",
                         "ZEROED — negative radius treated as invalid/ignored")
            else:
                log.warning(_FMT, _CMD, "param3 (Radius) -80 (positive only)",
                            f"ALTERED to {dl.param3:.4f}")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param3 (Radius) -80 (positive only)",
                     f"correctly NACKed: {reason} — stack validates sign")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    # ------------------------------------------------------------------
    # param4 (Yaw, RADIANS; NaN = use current heading; planes: loiter direction)
    # ------------------------------------------------------------------

    async def test_protocol_param4_yaw_nan(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw) = NaN: spec sentinel "use current system yaw heading mode"; must round-trip as NaN."""
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param4=NAN)
            assert math.isnan(dl.param4), (
                f"param4 (Yaw) NaN not preserved: downloaded {dl.param4}. "
                "Spec documents NaN as 'use current heading mode'; it should be stored as NaN."
            )
            log.info(_FMT, _CMD, "param4 (Yaw) NaN sentinel", "PRESERVED (NaN) — 'use current heading'")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param4 (Yaw) NaN sentinel", f"NACKed: {reason}")
            pytest.fail(f"param4 (Yaw=NaN) upload NACKed unexpectedly: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param4_yaw_specific(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw) = pi/2 rad (~90 deg east): a specific in-range RADIAN value round-trips.

        DOC DISCREPANCY note: common.xml gives param4 units="rad" with the
        general meaning "Yaw heading", but the same paragraph adds "For planes
        indicates loiter direction (0: clockwise, 1: counter clockwise)" — i.e.
        a *binary* direction flag, not a heading, when the vehicle is a plane
        (and, implicitly, when param3/Radius requests a loiter).  These two
        readings of the same field are difficult to reconcile (a heading of
        1 radian = ~57 deg is a perfectly valid compass heading, yet on a plane
        it would supposedly mean "counter-clockwise loiter").  This is logged
        here as an observation; if a real stack's behaviour resolves the
        ambiguity (e.g. FW interprets param4 as direction only when Radius != 0)
        that resolution is recorded in the per-stack README notes and an issue
        should be raised at https://github.com/mavlink/mavlink/issues if the
        spec text is confirmed ambiguous.
        """
        target = math.pi / 2  # ~1.5708 rad (~90 deg)
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param4=target)
            if abs(dl.param4 - target) < 1e-3:
                log.info(_FMT, _CMD, "param4 (Yaw) pi/2 rad",
                         f"PRESERVED ({dl.param4:.4f} rad) — probably supported")
            else:
                log.warning(_FMT, _CMD, "param4 (Yaw) pi/2 rad",
                            f"ALTERED to {dl.param4:.4f} rad")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param4 (Yaw) pi/2 rad", f"NACKed: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param4_yaw_zero(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw) = 0.0 rad (due north / CW loiter on planes): must be distinct from NaN.

        0.0 is a valid specific heading (and, per the plane-specific reading, a
        valid direction flag).  It must not be aliased to NaN ("use current
        heading"); a stack that does so confuses an explicit command with the
        sentinel — the same spec-violation pattern documented for NAV_TAKEOFF
        param4 (see nav_takeoff/test_protocol.py::test_protocol_param4_yaw_zero).

        Note: a stack that does NOT store param4 at all will also return 0.0,
        causing this test to PASS vacuously.
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param4=0.0)
            if math.isnan(dl.param4):
                log.warning(_FMT, _CMD, "param4 (Yaw) 0 rad",
                            "ALIASED to NaN — spec violation: 0 (north/CW) must be distinct from NaN (auto-heading)")
                pytest.fail(
                    "param4=0.0 (due north / CW loiter) aliased to NaN on storage — spec violation. "
                    "Zero is a valid explicit value; NaN means 'use current heading mode'."
                )
            assert abs(dl.param4) < 1e-4, (
                f"param4=0.0 not preserved: stored as {dl.param4:.4f}. "
                "Zero is a valid specific value and must round-trip faithfully."
            )
            log.info(_FMT, _CMD, "param4 (Yaw) 0 rad", f"PRESERVED ({dl.param4:.4f})")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param4 (Yaw) 0 rad", f"NACKed: {reason}")
            pytest.fail(f"param4=0.0 (due north / CW loiter) NACKed: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param4_yaw_out_of_range(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw) = 10.0 rad (> 2*pi ~ 6.283): outside the natural radian range — expected to be unsupported.

        A heading expressed in radians has a natural range of [0, 2*pi) (or
        [-pi, pi]); 10 rad is neither a valid heading nor a valid binary
        direction flag (the plane-specific reading only defines 0/1).  This
        characterises whether the stack enforces any range on param4; no
        single outcome is a protocol violation (the spec gives no explicit
        bounds), so this is observational.
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param4=10.0)
            two_pi = 2 * math.pi
            if abs(dl.param4 - 10.0) < 1e-3:
                outcome = "PRESERVED raw (10.0 rad) — no range enforcement on storage"
            elif abs(dl.param4 - (10.0 % two_pi)) < 1e-3:
                outcome = f"WRAPPED to {dl.param4:.4f} rad ([0, 2*pi) canonical form)"
            elif math.isnan(dl.param4):
                outcome = "ALIASED to NaN — stack treats out-of-range yaw as 'use current heading'"
            elif abs(dl.param4) < 1e-3:
                outcome = "ZEROED — out-of-range value reset to 0"
            else:
                outcome = f"ALTERED to {dl.param4:.4f} rad"
            log.info(_FMT, _CMD, "param4 (Yaw) 10 rad (out of range)", outcome)
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param4 (Yaw) 10 rad (out of range)",
                     f"NACKed: {reason} — stack validates range")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    # ------------------------------------------------------------------
    # Location (params 5/6/7 -> x/y/z; hasLocation + isDestination)
    # ------------------------------------------------------------------

    async def test_protocol_location_preserved(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """params 5/6/7 (Latitude/Longitude/Altitude): a specific, non-zero location round-trips."""
        lat_int = _LAT_INT + 50000   # ~0.005 deg north
        lon_int = _LON_INT + 50000   # ~0.005 deg east
        alt = 65.0
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, x=lat_int, y=lon_int, z=alt)
            assert dl.x == lat_int, f"Latitude (x) not preserved: {dl.x} != {lat_int}"
            assert dl.y == lon_int, f"Longitude (y) not preserved: {dl.y} != {lon_int}"
            assert abs(dl.z - alt) < 1e-4, f"Altitude (z) not preserved: {dl.z} != {alt}"
            log.info(_FMT, _CMD, "params 5/6/7 (Lat/Lon/Alt)",
                     f"PRESERVED (x={dl.x}, y={dl.y}, z={dl.z:.1f}) — probably supported")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "params 5/6/7 (Lat/Lon/Alt)", f"NACKed: {reason}")
            pytest.fail(f"Location params upload NACKed unexpectedly: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_location_int32max(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """params 5/6 (lat/lon) = INT32_MAX: 'use current position' sentinel.

        INT32_MAX (0x7FFF_FFFF) is the MISSION_ITEM_INT sentinel meaning "use
        current position" for integer lat/lon fields — the natural way to say
        "reposition only altitude/speed/yaw, keep current horizontal position".
        Whether this is honoured is the same spec-level question already probed
        for NAV_TAKEOFF (test_protocol_location_current_position); for a
        guided-style command like DO_REPOSITION the sentinel is, if anything,
        more semantically natural.  (Both x AND y must be set to INT32_MAX
        together — the generic per-slot "defined param sentinel tolerated"
        test inherited from Tier1MissionTestBase probes each independently,
        which is not a meaningful "use current position" combination on its
        own; this bespoke test is the sharp, correct probe.)
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, x=INT32_MAX, y=INT32_MAX, z=30.0)
            x_ok = dl.x == INT32_MAX
            y_ok = dl.y == INT32_MAX
            if x_ok and y_ok:
                log.info(_FMT, _CMD, "params 5/6 INT32_MAX (current pos)",
                         "PRESERVED — 'use current position' sentinel accepted")
            else:
                log.warning(_FMT, _CMD, "params 5/6 INT32_MAX (current pos)",
                            f"ALTERED: x={dl.x}, y={dl.y} — sentinel not preserved")
            assert x_ok and y_ok, (
                f"INT32_MAX lat/lon sentinel not preserved: got x={dl.x}, y={dl.y}."
            )
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.warning(_FMT, _CMD, "params 5/6 INT32_MAX (current pos)", f"NACKed: {reason}")
            pytest.fail(f"INT32_MAX lat/lon NACKed — 'use current position' not supported: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_location_out_of_range_latlon(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """params 5/6 (lat/lon) outside +-90deg / +-180deg: expected to be unsupported.

        Latitude=91 deg, Longitude=181 deg are physically impossible coordinates
        (encoded as int x 1e7).  The mock NACKs these with DENIED (see
        MockFlightStack docstring); a real stack accepting them silently is a
        spec gap (the same one already documented for the COMMAND_INT path —
        see tests/command/do_reposition/README.md "Spec gaps").  A NACK here is
        the sharp, unambiguous signal that the stack validates coordinate range.
        """
        lat_int = int(91.0 * 1e7)
        lon_int = int(181.0 * 1e7)
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, x=lat_int, y=lon_int, z=30.0)
            if dl.x == lat_int and dl.y == lon_int:
                log.warning(_FMT, _CMD, "params 5/6 out-of-range lat/lon",
                            "NOTE: out-of-range coordinates silently accepted and preserved raw — spec gap (should DENY)")
            else:
                log.warning(_FMT, _CMD, "params 5/6 out-of-range lat/lon",
                            f"ALTERED: x={dl.x}, y={dl.y} (uploaded x={lat_int}, y={lon_int})")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "params 5/6 out-of-range lat/lon",
                     f"correctly NACKed: {reason} — stack validates coordinate range")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_altitude_nan(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param 7 (Altitude) = NaN: characterise 'use current/default altitude' sentinel.

        NaN is the float sentinel for "use default / unspecified" per
        MISSION_ITEM_INT.  For a reposition target this plausibly means "keep
        current altitude" — a natural complement to the INT32_MAX 'keep current
        position' sentinel for lat/lon.  Outcome is observed and logged; no
        hard assertion (the spec does not define this combination explicitly).
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, z=NAN)
            if math.isnan(dl.z):
                log.info(_FMT, _CMD, "param7 (Alt) NaN",
                         "PRESERVED — NaN altitude accepted ('use current/default')")
            else:
                log.warning(_FMT, _CMD, "param7 (Alt) NaN",
                            f"ALTERED to {dl.z:.4f} — stack normalised NaN altitude")
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param7 (Alt) NaN",
                     f"NACKed ({reason}) — NaN altitude not accepted (may be intentional)")
        finally:
            await clear_all_mission_types(gcs_system_cls)
