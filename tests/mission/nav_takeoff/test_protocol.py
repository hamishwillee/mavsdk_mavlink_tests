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

Built on tests/mission/conftest.py's Tier1MissionTestBase/MissionItemSpec —
the shared mandatory Tier 1 tests (baseline-accepted, undefined-param
sentinel pair, defined-param sentinel-tolerated) are inherited; this file
only adds NAV_TAKEOFF's own bespoke per-parameter tests (round-trip of a
specific value, bitmask/location semantics, edge-case observational probes
— see tests/mission/CLAUDE.md § MAV_CMD support testing).

Unlike do_reposition/condition_gate (rejected outright, so every param-level
test — generic and bespoke alike — is skipped via `_skip_if_unsupported`),
NAV_TAKEOFF is genuinely supported everywhere, so the generic
`test_defined_param_sentinel_tolerated` test actually exercises real
per-stack quirks: ArduPilot's `sanity_check_params` rejects NaN in any
float param but param4 (see `ParamSpec.sentinel_xfail_reason` on params
1/3/7 below).  Per tests/mission/CLAUDE.md's documented convention, a
defined param's NaN sentinel is NOT itself spec-mandated (unlike an
undefined param's, or a hasLocation param's INT32_MAX) — a real-stack
rejection here is a known, xfailed limitation, not a hard failure.

MAV_CMD_NAV_TAKEOFF parameter table (common.xml)
------------------------------------------------
  param1  Pitch (deg)            Defined
  param2  —                      Unused (empty) — spec: NaN; baseline uses 0.0 (ArduCopter
                                 rejects NaN for param2 — spec violation; see the generic
                                 test_undefined_param_sentinel_accepted[param2])
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

import logging
import math

import pytest
from mavsdk.mission_raw import MissionRawError

from ..conftest import MissionItemSpec, Tier1MissionTestBase, clear_all_mission_types
from tests.param_spec import ParamSpec

log = logging.getLogger(__name__)

_CMD = "NAV_TAKEOFF"
_FMT = "%-14s | %-44s | %s"

NAN = float("nan")
INT32_MAX = 0x7FFF_FFFF

# SIH simulator home coordinates (47.3977°N, 8.5456°E)
_LAT_INT = 473977000
_LON_INT = 85456000

# ArduPilot's sanity_check_params() nan_mask for NAV_TAKEOFF permits NaN only
# in param4 (Yaw, which has a documented "use current heading" sentinel
# meaning); params 1-3 must be a concrete non-NaN value or the upload is
# rejected with MAV_MISSION_INVALID_PARAM<n> — a per-stack strictness on a
# NaN sentinel that isn't itself spec-mandated for these params (Pitch/Flags
# have no documented NaN meaning), matching tests/mission/CLAUDE.md's
# "defined float param NaN: no hard assertion" convention. z/Altitude is
# rejected via a separate, stack-specific altitude sanity check.
_ARDUPILOT_NAN_MASK_REASON = (
    "ArduPilot's sanity_check_params() nan_mask permits NaN only in param4 (Yaw) for "
    "NAV_TAKEOFF; this param's NaN sentinel is not itself spec-mandated (see CLAUDE.md)"
)
_ARDUPILOT_ALTITUDE_NAN_REASON = (
    "ArduPilot rejects NaN altitude via its own sanity check; NaN altitude is "
    "documented as observational, not spec-mandated, for a takeoff command (see CLAUDE.md)"
)

# ---------------------------------------------------------------------------
# Tier 1 spec
# ---------------------------------------------------------------------------

SPEC = MissionItemSpec(
    cmd_id=22,
    name=_CMD,
    mission_type=0,
    frame=5,  # MAV_FRAME_GLOBAL_INT
    baseline=dict(
        param1=15.0,   # Pitch: 15 deg
        param2=0.0,    # unused — 0.0 (ArduCopter rejects NaN for this param despite it being unused)
        param3=0.0,    # Flags: none
        param4=NAN,    # Yaw: NaN = use current heading
        x=_LAT_INT,
        y=_LON_INT,
        z=50.0,        # Altitude: 50 m AMSL
    ),
    params=[
        ParamSpec(1, "Pitch", defined=True, sentinel_xfail_reason=_ARDUPILOT_NAN_MASK_REASON),
        ParamSpec(2, "Empty", defined=False),
        ParamSpec(3, "Flags", defined=True, sentinel_xfail_reason=_ARDUPILOT_NAN_MASK_REASON),
        ParamSpec(4, "Yaw", defined=True),  # NaN sentinel IS spec-mandated here — no xfail
        ParamSpec(5, "Latitude", defined=True),  # INT32_MAX sentinel IS spec-mandated — no xfail
        ParamSpec(6, "Longitude", defined=True),
        ParamSpec(7, "Altitude", defined=True, sentinel_xfail_reason=_ARDUPILOT_ALTITUDE_NAN_REASON),
    ],
)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


@pytest.mark.asyncio(loop_scope="class")
class TestNavTakeoff(Tier1MissionTestBase):
    """Protocol-acceptance tests for MAV_CMD_NAV_TAKEOFF (cmd=22) as a mission item."""

    SPEC = SPEC

    async def test_protocol_param1_pitch_preserved(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param1 (Pitch): specific value round-trips correctly.

        Per root CLAUDE.md's "General testing philosophy" rule 4: a defined param
        that's accepted (not NACKed) but not preserved is a general, cross-stack spec
        violation, tracked as xfail rather than a hard failure — both PX4 (this test)
        and ArduCopter/ArduPlane (see nav_takeoff/CLAUDE.md's storage table) silently
        zero param1 instead of NACKing a value they don't store.
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param1=15.0)
            ok = abs(dl.param1 - 15.0) < 1e-4
            log.info(_FMT, _CMD, "param1 (Pitch)", f"PRESERVED ({dl.param1:.4f})" if ok else f"NOT preserved (downloaded {dl.param1})")
            if not ok:
                pytest.xfail(
                    f"param1 (Pitch) not preserved: uploaded 15.0, downloaded {dl.param1} — "
                    "accepted but silently zeroed instead of NACKed (cross-stack gap, see module docstring)"
                )
            assert ok
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param1 (Pitch)", f"NACKed: {reason}")
            pytest.fail(f"param1 (Pitch) upload NACKed unexpectedly: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param3_flags_preserved(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param3 (Flags / NAV_TAKEOFF_FLAGS): bit 0 (HORIZONTAL_POSITION_NOT_REQUIRED) round-trips.

        Same xfail reasoning as test_protocol_param1_pitch_preserved — see its docstring.
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param3=1.0)
            ok = abs(dl.param3 - 1.0) < 1e-4
            log.info(_FMT, _CMD, "param3 (Flags)", f"PRESERVED ({dl.param3:.4f})" if ok else f"NOT preserved (downloaded {dl.param3})")
            if not ok:
                pytest.xfail(
                    f"param3 (Flags) not preserved: uploaded 1.0, downloaded {dl.param3} — "
                    "accepted but silently zeroed instead of NACKed (cross-stack gap, see module docstring)"
                )
            assert ok
        except MissionRawError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param3 (Flags)", f"NACKed: {reason}")
            pytest.fail(f"param3 (Flags) upload NACKed unexpectedly: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_protocol_param4_yaw_specific(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw): specific degree value round-trips correctly."""
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param4=90.0)
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
        """param4 (Yaw): NaN (use current heading) is accepted and returned as NaN.

        Unlike the generic test_defined_param_sentinel_tolerated[param4] (which
        only checks accept/reject), this hard-asserts the *value* is preserved
        as NaN — param4's NaN sentinel has a documented spec meaning, so this
        is a real compliance check, not observational.
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param4=NAN)
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
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, x=lat_int, y=lon_int, z=alt)
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

        Both x AND y are set to INT32_MAX together — a meaningful "use current
        position" combination, unlike the generic per-slot
        test_defined_param_sentinel_tolerated[param5]/[param6], which probes
        each independently (same rationale as do_reposition's analogous
        bespoke location-sentinel test).
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, x=INT32_MAX, y=INT32_MAX, z=50.0)
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
        may legitimately NACK it.  Outcome is observed and logged; no hard assertion
        (same ground the generic test_defined_param_sentinel_tolerated[param7] covers
        with a hard xfail-on-reject instead — this bespoke test keeps the original,
        richer PRESERVED/ALTERED/NACKed logging).
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, z=NAN)
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
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param3=0.0)
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
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param3=2.0)
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
        and logged; no hard assertion (same ground the generic
        test_defined_param_sentinel_tolerated[param1] covers with a hard xfail-on-reject
        instead — this bespoke test keeps the original, richer PRESERVED/ALTERED logging).
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param1=NAN)
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
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param1=180.0)
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
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param4=-90.0)
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
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param4=450.0)
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
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param4=0.0)
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
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param1=89.0)
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
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param1=-10.0)
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
