"""
MAV_CMD_CONDITION_GATE (cmd=4501) — Tier 1 mission-protocol acceptance tests.

Built on tests/mission/conftest.py's Tier1MissionTestBase/MissionItemSpec —
the shared mandatory Tier 1 tests (baseline-accepted, undefined-param
sentinel pair, defined-param sentinel-tolerated) are inherited; this file
only adds CONDITION_GATE's own bespoke tests (enum semantics, the PX4
param-drop finding, and the near-waypoint feasibility check).

Why this command needs a RAW transport (not mission_raw)
----------------------------------------------------------
CONDITION_GATE is tagged ``<wip/>`` in common.xml.  MAVSDK's `mission_raw`
plugin (`mavsdk_server`) rejects it client-side with `INVALID_ARGUMENT`
*before anything reaches the wire*, because 4501 isn't in `mavsdk_server`'s
own internal command table — confirmed by cross-referencing this project's
findings against the sibling manual-verification tool at
``mavsdk_qgc_server_tests/condition_gate_tests`` (a pymavlink script, chosen
specifically because pymavlink has no such client-side allow-list).  This
file therefore sets ``MissionItemSpec(transport="raw", ...)``, which routes
every upload/download through `raw_upload_mission_items()` /
`raw_download_mission_items()` (tests/mission/conftest.py) — a direct
MISSION_COUNT/MISSION_REQUEST_INT/MISSION_ITEM_INT/MISSION_ACK handshake via
`mavlink_direct` that bypasses `mission_raw`'s allow-list entirely.  This is
the first command in this test suite that needs the raw path; see
CLAUDE.md for the full discovery writeup.

MAV_CMD_CONDITION_GATE parameter table (common.xml)
----------------------------------------------------
  param1  Geometry     (minValue=0; only 0 "orthogonal to path" is defined) Defined
  param2  UseAltitude  (enum=MAV_BOOL; 0/1 only, else invalid per spec)     Defined (enum)
  param3  Empty                                                             Undefined
  param4  Empty                                                             Undefined
  param5  Latitude     (x field, int x 1e7)                                 Location
  param6  Longitude    (y field, int x 1e7)                                 Location
  param7  Altitude     (z field, float m)                                   Location

Autopilot support (source-verified — see CLAUDE.md)
------------------------------------------------------
PX4: accepted as a mission item since PX4 v1.11 (`src/modules/mavlink/
mavlink_command_params.hpp`: `{4501, 0x73, 0x00}` — mission mask permits
params 1,2,5,6,7; command mask 0x00 means it is NOT usable as a direct
COMMAND_INT/LONG, mission-item-only).  Geometry/UseAltitude (params 1/2) are
never copied into the stored mission item at all (`mavlink_mission.cpp`'s
upload-parse switch and download-format switch both have no case for this
nav_cmd) — see test_params_1_2_zeroed_on_roundtrip_px4.  A gate within 5 cm
of a neighbouring waypoint is rejected as infeasible
(`FeasibilityChecker.cpp`) — see test_gate_coincident_with_waypoint_rejected.

ArduPilot: not implemented on any variant — absent from
`AP_Mission::mavlink_int_to_mission_cmd()`'s command switch, which falls
through to `MAV_MISSION_UNSUPPORTED` for any unrecognised command
(ArduPilot upstream issue: https://github.com/ArduPilot/ardupilot/issues/13778).
Every per-param test in this class is skipped when the baseline itself is
rejected (see Tier1MissionTestBase._skip_if_unsupported) — this is the
expected, spec-aligned outcome on ArduPilot, not a test gap.

Cross-checking mavlink-devguide PR #761
------------------------------------------
mavlink/mavlink-devguide#761 (co-authored by an AI assistant, not yet
test-verified at the time it was opened) documents CONDITION_GATE's
behaviour, including an "Autopilot Support" table claiming: PX4 supports it
from v1.11, ignores UseAltitude, and rejects a gate within 5cm of a
neighbouring waypoint as infeasible; ArduPilot does not support it at all
(citing ardupilot#13778).  This file's source-code findings CONFIRM those
specific claims, including UseAltitude: PX4 never even stores the value
(see test_params_1_2_zeroed_on_roundtrip_px4), which is one concrete
mechanism consistent with "ignored" — whether the value is dropped on
storage or read-and-discarded at execution time makes no observable
difference to a GCS, so this is confirmation of the PR's claim, not a
correction of it. See README.md's "mavlink-devguide PR #761 cross-check"
section for the full confirmed/refuted/inconclusive table (Tier 2,
test_flight.py, verifies the remaining behavioural claims — the gate not
bending the route, and the blocking/trigger-point mechanics — that only
manifest in a flying vehicle).

Running
-------
Against the mock (no autopilot required)::

    pytest tests/mission/condition_gate/test_protocol.py -v --log-cli-level=INFO

Against a real flight stack::

    pytest tests/mission/condition_gate/test_protocol.py --drone-address=udp://:14540 -v --log-cli-level=INFO
"""

import logging

import pytest
from mavsdk.mission_raw import MissionItem

from ..conftest import (
    MissionItemSpec,
    RawMissionError,
    Tier1MissionTestBase,
    clear_all_mission_types,
    raw_upload_mission_items,
)
from tests.param_spec import ParamSpec

log = logging.getLogger(__name__)

_CMD = "CONDITION_GATE"
_FMT = "%-14s | %-44s | %s"

NAN = float("nan")

# SIH simulator home coordinates (47.3977°N, 8.5456°E) — same as other mission tests.
_LAT_INT = 473977000
_LON_INT = 85456000
_FRAME = 5  # MAV_FRAME_GLOBAL_INT

# ---------------------------------------------------------------------------
# Tier 1 spec
# ---------------------------------------------------------------------------

SPEC = MissionItemSpec(
    cmd_id=4501,
    name=_CMD,
    mission_type=0,
    transport="raw",  # bypasses mission_raw's <wip/> client-side block — see module docstring
    frame=_FRAME,
    baseline=dict(
        param1=0.0,   # Geometry: 0 = orthogonal to path (only spec-defined value)
        param2=0.0,   # UseAltitude: MAV_BOOL_FALSE = ignore altitude
        # param3/4 default is **0.0, not the spec-correct NaN** ("Empty") — the
        # same ArduCopter baseline-probe pitfall documented in
        # tests/mission/nav_takeoff/CLAUDE.md:
        # ArduPilot's sanity_check_params() only permits NaN in the params of
        # commands it explicitly special-cases; CONDITION_GATE isn't one of
        # them (it isn't recognised at all), so its blanket nan_mask rejects
        # NaN in ANY param with MAV_MISSION_INVALID_PARAM3 *before* the
        # command-recognition switch that would otherwise report UNSUPPORTED
        # is ever reached — confirmed empirically (ArduCopter SITL returned
        # INVALID_PARAM3 for a NaN baseline, and UNSUPPORTED once param3/4
        # were changed to 0.0). The NaN-sentinel behaviour itself is still
        # exercised by the generic undefined-param tests inherited from
        # Tier1MissionTestBase, which probe it independently of this baseline.
        param3=0.0,
        param4=0.0,
        x=_LAT_INT + 30000,   # ~0.003 deg north of SIH home — an off-path gate location
        y=_LON_INT + 20000,
        z=30.0,
    ),
    params=[
        ParamSpec(1, "Geometry", defined=True),
        ParamSpec(2, "UseAltitude", defined=True),
        # PX4 DOES correctly reject a non-sentinel value here (confirmed:
        # mavlink_command_params.hpp's mask for cmd 4501 has bits 2/3 clear,
        # so _scan_params()/param_is_unset() rejects anything but NaN/0.0 for
        # these two slots with MAV_MISSION_INVALID_PARAM3/4) — this test only
        # XFAILs on the mock, which has no per-param validation at all and
        # accepts every value. The default fallback reason ("no known stack
        # validates...") would be actively wrong here, so it's overridden.
        ParamSpec(3, "Empty", defined=False,
                  reject_xfail_reason="Mock has no per-param validation and accepts every value; "
                                       "PX4 correctly rejects this with INVALID_PARAM3 (mask-driven, "
                                       "see mavlink_command_params.hpp)"),
        ParamSpec(4, "Empty", defined=False,
                  reject_xfail_reason="Mock has no per-param validation and accepts every value; "
                                       "PX4 correctly rejects this with INVALID_PARAM4 (mask-driven, "
                                       "see mavlink_command_params.hpp)"),
        ParamSpec(5, "Latitude", defined=True),
        ParamSpec(6, "Longitude", defined=True),
        ParamSpec(7, "Altitude", defined=True),
    ],
)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


@pytest.mark.asyncio(loop_scope="class")
class TestConditionGate(Tier1MissionTestBase):
    """Protocol-acceptance tests for MAV_CMD_CONDITION_GATE (cmd=4501) as a mission item."""

    SPEC = SPEC

    # ------------------------------------------------------------------
    # param1 (Geometry; minValue=0; only 0 is spec-defined; no enum= attribute)
    # ------------------------------------------------------------------

    async def test_geometry_only_value_zero_accepted(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param1 (Geometry) = 0: the only spec-defined value ("orthogonal to path"); upload must be accepted.

        Deliberately checks acceptance only, not round-trip: PX4 never copies
        Geometry into the stored item at all (see
        test_params_1_2_zeroed_on_roundtrip_px4), so 0.0 happens to be both a
        legitimate value and PX4's always-zeroed default — a round-trip
        assertion here would be a vacuous PASS on PX4 (see
        tests/mission/CLAUDE.md § "Vacuous PASS").
        """
        try:
            await self._upload_probe(gcs_system_cls, home_item_for_mission, param1=0.0)
            log.info(_FMT, _CMD, "param1 (Geometry) 0", "ACCEPTED")
        except RawMissionError as exc:
            reason = str(exc).split(":")[0].strip()
            pytest.fail(f"param1=0 (the only spec-defined Geometry value) NACKed unexpectedly: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_geometry_undocumented_value_observational(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param1 (Geometry) = 7: not a spec-defined value (only 0 is documented) — observational.

        No `enum=` attribute exists on this param in the XML despite only one
        value being documented, so a hard NACK is not spec-mandated; this
        characterises whether any stack validates it anyway.
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param1=7.0)
            log.info(_FMT, _CMD, "param1 (Geometry) 7 (undocumented)",
                     f"ACCEPTED, downloaded as {dl.param1}")
        except RawMissionError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param1 (Geometry) 7 (undocumented)", f"NACKed: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    # ------------------------------------------------------------------
    # param2 (UseAltitude, enum=MAV_BOOL: 0/1 only; else invalid per spec)
    # ------------------------------------------------------------------

    async def test_use_altitude_false_accepted(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param2 (UseAltitude) = 0 (MAV_BOOL_FALSE, "ignore altitude" / 2D gate): must be accepted."""
        try:
            await self._upload_probe(gcs_system_cls, home_item_for_mission, param2=0.0)
            log.info(_FMT, _CMD, "param2 (UseAltitude) MAV_BOOL_FALSE", "ACCEPTED")
        except RawMissionError as exc:
            reason = str(exc).split(":")[0].strip()
            pytest.fail(f"param2=0 (MAV_BOOL_FALSE) NACKed unexpectedly: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_use_altitude_true_accepted(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param2 (UseAltitude) = 1 (MAV_BOOL_TRUE, "include altitude" / 3D gate): must be accepted.

        Acceptance only — PX4 discards this value on storage regardless (see
        test_params_1_2_zeroed_on_roundtrip_px4), so whether it is *honoured*
        cannot be determined at Tier 1; test_flight.py verifies execution.
        """
        try:
            await self._upload_probe(gcs_system_cls, home_item_for_mission, param2=1.0)
            log.info(_FMT, _CMD, "param2 (UseAltitude) MAV_BOOL_TRUE", "ACCEPTED")
        except RawMissionError as exc:
            reason = str(exc).split(":")[0].strip()
            pytest.fail(f"param2=1 (MAV_BOOL_TRUE) NACKed unexpectedly: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_use_altitude_invalid_value_observational(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param2 (UseAltitude) = 2: spec says "Values not equal to 0 or 1 are invalid" — expected to be unsupported.

        A NACK is the sharp signal that the stack validates the enum; silent
        acceptance is a NOTE-level spec violation, not asserted as a failure
        (no stack in this project's test matrix is known to validate this).
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param2=2.0)
            log.warning(_FMT, _CMD, "param2 (UseAltitude) 2 (invalid)",
                        f"NOTE: invalid enum value silently accepted, downloaded as {dl.param2} — spec says invalid")
        except RawMissionError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "param2 (UseAltitude) 2 (invalid)",
                     f"correctly NACKed: {reason} — stack validates enum")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    # ------------------------------------------------------------------
    # params 1/2 dropped on PX4 storage (mechanism behind PR #761's
    # "UseAltitude ignored" claim)
    # ------------------------------------------------------------------

    async def test_params_1_2_zeroed_on_roundtrip_px4(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """
        params 1/2 (Geometry/UseAltitude): characterise whether they survive
        upload/download at all, using distinguishable non-zero values
        (param1=7.0, an undocumented-but-upload-tolerated value; param2=1.0,
        MAV_BOOL_TRUE) — 0.0 is both a legitimate value and PX4's
        always-zeroed default, so testing 0.0 would be a vacuous PASS (see
        test_geometry_only_value_zero_accepted).

        PX4 source (`mavlink_mission.cpp`) confirms these two params are
        never copied into the stored mission item at all — the upload-parse
        switch's CONDITION_GATE case only sets `nav_cmd`, and the
        download-format switch has no case for it either, silently leaving
        both at their zero-initialised default on every download regardless
        of what was uploaded. This is consistent with (not a correction of)
        mavlink-devguide PR #761's "UseAltitude field ignored" claim — from a
        GCS's perspective, "the value is dropped on storage" and "the value
        is read but never acted on" are indistinguishable; either way the
        param has no observable effect, which is what "ignored" already means.

        Observational elsewhere: e.g. the mock echoes raw values unchanged
        and will legitimately preserve both, which is itself useful evidence
        that this is a PX4-specific storage detail, not a protocol requirement.
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param1=7.0, param2=1.0)
            p1_zeroed = abs(dl.param1) < 1e-6
            p2_zeroed = abs(dl.param2) < 1e-6
            if p1_zeroed and p2_zeroed:
                log.info(
                    _FMT, _CMD, "params 1/2 (Geometry/UseAltitude)",
                    "both zeroed on download regardless of uploaded value (PX4 never copies "
                    "these into the stored item) — consistent with PR #761's 'UseAltitude ignored'",
                )
            else:
                log.info(
                    _FMT, _CMD, "params 1/2 (Geometry/UseAltitude)",
                    f"PRESERVED (param1={dl.param1}, param2={dl.param2}) — this stack does not "
                    "exhibit PX4's param-drop behaviour",
                )
        except RawMissionError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "params 1/2 (Geometry/UseAltitude)", f"NACKed: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    # ------------------------------------------------------------------
    # Location (params 5/6/7 -> x/y/z; hasLocation="true" — despite the XML
    # also marking isDestination="true", the devguide PR and PX4's own
    # navigator treat the gate as explicitly NOT a route destination; see
    # test_flight.py for the flown-path evidence)
    # ------------------------------------------------------------------

    async def test_location_preserved(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """params 5/6/7 (Latitude/Longitude/Altitude): a specific, non-zero location round-trips.

        Unlike Geometry/UseAltitude (params 1/2), PX4 source confirms
        lat/lon/alt ARE preserved — they flow through the generic
        global-frame handling path, separate from the per-command param-copy
        switch that drops params 1/2 (test_params_1_2_zeroed_on_roundtrip_px4).
        """
        lat_int = _LAT_INT + 40000
        lon_int = _LON_INT + 30000
        alt = 42.0
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, x=lat_int, y=lon_int, z=alt)
            assert dl.x == lat_int, f"Latitude (x) not preserved: {dl.x} != {lat_int}"
            assert dl.y == lon_int, f"Longitude (y) not preserved: {dl.y} != {lon_int}"
            assert abs(dl.z - alt) < 1e-4, f"Altitude (z) not preserved: {dl.z} != {alt}"
            log.info(_FMT, _CMD, "params 5/6/7 (Lat/Lon/Alt)",
                     f"PRESERVED (x={dl.x}, y={dl.y}, z={dl.z:.1f})")
        except RawMissionError as exc:
            reason = str(exc).split(":")[0].strip()
            pytest.fail(f"Location params upload NACKed unexpectedly: {reason}")
        finally:
            await clear_all_mission_types(gcs_system_cls)

    # ------------------------------------------------------------------
    # Near-waypoint feasibility check (PX4-specific; cross-checks PR #761)
    # ------------------------------------------------------------------

    async def test_gate_coincident_with_waypoint_rejected(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """
        A gate placed at the same location as an adjacent waypoint (0m
        separation, well within PX4's documented 5cm feasibility threshold —
        `FeasibilityChecker.cpp`) should be rejected as infeasible ("Distance
        between waypoint and gate too close"). Independently confirms
        mavlink-devguide PR #761's claim: "A gate within 5 cm of a
        neighboring waypoint is rejected as infeasible."

        This test builds its own 2-item mission directly (a waypoint plus the
        gate) rather than using the single-probe-item `_upload_probe()`
        helper, since the behaviour under test is inherently about item-pair
        geometry, not a single item's parameters.

        Observational elsewhere: only PX4 is known to implement this specific
        check; the mock has no feasibility logic and will accept the mission
        unmodified — itself useful confirmation that this is a PX4-specific
        safety check, not a protocol requirement.
        """
        lat_int = _LAT_INT + 60000
        lon_int = _LON_INT + 60000
        wp = MissionItem(
            seq=0, frame=_FRAME, command=16,  # MAV_CMD_NAV_WAYPOINT
            current=1, autocontinue=1,
            param1=0.0, param2=0.0, param3=0.0, param4=NAN,
            x=lat_int, y=lon_int, z=30.0,
            mission_type=0,
        )
        gate = MissionItem(
            seq=1, frame=_FRAME, command=SPEC.cmd_id,
            current=0, autocontinue=1,
            param1=0.0, param2=0.0, param3=NAN, param4=NAN,
            x=lat_int, y=lon_int, z=30.0,  # identical location as wp — 0 m separation
            mission_type=0,
        )
        try:
            await raw_upload_mission_items(gcs_system_cls, [wp, gate], mission_type=0)
            log.warning(
                _FMT, _CMD, "gate coincident with waypoint",
                "NOTE: accepted despite zero separation — this stack has no near-waypoint "
                "feasibility check (expected on non-PX4 stacks)",
            )
        except RawMissionError as exc:
            reason = str(exc).split(":")[0].strip()
            log.info(_FMT, _CMD, "gate coincident with waypoint",
                     f"correctly REJECTED: {reason} — feasibility check confirmed (matches PR #761)")
        finally:
            await clear_all_mission_types(gcs_system_cls)
