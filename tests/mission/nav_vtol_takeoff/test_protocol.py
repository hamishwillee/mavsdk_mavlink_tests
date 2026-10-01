"""
MAV_CMD_NAV_VTOL_TAKEOFF (cmd=84) — Tier 1 protocol-acceptance tests (mission item).

"Takeoff from ground using VTOL mode, and transition to forward flight with
specified heading. The command should be ignored by vehicles that dont support
both VTOL and fixed-wing flight (multicopters, boats, etc.)."

Tests whether the mission protocol accepts the command and stores each parameter
faithfully. Does NOT verify execution — see test_flight.py (Tier 2).

Built on tests/mission/conftest.py's Tier1MissionTestBase/MissionItemSpec: the
shared mandatory tests (baseline accepted, undefined-param sentinel pair,
defined-param sentinel tolerated, frame validation survey) are inherited; this
file adds NAV_VTOL_TAKEOFF's own bespoke per-parameter tests.

Round-trip evidence is asymmetric (same as nav_takeoff/test_protocol.py): a
value NOT preserved means the stack didn't store it and should have NACKed it
(rule 4 — reported as a compatibility error); a value preserved says nothing
about whether execution uses it (that's Tier 2).

MAV_CMD_NAV_VTOL_TAKEOFF parameter table (common.xml)
-----------------------------------------------------
  param1  Empty
  param2  Transition Heading   enum VTOL_TRANSITION_HEADING (0-4)
  param3  Empty
  param4  Yaw Angle (deg)      NaN = use the current system yaw heading mode
  param5  Latitude  (x, int × 1e7)
  param6  Longitude (y, int × 1e7)
  param7  Altitude  (z, float m)

VTOL_TRANSITION_HEADING: 0 VEHICLE_DEFAULT, 1 NEXT_WAYPOINT, 2 TAKEOFF,
3 SPECIFIED (use param4), 4 ANY.

Expected from source (blind read, 2026-10-01 — root CLAUDE.md Tier 2 pattern #5)
--------------------------------------------------------------------------------
- PX4 main (`mavlink_mission.cpp`): stores only param4 (yaw, wrapped to
  [0, 360)) for this command; `mavlink_command_params.hpp` lists
  `{ 84, 0x78, 0x7C }` (mission mask 0x78 = params 4-7), so a non-zero param1
  or param2 may be NACKed outright, if the mask is applied to position items.
- ArduPilot (`AP_Mission.cpp`): stores no param1-4 for this command at all —
  only the location — and its sanity check permits NaN only in param4. So
  every non-default param2/param4 value should come back zeroed (accepted but
  not stored → compatibility error), and NaN in param1-3 should be NACKed.

Running
-------
Against the mock (no autopilot required)::

    pytest tests/mission/nav_vtol_takeoff/test_protocol.py -v --log-cli-level=INFO

PX4 VTOL (Gazebo — root CLAUDE.md: prefer gz for PX4 VTOL)::

    pytest tests/mission/nav_vtol_takeoff/test_protocol.py --drone-address=udp://:14540 \\
        --px4-sitl=~/github/px4/PX4-Autopilot --px4-model=gz_standard_vtol \\
        --vehicle-type=vtol --autopilot=px4 -v --log-cli-level=INFO

ArduPlane QuadPlane::

    pytest tests/mission/nav_vtol_takeoff/test_protocol.py \\
        --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \\
        --ardupilot-sitl=~/github/ardupilot/ardupilot/build/sitl/bin/arduplane \\
        --ardupilot-model=quadplane --vehicle-type=quadplane --autopilot=ardupilot \\
        -v --log-cli-level=INFO
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

_CMD = "NAV_VTOL_TAKEOFF"
_CMD_NAME = _CMD  # read by tests/report.py's key_from_module()
_CMD_ID = 84  # MAV_CMD_NAV_VTOL_TAKEOFF
_FMT = "%-16s | %-44s | %s"

NAN = float("nan")
INT32_MAX = 0x7FFF_FFFF

# SIH/Gazebo simulator home (47.3977°N, 8.5456°E) — same convention as nav_takeoff.
_LAT_INT = 473977000
_LON_INT = 85456000

# VTOL_TRANSITION_HEADING (common.xml)
TRANSITION_HEADINGS = {
    0: "VEHICLE_DEFAULT",
    1: "NEXT_WAYPOINT",
    2: "TAKEOFF",
    3: "SPECIFIED",
    4: "ANY",
}
HEADING_SPECIFIED = 3

# ArduPilot's sanity_check_params() nan_mask for NAV_VTOL_TAKEOFF permits NaN only
# in param4 (AP_Mission.cpp) — a per-stack strictness on a sentinel this param's
# own text gives no meaning (same convention as nav_takeoff's Pitch/Flags).
_ARDUPILOT_NAN_MASK_REASON = (
    "ArduPilot's sanity_check_params() nan_mask permits NaN only in param4 (Yaw) for "
    "NAV_VTOL_TAKEOFF; this param's NaN sentinel is not itself spec-mandated"
)

SPEC = MissionItemSpec(
    cmd_id=_CMD_ID,
    name=_CMD,
    mission_type=0,
    frame=5,  # MAV_FRAME_GLOBAL_INT
    baseline=dict(
        param1=0.0,    # Empty — 0.0, not NaN: ArduPilot rejects NaN here (see generic sentinel test)
        param2=0.0,    # Transition Heading: VEHICLE_DEFAULT
        param3=0.0,    # Empty
        param4=NAN,    # Yaw: use the current heading mode
        x=_LAT_INT,
        y=_LON_INT,
        z=50.0,        # Altitude: 50 m AMSL
    ),
    params=[
        ParamSpec(1, "Empty", defined=False),
        ParamSpec(2, "Transition Heading", defined=True, sentinel_fail_reason=_ARDUPILOT_NAN_MASK_REASON),
        ParamSpec(3, "Empty", defined=False),
        ParamSpec(4, "Yaw Angle", defined=True),   # NaN sentinel IS spec-mandated here
        ParamSpec(5, "Latitude", defined=True),    # INT32_MAX sentinel IS spec-mandated
        ParamSpec(6, "Longitude", defined=True),
        ParamSpec(7, "Altitude", defined=True),
    ],
)


def _nack_reason(exc: Exception) -> str:
    return str(exc).split(":")[0].strip()


@pytest.mark.asyncio(loop_scope="class")
class TestNavVtolTakeoff(Tier1MissionTestBase):
    """Protocol-acceptance tests for MAV_CMD_NAV_VTOL_TAKEOFF (cmd=84) as a mission item."""

    SPEC = SPEC

    # ------------------------------------------------------------------
    # param2 (Transition Heading) — an enum: every defined value, one at a time
    # (root CLAUDE.md rule 4c exempts enumerations tested exhaustively here).
    # ------------------------------------------------------------------

    async def test_nav_vtol_takeoff_param2_transition_heading_values(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param2 (Transition Heading): every defined enum value is preserved or NACKed, never silently altered.

        Each VTOL_TRANSITION_HEADING value (0-4) is uploaded on its own. Per value:
        PRESERVED (stored as sent) or NACKed (a legitimate way not to support a value
        — rule 4a) are both fine; ALTERED (accepted, then stored as something else) is
        the compatibility error — the stack accepted a value it doesn't keep, so it
        can't act on it. VEHICLE_DEFAULT (0) can't distinguish "stored" from "not
        stored" (a stack that drops param2 returns 0 anyway); every other value can.
        """
        outcomes: dict[int, str] = {}
        for value, name in TRANSITION_HEADINGS.items():
            label = f"param2 = {value} ({name})"
            try:
                kw = dict(param2=float(value))
                if value == HEADING_SPECIFIED:
                    kw["param4"] = 90.0  # SPECIFIED means "use param4" — give it one
                dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, **kw)
                outcomes[value] = "PRESERVED" if abs(dl.param2 - value) < 1e-4 else f"ALTERED to {dl.param2}"
            except MissionRawError as exc:
                outcomes[value] = f"NACKed ({_nack_reason(exc)})"
            finally:
                await clear_all_mission_types(gcs_system_cls)
            log.info(_FMT, _CMD, label, outcomes[value])
        altered = {v: o for v, o in outcomes.items() if o.startswith("ALTERED")}
        summary = "; ".join(f"{v} {TRANSITION_HEADINGS[v]}: {o}" for v, o in outcomes.items())
        report.record_tier1_detail("mission", _CMD, f"param2 (Transition Heading) per value — {summary}")
        # Rule 10: every real (non-default) value NACKed is a verdict on its own;
        # anything accepted still needs Tier 2.
        report.record_nonsentinel_ack("mission", _CMD, "2_Transition Heading",
                                      nacked=all(o.startswith("NACKed") for v, o in outcomes.items() if v != 0))
        if altered:
            report.compat_fail(
                f"param2 (Transition Heading) accepted but not stored for "
                f"{', '.join(TRANSITION_HEADINGS[v] for v in altered)} — should have been NACKed: {summary}"
            )

    async def test_nav_vtol_takeoff_param2_transition_heading_out_of_range(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param2 (Transition Heading) = 5, one past the last enum value: characterisation, no assertion.

        The XML defines the enum but no rule for an undefined value; a NACK is the
        tidy answer, but storing or zeroing it is protocol-valid (rule 3).
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param2=5.0)
            outcome = "PRESERVED (5.0) — undefined enum value stored" if abs(dl.param2 - 5.0) < 1e-4 \
                else f"ALTERED to {dl.param2}"
        except MissionRawError as exc:
            outcome = f"NACKed ({_nack_reason(exc)})"
        finally:
            await clear_all_mission_types(gcs_system_cls)
        log.info(_FMT, _CMD, "param2 = 5 (out of range)", outcome)
        report.record_tier1_detail("mission", _CMD, f"param2 (Transition Heading) = 5 (out of range): {outcome}")

    # ------------------------------------------------------------------
    # param4 (Yaw Angle)
    # ------------------------------------------------------------------

    async def test_nav_vtol_takeoff_param4_yaw_preserved(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw Angle) = 90° with param2 = SPECIFIED: preserved, or NACKed.

        The value is sent with param2 = SPECIFIED, the one combination in which the
        XML says it decides the transition heading. If the stack NACKs it (e.g.
        because it rejects param2 = SPECIFIED), the same value is retried with
        param2 = VEHICLE_DEFAULT so param4's own storage is still checked.

        Only the SPECIFIED attempt counts as param4's ACK verdict (rule 10): with any
        other param2 the XML gives param4 no effect, so accepting it there says
        nothing about whether the stack can act on it.
        """
        attempts = [("param2=SPECIFIED", dict(param2=float(HEADING_SPECIFIED), param4=90.0)),
                    ("param2=VEHICLE_DEFAULT", dict(param2=0.0, param4=90.0))]
        seen = []
        specified_nacked = False
        for label, kw in attempts:
            try:
                dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, **kw)
            except MissionRawError as exc:
                log.info(_FMT, _CMD, f"param4 = 90° ({label})", f"NACKed ({_nack_reason(exc)})")
                seen.append(f"{label}: NACKed ({_nack_reason(exc)})")
                specified_nacked = specified_nacked or label == "param2=SPECIFIED"
                continue
            finally:
                await clear_all_mission_types(gcs_system_cls)
            ok = abs(dl.param4 - 90.0) < 1e-3
            log.info(_FMT, _CMD, f"param4 = 90° ({label})", f"PRESERVED ({dl.param4:.3f})" if ok else f"NOT preserved ({dl.param4})")
            seen.append(f"{label}: " + (f"PRESERVED ({dl.param4:.3f})" if ok else f"NOT preserved ({dl.param4})"))
            report.record_tier1_detail("mission", _CMD, f"param4 (Yaw Angle) = 90° — {'; '.join(seen)}")
            report.record_nonsentinel_ack("mission", _CMD, "4_Yaw Angle", nacked=specified_nacked)
            if not ok:
                report.compat_fail(
                    f"param4 (Yaw Angle) not preserved ({label}): uploaded 90.0, downloaded {dl.param4} — "
                    "accepted but not stored, should have been NACKed"
                )
            return
        log.info(_FMT, _CMD, "param4 = 90°", "NACKed with every param2 — rejects a real yaw value (legitimate)")
        report.record_tier1_detail("mission", _CMD, f"param4 (Yaw Angle) = 90° — {'; '.join(seen)}")
        report.record_nonsentinel_ack("mission", _CMD, "4_Yaw Angle", nacked=True)

    async def test_nav_vtol_takeoff_param4_yaw_nan(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw Angle) = NaN ("use current yaw heading mode") is accepted and stored as NaN.

        param4's NaN sentinel has a documented meaning, so a stack that accepts it
        must keep it — storing a number instead turns "use the current heading mode"
        into a specific heading.
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param4=NAN)
        except MissionRawError as exc:
            pytest.fail(f"param4 = NaN (spec-defined sentinel) NACKed: {_nack_reason(exc)}")
        finally:
            await clear_all_mission_types(gcs_system_cls)
        log.info(_FMT, _CMD, "param4 = NaN", "PRESERVED (NaN)" if math.isnan(dl.param4) else f"ALTERED to {dl.param4}")
        if not math.isnan(dl.param4):
            report.compat_fail(
                f"param4 = NaN (use current heading mode) stored as {dl.param4} — the sentinel's meaning is lost"
            )

    async def test_nav_vtol_takeoff_param4_yaw_zero(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw Angle) = 0° (north) is not aliased to NaN.

        0° is a real heading; NaN means "use the current heading mode". A stack that
        doesn't store param4 at all also returns 0, so a PASS here can be vacuous —
        this only catches the specific 0 → NaN aliasing.
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param4=0.0)
        except MissionRawError as exc:
            log.info(_FMT, _CMD, "param4 = 0°", f"NACKed ({_nack_reason(exc)})")
            return
        finally:
            await clear_all_mission_types(gcs_system_cls)
        log.info(_FMT, _CMD, "param4 = 0°", f"stored as {dl.param4}")
        if math.isnan(dl.param4):
            report.compat_fail("param4 = 0° (north) stored as NaN — a real heading aliased to 'use current heading mode'")

    async def test_nav_vtol_takeoff_param4_yaw_edge_values(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param4 (Yaw Angle) = -90° and 450°: characterisation — raw, wrapped, or altered.

        The XML gives no range for Yaw Angle, so any outcome is protocol-valid
        (rule 3); logged for the devguide and for cross-stack comparison.
        """
        seen = []
        for raw in (-90.0, 450.0):
            wrapped = raw % 360
            try:
                dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, param4=raw)
                if abs(dl.param4 - raw) < 1e-3:
                    outcome = f"PRESERVED raw ({raw:.0f}°)"
                elif abs(dl.param4 - wrapped) < 1e-3:
                    outcome = f"WRAPPED to {wrapped:.0f}°"
                else:
                    outcome = f"ALTERED to {dl.param4}"
            except MissionRawError as exc:
                outcome = f"NACKed ({_nack_reason(exc)})"
            finally:
                await clear_all_mission_types(gcs_system_cls)
            log.info(_FMT, _CMD, f"param4 = {raw:.0f}°", outcome)
            seen.append(f"{raw:.0f}°: {outcome}")
        report.record_tier1_detail("mission", _CMD, f"param4 (Yaw Angle) edge values — {'; '.join(seen)}")

    # ------------------------------------------------------------------
    # Location (params 5/6/7)
    # ------------------------------------------------------------------

    async def test_nav_vtol_takeoff_location_preserved(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """params 5/6/7 (Latitude/Longitude/Altitude): an explicit location round-trips."""
        lat_int, lon_int, alt = _LAT_INT + 10000, _LON_INT + 10000, 75.0
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
            report.compat_fail(
                f"Location not preserved: uploaded ({lat_int}, {lon_int}, {alt}), "
                f"downloaded ({dl.x}, {dl.y}, {dl.z}) — accepted but altered"
            )

    async def test_nav_vtol_takeoff_location_current_position(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """params 5/6 = INT32_MAX ("use current position"): accepted and preserved.

        Both x and y set together — the meaningful combination (the inherited
        per-slot sentinel tests send each on its own).
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, x=INT32_MAX, y=INT32_MAX)
        except MissionRawError as exc:
            pytest.fail(f"INT32_MAX lat/lon ('use current position', spec-defined) NACKed: {_nack_reason(exc)}")
        finally:
            await clear_all_mission_types(gcs_system_cls)
        ok = dl.x == INT32_MAX and dl.y == INT32_MAX
        log.info(_FMT, _CMD, "params 5/6 = INT32_MAX", "PRESERVED" if ok else f"ALTERED to x={dl.x}, y={dl.y}")
        if not ok:
            report.compat_fail(f"INT32_MAX lat/lon sentinel not preserved: stored x={dl.x}, y={dl.y}")

    async def test_nav_vtol_takeoff_location_nan_altitude(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param7 (Altitude) = NaN: characterisation — the XML gives Altitude no sentinel meaning."""
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, z=NAN)
            outcome = "PRESERVED (NaN)" if math.isnan(dl.z) else f"ALTERED to {dl.z}"
        except MissionRawError as exc:
            outcome = f"NACKed ({_nack_reason(exc)})"
        finally:
            await clear_all_mission_types(gcs_system_cls)
        log.info(_FMT, _CMD, "param7 = NaN", outcome)
        report.record_tier1_detail("mission", _CMD, f"param7 (Altitude) = NaN: {outcome}")
