"""
EXTENDED_SYS_STATE (message id 245) — https://mavlink.io/en/messages/common.html#EXTENDED_SYS_STATE

What the XML defines (README.md in this directory has the reasoning and results):

- `vtol_state` (MAV_VTOL_STATE): "The VTOL state if applicable. Is set to
  MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration."
- `landed_state` (MAV_LANDED_STATE): "Is set to MAV_LANDED_STATE_UNDEFINED if
  landed state is unknown."

So, with the vehicle disarmed on the ground:
- landed_state is a MAV_LANDED_STATE value, and ON_GROUND (or UNDEFINED, if
  the stack doesn't know) — anything else contradicts where it is (compat).
- vtol_state is a MAV_VTOL_STATE value, checked against the vehicle's own
  HEARTBEAT type (never --vehicle-type, root CLAUDE.md rule 5) — compat:
  a MAV_TYPE_VTOL_* type must send a VTOL state (not UNDEFINED); a multirotor
  or ground/surface type can't be "in VTOL configuration", so must send
  UNDEFINED. A fixed-wing (or any other) type is characterised only: ArduPilot
  reports a standard QuadPlane as MAV_TYPE_FIXED_WING (Q_MAV_TYPE 0), so a VTOL
  state there is a VTOL the HEARTBEAT doesn't announce, not an error.
"""

from __future__ import annotations

import logging
from collections import Counter

import pytest

from tests import report
from tests.messages_telemetry.conftest import (
    record_detail,
    record_field_fact,
    skip_untestable,
    vehicle_heartbeat,
)
from tests.messages_telemetry.emission import Emission

log = logging.getLogger(__name__)

_MSG_NAME = "EXTENDED_SYS_STATE"
_MSG_ID = 245
REQUIRED_MESSAGES = {"EXTENDED_SYS_STATE": "the message under test"}
report.declare_params(report.MESSAGE_PROTOCOL, _MSG_NAME, ["vtol_state", "landed_state"])

pytestmark = pytest.mark.timeout(180)

EMISSION = Emission(_MSG_NAME, _MSG_ID)
EMISSION.define_tests(globals(), "extended_sys_state")

VTOL_STATES = {0: "UNDEFINED", 1: "TRANSITION_TO_FW", 2: "TRANSITION_TO_MC", 3: "MC", 4: "FW"}
LANDED_STATES = {0: "UNDEFINED", 1: "ON_GROUND", 2: "IN_AIR", 3: "TAKEOFF", 4: "LANDING"}
# MAV_TYPE values that are VTOL configurations (MAV_TYPE_VTOL_*; 19-25).
VTOL_MAV_TYPES = set(range(19, 26))
# Types that can't be a VTOL configuration: QUADROTOR, COAXIAL, HELICOPTER,
# HEXAROTOR, OCTOROTOR, TRICOPTER, DODECAROTOR, DECAROTOR; GROUND_ROVER,
# SURFACE_BOAT, SUBMARINE.
NOT_VTOL_MAV_TYPES = {2, 3, 4, 13, 14, 15, 29, 35, 10, 11, 12}


def _counts(values, names) -> str:
    return ", ".join(f"{names.get(v, v)}×{n}" for v, n in Counter(values).most_common())


async def test_extended_sys_state_compat_landed_state_on_ground(gcs_system, request):
    """landed_state is a MAV_LANDED_STATE value, and ON_GROUND (or UNDEFINED) for a disarmed vehicle on the ground."""
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        skip_untestable(request, ["landed_state"], "not enough messages to analyse", w.na)
    values = [int(s.fields["landed_state"]) for s in w.samples]
    unknown = [v for v in values if v not in LANDED_STATES]
    wrong = [v for v in values if v in (2, 3, 4)]
    seen = _counts(values, LANDED_STATES)
    record_detail(request, f"{seen} ({len(values)} msgs)")
    if unknown or wrong:
        record_field_fact(request, "landed_state", supported=False)
        report.compat_fail(f"landed_state {seen} for a disarmed vehicle on the ground")
    if all(v == 0 for v in values):
        record_field_fact(request, "landed_state", supported=False, notes="Always UNDEFINED")
    else:
        record_field_fact(request, "landed_state", supported=True)


async def test_extended_sys_state_compat_vtol_state_matches_type(gcs_system, request):
    """vtol_state is UNDEFINED unless the vehicle's HEARTBEAT type is a VTOL, and a VTOL state if it is."""
    w = await EMISSION.sample_window(gcs_system)
    if w.na:
        skip_untestable(request, ["vtol_state"], "not enough messages to analyse", w.na)
    hb = await vehicle_heartbeat(gcs_system)
    mav_type = hb.get("type")
    if mav_type is None:
        skip_untestable(request, ["vtol_state"], "no HEARTBEAT type to compare against")
    values = [int(s.fields["vtol_state"]) for s in w.samples]
    seen = _counts(values, VTOL_STATES)
    unknown = [v for v in values if v not in VTOL_STATES]
    if mav_type in VTOL_MAV_TYPES:
        kind, wrong, expect = "VTOL", [v for v in values if v == 0], "a VTOL state (not UNDEFINED)"
    elif mav_type in NOT_VTOL_MAV_TYPES:
        kind, wrong, expect = "can't be a VTOL", [v for v in values if v != 0], "UNDEFINED"
    else:
        kind, wrong, expect = "type doesn't say", [], "either"
    record_detail(request, f"HEARTBEAT type {mav_type} ({kind}): {seen}, expected {expect}")
    if unknown or wrong:
        record_field_fact(request, "vtol_state", supported=False)
        report.compat_fail(f"vtol_state {seen} for HEARTBEAT type {mav_type} ({kind}); expected {expect}")
    if mav_type in VTOL_MAV_TYPES:
        record_field_fact(request, "vtol_state", supported=True)
    elif any(values):
        record_field_fact(request, "vtol_state", supported=True,
                          notes=f"VTOL state with HEARTBEAT type {mav_type} (a VTOL the type doesn't announce)")
    else:
        # UNDEFINED is the right answer, but it's also what a stack that doesn't
        # fill the field sends: only a VTOL can show the field is supported.
        record_field_fact(request, "vtol_state", untestable="not a VTOL: UNDEFINED either way")
