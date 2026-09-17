"""
Shared parameter-slot metadata for MAV_CMD-shaped protocols.

COMMAND_INT/COMMAND_LONG and MISSION_ITEM_INT all share the same param1-4
(float) / x,y (int32) / z (float) wire layout, slot for slot — a MAV_CMD's
7 parameter slots mean the same thing regardless of which message type
carries them.  ``ParamSpec`` captures that per-slot meaning (defined vs.
"Empty" in the XML, sentinel policy) once, for use by both
``tests/command/conftest.py``'s ``Tier1CommandTestBase`` (which sends a slot
via both COMMAND_INT and COMMAND_LONG — see ``sentinel_kwargs``/
``nonsentinel_kwargs``, consumed by ``probe_dual()``) and
``tests/mission/conftest.py``'s ``Tier1MissionTestBase`` (which sends a slot
via a single MISSION_ITEM_INT — see ``mission_sentinel_kwargs``/
``mission_nonsentinel_kwargs``, consumed by a ``MissionItem`` upload).

The two protocols differ in how a value is SENT (dual COMMAND_INT+COMMAND_LONG
vs. one MISSION_ITEM_INT) and how a RESULT is read back (a COMMAND_ACK result
code vs. an upload NACK or a downloaded, possibly-altered value) — neither
difference touches what a slot's wire value or sentinel *is*, so this
metadata is shared unchanged between them.  Protocol-specific transport and
result-interpretation logic stays in each package's own conftest.py.
"""

from dataclasses import dataclass

# Every MAV_FRAME value (common.xml) — shared by tests/command/conftest.py's
# Tier1CommandTestBase.test_frame_validation_survey (COMMAND_INT) and
# tests/mission/conftest.py's Tier1MissionTestBase.test_frame_validation_survey
# (MISSION_ITEM_INT): both send the command's own baseline under every listed
# frame and record whether the stack ever rejects one as unsupported — a
# generic, per-command-agnostic protocol probe, so the catalogue itself lives
# here rather than being duplicated per protocol.
MAV_FRAME_CATALOGUE: list[tuple[int, str]] = [
    (0, "MAV_FRAME_GLOBAL"),
    (1, "MAV_FRAME_LOCAL_NED"),
    (2, "MAV_FRAME_MISSION"),
    (3, "MAV_FRAME_GLOBAL_RELATIVE_ALT"),
    (4, "MAV_FRAME_LOCAL_ENU"),
    (5, "MAV_FRAME_GLOBAL_INT"),
    (6, "MAV_FRAME_GLOBAL_RELATIVE_ALT_INT"),
    (7, "MAV_FRAME_LOCAL_OFFSET_NED"),
    (8, "MAV_FRAME_BODY_NED"),
    (9, "MAV_FRAME_BODY_OFFSET_NED"),
    (10, "MAV_FRAME_GLOBAL_TERRAIN_ALT"),
    (11, "MAV_FRAME_GLOBAL_TERRAIN_ALT_INT"),
    (12, "MAV_FRAME_BODY_FRD"),
    (13, "MAV_FRAME_RESERVED_13"),
    (14, "MAV_FRAME_RESERVED_14"),
    (15, "MAV_FRAME_RESERVED_15"),
    (16, "MAV_FRAME_RESERVED_16"),
    (17, "MAV_FRAME_RESERVED_17"),
    (18, "MAV_FRAME_RESERVED_18"),
    (19, "MAV_FRAME_RESERVED_19"),
    (20, "MAV_FRAME_LOCAL_FRD"),
    (21, "MAV_FRAME_LOCAL_FLU"),
]

# Sentinel: "use current position" for int32 lat/lon fields — x/y in
# COMMAND_INT and MISSION_ITEM_INT, param5/6 in COMMAND_LONG (as a float).
INT32_MAX = 0x7FFF_FFFF

# Slot 5/6/7 are COMMAND_INT/MISSION_ITEM_INT's x/y/z — genuinely int32 for
# x/y (whether or not the command assigns them meaning) and always float for
# z, a fact of the wire struct, not of whether the command's spec uses that
# slot. COMMAND_LONG's equivalent fields (param5/6/7) are always float.
_SLOT_WIRE_FIELDS: dict[int, tuple[str, str]] = {
    5: ("long5", "int_x"),
    6: ("long6", "int_y"),
    7: ("long7", "int_z"),
}
# MISSION_ITEM_INT field name for each slot.
_MISSION_FIELDS: dict[int, str] = {
    1: "param1", 2: "param2", 3: "param3", 4: "param4",
    5: "x", 6: "y", 7: "z",
}
# A plausible non-sentinel value for the int32 x/y fields (SIH home lat/lon
# scale — arbitrary but realistic, mirrors external_wind_estimate's _REAL_INT).
_REAL_INT = 473977000


@dataclass
class ParamSpec:
    """
    Metadata for one MAV_CMD parameter slot (1-7).

    Drives the mandatory undefined/defined sentinel-pair Tier 1 tests in both
    tests/command/CLAUDE.md § Mandatory common tests (items 4 and 5) and its
    mission-protocol analogue in tests/mission/CLAUDE.md.
    """

    slot: int  # 1-7
    label: str  # human-readable name, e.g. "Wind speed" or "Empty"
    defined: bool  # False = no MAVLink meaning ("Empty" in the XML)
    sentinel_policy: str = "tolerate"  # defined params only: "tolerate"
        # (default) or "deny_required" (a mandatory field with no sentinel
        # fallback, e.g. DO_SET_GLOBAL_ORIGIN's lat/lon/altitude).
    reject_xfail_reason: str | None = None  # undefined params only: a
        # per-command known-behaviour xfail reason. Falls back to a generic
        # message (no known stack validates undefined params) if not given.
    sentinel_xfail_reason: str | None = None  # defined params with
        # sentinel_policy="tolerate" only: a known stack legitimately rejects
        # THIS param's own sentinel (e.g. ArduPilot's blanket sanity_check_params
        # nan_mask, which permits NaN only in a command-specific subset of
        # float params) even though the sentinel isn't itself spec-mandated
        # for this param — mirrors tests/mission/CLAUDE.md's documented
        # "defined float param NaN: no hard assertion, stack may legitimately
        # require an explicit value" convention. Leave None (hard-fail on
        # rejection) when the sentinel IS spec-mandated for this param (e.g.
        # a hasLocation param's INT32_MAX "use current position") and a
        # rejection is a genuine, worth-surfacing compliance gap.

    # -- COMMAND_INT + COMMAND_LONG dual-transport kwargs (probe_dual()) ----

    @property
    def sentinel_kwargs(self) -> dict:
        """probe_dual() kwargs that set this slot to its own sentinel value."""
        if self.slot <= 4:
            return {f"param{self.slot}": None}
        long_name, int_name = _SLOT_WIRE_FIELDS[self.slot]
        if int_name == "int_z":  # always float — NaN in both message types
            return {long_name: None, int_name: None}
        return {long_name: None, int_name: INT32_MAX}  # int_x / int_y

    @property
    def nonsentinel_kwargs(self) -> dict:
        """probe_dual() kwargs that set this slot to a real, non-sentinel value."""
        if self.slot <= 4:
            return {f"param{self.slot}": 1.0}
        long_name, int_name = _SLOT_WIRE_FIELDS[self.slot]
        if int_name == "int_z":
            return {long_name: 1.0, int_name: 1.0}
        return {long_name: 1.0, int_name: _REAL_INT}

    # -- single MISSION_ITEM_INT kwargs (Tier1MissionTestBase) --------------

    @property
    def mission_field(self) -> str:
        """The MissionItem constructor field name this slot maps to."""
        return _MISSION_FIELDS[self.slot]

    @property
    def mission_sentinel_kwargs(self) -> dict:
        """MissionItem kwargs that set this slot to its own sentinel value."""
        if self.slot in (5, 6):
            return {self.mission_field: INT32_MAX}
        return {self.mission_field: float("nan")}

    @property
    def mission_nonsentinel_kwargs(self) -> dict:
        """MissionItem kwargs that set this slot to a real, non-sentinel value."""
        if self.slot in (5, 6):
            return {self.mission_field: _REAL_INT}
        return {self.mission_field: 1.0}
