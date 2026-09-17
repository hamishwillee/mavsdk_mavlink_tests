"""
MAV_CMD_DO_SET_ACTUATOR (cmd=187) — Tier 1 protocol-acceptance tests.

Built specifically to verify PX4-Autopilot PR #28723 ("fix(mavlink): fix
COMMAND_INT/MISSION_ITEM_INT actuator scaling"), commits 0f029991b2 and
ea5734b460 — checked out locally at ~/github/PX4/PX4-Autopilot at the time
this file was written. See tests/command/do_set_actuator/test_command.py's
module docstring and this repo's CLAUDE.md for the full bug description;
this file covers the MISSION_ITEM_INT half specifically.

MAV_CMD_DO_SET_ACTUATOR parameter table (common.xml)
-----------------------------------------------------
  param1  Actuator 1  (-1..1, NaN=ignore)               Defined
  param2  Actuator 2  (-1..1, NaN=ignore)                Defined
  param3  Actuator 3  (-1..1, NaN=ignore)                Defined
  param4  Actuator 4  (-1..1, NaN=ignore)                Defined
  param5  Actuator 5  (x field) — INT32_MAX×1e7 in MISSION_ITEM_INT, NaN=ignore  Defined
  param6  Actuator 6  (y field) — INT32_MAX×1e7 in MISSION_ITEM_INT, NaN=ignore  Defined
  param7  Index       (z field, integer-valued float, minValue=0)  Defined

Unlike param1-4 (plain floats, no scaling question — a MISSION_ITEM_INT
float field round-trips as a float either way), param5/param6 map to
MISSION_ITEM_INT's `x`/`y` — genuinely int32 wire fields — and the spec
requires a *1e7 scaling* there ("If sent in COMMAND_INT or MISSION_ITEM_INT:
value is scaled by 1e7. INT32_MAX to ignore."), identical in kind to a
hasLocation command's lat/lon scaling but for a completely different
(non-location) semantic. This is the crux of what PR #28723 fixes for the
mission-item path: before the fix, `mavlink_mission.cpp`'s upload parser
stored the *raw, unscaled* integer as the internal actuator value, and the
download formatter had no case for this command's x/y mapping at all — the
round-trip was broken, not merely mis-scaled.

Because the fix stores/restores the internal "native -1..1" value
consistently, the most direct, discriminating Tier 1 evidence is an
upload→download round-trip at a *known, non-trivial* raw x/y value —
see the bespoke `test_do_set_actuator_*` tests below, using values chosen
to be unambiguous under 1e7 scaling (5,000,000 → 0.5) and clearly
distinguishable from the pre-fix "stored raw, or lost entirely" behaviour.
This command is ONLY a valid mission-item type under MAV_FRAME_MISSION
(frame=2) — confirmed by source and empirically (see
`test_do_set_actuator_requires_mission_frame`) — unlike a location command,
which the fix's "scaled by 1e7 regardless of frame" claim actually refers
to (once accepted, no frame changes the scaling factor; there is no
*other* valid frame for this command to compare against on the mission
side, unlike COMMAND_INT's local/body-frame branches — see
tests/command/do_set_actuator/test_flight.py's frame-independence test,
which does have alternate valid frames to compare against).

A pure round-trip test can't, on its own, rule out a *symmetric* scaling
bug (e.g. an internally-consistent-but-wrong 1e4 factor) — that absolute
check is what tests/command/do_set_actuator/test_flight.py's Tier 2
PWM-output observation provides instead (both paths share the same
`decode_scaled_int32_field()` helper per the PR diff, so a passing round
trip here is still strong, if not fully self-contained, evidence).

IMPORTANT — what these tests can and cannot show
-------------------------------------------------
Tier 1 (this file) proves the mission protocol stores and returns the
correct *raw wire* x/y for a given actuator value — i.e. that upload/
download fidelity is restored. It does NOT prove the vehicle's actuator
subsystem receives the correctly-scaled -1..1 value at execution time —
that needs a real command dispatch, which is Tier 2's job (test_flight.py,
this directory).

Running
-------
Against the mock (no autopilot required)::

    pytest tests/mission/do_set_actuator/test_protocol.py -v --log-cli-level=INFO

Against a real flight stack::

    pytest tests/mission/do_set_actuator/test_protocol.py --drone-address=udp://:14540 -v --log-cli-level=INFO
"""

import logging

import pytest

from ..conftest import MissionItemSpec, Tier1MissionTestBase, clear_all_mission_types
from tests.param_spec import ParamSpec

log = logging.getLogger(__name__)

_CMD = "DO_SET_ACTUATOR"
_FMT = "%-14s | %-44s | %s"

INT32_MAX = 0x7FFF_FFFF

# 0.5 scaled by 1e7 — an unambiguous, non-trivial actuator value (not 0 or a
# round PWM-adjacent number) chosen so a round-trip failure or a scaling
# error is impossible to mistake for a coincidental pass.
_ACTUATOR_HALF_INT = 5_000_000
_ACTUATOR_HALF = 0.5

# ---------------------------------------------------------------------------
# Tier 1 spec
# ---------------------------------------------------------------------------

SPEC = MissionItemSpec(
    cmd_id=187,
    name=_CMD,
    mission_type=0,
    # MAV_FRAME_MISSION (2) — confirmed by source (mavlink_mission.cpp's
    # parse_mavlink_mission_item): DO_SET_ACTUATOR, like other non-location
    # DO_* action commands (e.g. DO_CHANGE_SPEED — see tests/mission/CLAUDE.md's
    # PX4 behaviour notes), is only recognised as a valid mission-item command
    # under this frame; any other frame value falls into the location-command
    # switch, where it isn't a listed case, and the item is rejected outright
    # (MAV_MISSION_UNSUPPORTED) — a real, PR-unrelated finding hit while first
    # writing this file with frame=5 (see git history/PR notes for this repo).
    frame=2,

    baseline=dict(
        param1=_ACTUATOR_HALF,  # Actuator 1: 0.5
        param2=float("nan"),    # Actuator 2: ignore
        param3=float("nan"),    # Actuator 3: ignore
        param4=float("nan"),    # Actuator 4: ignore
        x=INT32_MAX,            # Actuator 5: ignore
        y=INT32_MAX,            # Actuator 6: ignore
        z=0.0,                  # Index: actuator set 0 (the default/only handled set)
    ),
    params=[
        ParamSpec(1, "Actuator 1", defined=True),
        ParamSpec(2, "Actuator 2", defined=True),
        ParamSpec(3, "Actuator 3", defined=True),
        ParamSpec(4, "Actuator 4", defined=True),
        ParamSpec(5, "Actuator 5", defined=True),
        ParamSpec(6, "Actuator 6", defined=True),
        ParamSpec(7, "Index", defined=True),
    ],
)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


@pytest.mark.asyncio(loop_scope="class")
class TestDoSetActuator(Tier1MissionTestBase):
    """Protocol-acceptance tests for MAV_CMD_DO_SET_ACTUATOR (cmd=187) as a mission item."""

    SPEC = SPEC

    async def test_do_set_actuator_actuator5_scales_by_1e7(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """
        param5/x (Actuator 5): uploading a raw x=5,000,000 (0.5 scaled by 1e7)
        downloads back as the same raw integer — the core PR #28723 fix
        (mavlink_mission.cpp previously stored/returned this unscaled or not
        at all; see module docstring).
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, x=_ACTUATOR_HALF_INT)
            ok = dl.x == _ACTUATOR_HALF_INT
            log.info(_FMT, _CMD, "param5 (Actuator 5) = 5,000,000 raw",
                     f"PRESERVED ({dl.x})" if ok else f"NOT preserved (downloaded {dl.x})")
            assert ok, (
                f"param5/x round-trip broken: uploaded {_ACTUATOR_HALF_INT}, downloaded {dl.x} — "
                "PR #28723's MISSION_ITEM_INT actuator-scaling fix is not present/working on this build"
            )
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_do_set_actuator_actuator6_scales_by_1e7(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param6/y (Actuator 6): same round-trip check as Actuator 5, independently — mirrors the x/y symmetry in the fix."""
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, y=_ACTUATOR_HALF_INT)
            ok = dl.y == _ACTUATOR_HALF_INT
            log.info(_FMT, _CMD, "param6 (Actuator 6) = 5,000,000 raw",
                     f"PRESERVED ({dl.y})" if ok else f"NOT preserved (downloaded {dl.y})")
            assert ok, (
                f"param6/y round-trip broken: uploaded {_ACTUATOR_HALF_INT}, downloaded {dl.y} — "
                "PR #28723's MISSION_ITEM_INT actuator-scaling fix is not present/working on this build"
            )
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_do_set_actuator_sentinel_independent_per_field(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """
        x=INT32_MAX (Actuator 5 ignored) paired with y=5,000,000 (Actuator 6
        used) round-trips with EACH field independently preserved — the
        fix's other half: "make the not-used int32 sentinel apply
        independently per param5/param6 ... instead of requiring both
        fields to match" (commit ea5734b460).
        """
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, x=INT32_MAX, y=_ACTUATOR_HALF_INT)
            x_ok = dl.x == INT32_MAX
            y_ok = dl.y == _ACTUATOR_HALF_INT
            log.info(_FMT, _CMD, "param5=ignore, param6=0.5 (mixed)",
                     f"x={'IGNORED' if x_ok else dl.x} y={'0.5 (' + str(dl.y) + ')' if y_ok else dl.y}")
            assert x_ok and y_ok, (
                f"Per-field sentinel independence broken: uploaded x=INT32_MAX/y={_ACTUATOR_HALF_INT}, "
                f"downloaded x={dl.x}/y={dl.y} — expected x to stay INT32_MAX (ignored) and y to stay "
                f"{_ACTUATOR_HALF_INT} independently, not clobber each other"
            )
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_do_set_actuator_index_preserved(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """param7/z (Index) — a plain float in both message types, no scaling question; round-trips at index=1 (actuator set 1)."""
        try:
            dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, z=1.0)
            ok = abs(dl.z - 1.0) < 1e-4
            log.info(_FMT, _CMD, "param7 (Index) = 1", f"PRESERVED ({dl.z})" if ok else f"NOT preserved (downloaded {dl.z})")
            assert ok, f"param7 (Index) not preserved: uploaded 1.0, downloaded {dl.z}"
        finally:
            await clear_all_mission_types(gcs_system_cls)

    async def test_do_set_actuator_requires_mission_frame(self, gcs_system_cls, mock_stack_cls, home_item_for_mission):
        """
        Characterisation, not part of PR #28723's own scope: MAV_FRAME_GLOBAL_INT
        (frame=5), a valid frame for location commands, is REJECTED for
        DO_SET_ACTUATOR (MAV_MISSION_UNSUPPORTED) — confirmed by source
        (mavlink_mission.cpp's parse_mavlink_mission_item): non-location DO_*
        action commands are only recognised as valid mission items under
        MAV_FRAME_MISSION (the class's SPEC.frame=2 default), matching
        DO_CHANGE_SPEED's documented behaviour (tests/mission/CLAUDE.md).
        Real assertion against a real stack (upload must fail — NACKed as
        MAV_MISSION_UNSUPPORTED); observational-only against the mock, which
        accepts any frame by default (it doesn't replicate PX4's per-command
        frame-validation logic) and would otherwise make this test flaky/wrong.
        """
        outcome = await self._probe_outcome(gcs_system_cls, home_item_for_mission, frame=5, x=_ACTUATOR_HALF_INT)
        log.info(_FMT, _CMD, "frame=5 (GLOBAL_INT, not MISSION)", outcome)
        if mock_stack_cls is None:
            assert outcome != "ACCEPTED", (
                f"Expected frame=5 (GLOBAL_INT) to be rejected for DO_SET_ACTUATOR "
                f"(MAV_FRAME_MISSION required), but upload was ACCEPTED"
            )
