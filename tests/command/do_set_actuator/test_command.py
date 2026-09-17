"""
MAV_CMD_DO_SET_ACTUATOR (cmd=187) via COMMAND_INT/COMMAND_LONG — Tier 1 ACK tests.

Built specifically to verify PX4-Autopilot PR #28723 ("fix(mavlink): fix
COMMAND_INT/MISSION_ITEM_INT actuator scaling"), commits 0f029991b2 ("fix
COMMAND_INT actuator scaling") and ea5734b460 ("fix MISSION_ITEM_INT
actuator scaling") — checked out locally at ~/github/PX4/PX4-Autopilot
(branch onelittlechildawa/fix-command-int-actuator-scaling) at the time
this file was written.

The bug (from the two commits' own messages/diffs)
----------------------------------------------------
DO_SET_ACTUATOR's param5/param6 map to COMMAND_INT's `x`/`y` — genuinely
int32 wire fields — and are spec-defined to scale by 1e7 (like a location's
lat/lon) *regardless of frame*, even though this command has no location
semantics at all (hasLocation="false"). Before the fix, `mavlink_receiver.cpp`'s
`handle_message_command_int()` decoded x/y using the *shared location-frame*
logic — 1e4 for local/body frames, 1e7 only for global/other frames — so an
actuator command sent with a local/body frame decoded at the wrong scale.
Separately, the old "not used" check required **both** x AND y to equal
INT32_MAX to treat either as ignored (`if (x == INT32_MAX && y == INT32_MAX)`)
— spec-incorrect: each of param5/param6 must independently accept its own
sentinel. Both bugs are fixed via a new shared helper,
`mavlink_cmd_params::decode_scaled_int32_field()`, used identically by both
the COMMAND_INT and MISSION_ITEM_INT paths.

MAV_CMD_DO_SET_ACTUATOR parameter table (common.xml)
-----------------------------------------------------
  param1  Actuator 1  (-1..1, NaN=ignore)                          Defined (float, both message types)
  param2  Actuator 2  (-1..1, NaN=ignore)                          Defined (float, both message types)
  param3  Actuator 3  (-1..1, NaN=ignore)                          Defined (float, both message types)
  param4  Actuator 4  (-1..1, NaN=ignore)                          Defined (float, both message types)
  param5  Actuator 5  (x in COMMAND_INT, ×1e7, NaN=ignore in COMMAND_LONG)  Defined
  param6  Actuator 6  (y in COMMAND_INT, ×1e7, NaN=ignore in COMMAND_LONG)  Defined
  param7  Index       (z field — always float, minValue=0)         Defined

Per tests/command/CLAUDE.md's COMMAND_INT/COMMAND_LONG selection rule:
hasLocation="false" and params 5/6 carry genuine floats (not a location) in
COMMAND_LONG, so **COMMAND_LONG is this command's primary message type** —
unusual among this repo's existing command tests, most of which have
hasLocation="true" and default to COMMAND_INT. The mandatory common checks
(inherited from Tier1CommandTestBase) still exercise both message types.

IMPORTANT — what these tests can and cannot show
-------------------------------------------------
PX4's Commander.cpp answers DO_SET_ACTUATOR with an **unconditional
ACCEPTED**, with no value/range validation of any kind:

    case vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR:
        answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
        return true;

This means **Tier 1 (ACK-only) tests in this file cannot distinguish
correct from incorrect x/y scaling at all** — every combination gets
ACCEPTED regardless of whether the decoded actuator value is right, wrong,
or wildly out of range. This is confirmed by source, not assumed. The tests
below therefore cover only what Tier 1 genuinely can show (the command is
supported; params tolerate their documented sentinels) plus the bespoke
per-field-sentinel-independence probes are kept as protocol-level
regression/documentation coverage, explicitly NOT claimed as proof of
correct scaling. **The actual PR fix is conclusively verified in Tier 2**
(test_flight.py, this directory) via real PWM output observation, since
that's the only place in this stack where the decoded value is externally
visible.

Running
-------
Paired mock (no autopilot)::

    pytest tests/command/do_set_actuator/test_command.py -v --log-cli-level=INFO

PX4 SIH multicopter::

    pytest tests/command/do_set_actuator/test_command.py \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \\
        --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO
"""

import logging

import pytest

from tests.command.conftest import (
    CommandSpec,
    INT32_MAX,
    ParamSpec,
    Tier1CommandTestBase,
    _check,
    gcs_system_cls,  # noqa: F401 — see do_set_global_origin/test_command.py comment
    mock_stack_cls,  # noqa: F401
)
from tests.mock_flight_stack import MAV_RESULT_UNSUPPORTED

log = logging.getLogger(__name__)

_CMD = "DO_SET_ACTUATOR"
_CMD_ID = 187  # MAV_CMD_DO_SET_ACTUATOR

SPEC = CommandSpec(
    cmd_id=_CMD_ID,
    name=_CMD,
    # has_location/float_params5_6 both stay at their False default (see
    # tests/command/conftest.py's CommandSpec) despite param5/6 (Actuator
    # 5/6) being real, defined float values — common.xml's own text for
    # this command documents an explicit DUAL encoding ("If sent in
    # COMMAND_LONG: value is scaled from [-1 to 1]... If sent in
    # COMMAND_INT or MISSION_ITEM_INT: value is scaled by 1e7"), so
    # test_float_params5_6_rejects_command_int's message-type-exclusivity
    # expectation genuinely does not apply here — both message types are
    # spec-valid, which is exactly what this whole test file (and PR
    # #28723) verifies. See tests/command/CLAUDE.md § Mandatory common
    # tests, check 7.
    baseline=dict(
        param1=0.5, param2=None, param3=None, param4=None,   # Actuator 1 = 0.5, others ignored
        long5=None, long6=None, long7=0.0,                    # COMMAND_LONG: Actuator 5/6 ignored, Index=0
        int_x=INT32_MAX, int_y=INT32_MAX, int_z=0.0,          # COMMAND_INT: same, Index via z
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


@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(300)
class TestDoSetActuatorCommand(Tier1CommandTestBase):
    """
    DO_SET_ACTUATOR — ACK result tests, sent via both COMMAND_INT and
    COMMAND_LONG. The six mandatory common checks are inherited from
    Tier1CommandTestBase; the bespoke tests below add per-field-sentinel-
    independence regression coverage (see module docstring for why these
    cannot conclusively verify PR #28723's scaling fix — that's Tier 2's job).
    """

    SPEC = SPEC

    async def test_do_set_actuator_command_int_sentinel_independent_per_field(self, gcs_system_cls, mock_stack_cls, request):
        """
        COMMAND_INT: x=INT32_MAX (Actuator 5 ignored) paired with a real
        y (Actuator 6 used) is not UNSUPPORTED — regression coverage for
        the fix's "ignore sentinel applies per-field, not jointly" behaviour
        (commit 0f029991b2). ACK-only: confirms this combination isn't
        rejected, not that the decoded values are individually correct
        (Commander always ACCEPTs — see module docstring).
        """
        from tests.command.conftest import probe_command_int
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await probe_command_int(gcs_system_cls, command=_CMD_ID, param1=None, x=INT32_MAX, y=5_000_000, z=0.0)
        result = int(ack["result"]) if ack is not None else None
        description = "COMMAND_INT: x=INT32_MAX (ignore), y=5,000,000 (0.5) — not UNSUPPORTED"
        _check(type(self), request, description, result, expect=lambda r: r != MAV_RESULT_UNSUPPORTED)

    async def test_do_set_actuator_command_int_local_frame_not_denied(self, gcs_system_cls, mock_stack_cls, request):
        """
        COMMAND_INT: a real actuator x value sent under MAV_FRAME_LOCAL_NED
        (frame=1) — the frame that triggered the pre-fix 1e4 mis-scaling
        (commit 0f029991b2's own commit message: "shared location path
        uses 1e4 for local and body frames") — is not UNSUPPORTED. ACK-only
        regression guard; see Tier 2 for the actual scaling-correctness check.
        """
        from tests.command.conftest import probe_command_int
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await probe_command_int(gcs_system_cls, command=_CMD_ID, frame=1, param1=None, x=5_000_000, y=INT32_MAX, z=0.0)
        result = int(ack["result"]) if ack is not None else None
        description = "COMMAND_INT: x=5,000,000 under frame=1 (LOCAL_NED) — not UNSUPPORTED"
        _check(type(self), request, description, result, expect=lambda r: r != MAV_RESULT_UNSUPPORTED)
