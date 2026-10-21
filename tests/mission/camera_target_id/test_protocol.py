"""
Camera commands with a target camera ID — Tier 1 protocol tests (mission items).

One Tier1MissionTestBase class per command in camera_commands.CAMERA_COMMANDS
(generated below), so each gets the shared mandatory tests — baseline accepted,
undefined-param sentinel pair, defined-param sentinel tolerated, frame survey —
plus one bespoke check: is the target camera ID stored as sent? Each class reports
under its own command (`reports/mission_<command>_...`).

Every item uses MAV_FRAME_MISSION: PX4 accepts these non-position DO items only in
that frame (see tests/mission/do_set_actuator/CLAUDE.md).

Running
-------
    pytest tests/mission/camera_target_id/test_protocol.py -v --log-cli-level=INFO   # mock
    pytest tests/mission/camera_target_id/test_protocol.py --drone-address=udp://:14540 \\
        --px4-sitl=~/github/px4/PX4-Autopilot-camera --px4-model=sihsim_quadx \\
        --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO
"""

import logging

import pytest
from mavsdk.plugins.mission_raw import MissionRawError

from tests import report
from ..conftest import MissionItemSpec, Tier1MissionTestBase, clear_all_mission_types, _check
from .camera_commands import CAMERA_COMMANDS, MAV_FRAME_MISSION, item_params, xml_params

log = logging.getLogger(__name__)

# Two real MAVLink camera component IDs (7-255) — neither is the 100 fallback.
STORED_IDS = (101.0, 200.0)


class _CameraIdChecks:
    """The bespoke Tier 1 check every camera command class shares."""

    CAMERA_CMD = None  # the camera_commands.CameraCommand this class tests

    async def test_target_camera_id_preserved(self, gcs_system_cls, mock_stack_cls, home_item_for_mission, request):
        """The target camera ID is stored as sent (101 and 200), or the item is NACKed."""
        cmd = self.CAMERA_CMD
        field = ("param1", "param2", "param3", "param4")[cmd.id_slot - 1]
        outcomes = []
        for camera_id in STORED_IDS:
            try:
                dl = await self._upload_probe(gcs_system_cls, home_item_for_mission, **{field: camera_id})
                got = getattr(dl, field)
                outcomes.append("PRESERVED" if abs(got - camera_id) < 1e-3 else f"ALTERED ({camera_id:.0f} -> {got})")
            except MissionRawError as exc:
                outcomes.append(f"NACKed ({str(exc).split(':')[0].strip()})")
            finally:
                await clear_all_mission_types(gcs_system_cls)
        detail = "; ".join(f"id {i:.0f}: {o}" for i, o in zip(STORED_IDS, outcomes))
        log.info("%s | target camera ID storage | %s", self.SPEC.name, detail)
        report.record_tier1_detail("mission", self.SPEC.name, f"param{cmd.id_slot} (target camera ID) — {detail}")
        report.record_nonsentinel_ack("mission", self.SPEC.name, f"{cmd.id_slot}_{xml_params(cmd.spec_name)[cmd.id_slot - 1].label}",
                                      nacked=all(o.startswith("NACKed") for o in outcomes))
        altered = [o for o in outcomes if o.startswith("ALTERED")]
        outcome = "ACCEPTED" if not altered else altered[0]
        _check(type(self), request, f"Target camera ID (param{cmd.id_slot}) stored as sent, or NACKed", outcome,
               expect=lambda o: o == "ACCEPTED",
               fail_reason=f"Target camera ID accepted but not stored ({detail}) — should have been NACKed")


# One class per MAV_CMD (the single-shot IMAGE_START_CAPTURE case is the same command).
for _cmd in CAMERA_COMMANDS:
    if _cmd.xml_name:
        continue
    _spec = MissionItemSpec(
        cmd_id=_cmd.cmd_id,
        name=_cmd.name,
        frame=MAV_FRAME_MISSION,
        baseline=item_params(_cmd, 0.0),
        params=list(xml_params(_cmd.spec_name)),
    )
    _cls_name = "Test" + "".join(w.capitalize() for w in _cmd.name.split("_"))
    globals()[_cls_name] = pytest.mark.asyncio(loop_scope="class")(
        type(_cls_name, (_CameraIdChecks, Tier1MissionTestBase), {"SPEC": _spec, "CAMERA_CMD": _cmd})
    )
