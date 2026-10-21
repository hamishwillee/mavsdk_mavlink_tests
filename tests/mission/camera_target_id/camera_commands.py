"""
The camera commands whose mission items carry a "target camera ID" param, and how
to recognise each one when the autopilot re-emits it as a COMMAND_LONG.

Shared by test_protocol.py (Tier 1, one Tier1MissionTestBase class per command)
and test_flight.py (Tier 2). Param slots, labels and defined/reserved come from
common.xml, so the specs can't drift from the spec.

common.xml, for every one of these: "Target camera ID. 7 to 255: MAVLink camera
component id. 1 to 6 for cameras attached to the autopilot, which don't have a
distinct component id. 0: all cameras. ... It is also used to target specific
cameras when the MAV_CMD is used in a mission." IMAGE_START_CAPTURE adds: "When
used in a mission, an autopilot should execute the MAV_CMD for a specified local
camera (param1 = 1-6), or resend it as a command if it is intended for a MAVLink
camera (param1 = 7 - 255), setting the command's target_component. If the param1
is 0 the autopilot should do both."

So id 7-255 -> re-emitted with target_component = id is spec-defined. What an
autopilot does with id 0 / unset is its own convention; PX4 documents
(docs/en/camera/mavlink_v2_camera.md): camera commands go to MAV_COMP_ID_CAMERA
(100), the trigger commands to MAV_COMP_ID_ALL (0 — the autopilot's camera_trigger
handles them too).
"""

from __future__ import annotations

import functools
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

from tests.param_spec import INT32_MAX, ParamSpec

_COMMON_XML = Path(__file__).resolve().parents[3] / "mavlink" / "message_definitions" / "v1.0" / "common.xml"

MAV_COMP_ID_ALL = 0
MAV_COMP_ID_CAMERA = 100
MAV_FRAME_MISSION = 2  # PX4 accepts non-position DO items only in this frame
NAN = float("nan")


@dataclass
class CameraCommand:
    name: str               # the case name (the MAV_CMD name without MAV_CMD_, or a variant of it)
    cmd_id: int
    id_slot: int            # param slot (1-4) holding the target camera ID
    fallback: int           # target_component PX4 documents when the id isn't set
    params: dict            # slot -> value for a valid item (the id slot is set per case)
    xml_name: str = ""      # MAV_CMD name in common.xml, if different from `name`
    # How to find this item's re-emitted COMMAND_LONG among everything the autopilot
    # sends: the command it's re-emitted as, and COMMAND_LONG fields that must match.
    emitted_cmd: int | None = None
    signature: dict = field(default_factory=dict)

    @property
    def spec_name(self) -> str:
        return self.xml_name or self.name

    @property
    def emitted(self) -> int:
        return self.emitted_cmd if self.emitted_cmd is not None else self.cmd_id


# The commands PX4 206bdc39f0 ("use target camera id for all camera commands in
# missions") covers. Distinctive non-id values let each item's re-emitted command be
# told apart from commands PX4 sends on its own (e.g. its own DO_TRIGGER_CONTROL has
# param1 = -1, and gimbal commands 1000/1001 appear on any flight).
CAMERA_COMMANDS: list[CameraCommand] = [
    CameraCommand("IMAGE_START_CAPTURE", 2000, 1, MAV_COMP_ID_CAMERA, {2: 1.5, 3: 0.0, 4: 0.0},
                  signature={"param2": 1.5, "param3": 0.0}),
    # Single-shot capture (param3 = 1) is re-emitted as DO_DIGICAM_CONTROL, shoot in param5.
    CameraCommand("IMAGE_START_CAPTURE_SINGLE", 2000, 1, MAV_COMP_ID_CAMERA, {2: 0.0, 3: 1.0, 4: 1.0},
                  xml_name="IMAGE_START_CAPTURE", emitted_cmd=203, signature={"param5": 1.0}),
    CameraCommand("IMAGE_STOP_CAPTURE", 2001, 1, MAV_COMP_ID_CAMERA, {}),
    CameraCommand("SET_CAMERA_MODE", 530, 1, MAV_COMP_ID_CAMERA, {2: 1.0}, signature={"param2": 1.0}),
    CameraCommand("SET_CAMERA_SOURCE", 534, 1, MAV_COMP_ID_CAMERA, {2: 2.0, 3: 0.0}, signature={"param2": 2.0}),
    CameraCommand("VIDEO_START_CAPTURE", 2500, 3, MAV_COMP_ID_CAMERA, {1: 0.0, 2: 0.0}),
    CameraCommand("VIDEO_STOP_CAPTURE", 2501, 2, MAV_COMP_ID_CAMERA, {1: 0.0}),
    CameraCommand("SET_CAMERA_ZOOM", 531, 3, MAV_COMP_ID_CAMERA, {1: 2.0, 2: 50.0}, signature={"param2": 50.0}),
    CameraCommand("SET_CAMERA_FOCUS", 532, 3, MAV_COMP_ID_CAMERA, {1: 2.0, 2: 40.0}, signature={"param2": 40.0}),
    CameraCommand("DO_SET_CAM_TRIGG_INTERVAL", 214, 3, MAV_COMP_ID_ALL, {1: 1234.0, 2: 0.0},
                  signature={"param1": 1234.0}),
    CameraCommand("DO_SET_CAM_TRIGG_DIST", 206, 4, MAV_COMP_ID_ALL, {1: 25.0, 2: 0.0, 3: 0.0},
                  signature={"param1": 25.0}),
    CameraCommand("DO_TRIGGER_CONTROL", 2003, 4, MAV_COMP_ID_ALL, {1: 1.0, 2: 0.0, 3: -1.0},
                  signature={"param1": 1.0}),
]

BY_NAME = {c.name: c for c in CAMERA_COMMANDS}


@functools.cache
def xml_params(xml_name: str) -> tuple[ParamSpec, ...]:
    """ParamSpec for slots 1-7 from common.xml: defined unless reserved, 'Empty' or absent."""
    root = ET.parse(_COMMON_XML).getroot()
    entry = next(e for e in root.iter("entry") if e.get("name") == f"MAV_CMD_{xml_name}")
    by_index = {int(p.get("index")): p for p in entry.findall("param")}
    specs = []
    for slot in range(1, 8):
        p = by_index.get(slot)
        text = (p.text or "").strip() if p is not None else ""
        reserved = p is None or p.get("reserved") == "true" or text in ("", "Empty")
        specs.append(ParamSpec(slot, "Empty" if reserved else (p.get("label") or text), defined=not reserved))
    return tuple(specs)


def item_params(cmd: CameraCommand, camera_id: float) -> dict:
    """MissionItem param1-4/x/y/z: valid values, `camera_id` in the id slot, sentinels in undefined slots."""
    specs = {p.slot: p for p in xml_params(cmd.spec_name)}
    kw = {}
    for slot, name in ((1, "param1"), (2, "param2"), (3, "param3"), (4, "param4"), (7, "z")):
        if slot == cmd.id_slot:
            kw[name] = camera_id
        elif slot in cmd.params:
            kw[name] = cmd.params[slot]
        else:
            kw[name] = 0.0 if specs[slot].defined else NAN
    for slot, name in ((5, "x"), (6, "y")):
        kw[name] = int(cmd.params.get(slot, 0 if specs[slot].defined else INT32_MAX))
    return kw


def matches(cmd: CameraCommand, fields: dict) -> bool:
    """Is this COMMAND_LONG (fields as decoded by mavlink_direct) the re-emitted form of `cmd`'s item?"""
    if int(fields.get("command", -1)) != cmd.emitted:
        return False
    for k, v in cmd.signature.items():
        got = fields.get(k)
        if got is None or not math.isclose(float(got), v, abs_tol=1e-3):
            return False
    return True
