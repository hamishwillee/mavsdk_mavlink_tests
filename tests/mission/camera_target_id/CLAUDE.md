# Camera commands — target camera ID in missions

Added 2026-10-01 to verify PX4 commit `206bdc39f0` ("fix(navigator): use target
camera id for all camera commands in missions", branch
`hamishwillee/mission_camera_target_id`). Covers every mission camera command
with a target camera ID param — IMAGE_START/STOP_CAPTURE, SET_CAMERA_MODE,
SET_CAMERA_SOURCE, VIDEO_START/STOP_CAPTURE, SET_CAMERA_ZOOM/FOCUS,
DO_SET_CAM_TRIGG_INTERVAL/DIST, DO_TRIGGER_CONTROL — in one directory, because
the question is the same for all of them and one flight can answer it for all.

## The question, and what's spec vs PX4

**Main question (user-specified): is a set camera ID respected?** common.xml:
id 7–255 is "MAVLink camera component id", and in a mission the autopilot should
"resend it as a command ... setting the command's target_component". So a set
id → re-emitted with `target_component = id` is a compatibility check
(`test_camera_compat_target_id_respected`), two ids per command (101, 200 —
rule 4c; neither is the 100 fallback, so a stack that ignores the id can't pass).

**Secondary, PX4's convention (`info`)**: unset id (0 or NaN) → camera commands
to MAV_COMP_ID_CAMERA (100), trigger commands to MAV_COMP_ID_ALL (0, since PX4's
own camera_trigger handles those too); and the IMAGE_STOP_CAPTURE PX4 sends when
a mission is paused goes to the camera that was started. The spec says id 0 is
"all cameras" (do both: local and resend), and in principle an autopilot should
find a connected camera rather than assume 100 — so these are PX4-behaviour
checks, not compliance.

## Method

- PX4 re-emits mission camera commands as COMMAND_LONG on every MAVLink link
  (`streams/COMMAND_LONG.hpp`), so the test GCS sees each one with its real
  target_component. Verified first against unpatched `main` (2026-10-01):
  IMAGE_START/STOP with id 101 arrived at 1/101, VIDEO_START with id 101 at
  1/100 — exactly the behaviour the commit changes.
- One flight per id case (101, 200, 0, NaN): takeoff, all twelve camera items,
  a waypoint. Each item's re-emitted command is found by command id plus a
  distinctive param value (`camera_commands.py`), because PX4 also sends
  commands of its own (gimbal 1000/1001, its own DO_TRIGGER_CONTROL with
  param1 = −1). Single-shot IMAGE_START_CAPTURE is re-emitted as
  DO_DIGICAM_CONTROL (param5 = 1) and is its own case.
- Each item is upload-probed alone first; a NACKed one is left out of the
  flight and reported REJECTED (rule 4a) instead of failing the mission upload
  (unpatched PX4 NACKs SET_CAMERA_ZOOM).
- Flights are cached per case (rule 7). One module reports into eleven command
  reports via `_tier2_key_for(node)` — a hook added to
  `tests/flight_helpers.py`'s Tier 2 recorder for exactly this.
- Tier 1 (`test_protocol.py`): one generated `Tier1MissionTestBase` class per
  command, params from common.xml, MAV_FRAME_MISSION, plus "is the id stored".

## Not tested — methodology notes

- **Camera discovery (user note, 2026-10-01).** PX4 should in theory not fall
  back to 100: it would first look for connected cameras (components whose
  HEARTBEAT type is MAV_TYPE_CAMERA) and use those — though 100 is a common
  convention. To validate that a discovered camera is found and used, add a fake
  camera component to the link: e.g. a second MAVSDK endpoint configured as a
  camera (MAVSDK's camera server) at a non-100 component id, sending camera
  heartbeats; then check unset-id commands go to that id, and that set ids still
  win.
- **ids 1–6** ("cameras attached to the autopilot"): the spec says execute
  locally, not resend; PX4 resends them to component 1–6. Not covered.
- A real camera's ACK isn't needed: the target_component is read from what the
  autopilot sends, and PX4's retransmissions are ignored (first match wins).

## Results

### PX4 `206bdc39f0` (the change), SIH multicopter, 2026-10-01

All 12 cases × ids 101 and 200: re-emitted with `target_component` = the id —
`test_camera_compat_target_id_respected` PASS for every command, single-shot
capture included. Unset (0 and NaN): camera commands → 100, trigger commands →
0 — `test_camera_info_unset_id_fallback` PASS for all. Pausing the mission
while capturing: IMAGE_STOP_CAPTURE to 101 / 200 / 100 (id 101 / 200 / unset)
— PASS. Nothing NACKed at upload.

**Separate PX4 bug — back-to-back camera items lose commands.** With the 12
camera items consecutive (no NAV_DELAY), only the last 8 are re-emitted; the
first 4 (IMAGE_START_CAPTURE ×2, IMAGE_STOP_CAPTURE, SET_CAMERA_MODE) never
appear. 8 is `VehicleCommand.ORB_QUEUE_LENGTH`: PX4's COMMAND_LONG stream
(`streams/COMMAND_LONG.hpp`) reads queued vehicle_commands each cycle, and
older ones are overwritten when more than 8 are published between reads.
Reproduced 3 times; spaced 1 s apart, all 12 arrive. Not caused by the change:
unpatched `main` (`7cb65787b3`) also delivers exactly 8 of its 11 consecutive
items (ZOOM is NACKed there), losing the first 3. `test_camera_info_consecutive_items_all_reemitted`
records it (FAIL); the routing tests space items so they test routing only.

**SET_CAMERA_ZOOM has no param-validation entry.** Tier 1: it accepts real
values in its reserved params 4–7 (compatibility errors), unlike every other
command here — `mavlink_command_params.hpp` has entries for the other ten but
not for 531, and the change now accepts it at mission upload. Suggested:
`{  531, 0x07, 0x07 }, // SET_CAMERA_ZOOM: p1:zoom_type,p2:value,p3:camera_id`
(matching SET_CAMERA_FOCUS).
**Fixed in `c0c3dbe7fd`** ("fix(mavlink): validate params of
MAV_CMD_SET_CAMERA_ZOOM", that exact line): re-run on it, SET_CAMERA_ZOOM's
reserved params are NACKed — whole suite 180 passed, 1 failed (only the
consecutive-items test, the older queue bug).

### Negative control — unpatched `main` `7cb65787b3`, 2026-10-01

The tests fail exactly where the change alters behaviour (set id 101/200):
single-shot IMAGE_START_CAPTURE → 100 (the target was read after param1 was
overwritten), VIDEO_START/STOP_CAPTURE → 100, SET_CAMERA_FOCUS and the three
trigger commands → 0 — 7 compat FAILs; SET_CAMERA_ZOOM NACKed at upload
(UNSUPPORTED — a legitimate REJECTED). Unset: SET_CAMERA_FOCUS → 0, not 100
(info FAIL). Already correct before the change: IMAGE_START (multi) /
IMAGE_STOP_CAPTURE, SET_CAMERA_MODE, SET_CAMERA_SOURCE.
