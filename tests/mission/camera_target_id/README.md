# Camera commands — target camera ID in missions

Do mission camera items send their command to the camera they name? Covers
every mission camera command with a target camera ID param:
IMAGE_START_CAPTURE (multi and single-shot), IMAGE_STOP_CAPTURE,
SET_CAMERA_MODE, SET_CAMERA_SOURCE, VIDEO_START/STOP_CAPTURE,
SET_CAMERA_ZOOM, SET_CAMERA_FOCUS, DO_SET_CAM_TRIGG_INTERVAL,
DO_SET_CAM_TRIGG_DIST and DO_TRIGGER_CONTROL. Method and findings are in
`CLAUDE.md`.

## What's tested

| Test | Category | Check |
|------|----------|-------|
| `test_camera_compat_target_id_respected[<cmd>]` | compat (spec) | id 101 and 200 → re-emitted with `target_component` = id |
| `test_camera_info_unset_id_fallback[<cmd>]` | info (PX4 docs) | id 0 and NaN → 100 (trigger commands: 0) |
| `test_camera_info_pause_stops_started_camera` | info (PX4) | pausing while capturing sends IMAGE_STOP_CAPTURE to the started camera |
| `test_camera_info_consecutive_items_all_reemitted` | info | 12 camera items back to back are all re-emitted |
| `test_protocol.py` | Tier 1 | per command: shared mandatory checks plus "the id is stored as sent" |

The test GCS reads each re-emitted COMMAND_LONG's `target_component`
directly, so no camera is needed. The tests don't cover camera discovery (an
unset id going to a connected camera found by its HEARTBEAT, rather than to
100). See `CLAUDE.md` for how a fake camera component could test it.

## Running

```bash
pytest tests/mission/camera_target_id/ -v --log-cli-level=INFO      # mock: Tier 1 only
pytest tests/mission/camera_target_id/ --drone-address=udp://:14540 --px4-sitl=<PX4 checkout> \
    --px4-model=sihsim_quadx --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO
```

About 10 minutes on PX4 SIH (seven short flights).

## Results — 2026-10-01

| | PX4 with the change (`c0c3dbe7fd`) | PX4 `main` before it (`7cb65787b3`) |
|---|---|---|
| set id respected | all 12 commands ✓ | ✗ single-shot capture, VIDEO_START/STOP (→ 100), FOCUS and trigger commands (→ 0); ZOOM NACKed |
| unset → 100 / 0 | ✓ | ✗ FOCUS → 0 |
| pause stop → started camera | ✓ | ✗ always 100 |
| consecutive items all re-emitted | ✗ only the last 8 | ✗ only the last 8 |
| Tier 1 | all pass | ZOOM rejected as a mission item |

The consecutive-items failure is an older PX4 bug, not caused by the change.
PX4's COMMAND_LONG stream reads queued commands 8 at a time
(`VehicleCommand.ORB_QUEUE_LENGTH`), so a mission with more than 8 camera items
in a row loses the earlier commands.
