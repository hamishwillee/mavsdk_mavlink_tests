# MAV_CMD_DO_SET_ACTUATOR (cmd=187) — mission protocol

Built specifically to verify [PX4-Autopilot PR #28723](https://github.com/PX4/PX4-Autopilot/pull/28723)
("fix(mavlink): fix COMMAND_INT/MISSION_ITEM_INT actuator scaling") — see
`CLAUDE.md` in this directory and `tests/command/do_set_actuator/README.md`
for the full bug/fix writeup.

## Command parameters (common.xml)

| Param | Label | Range | Meaning |
|---|---|---|---|
| param1 | Actuator 1 | -1..1, NaN=ignore | Defined |
| param2 | Actuator 2 | -1..1, NaN=ignore | Defined |
| param3 | Actuator 3 | -1..1, NaN=ignore | Defined |
| param4 | Actuator 4 | -1..1, NaN=ignore | Defined |
| param5 | Actuator 5 | x field — ×1e7 in MISSION_ITEM_INT, INT32_MAX=ignore | Defined |
| param6 | Actuator 6 | y field — ×1e7 in MISSION_ITEM_INT, INT32_MAX=ignore | Defined |
| param7 | Index | z field, integer-valued float, minValue=0 | Defined |

## Test files

| File | Tier | Coverage |
|---|---|---|
| `test_protocol.py` | 1 (protocol) | Baseline/sentinel checks (`Tier1MissionTestBase`) + bespoke actuator-scaling round-trip tests |
| `test_flight.py` | 2 (execution) | Full mission-execution verification via PWM output observation |

## Running

```
# Mock
pytest tests/mission/do_set_actuator/ -v --log-cli-level=INFO

# PX4 MC
pytest tests/mission/do_set_actuator/ \
  --drone-address=udp://:14540 --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \
  --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO
```

## Key finding: MAV_FRAME_MISSION required

DO_SET_ACTUATOR is only recognised as a valid mission-item command under
**MAV_FRAME_MISSION (frame=2)** — confirmed by source
(`mavlink_mission.cpp`'s `parse_mavlink_mission_item()`) and empirically,
via both a targeted regression test (`test_do_set_actuator_requires_mission_frame`,
frame=5 specifically) and the generic, exhaustive
`test_frame_validation_survey` (inherited from `Tier1MissionTestBase`,
sweeps all 22 `MAV_FRAME_CATALOGUE` values). Confirmed against real PX4 MC:
**exactly** frame=2 is `ACCEPTED`; all other 21 frames return `UNSUPPORTED`
(no other frame partially works). This matches the existing documented
behaviour of other non-location `DO_*` action commands (e.g.
`DO_CHANGE_SPEED` — see `tests/mission/CLAUDE.md`'s PX4 behaviour notes)
and is **not** related to PR #28723 — it's a pre-existing PX4 convention.
`SPEC.frame` in `test_protocol.py` defaults to 2 for this reason.

Note: `test_frame_validation_survey`'s outcome column only ever reads
`ACCEPTED`/`UNSUPPORTED` here, never a frame-specific NACK reason —
MAVSDK's `mission_raw` plugin collapses `MAV_MISSION_UNSUPPORTED_FRAME`
and `MAV_MISSION_UNSUPPORTED` into one client-side `Result.UNSUPPORTED`
(see `tests/mission/CLAUDE.md`), so this survey can't distinguish "frame
itself invalid" from "command not recognised under this frame" the way
the command-protocol equivalent can.

## Results (PX4 MC, 1.18.0-beta, PR #28723 branch, rebuilt 2026-09-17)

All tests **PASS** once built from a binary that actually includes both PR
commits (see `CLAUDE.md`'s stale-binary note — the first verification
attempt against a not-yet-rebuilt binary produced 3 real failures that
disappeared entirely after a clean rebuild):

- `test_do_set_actuator_actuator5_scales_by_1e7` / `_actuator6_scales_by_1e7` — round-trip PASS.
- `test_do_set_actuator_sentinel_independent_per_field` — PASS (x=INT32_MAX ignored independently of y).
- `test_do_set_actuator_index_preserved` — PASS.
- `test_do_set_actuator_requires_mission_frame` — characterisation, confirms frame=5 is rejected (see above).
- `test_actuator_compat_mission_item_scales_by_1e7` (Tier 2) — PASS: a single-item mission
  (DO_SET_ACTUATOR, x=5,000,000) executes and the vehicle's actuator output
  reaches 1750us (the correct PWM for actuator value 0.5), confirming the
  full upload → mission-execution → vehicle_command dispatch → actuator
  output pipeline, not just protocol-level storage.
