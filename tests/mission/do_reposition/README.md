# MAV_CMD_DO_REPOSITION (cmd=192) — Mission Protocol Conformance Tests

MAV_CMD_DO_REPOSITION is not supported on any of the main flight stacks or frame versions IN missions.

This directory contains Tier 1 (protocol acceptance) tests for `MAV_CMD_DO_REPOSITION` as a _mission item_ (MISSION_ITEM_INT upload).
See the root `CLAUDE.md` for the two-tier testing model.
See `tests/command/do_reposition/` for the COMMAND_INT (guided-mode) tests — that is the spec-correct execution surface for this command (see Finding, below).

## Finding (read this first)

`MAV_CMD_DO_REPOSITION` is **rejected outright as a mission item** (`MAV_MISSION_UNSUPPORTED` → MAVSDK `MissionRawResult.Result.UNSUPPORTED`) by **every stack and vehicle/frame type tested**: PX4 (multicopter, fixed-wing, VTOL), ArduCopter, ArduPlane fixed-wing, and QuadPlane.

This is **spec-aligned, not a bug**: the MAVLink spec for DO_REPOSITION says outright "This command is intended for guided commands (for missions use MAV_CMD_NAV_WAYPOINT instead)".
Both stacks' source code confirms the rejection is by design — see "Source verification" below.

Because the command cannot be uploaded into a mission on any tested combination:

- All 21 param-level Tier 1 tests are **skipped** — probing the parameters of an item the stack will never store has no diagnostic value.
- **No Tier 2 (execution) test exists or is possible** — there is no mission containing a DO_REPOSITION item to fly.
  See "Tier 2 — not applicable", below.

## Command parameters (MAVLink spec)

| #   | Label     | Type                                                 | Notes                                                                                                                                                                                                                                          |
| --- | --------- | ---------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1   | Speed     | float, m/s                                           | Ground speed; <0 (minValue=−1) = use default                                                                                                                                                                                                   |
| 2   | Bitmask   | float, `MAV_DO_REPOSITION_FLAGS`                     | bit 0 = CHANGE_MODE (1), bit 1 = RELATIVE_YAW (2)                                                                                                                                                                                              |
| 3   | Radius    | float, m                                             | Loiter radius (planes only); positive values only; 0/NaN = ignored                                                                                                                                                                             |
| 4   | Yaw       | float, **radians** (NOT degrees, unlike NAV_TAKEOFF) | NaN = use current heading; spec text adds "for planes indicates loiter direction (0: clockwise, 1: counter clockwise)" — ambiguous relative to the NaN/current-heading semantics, but not testable here since the command is rejected outright |
| 5   | Latitude  | int ×1e7                                             | INT32_MAX = use current position                                                                                                                                                                                                               |
| 6   | Longitude | int ×1e7                                             | INT32_MAX = use current position                                                                                                                                                                                                               |
| 7   | Altitude  | float, m                                             | Target altitude                                                                                                                                                                                                                                |

The spec marks `hasLocation="true" isDestination="true"` — the same classification as NAV_TAKEOFF.

## Test files

| File               | Tier   | Description                                                                                                                                                 |
| ------------------ | ------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `test_protocol.py` | Tier 1 | 22 tests: 1 baseline ("is the command accepted as a mission item at all?") + 21 param-probes that self-skip once the baseline shows the command is rejected |

No `test_flight.py` — see "Tier 2 — not applicable" below.

## Running

```bash
# Paired (mock) — all 22 tests pass; mock accepts every command
pytest tests/mission/do_reposition/ -v --log-cli-level=INFO

# PX4 SIH multicopter
pytest tests/mission/do_reposition/test_protocol.py \
    --drone-address=udp://:14540 --vehicle-type=quadcopter --autopilot=px4 \
    --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx -v --log-cli-level=INFO

# ArduCopter SITL
pytest tests/mission/do_reposition/test_protocol.py \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
    --ardupilot-sitl=~/ardu_sitl/arducopter --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 \
    --vehicle-type=copter --autopilot=ardupilot -v --log-cli-level=INFO
```

---

## Tier 1 Results — Protocol acceptance

Tested: 2026-06-07.

Logs:

- PX4 MC: `logs/mission_do_reposition_protocol_px4_quadcopter_20260607.log`
- PX4 FW: `logs/mission_do_reposition_protocol_px4_fixedwing_20260607.log`
- PX4 VTOL: `logs/mission_do_reposition_protocol_px4_vtol_20260607.log`
- ArduCopter: `logs/mission_do_reposition_protocol_arducopter_20260607.log`
- ArduPlane FW: `logs/mission_do_reposition_protocol_arduplane_fixedwing_20260607.log`
- QuadPlane: `logs/mission_do_reposition_protocol_quadplane_20260607.log`
- Mock: `logs/mission_do_reposition_protocol_mock_20260607.log`

### test_protocol_command_accepted (baseline)

Upload a baseline DO*REPOSITION item (`param1=-1` "use default speed", `param2=0` "no flags", `param3=0` "ignored", `param4=0.0`¹, distinct lat/lon, `z=30.0`); observe whether the upload is accepted or NACKed.
**Observational — passes either way**; the result \_is* the finding.

| PX4 MC                  | PX4 FW                  | PX4 VTOL                | ArduCopter              | ArduPlane FW            | QuadPlane               | Mock     |
| ----------------------- | ----------------------- | ----------------------- | ----------------------- | ----------------------- | ----------------------- | -------- |
| NACKed: **UNSUPPORTED** | NACKed: **UNSUPPORTED** | NACKed: **UNSUPPORTED** | NACKed: **UNSUPPORTED** | NACKed: **UNSUPPORTED** | NACKed: **UNSUPPORTED** | ACCEPTED |

¹ The baseline deliberately uses `param4=0.0`, **not** the spec-correct `NaN` ("use current heading") — see "Baseline param4 note" below for why this matters.

### Remaining 21 param-level tests (param1 Speed, param2 Bitmask, param3 Radius, param4 Yaw, params 5/6/7 Lat/Lon/Alt)

| PX4 (MC/FW/VTOL)   | ArduCopter         | ArduPlane FW / QuadPlane | Mock                                                               |
| ------------------ | ------------------ | ------------------------ | ------------------------------------------------------------------ |
| all 21 **SKIPPED** | all 21 **SKIPPED** | all 21 **SKIPPED**       | all 21 **PASS** (mock accepts every command and stores params raw) |

Skip reason (identical on every real stack/frame): _"DO_REPOSITION rejected outright as a mission item (UNSUPPORTED); param-level probing is moot — see test_protocol_command_accepted"_.
A class-scoped, cached `do_reposition_mission_support` fixture probes the baseline exactly once per stack run; an `autouse` skip-fixture (`_skip_unsupported_param_tests`) then skips every other test in the class with that single shared message — far clearer than 21 redundant "command rejected" failures.

### Baseline param4 note — an ArduCopter-specific pitfall this test avoids

The spec-correct sentinel for "use current heading" is `param4=NaN`.
An early version of this test's baseline used that value and observed ArduCopter returning a _different_ NACK reason — `MAV_MISSION_INVALID_PARAM4` (MAVSDK `INVALID_ARGUMENT`) — instead of PX4's `UNSUPPORTED`.
At first glance this looked like a genuine difference in how the two stacks handle DO_REPOSITION.

Source analysis of `AP_Mission::sanity_check_params()` (see "Source verification" below) showed otherwise: ArduPilot only permits NaN in the params of commands it explicitly special-cases (NAV*WAYPOINT, NAV_LOITER_UNLIM, NAV_LAND, NAV_TAKEOFF, NAV_ARC_WAYPOINT, NAV_VTOL_TAKEOFF, NAV_VTOL_LAND).
DO_REPOSITION is **absent** from that list, so its blanket `nan_mask = 0xff` rejects NaN in \_any* of params 1–4 — tripping `sanity_check_params()` and returning `MAV_MISSION_INVALID_PARAM4` _before_ the command-recognition switch (whose `default:` would otherwise report `UNSUPPORTED`, exactly like PX4) is ever reached.

Re-probing with `param4=0.0` confirmed the hypothesis: ArduCopter then also returns `UNSUPPORTED`, identical to PX4.
**The `INVALID_ARGUMENT` vs `UNSUPPORTED` discrepancy was an artifact of the probe's sentinel choice — not a real difference in command support.** The baseline now uses `param4=0.0` (matching the established NAV_TAKEOFF param2 workaround pattern) so the "is the command supported?" question is answered cleanly on both stacks.
The NaN-rejection behaviour itself is captured by the dedicated `test_protocol_param4_yaw_nan` test — which is, like every other param-level test, skipped once the baseline shows the command is unsupported.

---

## Source verification — does the observed MAV_RESULT match the source?

**Yes, on both stacks.** `UNSUPPORTED` is the _intended_, by-design result — not a bug, omission, or inconsistency between PX4 and ArduPilot.

**PX4** — `src/modules/mavlink/mavlink_mission.cpp`, the `mavlink_mission_item->command` switch (~line 1488) lists every mission command PX4 recognises (NAV*WAYPOINT, NAV_LOITER*_, NAV_LAND, NAV_TAKEOFF, NAV_LOITER_TO_ALT, NAV_ROI, DO_SET_ROI_, NAV_VTOL_TAKEOFF/LAND, CONDITION_GATE, fence/rally items, COMPONENT_ARM_DISARM, DO_AUTOTUNE_ENABLE, …).
`MAV_CMD_DO_REPOSITION` (192) is **absent**; execution falls through to `default: return MAV_MISSION_UNSUPPORTED;` (~line 1605).
The switch has no vehicle-type branching, so the result is identical across multicopter, fixed-wing, and VTOL — exactly as observed.

**ArduPilot** — `libraries/AP_Mission/AP_Mission.cpp`, `mavlink_int_to_mission_cmd()` first runs `sanity_check_params()` (the generic NaN/Inf check described in the baseline note above), then switches on `cmd.id` (~line 1064).
The switch lists NAV*WAYPOINT, NAV_LOITER*\*, NAV_LAND, NAV_TAKEOFF, NAV_LOITER_TO_ALT, NAV_ROI, DO_SET_ROI, NAV_VTOL_TAKEOFF/LAND, CONDITION_GATE, fence/rally points, COMPONENT_ARM_DISARM, DO_AUTOTUNE_ENABLE, … `MAV_CMD_DO_REPOSITION` (192) is **absent**; the `default:` case (~line 1472) returns `MAV_MISSION_UNSUPPORTED`.
This switch is shared by all ArduPilot vehicle types (Copter/Plane/QuadPlane), confirming the frame-independence observed.

A grep for `DO_REPOSITION` across `AP_Mission`/`GCS_MAVLink` finds exactly **one** other hit: `GCS_Common.cpp:5312`, inside `command_long_stores_location()` — which governs the **COMMAND_INT/COMMAND_LONG path** (direct guided-mode execution, tested in `tests/command/do_reposition/`), not the mission-item path.
This is the spec's "intended for guided commands" surface working exactly as designed: DO*REPOSITION is recognised by both stacks' \_command* handling and absent from both stacks' _mission-item_ handling.

---

## Tier 2 — not applicable

The task brief asked for Tier 2 to "construct a mission using the params that passed Tier 1 testing as probably supported, all in one mission, to verify that the params behave as expected".

**This is not possible for DO_REPOSITION, on any tested stack or frame type**: the baseline upload itself is rejected with `UNSUPPORTED` everywhere, so _zero_ params "passed" Tier 1 — there is no mission containing a DO_REPOSITION item that can even be uploaded, let alone flown.
No `test_flight.py` exists in this directory; writing one would have nothing to exercise.

This is the expected, spec-aligned outcome (see Finding, above): DO*REPOSITION is a \_guided* command, not a mission command.
Its actual execution semantics — does the vehicle reposition at the commanded speed/location/yaw, does the `CHANGE_MODE` flag switch to guided/hold mode, how do mode-dependent ACKs behave, etc. — are properly exercised via the **COMMAND_INT** path in `tests/command/do_reposition/` (`test_command.py` Tier 1, `test_flight.py` Tier 2), which is the spec-correct surface for this command.
See `tests/command/do_reposition/README.md`.

---

## Summary

|                               | PX4 (MC/FW/VTOL)                           | ArduCopter                                          | ArduPlane FW / QuadPlane     | Mock                        |
| ----------------------------- | ------------------------------------------ | --------------------------------------------------- | ---------------------------- | --------------------------- |
| Accepted as mission item?     | ✗ `UNSUPPORTED`                            | ✗ `UNSUPPORTED`                                     | ✗ `UNSUPPORTED`              | ✓ (mock accepts everything) |
| Matches source?               | ✓ absent from `mavlink_mission.cpp` switch | ✓ absent from `mavlink_int_to_mission_cmd()` switch | ✓ (shared ArduPilot switch)  | n/a                         |
| Frame/vehicle-type dependent? | no — identical MC/FW/VTOL                  | no                                                  | no — identical to ArduCopter | n/a                         |
| Tier 2 possible?              | no — nothing to upload/fly                 | no                                                  | no                           | n/a                         |

**Bottom line:** `MAV_CMD_DO_REPOSITION` is, by design and confirmed in source on both major flight-stack families, **not a mission-protocol command** — it exists exclusively as a guided-mode COMMAND_INT.
The MAVLink spec says so explicitly ("for missions use MAV_CMD_NAV_WAYPOINT instead"), and both PX4 and ArduPilot enforce that boundary identically and consistently across every vehicle type and frame tested.
See `tests/command/do_reposition/README.md` for the command-protocol (guided-mode) results, which is where this command's real behaviour is exercised.
