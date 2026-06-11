# DO_REPOSITION — mission protocol notes

See `README.md` for the full Tier 1 results table and source-verification writeup.

## One-line summary

`MAV_CMD_DO_REPOSITION` (192) is rejected with `MAV_MISSION_UNSUPPORTED` as a mission item by **every** stack and vehicle/frame type tested — PX4 (MC/FW/VTOL), ArduCopter, ArduPlane FW, QuadPlane.
Confirmed in source on both stacks: the command is simply absent from the mission-command recognition switch (`mavlink_mission.cpp` for PX4, `AP_Mission::mavlink_int_to_mission_cmd()` for ArduPilot — both fall through to a `default:` that returns `UNSUPPORTED`).
This is **spec-aligned**: the spec says "for missions use MAV_CMD_NAV_WAYPOINT instead".
No Tier 2 flight test exists or is possible — there is nothing to upload, so nothing to fly.
Real execution-semantics testing happens via COMMAND_INT in [[../../command/do_reposition/CLAUDE.md|tests/command/do_reposition]].

## Baseline-probe pitfall: param4=NaN masks the real finding on ArduCopter

The spec-correct sentinel for param4 (Yaw) is `NaN` ("use current heading").
Using it in the baseline probe makes ArduCopter return `MAV_MISSION_INVALID_PARAM4` (→ MAVSDK `INVALID_ARGUMENT`) instead of `UNSUPPORTED` — looking like a stack-specific difference in DO_REPOSITION support.
It isn't: ArduPilot's `sanity_check_params()` only allows NaN in the params of commands it special-cases (NAV_WAYPOINT, NAV_TAKEOFF, …); DO_REPOSITION isn't one of them, so its blanket `nan_mask = 0xff` rejects NaN in *any* of params 1–4 *before* the command-recognition switch is ever reached.
**Lesson — generalising the existing [[../nav_takeoff/CLAUDE.md|nav_takeoff]] note**: when constructing a baseline "is this command supported at all?" probe for ArduPilot, every float param must be a concrete non-NaN value (`0.0` is the safe default), *regardless* of what the spec says the sentinel "should" be — otherwise a generic param-sanity rejection can masquerade as a command-support rejection and produce a misleading `INVALID_ARGUMENT` instead of the true `UNSUPPORTED`.
Once `param4=0.0` is used, ArduCopter's result matches PX4's exactly.

## Why no per-frame tables

PX4's mission-command switch and ArduPilot's `mavlink_int_to_mission_cmd()` switch both have **zero vehicle-type branching** — the `UNSUPPORTED` result is identical across every frame type within each firmware family (verified empirically: PX4 MC/FW/VTOL and ArduCopter/ArduPlane FW/QuadPlane all produce byte-identical NACK reasons).
A per-frame comparison table would just repeat "UNSUPPORTED" seven times; the single summary table in the README is the honest representation of the finding.

## Log references

- PX4 MC: `logs/mission_do_reposition_protocol_px4_quadcopter_20260607.log`
- PX4 FW: `logs/mission_do_reposition_protocol_px4_fixedwing_20260607.log`
- PX4 VTOL: `logs/mission_do_reposition_protocol_px4_vtol_20260607.log`
- ArduCopter: `logs/mission_do_reposition_protocol_arducopter_20260607.log`
- ArduPlane FW: `logs/mission_do_reposition_protocol_arduplane_fixedwing_20260607.log`
- QuadPlane: `logs/mission_do_reposition_protocol_quadplane_20260607.log`
- Mock: `logs/mission_do_reposition_protocol_mock_20260607.log`
