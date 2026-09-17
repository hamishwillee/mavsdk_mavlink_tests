# DO_SET_ACTUATOR — mission protocol notes

Built on `Tier1MissionTestBase`/`MissionItemSpec` (see `tests/mission/CLAUDE.md` §
"Shared Tier 1 infrastructure"). Bespoke tests follow root `CLAUDE.md` rule 9's
naming convention (`test_do_set_actuator_<rest>`, not `test_protocol_<rest>` —
this is a per-command test file, not generic protocol mechanics).

## Why this directory exists

Added 2026-09-17 specifically to verify [PX4-Autopilot PR #28723](https://github.com/PX4/PX4-Autopilot/pull/28723)
(commits `0f029991b2` "fix COMMAND_INT actuator scaling" and `ea5734b460`
"fix MISSION_ITEM_INT actuator scaling"), checked out locally at
`~/github/PX4/PX4-Autopilot` (branch `onelittlechildawa/fix-command-int-actuator-scaling`).

**The bug**: DO_SET_ACTUATOR's param5/param6 map to MISSION_ITEM_INT's `x`/`y`
— genuinely int32 wire fields, spec-scaled by 1e7 regardless of frame (same
numeric convention as a location's lat/lon, for an unrelated semantic).
Before the fix, `mavlink_mission.cpp`'s upload parser stored the *raw,
unscaled* integer as the internal actuator value, and the download
formatter had no case for this command's x/y mapping at all — the
round-trip was broken, not merely mis-scaled ("falling through to an
error" per the fix commit's own message).

## Stale-binary pitfall hit while verifying this (root CLAUDE.md design decision 4c)

First verification attempt (binary built 08:40, before `ea5734b460` landed
at 12:31 the same day) produced 3 real-looking failures: a mission-Tier2
test and two command-Tier2 sentinel-independence/frame tests. Root-caused
via `git log -1 -- <file>` vs. binary build timestamp — the running binary
still had the OLD joint sentinel check
(`if (x == INT32_MAX && y == INT32_MAX)`) from before `ea5734b460`
refactored it into the independent-per-field `decode_scaled_int32_field()`
helper. A clean rebuild (`make px4_sitl_default`, after also running
`git submodule update --init --recursive` on `src/drivers/gps/devices` and
`src/modules/mavlink/mavlink` — both were separately out of sync and broke
the build with unrelated GPS-driver compile errors) resolved all 3
failures completely. **Always verify the binary's build timestamp against
the fix commit's timestamp before treating a Tier 2 failure as a real
finding** — this is not a one-off; it has now recurred multiple times this
session with different commands.

## Real, PR-unrelated finding: MAV_FRAME_MISSION required

See `README.md` § "Key finding: MAV_FRAME_MISSION required". First test
draft used `frame=5` (MAV_FRAME_GLOBAL_INT, the convention every other
mission command in this repo uses) and got `MAV_MISSION_UNSUPPORTED` on
every upload — root-caused via `mavlink_mission.cpp`'s
`parse_mavlink_mission_item()`: DO_SET_ACTUATOR is a non-location `DO_*`
action command, only listed as a valid case under the `MAV_FRAME_MISSION`
branch of the frame switch, not the location-command branch frame=5 falls
into. Fixed by setting `SPEC.frame=2` and rewriting the originally-planned
"frame independence" bespoke test (which assumed multiple valid frames to
compare, mirroring the command-protocol side) into
`test_do_set_actuator_requires_mission_frame`, a characterisation test of
this real, pre-existing PX4 convention instead — there is no second valid
frame for a mission-item DO_SET_ACTUATOR to test scaling-frame-independence
against; that comparison is only meaningful (and only tested) on the
command-protocol side, where COMMAND_INT genuinely does have multiple valid
frames (see `tests/command/do_set_actuator/test_flight.py`'s frame=1
LOCAL_NED test — the exact frame the pre-fix COMMAND_INT bug mis-scaled).

## Results

See `README.md` § "Results" — all tests pass against a correctly-rebuilt
PX4 MC binary with both PR #28723 commits present.
