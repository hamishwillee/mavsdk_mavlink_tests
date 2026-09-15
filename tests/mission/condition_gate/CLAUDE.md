# CONDITION_GATE — mission protocol notes

See `README.md` for the full Tier 1/Tier 2 results and the mavlink-devguide PR #761 cross-check table.

## One-line summary

`MAV_CMD_CONDITION_GATE` (4501) is tagged `<wip/>` in `common.xml`, which makes MAVSDK's `mission_raw` plugin reject it client-side before anything reaches the wire — this test suite is the first in the repo to need the raw `mavlink_direct` transport (`raw_upload_mission_items()`/`raw_download_mission_items()` in `tests/mission/conftest.py`).
PX4 accepts it as a mission item (since v1.11) but never stores Geometry/UseAltitude (params 1/2) — a structural drop confirmed in source, not just an execution-time "ignored" quirk.
ArduPilot does not implement it at all (absent from `AP_Mission`'s command switch).
A full Tier 2 flight test confirms the mavlink-devguide PR #761 behavioural claims directly from telemetry: the vehicle flies a straight line between the flanking waypoints regardless of the gate's off-path position, and a `DO_CHANGE_SPEED` placed after the gate only executes once the vehicle crosses the gate's projected line — not at either waypoint.

## Why this command needed new shared infrastructure

MAVSDK's `mavsdk_server` (the process behind `mission_raw`) keeps its own internal table of commands it understands, separate from what the flight stack actually supports.
`<wip/>`-tagged commands like this one aren't in that table, so `mission_raw.upload_mission()` raises `INVALID_ARGUMENT` immediately — the item never reaches the drone, real or mock.
This was discovered by cross-referencing the sibling manual-verification repo `mavsdk_qgc_server_tests/condition_gate_tests/`, which uses raw `pymavlink` specifically to route around the same restriction.

`tests/mission/conftest.py` now has `raw_upload_mission_items()` / `raw_download_mission_items()`: a direct `MISSION_COUNT`/`MISSION_REQUEST_INT`/`MISSION_ITEM_INT`/`MISSION_ACK` handshake over `mavlink_direct`, selected per-command via `MissionItemSpec(transport="raw")`.
This is shared infrastructure, not specific to this command — any future `<wip/>` or MAVSDK-blocklisted command can use the same path.

**Bug found and fixed while building this**: the first version of `raw_upload_mission_items()` only listened for `MISSION_REQUEST_INT`.
Against real PX4 this was fine, but ArduCopter requests items using the *deprecated* `MISSION_REQUEST` (no int32 x/y) instead — confirmed by capturing the wire traffic directly (`MISSION_REQUEST` retried 8 times, then ArduCopter gave up with `MAV_MISSION_OPERATION_CANCELLED` + a `STATUSTEXT` "Mission upload timeout").
The spec requires a modern GCS to respond to either request form with `MISSION_ITEM_INT` (see the "Deprecated message handling" section of this file's parent `CLAUDE.md` — MAVSDK's own `mission_raw` plugin already does this transparently, which is why the bug wasn't visible via that path). Fixed by serving both message types identically.

## Source verification (PX4)

`~/github/PX4/PX4-Autopilot`, checked against the actual behaviour of the SITL binary (not just static reading — the binary had to be **rebuilt** during this work; see below).

- **Validation mask**: `src/modules/mavlink/mavlink_command_params.hpp`: `{ 4501, 0x73, 0x00 }` — mission mask `0x73` = `0b01110011` permits real values in params 1, 2, 5, 6, 7; params 3/4 must be unset (NaN or 0.0). Command mask `0x00` — CONDITION_GATE is mission-item-only, never usable as a direct COMMAND_INT/LONG.
- **This mask was patched during this same effort, by an earlier session, but the SITL binary hadn't been rebuilt yet.** Commit `2218cecc63` ("fix(mavlink): allow CONDITION_GATE's location params in mission upload", 2026-09-10, co-authored by Claude) changed the mask from `0x00` to `0x73` — matching exactly what the sibling `condition_gate_tests` repo's README already documented as the required one-line fix. Before rebuilding (`make px4_sitl_default`), every upload — even a lone gate item with no other mission items — was rejected with `MAV_MISSION_INVALID_PARAM5_X`, because the stale binary still had the old `0x00` mask (every param required to be unset). After the rebuild, the fix took effect exactly as the commit describes. **Lesson for future sessions**: if a mission-protocol test result doesn't match what the PX4 source appears to say, check `git log` on the relevant file against the SITL binary's build timestamp before assuming the source reading is wrong.
- **Params 1/2 (Geometry/UseAltitude) never survive storage**: `src/modules/mavlink/mavlink_mission.cpp`'s upload-parse switch (`case MAV_CMD_CONDITION_GATE:`) only sets `mission_item->nav_cmd`, never copying `param1`/`param2` into `mission_item->params[]`; the download-format switch has no case for this `nav_cmd` at all, silently leaving both fields at their zero-initialised default on every download. Confirmed empirically: uploading `param1=7.0, param2=1.0` (distinguishable, non-default values) downloads as `0.0, 0.0` every time — see `test_params_1_2_zeroed_on_roundtrip_px4`. This is the storage-level mechanism consistent with mavlink-devguide PR #761's "UseAltitude field ignored" claim — whether the value is dropped on storage or read-and-discarded at execution is not observable from a GCS, and either way it's not documentation-worthy as a discrepancy: the param simply isn't used, which is exactly what "ignored" already says.
- **Execution**: `src/modules/navigator/mission_block.cpp` — `item_contains_gate()` (`item.nav_cmd == NAV_CMD_CONDITION_GATE`) flags the item; the actual crossing check is a horizontal-only dot-product/line-crossing test against `_mission_item.lat/lon` and the *next* waypoint — it never reads `params[0]`/`params[1]`, matching the storage finding (the params aren't just lost in serialisation, they're not consumed by execution either, even if they somehow were preserved).
- **Feasibility check**: `src/modules/navigator/MissionFeasibility/FeasibilityChecker.cpp::checkDistancesBetweenWaypoints()` rejects a gate within 0.05 m of an adjacent waypoint ("Distance between waypoint and gate too close") — but only checks *immediately adjacent* items in upload order, not general path geometry (confirmed: `test_gate_coincident_with_waypoint_rejected` uses 0 m separation and gets a real NACK on PX4; the mock, with no feasibility logic, accepts it).

## Source verification (ArduPilot)

Absent from `AP_Mission::mavlink_int_to_mission_cmd()`'s command switch in `libraries/AP_Mission/AP_Mission.cpp` — falls through to `default:` → `MAV_MISSION_UNSUPPORTED`, exactly the same "recognised by neither NAV_* switch" pattern as [[../do_reposition/CLAUDE.md|DO_REPOSITION]] (a different command, unrelated reason: DO_REPOSITION is guided-only by spec; CONDITION_GATE simply has zero ArduPilot implementation — tracked upstream at [ardupilot#13778](https://github.com/ArduPilot/ardupilot/issues/13778)).

**Baseline-probe NaN pitfall — same family as [[../do_reposition/CLAUDE.md|do_reposition]] and [[../nav_takeoff/CLAUDE.md|nav_takeoff]]**: the spec-correct sentinel for params 3/4 ("Empty") is NaN, but ArduPilot's blanket `sanity_check_params()` only allows NaN for commands it explicitly recognises. CONDITION_GATE isn't recognised at all, so a NaN baseline is rejected with `MAV_MISSION_INVALID_PARAM3` *before* the command-switch's `default:` is ever reached — masking the real "unsupported" finding behind a misleading "invalid argument". `SPEC.baseline` uses `param3=0.0, param4=0.0` for exactly this reason (see the comment at its definition in `test_protocol.py`); confirmed empirically both ways (NaN → `INVALID_PARAM3`, 0.0 → `UNSUPPORTED`).

## Tier 2 — flight test design notes

`test_flight.py` builds a 6-item mission (takeoff → wp1 → gate [25 m off-path] → `DO_CHANGE_SPEED` → wp2 → RTL), uploaded via the raw transport, started via a raw `MAV_CMD_MISSION_START` (300) COMMAND_LONG (bypassing `mission_raw.start_mission()`, whose internal state tracking is untested against items uploaded through a different transport).
Telemetry evidence comes from sampling `GLOBAL_POSITION_INT` directly (lat/lon + vx/vy for groundspeed) rather than MAVSDK's higher-level telemetry streams, since both position and instantaneous groundspeed are needed together at a known rate.

Two geometric assertions directly test mavlink-devguide PR #761's specific prose:
1. **Not a destination**: max cross-track deviation from the direct wp1→wp2 line must stay well under the gate's 25 m offset, and the closest approach to the gate's own coordinates must not be meaningfully closer than that offset (i.e. the route never bends toward it).
2. **Blocking/trigger point**: the `DO_CHANGE_SPEED` speed drop must occur measurably after wp1 and before wp2, close to the gate's perpendicular projection onto the leg (generous tolerance — PX4 needs some distance to decelerate to the new setpoint after the trigger fires, so the *detected* drop point lags the true crossing point).

UseAltitude (param2) gets a Tier 2 test (`test_use_altitude_gates_on_altitude_if_supported`) that **always flies** (gate 80 m above cruise altitude, UseAltitude=1, observe whether `DO_CHANGE_SPEED` still fires) — per root `CLAUDE.md`'s Tier 2 design pattern #1, only an actual upload NACK is a legitimate skip basis. An inline probe still uploads a solo gate item with `param2=1.0` first and logs whether it survives the round trip, but that result is context, not a gate: Tier 1's `test_params_1_2_zeroed_on_roundtrip_px4` shows it currently never does, yet the flight always runs anyway and logs whichever outcome is observed (fired anyway / never fired) as a real finding either way — "not stored" is an inference about *why*, not proof the value can't affect execution through some other path this specific round-trip probe doesn't see. The command-level flight test's NACK-based skip (whole command `UNSUPPORTED` on ArduCopter, confirmed skipping in ~27 s without ever arming) remains a legitimate skip — that's an explicit protocol rejection, not an inference.

**Timing budget** (root `CLAUDE.md` § 2/3/4 of the same convention): the sampling window is derived from the measured `MPC_XY_CRUISE` param plus one flat, environment-overridable allowance for climb/transit (`CONDITION_GATE_CLIMB_ALLOWANCE_S`, default 25 s) rather than a single hand-tuned total-duration guess — a stack with a different configured cruise speed gets proportionally more or less time automatically. If a future run (e.g. against a different stack, should one ever support this command) exhausts the budget before reaching the relevant part of the mission, the relevant assertions fail with a distinct "LIKELY INSUFFICIENT SAMPLING TIME FOR THIS STACK" prefix (`_timing_caveat()`) rather than reporting it as a behavioural difference.

Five runs (`--px4-sitl` freshly rebuilt after the mask fix, `sihsim_quadx`, PX4 v1.18.0-beta): max cross-track deviation 0.38–0.45 m (gate offset 25.0 m), closest approach to gate 24.97–25.01 m, speed drop detected at 116.2–116.5 m along the leg (gate projects to 120.0 m), trigger always detected by t=48.6–48.7s of a ~105s budget (roughly half the budget unused every time) — consistent to within ~1% across every run, no flakiness observed.

## Log references

- PX4 MC Tier 1: `logs/mission_condition_gate_tier1_px4_quadcopter_1.18.0-beta_20260913_123542.log` (pre-rebuild run showing the mask bug is `logs/mission_condition_gate_tier1_px4_quadcopter_1.18.0-beta_20260913_122000.log`)
- ArduCopter Tier 1: `logs/mission_condition_gate_tier1_ardupilot_copter_4.8.0-dev_20260913_124723.log`
- Mock Tier 1: `logs/mission_condition_gate_tier1_mock_mock_20260913_121721.log`
- PX4 MC Tier 2 (latest of 5 runs): `logs/mission_condition_gate_flight_px4_quadcopter_1.18.0-beta_20260913_140023.log`
- ArduCopter Tier 2 skip confirmation: `logs/mission_condition_gate_flight_ardupilot_copter_4.8.0-dev_20260913_140325.log`
