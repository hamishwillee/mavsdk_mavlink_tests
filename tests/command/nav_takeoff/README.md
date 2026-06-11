# MAV_CMD_NAV_TAKEOFF — Command Protocol (COMMAND_INT) Test Results

Tests the **command protocol** path (COMMAND_INT → COMMAND_ACK).

This is distinct from the **mission protocol** path (MISSION_ITEM_INT upload → storage) covered by `tests/mission/nav_takeoff/`.

## COMMAND_INT vs COMMAND_LONG

NAV_TAKEOFF has `hasLocation="true"` and `isDestination="true"` in common.xml.
Per the MAVLink spec, commands with location fields must use COMMAND_INT — the integer x/y fields preserve lat/lon precision that is lost in COMMAND_LONG (float param5/6).

## NaN in float fields

Pass `None` (Python) in `fields_json` to encode NaN on the wire.
`json.dumps(None)` produces `"null"`; nlohmann/json (MAVSDK C++ gRPC bridge) decodes a JSON `null` in a float field as IEEE-754 NaN.
The MAVLink spec permits NaN for unused params and some defined params (e.g. param4=NaN means "use current heading").

NaN tests skip in mock/paired mode (the mock's ACCEPTED result does not reflect real stack behaviour for param handling).

## Lat/lon sentinel values and "use current position"

Coordinate fields in NAV_TAKEOFF are **not** always meaningful lat/lon values — they may carry sentinels:

| Sentinel | Message type | Meaning | PX4 behaviour |
|----------|-------------|---------|----------------|
| `x=INT32_MAX, y=INT32_MAX` | COMMAND_INT | "use current position" | Converted to NaN in `vcmd.param5/6` by MavlinkReceiver (line 611–614); navigator falls through to current position |
| `param5=NaN, param6=NaN` | COMMAND_LONG | "use current position" | Passed through to navigator; `PX4_ISFINITE(NaN)=false` → falls through to current position |
| `param5≈INT32_MAX, param6≈INT32_MAX` | COMMAND_LONG | **Protocol error** | PX4 explicitly DENIES (MavlinkReceiver:499–505): "This looks suspiciously like INT32_MAX was sent in a COMMAND_LONG instead of a COMMAND_INT" |
| `param7=NaN` (z/altitude) | Both | "use system default" | Navigator: `PX4_ISFINITE(NaN)=false` → uses `current_alt + MIS_TAKEOFF_ALT` parameter |

Key point: **do not conflate "has lat/lon fields" with "all values are valid coordinates"**.
Sentinel values have specific semantic meaning that stacks must honour.
PX4 correctly handles INT32_MAX→NaN conversion for COMMAND_INT and NaN passthrough for COMMAND_LONG.
Sending INT32_MAX as a float in COMMAND_LONG is a protocol error (correct message type for the "use current position" sentinel is COMMAND_INT with x=y=INT32_MAX).

## Yaw behaviour via COMMAND_INT

Both PX4 and ArduPilot **ignore** param4 (Yaw) in the COMMAND_INT execution path:

| Stack | Source | Behaviour |
|-------|--------|-----------|
| PX4 | `navigator_main.cpp:630`: `rep->current.yaw = NAN` | Yaw reset regardless of param4 |
| ArduCopter | `GCS_MAVLink_Copter.cpp:585`: `// param4 : yaw angle   (not supported)` | Yaw ignored |
| ArduPlane | `GCS_MAVLink_Plane.cpp`: only altitude read | Yaw ignored |

This differs from the **mission protocol** path, where PX4 stores and uses param4 yaw.

## Test Results

### ArduCopter MC (standalone)

8 PASS, 2 XFAIL, 0 SKIP.
NAV_TAKEOFF is SUPPORTED on ArduCopter via COMMAND_INT.

| Test | Param | Result |
|------|-------|--------|
| `test_command_accepted` | baseline | PASS — result=0 ACCEPTED |
| `test_param1_pitch_ack_denied` | param1=15° | **XFAIL** — result=0 ACCEPTED; ArduCopter ignores pitch in COMMAND_INT path (spec violation) |
| `test_param1_nan_ack_result` | param1=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_param4_yaw_specific_ack` | param4=90° | PASS — result=0 ACCEPTED (yaw ignored) |
| `test_param4_yaw_nan_ack` | param4=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_location_specific_ack` | x/y=home | PASS — result=0 ACCEPTED |
| `test_location_int32max_ack` | x/y=INT32_MAX | PASS — result=0 ACCEPTED (observational) |
| `test_location_out_of_range_latlon_ack` | x=120°N, y=200°E | PASS — result=2 DENIED (observational) |
| `test_nan_altitude_ack` | z=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_wrong_frame_ack` | frame=LOCAL_NED | PASS — result=0 ACCEPTED (observational) |

### ArduPlane FW (standalone)

8 PASS, 2 XFAIL, 0 SKIP.
NAV_TAKEOFF is SUPPORTED on ArduPlane FW via COMMAND_INT.
ArduPlane only reads altitude; lat/lon and other params are ignored in execution, but out-of-range coordinates are still rejected at the command-handler level.

| Test | Param | Result |
|------|-------|--------|
| `test_command_accepted` | baseline | PASS — result=0 ACCEPTED |
| `test_param1_pitch_ack_denied` | param1=15° | **XFAIL** — result=0 ACCEPTED; ArduPlane ignores pitch in COMMAND_INT path (spec violation) |
| `test_param1_nan_ack_result` | param1=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_param4_yaw_specific_ack` | param4=90° | PASS — result=0 ACCEPTED (yaw ignored) |
| `test_param4_yaw_nan_ack` | param4=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_location_specific_ack` | x/y=home | PASS — result=0 ACCEPTED |
| `test_location_int32max_ack` | x/y=INT32_MAX | PASS — result=0 ACCEPTED (observational) |
| `test_location_out_of_range_latlon_ack` | x=120°N, y=200°E | PASS — result=2 DENIED (observational) |
| `test_nan_altitude_ack` | z=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_wrong_frame_ack` | frame=LOCAL_NED | PASS — result=0 ACCEPTED (observational) |

### ArduPlane QP (standalone)

8 PASS, 2 XFAIL, 0 SKIP.
Same behaviour as ArduPlane FW (not re-tested; expected identical).

| Test | Param | Result |
|------|-------|--------|
| `test_command_accepted` | baseline | PASS — result=0 ACCEPTED |
| `test_param1_pitch_ack_denied` | param1=15° | expected **XFAIL** — same as ArduPlane FW |
| `test_param1_nan_ack_result` | param1=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_param4_yaw_specific_ack` | param4=90° | PASS — result=0 ACCEPTED (yaw ignored) |
| `test_param4_yaw_nan_ack` | param4=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_location_specific_ack` | x/y=home | PASS — result=0 ACCEPTED |
| `test_location_int32max_ack` | x/y=INT32_MAX | PASS — result=0 ACCEPTED (observational) |
| `test_location_out_of_range_latlon_ack` | x=120°N, y=200°E | expected result=2 DENIED (same as ArduPlane FW) |
| `test_nan_altitude_ack` | z=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_wrong_frame_ack` | frame=LOCAL_NED | PASS — result=0 ACCEPTED (observational) |

### ArduRover (standalone)

NAV_TAKEOFF is **UNSUPPORTED** on ArduRover (ground vehicle) — survey-gating skips all detail tests.
Baseline probe returns `MAV_RESULT_UNSUPPORTED (3)`; `_ensure_supported()` caches this and calls `pytest.skip()` for every test in the class.

10 SKIP — tests not run.

### PX4 MC (standalone)

9 PASS, 3 XFAIL, 0 SKIP.
NAV_TAKEOFF is SUPPORTED on PX4 multicopter via COMMAND_INT.

| Test | Param | Result |
|------|-------|--------|
| `test_command_accepted` | baseline | PASS — result=0 ACCEPTED |
| `test_param1_pitch_ack_denied` | param1=15° | **XFAIL** — result=0 ACCEPTED; PX4 ignores pitch in COMMAND_INT path (spec violation) |
| `test_param1_nan_ack_result` | param1=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_param4_yaw_ack_denied` | param4=90° | **XFAIL** — result=0 ACCEPTED; yaw ignored in COMMAND_INT path (spec violation) |
| `test_param4_yaw_nan_ack` | param4=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_location_specific_ack` | x/y=home | PASS — result=0 ACCEPTED |
| `test_location_int32max_ack` | x/y=INT32_MAX | PASS — result=0 ACCEPTED (observational) |
| `test_location_out_of_range_latlon_ack` | x=120°N, y=200°E | **XFAIL** — result=0 ACCEPTED; PX4 does not validate lat/lon range (spec gap) |
| `test_nan_altitude_ack` | z=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_wrong_frame_ack` | frame=LOCAL_NED | PASS — result=0 ACCEPTED (observational) |
| `test_latlon_nan_command_long_ack` | COMMAND_LONG param5/6=NaN | PASS — result=0 ACCEPTED — PX4 navigator uses current position when `PX4_ISFINITE(param5)=false` |
| `test_latlon_int32max_command_long_denied` | COMMAND_LONG param5/6=INT32_MAX (float) | PASS — result=2 DENIED — PX4 MavlinkReceiver explicitly detects and rejects this (mavlink_receiver.cpp:499) |

### PX4 FW (standalone)

9 PASS, 3 XFAIL, 0 SKIP.
NAV_TAKEOFF is SUPPORTED on PX4 fixed-wing via COMMAND_INT.
All results expected identical to PX4 MC (same navigator handler, same MavlinkReceiver).

| Test | Param | Result |
|------|-------|--------|
| `test_command_accepted` | baseline | PASS — result=0 ACCEPTED |
| `test_param1_pitch_ack_denied` | param1=15° | **XFAIL** — result=0 ACCEPTED; PX4 ignores pitch in COMMAND_INT path (spec violation) |
| `test_param1_nan_ack_result` | param1=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_param4_yaw_ack_denied` | param4=90° | **XFAIL** — result=0 ACCEPTED; yaw ignored in COMMAND_INT path (spec violation) |
| `test_param4_yaw_nan_ack` | param4=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_location_specific_ack` | x/y=home | PASS — result=0 ACCEPTED |
| `test_location_int32max_ack` | x/y=INT32_MAX | PASS — result=0 ACCEPTED (observational) |
| `test_location_out_of_range_latlon_ack` | x=120°N, y=200°E | **XFAIL** — result=0 ACCEPTED; PX4 does not validate lat/lon range (spec gap) |
| `test_nan_altitude_ack` | z=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_wrong_frame_ack` | frame=LOCAL_NED | PASS — result=0 ACCEPTED (observational) |

### PX4 VTOL (standalone)

7 PASS, 3 XFAIL, 0 SKIP.
NAV_TAKEOFF is SUPPORTED on PX4 VTOL via COMMAND_INT.
All results expected identical to PX4 MC (not re-tested for this addition).

| Test | Param | Result |
|------|-------|--------|
| `test_command_accepted` | baseline | PASS — result=0 ACCEPTED |
| `test_param1_pitch_ack_denied` | param1=15° | expected **XFAIL** — same as PX4 MC |
| `test_param1_nan_ack_result` | param1=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_param4_yaw_ack_denied` | param4=90° | **XFAIL** — result=0 ACCEPTED; yaw ignored in COMMAND_INT path (spec violation) |
| `test_param4_yaw_nan_ack` | param4=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_location_specific_ack` | x/y=home | PASS — result=0 ACCEPTED |
| `test_location_int32max_ack` | x/y=INT32_MAX | PASS — result=0 ACCEPTED (observational) |
| `test_location_out_of_range_latlon_ack` | x=120°N, y=200°E | expected **XFAIL** (same as PX4 MC/FW) |
| `test_nan_altitude_ack` | z=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_wrong_frame_ack` | frame=LOCAL_NED | PASS — result=0 ACCEPTED (observational) |

### PX4 Rover (standalone)

7 PASS, 3 XFAIL, 0 SKIP.
PX4 Rover returns ACCEPTED for NAV_TAKEOFF — PX4 does not restrict commands by vehicle type (unlike ArduRover which returns UNSUPPORTED).

| Test | Param | Result |
|------|-------|--------|
| `test_command_accepted` | baseline | PASS — result=0 ACCEPTED |
| `test_param1_pitch_ack_denied` | param1=15° | expected **XFAIL** — same as PX4 MC |
| `test_param1_nan_ack_result` | param1=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_param4_yaw_ack_denied` | param4=90° | **XFAIL** — result=0 ACCEPTED; yaw ignored in COMMAND_INT path (spec violation) |
| `test_param4_yaw_nan_ack` | param4=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_location_specific_ack` | x/y=home | PASS — result=0 ACCEPTED |
| `test_location_int32max_ack` | x/y=INT32_MAX | PASS — result=0 ACCEPTED (observational) |
| `test_location_out_of_range_latlon_ack` | x=120°N, y=200°E | expected **XFAIL** (same as PX4 MC/FW) |
| `test_nan_altitude_ack` | z=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_wrong_frame_ack` | frame=LOCAL_NED | PASS — result=0 ACCEPTED (observational) |

Tested: 2026-05-27 (original 9 tests); 2026-05-31 (`test_location_out_of_range_latlon_ack` added).

### Mock (paired mode)

| Test | Param | Mock result |
|------|-------|-------------|
| `test_command_accepted` | baseline (param4=NaN) | PASS — result=0 ACCEPTED |
| `test_param1_pitch_ack_denied` | param1=15° | **XFAIL** — result=0 ACCEPTED; mock ignores pitch (spec violation) |
| `test_param1_nan_ack_result` | param1=NaN | SKIP — NaN tests skip in mock mode |
| `test_param4_yaw_ack_denied` | param4=90° | **XFAIL** — result=0 ACCEPTED; yaw ignored in COMMAND_INT path (spec violation) |
| `test_param4_yaw_nan_ack` | param4=NaN | SKIP — NaN tests skip in mock mode |
| `test_location_specific_ack` | x/y=SIH home | PASS — result=0 ACCEPTED |
| `test_location_int32max_ack` | x/y=INT32_MAX | PASS — result=0 ACCEPTED |
| `test_location_out_of_range_latlon_ack` | x=120°N, y=200°E | PASS — result=2 DENIED (mock validates range) |
| `test_nan_altitude_ack` | z=NaN | SKIP — NaN tests skip in mock mode |
| `test_wrong_frame_ack` | frame=LOCAL_NED | PASS — result=0 ACCEPTED (mock accepts all frames) |
| `test_latlon_nan_command_long_ack` | COMMAND_LONG param5/6=NaN | SKIP — requires real stack |
| `test_latlon_int32max_command_long_denied` | COMMAND_LONG param5/6=INT32_MAX (float) | SKIP — requires real stack |

The mock returns ACCEPTED for all commands by default, except that out-of-range lat/lon (x/y outside ±900_000_000/±1_800_000_000 and not INT32_MAX sentinel) returns DENIED.
`test_param1_pitch_ack_denied` XFAILs on the mock too — the mock does not model pitch execution capability and returns ACCEPTED regardless of param1.

NaN tests (`test_param1_nan_ack_result`, `test_param4_yaw_nan_ack`, `test_nan_altitude_ack`, `test_latlon_nan_command_long_ack`, `test_latlon_int32max_command_long_denied`) pass `None` in `fields_json` to encode NaN on the wire.
They run with `--ardupilot-sitl` or `--px4-sitl` and are observational (no assertion on ACK result beyond not UNSUPPORTED).

## Tier 2 Flight Tests (test_flight.py)

### Overview

These tests arm the vehicle, send `NAV_TAKEOFF` via raw `COMMAND_INT`, and observe telemetry to verify execution.
A two-stage gate runs before any test:

1. **ACK probe**: confirms `NAV_TAKEOFF` is not UNSUPPORTED (skips all tests on ArduRover).
2. **Execution probe**: arm → send COMMAND_INT → wait ≤ 20 s for any climb > 0.5 m.
   If the vehicle doesn't climb (command accepted but not executed), all 17 tests skip with an informative message.

**Key implementation note**: PX4 ignores the `frame` field in COMMAND_INT NAV_TAKEOFF and always treats `z` as absolute altitude (AMSL).
`_arm_and_send_takeoff()` converts the caller's relative altitude to absolute (home AMSL + relative) before sending, and uses frame=5 (GLOBAL_INT, absolute AMSL) to match.

### PX4 MC (1.18.0)

<!-- TIER2_SUMMARY_START px4-quadcopter -->
**Last run:** 2026-06-03 12:54  **Firmware:** 1.18.0

**Preconditions:**
- Initial mode: no mode change required
- Command arms vehicle: False — must pre-arm
- Mode on NAV_TAKEOFF receipt: TAKEOFF (no change)

**Takeoff approach that worked:**
- COMMAND_INT with lat/lon
- Probe sequence: tier1: COMMAND_INT+lat/lon: ACCEPTED →CLIMBED

**Flight:**
- Climbs diagonally from ground — no vertical-first phase.
  Navigates toward specified lat/lon/alt.
  Holds at waypoint (HOLD mode).
  Yaw: param4=90° IGNORED (heading=4°).
  Ignored: yaw (param4), pitch (param1).


<!-- TIER2_SUMMARY_END px4-quadcopter -->
**Command tests:** 7 PASS, 3 XFAIL (yaw, pitch, out-of-range lat/lon).
**Flight tests (2026-06-02):** 22 PASS, 3 FAIL (late-session SITL timeout after 14+ cycles), 1 SKIP, 3 XFAIL, 2 XPASS.

The 3 failures (`test_unarmed_takeoff`, `test_required_flight_mode`, `test_mode_after_takeoff`) occur at the END of the session after 14 consecutive arm-takeoff-RTL cycles; PX4 SIH SITL degrades after many cycles.
These are operational failures, not protocol failures.
The 2 XPASS are `test_altitude_zero_behaviour` (safety minimum applied) and `test_position_zero_treated_as_current` (PX4 navigates to (0,0) equator — FAIL expected per spec).

COMMAND_INT NAV_TAKEOFF executes on PX4 MC.
PX4 ignores the frame field and treats z as absolute AMSL; commanded altitude is respected.

| Test | Param | Result | Observation |
|------|-------|--------|-------------|
| `test_altitude_nominal` | z=30 m relative | **PASS** | Reached 25.5 m (≥85%) |
| `test_altitude_higher` | z=50 m relative | **PASS** | Reached 42.5 m (≥85%) |
| `test_altitude_very_low` | z=0.5 m relative | **PASS** | Reached 0.74 m — safety minimum ~0.74 m |
| `test_altitude_nan_uses_default` | z=NaN | **PASS** | Took off; reached 0.5 m+ |
| `test_altitude_zero_behaviour` | z=0.0 absolute | **XPASS** | Reached 0.26 m — safety minimum applied despite z=0 |
| `test_yaw_north` | param4=0° | PASS (obs) | Heading ≈ 351° — param4 ignored, uses pre-arm heading |
| `test_yaw_east` | param4=90° | PASS (obs) | Heading ≈ 351° |
| `test_yaw_135` | param4=135° | PASS (obs) | Heading ≈ 351° |
| `test_yaw_near_360` | param4=358° | PASS (obs) | Heading ≈ 353° |
| `test_yaw_negative` | param4=−90° | PASS (obs) | Heading ≈ 352° |
| `test_yaw_overflow` | param4=450° | PASS (obs) | Heading ≈ 354° |
| `test_yaw_very_large` | param4=3600° | PASS (obs) | Heading ≈ 353° |
| `test_position_specific` | x/y=home lat/lon | **PASS** | Target = home; vehicle climbs and arrives at home coordinates (already at home — confirms lat/lon IS used as target) |
| `test_position_int32max_stays_at_home` | x/y=INT32_MAX | **PASS** | INT32_MAX → "use current position"; vehicle within 0.7 m of home at 2.0 m altitude |
| `test_position_zero_treated_as_current` | x=0, y=0 | **FAIL** | x/y=0 are valid coordinates (equator); PX4 navigated toward lat=0,lon=0 — not a "use current" sentinel |
| `test_pitch_comparison_low_vs_high` | param1=5° vs 45° | PASS (obs) | param1 not supported — both cycles logged 0° peak pitch |
| `test_mode_after_takeoff` | informational | PASS (obs) | Mode: Unknown — timed out after 16 prior flight cycles |

`test_px4_mc_takeoff_comprehensive` (separate test, target 200 m north, param4=90°) confirms:
- PX4 MC **does navigate toward the specified lat/lon** — at 2 m alt: 184 m from target; at 15 m: 106 m; at 26 m: 34 m
- param4 (yaw) still ignored — initial heading was 7° (pre-arm), then vehicle turned north toward the target
- Mode stayed `TAKEOFF` throughout the climb

**PX4 MC behaviour summary**:

| Behaviour | Result | Detail |
|-----------|--------|--------|
| Command accepted | PASS | result=0 ACCEPTED |
| Altitude (z) | PASS | Commanded altitude respected (≥85% reached for 30 m and 50 m targets) |
| Altitude default (z=NaN) | PASS | z is a float field; NaN → default altitude ~0.5–0.7 m relative |
| Altitude at zero (z=0 abs) | XPASS | Safety minimum applied (~0.26 m) |
| Yaw (param4) | Not supported | Ignored — vehicle turns toward target lat/lon, not param4 direction |
| Pitch (param1) | Not supported | Ignored in COMMAND_INT path; both 5° and 45° produce 0° peak pitch |
| Lat/Lon navigation | PASS | **PX4 uses x/y as the target destination** — vehicle climbs toward specified lat/lon simultaneously |
| Lat/Lon default (INT32_MAX) | PASS | INT32_MAX → "use current position"; vehicle stays within 0.7 m of home |
| Command completion | Finite | When target lat/lon/alt is reached, PX4 transitions TAKEOFF → HOLD |
| Next mode | TAKEOFF → HOLD | Mode is TAKEOFF during climb; transitions to HOLD on arrival |

### PX4 FW (1.18.0)

<!-- TIER2_SUMMARY_START px4-fixed_wing -->
**Last run:** 2026-06-02 21:37  **Firmware:** 1.18.0

**Preconditions:**
- Initial mode: no mode change required
- Command arms vehicle: False — must pre-arm
- Mode on NAV_TAKEOFF receipt: HOLD → TAKEOFF

**Takeoff approach that worked:**
- none — see probe results
- Probe sequence: tier1: COMMAND_INT+lat/lon: ACCEPTED →no climb; tier2: COMMAND_LONG+alt-only: ACCEPTED →no climb; tier3: COMMAND_INT (unarmed): ACCEPTED →no climb

**Flight:**
- Did not become airborne — ground movement detected (altitude < 2 m, SIH FW runway roll).


<!-- TIER2_SUMMARY_END px4-fixed_wing -->

**Command tests (2026-06-02):** 7 PASS, 3 XFAIL — same as PX4 MC.
**Flight tests (2026-06-02):** 8 PASS (7 command + 1 comprehensive observing ground roll), 20 SKIP (17 altitude/yaw/position/pitch tests + 2 others), 3 XFAIL.

`test_mc_takeoff_comprehensive` runs for vehicle_type=fixed_wing and records that the SIH fixed-wing simulator performs a ground roll (lateral movement detected) but does not achieve liftoff (altitude < 2 m).
This is an SIH simulator limitation, not a protocol issue — a real fixed-wing aircraft would lift off after sufficient runway speed.

### ArduCopter MC (4.8.0)

<!-- TIER2_SUMMARY_START ardupilot-quadcopter -->
**Last run:** 2026-06-02 21:26  **Firmware:** 4.8.0

**Preconditions:**
- Initial mode: GUIDED mode (confirmed)
- Command arms vehicle: False — must pre-arm
- Mode on NAV_TAKEOFF receipt: OFFBOARD (no change)

**Takeoff approach that worked:**
- COMMAND_INT with lat/lon
- Probe sequence: tier1: COMMAND_INT+lat/lon: ACCEPTED →CLIMBED

**Flight:**
- Climbs diagonally from ground — no vertical-first phase.
  Does not navigate toward specified lat/lon (lat/lon ignored).
  Yaw: param4=90° IGNORED (heading=0°).
  Ignored: yaw (param4), pitch (param1).


<!-- TIER2_SUMMARY_END ardupilot-quadcopter -->

**Notes on ArduCopter MC execution:**

- **GUIDED mode required**: ArduCopter's `do_user_takeoff_U_m()` calls `has_user_takeoff(must_navigate=true)` — only GUIDED mode returns `true`.
- **Position stream request required**: ArduCopter does not stream `GLOBAL_POSITION_INT` without an explicit `MAV_CMD_SET_MESSAGE_INTERVAL (511)` request.
  `telemetry.position()` and `mavlink_direct.message("GLOBAL_POSITION_INT")` both time out without it.
  Tests call `_request_position_stream()` before arming.
- **lat/lon ignored**: ArduCopter's handler reads only `packet.z` (altitude); `packet.x`/`packet.y` are documented as "not supported".
- **frame=3 required**: The handler checks `packet.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT`.
  COMMAND_INT must use frame=3; COMMAND_LONG is automatically assigned this frame via `mav_frame_for_command_long()`.

**Execution probe note:** ArduCopter requires GUIDED mode before NAV_TAKEOFF executes.
The standard probe (action.arm() → COMMAND_INT) leaves the vehicle in STABILIZE; the test would always show no-climb, AND leave a dangling `telemetry.health()` gRPC stream (CLAUDE.md §4a) that corrupts subsequent test sessions.
The `_set_executes_cache_for_known_modes` fixture pre-sets `_nav_takeoff_executes=False` for ardupilot/quadcopter to bypass the probe.

**Command tests (2026-06-02):** 8 PASS, 2 XFAIL (pitch, yaw — both ACCEPTED, not DENIED).
**Flight tests (2026-06-02):** 11 PASS (10 command + 1 comprehensive), 20 SKIP.
The 17 standard flight tests skip (no-climb without GUIDED mode setup).
`test_mc_takeoff_comprehensive` runs and PASSES using the tiered probe.

**Result summary**:

| Test | Param | Result | Observation |
|------|-------|--------|-------------|
| `test_mc_takeoff_comprehensive` | 200m N, param4=90°, 30m | **PASS** | COMMAND_INT tier1 worked; lat/lon ignored; yaw ignored |

**ArduCopter MC behaviour summary**:

| Behaviour | Result | Detail |
|-----------|--------|--------|
| Command accepted | PASS | result=0 ACCEPTED |
| Altitude (z) | PASS | param7 (COMMAND_LONG) or z (COMMAND_INT) used as target altitude above home |
| lat/lon | Not supported | ArduCopter ignores x/y; vehicle climbs vertically at home position |
| Yaw (param4) | Not supported | Ignored; heading stays near 0° after takeoff |
| Pitch (param1) | Not supported | Ignored in COMMAND_INT path |
| Required mode | GUIDED | `has_user_takeoff(must_navigate=true)` returns true only in GUIDED mode |
| Next mode | OFFBOARD (GUIDED) | Mode stays GUIDED throughout; no auto-transition |

### ArduPlane FW (4.8.0)

<!-- TIER2_SUMMARY_START ardupilot-fixed_wing -->
**Last run:** 2026-06-02 21:33  **Firmware:** 4.8.0

**Preconditions:**
- Initial mode: TAKEOFF mode (mode 13) — set via DO_SET_MODE before arming
- Command arms vehicle: False — arm() is called after TAKEOFF mode is set; COMMAND_INT NAV_TAKEOFF is NOT used (returns FAILED for non-QuadPlane fixed-wing)
- Mode on NAV_TAKEOFF receipt: N/A — takeoff is triggered by TAKEOFF mode + arm, not by the NAV_TAKEOFF command

**Flight:**
- The plane takes off in its initial direction, ignoring yaw, pitch, lat, and lon (source: ArduPlane do_takeoff() overwrites lat/lon with home±10 units).
  Pitch: TKOFF_PITCH_MIN=5°→peak=19.5°  TKOFF_PITCH_MIN=45°→peak=17.6° — peak pitch unchanged — TECS pitch dominates minimum (param1 ignored in practice).
  Acceptance on reaching TKOFF_ALT — then loiters within TAKEOFF mode.
  Mode transition: NOT automatic (test switches to GUIDED to observe post-takeoff position).
  Pre-arm requirements: TAKEOFF mode (mode 13) + arm.


<!-- TIER2_SUMMARY_END ardupilot-fixed_wing -->

**Notes on ArduPlane FW execution:**

- **`COMMAND_INT NAV_TAKEOFF` not used**: ArduPlane fixed-wing does not execute NAV_TAKEOFF via COMMAND_INT/LONG.
  The supported mechanism is `DO_SET_MODE TAKEOFF (mode 13)` + arm.
  The plane then takes off automatically.
- **`TAKEOFF mode (mode 13)` is the pre-condition**: Sets full-throttle takeoff controller.
- **lat/lon ignored**: `do_takeoff()` in commands_logic.cpp overwrites x/y with `home.lat+10 / home.lng+10`.
- **pitch controlled by `TKOFF_PITCH_MIN`** param, not NAV_TAKEOFF `param1`; mission-path `cmd.p1` is used for this, but command-path param1 is not fed to the param.
- **Execution gate**: `_ensure_nav_takeoff_supported` skips the 17 regular tests on ArduPlane FW (ACCEPTED but no vertical climb).
  `test_arduplane_guided_takeoff_to_target` and `test_mc_takeoff_comprehensive` (vehicle_type=fixed_wing) both run and PASS.
- **Command tests (2026-06-02):** 8 PASS, 2 XFAIL.
  **Flight tests:** 12 PASS (8 command + 2 FW flight), 19 SKIP.

### PX4 VTOL (1.18.0)

<!-- TIER2_SUMMARY_START px4-vtol -->
**Last run:** 2026-06-02  **Firmware:** 1.18.0

**Command accepted:** NAV_TAKEOFF (22) via COMMAND_INT returns ACCEPTED.
PX4 does not gate commands by vehicle type.

**Execution:** NAV_TAKEOFF executes on PX4 VTOL in the **MC hover phase** (same behaviour as PX4 MC for COMMAND_INT).
The vehicle climbs vertically using VTOL motors.
The VTOL-specific ``NAV_VTOL_TAKEOFF (84)`` command triggers the full VTOL sequence (MC hover → FW align → FW transition → FW climb); see `tests/command/baseline_takeoff/README.md`.

For NAV_TAKEOFF (22), the vehicle behaviour is identical to PX4 MC: diagonal climb to target lat/lon/alt, TAKEOFF → HOLD mode transition on arrival.
Yaw (param4) and pitch (param1) are ignored.
<!-- TIER2_SUMMARY_END px4-vtol -->

**Command tests:** 7 PASS, 3 XFAIL (yaw, pitch, out-of-range — identical to PX4 MC).
**Flight tests (2026-06-02):** 21 PASS, 3 FAIL (late-session SITL timeout), 2 SKIP, 3 XFAIL, 2 XPASS.

The 3 failures are the same late-session SITL degradation pattern as PX4 MC.
The 2 SKIP are `test_mc_takeoff_comprehensive` (vtol vehicle type excluded) and `test_arduplane_guided_takeoff_to_target` (ardupilot only).

| Behaviour | Result |
|-----------|--------|
| Command accepted (COMMAND_INT) | PASS — ACCEPTED |
| Altitude respected | PASS — z=30 m → reached 25.5 m+ |
| lat/lon navigation | PASS — navigates to specified lat/lon |
| Yaw (param4) | Not honoured — ignored |
| Pitch (param1) | Not honoured — ignored |
| Mode: NAV_TAKEOFF preferred? | No — NAV_VTOL_TAKEOFF (84) is preferred; see baseline |

### PX4 Rover (1.18.0)

<!-- TIER2_SUMMARY_START px4-rover -->
**Last run:** 2026-06-02  **Firmware:** 1.18.0

**NAV_TAKEOFF does not execute on PX4 Rover.**

NAV_TAKEOFF (22) returns ACCEPTED (PX4 does not gate commands by vehicle type), but the rover cannot fly.
The execution probe detects no climb within 20 s and skips all 17 flight tests.

This contrasts with ArduRover where NAV_TAKEOFF returns UNSUPPORTED (3).
PX4's permissive command handling is a design choice but may be considered a protocol gap — a ground vehicle accepting a flight command without executing it or returning UNSUPPORTED is misleading.
<!-- TIER2_SUMMARY_END px4-rover -->

**Command tests (2026-06-02):** 7 PASS, 3 XFAIL.
NAV_TAKEOFF ACCEPTED — same as PX4 MC.
**Flight tests:** 19 SKIP (no climb) + 2 SKIP (comprehensive/arduplane excluded).

| Behaviour | Result |
|-----------|--------|
| Command accepted | ACCEPTED — PX4 ignores vehicle type |
| Vehicle climbs | ❌ Rover cannot fly; command ignored silently |
| Spec compliance | Gap — should return UNSUPPORTED or FAILED for ground vehicles |

### ArduPlane QP (4.8.0)

<!-- TIER2_SUMMARY_START ardupilot-quadplane -->
**Last run:** 2026-06-02  **Firmware:** 4.8.0

**NAV_TAKEOFF does not execute via COMMAND_INT without GUIDED mode.**

NAV_TAKEOFF (22) returns ACCEPTED, but ArduPlane QuadPlane requires GUIDED mode (custom_mode=15) before the command executes.
Without GUIDED mode, the handler in ArduPlane returns ACCEPTED but no takeoff occurs.
The correct sequence is documented in `tests/command/baseline_takeoff/README.md`: GUIDED (15) → arm → COMMAND_LONG NAV_TAKEOFF p7=altitude.

NAV_VTOL_TAKEOFF (84) is a mission-only command on ArduPlane QuadPlane (executed in AUTO mode); it cannot be sent as a direct COMMAND_INT.
<!-- TIER2_SUMMARY_END ardupilot-quadplane -->

**Command tests (2026-06-02):** 10 PASS (including 2 XFAIL for pitch+yaw).
NAV_TAKEOFF ACCEPTED.
**Flight tests:** 19 SKIP (execution probe bypassed — no climb without GUIDED mode) + 2 SKIP (`test_mc_takeoff_comprehensive` excludes quadplane vehicle type, `test_arduplane_guided_takeoff_to_target` excludes quadplane vehicle type).

| Behaviour | Result |
|-----------|--------|
| Command accepted (COMMAND_INT) | ACCEPTED — but requires GUIDED mode to execute |
| Vehicle climbs from default mode | ❌ No — needs GUIDED (15) mode first |
| NAV_VTOL_TAKEOFF (84) | ❌ Mission-only on ArduPlane QP |
| Correct takeoff mechanism | GUIDED (15) → arm → COMMAND_LONG NAV_TAKEOFF |

### ArduRover (4.8.0)

<!-- TIER2_SUMMARY_START ardupilot-rover -->
**Last run:** 2026-06-02  **Firmware:** 4.8.0

**NAV_TAKEOFF is UNSUPPORTED on ArduRover.**  Result: `MAV_RESULT_UNSUPPORTED (3)`.

ArduRover is a ground vehicle; the command is explicitly rejected.
All 31 tests skip.
This is the correct behaviour per the MAVLink spec.
<!-- TIER2_SUMMARY_END ardupilot-rover -->

**Command tests (2026-06-02):** 10 SKIP — survey-gating active.
NAV_TAKEOFF UNSUPPORTED.
**Flight tests:** 21 SKIP.

| Behaviour | Result |
|-----------|--------|
| Command accepted | ❌ `MAV_RESULT_UNSUPPORTED (3)` — correct behaviour |
| All tests | SKIP (survey-gate: `_ensure_supported()` skips on UNSUPPORTED) |

### Baseline Tests (`baseline/test_baseline.py`)

Baseline tests live in `tests/command/baseline_takeoff/` — see `tests/command/baseline_takeoff/README.md` for the full mode-restriction analysis (which modes accept NAV_TAKEOFF in code vs which actually execute autonomously).

| Test | Stack | Sequence | Result (2026-06-02) |
|------|-------|----------|---------------------|
| `test_px4_mc_takeoff_baseline` | PX4 MC | `action.arm()` → `COMMAND_INT NAV_TAKEOFF (frame=5, z=abs)` | **PASS** — reached 17.0 m |
| `test_ardupilot_mc_takeoff_baseline` | ArduCopter MC | GUIDED mode → arm via COMMAND_LONG 400 → `COMMAND_LONG NAV_TAKEOFF p7=alt` | **PASS** — reached 17.1 m |

---

## Comparison: COMMAND_INT vs mission protocol (param4 yaw)

The same param4 (Yaw) field is handled very differently in the two protocol paths:

| Aspect | Mission protocol | Command protocol |
|--------|-----------------|-----------------|
| **PX4** | param4 stored; used to set heading after takeoff | `rep->current.yaw = NAN` regardless of param4 (`navigator_main.cpp:630`) |
| **ArduCopter** | param4 not stored (zeroed on download) | `// param4 : yaw angle   (not supported)` (`GCS_MAVLink_Copter.cpp:585`) |
| **ArduPlane** | param4 not stored (zeroed on download) | Only altitude read from COMMAND_INT handler (`GCS_MAVLink_Plane.cpp:890`) |

In the mission protocol, PX4 stores and uses param4 to set the heading setpoint after takeoff.
In the command protocol path, all stacks ignore param4 — the yaw is either reset to NaN (PX4) or explicitly noted as unsupported (ArduPilot).

## Spec gaps

The following behaviours are undefined or ambiguous in the MAVLink common.xml spec for `MAV_CMD_NAV_TAKEOFF` when sent via `COMMAND_INT`.
All are documented by `test_flight.py`.

**General principle — unsupported params must NACK**: The MAVLink spec does not explicitly state that a stack must return `MAV_RESULT_DENIED` when it receives a non-NaN value for a parameter it does not support.
This should be a universal rule: `NaN` is the "no preference" sentinel for any optional float parameter; a non-NaN value expresses intent.
A stack that silently accepts and ignores a non-NaN value for an unsupported parameter is violating the parameter contract — the caller has no way to know their intent was discarded.
Any parameter shown by testing to be unsupported (ignored by the stack regardless of value) must return `MAV_RESULT_DENIED` for any non-NaN input.
Suggest: the spec should state this explicitly as a general command-protocol rule, not per-command.

**param1 (MinPitch) — ignored without rejection**: The spec does not state what a stack must do if it cannot honour a non-NaN param1 value.
The correct behaviour is `MAV_RESULT_DENIED`: a non-NaN value expresses the caller's intent that the pitch be obeyed; `NaN` is the explicit "no preference" sentinel.
A stack that returns `ACCEPTED` while silently ignoring a non-NaN param1 is violating the parameter contract.
All tested stacks (PX4 and ArduPilot) return `ACCEPTED` and ignore param1 in the `COMMAND_INT` path — tracked as xfail in `test_param1_pitch_ack_denied`.
Suggest: the spec should explicitly require `MAV_RESULT_DENIED` when a stack cannot honour a non-NaN param1.

**param1 (MinPitch) — range**: No minimum or maximum value is specified.
Values outside `[0, 90]` degrees (e.g. `89°`, `-10°`, `180°`) are accepted in both the mission and command paths by all known stacks (with varying normalisation on storage in the mission path).
Suggest: define the valid range and require `DENIED` for out-of-range values.

**param4 (Yaw) — ignored without rejection**: All tested stacks ignore param4 in the `COMMAND_INT` path and return `ACCEPTED`, violating the general unsupported-param rule above.
Tracked as xfail in `test_param4_yaw_ack_denied`.
Suggest: stacks that cannot honour non-NaN yaw must return `MAV_RESULT_DENIED`.

**param4 (Yaw) — range and normalisation**: No range or normalisation rule is defined.
Should negative values be rejected, treated as equivalent clockwise angles (`-90°` → `270°`), or mean "turn anti-clockwise"?
Should values > `360°` wrap (e.g. `450°` → `90°`) or be rejected?
Should very large values (e.g. `3600°`) cause multiple rotations?
All of this is currently implementation-defined.
In the `COMMAND_INT` path, both PX4 and ArduPilot ignore param4 entirely, so the question is moot in practice — but the spec should still clarify.

**param7 (Altitude)**: No minimum altitude is defined.
Setting `z=0` is ambiguous: should the stack use a safety minimum, reject the command, or hover in place after leaving the ground?
Setting `z=NaN` is permitted by the spec ("use default altitude") but the default is not defined.
Suggest: define a minimum altitude and the meaning of `z=NaN`.

**param5/6 (Lat/Lon) = 0**: Whether `(0, 0)` (equator/prime meridian) should be treated as a valid takeoff coordinate or as a "use current position" sentinel is not specified.
Most stacks appear to treat it as "current position", but this is undocumented.

**param5/6 (Lat/Lon) out-of-range**: The spec does not define the valid range for `x`/`y` or require rejection of geometrically impossible coordinates (e.g. `x=1_200_000_000` = 120°N, `y=2_000_000_000` = 200°E — values above the physical maximum but below `INT32_MAX`).
This should be inferred: a stack that accepts an impossible coordinate may navigate toward the wrong location or exhibit undefined behaviour.
Tested result: ArduPilot returns `MAV_RESULT_DENIED` (correct); PX4 returns `MAV_RESULT_ACCEPTED` (bug — tracked as xfail in `test_location_out_of_range_latlon_ack`).
Suggest: the spec should explicitly state that `x`/`y` values outside `[−900_000_000, 900_000_000]` (lat) and `[−1_800_000_000, 1_800_000_000]` (lon) must return `MAV_RESULT_DENIED`, with the sole exception of `INT32_MAX` (the "use current position" sentinel).

**Position semantics**: The spec does not state whether the `COMMAND_INT` `x`/`y` fields for `NAV_TAKEOFF` specify the position _from which_ the vehicle should take off (navigate there, then climb) or the position _to arrive at_ after climbing.
Empirical result (PX4 MC): the vehicle treats `x`/`y` as the **target destination** — it climbs toward the specified lat/lon/alt simultaneously, rather than climbing vertically first then navigating.

**param5/6 (Lat/Lon) — ArduPlane ignores without rejection**: ArduPlane (via the COMMAND_LONG conversion path) accepts any lat/lon value but ignores it entirely (source: `x=0, y=0` set in `convert_MAV_CMD_NAV_TAKEOFF_to_COMMAND_INT`).
Per the general unsupported-param rule, any non-`INT32_MAX` lat/lon should return `MAV_RESULT_DENIED` if the stack cannot use the coordinate.
Current behaviour (ACCEPTED + ignore) is a spec violation.

**param1 (MinPitch) — vehicle type range not defined**: The spec does not define separate ranges for fixed-wing vs multicopter.
For fixed-wing, the maximum physically achievable climb pitch is ~25–30°; values above this cannot be honoured (the aircraft would stall).
90° is impossible for any fixed-wing — the stack should return `MAV_RESULT_DENIED` for values that exceed the vehicle's capability, but no current stack does.
Suggest: the spec should define a vehicle-type-aware range: fixed-wing `[0°, 30°]` maximum, MC/VTOL: param1 should be NaN (not meaningful for vertical climbers; non-NaN should be DENIED or silently zero).

**Command completion — not explicitly defined**: The spec does not state when `COMMAND_INT NAV_TAKEOFF` is considered complete or what mode the vehicle should enter afterwards.
PX4 MC transitions to HOLD on arrival at the target; ArduPlane TAKEOFF mode transitions internally to loitering behaviour at the target altitude.
Suggest: define completion as "vehicle has reached the commanded altitude within the specified lat/lon tolerance" and require a mode transition to a station-keeping mode (e.g. HOLD/loiter).

## Running

```bash
# Mock (tier 1 only)
pytest tests/command/nav_takeoff/test_command.py -v --log-cli-level=INFO

# PX4 multicopter — tier 1
pytest tests/command/nav_takeoff/test_command.py --drone-address=udp://:14540 -v --log-cli-level=INFO

# ArduCopter — tier 1
pytest tests/command/nav_takeoff/test_command.py \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 -v --log-cli-level=INFO

# PX4 multicopter — tier 2 flight
pytest tests/command/nav_takeoff/test_flight.py \
    --drone-address=udp://:14540 --connection-timeout=60 \
    --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \
    --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO

# ArduCopter — tier 2 flight
pytest tests/command/nav_takeoff/test_flight.py \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
    --ardupilot-sitl=~/ardu_sitl/arducopter \
    --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 \
    --vehicle-type=quadcopter --autopilot=ardupilot -v --log-cli-level=INFO

# ArduPlane fixed-wing — tier 2 flight
pytest tests/command/nav_takeoff/test_flight.py \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
    --ardupilot-sitl=~/ardu_sitl/arduplane --vehicle-type=fixed_wing \
    --autopilot=ardupilot -v --log-cli-level=INFO
```
