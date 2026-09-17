# MAV_CMD_DO_REPOSITION (cmd=192) — command protocol tests

Reposition the vehicle to a specific WGS84 global position. Intended for guided commands; for missions use MAV_CMD_NAV_WAYPOINT.

## Parameter definition

| Param | Label | Description | Values | Units |
|-------|-------|-------------|--------|-------|
| 1 | Speed | Ground speed; <0 (−1) for default | min: -1 | m/s |
| 2 | Bitmask | MAV_DO_REPOSITION_FLAGS | see below | |
| 3 | Radius | Loiter radius (planes only); 0 or NaN ignored | positive only | m |
| 4 | Yaw | Heading; NaN = use current system heading mode | | rad |
| 5 | Latitude | | | |
| 6 | Longitude | | | |
| 7 | Altitude | | | m |

**MAV_DO_REPOSITION_FLAGS bitmask:**
- `1` (bit 0): `CHANGE_MODE` — switch vehicle to guided/hold mode immediately
- `2` (bit 1): `RELATIVE_YAW` — yaw relative to vehicle current heading (not North)

## Command type

`hasLocation="true"`, `isDestination="true"` → **COMMAND_INT** (integer lat/lon × 1e7).

## PX4 mode-dependent ACK (commit bc236e7178)

Before the fix, PX4 returned `UNSUPPORTED` for all inputs. After (`dakejahl/do-reposition-ack` branch):

| Condition | ACK |
|-----------|-----|
| `param2` bit-0 set (CHANGE_MODE) | `ACCEPTED` — switches to AUTO_LOITER |
| `param2=0` AND already in AUTO_LOITER | `ACCEPTED` — repositions hold point |
| `param2=0` AND not in AUTO_LOITER | `DENIED` |

## Tier 1 test results

### PX4 MC 1.18.0-alpha — unpatched (before bc236e7178)

All tests SKIP — `DO_REPOSITION` returns `UNSUPPORTED` (confirmed by survey).

### PX4 MC 1.18.0-alpha — patched (dakejahl/do-reposition-ack, git `78716e23bc`) — 2026-06-04

27 passed, 4 xfailed.

| Test | Result | ACK | Notes |
|------|--------|-----|-------|
| `test_do_reposition_denied_not_in_hold` | XFAIL | 0 | PX4 SIH auto-transitions to AUTO_LOITER after EKF convergence before the test runs, so param2=0 → ACCEPTED (branch 2), not DENIED (branch 3). Only verifiable on real hardware or with an explicit pre-test mode reset. |
| `test_do_reposition_accepted_change_mode` | PASS | 0 | CHANGE_MODE → ACCEPTED, switches to AUTO_LOITER |
| `test_do_reposition_accepted_already_in_hold` | PASS | 0 | param2=0 while in Hold → ACCEPTED (branch 2) |
| `test_do_reposition_command_accepted` | PASS | 0 | |
| `test_do_reposition_param2_change_mode_flag` | PASS | 0 | |
| `test_do_reposition_param2_flags_zero` | PASS | 0 | already in Hold from prior tests |
| `test_do_reposition_param2_relative_yaw_only` | PASS | 0 | already in Hold |
| `test_do_reposition_param2_all_flags` | PASS | 0 | |
| `test_do_reposition_param2_undefined_bits` | PASS | 0 | bit 0 (CHANGE_MODE) set in 255 |
| `test_do_reposition_param1_default_speed` | PASS | 0 | |
| `test_do_reposition_param1_positive_speed` | PASS | 0 | |
| `test_do_reposition_param1_zero_speed` | PASS | 0 | treated same as −1 (use default) |
| `test_do_reposition_param1_nan_speed` | PASS | 0 | treated same as −1 (use default) |
| `test_do_reposition_param1_below_min` | XFAIL | 0 | PX4 treats any param1 ≤ 0 as default; −5 accepted silently |
| `test_do_reposition_param4_yaw_nan` | PASS | 0 | |
| `test_do_reposition_param4_yaw_zero` | PASS | 0 | applied as heading setpoint |
| `test_do_reposition_param4_yaw_specific` | PASS | 0 | |
| `test_do_reposition_param4_relative_yaw_with_flag` | PASS | 0 | |
| `test_do_reposition_param3_zero` | PASS | 0 | |
| `test_do_reposition_param3_nan` | PASS | 0 | |
| `test_do_reposition_param3_positive` | PASS | 0 | MC accepts non-zero radius — spec gap, should DENY |
| `test_do_reposition_param3_negative` | PASS | 0 | negative radius accepted |
| `test_do_reposition_location_specific` | PASS | 0 | |
| `test_do_reposition_location_int32max` | PASS | 0 | |
| `test_do_reposition_location_out_of_range_latlon` | XFAIL | 0 | coordinate range not validated — spec gap |
| `test_do_reposition_altitude_nan` | PASS | 0 | |
| `test_do_reposition_altitude_zero` | PASS | 0 | |
| `test_do_reposition_altitude_only_reposition` | PASS | 0 | |
| `test_do_reposition_all_nan_pause` | PASS | 0 | all-NaN "pause" accepted via COMMAND_LONG |
| `test_do_reposition_command_long_nan_latlon` | PASS | 0 | |
| `test_do_reposition_command_long_int32max_float` | XFAIL | 2 | DENIED — PX4 treats float(INT32_MAX) as a protocol error rather than the valid "use current position" sentinel |

### ArduCopter MC — 2026-06-08

26 passed, 3 failed, 1 skipped, 1 xfailed. Log: `logs/command_do_reposition_arducopter_20260608.log`.

| Test | Result | ACK | Notes |
|------|--------|-----|-------|
| `test_do_reposition_denied_not_in_hold` | PASS | 2 | not in Hold → DENIED, matches PX4 mode-gating |
| `test_do_reposition_accepted_change_mode` | **FAIL** | 4 | CHANGE_MODE set; expected ACCEPTED, got FAILED — see finding 1 below |
| `test_do_reposition_accepted_already_in_hold` | SKIP | — | depends on a prior test reaching Hold mode |
| `test_do_reposition_command_accepted` | PASS | 4 | baseline — ArduCopter responds FAILED, not UNSUPPORTED |
| `test_do_reposition_param2_change_mode_flag` | **FAIL** | 4 | expected ACCEPTED, got FAILED |
| `test_do_reposition_param2_flags_zero` | PASS | 2 | no flags → DENIED |
| `test_do_reposition_param2_relative_yaw_only` | PASS | 2 | RELATIVE_YAW, no CHANGE_MODE → DENIED |
| `test_do_reposition_param2_all_flags` | **FAIL** | 4 | CHANGE_MODE\|RELATIVE_YAW(3); expected ACCEPTED, got FAILED |
| `test_do_reposition_param2_undefined_bits` | PASS | 4 | |
| `test_do_reposition_param1_default_speed` … `test_do_reposition_param4_relative_yaw_with_flag` | PASS | 4 | 8 param1/param4 value sweeps, no denial |
| `test_do_reposition_param3_zero` / `test_do_reposition_param3_nan` / `test_do_reposition_param3_positive` / `test_do_reposition_param3_negative` | PASS | 4 | |
| `test_do_reposition_location_specific` | PASS | 4 | |
| `test_do_reposition_location_int32max` | PASS | 2 | INT32_MAX ("use current position") → DENIED — spec violation, same family as ArduPilot mission-protocol NAV_TAKEOFF sentinel rejection |
| `test_do_reposition_location_out_of_range_latlon` | PASS | 2 | out-of-range lat/lon correctly DENIED |
| `test_do_reposition_altitude_nan` | PASS | 2 | NaN ("use current altitude") → DENIED — spec violation, should accept |
| `test_do_reposition_altitude_zero` / `test_do_reposition_altitude_only_reposition` / `test_do_reposition_all_nan_pause` / `test_do_reposition_command_long_nan_latlon` / `test_do_reposition_command_long_int32max_float` | PASS | — | UNKNOWN — no ACK within 5s (see finding 2 below) |
| `test_do_reposition_param1_below_min` | XFAIL | 4 | −5.0 (below minValue=−1) accepted instead of denied — spec gap, same as PX4 |

**Source-traced root causes** (`ArduCopter/GCS_MAVLink_Copter.cpp:430-470` `handle_command_int_do_reposition()`; `ArduCopter/mode_guided.cpp:466` `set_destination()`; `libraries/AC_WPNav/AC_WPNav.cpp:322,983`):

1. **`set_destination()` fails for nearly every probe — not CHANGE_MODE-specific.** `_reposition_cmd()` (`test_command.py:85`) defaults `param2=1.0` (CHANGE_MODE), so almost every probe carries that flag. With `change_modes=true`, the handler skips the mode-gating DENIED branch and calls `mode_guided.set_destination(...)` directly; if that returns `false` it returns `FAILED` immediately, before ever attempting the mode switch. So FAILED(4) is the generic "could not set destination" path, not a CHANGE_MODE-specific one — the three failing tests are simply the only ones with a hard `ACCEPTED` assertion; every other CHANGE_MODE-flagged probe hits the same failure but is observational. Tracing into `set_wp_destination_loc()` → `get_vector_NED_m()` identifies three candidate failure points: geofence breach, EKF origin not yet established, or altitude-frame conversion failure (`frame=6`/GLOBAL_RELATIVE_ALT_INT needs a valid origin↔home altitude offset). Source review can't distinguish which fires at runtime — needs `AC_Fence`/`AC_WPNav`/`LOGGER_WRITE_ERROR` debug capture.
2. **ACK responsiveness degrades mid-sequence.** From `test_do_reposition_param1_nan_speed` onward roughly 14 probes in, the pattern trails off into no-ACK; no command-queue/dedup/throttle logic in `GCS_Common.cpp` would explain a permanent stop mid-session. Candidates: a downlink backlog from repeated `LOGGER_WRITE_ERROR`/STATUSTEXT emissions (one per failed `set_destination()`) starving `COMMAND_ACK`, or a test-harness-side `mavlink_direct`/gRPC artifact (see `[[grpc_cancel_pattern]]`). Needs runtime packet capture (`mavlogdump`/Wireshark) to confirm, not further source review.

### ArduRover — 2026-06-08

27 passed, 3 failed, 1 skipped. Log: `logs/command_do_reposition_ardurover_20260608.log`.

Same outward pattern as ArduCopter (mode-gated DENIED when correctly gated, FAILED on the three CHANGE_MODE-asserting tests, ACKs stop entirely partway through — after ~12 probes here, from `test_do_reposition_param1_nan_speed` onward, vs. ~14 on ArduCopter but at the identical trigger test) — full per-test table: `logs/command_do_reposition_ardurover_20260608.log`.

**Source-traced root causes** (`Rover/GCS_MAVLink_Rover.cpp:509-545` `handle_command_int_do_reposition`) — same conclusions as ArduCopter but via a different path, worth noting because it changes the likely trigger for finding 2:

1. Rover's handler order is: mode-gating check → validate location → **attempt mode switch to GUIDED** → `set_desired_speed()` → `mode_guided.set_desired_location()` → ACCEPTED. Because CHANGE_MODE is set on nearly every probe, Rover — unlike Copter, which fails at `set_destination()` before ever attempting a mode switch — actually attempts, and likely succeeds at, `set_mode(GUIDED)` on the first CHANGE_MODE-flagged probe, then fails at `set_desired_location()`. So on Rover a real mode transition to GUIDED plausibly occurs mid-sequence, making it a stronger candidate trigger for finding 2 than anything available on the Copter side.
2. ACK loss starts at the same test (`test_do_reposition_param1_nan_speed`) on both vehicles despite different firmware — argues against coincidence. No ACK-suppression mechanism found in `GCS_Common.cpp` for either vehicle. Two live candidates: the GUIDED-mode transition itself (plus associated failsafe/STATUSTEXT traffic), or something in the `param1=NaN` encoding common to both vehicles. Needs runtime packet capture to distinguish, not further source review.

### Mock (paired mode) — 2026-06-04

20 passed, 10 skipped, 1 xfailed.

| Test | Result | Notes |
|------|--------|-------|
| `test_do_reposition_denied_not_in_hold` / `test_do_reposition_accepted_change_mode` / `test_do_reposition_accepted_already_in_hold` | SKIP | mock has no mode state |
| `test_do_reposition_param1_nan_speed` / `test_do_reposition_param4_relative_yaw_with_flag` / `test_do_reposition_param3_nan` / `test_do_reposition_altitude_nan` / `test_do_reposition_all_nan_pause` / `test_do_reposition_command_long_nan_latlon` / `test_do_reposition_command_long_int32max_float` | SKIP | requires real stack |
| `test_do_reposition_param1_below_min` | XFAIL | mock accepts −5 m/s (below minValue=−1) |
| `test_do_reposition_location_out_of_range_latlon` | PASS | mock correctly DENIEDs out-of-range coordinates |
| everything else | PASS | mock always accepts |

## Tier 2 (flight) test results

PX4 MC (patched) and ArduCopter MC: not yet run.

## Spec gaps and violations

| Issue | Stacks affected | Type |
|-------|-----------------|------|
| `param2=0` returned UNSUPPORTED instead of DENIED when not in mode-appropriate state | PX4 pre-fix | Fixed by bc236e7178 |
| `param1 minValue=−1` not enforced; values below −1 accepted as default | PX4, ArduCopter | Spec gap |
| Non-zero `param3` (loiter radius) accepted on multicopter, which can't honour it | PX4 MC | Spec gap — should DENY |
| Out-of-range lat/lon accepted (>90°N, >180°E) | PX4 | Spec gap — should DENY |
| `float(INT32_MAX)` in COMMAND_LONG param5/6 rejected as a protocol error | PX4 | Spec violation — INT32_MAX is the valid "use current position" sentinel |
| CHANGE_MODE set → `FAILED(4)` instead of `ACCEPTED(0)` + mode switch — command acknowledged but never executed; contrasts with PX4's patched ACCEPTED+AUTO_LOITER behaviour. Traced to `set_destination()`/`set_desired_location()` failing (see per-stack findings above); underlying cause (geofence/EKF-origin/altitude-frame candidates) needs runtime log capture | ArduCopter, ArduRover | Spec violation |
| `COMMAND_ACK` stops being sent partway through a probe sequence (same trigger test on both vehicles) | ArduCopter, ArduRover | Observed only — no ACK-suppression mechanism found in source; spec requires an ACK for every command. Needs runtime packet capture — see per-stack findings above |
