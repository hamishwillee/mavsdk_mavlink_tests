# MAV_CMD_DO_REPOSITION (cmd=192) — command protocol tests

Reposition the vehicle to a specific WGS84 global position.
Intended for guided commands; for missions use MAV_CMD_NAV_WAYPOINT.

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

`hasLocation="true"`, `isDestination="true"` → use **COMMAND_INT** (integer lat/lon × 1e7).

## PX4 mode-dependent ACK (commit bc236e7178)

Before the fix, PX4 returned `UNSUPPORTED` for all inputs.
After the fix (`dakejahl/do-reposition-ack` branch), the ACK is mode-dependent:

| Condition | ACK |
|-----------|-----|
| `param2` bit-0 set (CHANGE_MODE) | `ACCEPTED` — switches to AUTO_LOITER |
| `param2=0` AND already in AUTO_LOITER | `ACCEPTED` — repositions hold point |
| `param2=0` AND not in AUTO_LOITER | `DENIED` |

## Tier 1 test results

### Mock (paired mode) — 2026-06-04

20 passed, 10 skipped, 1 xfailed

| Test | Result | ACK | Notes |
|------|--------|-----|-------|
| `test_denied_not_in_hold` | SKIP | — | mock has no mode state |
| `test_accepted_change_mode` | SKIP | — | mock has no mode state |
| `test_accepted_already_in_hold` | SKIP | — | mock has no mode state |
| `test_command_accepted` | PASS | 0 | |
| `test_param2_change_mode_flag` | PASS | 0 | |
| `test_param2_flags_zero` | PASS | 0 | mock always accepts |
| `test_param2_relative_yaw_only` | PASS | 0 | mock always accepts |
| `test_param2_all_flags` | PASS | 0 | |
| `test_param2_undefined_bits` | PASS | 0 | |
| `test_param1_default_speed` | PASS | 0 | |
| `test_param1_positive_speed` | PASS | 0 | |
| `test_param1_zero_speed` | PASS | 0 | |
| `test_param1_nan_speed` | SKIP | — | requires real stack |
| `test_param1_below_min` | XFAIL | 0 | mock accepts −5 m/s (below minValue=−1) |
| `test_param4_yaw_nan` | PASS | 0 | |
| `test_param4_yaw_zero` | PASS | 0 | |
| `test_param4_yaw_specific` | PASS | 0 | |
| `test_param4_relative_yaw_with_flag` | SKIP | — | requires real stack |
| `test_param3_zero` | PASS | 0 | |
| `test_param3_nan` | SKIP | — | requires real stack |
| `test_param3_positive` | PASS | 0 | |
| `test_param3_negative` | PASS | 0 | |
| `test_location_specific` | PASS | 0 | |
| `test_location_int32max` | PASS | 0 | |
| `test_location_out_of_range_latlon` | PASS | 2 | mock correctly returns DENIED for out-of-range coordinates |
| `test_altitude_nan` | SKIP | — | requires real stack |
| `test_altitude_zero` | PASS | 0 | |
| `test_altitude_only_reposition` | PASS | 0 | |
| `test_all_nan_pause` | SKIP | — | requires real stack |
| `test_command_long_nan_latlon` | SKIP | — | requires real stack |
| `test_command_long_int32max_float` | SKIP | — | requires real stack |

### PX4 MC 1.18.0-alpha — unpatched (before bc236e7178)

All tests: SKIP — `DO_REPOSITION` returns `UNSUPPORTED` (confirmed by survey).

### PX4 MC 1.18.0-alpha — patched (dakejahl/do-reposition-ack, git `78716e23bc`) — 2026-06-04

27 passed, 4 xfailed

| Test | Result | ACK | Notes |
|------|--------|-----|-------|
| `test_denied_not_in_hold` | XFAIL | 0 | PX4 SIH auto-transitions to AUTO_LOITER after EKF convergence before the test runs; vehicle is already in Hold so param2=0 → ACCEPTED (branch 2) not DENIED (branch 3). Only verifiable on a real vehicle or with an explicit pre-test mode reset. |
| `test_accepted_change_mode` | PASS | 0 | CHANGE_MODE → ACCEPTED, switches to AUTO_LOITER ✓ |
| `test_accepted_already_in_hold` | PASS | 0 | param2=0 while in Hold → ACCEPTED ✓ (branch 2 of fix) |
| `test_command_accepted` | PASS | 0 | |
| `test_param2_change_mode_flag` | PASS | 0 | |
| `test_param2_flags_zero` | PASS | 0 | already in Hold from prior tests → ACCEPTED |
| `test_param2_relative_yaw_only` | PASS | 0 | already in Hold → ACCEPTED |
| `test_param2_all_flags` | PASS | 0 | |
| `test_param2_undefined_bits` | PASS | 0 | bit 0 (CHANGE_MODE) set in 255; accepted |
| `test_param1_default_speed` | PASS | 0 | |
| `test_param1_positive_speed` | PASS | 0 | |
| `test_param1_zero_speed` | PASS | 0 | treated same as −1 (use default) |
| `test_param1_nan_speed` | PASS | 0 | treated same as −1 (use default) |
| `test_param1_below_min` | XFAIL | 0 | PX4 treats any param1 ≤ 0 as default; −5 accepted silently |
| `test_param4_yaw_nan` | PASS | 0 | |
| `test_param4_yaw_zero` | PASS | 0 | PX4 navigator applies finite param4 as heading setpoint |
| `test_param4_yaw_specific` | PASS | 0 | |
| `test_param4_relative_yaw_with_flag` | PASS | 0 | |
| `test_param3_zero` | PASS | 0 | |
| `test_param3_nan` | PASS | 0 | |
| `test_param3_positive` | PASS | 0 | PX4 MC accepts non-zero radius (spec gap: MC should DENIED) |
| `test_param3_negative` | PASS | 0 | PX4 accepts negative radius |
| `test_location_specific` | PASS | 0 | |
| `test_location_int32max` | PASS | 0 | |
| `test_location_out_of_range_latlon` | XFAIL | 0 | PX4 does not validate coordinate range (spec gap) |
| `test_altitude_nan` | PASS | 0 | |
| `test_altitude_zero` | PASS | 0 | |
| `test_altitude_only_reposition` | PASS | 0 | |
| `test_all_nan_pause` | PASS | 0 | PX4 accepts all-NaN "pause" via COMMAND_LONG |
| `test_command_long_nan_latlon` | PASS | 0 | |
| `test_command_long_int32max_float` | XFAIL | 2 | PX4 DENIED — incorrectly treats float(INT32_MAX) as protocol error; INT32_MAX is a valid "use current position" sentinel |

### ArduCopter MC

Not yet run.

### ArduRover

Not yet run.

## Tier 2 (flight) test results

### PX4 MC (patched dakejahl/do-reposition-ack)

Not yet run.

### ArduCopter MC

Not yet run.

## Spec gaps and violations

| Issue | Stacks affected | Type |
|-------|-----------------|------|
| `param2=0` returned UNSUPPORTED instead of DENIED when not in mode-appropriate state | PX4 pre-fix | Fixed by bc236e7178 |
| `param1 minValue=−1` not enforced; values below −1 accepted as default | PX4, ArduCopter | Spec gap |
| Non-zero `param3` (loiter radius) accepted on multicopter; MC cannot honour it | PX4 MC | Spec gap — should DENIED |
| Out-of-range lat/lon accepted (>90°N, >180°E) | PX4 | Spec gap — should DENIED |
| `float(INT32_MAX)` in COMMAND_LONG param5/6 rejected as protocol error | PX4 | Spec violation — INT32_MAX is valid "use current position" sentinel |
