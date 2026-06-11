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

### ArduCopter MC — 2026-06-08

26 passed, 3 failed, 1 skipped, 1 xfailed.
Log: `logs/command_do_reposition_arducopter_20260608.log`.

| Test | Result | ACK | Notes |
|------|--------|-----|-------|
| `test_denied_not_in_hold` | PASS | 2 | param2=0 (not in Hold) → DENIED, matches PX4 mode-gating |
| `test_accepted_change_mode` | **FAIL** | 4 | CHANGE_MODE bit set; expected ACCEPTED (0), got **FAILED** (4) |
| `test_accepted_already_in_hold` | SKIP | — | depends on a prior test reaching Hold mode |
| `test_command_accepted` | PASS | 4 | baseline COMMAND_INT — observational; ArduCopter responds FAILED, not UNSUPPORTED |
| `test_param2_change_mode_flag` | **FAIL** | 4 | expected ACCEPTED (0), got **FAILED** (4) |
| `test_param2_flags_zero` | PASS | 2 | param2=0 (no flags) → DENIED |
| `test_param2_relative_yaw_only` | PASS | 2 | param2=RELATIVE_YAW(2), no CHANGE_MODE bit → DENIED |
| `test_param2_all_flags` | **FAIL** | 4 | param2=CHANGE_MODE\|RELATIVE_YAW(3); expected ACCEPTED (0), got **FAILED** (4) |
| `test_param2_undefined_bits` | PASS | 4 | param2=255 (all bits) — observational |
| `test_param1_default_speed` | PASS | 4 | param1=−1 (default) — observational |
| `test_param1_positive_speed` | PASS | 4 | param1=5.0 m/s — observational |
| `test_param1_zero_speed` | PASS | 4 | param1=0.0 — observational |
| `test_param1_nan_speed` | PASS | 4 | param1=NaN — observational |
| `test_param1_below_min` | XFAIL | 4 | stack accepts param1=−5.0 (below minValue=−1) instead of denying — spec gap, same as PX4 |
| `test_param4_yaw_nan` | PASS | 4 | param4=NaN — observational |
| `test_param4_yaw_zero` | PASS | 4 | param4=0.0 rad (North) — observational |
| `test_param4_yaw_specific` | PASS | 4 | param4=π/2 rad (East) — observational |
| `test_param4_relative_yaw_with_flag` | PASS | 4 | param2=CHANGE_MODE\|RELATIVE_YAW, param4=π/4 — observational |
| `test_param3_zero` | PASS | 4 | param3=0.0 (ignored) — observational |
| `test_param3_nan` | PASS | 4 | param3=NaN (ignored) — observational |
| `test_param3_positive` | PASS | 4 | param3=100.0 m — observational |
| `test_param3_negative` | PASS | 4 | param3=−50.0 (negative, invalid) — observational |
| `test_location_specific` | PASS | 4 | params 5/6 (lat/lon) specific — observational |
| `test_location_int32max` | PASS | 2 | params 5/6 = INT32_MAX ("use current position") → DENIED — spec violation, same family as ArduPilot mission-protocol NAV_TAKEOFF sentinel rejection |
| `test_location_out_of_range_latlon` | PASS | 2 | params 5/6 out-of-range lat/lon → correctly DENIED |
| `test_altitude_nan` | PASS | 2 | param7=NaN ("use current altitude") → DENIED — spec violation (NaN should be accepted) |
| `test_altitude_zero` | PASS | — | UNKNOWN — no ACK within 5 s |
| `test_altitude_only_reposition` | PASS | — | UNKNOWN — no ACK within 5 s |
| `test_all_nan_pause` | PASS | — | UNKNOWN — no ACK within 5 s (COMMAND_LONG all-NaN "pause") |
| `test_command_long_nan_latlon` | PASS | — | UNKNOWN — no ACK within 5 s (COMMAND_LONG param5/6=NaN) |
| `test_command_long_int32max_float` | PASS | — | UNKNOWN — no ACK within 5 s (COMMAND_LONG param5/6=INT32_MAX as float) |

**Source-traced root causes for the two ArduCopter behaviours below** (`ArduCopter/GCS_MAVLink_Copter.cpp:430-470`, `handle_command_int_do_reposition()`; `ArduCopter/mode_guided.cpp:466`, `set_destination()`; `libraries/AC_WPNav/AC_WPNav.cpp:322,983`):

1. **`set_destination()` fails for virtually every probe — not a CHANGE_MODE-specific bug.**  `_reposition_cmd()` (`tests/command/do_reposition/test_command.py:85`) defaults `param2=1.0` (`CHANGE_MODE` bit) — "valid regardless of starting mode" — so nearly every probe in the suite carries that flag.  In ArduCopter's handler, `change_modes=true` skips the mode-gating `DENIED` branch and goes straight to `copter.mode_guided.set_destination(request_location, …)`; **if that returns `false` the handler returns `MAV_RESULT_FAILED` immediately**, *before* ever attempting the mode switch — regardless of whether `CHANGE_MODE` is set.  So FAILED(4) is the generic "could not set destination" path, not a `CHANGE_MODE`-specific code path; the three failing tests (`test_accepted_change_mode`, `test_param2_change_mode_flag`, `test_param2_all_flags`) simply happen to be the only ones with a hard `result == ACCEPTED` assertion — every other `CHANGE_MODE`-flagged probe *also* gets FAILED(4) but is purely observational and passes regardless.  Tracing further into `set_destination()` → `AC_WPNav::set_wp_destination_loc()` → `get_vector_NED_m()` shows three candidate failure points: (a) geofence breach (`AC_Fence::check_location_within_fence`), (b) EKF origin not yet established (`Location::get_vector_xy_from_origin_NE_m`), (c) altitude-frame conversion failure (`Location::get_alt_m(ABOVE_ORIGIN, …)` — relevant because the probe uses `frame=6`/`GLOBAL_RELATIVE_ALT_INT`, which requires a valid origin↔home altitude offset).  Source inspection alone cannot distinguish which one fires at runtime; that needs a run with `AC_Fence`/`AC_WPNav`/`LOGGER_WRITE_ERROR` debug output captured.
2. **ACK responsiveness degrades mid-sequence** — the first ~14 probes (through `test_param1_zero_speed`) all receive a `result=4` (FAILED) ACK; from `test_param1_nan_speed` onward, **every remaining probe gets no ACK at all** (`UNKNOWN — no ACK within 5.0 s`).  Grepping `GCS_Common.cpp` for command-queue / dedup / throttle logic found **no mechanism that would make ArduCopter permanently stop ACKing one command type partway through a session** — so this is not an obvious source-level "give up after N attempts" behaviour.  The cut-over coinciding with the first NaN-valued `param1` probe is suggestive but inconclusive from source alone: candidates include a downlink backlog from repeated `LOGGER_WRITE_ERROR`/STATUSTEXT emissions (one per failed `set_destination()` call) starving `COMMAND_ACK`, or a test-harness-side artifact (this project has documented `mavlink_direct`/gRPC-stream subscription quirks — see `[[grpc_cancel_pattern]]`).  Confirming the real cause needs runtime packet capture (e.g. `mavlogdump`/Wireshark) across the cut-over point, not source review.

### ArduRover — 2026-06-08

27 passed, 3 failed, 1 skipped.
Log: `logs/command_do_reposition_ardurover_20260608.log`.

| Test | Result | ACK | Notes |
|------|--------|-----|-------|
| `test_denied_not_in_hold` | PASS | 2 | param2=0 (not in Hold) → DENIED, matches PX4 mode-gating |
| `test_accepted_change_mode` | **FAIL** | 4 | CHANGE_MODE bit set; expected ACCEPTED (0), got **FAILED** (4) |
| `test_accepted_already_in_hold` | SKIP | — | depends on a prior test reaching Hold mode |
| `test_command_accepted` | PASS | 4 | baseline COMMAND_INT — observational; ArduRover responds FAILED, not UNSUPPORTED |
| `test_param2_change_mode_flag` | **FAIL** | 4 | expected ACCEPTED (0), got **FAILED** (4) |
| `test_param2_flags_zero` | PASS | 4 | param2=0 (no flags) — observational (note: differs from `test_denied_not_in_hold`'s DENIED — see below) |
| `test_param2_relative_yaw_only` | PASS | 4 | param2=RELATIVE_YAW(2) — observational |
| `test_param2_all_flags` | **FAIL** | 4 | param2=CHANGE_MODE\|RELATIVE_YAW(3); expected ACCEPTED (0), got **FAILED** (4) |
| `test_param2_undefined_bits` | PASS | 4 | param2=255 (all bits) — observational |
| `test_param1_default_speed` | PASS | 4 | param1=−1 (default) — observational |
| `test_param1_positive_speed` | PASS | 4 | param1=5.0 m/s — observational |
| `test_param1_zero_speed` | PASS | 4 | param1=0.0 — observational |
| `test_param1_nan_speed` | PASS | — | UNKNOWN — no ACK within 5 s |
| `test_param1_below_min` | PASS | — | UNKNOWN — no ACK within 5 s (no XFAIL — assertion only fires when an ACK is received) |
| `test_param4_yaw_nan` | PASS | — | UNKNOWN — no ACK within 5 s |
| `test_param4_yaw_zero` | PASS | — | UNKNOWN — no ACK within 5 s |
| `test_param4_yaw_specific` | PASS | — | UNKNOWN — no ACK within 5 s |
| `test_param4_relative_yaw_with_flag` | PASS | — | UNKNOWN — no ACK within 5 s |
| `test_param3_zero` | PASS | — | UNKNOWN — no ACK within 5 s |
| `test_param3_nan` | PASS | — | UNKNOWN — no ACK within 5 s |
| `test_param3_positive` | PASS | — | UNKNOWN — no ACK within 5 s |
| `test_param3_negative` | PASS | — | UNKNOWN — no ACK within 5 s |
| `test_location_specific` | PASS | — | UNKNOWN — no ACK within 5 s |
| `test_location_int32max` | PASS | — | UNKNOWN — no ACK within 5 s |
| `test_location_out_of_range_latlon` | PASS | — | UNKNOWN — no ACK within 5 s |
| `test_altitude_nan` | PASS | — | UNKNOWN — no ACK within 5 s |
| `test_altitude_zero` | PASS | — | UNKNOWN — no ACK within 5 s |
| `test_altitude_only_reposition` | PASS | — | UNKNOWN — no ACK within 5 s |
| `test_all_nan_pause` | PASS | — | UNKNOWN — no ACK within 5 s (COMMAND_LONG all-NaN "pause") |
| `test_command_long_nan_latlon` | PASS | — | UNKNOWN — no ACK within 5 s (COMMAND_LONG param5/6=NaN) |
| `test_command_long_int32max_float` | PASS | — | UNKNOWN — no ACK within 5 s (COMMAND_LONG param5/6=INT32_MAX as float) |

**ArduRover shows the same two outward behaviours as ArduCopter, but its handler (`GCS_MAVLINK_Rover::handle_command_int_do_reposition`, `Rover/GCS_MAVLink_Rover.cpp:509-545`) is structured differently in a way that matters for finding 2:**

1. **`set_desired_location()` fails for virtually every probe — not CHANGE_MODE-specific, same conclusion as ArduCopter but via a different code path.**  Rover's handler order is: mode-gating check (`DENIED` if not in guided mode and `!change_modes`) → `location_from_command_t` (`DENIED` if invalid) → **attempt the mode switch to `GUIDED` first** (`FAILED` if `set_mode()` fails) → `set_desired_speed()` if `param1 > 0` (`FAILED` if it fails) → `mode_guided.set_desired_location()` (`FAILED` if it fails) → `ACCEPTED`.  Because the suite's default `param2=1.0` (`CHANGE_MODE`) is carried by nearly every probe, `change_modes=true` for almost all of them, so (unlike on Copter, which never reaches the mode-switch call when the destination-set fails first) **Rover actually attempts — and very likely succeeds at — `set_mode(GUIDED)` on the first `CHANGE_MODE`-flagged probe that reaches it**, then fails at `set_desired_location()` and returns FAILED(4).  This produces the same three failing tests as ArduCopter (`test_accepted_change_mode`, `test_param2_change_mode_flag`, `test_param2_all_flags` all expect ACCEPTED but get FAILED), but on Rover **a real mode change to GUIDED plausibly does occur mid-sequence** — unlike on Copter, where the destination-set failure short-circuits the handler before any mode switch is attempted.  This real state transition is a stronger candidate trigger for finding 2 below than anything available on the Copter side.
2. **ACK responsiveness degrades mid-sequence, and earlier** — only the first 12 probes (through `test_param1_zero_speed`) receive a `result=4` (FAILED) ACK; from `test_param1_nan_speed` onward (19 consecutive probes, including all of the COMMAND_LONG variants), ArduRover **never sends another `COMMAND_ACK` for `DO_REPOSITION`** for the rest of the run.  The cut-over point is the *same test* (`test_param1_nan_speed`, the first NaN-valued `param1` probe) as on ArduCopter — on different vehicle firmware but at the identical trigger, which argues against pure coincidence.  Two candidate explanations now stand out from the source review: (a) a genuine state change — the `GUIDED`-mode switch traced in point 1 above (plus the resulting stream of `set_desired_location` FAILED responses and any associated failsafe/STATUSTEXT traffic) could plausibly alter how/whether Rover continues to ACK `DO_REPOSITION`; or (b) the `param1=NaN` encoding itself triggers a parsing or state fault common to both Copter and Rover (since Copter shows the identical cut-over despite never changing mode).  Source review of `GCS_Common.cpp` found no explicit command-level ACK-suppression mechanism in either vehicle's MAVLink layer; distinguishing (a) from (b) — and ruling out a test-harness-side `mavlink_direct`/gRPC artifact (see `[[grpc_cancel_pattern]]`) — needs runtime packet capture (`mavlogdump`/Wireshark) across the cut-over, not further source review.

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
| `DO_REPOSITION` with `CHANGE_MODE` set returns `MAV_RESULT_FAILED (4)` instead of `ACCEPTED (0)` + mode switch — traced to `set_destination()`/`set_desired_location()` failing inside `handle_command_int_do_reposition()` (`GCS_MAVLink_Copter.cpp:430`, `GCS_MAVLink_Rover.cpp:509`); not a `CHANGE_MODE`-specific code path — the suite's default `param2=1.0` simply routes nearly every probe through that failing call | ArduCopter, ArduRover | Spec violation — command is acknowledged but never executed; contrasts with PX4's patched ACCEPTED+AUTO_LOITER behaviour. Underlying `set_destination`/`set_desired_location` failure cause (geofence / EKF-origin / altitude-frame conversion candidates identified in source — see per-stack notes above) needs runtime log capture to confirm |
| `COMMAND_ACK` for `DO_REPOSITION` stops being sent partway through a probe sequence (observed cut-over at the *same* test, the first NaN-`param1` probe, on both vehicles; ArduCopter loses ACKs after ~14 probes, ArduRover after ~12) | ArduCopter, ArduRover | Observed only — no command-level ACK-suppression mechanism found in `GCS_Common.cpp` for either vehicle; spec requires every command to receive an ACK. Root cause needs runtime packet capture, not source review — see per-stack notes above |
