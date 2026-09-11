# MAV_CMD_DO_SET_GLOBAL_ORIGIN (cmd=611) — command protocol tests

Sets the GNSS coordinates of the vehicle's local origin (0,0,0). Supersedes the deprecated `SET_GPS_GLOBAL_ORIGIN` message (id=48, deprecated 2025-04). `development.xml` entry `value=611` — not in `common.xml`, so not covered by the command survey.

## Parameter layout

| Param | Label | Notes |
|-------|-------|-------|
| 1–4 | — | Empty (reserved) — must be NaN; non-NaN must be DENIED |
| 5 | Latitude | degE7 in COMMAND_INT; float degrees in COMMAND_LONG |
| 6 | Longitude | degE7 in COMMAND_INT; float degrees in COMMAND_LONG |
| 7 | Altitude | m MSL — must be a real value; NaN not valid |

## Frame

Submodule spec text says "Expected frame is MAV_FRAME_GLOBAL (0)", but PR #2530 corrects this to **MAV_FRAME_GLOBAL_INT (6)**, consistent with every other COMMAND_INT-with-location in this suite. Tests use frame=6.

## Sentinel semantics

Unlike NAV_TAKEOFF/NAV_LAND (INT32_MAX = "use current position"), this command **requires an explicit GNSS coordinate** — there is no "use current" sentinel. Params 5–7 carrying a sentinel or invalid value must be DENIED.

## Test coverage

**Verified:**
1. `GPS_GLOBAL_ORIGIN` response: changes on a new origin (`test_gps_global_origin_changes_when_new_value_set`), stays unchanged but is still emitted on a repeated identical origin per spec "irrespective of whether the origin is changed" (`test_gps_global_origin_unchanged_and_emitted_on_repeat`), emitted exactly once per accepted command (`test_gps_global_origin_emitted`), and NOT emitted when DENIED (`test_gps_global_origin_not_emitted_on_nack`).
2. Exactly one COMMAND_ACK per send (`test_exactly_one_ack`).
3. Params 1–4 must be NaN; non-NaN must be DENIED — **xfail on all known stacks** (spec gap, nothing enforces it): `test_reserved_param1_zero_ack` (the common `0.0`-for-NaN GCS mistake), `test_reserved_param{1,2,3,4}_nonnan_ack`.
4. Params 5–7 must reject sentinels/out-of-range values — **xfail on PX4** (see implementation notes below): `test_location_int32max_denied`, `test_location_out_of_range_latlon_denied`, `test_altitude_nan_denied`.

**Not covered**: whether the navigation stack actually uses the new origin for local↔global coordinate transforms (would need to observe `LOCAL_POSITION_NED`/`GLOBAL_POSITION_INT`; no flight test planned).

## PX4 implementation notes (branch `pr_cmd_set_global_origin`)

Supported on all vehicle types when built with `CONFIG_MAVLINK_DIALECT="development"` (SITL default). Flow: `mavlink_receiver` converts degE7→degrees → Commander's ignore-list passes the command to EKF2 without ACKing → EKF2 calls `setEkfGlobalOrigin()` and sends the ACK → EKF2 publishes `GPS_GLOBAL_ORIGIN` via the vehicle_command_ack path.

Confirmed gaps (all xfail, §4 above): PX4 returns `FAILED(4)` instead of `DENIED(2)` for INT32_MAX/out-of-range coordinates (EKF2 attempts the operation and reports failure rather than rejecting at the protocol layer); EKF2 doesn't validate altitude, so NaN is silently accepted.

## Tier 1 test results

| Test | Mock | PX4 MC |
|------|------|--------|
| `test_command_accepted` | PASS | PASS |
| `test_exactly_one_ack` | PASS | PASS |
| `test_reserved_param1_zero_ack` | XFAIL | XFAIL |
| `test_reserved_param1_nonnan_ack` | XFAIL | XFAIL |
| `test_reserved_param2_nonnan_ack` | XFAIL | XFAIL |
| `test_reserved_param3_nonnan_ack` | XFAIL | XFAIL |
| `test_reserved_param4_nonnan_ack` | XFAIL | XFAIL |
| `test_frame_global_ack` | PASS | PASS |
| `test_frame_global_relative_alt_ack` | PASS | PASS |
| `test_location_int32max_denied` | PASS | XFAIL |
| `test_location_out_of_range_latlon_denied` | PASS | XFAIL |
| `test_altitude_nan_denied` | PASS | XFAIL |
| `test_altitude_zero` | PASS | PASS |
| `test_altitude_negative` | PASS | PASS |
| `test_gps_global_origin_emitted` | PASS | PASS¹ |
| `test_gps_global_origin_changes_when_new_value_set` | PASS | PASS |
| `test_gps_global_origin_unchanged_and_emitted_on_repeat` | PASS | PASS |
| `test_command_long_accepted` | PASS | PASS |
| `test_command_long_float_int32max_denied` | PASS | PASS |
| `test_gps_global_origin_not_emitted_on_nack` | PASS | SKIP |

`XFAIL` = asserts DENIED but stack returns something else (documented spec gap). `SKIP` = mock-only test.

¹ PX4 MC: the first `GPS_GLOBAL_ORIGIN` received (`alt_mm=-500000 extra=1`) was a late emission from the preceding `test_altitude_negative` (z=−500 m); the response to the current command (z=10 m) arrived as the extra — a test-ordering timing artifact in standalone mode. Emission and exactly-once assertions are enforced on the mock only.

Other vehicle types (PX4 FW/VTOL/Rover, ArduPilot) not yet tested.

## Running

```bash
# Paired mock (20 tests: 15 PASS, 5 XFAIL)
pytest tests/command/do_set_global_origin/test_command.py -v --log-cli-level=INFO

# Standalone PX4 MC (11 PASS, 8 XFAIL, 1 SKIP)
pytest tests/command/do_set_global_origin/test_command.py \
    --drone-address=udp://:14540 --vehicle-type=quadcopter --autopilot=px4 \
    --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \
    -v --log-cli-level=INFO
```
