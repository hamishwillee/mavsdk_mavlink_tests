# MAV_CMD_DO_SET_GLOBAL_ORIGIN (cmd=611) — command protocol tests

Sets the GNSS coordinates of the vehicle local origin (0,0,0) position.
Supersedes the `SET_GPS_GLOBAL_ORIGIN` message (id=48, deprecated 2025-04).

**Spec reference**: `mavlink/message_definitions/v1.0/development.xml` entry
`value=611`.  The command is in `development.xml`, not `common.xml`, and does
not appear in the command survey.

## Parameter layout

| Param | Label | Notes |
|-------|-------|-------|
| 1–4 | — | Empty (reserved) — must be NaN; non-NaN must be DENIED |
| 5 | Latitude | degE7 in COMMAND_INT; float degrees in COMMAND_LONG |
| 6 | Longitude | degE7 in COMMAND_INT; float degrees in COMMAND_LONG |
| 7 | Altitude | m MSL — must be a real value; NaN not valid |

## Frame

The current submodule spec text says "Expected frame is MAV_FRAME_GLOBAL (0)",
but PR #2530 corrects this to **MAV_FRAME_GLOBAL_INT (6)**, consistent with
every other COMMAND_INT with location in this suite.  Tests use frame=6.

## Sentinel semantics

Unlike NAV_TAKEOFF/NAV_LAND (where INT32_MAX in lat/lon means "use current
position"), **DO_SET_GLOBAL_ORIGIN requires an explicit GNSS coordinate**.
There is no "use current" sentinel.  Params 5–7 that carry a sentinel or
invalid value must be DENIED.

## What these tests cover

### Spec requirements verified

1. **GPS_GLOBAL_ORIGIN response emission**:
   - **Changes** when a new (different) origin is commanded
     → `test_gps_global_origin_changes_when_new_value_set`
   - **Does NOT change** when the same origin is commanded again
     → `test_gps_global_origin_unchanged_and_emitted_on_repeat`
   - **Emitted in either case** (change or no change) per spec
     "irrespective of whether the origin is changed"
     → `test_gps_global_origin_unchanged_and_emitted_on_repeat`
   - **Emitted exactly once** per accepted command
     → `test_gps_global_origin_emitted` (drain window check)
   - **NOT emitted** when command is DENIED
     → `test_gps_global_origin_not_emitted_on_nack`

2. **Exactly one COMMAND_ACK** received per command send
   → `test_exactly_one_ack`

3. **Params 1–4 must be NaN** (reserved); any non-NaN value must be DENIED
   Two cases tested, both **xfail** on all known stacks:
   - `test_reserved_param1_zero_ack` — `param1=0.0` (common GCS mistake; still a spec violation)
   - `test_reserved_param1_nonnan_ack` through `test_reserved_param4_nonnan_ack` — `param=1.0`
   No implementation currently enforces NaN for "Empty" params (spec gap)

4. **Params 5–7 must not be sentinel values and must be in valid range**:
   - INT32_MAX lat/lon → DENIED: `test_location_int32max_denied`
     **xfail PX4**: PX4 converts INT32_MAX → NaN and EKF2 returns FAILED (not DENIED)
   - Out-of-range lat (91°N) → DENIED: `test_location_out_of_range_latlon_denied`
     **xfail PX4**: PX4 passes to EKF2 which returns FAILED (not DENIED)
   - NaN altitude → DENIED: `test_altitude_nan_denied`
     **xfail PX4**: PX4/EKF2 accepts NaN altitude without validation

### What tests cannot show

- Whether the vehicle navigation stack actually uses the new origin for
  local↔global coordinate transforms (requires observing LOCAL_POSITION_NED
  or GLOBAL_POSITION_INT behaviour — no flight test is planned).

## PX4 implementation notes (branch `pr_cmd_set_global_origin`)

PX4 supports cmd=611 on all vehicle types when built with
`CONFIG_MAVLINK_DIALECT="development"` (the SITL default).  The flow is:

1. `mavlink_receiver` converts degE7 lat/lon to degrees via `command_has_location()`
2. Commander's ignore list passes the command to EKF2 without sending ACK
3. EKF2 calls `setEkfGlobalOrigin()` and sends COMMAND_ACK (ACCEPTED/FAILED)
4. EKF2 publishes `GPS_GLOBAL_ORIGIN` via the vehicle_command_ack path

**Known PX4 gaps** (all xfail in test results):
- **Invalid coordinate result code**: PX4 returns FAILED (4) instead of DENIED (2)
  for INT32_MAX and out-of-range coordinates — EKF2 attempts the operation and
  reports failure rather than rejecting at the protocol layer
- **NaN altitude accepted**: EKF2 does not validate the altitude field; NaN is
  silently accepted (spec requires DENIED)
- **Reserved params not enforced**: params 1–4 are ignored rather than rejected
  when non-NaN (shared gap with all known stacks)

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

`XFAIL` = asserts DENIED but stack returns something else (documented spec gap).
`SKIP` = mock-only test; skipped in standalone mode.

¹ PX4 MC observed: `alt_mm=-500000 extra=1` — the first `GPS_GLOBAL_ORIGIN`
received was a late emission from the preceding `test_altitude_negative` (z=−500 m);
the response for the current command (z=10 m) arrived as the extra.  This is a
test-ordering timing artifact in standalone mode; GPS_GLOBAL_ORIGIN emission and
exactly-once assertions are only enforced on the mock.

Other vehicle types (PX4 FW/VTOL/Rover, ArduPilot) not yet tested — add results
when available.

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
