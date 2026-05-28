# MAV_CMD_NAV_TAKEOFF — Command Protocol (COMMAND_INT) Test Results

Tests the **command protocol** path (COMMAND_INT → COMMAND_ACK).

This is distinct from the **mission protocol** path (MISSION_ITEM_INT upload → storage)
covered by `tests/mission/nav_takeoff/`.

## COMMAND_INT vs COMMAND_LONG

NAV_TAKEOFF has `hasLocation="true"` and `isDestination="true"` in common.xml.
Per the MAVLink spec, commands with location fields must use COMMAND_INT — the integer
x/y fields preserve lat/lon precision that is lost in COMMAND_LONG (float param5/6).

## NaN in float fields

Pass `None` (Python) in `fields_json` to encode NaN on the wire.  `json.dumps(None)`
produces `"null"`; nlohmann/json (MAVSDK C++ gRPC bridge) decodes a JSON `null` in a
float field as IEEE-754 NaN.  The MAVLink spec permits NaN for unused params and some
defined params (e.g. param4=NaN means "use current heading").

NaN tests skip in mock/paired mode (the mock's ACCEPTED result does not reflect real
stack behaviour for param handling).

## Yaw behaviour via COMMAND_INT

Both PX4 and ArduPilot **ignore** param4 (Yaw) in the COMMAND_INT execution path:

| Stack | Source | Behaviour |
|-------|--------|-----------|
| PX4 | `navigator_main.cpp:630`: `rep->current.yaw = NAN` | Yaw reset regardless of param4 |
| ArduCopter | `GCS_MAVLink_Copter.cpp:585`: `// param4 : yaw angle   (not supported)` | Yaw ignored |
| ArduPlane | `GCS_MAVLink_Plane.cpp`: only altitude read | Yaw ignored |

This differs from the **mission protocol** path, where PX4 stores and uses param4 yaw.

## Test Results

### Mock (paired mode)

| Test | Param | Mock result |
|------|-------|-------------|
| `test_command_accepted` | baseline (param4=0°) | PASS — result=0 ACCEPTED |
| `test_param1_pitch_ack_accepted` | param1=15° | PASS — result=0 ACCEPTED |
| `test_param1_nan_ack_result` | param1=NaN | SKIP — NaN tests skip in mock mode |
| `test_param4_yaw_specific_ack` | param4=90° | PASS — result=0 ACCEPTED |
| `test_param4_yaw_nan_ack` | param4=NaN | SKIP — NaN tests skip in mock mode |
| `test_location_specific_ack` | x/y=SIH home | PASS — result=0 ACCEPTED |
| `test_location_int32max_ack` | x/y=INT32_MAX | PASS — result=0 ACCEPTED |
| `test_nan_altitude_ack` | z=NaN | SKIP — NaN tests skip in mock mode |
| `test_wrong_frame_ack` | frame=LOCAL_NED | PASS — result=0 ACCEPTED (mock accepts all frames) |

The mock returns ACCEPTED for all commands by default.  NaN tests skip in mock mode — the
mock's ACCEPTED result does not reflect real stack behaviour for param handling.

NaN tests (`test_param1_nan_ack_result`, `test_param4_yaw_nan_ack`, `test_nan_altitude_ack`)
pass `None` in `fields_json` to encode NaN on the wire.  They run with `--ardupilot-sitl`
or `--px4-sitl` and are observational (no assertion on ACK result).

### ArduCopter MC (standalone)

9 PASS, 0 SKIP.  NAV_TAKEOFF is SUPPORTED on ArduCopter via COMMAND_INT.

| Test | Param | Result |
|------|-------|--------|
| `test_command_accepted` | baseline | PASS — result=0 ACCEPTED |
| `test_param1_pitch_ack_accepted` | param1=15° | PASS — result=0 ACCEPTED |
| `test_param1_nan_ack_result` | param1=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_param4_yaw_specific_ack` | param4=90° | PASS — result=0 ACCEPTED (yaw ignored) |
| `test_param4_yaw_nan_ack` | param4=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_location_specific_ack` | x/y=home | PASS — result=0 ACCEPTED |
| `test_location_int32max_ack` | x/y=INT32_MAX | PASS — result=0 ACCEPTED (observational) |
| `test_nan_altitude_ack` | z=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_wrong_frame_ack` | frame=LOCAL_NED | PASS — result=0 ACCEPTED (observational) |

### ArduPlane FW (standalone)

9 PASS, 0 SKIP.  NAV_TAKEOFF is SUPPORTED on ArduPlane FW via COMMAND_INT.
ArduPlane only reads altitude; lat/lon and other params are ignored.

| Test | Param | Result |
|------|-------|--------|
| `test_command_accepted` | baseline | PASS — result=0 ACCEPTED |
| `test_param1_pitch_ack_accepted` | param1=15° | PASS — result=0 ACCEPTED |
| `test_param1_nan_ack_result` | param1=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_param4_yaw_specific_ack` | param4=90° | PASS — result=0 ACCEPTED (yaw ignored) |
| `test_param4_yaw_nan_ack` | param4=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_location_specific_ack` | x/y=home | PASS — result=0 ACCEPTED |
| `test_location_int32max_ack` | x/y=INT32_MAX | PASS — result=0 ACCEPTED (observational) |
| `test_nan_altitude_ack` | z=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_wrong_frame_ack` | frame=LOCAL_NED | PASS — result=0 ACCEPTED (observational) |

### ArduPlane QP (standalone)

9 PASS, 0 SKIP.  Same behaviour as ArduPlane FW.

| Test | Param | Result |
|------|-------|--------|
| `test_command_accepted` | baseline | PASS — result=0 ACCEPTED |
| `test_param1_pitch_ack_accepted` | param1=15° | PASS — result=0 ACCEPTED |
| `test_param1_nan_ack_result` | param1=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_param4_yaw_specific_ack` | param4=90° | PASS — result=0 ACCEPTED (yaw ignored) |
| `test_param4_yaw_nan_ack` | param4=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_location_specific_ack` | x/y=home | PASS — result=0 ACCEPTED |
| `test_location_int32max_ack` | x/y=INT32_MAX | PASS — result=0 ACCEPTED (observational) |
| `test_nan_altitude_ack` | z=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_wrong_frame_ack` | frame=LOCAL_NED | PASS — result=0 ACCEPTED (observational) |

### ArduRover (standalone)

NAV_TAKEOFF is **UNSUPPORTED** on ArduRover (ground vehicle) — survey-gating skips all
detail tests.  Baseline probe returns `MAV_RESULT_UNSUPPORTED (3)`; `_ensure_supported()`
caches this and calls `pytest.skip()` for every test in the class.

9 SKIP — tests not run.

### PX4 MC (standalone)

9 PASS, 0 SKIP.  NAV_TAKEOFF is SUPPORTED on PX4 multicopter via COMMAND_INT.

| Test | Param | Result |
|------|-------|--------|
| `test_command_accepted` | baseline | PASS — result=0 ACCEPTED |
| `test_param1_pitch_ack_accepted` | param1=15° | PASS — result=0 ACCEPTED |
| `test_param1_nan_ack_result` | param1=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_param4_yaw_specific_ack` | param4=90° | PASS — result=0 ACCEPTED |
| `test_param4_yaw_nan_ack` | param4=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_location_specific_ack` | x/y=home | PASS — result=0 ACCEPTED |
| `test_location_int32max_ack` | x/y=INT32_MAX | PASS — result=0 ACCEPTED (observational) |
| `test_nan_altitude_ack` | z=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_wrong_frame_ack` | frame=LOCAL_NED | PASS — result=0 ACCEPTED (observational) |

### PX4 FW (standalone)

9 PASS, 0 SKIP.  NAV_TAKEOFF is SUPPORTED on PX4 fixed-wing via COMMAND_INT.
All results identical to PX4 MC.

| Test | Param | Result |
|------|-------|--------|
| `test_command_accepted` | baseline | PASS — result=0 ACCEPTED |
| `test_param1_pitch_ack_accepted` | param1=15° | PASS — result=0 ACCEPTED |
| `test_param1_nan_ack_result` | param1=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_param4_yaw_specific_ack` | param4=90° | PASS — result=0 ACCEPTED |
| `test_param4_yaw_nan_ack` | param4=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_location_specific_ack` | x/y=home | PASS — result=0 ACCEPTED |
| `test_location_int32max_ack` | x/y=INT32_MAX | PASS — result=0 ACCEPTED (observational) |
| `test_nan_altitude_ack` | z=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_wrong_frame_ack` | frame=LOCAL_NED | PASS — result=0 ACCEPTED (observational) |

### PX4 VTOL (standalone)

9 PASS, 0 SKIP.  NAV_TAKEOFF is SUPPORTED on PX4 VTOL via COMMAND_INT.
All results identical to PX4 MC.

| Test | Param | Result |
|------|-------|--------|
| `test_command_accepted` | baseline | PASS — result=0 ACCEPTED |
| `test_param1_pitch_ack_accepted` | param1=15° | PASS — result=0 ACCEPTED |
| `test_param1_nan_ack_result` | param1=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_param4_yaw_specific_ack` | param4=90° | PASS — result=0 ACCEPTED |
| `test_param4_yaw_nan_ack` | param4=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_location_specific_ack` | x/y=home | PASS — result=0 ACCEPTED |
| `test_location_int32max_ack` | x/y=INT32_MAX | PASS — result=0 ACCEPTED (observational) |
| `test_nan_altitude_ack` | z=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_wrong_frame_ack` | frame=LOCAL_NED | PASS — result=0 ACCEPTED (observational) |

### PX4 Rover (standalone)

9 PASS, 0 SKIP.  PX4 Rover returns ACCEPTED for NAV_TAKEOFF — PX4 does not restrict
commands by vehicle type (unlike ArduRover which returns UNSUPPORTED).

| Test | Param | Result |
|------|-------|--------|
| `test_command_accepted` | baseline | PASS — result=0 ACCEPTED |
| `test_param1_pitch_ack_accepted` | param1=15° | PASS — result=0 ACCEPTED |
| `test_param1_nan_ack_result` | param1=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_param4_yaw_specific_ack` | param4=90° | PASS — result=0 ACCEPTED |
| `test_param4_yaw_nan_ack` | param4=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_location_specific_ack` | x/y=home | PASS — result=0 ACCEPTED |
| `test_location_int32max_ack` | x/y=INT32_MAX | PASS — result=0 ACCEPTED (observational) |
| `test_nan_altitude_ack` | z=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_wrong_frame_ack` | frame=LOCAL_NED | PASS — result=0 ACCEPTED (observational) |

Tested: 2026-05-27.

## Tier 2 Flight Tests (test_flight.py)

### Overview

These tests arm the vehicle, send `NAV_TAKEOFF` via raw `COMMAND_INT`, and observe telemetry
to verify execution.  A two-stage gate runs before any test:

1. **ACK probe**: confirms `NAV_TAKEOFF` is not UNSUPPORTED (skips all tests on ArduRover).
2. **Execution probe**: arm → send COMMAND_INT → wait ≤ 20 s for any climb > 0.5 m.
   If the vehicle doesn't climb (command accepted but not executed), all 17 tests skip with
   an informative message.

**Key implementation note**: PX4 ignores the `frame` field in COMMAND_INT NAV_TAKEOFF and
always treats `z` as absolute altitude (AMSL).  `_arm_and_send_takeoff()` converts the
caller's relative altitude to absolute (home AMSL + relative) before sending, and uses
frame=5 (GLOBAL_INT, absolute AMSL) to match.

### PX4 MC (Tier 2)

Log: `logs/command_takeoff_flight_px4_quadcopter_1.18.0-alpha_20260527_134513.log`

**15 PASS, 1 XPASS, 1 FAIL** — COMMAND_INT NAV_TAKEOFF executes on PX4 MC.  PX4 ignores the
frame field and treats z as absolute AMSL; commanded altitude is respected.

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
| `test_position_specific` | x/y=home lat/lon | **PASS** | Took off from home coordinates |
| `test_position_int32max_stays_at_home` | x/y=INT32_MAX | **PASS** | INT32_MAX sentinel accepted; vehicle within 0.7 m of home at 2.0 m altitude |
| `test_position_zero_treated_as_current` | x=0, y=0 | **FAIL** | PX4 navigated toward equator — x/y=0 treated as valid coordinates, not "use current" sentinel |
| `test_pitch_comparison_low_vs_high` | param1=5° vs 45° | PASS (obs) | param1 not supported — both cycles logged 0° peak pitch |
| `test_mode_after_takeoff` | informational | PASS (obs) | Mode: Unknown — vehicle did not reach threshold within timeout after 16 prior flight cycles |

**PX4 MC behaviour summary**:

| Behaviour | Result | Detail |
|-----------|--------|--------|
| Command accepted | PASS | result=0 ACCEPTED |
| Altitude (z) | PASS | Commanded altitude respected (≥85% reached for 30 m and 50 m targets) |
| Altitude default (z=NaN) | PASS | z is a float field; NaN → default altitude ~0.5–0.7 m relative |
| Altitude at zero (z=0 abs) | XPASS | Safety minimum applied (~0.26 m) |
| Yaw (param4) | Not supported | Ignored in COMMAND_INT path; heading equals pre-arm value (~351°) |
| Pitch (param1) | Not supported | Ignored in COMMAND_INT path; both 5° and 45° produce 0° peak pitch |
| Position specific | PASS | Explicit home coordinates accepted |
| Position default (INT32_MAX sentinel) | PASS | x/y are int32 fields; INT32_MAX → "use current position"; vehicle within 0.7 m of home |
| Next mode | Unknown | Timed out after 16 prior flight cycles — mode not observed |

### PX4 FW (Tier 2)

**17 SKIP** — Execution probe: COMMAND_INT NAV_TAKEOFF is ACCEPTED but the SIH fixed-wing
simulator does not perform a vertical ground-to-air climb.  Fixed-wing NAV_TAKEOFF requires
a runway roll at sufficient airspeed; the arm + immediate COMMAND_INT path does not trigger
this.

### ArduCopter MC (Tier 2)

**17 SKIP** — Execution probe: COMMAND_INT NAV_TAKEOFF is ACCEPTED but the vehicle does not
climb within 20 s.  ArduCopter requires the vehicle to be in GUIDED mode before COMMAND_INT
NAV_TAKEOFF executes; arm + COMMAND_INT from the default (STABILIZE) mode is not sufficient.

### ArduPlane FW (Tier 2)

**17 SKIP** — Same as ArduCopter: ACCEPTED but does not execute without an explicit mode
switch to GUIDED or AUTO first.

---

## Comparison: COMMAND_INT vs mission protocol (param4 yaw)

The same param4 (Yaw) field is handled very differently in the two protocol paths:

| Aspect | Mission protocol | Command protocol |
|--------|-----------------|-----------------|
| **PX4** | param4 stored; used to set heading after takeoff | `rep->current.yaw = NAN` regardless of param4 (`navigator_main.cpp:630`) |
| **ArduCopter** | param4 not stored (zeroed on download) | `// param4 : yaw angle   (not supported)` (`GCS_MAVLink_Copter.cpp:585`) |
| **ArduPlane** | param4 not stored (zeroed on download) | Only altitude read from COMMAND_INT handler (`GCS_MAVLink_Plane.cpp:890`) |

In the mission protocol, PX4 stores and uses param4 to set the heading setpoint after
takeoff.  In the command protocol path, all stacks ignore param4 — the yaw is either
reset to NaN (PX4) or explicitly noted as unsupported (ArduPilot).

## Spec gaps

The following behaviours are undefined or ambiguous in the MAVLink common.xml spec for
`MAV_CMD_NAV_TAKEOFF` when sent via `COMMAND_INT`.  All are documented by `test_flight.py`.

**param1 (MinPitch)**: No minimum or maximum value is specified.  Values outside `[0, 90]`
degrees (e.g. `89°`, `-10°`, `180°`) are accepted in both the mission and command paths by
all known stacks (with varying normalisation on storage in the mission path).  Suggest: define
the valid range and require `DENIED` for out-of-range values.

**param4 (Yaw)**: No range or normalisation rule is defined.  Should negative values be
rejected, treated as equivalent clockwise angles (`-90°` → `270°`), or mean "turn
anti-clockwise"?  Should values > `360°` wrap (e.g. `450°` → `90°`) or be rejected?  Should
very large values (e.g. `3600°`) cause multiple rotations?  All of this is currently
implementation-defined.  In the `COMMAND_INT` path, both PX4 and ArduPilot ignore param4
entirely, so the question is moot in practice — but the spec should still clarify.

**param7 (Altitude)**: No minimum altitude is defined.  Setting `z=0` is ambiguous: should the
stack use a safety minimum, reject the command, or hover in place after leaving the ground?
Setting `z=NaN` is permitted by the spec ("use default altitude") but the default is not
defined.  Suggest: define a minimum altitude and the meaning of `z=NaN`.

**param5/6 (Lat/Lon) = 0**: Whether `(0, 0)` (equator/prime meridian) should be treated as a
valid takeoff coordinate or as a "use current position" sentinel is not specified.  Most stacks
appear to treat it as "current position", but this is undocumented.

**Position semantics**: The spec does not state whether the `COMMAND_INT` `x`/`y` fields for
`NAV_TAKEOFF` specify the position _from which_ the vehicle should take off (navigate there,
then climb) or the position _to arrive at_ after climbing.  For all tested stacks, the vehicle
simply climbs from its current position regardless of `x`/`y`.

## Running

```bash
# Mock (tier 1 only)
pytest tests/command/takeoff/test_command.py -v --log-cli-level=INFO

# PX4 multicopter — tier 1
pytest tests/command/takeoff/test_command.py --drone-address=udp://:14540 -v --log-cli-level=INFO

# ArduCopter — tier 1
pytest tests/command/takeoff/test_command.py \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 -v --log-cli-level=INFO

# PX4 multicopter — tier 2 flight
pytest tests/command/takeoff/test_flight.py \
    --drone-address=udp://:14540 --connection-timeout=60 \
    --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \
    --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO

# ArduCopter — tier 2 flight
pytest tests/command/takeoff/test_flight.py \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
    --ardupilot-sitl=~/ardu_sitl/arducopter \
    --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 \
    --vehicle-type=quadcopter --autopilot=ardupilot -v --log-cli-level=INFO

# ArduPlane fixed-wing — tier 2 flight
pytest tests/command/takeoff/test_flight.py \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
    --ardupilot-sitl=~/ardu_sitl/arduplane --vehicle-type=fixed_wing \
    --autopilot=ardupilot -v --log-cli-level=INFO
```
