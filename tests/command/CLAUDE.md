# Command protocol — implementation notes

See `tests/command/README.md` for survey result tables and per-stack test results.

## COMMAND_INT vs COMMAND_LONG selection rules (per MAVLink spec)

- **COMMAND_INT**: required for commands where params 5/6 carry lat/lon (integer ×1e7
  in `x`/`y` fields).  Preserves coordinate precision.  Explicitly required for commands
  with `hasLocation="true"` or `isDestination="true"` in common.xml.
- **COMMAND_LONG**: required when params 5/6 carry non-integer float values (e.g. speed,
  duration, camera ID).  All 7 params are floats; coordinate precision is lost for lat/lon.
- NAV_TAKEOFF → **COMMAND_INT** (hasLocation + isDestination).

## MAV_RESULT values

| Value | Name | Meaning |
|-------|------|---------|
| 0 | ACCEPTED | Command accepted, executing |
| 1 | TEMPORARILY_REJECTED | Temporarily rejected (stack busy); retry |
| 2 | DENIED | Refused (params or state issue) |
| 3 | UNSUPPORTED | Command not known to this stack |
| 4 | FAILED | Attempted but failed |
| 5 | IN_PROGRESS | Still executing; more ACKs will follow |
| 6 | CANCELLED | Was executing; now cancelled |
| 7 | COMMAND_LONG_ONLY | Must use COMMAND_LONG, not COMMAND_INT |
| 8 | COMMAND_INT_ONLY | Must use COMMAND_INT, not COMMAND_LONG |

Values 0–8 are defined in MAVLink master common.xml.  CANCELLED (6) is absent from
pymavlink 2.4.49 — always use the MAVLink submodule XML as the authoritative source.

## Command protocol flow

1. GCS sends COMMAND_INT (or COMMAND_LONG).
2. Stack sends COMMAND_ACK with MAV_RESULT.
3. If no ACK within timeout (default 5s): retransmit.  COMMAND_INT retransmissions are
   identical (no confirmation field).  COMMAND_LONG increments `confirmation` (0, 1, 2…).
4. After max retries with no ACK: treat as UNKNOWN (not UNSUPPORTED).

## No-ACK policy

Per the MAVLink spec, every command must receive a COMMAND_ACK.  Not sending one is a
spec violation.  However, a missing ACK is ambiguous:
- The stack may be too busy / dropped the message (transient, retry appropriate).
- The stack received it but chose not to ACK (spec violation, but command may be executing).
- The stack does not recognise the command (truly unsupported).

**Policy**: treat "no ACK within timeout" as result `UNKNOWN`, never `UNSUPPORTED`.
Log at WARNING level: "No COMMAND_ACK received within Xs — command may be executing
without acknowledgement (spec violation) or truly unsupported; further testing required."

## NaN in float fields

Pass `None` (Python) in `fields_json` to encode NaN on the wire.

**How it works**: `json.dumps(None)` → `"null"`; nlohmann/json (MAVSDK C++ gRPC bridge)
decodes a JSON `null` in a float field as IEEE-754 NaN; pymavlink encodes the received MAVLink
binary back to JSON as `null`; Python receives `None`.  The round-trip is: `None` → `null` →
NaN → `null` → `None`.

**Note**: `json.dumps(float('nan'))` produces `'NaN'` (non-standard JSON) which nlohmann/json
rejects as `INVALID_FIELD`.  Always use `None`, never `float('nan')`, in `fields_json`.

The MAVLink spec permits NaN for unused params and some defined params (e.g. param4=NaN
means "use current heading").

## Command vs mission protocol differences

The same MAV_CMD may behave differently in the two paths:
- **Mission protocol** (MISSION_ITEM_INT upload): parameters are *stored* and used during
  mission execution.  The autopilot may normalise them on storage.
- **Command protocol** (COMMAND_INT direct): parameters are used immediately.  The
  autopilot may ignore parameters it doesn't act on in real time.

Known example — NAV_TAKEOFF param4 (Yaw):
- **PX4 mission**: param4 stored; used to set heading after takeoff.
- **PX4 COMMAND_INT**: `rep->current.yaw = NAN` regardless of param4 (`navigator_main.cpp:630`).
- **ArduPilot mission**: param4 NOT stored (only param1 stored).
- **ArduPilot COMMAND_INT**: `// param4 : yaw angle   (not supported)` (`GCS_MAVLink_Copter.cpp:585`).

## MAVLink XML submodule

`mavlink/message_definitions/v1.0/common.xml` contains 168 MAV_CMD entries.
common.xml includes standard.xml which includes minimal.xml (common is the full superset).

```bash
# Initialise submodule (one-time after cloning)
git submodule update --init mavlink

# Use alternate definitions dir
pytest tests/command/ --mavlink-definitions-dir=/path/to/other/message_definitions/v1.0
```

Pymavlink 2.4.49 bundles only 155 commands and is missing `MAV_RESULT_CANCELLED = 6`.
Always use the submodule XML for the command survey and authoritative command/result lookups.

## Autopilot-specific behaviour (PX4 MC)

Tested against PX4 1.18.0-alpha, SIH simulator (sihsim_quadx), connected via `udp://:14540`.
Log: `logs/command_survey_px4_quadcopter_1.18.0-alpha_20260527_075001.log`.
Results: **36 SUPPORTED, 105 UNSUPPORTED, 27 UNKNOWN** (command survey).

### Key behaviours

- **NAV_TAKEOFF (cmd=22) is SUPPORTED** — `MAV_RESULT_ACCEPTED`.  All 9 takeoff command tests PASS.
- **NAV_VTOL_TAKEOFF (cmd=84) is SUPPORTED** — PX4 multicopter accepts VTOL_TAKEOFF.
- **NAV_VTOL_LAND (cmd=85) is UNSUPPORTED** on multicopter (VTOL_LAND requires a VTOL-capable vehicle in PX4).
- **NAV_LOITER_UNLIM (17), CONDITION_YAW (115), DO_REPOSITION (192)** are all UNSUPPORTED via COMMAND_INT (PX4 uses different execution paths for these).
- **27 UNKNOWN** commands: mostly camera/gimbal commands that PX4 does not ACK via COMMAND_INT within 2s.  These may be handled by companion computers or camera managers.
- **DO_FIGURE_EIGHT (35) is UNKNOWN** (no ACK within 2s) on multicopter; SUPPORTED on FW and VTOL.

## Autopilot-specific behaviour (PX4 FW)

Tested against PX4 1.18.0-alpha, SIH simulator (sihsim_airplane), connected via `udp://:14540`.
Log: `logs/command_survey_px4_fixed_wing_1.18.0-alpha_20260527_075139.log`.
Results: **37 SUPPORTED, 105 UNSUPPORTED, 26 UNKNOWN** (command survey).

### Key behaviours

- Identical to PX4 MC except: **DO_FIGURE_EIGHT (cmd=35)** is SUPPORTED (FW manoeuvre; MC gives no ACK).
- All 9 NAV_TAKEOFF command tests PASS (result=0 ACCEPTED).

## Autopilot-specific behaviour (PX4 VTOL)

Tested against PX4 1.18.0-alpha, SIH simulator (sihsim_standard_vtol), connected via `udp://:14540`.
Log: `logs/command_survey_px4_vtol_1.18.0-alpha_20260527_075317.log`.
Results: **38 SUPPORTED, 105 UNSUPPORTED, 25 UNKNOWN** (command survey).

### Key behaviours

- Identical to PX4 MC except: **DO_FIGURE_EIGHT (cmd=35)** and **DO_VTOL_TRANSITION (cmd=3000)** are both SUPPORTED (VTOL-capable vehicle).
- All 9 NAV_TAKEOFF command tests PASS (result=0 ACCEPTED).

## Autopilot-specific behaviour (PX4 Rover)

Tested against PX4 1.18.0-alpha, SIH simulator (sihsim_rover_ackermann), connected via `udp://:14540`.
Log: `logs/command_survey_px4_rover_1.18.0-alpha_20260527_075459.log`.
Results: **35 SUPPORTED, 106 UNSUPPORTED, 27 UNKNOWN** (command survey).

### Key behaviours

- Identical to PX4 MC except: **DO_AUTOTUNE_ENABLE (cmd=212)** is UNSUPPORTED (no flight-controller PID to tune on a rover).
- **NAV_TAKEOFF (cmd=22) is SUPPORTED** (result=0 ACCEPTED) — PX4 does not gate commands by vehicle type.  This contrasts with ArduRover which returns UNSUPPORTED.  All 9 NAV_TAKEOFF command tests PASS.

## Autopilot-specific behaviour (ArduRover)

Tested against ArduRover V4.8.0-dev (70fe7125, `--model rover`) connected via TCP port 5760.
Log: `logs/command_survey_ardupilot_rover_4.8.0-dev_20260526_212652.log`.
Results: **49 SUPPORTED, 118 UNSUPPORTED, 1 UNKNOWN** (command survey).

### Key behaviours

- **NAV_TAKEOFF (cmd=22) is UNSUPPORTED** — `MAV_RESULT_UNSUPPORTED (3)`.  ArduRover is a ground vehicle; the 3 command tests that assert `result != UNSUPPORTED` fail by design.  Observational tests still pass.

## ArduCopter mode restriction for NAV_TAKEOFF

`GCS_MAVLink_Copter.cpp:578` checks `has_user_takeoff(must_navigate)` before executing NAV_TAKEOFF.  With default param3=0 (`must_navigate=true`):

| Mode | Number | Accepts NAV_TAKEOFF | Autonomous climb (no RC) |
|------|--------|---------------------|--------------------------|
| STABILIZE | 0 | ❌ | — |
| ALT_HOLD | 2 | ❌ | — |
| **GUIDED** | **4** | **✅** | **✅ uses `_AutoTakeoff::run()`** |
| LOITER | 5 | ✅ | ❌ uses pilot controller |
| POSHOLD | 16 | ✅ | ❌ uses pilot controller |

Only GUIDED overrides `do_user_takeoff_start_m()` to use the autonomous
`_AutoTakeoff::run()` controller.  LOITER/POSHOLD accept the command but use
`_TakeOff::do_pilot_takeoff_ms()` which reads the RC throttle channel — no
autonomous climb without RC input.  All ArduPilot autotest calls to
`user_takeoff()` are preceded by `change_mode("GUIDED")`.

Additional ArduPilot MAVSDK quirks (see `tests/command/baseline_takeoff/README.md`):
- MAVSDK reports ArduCopter GUIDED (custom_mode=4) as `"OFFBOARD"` in `telemetry.flight_mode()`
- MAVSDK reports ArduPlane GUIDED (custom_mode=15) as `"GUIDED"` or `"OFFBOARD"`
- ArduCopter/ArduPlane does not stream `GLOBAL_POSITION_INT` by default — must send
  `MAV_CMD_SET_MESSAGE_INTERVAL (511)` with `param1=33` first

## Autopilot-specific behaviour (ArduRover, continued)

### Key behaviours (ArduRover)

- **DO_FLIGHTTERMINATION (cmd=185) is UNSUPPORTED** on rover (vs SUPPORTED on copter/plane).
- **DO_REPOSITION (cmd=192), DO_FENCE_ENABLE (207), DO_SET_MISSION_CURRENT (224), MISSION_START (300)** are all SUPPORTED — rover has full mission-management capability.
- **Broader DO_ coverage** than ArduPlane: 49 SUPPORTED commands including camera, relay, servo, gimbal, and logging commands that ArduPlane doesn't respond to.
