# Command protocol — implementation notes

See `tests/command/README.md` for survey result tables and per-stack test results.

## COMMAND_INT vs COMMAND_LONG selection rules (per MAVLink spec)

- **COMMAND_INT**: required for commands where params 5/6 carry lat/lon (integer ×1e7 in `x`/`y` fields).
  Preserves coordinate precision.
  Explicitly required for commands with `hasLocation="true"` or `isDestination="true"` in common.xml.
- **COMMAND_LONG**: required when params 5/6 carry non-integer float values (e.g. speed, duration, camera ID).
  All 7 params are floats; coordinate precision is lost for lat/lon.
- NAV_TAKEOFF → **COMMAND_INT** (hasLocation + isDestination).
- NAV_LAND → **COMMAND_INT** (hasLocation + isDestination); see `tests/command/nav_land/README.md`.

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

Values 0–8 are defined in MAVLink master common.xml.
CANCELLED (6) is absent from pymavlink 2.4.49 — always use the MAVLink submodule XML as the authoritative source.

## Command protocol flow

1. GCS sends COMMAND_INT (or COMMAND_LONG).
2. Stack sends COMMAND_ACK with MAV_RESULT.
3. If no ACK within timeout (default 5s): retransmit.
   COMMAND_INT retransmissions are identical (no confirmation field).
   COMMAND_LONG increments `confirmation` (0, 1, 2…).
4. After max retries with no ACK: treat as UNKNOWN (not UNSUPPORTED).

## No-ACK policy

Per the MAVLink spec, every command must receive a COMMAND_ACK.
Not sending one is a spec violation.
However, a missing ACK is ambiguous:
- The stack may be too busy / dropped the message (transient, retry appropriate).
- The stack received it but chose not to ACK (spec violation, but command may be executing).
- The stack does not recognise the command (truly unsupported).

**Policy**: treat "no ACK within timeout" as result `UNKNOWN`, never `UNSUPPORTED`.
Log at WARNING level: "No COMMAND_ACK received within Xs — command may be executing without acknowledgement (spec violation) or truly unsupported; further testing required."

## NaN in float fields

Pass `None` (Python) in `fields_json` to encode NaN on the wire.

**How it works**: `json.dumps(None)` → `"null"`; nlohmann/json (MAVSDK C++ gRPC bridge) decodes a JSON `null` in a float field as IEEE-754 NaN; pymavlink encodes the received MAVLink binary back to JSON as `null`; Python receives `None`.
The round-trip is: `None` → `null` → NaN → `null` → `None`.

**Note**: `json.dumps(float('nan'))` produces `'NaN'` (non-standard JSON) which nlohmann/json rejects as `INVALID_FIELD`.
Always use `None`, never `float('nan')`, in `fields_json`.

The MAVLink spec permits NaN for unused params and some defined params (e.g. param4=NaN means "use current heading").

## Command vs mission protocol differences

The same MAV_CMD may behave differently in the two paths:
- **Mission protocol** (MISSION_ITEM_INT upload): parameters are *stored* and used during mission execution.
  The autopilot may normalise them on storage.
- **Command protocol** (COMMAND_INT direct): parameters are used immediately.
  The autopilot may ignore parameters it doesn't act on in real time.

Known example — NAV_TAKEOFF param4 (Yaw):
- **PX4 mission**: param4 stored; used to set heading after takeoff.
- **PX4 COMMAND_INT**: `rep->current.yaw = NAN` regardless of param4 (`navigator_main.cpp:630`).
- **ArduPilot mission**: param4 NOT stored (only param1 stored).
- **ArduPilot COMMAND_INT**: `// param4 : yaw angle   (not supported)` (`GCS_MAVLink_Copter.cpp:585`).

## NAV_LAND (cmd=21) — see `tests/command/nav_land/README.md`

Structurally similar to NAV_TAKEOFF (param1 numeric / param2 mode-ish / param4 Yaw / params 5–7 location), but **param7 has fundamentally different semantics**: NAV_TAKEOFF's `z` is a destination to climb to; NAV_LAND's `z` is documented as "Landing altitude (ground level in current frame)" — a touchdown/ground reference, not a waypoint. Whether a stack actually treats it that way (and whether the commanded x/y/z is the touchdown point or some other reference, e.g. for a fixed-wing approach pattern) is an execution-semantics question that ACK-level tests cannot answer — it required flight observation (Tier 2, now complete; see the README's "Tier 2 results" section and the cross-platform summary below).

Two spec gaps surfaced while designing the tests (not present for NAV_TAKEOFF, where the equivalent fields *do* define NaN sentinels):
- **param1 (Abort Alt)**: spec defines `0` as "use system default" but is silent on `NaN` — neither ACCEPTED nor DENIED for `NaN` would be a spec violation, so the corresponding test (`test_param1_abort_alt_nan`) is observational only, not an assertion/xfail.
- **param7 (Altitude)**: unlike NAV_TAKEOFF (where `NaN` is explicitly "use default altitude"), the spec does not define what `NaN` means for a "ground level" reference — observational only (`test_altitude_nan_ack`).

### NAV_LAND Tier 2 results — flight observation resolves spec gaps #3/#4 (2026-06-08)

Four consolidated comprehensive flight tests in `tests/command/nav_land/test_flight.py`
(one per vehicle type, reusing `nav_takeoff/test_flight.py`'s arming/telemetry helpers
via cross-module import) answer the execution-semantics questions ACK-level tests
couldn't: **does the commanded x/y/z (params 5–7) determine where/how the vehicle
lands?**

**Headline finding — landing-point identity (Spec gap #4) resolved, surprisingly**:
on every platform where landing could be observed (PX4 MC, ArduCopter MC, PX4 VTOL),
**the commanded coordinate is *not* the touchdown point** — the vehicle simply
descends from wherever it already is ("descend-in-place" / "lands in current mode"),
landing 80–96% of the commanded lateral offset away from the target. Unlike
NAV_TAKEOFF (where the commanded lat/lon/alt *is* the destination), NAV_LAND's
params 5/6 appear to be **ignored for the actual landing manoeuvre** on rotary-wing
and VTOL vehicles.

**param7 semantic ambiguity (Spec gap #3) — partially resolved**: since the vehicle
descends in place to actual ground level (`landed_state() == ON_GROUND`) regardless
of the commanded altitude, "ground level in current frame" appears to be a
hint/no-op rather than an actively-used reference for these vehicle types.

Per-stack findings:
- **PX4 MC**: `ACCEPTED`, descends in place, touches down 80.2 m from the commanded
  point (≈ the full commanded offset — i.e. it never moved toward the target).
- **ArduCopter MC**: `ACCEPTED`, descends (touchdown 80.0 m from target — same
  descend-in-place pattern as PX4 MC), but `landed_state()` never reported
  `ON_GROUND` within 300 s despite the vehicle being at 0.0 m relative altitude — a
  `landed_state()` reporting-lag quirk (telemetry-settling artifact), not a failed
  landing.
- **PX4 FW**: inconclusive — NAV_TAKEOFF was `ACCEPTED` but the aircraft never left
  the ground (documented SIH ground-roll-only limitation, see `nav_takeoff/README.md`
  § PX4 FW), so NAV_LAND was never sent. A simulator constraint, not a NAV_LAND gap.
- **PX4 VTOL**: stays in MC/hover throughout (no transition observed — it was
  already in the mode needed for vertical landing), descends and touches down
  79.6 m from the commanded point. Classification: **(b) lands in the current mode
  without transitioning** (of the three possible buckets — (a) transitions to
  hover, (b) lands in current mode, (c) doesn't land/inert).
- **PX4 Rover**: `ACCEPTED` but produces no landing-like behaviour (mode, position
  unchanged) — confirming the same permissive-but-meaningless acceptance pattern
  already documented for NAV_TAKEOFF on PX4 Rover. (The `landed_state()` `IN_AIR`
  → `ON_GROUND` transition observed during the test is a spawn/arm telemetry-settling
  artifact, confirmed present *before* NAV_LAND was even sent — not a genuine
  response to the command.)

### NAV_LAND Tier 1 results — all stacks (2026-06-08)

Full per-test tables in `tests/command/nav_land/README.md` § Tier 1 test results. Survey predictions confirmed exactly:

- **PX4 (MC/FW/VTOL/Rover)**: NAV_LAND SUPPORTED on every vehicle type, byte-identical results across all four (18 PASS, 1 XFAIL — the documented `float(INT32_MAX)` COMMAND_LONG rejection, same gap as NAV_TAKEOFF).
  **New finding**: PX4 *validates* `param1` (Abort Alt) and returns `DENIED (2)` for any non-zero finite value (`10.0` and `-5.0` both denied; `0.0` and `NaN` both accepted) — unlike NAV_TAKEOFF, where PX4 ignores param1/pitch entirely (`xfail`s `test_param1_pitch_ack_denied`). This is plausibly correct ("0 = use default" implies other values are checked against an internal range), but means a GCS cannot assume "any finite abort altitude is accepted".
- **ArduCopter MC**: NAV_LAND SUPPORTED (18 PASS, 1 XFAIL — `test_location_out_of_range_latlon_ack` accepts geometrically impossible lat/lon, same documented gap as NAV_TAKEOFF). Unlike PX4, ArduCopter does **not** validate `param1` — `10.0`, `-5.0`, `0.0`, and `NaN` are all `ACCEPTED`.
  **New finding**: for the COMMAND_LONG `float(INT32_MAX)` lat/lon sentinel test, ArduCopter gives **no ACK at all** (`UNKNOWN`, logged per the no-ACK policy, not asserted) — a different failure mode than PX4's explicit `DENIED`. Both are arguably spec violations (the sentinel should be `ACCEPTED`), but ArduCopter's silent drop is harder to distinguish from "busy/transient" without further probing.
- **ArduPlane FW / ArduPlane QP / ArduRover**: NAV_LAND **UNSUPPORTED** on all three — `_ensure_supported()` skips all 19 tests (19 SKIP each), exactly as the survey predicted. Consistent with NAV_LAND being aerial-landing-specific: ArduPlane and ArduRover gate it out entirely, while PX4 accepts it on every vehicle type including rover.

## DO_SET_MISSION_CURRENT (cmd=224) — see `tests/command/do_set_mission_current/README.md`

`hasLocation="false" isDestination="false"` → COMMAND_LONG is the primary message type (no location, no float coordinate params — see COMMAND_INT vs COMMAND_LONG selection rules above).

**Authoritative behaviour matrix** (provided by the project maintainer, refining the bare common.xml text — this is what the test assertions in `test_command.py` encode, not just the XML alone):

- **No mission uploaded**: ANY param1/param2 combination → `MAV_RESULT_FAILED`. A precondition-failure gate, not just an instance of the out-of-range case — takes priority over param-level validation entirely.
- **Mission uploaded, param1 ("Number")**: `-1` → `ACCEPTED` (keeps current item unchanged); `> number of mission items` → `FAILED`; a valid index → `ACCEPTED` (sets current item); any other value (e.g. negative, not `-1`) → `DENIED`.
- **Mission uploaded, param2 ("Reset Mission", `MAV_BOOL`)**: `0` → `ACCEPTED` (jump counters untouched); `1` → `ACCEPTED` (resets `DO_JUMP` repeat counters + promotes a `MISSION_STATE_COMPLETE` mission to `PAUSED`/`ACTIVE`, making a completed mission restartable); any other value → `DENIED`.
- params 3–7 (`Empty`, reserved) aren't covered by the matrix above — spec doesn't name a result code for non-NaN values there, so those stay the usual ambiguous-result xfail convention.

Because "no mission" masks per-parameter validation, `test_command.py` has two classes: `TestDoSetMissionCurrentNoMission` (confirms the gate itself, 3 tests) and `TestDoSetMissionCurrentWithMission` (the full matrix above, 15 tests, mission uploaded via an autouse fixture). Every matrix case is a hard assertion on real stacks (xfail-with-DOC-DISCREPANCY-log-line as a regression guard if not met, rather than a bare assert), observational in mock mode (`MockFlightStack` has no per-command mission-state tracking for cmd 224).

Survey status (`tests/command/README.md`): SUPPORTED on ArduCopter MC and ArduRover; UNSUPPORTED on all PX4 vehicle types (MC/FW/VTOL/Rover) per the 2026-05-27 survey (PX4 1.18.0-alpha); UNKNOWN (no ACK) on ArduPlane FW/QP; SUPPORTED on the Mock. **This PX4 status is known stale** — live testing shows PX4 MC actively processes the command (never `UNSUPPORTED`); survey table not yet regenerated.

One execution-semantics question remains out of ACK-level test scope and is documented as a **design outline only** (not implemented) in `tests/command/do_set_mission_current/README.md`: does the command actually move the current mission item (would need `MISSION_CURRENT` message observation, requiring a `MockFlightStack` extension analogous to `emit_gps_global_origin`).

The second — does `param2` actually reset a `DO_JUMP` repeat counter — **is implemented** (`tests/command/do_set_mission_current/test_flight.py::test_param2_resets_jump_counter`, Tier 2 flight-execution) and has been run against Mock (SKIP — no mission executor) and PX4 MC (**PASS** — confirms `param2=1` genuinely resets the counter, including the `param1=-1` reset sentinel working correctly mid-flight). ArduCopter MC is blocked by a SITL initialisation issue confirmed present across the prebuilt binary, a fresh ArduPilot master build, and a fresh `Copter-4.6.3` stable build — see item 7 in the root `CLAUDE.md`'s Future work list and `tests/command/do_set_mission_current/README.md` § ArduCopter SITL boot issue.

### Findings (PX4 MC 1.18.0-beta)

Live testing found PX4 MC actively processes DO_SET_MISSION_CURRENT — contradicting the 2026-05-27 survey's `UNSUPPORTED` (survey table not yet regenerated to reflect this). All 18 Tier 1 tests pass with zero deviation from the authoritative matrix above, and the Tier 2 jump-counter test confirms `param2=1` genuinely resets a `DO_JUMP` repeat counter, including via the spec-correct `param1=-1` sentinel sent mid-flight.

Also found: PX4's `MISSION_CURRENT`/`mission_progress()` stream oscillates rapidly (alternating seq values at ~1 Hz, no real vehicle movement) around a `DO_JUMP` item — a reporting artifact that breaks naive "count seq transitions" visit-tallying; `test_flight.py` works around it by requiring a genuine loop traversal (an intervening waypoint) before counting a revisit. Full detail, raw traces, and per-stack Tier 1/Tier 2 result tables: `tests/command/do_set_mission_current/README.md`.

## MAVLink XML submodule

`mavlink/message_definitions/v1.0/common.xml` contains 168 MAV_CMD entries. common.xml includes standard.xml which includes minimal.xml (common is the full superset).

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

- **NAV_TAKEOFF (cmd=22) is SUPPORTED** — `MAV_RESULT_ACCEPTED`.
  All 9 takeoff command tests PASS.
- **NAV_VTOL_TAKEOFF (cmd=84) is SUPPORTED** — PX4 multicopter accepts VTOL_TAKEOFF.
- **NAV_VTOL_LAND (cmd=85) is UNSUPPORTED** on multicopter (VTOL_LAND requires a VTOL-capable vehicle in PX4).
- **NAV_LOITER_UNLIM (17), CONDITION_YAW (115), DO_REPOSITION (192)** are all UNSUPPORTED via COMMAND_INT (PX4 uses different execution paths for these).
- **27 UNKNOWN** commands: mostly camera/gimbal commands that PX4 does not ACK via COMMAND_INT within 2s.
  These may be handled by companion computers or camera managers.
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
- **NAV_TAKEOFF (cmd=22) is SUPPORTED** (result=0 ACCEPTED) — PX4 does not gate commands by vehicle type.
  This contrasts with ArduRover which returns UNSUPPORTED.
  All 9 NAV_TAKEOFF command tests PASS.

## Autopilot-specific behaviour (ArduRover)

Tested against ArduRover V4.8.0-dev (70fe7125, `--model rover`) connected via TCP port 5760.
Log: `logs/command_survey_ardupilot_rover_4.8.0-dev_20260526_212652.log`.
Results: **49 SUPPORTED, 118 UNSUPPORTED, 1 UNKNOWN** (command survey).

### Key behaviours

- **NAV_TAKEOFF (cmd=22) is UNSUPPORTED** — `MAV_RESULT_UNSUPPORTED (3)`.
  ArduRover is a ground vehicle; the 3 command tests that assert `result != UNSUPPORTED` fail by design.
  Observational tests still pass.

## ArduCopter mode restriction for NAV_TAKEOFF

`GCS_MAVLink_Copter.cpp:578` checks `has_user_takeoff(must_navigate)` before executing NAV_TAKEOFF.
With default param3=0 (`must_navigate=true`):

| Mode | Number | Accepts NAV_TAKEOFF | Autonomous climb (no RC) |
|------|--------|---------------------|--------------------------|
| STABILIZE | 0 | ❌ | — |
| ALT_HOLD | 2 | ❌ | — |
| **GUIDED** | **4** | **✅** | **✅ uses `_AutoTakeoff::run()`** |
| LOITER | 5 | ✅ | ❌ uses pilot controller |
| POSHOLD | 16 | ✅ | ❌ uses pilot controller |

Only GUIDED overrides `do_user_takeoff_start_m()` to use the autonomous `_AutoTakeoff::run()` controller.
LOITER/POSHOLD accept the command but use `_TakeOff::do_pilot_takeoff_ms()` which reads the RC throttle channel — no autonomous climb without RC input.
All ArduPilot autotest calls to `user_takeoff()` are preceded by `change_mode("GUIDED")`.

Additional ArduPilot MAVSDK quirks (see `tests/command/baseline_takeoff/README.md`):
- MAVSDK reports ArduCopter GUIDED (custom_mode=4) as `"OFFBOARD"` in `telemetry.flight_mode()`
- MAVSDK reports ArduPlane GUIDED (custom_mode=15) as `"GUIDED"` or `"OFFBOARD"`
- ArduCopter/ArduPlane does not stream `GLOBAL_POSITION_INT` by default — must send `MAV_CMD_SET_MESSAGE_INTERVAL (511)` with `param1=33` first

## Autopilot-specific behaviour (ArduRover, continued)

### Key behaviours (ArduRover)

- **DO_FLIGHTTERMINATION (cmd=185) is UNSUPPORTED** on rover (vs SUPPORTED on copter/plane).
- **DO_REPOSITION (cmd=192), DO_FENCE_ENABLE (207), DO_SET_MISSION_CURRENT (224), MISSION_START (300)** are all SUPPORTED — rover has full mission-management capability.
- **Broader DO_ coverage** than ArduPlane: 49 SUPPORTED commands including camera, relay, servo, gimbal, and logging commands that ArduPlane doesn't respond to.
