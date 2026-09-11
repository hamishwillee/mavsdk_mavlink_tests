# Command protocol — implementation notes

See `tests/command/README.md` for survey result tables and per-stack test results.

## ACK uniqueness (`test_ack_uniqueness.py`)

Complements `test_survey.py` (which asks "what result?") by asking "how many terminal ACKs?" — sends every MAV_CMD via COMMAND_INT and, unlike the survey's first-match-then-cancel `probe_command_int()`, stays subscribed for a fixed 1.5 s window per command so a duplicate ACK arriving shortly after the first is not missed. IN_PROGRESS ACKs are exempt (a command may legitimately emit any number before its one terminal result); receiving the terminal result more than once is asserted as a failure — unlike the survey, this test is not purely observational.

Confirmed clean on real PX4 MC 1.18.0-beta (2026-08-19): 145 commands with exactly one terminal ACK, 23 UNKNOWN (no response — already tracked by the survey), 0 duplicates. Found one duplicate in **mock mode only**: `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` (cmd=520) gets ACK'd twice by the mavsdk_server binary itself (a legacy auto-responder, confirmed via raw wire inspection — not a bug in `MockFlightStack`'s Python logic, and not something a real vehicle exhibits). See root `CLAUDE.md` § Capability response interaction for detail; the test excludes cmd 520 from its mock-mode assertion via `_MOCK_ONLY_KNOWN_DUPLICATES`.

## COMMAND_INT vs COMMAND_LONG selection rules (per MAVLink spec)

For a command's **per-parameter semantic tests** (Group C onward in a typical `test_command.py` — the tests that send real, meaningful values and check per-field behaviour), pick ONE primary message type:

1. **Has location** (`hasLocation="true"` or `isDestination="true"` in the XML, i.e. params 5/6 carry lat/lon): prefer **COMMAND_INT** (integer ×1e7 in `x`/`y` preserves coordinate precision). If COMMAND_INT is rejected (`UNSUPPORTED`, or an explicit `MAV_RESULT_COMMAND_LONG_ONLY`), fall back to COMMAND_LONG for that command's tests.
2. **No location, but params 5/6 carry non-integer float values** (e.g. speed, duration, camera ID): use **COMMAND_LONG** — all 7 params are floats, so nothing is lost, and COMMAND_INT would force those values through `x`/`y` int32 fields.
3. **Otherwise**: use **COMMAND_INT** by default.

Examples: NAV_TAKEOFF / NAV_LAND → COMMAND_INT (hasLocation + isDestination; see `nav_land/README.md`). DO_SET_MISSION_CURRENT / EXTERNAL_WIND_ESTIMATE → COMMAND_LONG (no location, meaningful floats).

This per-command "primary" choice is separate from the **mandatory common tests** below, which are always sent via both message types regardless of which one is primary.

## Mandatory common tests (every command's `test_command.py`)

Every per-command `test_command.py` must include these six checks, in addition to whatever per-parameter tests are specific to that command. Checks 1–5 are sent **via COMMAND_INT, then via COMMAND_LONG** (`probe_dual()` in `conftest.py` does both sends and reduces each to its `effective_ack()` — see its docstring for the dual-ACK-race rationale). **Report the result once**; only call out both message types separately when they *disagree* (log it as a finding — a real protocol inconsistency, not just noise).

1. **Always ACKs** — a non-response (`UNKNOWN`, no ACK from either message type) is a spec violation. Assert an ACK was received.
2. **Not UNSUPPORTED** — the baseline (valid, meaningful parameters) must not ACK `UNSUPPORTED(3)`.
3. **Exactly one terminal ACK** — collect the full window (`probe_command_*_all_acks()`); more than one terminal result for a single send is a bug (own bug, or a stack-side race — see the confirmed PX4 Commander/EKF2 dual-ACK bug in § EXTERNAL_WIND_ESTIMATE below for a worked example of the latter).
4. **Undefined ("Empty") params accept their own sentinel, reject anything else** — for every param with no MAVLink definition, as an explicit **accepted/rejected pair**:
   - accepted: send the correct sentinel for each wire form together, expect not `UNSUPPORTED`/`DENIED`.
   - rejected: send a real (non-sentinel) value in each wire form, expect `DENIED`.

   **The sentinel — the ONE value that means "no value" — is `NaN` for a float slot and `INT32_MAX` for an int32 slot.** Get this right per param, not per command:
   - `INT32_MAX` is **only ever relevant for a param whose COMMAND_INT wire form is `x` or `y`** (the only genuinely int32 fields either message type has — this is normally param5/param6 for a `hasLocation` command, or whatever param5/param6 map to for a non-location command). When that same param is sent via COMMAND_LONG, it's a float there too, so its sentinel is `NaN`, not `INT32_MAX` — do **not** test `float(INT32_MAX)` as a separate "is this denied" case; it's just an arbitrary non-NaN float, no different from `1.0`.
   - Every other param — including COMMAND_INT's `z` (↔ some command's param7) and param1–4 — is a float in **both** message types and **never** has an INT32_MAX form. Testing `INT32_MAX` there is a bug, not extra coverage (caught and fixed in `external_wind_estimate/test_command.py` — an earlier version had a `test_param7_int32max_denied`, which cannot happen since param7 is never int32).
5. **Defined (used) params tolerate their sentinel** — the converse of (4): sending the sentinel (`NaN` for a used float param, `INT32_MAX` for a used int32 param such as a hasLocation command's `x`/`y`) must **not** return `DENIED`, even where the command's own spec text doesn't explicitly document a NaN/INT32_MAX meaning for that specific field. This generalises the general MAVLink "value not specified" convention to every used param, not just the ones with documented sentinel text — treat it as the default expectation, and only exempt a specific param if the command's docs make clear the value is *mandatory* with no fallback (e.g. DO_SET_GLOBAL_ORIGIN's lat/lon, which must be a real coordinate).
6. **Frame validation survey** — COMMAND_INT only (COMMAND_LONG has no `frame` field, so there is no COMMAND_LONG counterpart). Send the baseline command across the full `MAV_FRAME_CATALOGUE` (`conftest.py`, 22 values, 0–21 — same catalogue `tests/mission/test_frame_types.py` uses) and check whether *any* response is `MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME` (9). This is **observational, not a pass/fail assertion**:
   - Seeing 9 for at least one frame is positive evidence the stack validates `frame` for this command — record it as the finding (a real result, not a guess).
   - Seeing it for **no** frame proves nothing either way — the stack may still validate frame using a value outside the tested catalogue, or check it via some other mechanism. Record this as **INCONCLUSIVE**, never as a failure or as "frame is unvalidated".
   - A location-bearing command (frame genuinely affects interpretation of `x`/`y`/`z`) is far more likely to actually exercise frame validation than a non-location command like EXTERNAL_WIND_ESTIMATE (where PX4's handler never reads `frame` at all, so INCONCLUSIVE is the expected, source-consistent outcome there) — don't read too much into an INCONCLUSIVE result for a non-location command.
   - **Report format**: log the full per-frame breakdown (frame id/name → result or `UNKNOWN`) only if at least one frame got no ACK at all; otherwise a single terse "all N frames ACKed" note is enough — but always state the primary finding (which frame(s) hit 9, or that none did) regardless of which report format is used. `_record_detail()` / `_DETAILS` (`external_wind_estimate/test_command.py`) appends this as a supplementary block in the Tier 1 log, since it doesn't fit the one-line-per-test table.

**Naming convention**: a test name states (a) which param, (b) whether it is **defined** (has a MAVLink meaning) or **undefined** (Empty — no MAVLink meaning), and (c) the pass case in one word: `test_param{N}_defined_nan_not_denied` / `test_param{N}_undefined_{sentinel}_accepted` / `test_param{N}_undefined_nonsentinel_rejected`. Each test's docstring first line is a one-sentence statement of its pass case, and is the single source of truth for the test's name, the auto-generated results log (below), and any results table in the command's own README.md — write it once, don't duplicate it as a separate string. For a command migrated onto `Tier1CommandTestBase` (see below), checks 4/5 are pytest-parametrized (`test_undefined_param_sentinel_accepted[param5]`, `test_defined_param_sentinel_tolerated[param1]`) rather than individually hand-named per param — the bracketed parametrize id plays the same role the hand-chosen name used to.

**Auto-generated results log**: every `test_command.py` run (any mode — mock or a real stack) must write its Tier 1 results to `logs/command_<command>_tier1_<autopilot>_<vehicle>_<version>_<timestamp>.log` (test name, outcome, observed `MAV_RESULT`, one-line pass case), always — regardless of pass/fail — mirroring the existing `test_survey.py` / `test_ack_uniqueness.py` convention. `_check()` / `_record()` / `_write_tier1_log` live in `conftest.py` (shared — see below), reusing `_format_autopilot_header()` from `tests/conftest.py`.

### Shared Tier 1 infrastructure (`Tier1CommandTestBase`, `conftest.py`)

The six mandatory checks above are implemented **once**, in `tests/command/conftest.py`, rather than copy-pasted per command:

- **`CommandSpec` / `ParamSpec`** (dataclasses, `conftest.py`) declaratively describe one command: `CommandSpec.baseline` (a valid `probe_dual()` kwargs dict) and `CommandSpec.params` (a `ParamSpec` per slot 1–7 — `label`, `defined`, and for a defined param an optional `sentinel_policy="deny_required"` for the DO_SET_GLOBAL_ORIGIN-style "mandatory field, no sentinel fallback" exemption, or for an undefined param an optional `reject_xfail_reason` for a documented per-command known-behaviour gap). `ParamSpec.slot` alone determines the COMMAND_INT wire mapping (5→`x` int32, 6→`y` int32, 7→`z` float, 1-4→`paramN` float in both message types) — this is a property of the MAVLink message struct, not of whether the command's spec assigns that slot meaning, so it needs no separate field.
- **`Tier1CommandTestBase`** (`conftest.py`) provides the four fixed-shape checks as real methods (`test_command_ack_received`, `test_command_supported`, `test_exactly_one_ack`, `test_frame_validation_survey`) plus the two count-varying ones (`test_undefined_param_sentinel_accepted`/`_nonsentinel_rejected`, `test_defined_param_sentinel_tolerated`), parametrized per-command by a `pytest_generate_tests` hook (also `conftest.py`) that reads the subclass's `SPEC.undefined_params`/`SPEC.defined_params`. A command's own `test_command.py` needs only: a module-level `SPEC = CommandSpec(...)`, `class TestXxxCommand(Tier1CommandTestBase): SPEC = SPEC`, and that command's own bespoke per-parameter tests as ordinary methods alongside the inherited ones — see `external_wind_estimate/test_command.py`'s Group D onward.
- `__init_subclass__` gives every subclass its own fresh `_RESULTS`/`_DETAILS`/`_supported` — **do not** move these onto the base class itself; multiple `Test*Command` classes (different commands) run in one pytest session and must not share state.
- A test overriding one of the inherited methods (e.g. `external_wind_estimate`'s `test_exactly_one_ack`, which attaches its own documented PX4 root-cause text to the xfail reason) is expected and fine — subclass it like any other Python method override.

**Migration status**: `external_wind_estimate` and `nav_takeoff` are migrated onto `Tier1CommandTestBase` (2026-09-11 pilot). `nav_land`, `do_reposition`, `do_set_mission_current`, and `do_set_global_origin` are **not yet migrated** — they still hand-roll their own version of some or all of the six checks (most test only one message type, most survey only 1–2 hardcoded frames instead of the full catalogue, two hand-roll their own exactly-one-ACK loop). Migrating one: define its `SPEC`, subclass `Tier1CommandTestBase`, delete whatever of its own Group A/B/C tests the base class now covers, keep everything else.

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
| 9 | COMMAND_UNSUPPORTED_MAV_FRAME | A frame is required and the specified frame is not supported |
| 10 | NOT_IN_CONTROL | Source system is not in control of the target (`<wip/>` in common.xml) |

Values 0–10 are defined in MAVLink master common.xml.
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

A third — does `param2=1` actually make a `MISSION_STATE_COMPLETE` mission restartable (`ACTIVE`/`PAUSED`), while `param2=0` leaves it completed — **is now also implemented**, in two versions:

- `test_param2_restarts_completed_mission`: flies a minimal takeoff→waypoint→RTL mission to completion and observes raw `MISSION_CURRENT.mission_state` before/after each restart attempt (`param1=-1` both times). **Run against PX4 MC 1.18.0-beta (2026-07-30): XFAIL.** `DO_SET_MISSION_CURRENT`'s reset only propagates to `mission_state` `if (isActive())` (Mission navigator mode currently active) — but PX4 switches the vehicle out of `AUTO.MISSION` into a dedicated `RETURN_TO_LAUNCH` mode once the mission's `RTL` item is reached, so the reset command's effect never propagates regardless of `param2`.
- `test_param2_restarts_from_early_item_after_hold`: the corrected follow-up. Ends the mission on an ORDINARY waypoint (completes into **Hold**, not RTL — confirmed via `flight_mode()`), targets an early *valid index* with `param1` (not `-1`), and follows up with a raw `MAV_CMD_MISSION_START(param1=-1)` to re-engage `AUTO.MISSION` (PX4's Navigator ignores `MISSION_START`'s `param1` unless `>= 0`, so `-1` reactivates the mode without Navigator overwriting the index just set). **Run against PX4 MC 1.18.0-beta (2026-07-30): PASS — but with a surprising finding, confirmed both by code inspection and by running the test**: `param2` does **not** gate resumption on PX4. Attempt A (`param1=<early index>, param2=0`) followed by `MISSION_START` resumed the mission just as attempt B (`param2=1`) did — `isMissionValid()` never checks `mission_result.finished`, and moving to a valid non-terminal index sets `_is_current_planned_mission_item_valid = true` unconditionally in `mission.cpp`. `param2`'s only demonstrated PX4-side effect remains the `DO_JUMP` counter reset (`test_param2_resets_jump_counter`), which is moot for a mission with no `DO_JUMP` item. Full trace, prediction, and log: `tests/command/do_set_mission_current/README.md` § "Tier 2 — restart-after-Hold test, corrected design".

**PX4 completion mechanism** (source-traced): purely positional — `MissionBase::goToNextItem()` fails once `current_seq + 1 >= count` (`mission_base.cpp`), which sets `mission_result.finished = true`, the sole input to `MISSION_CURRENT.mission_state == COMPLETE` (`mavlink_mission.cpp`). No landed/at-home check. A `RETURN_TO_LAUNCH` last item completes instantly (zero travel); an ordinary waypoint only on actually reaching it. See `do_set_mission_current/README.md` § "What PX4 sees as 'mission completion'".

### Findings (PX4 MC 1.18.0-beta)

Live testing found PX4 MC actively processes DO_SET_MISSION_CURRENT — contradicting the 2026-05-27 survey's `UNSUPPORTED` (survey table not yet regenerated to reflect this). All 18 Tier 1 tests pass with zero deviation from the authoritative matrix above, and the Tier 2 jump-counter test confirms `param2=1` genuinely resets a `DO_JUMP` repeat counter, including via the spec-correct `param1=-1` sentinel sent mid-flight.

Also found: PX4's `MISSION_CURRENT`/`mission_progress()` stream oscillates rapidly (alternating seq values at ~1 Hz, no real vehicle movement) around a `DO_JUMP` item — a reporting artifact that breaks naive "count seq transitions" visit-tallying; `test_flight.py` works around it by requiring a genuine loop traversal (an intervening waypoint) before counting a revisit. Full detail, raw traces, and per-stack Tier 1/Tier 2 result tables: `tests/command/do_set_mission_current/README.md`.

## EXTERNAL_WIND_ESTIMATE (cmd=43004) — see `tests/command/external_wind_estimate/README.md`

`development.xml`-only command (not in the survey). `hasLocation="false" isDestination="false"`,
all params float → COMMAND_LONG is primary. Params: 1=Wind speed (m/s, min=0),
2=Wind speed accuracy (m/s, NaN=unknown), 3=Direction (deg 0–360, azimuth wind
blows FROM), 4=Direction accuracy (deg, NaN=unknown), 5–7=Empty (undefined).

**Confirmed PX4 bug — Commander/EKF2 dual-ACK race** (source-traced and
empirically reproduced, PX4 MC 1.18.0-beta-dev HEAD `c1808fb4`, 2026-09-09):
`Commander::handle_command()` (`Commander.cpp`) has an explicit ignore-list of
commands "handled by other parts of the system" that includes
`EXTERNAL_ATTITUDE_ESTIMATE`, `EXTERNAL_POSITION_ESTIMATE` and
`ESTIMATOR_SENSOR_ENABLE` — EKF2's other three vehicle_command handlers added
in the same family of work — but `EXTERNAL_WIND_ESTIMATE` is missing from it.
Commander falls through to its `default:` case and answers `UNSUPPORTED(3)`
for every send — via EITHER message type (COMMAND_INT and COMMAND_LONG both
funnel into the same internal `vehicle_command_s`) — racing EKF2's own
unconditional `ACCEPTED(0)`; which ACK reaches the GCS first is
non-deterministic (observed flipping between runs, between consecutive
sends, and independently between COMMAND_INT and COMMAND_LONG within the same
run — e.g. one run saw `COMMAND_INT: [0, 3]` and `COMMAND_LONG: [3, 0]`
moments apart). `test_ack_uniqueness.py`-style all-in-window ACK collection
(`probe_command_int_all_acks()` / `probe_command_long_all_acks()`, plus the
shared `effective_ack()` and `probe_dual()` helpers, all in
`tests/command/conftest.py`) is required to test this command's real
behaviour rather than the race; a dedicated test XFAILs the race itself, for
both message types. Likely a one-line upstream fix (add the missing case).

**DOC DISCREPANCY — ground vs air** (source-traced and empirically confirmed
via Tier 2 flight test, same PX4 build): `Ekf::resetWindToExternalObservation()`
(`wind.cpp`) is gated `if (!_control_status.flags.in_air)` — the wind-state
reset, and the flag that makes PX4 publish `WIND_COV` at all
(`get_wind_status()` = `_control_status.flags.wind || _external_wind_init`),
only applies while landed. The `COMMAND_ACK` path has no such gate (always
`ACCEPTED`). The command's own `development.xml` description explicitly
describes an in-flight ("operating at altitude") use case — PX4 currently
only implements the on-the-ground half of it. Empirically confirmed: ground
send of (8 m/s, 90°) → WIND_COV moved to exactly (north=0, east=−8); air send
of a different (15 m/s, 180°) → WIND_COV stayed pinned at the prior ground
value, completely unmoved, despite an `ACCEPTED` ack. Full write-up,
mechanism, and result tables: `tests/command/external_wind_estimate/README.md`.

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
