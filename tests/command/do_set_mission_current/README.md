# MAV_CMD_DO_SET_MISSION_CURRENT (cmd=224) — Command Protocol Test Results

Command-protocol path only (COMMAND_LONG/COMMAND_INT → COMMAND_ACK) — there's no mission-protocol counterpart, since this is a control command, not a mission item.

## Key findings

- Tests encode an **authoritative behaviour matrix** (project-maintainer-provided, beyond the bare common.xml text — see below). `TestDoSetMissionCurrentNoMission` confirms the "no mission → FAILED" gate; `TestDoSetMissionCurrentWithMission` covers the full param1/param2 matrix.
- **PX4 MC supports the command** — `tests/command/README.md`'s survey (2026-05-27, PX4 1.18.0-alpha) says UNSUPPORTED; live testing against 1.18.0-beta shows it's actively processed. Survey table not yet regenerated (DOC DISCREPANCY #1 below).
- **PX4 MC matches the authoritative matrix exactly** — all 18 Tier 1 tests pass, zero deviation.
- **`param2=1` genuinely resets a `DO_JUMP` repeat counter** on PX4 MC, confirmed in flight (Tier 2): a `JUMP_REPEAT=2` loop visits the target 3 times normally, 4 times when reset mid-loop.
- **`param2` does NOT gate whether a completed mission can resume** on PX4 — that's purely positional (`param1` pointed at a valid, non-terminal index + re-engaging Mission mode). See DOC DISCREPANCY #2/#3.
- **`MISSION_CURRENT` oscillates rapidly around a `DO_JUMP` item** on PX4 (same seq pair reported alternately, no real travel) — a reporting artifact that broke naive seq-transition counting; see Tier 2 below for the fix.
- **ArduCopter SITL doesn't complete initialisation in this environment** (confirmed on three independent builds; PX4 SITL is unaffected) — see § ArduCopter SITL boot issue.

## Authoritative behaviour matrix

Maintainer-provided, refining the common.xml text quoted below — this is what the test assertions encode.

**No mission uploaded**: any param1/param2 → `FAILED`. A precondition gate, independent of whether the params would otherwise be valid.

**Mission uploaded — param1 ("Number")**:

| param1 | Result |
|--------|--------|
| `-1` | `ACCEPTED` — keeps current item unchanged |
| `> number of mission items` | `FAILED` |
| a valid mission item index | `ACCEPTED` — sets current item |
| any other value | `DENIED` |

**Mission uploaded — param2 ("Reset Mission")**:

| param2 | Result |
|--------|--------|
| `0` | `ACCEPTED` — jump counters untouched |
| `1` | `ACCEPTED` — resets `DO_JUMP` counters and changes mission state COMPLETE → ACTIVE/PAUSED (in principle — see DOC DISCREPANCY #3: on PX4 this isn't actually what makes it resumable) |
| any other value | `DENIED` |

params 3–7 (reserved) aren't in the matrix — spec names no result code for non-NaN values there, so they stay the usual xfail convention.

## Parameter definition (common.xml, entry 224)

`hasLocation="false"`, `isDestination="false"` → **COMMAND_LONG** is primary (`CLAUDE.md` § selection rules). One observational COMMAND_INT-equivalence test is included.

| Param | Label | Description | Values |
|-------|-------|-------------|--------|
| 1 | Number | Mission sequence to set. `-1` = keep current, just reset | minValue=-1, increment=1, no max |
| 2 | Reset Mission | Resets `DO_JUMP` counters; changes state COMPLETE→ACTIVE/PAUSED. Non-0/1 invalid | MAV_BOOL (0/1) |
| 3–7 | — | Empty (reserved) | |

Spec text on out-of-range param1: *"ACK with MAV_RESULT_FAILED if the sequence number is out of range (including if there is no mission item)."* The matrix above refines this: "no mission" is its own unconditional gate; "out of range" means `> number of mission items` specifically.

`MISSION_CURRENT` (msg 42) "should be emitted following" this command — relevant to the deferred design outline below, not to Tier 1.

## Tier 1 test groups (`test_command.py`)

A COMMAND_ACK only reports ACCEPTED/DENIED/FAILED/etc, not what the mission executor actually did — whether the command really moves the current item (design only, below) or resets a `DO_JUMP` counter (Tier 2, implemented) are out of Tier 1's scope.

| Class | Tests | Approach |
|-------|-------|----------|
| `TestDoSetMissionCurrentNoMission` | `test_no_mission_{sentinel,valid_looking_index,out_of_range}_failed` | No mission; hard-assert `FAILED` on real stacks (xfail+DOC-DISCREPANCY-log if not); observational in mock |
| `WithMission` Group A (baseline) | `test_command_accepted`, `test_exactly_one_ack`, `test_command_int_variant_observational` | Mission uploaded (`simple_mission.json`); not-UNSUPPORTED / exactly-one-ACK / observational |
| Group B (param1) | `test_param1_{negative_one_keeps_unchanged,valid_index}_accepted`, `test_param1_out_of_range_failed`, `test_param1_other_invalid_denied` | Hard-assert per the matrix; xfail+log on real stacks if not, observational in mock |
| Group C (param2) | `test_param2_{zero,one}_accepted`, `test_param2_invalid_denied` | Hard-assert per the matrix |
| Group D (reserved 3–7) | `test_reserved_param{3..7}_nonnan_ack` | Expect DENIED; xfail — spec names no result code here |

`NoMission` is deliberately minimal (confirms the gate only) — testing param1/param2 without a mission would be confounded by the FAILED gate, so all substantive assertions live in `WithMission`.

Mock mode: several checks are observational rather than asserted because `MockFlightStack`'s generic fallback ACKs any unconfigured command `ACCEPTED` — it has no cmd-224-specific mission-state tracking (see § Design below for what that would take).

## Tier 1 test results

### Mock — 13 passed, 5 xfailed (18 collected)

Every test returns `ACCEPTED` (mock's generic accept-all fallback) — so ACCEPTED/valid-index cases pass trivially and DENIED/FAILED cases are observational-pass or xfailed:

| Test | Result |
|------|--------|
| `test_no_mission_{sentinel,valid_looking_index,out_of_range}_failed` | PASS — ACCEPTED (observational) |
| `test_command_accepted`, `test_exactly_one_ack`, `test_command_int_variant_observational` | PASS — ACCEPTED |
| `test_param1_{negative_one_keeps_unchanged,valid_index}_accepted` | PASS — ACCEPTED |
| `test_param1_out_of_range_failed`, `test_param1_other_invalid_denied` | PASS — ACCEPTED (observational) |
| `test_param2_{zero,one}_accepted` | PASS — ACCEPTED |
| `test_param2_invalid_denied` | PASS — ACCEPTED (observational) |
| `test_reserved_param{3..7}_nonnan_ack` | XFAIL ×5 — ACCEPTED |

### PX4 MC (standalone, 1.18.0-beta) — 18 passed, 0 xfailed

Zero deviation from the authoritative matrix:

| Test | Result |
|------|--------|
| `test_no_mission_{sentinel,valid_looking_index,out_of_range}_failed` | PASS — FAILED(4) |
| `test_command_accepted`, `test_exactly_one_ack`, `test_command_int_variant_observational` | PASS — ACCEPTED |
| `test_param1_{negative_one_keeps_unchanged,valid_index}_accepted` | PASS — ACCEPTED |
| `test_param1_out_of_range_failed` | PASS — FAILED(4) |
| `test_param1_other_invalid_denied` | PASS — DENIED(2) |
| `test_param2_{zero,one}_accepted` | PASS — ACCEPTED |
| `test_param2_invalid_denied` | PASS — DENIED(2) |
| `test_reserved_param{3..7}_nonnan_ack` | PASS ×5 — DENIED(2) (PX4 genuinely validates these with a mission loaded — matches the xfail target) |

Tier 2 below confirms the `param1=-1` sentinel and `param2=1` reset also hold mid-flight, not just pre-flight.

### ArduCopter MC — blocked

SITL doesn't reach `is_armable` in this environment — see § ArduCopter SITL boot issue. Both Tier 1 and Tier 2 pending a working instance.

### ArduRover, ArduPlane FW/QP, other PX4 vehicle types — not run this session

Per the (partially stale — see DOC DISCREPANCY #1) survey in `tests/command/README.md`: ArduRover expected SUPPORTED, ArduPlane FW/QP expected UNKNOWN (no ACK), PX4 FW/VTOL/Rover expected UNSUPPORTED. Given the PX4 MC discrepancy found this session, don't trust these without re-verification.

## ArduCopter SITL boot issue

SITL never reaches `is_armable` — gyro/accel/mag `calibration_ok` stay false (40s+ probe), console log stalls on `Waiting for internal clock bits to be set (current=0x00)` after loading default params, no further progress. Params file, CPU/memory contention, and the `-S`/`--synthetic-clock` deprecation warning are all ruled out.

Confirmed identically on three independent builds: the prebuilt `~/ardu_sitl/arducopter` binary; a fresh `ArduPilot master` build via `sim_vehicle.py` (rules out stale binary / wrong launch method); a fresh `Copter-4.6.3` stable build via `sim_vehicle.py` (rules out a broken dev commit). Since a stable release fails identically to same-day master, and PX4 SITL works fine in this same environment, this looks environment-specific rather than an ArduPilot bug. Root cause not identified — needs `strace`/`gdb` on a hung process, unavailable here (no `sudo`).

## Tier 2 — jump-counter reset (`test_param2_resets_jump_counter`)

Flies a 6-item mission twice (control, then with a mid-loop reset) and compares `DO_JUMP` target revisits:

```
seq  command         purpose
0    NAV_TAKEOFF     climb to 15 m
1    NAV_WAYPOINT A  loop target (20 m north of home)
2    NAV_WAYPOINT B  loop far point (40 m north of home)
3    DO_JUMP(target=seq(A), repeat=2)
4    NAV_WAYPOINT C  only reached once the loop is exhausted (60 m north)
5    NAV_RETURN_TO_LAUNCH
```

Items are built at runtime from the vehicle's actual home position (`_get_home_position()`/`_north_of()`, reused from `tests/mission/nav_takeoff/test_flight.py`) rather than a static plan, since fixed coordinates wouldn't suit an arbitrary SITL home.

**Methodology pitfall found and fixed**: a first version counted every `MISSION_CURRENT.seq` transition into wp_a as a "visit" — against PX4, an expected-3-visit control run tallied 13. Tracing showed PX4 re-reports `MISSION_CURRENT` oscillating between the jump target and the `DO_JUMP` item itself several times per real jump (no travel between reports, no seq=2/wp_b in between) — a reporting artifact, not re-execution. **Fix**: only count a wp_a visit if wp_b was observed since the last counted visit (require a genuine loop traversal). With the fix, a `JUMP_REPEAT=2` control run reliably tallies 3.

`param1=-1` is sent as the spec-correct reset sentinel; if a stack denied it, the test would fall back to an explicit current-seq retry so a `-1`-sentinel bug wouldn't mask the actual `param2` signal — never triggered on PX4, where `-1` is accepted first-try, including mid-flight.

**Results**:

| Stack | Result | control_visits | test_visits (with reset) | Notes |
|-------|--------|-----------------|---------------------------|-------|
| Mock | SKIP | — | — | `MockFlightStack` has no mission executor (`require_real_stack` skip) |
| PX4 MC (1.18.0-beta) | **PASS** | 3 | 4 | matches control+1 — `param2=1` genuinely resets the counter, mid-flight, via the `-1` sentinel |
| ArduCopter MC | **BLOCKED** | — | — | see § ArduCopter SITL boot issue |

## Tier 2 — completed-mission restart (`test_param2_restarts_completed_mission`)

Checks the other half of the matrix's param2 claim: does `param2=1` make a `MISSION_STATE_COMPLETE` mission resumable, and does `param2=0` leave it completed? A minimal non-looping mission (takeoff → one waypoint → RTL) drives to completion fast; the jump-loop mission above already covers the `DO_JUMP`-reset half via a mid-flight, never-completed reset.

**Method**: subscribes to raw `MISSION_CURRENT` (msg 42), tracking `mission_state` (`MAV_MISSION_STATE`: ACTIVE=3, PAUSED=4, COMPLETE=5 — `mission_raw.mission_progress()` has no equivalent). Flies to COMPLETE, then sends `param1=-1, param2=0` (expect state stays COMPLETE) and `param1=-1, param2=1` (expect ACTIVE/PAUSED), with the same `-1`-sentinel-DENIED fallback as the jump-counter test. If `mission_state` never leaves UNKNOWN(0) (spec-legal — state reporting isn't mandatory), the test skips as inconclusive rather than failing.

**Result on PX4 MC (1.18.0-beta): XFAIL.** `param2=0` correctly left `COMPLETE`; `param2=1` was `ACCEPTED` but `mission_state` stayed `COMPLETE` (expected ACTIVE/PAUSED). Log: `logs/command_do_set_mission_current_flight_px4_quadcopter_1.18.0-beta_20260730_122757.log`.

**What PX4 sees as "mission completion"**: purely positional. `MissionBase::goToNextItem()` (`mission_base.cpp`) fails once `current_seq + 1 >= count`, setting `mission_result.finished = true` — the sole input to `MISSION_CURRENT.mission_state == COMPLETE` (`mavlink_mission.cpp`). No landed/at-home check. An `RTL` last item completes instantly (zero travel); an ordinary waypoint only on actually reaching it.

**Root cause of the XFAIL** (traced in PX4 source, ruling out a test-ordering bug first — `_rtl_and_land()`'s disarm was confirmed to run after, not before, the restart commands): `update_mission_state()` sets `mission_state = COMPLETE` whenever `mission_result.finished` is true, independent of flight mode. `mission_result.finished` is only cleared by `set_mission_result()`, reached via `update_mission()`/`set_mission_items()`. `Mission::set_current_mission_index()` (what `DO_SET_MISSION_CURRENT` calls) only invokes those `if (isActive())` — i.e. only while Mission mode is the vehicle's *current* flight mode. But once the mission's last item (`NAV_RETURN_TO_LAUNCH`) is reached, PX4 switches to a dedicated `RETURN_TO_LAUNCH` mode (confirmed via `flight_mode()`), so `isActive()` is false by the time the reset is sent and its effect never propagates to `mission_state`. Separately, `param2`'s only concrete PX4-side effect is `resetMissionJumpCounter()` — moot for a mission with no `DO_JUMP` item (already exercised properly by the jump-counter test above, which stays in `AUTO.MISSION` throughout).

The XFAIL is a genuine result, not a test artifact — but it shows the matrix's "makes a completed mission restartable" claim needs Mission mode reactivated before `mission_state` reflects anything, and is untestable with a `DO_JUMP`-less mission. Corrected in the follow-up test below.

## Tier 2 — restart-after-Hold, corrected design (`test_param2_restarts_from_early_item_after_hold`)

Fixes two things per the maintainer's design (given ahead of running it): (1) end the mission on an ordinary waypoint, not RTL — reaching RTL is *why* the previous test's reset had nothing to propagate into; ending on a plain waypoint completes into **Hold** mode instead. (2) target the restart at an early valid index (not `-1`), followed by a raw `MISSION_START(param1=-1)` to re-engage `AUTO.MISSION` — `param1=-1` matters because PX4's Navigator only acts on `MISSION_START`'s param1 when `>= 0`, so `-1` reactivates the mode without overwriting the index just set.

```
seq  command       purpose
0    NAV_TAKEOFF   climb to HOLD_ALT_M
1    NAV_WAYPOINT  early item / restart target (15 m north of home)
2    NAV_WAYPOINT  last item — completes into Hold (30 m north of home)
```

**Code-inspection prediction** (made before running, per the maintainer's request), from `Mission::set_current_mission_index()` and `MissionBase::isMissionValid()`/`update_mission()`/`set_mission_items()`: setting a genuinely different, non-`-1` target index sets `_is_current_planned_mission_item_valid = true` **unconditionally** — not gated by `param2`. `isMissionValid()` checks `current_seq`, dataman id, timestamp, and `mission_result.valid`, but never `mission_result.finished`. So once Mission mode reactivates, the resolved current item is the early index — prediction: resumption doesn't depend on `param2` at all, only on a `DO_JUMP` item existing for `param2` to reset (this mission has none).

**Empirical result (PX4 MC 1.18.0-beta): PASS — prediction confirmed.** Log: `logs/command_do_set_mission_current_flight_px4_quadcopter_1.18.0-beta_20260730_hold_restart.log`.

```
Mission run finished — mission_state=5 (seq=2) flight_mode='HOLD'
Attempt A: DO_SET_MISSION_CURRENT(param1=1, param2=0) ack=0
Attempt A: MISSION_START(param1=-1) ack=0 -> mission_state=3 flight_mode='MISSION' (seq=1)
Attempt A resumed the mission (param2=0!) — waited for re-completion: mission_state=5
Attempt B: DO_SET_MISSION_CURRENT(param1=1, param2=1) ack=0
Attempt B: MISSION_START(param1=-1) ack=0 -> mission_state=3 flight_mode='MISSION' (seq=1)
PASSED
```

Ending on a plain waypoint does land in Hold, not RTL, confirming design fix (1). **Both attempts resumed the mission** — including attempt A with `param2=0`, which the matrix's plain reading wouldn't predict. **Conclusion**: on PX4 MC, `param2` does not gate mission resumption; what gates it is `param1` pointing at a valid non-terminal item plus Mission mode reactivation. `param2`'s only demonstrated PX4-side effect remains the `DO_JUMP` counter reset. The test asserts only the matrix's literal claim (`param2=1` must restart) and passes; the `param2=0`-also-resumes finding is logged as DOC DISCREPANCY #3, not asserted, since the matrix is silent on what `param1` alone does.

## DOC DISCREPANCY summary

1. **Survey staleness**: `tests/command/README.md`'s survey (2026-05-27, PX4 1.18.0-alpha) shows PX4 MC UNSUPPORTED; live testing shows it's actively processed. Table not regenerated (out of scope — a 168-command survey vs. one command's deep-dive).
2. **RTL-ending mission, XFAIL** — see § Tier 2 completed-mission restart above for the full trace. Superseded by #3, which corrects the mission shape and confirms the underlying claim can hold with the right setup.
3. **`param2` doesn't gate mission resumption on PX4** — see § Tier 2 restart-after-Hold above for the trace and log. Not treated as a bug: the matrix is silent on what `param1` alone does, and `param2`'s one demonstrated effect (`DO_JUMP` reset) is moot for a `DO_JUMP`-less mission.

#2 and #3 are the same underlying question (does `mission_state` reflect a reset) approached with two mission shapes; no other discrepancies found. PX4 MC otherwise matches the authoritative matrix exactly across all 18 Tier 1 tests and the Tier 2 jump-counter test. Assertions xfail-and-log rather than bare-assert, so a regression would surface as an XFAIL with a "DOC DISCREPANCY:" line, not a silent pass.

## Design: verifying DO_SET_MISSION_CURRENT changes the current mission item

Not implemented — deferred. Upload a known 4-item mission; subscribe to raw `MISSION_CURRENT` (same subscribe-then-settle-then-send pattern as `_probe_with_send` in `conftest.py`); send `param1=2`, await ACK then `MISSION_CURRENT`, assert `seq==2`; repeat with `param1=0` and assert the seq differs (mirrors `do_set_global_origin`'s change-detection test shape); cross-check against `mission_raw.mission_progress()` as a second signal, debouncing for the `DO_JUMP`-oscillation artifact found in Tier 2 above. Needs either a `MockFlightStack` extension (a `_current_seq` tracker + `emit_mission_current` flag, analogous to `emit_gps_global_origin`) or standalone-only against ArduCopter/ArduRover.

## Running

```bash
# Tier 1 — Mock (paired)
pytest tests/command/do_set_mission_current/test_command.py -v --log-cli-level=INFO

# Tier 1 — ArduCopter SITL
pytest tests/command/do_set_mission_current/test_command.py \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
    --ardupilot-sitl=~/ardu_sitl/arducopter \
    --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 \
    --vehicle-type=copter --autopilot=ardupilot -v --log-cli-level=INFO

# Tier 1 — ArduRover SITL
pytest tests/command/do_set_mission_current/test_command.py \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
    --ardupilot-sitl=~/ardu_sitl/ardurover --ardupilot-model=rover \
    --vehicle-type=rover --autopilot=ardupilot -v --log-cli-level=INFO

# Tier 1 — PX4 SIH multicopter
pytest tests/command/do_set_mission_current/test_command.py \
    --drone-address=udp://:14540 --connection-timeout=60 \
    --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \
    --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO

# Tier 2 — jump-counter reset + completed-mission restart (skips on mock; needs a real stack)
pytest tests/command/do_set_mission_current/test_flight.py \
    --drone-address=udp://:14540 --connection-timeout=60 \
    --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \
    --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO
```

If ArduCopter SITL hangs (§ ArduCopter SITL boot issue), launch externally via `sim_vehicle.py -v ArduCopter --console --map` and point `--drone-address` at it once armable, omitting `--ardupilot-sitl`.
