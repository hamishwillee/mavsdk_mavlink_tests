# MAV_CMD_DO_SET_MISSION_CURRENT (cmd=224) — Command Protocol Test Results

Tests the **command protocol** path (COMMAND_LONG/COMMAND_INT → COMMAND_ACK).
There is no mission-protocol counterpart directory for this command —
DO_SET_MISSION_CURRENT is a control command, not a mission item you upload.

## Key findings

- **Tests encode a precise, authoritative behaviour matrix** (provided by the
  project maintainer, going beyond the bare common.xml text) — see
  "Authoritative behaviour matrix" below. The test module is structured
  around it: `TestDoSetMissionCurrentNoMission` (confirms the "no mission →
  FAILED" precondition gate) and `TestDoSetMissionCurrentWithMission` (the
  full param1/param2 matrix, since "no mission" masks per-parameter
  validation).
- **PX4 MC supports DO_SET_MISSION_CURRENT** — the command survey
  (`tests/command/README.md`, tested 2026-05-27 against PX4 1.18.0-alpha)
  says UNSUPPORTED for this row; live testing shows PX4 actively processes
  the command (never `UNSUPPORTED`). Survey table not yet regenerated — see
  "DOC DISCREPANCY" below.
- **PX4 MC matches the authoritative matrix exactly** — all 18 Tier 1 tests
  pass with zero deviation: correct `ACCEPTED` for the `-1` sentinel and
  valid indices, correct `FAILED` for out-of-range `param1`, correct
  `DENIED` for invalid param1/param2 values, and correct `FAILED` across the
  board with no mission uploaded.
- **PX4 MC's `param2=1` (Reset Mission) genuinely does reset a `DO_JUMP`
  repeat counter** — confirmed behaviourally in Tier 2 (`test_flight.py`):
  a `JUMP_REPEAT=2` mission visits the jump target 3 times untouched, and 4
  times when `DO_SET_MISSION_CURRENT(param1=-1, param2=1)` is sent mid-loop
  (exactly the expected +1). The reset command itself is `ACCEPTED`
  immediately — no workaround needed.
- **`MISSION_CURRENT` oscillates rapidly around a `DO_JUMP` item** on PX4 (the
  same seq pair reported alternately several times per second, with no
  intervening travel) — a reporting artifact that broke a naive "count
  `MISSION_CURRENT.seq` transitions" visit-counting approach; see Tier 2 for
  the fix and the raw trace.
- **ArduCopter SITL cannot complete initialisation in this environment** —
  confirmed on three independent builds (a prebuilt binary, a fresh build of
  ArduPilot master, and a fresh build of the latest stable release,
  `Copter-4.6.3`), the latter two via the officially-recommended
  `Tools/autotest/sim_vehicle.py`. All three hang identically (never reports
  `is_armable`). Since the stable release fails the same way as a
  same-day development build, and PX4 SITL works fine in this same
  environment, this looks like an environment-specific issue rather than an
  ArduPilot bug — see "ArduCopter SITL boot issue" below.

## Authoritative behaviour matrix

Provided by the project maintainer to refine/extend the bare common.xml text
(quoted in "Parameter definition" below) — this is what the test assertions
encode, and is more specific than the XML alone about result codes per case.

**No mission uploaded**: ANY param1/param2 combination → `MAV_RESULT_FAILED`.
This is a precondition-failure gate — "no mission" fails outright, independent
of whether param1/param2 would otherwise be valid. Not just one instance of
the common.xml out-of-range case; it takes priority over param-level
validation entirely.

**Mission uploaded — param1 ("Number")**:

| param1 | Result |
|--------|--------|
| `-1` | `ACCEPTED` — keeps the current item unchanged |
| `> number of mission items` | `FAILED` |
| a valid mission item index | `ACCEPTED` — sets the current item |
| any other value (e.g. a negative value other than `-1`) | `DENIED` |

**Mission uploaded — param2 ("Reset Mission")**:

| param2 | Result |
|--------|--------|
| `0` (false) | `ACCEPTED` — jump counters untouched |
| `1` | `ACCEPTED` — resets jump counters AND changes mission state "completed" to "active"/"paused" (e.g. on PX4: a completed mission normally can't be restarted just by re-entering mission mode or sending a start command — resetting the jump counters via `param2=1`, with a valid current-item `param1`, makes it restartable) |
| any other value | `DENIED` |

params 3–7 (reserved) aren't covered by this matrix — the spec doesn't name a
result code for non-NaN reserved-param values, so those stay the usual
ambiguous-result xfail convention used elsewhere in this suite.

## Parameter definition (common.xml, entry 224)

`hasLocation="false"`, `isDestination="false"` → per `tests/command/CLAUDE.md` §
COMMAND_INT vs COMMAND_LONG selection rules, no param carries lat/lon or a float
location, so **COMMAND_LONG** is the correct/primary message type. One
observational COMMAND_INT-equivalence test is included since the spec doesn't
forbid it.

| Param | Label | Description | Values |
|-------|-------|-------------|--------|
| 1 | Number | Mission sequence value to set. `-1` = keep current mission item, just reset (see param2) | minValue=-1, increment=1, no max |
| 2 | Reset Mission | Reset mission (`MAV_BOOL_TRUE`). Resets `DO_JUMP` repeat counters to initial values and changes mission state "completed" to "active"/"paused". "Values not equal to 0 or 1 are invalid." | MAV_BOOL (0/1) |
| 3–7 | — | Empty (reserved) | |

Spec text on out-of-range `param1`: *"The command will ACK with
MAV_RESULT_FAILED if the sequence number is out of range (including if there
is no mission item)."* The authoritative matrix above refines this: "no
mission" is its own unconditional-FAILED gate, and "out of range" specifically
means `> number of mission items`.

`MISSION_CURRENT` (msg id 42) "should be emitted following a call to
MAV_CMD_DO_SET_MISSION_CURRENT" — relevant to the still-deferred "mission item
change" design outline below, not to the Tier 1 tests.

## What ACK-level (Tier 1) tests can and cannot show

A `COMMAND_ACK` only reports `ACCEPTED` / `DENIED` / `FAILED` / `UNSUPPORTED` /
etc. — it cannot show *what the mission executor actually did*. Two
execution-semantics questions are out of scope for Tier 1:

- Does the command actually move the current mission item (observable via the
  `MISSION_CURRENT` message)? — **still deferred, design only** (see below).
- Does `param2=1` actually reset a `DO_JUMP` repeat counter? — **implemented
  and run**, see the Tier 2 section below.

## Tier 1 test groups (`tests/command/do_set_mission_current/test_command.py`)

| Class | Tests | Approach |
|-------|-------|----------|
| `TestDoSetMissionCurrentNoMission` | `test_no_mission_sentinel_failed`, `test_no_mission_valid_looking_index_failed`, `test_no_mission_out_of_range_failed` | No mission uploaded; hard-assert `FAILED` on real stacks for all three (param1=-1 / 0 / 999999) — xfail with a DOC DISCREPANCY log line if not; observational in mock mode |
| `TestDoSetMissionCurrentWithMission` — Group A (baseline/hygiene) | `test_command_accepted`, `test_exactly_one_ack`, `test_command_int_variant_observational` | Mission uploaded (`simple_mission.json`); assert not UNSUPPORTED / exactly one ACK / observational |
| Group B (param1 matrix) | `test_param1_negative_one_keeps_unchanged_accepted`, `test_param1_valid_index_accepted`, `test_param1_out_of_range_failed`, `test_param1_other_invalid_denied` | Hard-assert ACCEPTED / ACCEPTED / FAILED / DENIED per the authoritative matrix; xfail-with-DOC-DISCREPANCY on real stacks if not, observational in mock mode |
| Group C (param2 matrix) | `test_param2_zero_accepted`, `test_param2_one_accepted`, `test_param2_invalid_denied` | Hard-assert ACCEPTED / ACCEPTED / DENIED per the authoritative matrix |
| Group D (reserved params 3–7) | `test_reserved_param{3,4,5,6,7}_nonnan_ack` | Expect DENIED; xfail — spec doesn't name a result code for these, unlike param1/param2 above |

Every substantive param1/param2 assertion lives in `WithMission` — testing
them without a mission uploaded would be confounded by the "no mission →
FAILED" gate (every result would be FAILED regardless of what's actually
being tested), so `NoMission` is deliberately small: it exists only to confirm
the gate itself.

### Why several tests are observational-only in mock mode

`MockFlightStack`'s generic fallback (`tests/mock_flight_stack.py`) ACKs any
command not explicitly configured via its `command_results` constructor kwarg
with `MAV_RESULT_ACCEPTED` — it has no cmd-224-specific mission-state tracking
or per-parameter validation. Extending it to do so is exactly the mock-side
work called out in the "Design: verifying that DO_SET_MISSION_CURRENT changes
the current mission item" section below — not implemented as part of this
change.

## Tier 1 test results

### Mock (paired mode)

18 collected: **13 passed, 5 xfailed**.

| Test | Mock result |
|------|-------------|
| `test_no_mission_sentinel_failed` | PASS — result=0 ACCEPTED (observational — mock has no mission-state gate) |
| `test_no_mission_valid_looking_index_failed` | PASS — result=0 ACCEPTED (observational) |
| `test_no_mission_out_of_range_failed` | PASS — result=0 ACCEPTED (observational) |
| `test_command_accepted` | PASS — result=0 ACCEPTED |
| `test_exactly_one_ack` | PASS — result=0 ACCEPTED, exactly 1 ACK |
| `test_command_int_variant_observational` | PASS — result=0 ACCEPTED |
| `test_param1_negative_one_keeps_unchanged_accepted` | PASS — result=0 ACCEPTED |
| `test_param1_valid_index_accepted` | PASS — result=0 ACCEPTED |
| `test_param1_out_of_range_failed` | PASS — result=0 ACCEPTED (observational) |
| `test_param1_other_invalid_denied` | PASS — result=0 ACCEPTED (observational) |
| `test_param2_zero_accepted` | PASS — result=0 ACCEPTED |
| `test_param2_one_accepted` | PASS — result=0 ACCEPTED |
| `test_param2_invalid_denied` | PASS — result=0 ACCEPTED (observational) |
| `test_reserved_param{3..7}_nonnan_ack` | XFAIL ×5 — result=0 ACCEPTED |

The mock returns `ACCEPTED` for everything (generic accept-all fallback), so
every ambiguous-result or mission-state-dependent check is either trivially
satisfied (the ACCEPTED/valid-index cases) or observational/xfailed (the
DENIED/FAILED cases) there.

### PX4 MC (standalone) — 1.18.0-beta

18 collected: **18 passed, 0 xfailed.** Zero deviation from the authoritative
matrix:

| Test | Result |
|------|--------|
| `test_no_mission_sentinel_failed` | PASS — result=4 FAILED |
| `test_no_mission_valid_looking_index_failed` | PASS — result=4 FAILED |
| `test_no_mission_out_of_range_failed` | PASS — result=4 FAILED |
| `test_command_accepted` | PASS — result=0 ACCEPTED |
| `test_exactly_one_ack` | PASS — result=0 ACCEPTED, exactly 1 ACK |
| `test_command_int_variant_observational` | PASS — result=0 ACCEPTED |
| `test_param1_negative_one_keeps_unchanged_accepted` | PASS — result=0 ACCEPTED |
| `test_param1_valid_index_accepted` | PASS — result=0 ACCEPTED |
| `test_param1_out_of_range_failed` | PASS — result=4 FAILED |
| `test_param1_other_invalid_denied` | PASS — result=2 DENIED |
| `test_param2_zero_accepted` | PASS — result=0 ACCEPTED |
| `test_param2_one_accepted` | PASS — result=0 ACCEPTED |
| `test_param2_invalid_denied` | PASS — result=2 DENIED |
| `test_reserved_param{3..7}_nonnan_ack` | PASS ×5 — result=2 DENIED (matches the xfail target — PX4 genuinely validates these with a mission loaded) |

A clean, complete confirmation of the authoritative matrix. The Tier 2 section
below additionally confirms the `param1=-1` sentinel and `param2=1` reset
behaviour hold up mid-flight too, not just in this "uploaded, not flying"
context.

### ArduCopter MC (standalone) — in progress

Not yet completed this session. ArduCopter SITL initialisation hangs
indefinitely before reporting `is_armable` — see "ArduCopter SITL boot issue"
below for the investigation. Currently retrying against a clean build of
`Copter-4.6.3` (latest stable release) launched via `sim_vehicle.py`. Once a
working instance is available, both Tier 1 (`test_command.py`) and Tier 2
(`test_flight.py`) will be run and this section updated.

### ArduRover, ArduPlane FW/QP, other PX4 vehicle types — not run this session

Per the (now partially stale, see DOC DISCREPANCY) survey table in
`tests/command/README.md`: ArduRover expected SUPPORTED (same as ArduCopter
MC); ArduPlane FW/QP expected UNKNOWN (no ACK); PX4 FW/VTOL/Rover expected
UNSUPPORTED per the 2026-05-27 survey — but given the PX4 MC discrepancy found
this session, that expectation should not be trusted without re-verification.

## ArduCopter SITL boot issue

ArduCopter SITL does not reach `is_armable` in this environment — gyro/accel/
mag `calibration_ok` telemetry flags never go true (confirmed via a direct
`telemetry.health()` probe out to 40s+), and the SITL console log shows
initialisation stalling shortly after connection with `Waiting for internal
clock bits to be set (current=0x00)` followed by a `Loaded defaults from
@ROMFS/default_params/copter.parm` loop with no further progress.

Ruled out: the params file (unchanged, sane), CPU/memory contention (load
average <0.1, 9GB free), and the harmless `-S`/`--synthetic-clock` CLI
deprecation warning (confirmed cosmetic in ArduPilot's own source). Confirmed
present, identically, on three independent builds:

1. The prebuilt `~/ardu_sitl/arducopter` binary, launched directly (the
   pattern `tests/conftest.py` uses).
2. A **fresh build** of ArduPilot `master` (dated the same morning), launched
   via the officially-recommended `Tools/autotest/sim_vehicle.py` rather than
   a raw binary invocation — ruling out both "stale binary" and "wrong launch
   method" as the cause.
3. A **fresh build of `Copter-4.6.3`**, the latest stable release tag (not a
   bleeding-edge development commit), also via `sim_vehicle.py` — ruling out
   "broken development commit" as the cause too.

All three hang identically at the same point. Since a widely-used stable
release fails the same way as a same-day development build, and PX4 SITL
runs without issue in this same environment (see the PX4 Tier 1/Tier 2
results above), this points to something specific to this machine/session's
interaction with ArduCopter's SITL HAL — not an ArduPilot bug, and not
something a different ArduPilot version is likely to fix. Root cause not yet
identified; further progress likely needs lower-level debugging (`strace`/
`gdb` attached to a hung process) than is currently available in this
environment (no `sudo`, no `strace` installed).

## Tier 2 — jump-counter reset test (`tests/command/do_set_mission_current/test_flight.py`)

Implements `test_param2_resets_jump_counter`: flies a 6-item mission twice
(control, then with a mid-loop reset) and compares how many times the
`DO_JUMP` target waypoint is genuinely revisited.

```
seq  command         purpose
0    NAV_TAKEOFF     climb to 15 m
1    NAV_WAYPOINT A  loop target (20 m north of home)
2    NAV_WAYPOINT B  loop far point (40 m north of home)
3    DO_JUMP(target=seq(A), repeat=2)
4    NAV_WAYPOINT C  only reached once the loop is exhausted (60 m north)
5    NAV_RETURN_TO_LAUNCH
```

Mission items are built at runtime from the vehicle's actual home position
(`_get_home_position()` + `_north_of()`, reused from
`tests/mission/nav_takeoff/test_flight.py` via cross-module import) — not a
static plan file, since fixed absolute coordinates would be impractical to
fly to from an arbitrarily-configured SITL home.

### Methodology pitfall found and fixed: MISSION_CURRENT oscillation around DO_JUMP

A first version counted every `MISSION_CURRENT.seq` transition into wp_a's
seq as a "visit". Against PX4, a `JUMP_REPEAT=2` control run (expected 3
visits) tallied **13**. Manual `mission_progress()` tracing revealed why:

```
17.80s  current=1        (real visit #1)
23.16s  current=2
28.19s  current=3        (DO_JUMP evaluated)
28.21s  current=1        (jump back — real visit #2)
28.21s  current=3        ┐
29.23s  current=1        │
29.23s  current=3        │  oscillates 1↔3 at ~1 Hz for ~6s,
30.26s  current=1        │  no seq=2 (wp_b) in between,
...                      │  no real vehicle movement
34.35s  current=3        ┘
34.95s  current=2        (finally makes real progress)
```

PX4 re-reports `MISSION_CURRENT` alternating between the jump target and the
`DO_JUMP` item itself several times per real jump — a reporting artifact, not
genuine re-execution (confirmed: no travel between the alternating reports).
**Fix**: only count a wp_a visit if wp_b was observed as current since the
last counted visit (i.e. require a genuine loop traversal, not just a raw seq
change). With this fix, a `JUMP_REPEAT=2` control run reliably tallies
exactly 3 — the spec-correct count.

### Design note: `param1=-1` fallback

`test_param2_resets_jump_counter` sends the spec-correct `DO_SET_MISSION_CURRENT(param1=-1, param2=1)`
reset. If a stack were to `DENY` the `-1` sentinel specifically (a spec
violation — it's explicitly defined for exactly this "keep current item, just
reset" use case), the test falls back to an explicit-current-seq retry with
`param2=1`, so a stack's `-1`-sentinel bug wouldn't mask the actual signal
being measured: whether `param2` resets the jump counter. On PX4 MC this
fallback path has never been triggered — `-1` is `ACCEPTED` on the first
attempt every time, including mid-flight.

### Results

| Stack | Result | control_visits | test_visits (with reset) | Notes |
|-------|--------|-----------------|---------------------------|-------|
| Mock | SKIP | — | — | `MockFlightStack` has no mission executor at all — nothing would ever advance past the first item; skipped by design (`require_real_stack`, `--drone-address` not set) |
| PX4 MC (1.18.0-beta) | **PASS** | 3 | 4 | `param1=-1, param2=1` reset `ACCEPTED` on the first attempt (mid-flight); test_visits (4) is exactly control_visits+1, matching the expected effect of restoring the counter from 1→2 mid-loop. **param2=1 genuinely resets the DO_JUMP counter on PX4, and the -1 sentinel works correctly mid-flight.** |
| ArduCopter MC | **BLOCKED** | — | — | See "ArduCopter SITL boot issue" above — Tier 2 requires arming/flight, so it's blocked until a working ArduCopter SITL instance is available. |

## DOC DISCREPANCY summary

Per `CLAUDE.md`'s spec-discrepancy workflow — logged in test output, recorded
here, and in `tests/command/CLAUDE.md`:

1. **Survey staleness**: `tests/command/README.md`'s survey table (tested
   2026-05-27 against PX4 1.18.0-alpha) shows PX4 MC as `UNSUPPORTED` for
   cmd 224. Live testing shows it is actively processed (never
   `UNSUPPORTED`). Re-running the official survey script
   (`scripts/generate_command_tables.py` after `test_survey.py`) is
   recommended to refresh the table; not done here as out of scope for this
   change (168-command survey vs. one command's deep-dive).

No other discrepancies found — PX4 MC matches the authoritative behaviour
matrix exactly across all 18 Tier 1 tests and the Tier 2 jump-counter test.
The corresponding test assertions are hard asserts on real stacks with an
xfail-and-log fallback (not bare asserts), so a future regression would
surface as an `XFAIL` with a "DOC DISCREPANCY:" log line rather than a silent
pass, worth watching given PX4 appears to be under active development.

## Design: verifying that DO_SET_MISSION_CURRENT changes the current mission item

Not implemented — deferred design only. Approach:

1. Upload a known multi-item mission (`tests/mission/plans/simple_mission.json`,
   4 items) via `gcs_system.mission_raw.upload_mission()`.
2. Subscribe to raw `MISSION_CURRENT` (msg id 42) via
   `system.mavlink_direct.message("MISSION_CURRENT")`, using the same
   subscribe-then-settle-then-send background-task pattern as
   `_probe_with_send` in `tests/command/conftest.py` / `do_set_global_origin`'s
   `_subscribe_gps` (avoids the race where a fast stack emits before the gRPC
   stream is registered).
3. Send `DO_SET_MISSION_CURRENT(param1=2)`, await `ACK == ACCEPTED`, then await
   the `MISSION_CURRENT` message (timeout ~5s — the spec says it "should be
   emitted following" the command) and assert `seq == 2`.
4. Repeat with a different target (`param1=0`) and assert the newly-emitted
   `seq` differs from step 3's value — the actual "changes" proof (mirrors
   `do_set_global_origin`'s `test_gps_global_origin_changes_when_new_value_set`
   change-detection shape: two sends, two distinct observed values).
5. Cross-check with MAVSDK's higher-level `mission_raw.mission_progress()`
   (`MissionProgress.current`) as a second, independent signal — a disagreement
   with the raw `MISSION_CURRENT.seq` would be a MAVSDK-layer discrepancy worth
   flagging separately from stack conformance. **Caveat learned from Tier 2**:
   PX4's `MISSION_CURRENT`/`mission_progress()` stream can report transient,
   non-final seq values in quick succession around control-flow items (see the
   DO_JUMP oscillation finding above) — a robust implementation of this design
   should debounce/require dwell time, not trust the first reported value.
6. Requires either extending `MockFlightStack` to track a `_current_seq` and
   actually emit `MISSION_CURRENT` (confirmed: it does neither today — only a
   generic accept-all ACK), or running this class only in standalone mode
   against ArduCopter/ArduRover. Given the existing `emit_gps_global_origin`
   precedent in `MockFlightStack`, the lower-effort path is a matching
   `emit_mission_current: bool` constructor flag for a future PR.

## Running

```bash
# Tier 1 — Mock (paired)
pytest tests/command/do_set_mission_current/test_command.py -v --log-cli-level=INFO

# Tier 1 — ArduCopter SITL (raw binary, per tests/conftest.py's --ardupilot-sitl management)
pytest tests/command/do_set_mission_current/test_command.py \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
    --ardupilot-sitl=~/ardu_sitl/arducopter \
    --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 \
    --vehicle-type=copter --autopilot=ardupilot -v --log-cli-level=INFO

# Tier 1 — ArduCopter SITL (externally managed via sim_vehicle.py — see "ArduCopter SITL boot issue")
#   1. In a separate shell: cd ~/github/ArduPilot/ardupilot/ArduCopter && \
#        ~/github/ArduPilot/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
#   2. Then, once armable: pytest tests/command/do_set_mission_current/test_command.py \
#        --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
#        --vehicle-type=copter --autopilot=ardupilot -v --log-cli-level=INFO

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

# Tier 2 — jump-counter reset test (skips on mock; needs a real flight stack)
pytest tests/command/do_set_mission_current/test_flight.py \
    --drone-address=udp://:14540 --connection-timeout=60 \
    --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \
    --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO
```
