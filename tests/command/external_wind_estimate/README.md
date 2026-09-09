# MAV_CMD_EXTERNAL_WIND_ESTIMATE (cmd=43004) — command protocol tests

Sets an external estimate of wind speed/direction on the EKF wind estimator —
intended to extend GPS-denied dead-reckoning time by giving the estimator a
head start on the wind vector, sent every few minutes while operating at
altitude.

**Spec reference**: `mavlink/message_definitions/v1.0/development.xml` entry
`value=43004`. The command is in `development.xml`, not `common.xml`, and
does not appear in the command survey (`test_survey.py`).

## Parameter layout

| Param | Label | Notes |
|-------|-------|-------|
| 1 | Wind speed | m/s, minValue=0. Horizontal wind speed. |
| 2 | Wind speed accuracy | m/s. 1-sigma accuracy; NaN if unknown. |
| 3 | Direction | deg, minValue=0, maxValue=360. Azimuth wind blows FROM (true north). |
| 4 | Direction accuracy | deg. 1-sigma accuracy; NaN if unknown. |
| 5–7 | — | Empty — no MAVLink definition |

`hasLocation="false" isDestination="false"`, and every defined param is a
plain float (no lat/lon) → per CLAUDE.md's COMMAND_INT/COMMAND_LONG selection
rule, **COMMAND_LONG** is this command's "primary" type for per-parameter
tests. Per CLAUDE.md § Mandatory common tests, **every test in this file
sends the command via both COMMAND_INT and COMMAND_LONG** (`probe_dual()` in
`tests/command/conftest.py`) and reports one merged result — unless the two
disagree, in which case both are logged explicitly as a finding. On every run
against PX4 so far, the two message types have agreed on every single check.

Since this command has no location, COMMAND_INT's `x`/`y` (int32) and `z`
(float) — the wire-level equivalents of COMMAND_LONG's param5/6/7 — carry no
meaning either; they default to the general "not specified" sentinel for
their type (`INT32_MAX` for `x`/`y`, NaN for `z`), mirroring param5/6/7's NaN
default.

## What these tests cover

### Naming convention

A test name states (a) which param, (b) whether it's **defined** (has a
MAVLink meaning — param1–4) or **undefined** (Empty — param5–7), and (c) the
pass case in one word: `test_param{N}_defined_nan_not_denied` /
`test_param{N}_undefined_{sentinel}_accepted` /
`test_param{N}_undefined_nonsentinel_rejected`. Each test's docstring first
line is the authoritative one-sentence statement of its pass case — it is
the single source for the test's name, the auto-generated log (below), and
the "Pass case" column in the results tables further down.

**INT32_MAX only ever matters for param5/param6** (COMMAND_INT's `x`/`y` —
the only genuinely int32 wire fields either message type has). It is
**never** relevant to param7 (COMMAND_INT's `z` is a float, same as
COMMAND_LONG's param7) or to any of param1–4 (also always floats) —
`float(INT32_MAX)` in a float field is just an arbitrary non-NaN float,
already covered by the general non-sentinel case. An earlier version of this
file had a `test_param7_int32max_denied` test, which was a bug (param7 can
never be int32 in either message type) — removed, and the param5/param6
equivalents restructured into explicit accepted/rejected pairs (below) rather
than testing `float(INT32_MAX)` as a separate COMMAND_LONG-only case.

### Mandatory common tests (apply to every command — see root CLAUDE.md)

1. **Supported** — must not ACK with `UNSUPPORTED(3)` → `test_command_supported`
2. **ACKs at all** — a non-response (`UNKNOWN`) is a spec violation, checked
   for both message types independently → `test_command_ack_received`
3. **Exactly one terminal ACK per send** (checked for both message types) →
   `test_exactly_one_ack` (see "Confirmed PX4 bug" below — **XFAIL on real
   PX4**, for both COMMAND_INT and COMMAND_LONG)
4. **Undefined ("Empty") params accept their own sentinel, reject anything
   else** — param5/6/7 (↔ COMMAND_INT's `x`/`y`/`z`), each as an
   accepted/rejected pair:
   - `test_param{5,6}_undefined_int32max_accepted` — the correct sentinel for
     each wire form together (`x`/`y=INT32_MAX` via COMMAND_INT,
     `param5`/`param6=NaN` via COMMAND_LONG) → **PASS** (already the suite's
     baseline default).
   - `test_param{5,6}_undefined_nonsentinel_rejected` — a real value in each
     wire form (a real-looking int for `x`/`y`, `1.0` for param5/param6) →
     expect `DENIED`, **XFAIL** (PX4 never reads param5/param6/`x`/`y`).
   - `test_param7_undefined_nan_accepted` — `NaN` in both wire forms (param7
     and `z` are always float) → **PASS**.
   - `test_param7_undefined_nonsentinel_rejected` — `1.0` in both wire forms
     → expect `DENIED`, **XFAIL**.
5. **Defined (used) params tolerate their sentinel** — sending NaN for each
   of param1–4 must **not** return `DENIED`, even for param1/param3 where the
   spec text doesn't explicitly document a NaN meaning (only param2/param4
   do — "NaN if unknown") — the general MAVLink "not specified" convention is
   treated as the default expectation for any used param →
   `test_param{1,2,3,4}_defined_nan_not_denied`, all **PASS** (PX4 never
   validates any of param1–4)
6. **Frame validation survey** — COMMAND_INT only (COMMAND_LONG has no `frame`
   field). Sends the baseline command across all 22 canonical `MAV_FRAME`
   values and checks whether any response is
   `MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME` (9) → `test_frame_validation_survey`.
   Purely observational — **not** a pass/fail assertion: seeing result=9 for
   any frame is positive evidence of frame validation; seeing it for none
   proves nothing either way (recorded **INCONCLUSIVE**, not a failure). See
   "Frame validation survey" below for the result.

### Per-parameter tests

Each of param1–4 also gets: a nominal/in-range value (hard assert: not
`UNSUPPORTED`), boundary values where the spec defines one (min/max), and an
out-of-range value (expect `DENIED`; **XFAIL on PX4** — it clamps/wraps
instead of rejecting).

### Auto-generated results log

Every run (mock or real stack) writes a Tier 1 results table to
`logs/command_external_wind_estimate_tier1_<autopilot>_<vehicle>_<version>_<timestamp>.log`
— test name, outcome, observed `MAV_RESULT`, and the one-line pass case —
mirroring the existing `test_survey.py` / `test_ack_uniqueness.py` convention
of always writing a results file regardless of pass/fail, so a run's full
results are available without needing `--log-cli-level=INFO`.

## Confirmed PX4 bug — Commander/EKF2 dual-ACK race

Source-traced **and empirically reproduced** (PX4 MC 1.18.0-beta-dev, HEAD
`c1808fb4`, 2026-09-09): `Commander::handle_command()`
(`src/modules/commander/Commander.cpp`) has an explicit switch-case list of
commands it deliberately does **not** answer, because "commands ... handled
by other parts of the system":

```cpp
case vehicle_command_s::VEHICLE_CMD_EXTERNAL_POSITION_ESTIMATE:
case vehicle_command_s::VEHICLE_CMD_EXTERNAL_ATTITUDE_ESTIMATE:
case vehicle_command_s::VEHICLE_CMD_ESTIMATOR_SENSOR_ENABLE:
    /* ignore commands that are handled by other parts of the system: no reply from here */
    return true;

default:
    /* Command not handled above: reply UNSUPPORTED. */
    cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
    break;
```

`VEHICLE_CMD_EXTERNAL_WIND_ESTIMATE` — added to EKF2 in the same family of
work as the three commands above, and handled in an adjacent code block in
`EKF2.cpp` — is **missing from this list**. Commander therefore falls through
to its `default:` case and answers `UNSUPPORTED(3)` for **every single send**
of this command — via *either* message type, since COMMAND_INT and
COMMAND_LONG both funnel into the same internal `vehicle_command_s` topic —
racing against EKF2's own unconditional `ACCEPTED(0)`. Whichever ACK reaches
the GCS's listener first is non-deterministic: observed result flipping
between `0` and `3` on consecutive sends of the identical command, within the
same test session, no pattern — confirmed independently for COMMAND_INT and
COMMAND_LONG in the same run (`test_exactly_one_ack` observed
`COMMAND_INT: results=[0, 3]` and, moments later, `COMMAND_LONG:
results=[3, 0]` — same race, independent instances, opposite order).

This means a GCS using a naive "wait for the first ACK" client — which is
exactly what every other test in this suite's `probe_command_long()`/
`probe_command_int()` helpers do — sees this command as **randomly
unsupported roughly half the time**, even though it is genuinely implemented
and executed by EKF2 every time.

**Impact on test design**: rather than let every assertion in this file be
flaky on that account, `effective_ack()` (`tests/command/conftest.py`)
collects *all* ACKs in a 1.5 s window per message type
(`probe_command_int_all_acks()` / `probe_command_long_all_acks()`) and
prefers a non-`UNSUPPORTED` one when present; `probe_dual()` runs this for
both COMMAND_INT and COMMAND_LONG and returns both results, which
`test_command.py`'s `_reduce()` merges into one reported outcome (flagging
disagreement explicitly — see CLAUDE.md § Mandatory common tests). This tests
EKF2's real, intended handling of the command rather than which of the two
racing PX4 modules happened to answer first. The race itself is asserted
explicitly (and reliably reproduces, on both message types, so it is a hard
`xfail`, not just a log line) in `test_exactly_one_ack`.

**Suggested upstream fix**: add `case vehicle_command_s::VEHICLE_CMD_EXTERNAL_WIND_ESTIMATE:`
to the ignore-list alongside its three siblings in `Commander.cpp`. A
one-line PR — this looks like a simple oversight (the sibling commands were
added together and three of the four were remembered) rather than a design
decision.

**FIXED** (2026-09-09, commit `793d308c53`, branch `fix_external_wind_estimate_mavlink`):
"fix(mavlink): MAV_CMD_EXTERNAL_WIND_ESTIMATE double ACKing and incorrect ACK"
does exactly the above — adds the missing `case` to `Commander.cpp` — and
additionally makes `Ekf::resetWindToExternalObservation()` return `bool`
(`false` when `in_air`), with `EKF2.cpp`'s ACK now `TEMPORARILY_REJECTED`
instead of `ACCEPTED` when the reset was not applied. Verified by rebuilding
and re-running Tier 1 against this branch: `test_exactly_one_ack` now
**PASSES** (was XFAIL on every prior run against `main`); every other Tier 1
result is unchanged. Mock and PX4 MC now agree on every single Tier 1 check
for the first time — 18 PASS / 8 XFAIL on both.

## Frame validation survey

`test_frame_validation_survey` sends the baseline command via COMMAND_INT
across all 22 canonical `MAV_FRAME` values (`MAV_FRAME_CATALOGUE`,
`tests/command/conftest.py`) and checks whether any response is
`MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME` (9) — positive evidence the stack
validates the `frame` field for this command. COMMAND_LONG has no `frame`
field, so there is no COMMAND_LONG counterpart to this test.

**Result on both Mock and PX4 MC (HEAD `793d308c53`, 2026-09-09): INCONCLUSIVE
— all 22 frames ACKed, none returned result=9.** This is the expected,
source-consistent outcome, not a surprise: PX4's `EXTERNAL_WIND_ESTIMATE`
handler in `EKF2.cpp` reads only `param1`–`param4` and never inspects `frame`
(or `x`/`y`/`z`) at all — there is nothing in the implementation that *could*
produce result=9 for this command. An INCONCLUSIVE result here says nothing
about whether PX4 validates `frame` in general (it demonstrably does for at
least some commands — this survey would be far more interesting run against a
location-bearing command such as NAV_TAKEOFF or NAV_LAND, where `frame`
genuinely affects how `x`/`y`/`z` are interpreted).

## Ground vs air — the real headline finding

Tier 1 (ACK-level) tests cannot show whether the command actually changed
anything — only Tier 2 (`test_flight.py`) can, by observing `WIND_COV` (msg
id=231), which mirrors EKF2's internal `wind` uORB topic
(`src/modules/mavlink/streams/WIND_COV.hpp`:
`wind_x = windspeed_north`, `wind_y = windspeed_east`).

**Source-traced mechanism**: `Ekf::resetWindToExternalObservation()`
(`src/modules/ekf2/EKF/wind.cpp`) is gated:

```cpp
void Ekf::resetWindToExternalObservation(...)
{
    if (!_control_status.flags.in_air) {
        ...
        _external_wind_init = true;
        resetWindTo(wind, wind_var);
    }
}
```

The reset — and the flag that makes `WIND_COV` start publishing at all
(`get_wind_status()` in `estimator_interface.h` is
`_control_status.flags.wind || _external_wind_init`) — only happens **while
landed**. The `COMMAND_ACK` path in `EKF2.cpp` has no equivalent gate: it
returns `ACCEPTED` unconditionally in both states. A GCS cannot tell from the
ACK alone whether the estimate was applied.

**This is a DOC DISCREPANCY worth flagging**: the command's own description
in `development.xml` explicitly describes an *in-flight* use case —

> "...extending the time when operating without GPS before position drift
> builds to an unsafe level... the command might reasonably be sent every few
> minutes when operating at altitude."

— yet the current PX4 implementation only honours the command while landed,
silently discarding it in exactly the scenario the spec describes. This
reads as a PX4 implementation gap rather than a MAVLink spec problem — the
spec text is unambiguous; PX4 simply doesn't implement the in-air half of it.

### Empirical confirmation (PX4 MC 1.18.0-beta-dev, HEAD `c1808fb4`, SIH, 2026-09-09)

| State | Commanded (speed, direction-from) | Expected (north, east) | Observed WIND_COV | Result |
|-------|-----------------------------------|--------------------------|--------------------|--------|
| Ground (disarmed) | 8.0 m/s, 90° (from east) | (0.00, −8.00) | **(−0.00, −8.00)** | **Applied** — matches exactly |
| Air (in flight, ~20 m AGL) | 15.0 m/s, 180° (from south) | (15.00, −0.00) | **(−0.00, −8.00)** — unchanged | **Ignored** — stayed pinned at the *ground* test's stale value, did not move at all |

Both `COMMAND_ACK`s were `ACCEPTED(0)` (after resolving the dual-ACK race
above) — there is no observable difference at the ACK level between the two
states; only the `WIND_COV` telemetry reveals it.

A secondary finding falls out of this: `_external_wind_init` is **sticky for
the life of the PX4 boot** once set on the ground — it is not cleared by
arming or taking off, so the air test's "before" `WIND_COV` sample already
carried the ground test's committed value (confirming EKF2 processes the
command regardless of which ACK a listener happens to see). The test design
accounts for this: the assertion that matters is that the value *after* the
air-specific command does not move to what was *just* commanded, not that
`WIND_COV` is absent.

Per-run logs: `logs/command_external_wind_estimate_ground_px4_quadcopter_20260909_150449.log`,
`logs/command_external_wind_estimate_air_px4_quadcopter_20260909_150529.log`.

**Reproduced on a second, independent PX4 boot** (same day, run immediately
after `test_command.py`'s Tier 1 tests in the same session): identical
outcome — ground WIND_COV = `(north=−0.00, east=−8.00)`, air WIND_COV
unchanged at `(north=−0.00, east=−8.00)` despite commanding a completely
different vector. In this run the "before" baseline for *both* tests already
showed the committed value rather than "none received", because Tier 1's own
default probe parameters (`param1=8.0, param3=90.0` — the same values used
here for the ground test) had already set `_external_wind_init` earlier in
the same PX4 boot — a nice independent confirmation of the "sticky for the
life of the boot" finding above. The dual-ACK race also reproduced, with the
order flipped (`results=[3, 0]` this run vs `[0, 3]` the first time),
confirming it really is non-deterministic and not an artifact of a specific
run.

**Reproduced a third time** after refactoring the test suite to send every
check via both COMMAND_INT and COMMAND_LONG (see "Mandatory common tests"
above): identical ground/air outcome again, and COMMAND_INT/COMMAND_LONG
agreed with each other on every one of the 25 Tier 1 checks in that run (no
`INCONSISTENT` log lines) — the dual-ACK race is the one place the two
message types *do* diverge from each other run-to-run, but never within a
single send's own COMMAND_INT-vs-COMMAND_LONG comparison in this run.

**Reproduced a fourth time** after the test-naming/restructuring pass
(defined/undefined naming convention, the param7 INT32_MAX bug fix, and the
auto-generated Tier 1 log — see "Naming convention" and "Auto-generated
results log" above): identical outcome again, both Tier 2 tests PASS.

## Tier 1 test results

Latest run (2026-09-09) against PX4 MC HEAD `793d308c53` (the fix branch,
`fix_external_wind_estimate_mavlink` — see "FIXED" note above) and the Mock.
Every test sends via both COMMAND_INT and COMMAND_LONG except
`test_frame_validation_survey` (COMMAND_INT only — see "Frame validation
survey" above); COMMAND_INT and COMMAND_LONG agreed on every test that sends
both. Pulled from the auto-generated logs
(`logs/command_external_wind_estimate_tier1_mock_mock_20260909_174111.log`,
`logs/command_external_wind_estimate_tier1_px4_quadcopter_1.18.0-beta_20260909_174429.log`).

**Historical note**: against `main` HEAD `c1808fb4` (pre-fix, before
`test_frame_validation_survey` existed), `test_exactly_one_ack` was **XFAIL**
on PX4 — the confirmed Commander/EKF2 dual-ACK race described above. Every
other row below was identical pre- and post-fix.

| Test | Mock | PX4 MC | Pass case |
|------|------|--------|-----------|
| `test_command_ack_received` | PASS | PASS | ACKs (via both COMMAND_INT and COMMAND_LONG) for a baseline, valid send |
| `test_command_supported` | PASS | PASS | Not UNSUPPORTED for a baseline, valid send |
| `test_exactly_one_ack` | PASS | PASS | Exactly one terminal COMMAND_ACK per send, via each message type — XFAIL pre-fix (dual-ACK race), PASS post-fix (see "FIXED" above) |
| `test_frame_validation_survey` | INCONCLUSIVE | INCONCLUSIVE | Any MAV_FRAME value returns MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME(9) — see "Frame validation survey" above; observational, not a pass/fail check |
| `test_param5_undefined_int32max_accepted` | PASS | PASS | Accepted when param5/x is sent as its own sentinel (undefined param) |
| `test_param5_undefined_nonsentinel_rejected` | XFAIL | XFAIL | Rejected when param5/x is sent a real (non-sentinel) value (undefined param) |
| `test_param6_undefined_int32max_accepted` | PASS | PASS | Accepted when param6/y is sent as its own sentinel (undefined param) |
| `test_param6_undefined_nonsentinel_rejected` | XFAIL | XFAIL | Rejected when param6/y is sent a real (non-sentinel) value (undefined param) |
| `test_param7_undefined_nan_accepted` | PASS | PASS | Accepted when param7/z is sent as NaN (undefined param; never int32 in either message type) |
| `test_param7_undefined_nonsentinel_rejected` | XFAIL | XFAIL | Rejected when param7/z is sent a real (non-NaN) value (undefined param) |
| `test_param1_defined_nan_not_denied` | PASS | PASS | Not denied when param1 (Wind speed, defined) is sent as NaN |
| `test_param2_defined_nan_not_denied` | PASS | PASS | Not denied when param2 (Wind speed accuracy, defined) is sent as NaN — spec-documented 'unknown' sentinel |
| `test_param3_defined_nan_not_denied` | PASS | PASS | Not denied when param3 (Direction, defined) is sent as NaN |
| `test_param4_defined_nan_not_denied` | PASS | PASS | Not denied when param4 (Direction accuracy, defined) is sent as NaN — spec-documented 'unknown' sentinel |
| `test_param1_nominal` | PASS | PASS | Not UNSUPPORTED for param1 (Wind speed) = 8.0 m/s (nominal value) |
| `test_param1_zero_boundary` | PASS | PASS | Not UNSUPPORTED for param1 (Wind speed) = 0.0 m/s (minValue boundary, no wind) |
| `test_param1_negative_denied` | XFAIL | XFAIL | Denied for param1 (Wind speed) = -1.0 m/s (below minValue=0) — PX4 clamps via `math::max(speed, 0)` |
| `test_param2_specific` | PASS | PASS | Not UNSUPPORTED for param2 (Wind speed accuracy) = 1.5 m/s (specific 1-sigma estimate) |
| `test_param2_negative_denied` | XFAIL | XFAIL | Denied for param2 (Wind speed accuracy) = -1.0 (negative accuracy is physically meaningless) — PX4 squares unconditionally |
| `test_param3_nominal` | PASS | PASS | Not UNSUPPORTED for param3 (Direction) = 90.0 deg (nominal, wind from east) |
| `test_param3_zero_boundary` | PASS | PASS | Not UNSUPPORTED for param3 (Direction) = 0.0 deg (minValue boundary, wind from true north) |
| `test_param3_max_boundary` | PASS | PASS | Not UNSUPPORTED for param3 (Direction) = 360.0 deg (maxValue boundary) |
| `test_param3_negative_denied` | XFAIL | XFAIL | Denied for param3 (Direction) = -10.0 deg (below minValue=0) — PX4 wraps via `wrap_pi()` unconditionally |
| `test_param3_over_max_denied` | XFAIL | XFAIL | Denied for param3 (Direction) = 370.0 deg (above maxValue=360) — same reason |
| `test_param4_specific` | PASS | PASS | Not UNSUPPORTED for param4 (Direction accuracy) = 5.0 deg (specific 1-sigma estimate) |
| `test_param4_negative_denied` | XFAIL | XFAIL | Denied for param4 (Direction accuracy) = -5.0 (negative accuracy is physically meaningless) — no sign validation |

**Mock and PX4 MC (post-fix) now agree exactly: 17 PASS, 8 XFAIL, 1 INCONCLUSIVE
(26 tests) on both.** (At the pytest level — where an INCONCLUSIVE test still
reports PASSED, since it makes no assertion — this is 18 passed, 8 xfailed.)
Pre-fix, PX4 had one more XFAIL than Mock (`test_exactly_one_ack`) because
MockFlightStack has a single command handler and cannot reproduce the
Commander/EKF2 race.

All PX4 XFAILs above are consistent with the source-level read: EKF2 never
validates param1–4 for range or sign before use — everything short of a
malformed COMMAND_LONG is silently `ACCEPTED`.

## Tier 2 test results (`test_flight.py`)

| Test | PX4 MC |
|------|--------|
| `test_ground_wind_estimate_applied` | **PASS** — WIND_COV moved to the commanded vector |
| `test_air_wind_estimate_ignored` | **PASS** — WIND_COV did not move to the commanded vector (stayed at the prior, stale value) |

Skipped entirely in paired/mock mode (`require_real_stack` autouse fixture) —
`MockFlightStack` has no EKF and does not publish `WIND_COV`.

Not yet run against ArduCopter/ArduPlane/ArduRover, or PX4 FW/VTOL/Rover — add
results when available. (`MAV_CMD_EXTERNAL_WIND_ESTIMATE` is a PX4/EKF2-family
command from `development.xml`; ArduPilot's wind-estimation is a different
subsystem entirely and this command is not expected to be recognised there —
untested assumption, not yet confirmed.)

## Running

```bash
# Paired mock (26 Tier 1 tests: 18 passed / 8 xfailed at the pytest level --
# 17 PASS / 8 XFAIL / 1 INCONCLUSIVE in this file's own results table;
# Tier 2 skipped entirely)
pytest tests/command/external_wind_estimate/ -v --log-cli-level=INFO

# PX4 SIH multicopter, fix branch (fix_external_wind_estimate_mavlink, commit
# 793d308c53): 26 Tier 1 (same 18 passed / 8 xfailed as mock, post-fix); 2 Tier 2: 2 PASS
pytest tests/command/external_wind_estimate/ \
    --drone-address=udp://:14540 --connection-timeout=60 \
    --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \
    --vehicle-type=quadcopter --autopilot=px4 \
    -v --log-cli-level=INFO
```

Every run also writes a Tier 1 results log to `logs/` regardless of pass/fail
— see "Auto-generated results log" above.
