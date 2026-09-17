# MAV_CMD_EXTERNAL_WIND_ESTIMATE (cmd=43004) — command protocol tests

Sets an external estimate of wind speed/direction on the EKF wind estimator, intended to extend GPS-denied dead-reckoning time.

**Spec reference**: `mavlink/message_definitions/v1.0/development.xml`, `value=43004`. Not in `common.xml`, so not covered by `test_survey.py`.

## Parameter layout

| Param | Label | Notes |
|-------|-------|-------|
| 1 | Wind speed | m/s, minValue=0. Horizontal wind speed. |
| 2 | Wind speed accuracy | m/s. 1-sigma accuracy; NaN if unknown. |
| 3 | Direction | deg, minValue=0, maxValue=360. Azimuth wind blows FROM (true north). |
| 4 | Direction accuracy | deg. 1-sigma accuracy; NaN if unknown. |
| 5–7 | — | Empty — no MAVLink definition |

`hasLocation="false"`, every defined param a plain float → **COMMAND_LONG** is the primary type (CLAUDE.md's selection rule). Every test sends both COMMAND_INT and COMMAND_LONG (`probe_dual()`) and reports one merged result — the two have agreed on every check in every run to date. With no location, COMMAND_INT's `x`/`y`/`z` (param5/6/7's wire equivalents) default to their "not specified" sentinel (`INT32_MAX`/`INT32_MAX`/NaN).

## Test coverage

The six mandatory common checks (CLAUDE.md) plus, per used param (1–4): a nominal value, boundary values where the spec defines one, and an out-of-range value (expect `DENIED`; XFAIL on PX4 — it clamps/wraps instead of rejecting). See the results table below for per-test outcomes and pass cases.

Every run writes `logs/command_external_wind_estimate_tier1_<autopilot>_<vehicle>_<version>_<timestamp>.log`, always, regardless of pass/fail.

## Confirmed PX4 bug — Commander/EKF2 dual-ACK race — FIXED 2026-09-09

Source-traced and empirically reproduced (PX4 MC 1.18.0-beta-dev, HEAD `c1808fb4`). `Commander::handle_command()` (`src/modules/commander/Commander.cpp`) has an explicit ignore-list of commands "handled by other parts of the system":

```cpp
case vehicle_command_s::VEHICLE_CMD_EXTERNAL_POSITION_ESTIMATE:
case vehicle_command_s::VEHICLE_CMD_EXTERNAL_ATTITUDE_ESTIMATE:
case vehicle_command_s::VEHICLE_CMD_ESTIMATOR_SENSOR_ENABLE:
    /* ignore commands that are handled by other parts of the system: no reply from here */
    return true;

default:
    cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
    break;
```

`VEHICLE_CMD_EXTERNAL_WIND_ESTIMATE` — added to EKF2 in the same family of work as the three siblings above — is missing from this list. Commander falls through to `default:` and answers `UNSUPPORTED(3)` for every send (either message type — both funnel into the same `vehicle_command_s` topic), racing EKF2's own unconditional `ACCEPTED(0)`. Whichever reaches the GCS first is non-deterministic — observed flipping between consecutive sends, and independently between COMMAND_INT and COMMAND_LONG in the same run (e.g. `COMMAND_INT: results=[0, 3]`, `COMMAND_LONG: results=[3, 0]` moments apart). A naive "wait for first ACK" client — exactly what `probe_command_long()`/`probe_command_int()` do — sees this command as randomly unsupported roughly half the time, despite EKF2 genuinely implementing and executing it every time.

**Test design**: `effective_ack()` collects all ACKs in a 1.5s window per message type and prefers a non-`UNSUPPORTED` one when present, testing EKF2's real handling rather than which racing module answered first. The race itself is asserted as a hard `xfail` (both message types) in `test_exactly_one_ack`, reliably reproducing pre-fix.

**Fixed** in commit `793d308c53` (branch `fix_external_wind_estimate_mavlink`): adds the missing `case` to `Commander.cpp`, and makes `Ekf::resetWindToExternalObservation()` return `bool` (`false` when `in_air`), with `EKF2.cpp`'s ACK now `TEMPORARILY_REJECTED` instead of `ACCEPTED` when the reset wasn't applied. Verified: `test_exactly_one_ack` now PASSES; every other Tier 1 result unchanged; Mock and PX4 MC agree on all 18 PASS / 8 XFAIL.

**Follow-up commits on the same branch** (from PR review, 2026-09-17), re-verified against a fresh build at HEAD `ae61d09f9a`:
- `b4a5854c62` — re-sources the landed/in-air gate from `_control_status.flags.in_air` (an EKF-internal flag) to the `vehicle_land_detected` uORB topic directly (with a 3s staleness check), renaming the `resetWindToExternalObservation()` parameter to an explicit `vehicle_landed bool` passed in from `EKF2.cpp`. Behaviourally equivalent for this test suite's purposes — see the updated "Ground vs air" section below.
- `73bcd5fb67` — gates `COMMAND_ACK` publication for this command (and its `SET_GPS_GLOBAL_ORIGIN`/`DO_SET_GLOBAL_ORIGIN` siblings) to only the primary EKF2 instance (`!_multi_mode || (_instance == 0)`) in a multi-EKF configuration, avoiding a second, redundant ACK from a non-primary instance.
- `ae61d09f9a` — pure `astyle` formatting fix (CI `check_format`), no behaviour change.

Re-running the full Tier 1 + Tier 2 suite against this HEAD (single-EKF SIH, so the multi-EKF gating change is not exercised) reproduces the identical 18 PASS / 8 XFAIL Tier 1 result and both Tier 2 tests PASS — see below for the one Tier 2 behavioural difference this HEAD introduces at the ACK level.

## Ground vs air — DOC DISCREPANCY

Tier 1 (ACK-level) tests can't show whether the command changed anything — only Tier 2 (`test_flight.py`), by observing `WIND_COV` (msg 231, mirrors EKF2's internal `wind` uORB topic).

`Ekf::resetWindToExternalObservation()` (`src/modules/ekf2/EKF/wind.cpp`) is gated:

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

The reset — and the flag that makes `WIND_COV` publish at all (`get_wind_status()` = `_control_status.flags.wind || _external_wind_init`) — only fires while landed. Pre-`793d308c53`, the `COMMAND_ACK` path had no equivalent gate: `ACCEPTED` unconditionally in both states, so a GCS could not tell from the ACK whether the estimate was applied.

This is a **DOC DISCREPANCY**: `development.xml`'s own description explicitly targets an in-flight use case ("the command might reasonably be sent every few minutes when operating at altitude"), yet PX4 only honours it while landed — an implementation gap, not a spec problem.

**Empirical confirmation, pre-fix** (PX4 MC 1.18.0-beta-dev, HEAD `c1808fb4`, SIH):

| State | Commanded (speed, dir-from) | Expected (N, E) | Observed WIND_COV | ACK | Result |
|-------|------------------------------|-------------------|---------------------|-----|--------|
| Ground (disarmed) | 8.0 m/s, 90° | (0.00, −8.00) | (−0.00, −8.00) | ACCEPTED(0) | Applied — matches exactly |
| Air (~20 m AGL) | 15.0 m/s, 180° | (15.00, −0.00) | (−0.00, −8.00), unchanged | ACCEPTED(0) | Ignored — stayed at the ground test's stale value |

Both ACKs were `ACCEPTED(0)` — no observable difference at the ACK level, only in `WIND_COV`. Reproduced identically across four independent runs (two separate PX4 boots, before and after the dual-COMMAND-type and test-restructuring changes); the dual-ACK race reproduced too, with the win order flipping between runs, confirming it's genuinely non-deterministic.

Secondary finding: `_external_wind_init` is sticky for the life of the PX4 boot once set on the ground — not cleared by arming/takeoff, so a later "before" `WIND_COV` sample can already carry an earlier ground test's value. The air test accounts for this: it asserts the value doesn't move *toward* what was just commanded, not that `WIND_COV` is absent.

Logs: `logs/command_external_wind_estimate_ground_px4_quadcopter_20260909_150449.log`, `logs/command_external_wind_estimate_air_px4_quadcopter_20260909_150529.log`.

**Re-verified post-fix, 2026-09-17, HEAD `ae61d09f9a`** (fresh build, PX4 MC 1.18.0-beta, SIH) — the ACK-level half of this discrepancy is now closed by `793d308c53`/`b4a5854c62`:

| State | Commanded (speed, dir-from) | Expected (N, E) | Observed WIND_COV | ACK | Result |
|-------|------------------------------|-------------------|---------------------|-----|--------|
| Ground (disarmed) | 8.0 m/s, 90° | (−0.00, −8.00) | (−0.00, −8.00) | ACCEPTED(0) | Applied — matches exactly |
| Air (~20 m AGL) | 15.0 m/s, 180° | (15.00, −0.00) | (−0.00, −8.00), unchanged | **TEMPORARILY_REJECTED(1)** | Not applied — WIND_COV stayed at the ground test's stale value |

The air ACK changed from `ACCEPTED(0)` to `TEMPORARILY_REJECTED(1)` — a GCS can now correctly infer from the ACK alone that the reset was not applied, closing the "ACK says yes, nothing happened" gap this section originally flagged. **What remains open**: PX4 still does not implement the spec's in-flight use case at all (the reset is still unconditionally rejected while airborne, landed-state gate unchanged in effect by `b4a5854c62` — it only changed which uORB topic supplies that state) — that residual gap is a real, and apparently deliberate, functional limitation rather than a doc/implementation mismatch, since the ACK is now honest about it. Per this project's general testing philosophy (root `CLAUDE.md` rule 4a), a `TEMPORARILY_REJECTED` ACK for a value the stack cannot currently act on is a legitimate terminal outcome, not a compatibility error.

Logs: `logs/command_external_wind_estimate_ground_px4_quadcopter_20260917_160432.log`, `logs/command_external_wind_estimate_air_px4_quadcopter_20260917_160512.log`.

## Tier 1 results (PX4 MC HEAD `793d308c53`, fix branch, and Mock — 2026-09-09)

Pre-fix (`main` HEAD `c1808fb4`), `test_exactly_one_ack` was XFAIL on PX4 (the dual-ACK race); every other row was identical pre/post-fix. `test_frame_validation_survey` is COMMAND_INT-only (COMMAND_LONG has no `frame` field) — result: **INCONCLUSIVE** on both Mock and PX4, all 22 `MAV_FRAME` values ACKed, none returned `UNSUPPORTED_MAV_FRAME(9)` — expected, since EKF2's handler never reads `frame` at all.

| Test | Mock | PX4 MC | Pass case |
|------|------|--------|-----------|
| `test_command_ack_received` | PASS | PASS | ACKs (both message types) for a baseline, valid send |
| `test_command_supported` | PASS | PASS | Not UNSUPPORTED for a baseline, valid send |
| `test_exactly_one_ack` | PASS | PASS | Exactly one terminal ACK per send, per message type (XFAIL pre-fix) |
| `test_frame_validation_survey` | INCONCLUSIVE | INCONCLUSIVE | Any MAV_FRAME returns UNSUPPORTED_MAV_FRAME(9) — observational |
| `test_param5_undefined_int32max_accepted` | PASS | PASS | Accepted when param5/x is sent as its own sentinel |
| `test_param5_undefined_nonsentinel_rejected` | XFAIL | XFAIL | Rejected when param5/x is sent a real value — PX4 never reads it |
| `test_param6_undefined_int32max_accepted` | PASS | PASS | Accepted when param6/y is sent as its own sentinel |
| `test_param6_undefined_nonsentinel_rejected` | XFAIL | XFAIL | Rejected when param6/y is sent a real value — PX4 never reads it |
| `test_param7_undefined_nan_accepted` | PASS | PASS | Accepted when param7/z is sent as NaN |
| `test_param7_undefined_nonsentinel_rejected` | XFAIL | XFAIL | Rejected when param7/z is sent a real value — PX4 never reads it |
| `test_param1_defined_nan_not_denied` | PASS | PASS | Not denied when param1 (Wind speed) is sent as NaN |
| `test_param2_defined_nan_not_denied` | PASS | PASS | Not denied when param2 (Wind speed accuracy) is sent as NaN — spec-documented sentinel |
| `test_param3_defined_nan_not_denied` | PASS | PASS | Not denied when param3 (Direction) is sent as NaN |
| `test_param4_defined_nan_not_denied` | PASS | PASS | Not denied when param4 (Direction accuracy) is sent as NaN — spec-documented sentinel |
| `test_param1_nominal` | PASS | PASS | Not UNSUPPORTED at 8.0 m/s |
| `test_param1_zero_boundary` | PASS | PASS | Not UNSUPPORTED at 0.0 m/s (minValue) |
| `test_param1_negative_denied` | XFAIL | XFAIL | Denied at −1.0 m/s (below minValue) — PX4 clamps via `math::max(speed, 0)` |
| `test_param2_specific` | PASS | PASS | Not UNSUPPORTED at 1.5 m/s |
| `test_param2_negative_denied` | XFAIL | XFAIL | Denied at −1.0 (meaningless accuracy) — PX4 squares unconditionally |
| `test_param3_nominal` | PASS | PASS | Not UNSUPPORTED at 90.0 deg |
| `test_param3_zero_boundary` | PASS | PASS | Not UNSUPPORTED at 0.0 deg (minValue) |
| `test_param3_max_boundary` | PASS | PASS | Not UNSUPPORTED at 360.0 deg (maxValue) |
| `test_param3_negative_denied` | XFAIL | XFAIL | Denied at −10.0 deg (below minValue) — PX4 wraps via `wrap_pi()` unconditionally |
| `test_param3_over_max_denied` | XFAIL | XFAIL | Denied at 370.0 deg (above maxValue) — same reason |
| `test_param4_specific` | PASS | PASS | Not UNSUPPORTED at 5.0 deg |
| `test_param4_negative_denied` | XFAIL | XFAIL | Denied at −5.0 (meaningless accuracy) — no sign validation |

18 PASS / 8 XFAIL on both Mock and PX4 MC (post-fix); at the pytest level (`INCONCLUSIVE` reports PASSED) that's 18 passed, 8 xfailed. Every PX4 XFAIL is consistent with source: EKF2 never validates param1–4 range or sign — anything short of a malformed COMMAND_LONG is silently ACCEPTED.

## Tier 2 results (`test_flight.py`)

| Test | PX4 MC (HEAD `793d308c53`) | PX4 MC (HEAD `ae61d09f9a`, 2026-09-17) |
|------|--------|--------|
| `test_ground_wind_estimate_applied` | PASS — WIND_COV moved to the commanded vector, ACK=ACCEPTED(0) | PASS — same |
| `test_air_wind_estimate_ignored` | PASS — WIND_COV stayed at the prior, stale value, ACK=ACCEPTED(0) | PASS — WIND_COV stayed at the prior, stale value, ACK=**TEMPORARILY_REJECTED(1)** |

The air-test ACK value changed between these two HEADs (see "Ground vs air" above for the full explanation) but neither test's assertions needed updating — both already tolerated any non-`UNSUPPORTED` ACK and asserted on `WIND_COV` alone, which is unchanged in either state.

Skipped entirely in paired/mock mode (`require_real_stack`) — `MockFlightStack` has no EKF and doesn't publish `WIND_COV`. Not yet run against ArduPilot or PX4 FW/VTOL/Rover — this is a PX4/EKF2-family command from `development.xml`; ArduPilot's wind estimation is a different subsystem and likely doesn't recognise it, but that's unconfirmed.

## Running

```bash
pytest tests/command/external_wind_estimate/ -v --log-cli-level=INFO   # paired (mock); Tier 2 skipped

pytest tests/command/external_wind_estimate/ \
    --drone-address=udp://:14540 --connection-timeout=60 \
    --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \
    --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO
```
