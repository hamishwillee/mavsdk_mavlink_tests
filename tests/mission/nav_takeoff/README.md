# NAV_TAKEOFF (cmd=22) — Protocol Conformance Tests

Tier 1 (protocol acceptance) and Tier 2 (execution verification) tests for `MAV_CMD_NAV_TAKEOFF` as a mission item.
See the root `CLAUDE.md` for the two-tier testing model.

**Migrated onto `Tier1MissionTestBase`/`MissionItemSpec`** (`tests/mission/conftest.py`) — see `tests/mission/CLAUDE.md` § "Shared Tier 1 infrastructure". Unlike `do_reposition`/`condition_gate` (rejected outright, so every param-level test is skipped), NAV_TAKEOFF is supported everywhere, so the generic `test_defined_param_sentinel_tolerated` test genuinely exercises ArduPilot's NaN-rejection quirk — see `ParamSpec.sentinel_xfail_reason` on params 1/3/7 in `test_protocol.py`.

## Command parameters (MAVLink spec)

| # | Label | Type | Notes |
|---|-------|------|-------|
| 1 | Pitch | float, deg | Minimum pitch for fixed-wing; ignored by multicopters |
| 2 | — | float | **Unused** — spec requires NaN; some stacks reject NaN |
| 3 | Flags | float | `NAV_TAKEOFF_FLAGS` bitmask; bit 0 = HORIZONTAL_POSITION_NOT_REQUIRED |
| 4 | Yaw | float, deg | NaN = use current heading |
| 5 | Latitude | int ×1e7 | INT32_MAX = use current position |
| 6 | Longitude | int ×1e7 | INT32_MAX = use current position |
| 7 | Altitude | float, m | Target altitude AMSL |

## Test files

| File | Tier | Description |
|------|------|-------------|
| `test_protocol.py` | Tier 1 | Baseline + 3 generic sentinel tests + 13 bespoke per-parameter tests |
| `test_flight.py` | Tier 2 | 11 execution tests (arm + fly + assert telemetry; skip without `--drone-address`) |

## Running

```bash
pytest tests/mission/nav_takeoff/ -v --log-cli-level=INFO
pytest tests/mission/nav_takeoff/ --drone-address=udp://:14540 --vehicle-type=quadcopter --autopilot=px4 --px4-sitl=~/github/PX4/PX4-Autopilot -v --log-cli-level=INFO
pytest tests/mission/nav_takeoff/test_protocol.py --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 --ardupilot-sitl=~/ardu_sitl/arducopter --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 --vehicle-type=copter --autopilot=ardupilot -v --log-cli-level=INFO
```

---

## Tier 1 Results — Protocol acceptance

PX4 MC/ArduCopter MC re-verified 2026-09-13 (PX4 v1.18.0-beta, `sihsim_quadx`; ArduCopter V4.8.0-dev/70fe7125) against the migrated framework.
PX4 FW/VTOL and ArduPlane FW/QuadPlane columns are carried over unchanged from the original 2026-05-25 run (not re-verified this session — same source code family, no reason to expect drift, but flagged here for honesty).

`✓` preserved/accepted as expected · `✗` FAIL (spec violation or storage bug) · `~` observational (no assertion) · `→` NACKed (upload rejected)

| Test | PX4 MC | PX4 FW/VTOL¹ | ArduCopter MC | ArduPlane FW/QuadPlane¹ | Mock |
|------|:------:|:------:|:------:|:------:|:------:|
| Command accepted | ✓ | ✓ | ✓ | ✓ | ✓ |
| param1 (Pitch) 15° preserved | ✗ zeroed | ✗ zeroed | ✗ zeroed² | ✓ | ✓ |
| param2 (unused) NaN sentinel accepted | ✓ | ✓ | ✗ rejected | ✗ rejected | ✓ |
| param2 (unused) non-NaN (1.0) rejected | → NACKed³ | ~ accepted, zeroed | ~ accepted, zeroed | ~ accepted, zeroed | → NACKed (xfail — mock has no validation) |
| param3 (Flags) 1.0 preserved | ✗ → NACKed³ | ✗ zeroed | ✗ zeroed | ✗ zeroed | ✓ |
| param3 (Flags) 0 (no flags) | ✓ | ✓ | ✓ | ✓ | ✓ |
| param3 undefined bit (2.0) | → NACKed³ | ~ zeroed | ~ zeroed | ~ zeroed | ~ accepted (NACK preferred) |
| param4 (Yaw) 90° preserved | ✓ | ✓ | ✗ zeroed | ✗ zeroed | ✓ |
| param4 (Yaw) NaN sentinel | ✓ | ✓ | ✗ → 0.0 | ✗ → 0.0 | ✓ |
| param4 (Yaw) 0° ≠ NaN | ✓ | ✓ | ✓ (vacuous⁴) | ✓ (vacuous⁴) | ✓ |
| param4 (Yaw) −90°/450° | ~ wrapped to [0,360) | ~ wrapped | ~ zeroed | ~ zeroed | ~ preserved raw |
| params 5/6/7 (Lat/Lon/Alt) preserved | ✓ | ✓ | ✗ zeroed⁵ | ✓ | ✓ |
| params 5/6 INT32_MAX ("current pos") | ✓ | ✓ | ✗ NACKed | ✗ NACKed | ✓ |
| param7 (Alt) NaN | ~ accepted | ~ | ~ NACKed | ~ | ~ accepted |
| param1 NaN/180°/89°/−10° (edge cases) | ~ zeroed (all, vacuous⁴) | ~ | ~ preserved raw, no bounds | ~ | ~ preserved raw |

¹ Not re-verified this session — see note above.
² ArduCopter's `mavlink_int_to_mission_cmd` normally preserves param1 (Pitch) — see finding below; this session's run found it zeroed instead, alongside location (⁵).
³ **New this session**: PX4 now NACKs param2/param3 non-default values outright (`INVALID_ARGUMENT`) instead of the previously-documented silent accept-and-zero. Source-confirmed: `mavlink_command_params.hpp`'s base mask for cmd 22 (`0x78`) permits only params 4-7; a `VehicleParamOverride` adds param1 for FW/VTOL/MC, but nothing extends params 2/3, so any non-default value there is now rejected — see `CLAUDE.md`.
⁴ Vacuous PASS/observation: the param isn't stored by this stack at all, so any zero-valued or NaN-round-trip result is indistinguishable from "not stored."
⁵ **Anomaly, reproduced twice on a fresh SITL instance** (same firmware git hash, `70fe7125`, as the original passing run): param1 and all of params 5/6/7 came back zeroed. Not a migration artifact, not a test-ordering artifact, and — per this session's source review (`CLAUDE.md`) — not explained by the mission-item source either: ArduPilot's storage/conversion pipeline for both param1 and location is generic and vehicle-type-independent in the exact tested commit, so nothing there predicts a Copter-specific failure. Leading hypothesis: a stale ArduCopter binary vs. its reported git hash (same class of issue as root `CLAUDE.md` item 4c's PX4 lesson) — unconfirmed without rebuilding from source.

Full logs: `logs/mission_nav_takeoff_tier1_px4_quadcopter_1.18.0-beta_20260913_*.log`, `logs/mission_nav_takeoff_tier1_ardupilot_copter_4.8.0-dev_20260913_*.log`; original 2026-05-25 FW/VTOL/QuadPlane logs referenced in `CLAUDE.md`.

### Known ArduPilot storage pattern (unchanged, still explains most FAILs above)

`AP_Mission::mavlink_int_to_mission_cmd` for NAV_TAKEOFF stores only `cmd.p1 = packet.param1`; param3/param4 are never read on upload or written on download. `sanity_check_params()`'s `nan_mask = ~(1<<3)` permits NaN only in param4 — params 1-3 must be a concrete non-NaN value or the whole upload is rejected before the command-specific logic ever runs. INT32_MAX for params 5/6 ("use current position") is rejected outright, a spec violation for a `hasLocation`/`isDestination` command.

---

## Tier 2 Results — Execution verification

**PX4 MC, run 2026-09-13** (first real run — the original README only listed *expected* outcomes; these are now measured):

| Test | Result |
|------|--------|
| `test_takeoff_with_yaw` (renamed `test_takeoff_compat_tracks_yaw`, target 137°) | **FAIL** — heading came back at 1.0° (136° off target, tolerance ±20°). Yaw is correctly *stored* on upload (Tier 1 confirms this) but the vehicle did not turn to it during takeoff execution — a genuinely new finding, not previously tested. **Source-confirmed** (see `CLAUDE.md`): `mission_block.cpp`'s takeoff setpoint-conversion case unconditionally sets `yaw = NAN` when not already flying, with no vehicle-type guard — the same limitation already documented for the COMMAND_INT path (below) also applies to mission-item execution, on MC/FW/VTOL alike. |
| `test_takeoff_obs_with_negative_yaw` / `test_takeoff_obs_with_overflow_yaw` | SKIP — PX4 normalises these on storage (Tier 1), so the conditional Tier 2 pattern correctly judges execution unambiguous and skips |

**ArduCopter**: not run (Tier 2 blocked in this environment — see root `CLAUDE.md` item 7, `is_armable` never goes true). Given param4 is never stored on ArduCopter (Tier 1), `test_takeoff_compat_tracks_yaw` would skip cleanly there regardless.

### PX4 v1.17.0 re-verification, 2026-09-14 (MC full; fixed-wing full; VTOL blocked)

Full test suite rebuilt around root `CLAUDE.md`'s "General testing philosophy for MAV_CMD support" (see `CLAUDE.md`'s dated entry for detail) — tests split cleanly into "is it honoured" (real assertion, `xfail` if the stack accepts-but-ignores) vs. characterisation (observational, edge/sentinel/position values). Tested against a genuine `v1.17.0` release tag build, not a dev branch.

**Table below is from the 11-test suite as it stood earlier on 2026-09-14 — since superseded by a same-day redesign (13 tests: yaw/pitch tests renamed, pitch redesigned from a 5°-vs-45° comparison to a single 10° value not gated on `--vehicle-type`, plus two new position/trajectory characterisation tests) — kept for the record, but pending a re-run against the current test file before being treated as current.** The underlying MC-vs-fixed-wing headline (MC clean, fixed-wing fails to climb at all) is not expected to change; the individual pitch numbers will.

| Test (as named at the time) | PX4 MC | PX4 fixed-wing |
|------|:------:|:------:|
| `test_takeoff_info_implicit_from_waypoint` | ✓ PASS (17.0 m) | ✗ **FAIL** (timeout, never climbed) |
| `test_takeoff_with_yaw` → `test_takeoff_compat_tracks_yaw` (137°, honoured?) | XFAIL — heading 13.6°, not honoured | ✗ FAIL (timeout) |
| `test_takeoff_compat_with_yaw_sentinel` (NaN) | ✓ PASS (observational — heading 11.5°) | ✗ FAIL (timeout) |
| `test_takeoff_obs_with_negative_yaw` (−90°) | XFAIL — not honoured | ✗ FAIL (timeout) |
| `test_takeoff_obs_with_overflow_yaw` (450°) | XFAIL — not honoured | ✗ FAIL (timeout) |
| `test_takeoff_compat_tracks_pitch` → `test_takeoff_compat_tracks_pitch`, redesigned (5° vs 45°, now a single 10°) | XFAIL — peaks 2.4° vs 2.8°, indistinguishable | ✗ FAIL (timeout) |
| `test_takeoff_compat_with_pitch_sentinel` (NaN) | ✓ PASS (observational) | ✗ FAIL (timeout) |
| `test_takeoff_obs_with_large_pitch` (89°) | ✓ PASS (still climbs) | ✗ FAIL (timeout) |
| `test_takeoff_obs_with_negative_pitch` (−10°) | ✓ PASS (still climbs) | ✗ FAIL (timeout) |
| `test_takeoff_obs_with_pitch_overflow` (450°) | ✓ PASS (still climbs) | ✗ FAIL (timeout) |
| `test_takeoff_compat_from_current_position` (INT32_MAX) | ✓ PASS (0.2 m offset) | ✗ FAIL (timeout) |
| `test_takeoff_compat_respects_position` — **new, not yet run against real hardware** | — | — |
| `test_takeoff_obs_ascends_before_lateral_movement` — **new, not yet run against real hardware** | — | — |
| **Total** | **7 PASS, 4 XFAIL, 0 FAIL** | **0 PASS, 0 XFAIL, 11 FAIL** |

**MC**: clean across the board — confirms yaw/pitch are accepted-but-ignored at execution (matching the 2026-09-13 source-level finding, now shown to hold on the released v1.17.0 too, not just the 1.18.0-beta dev build), and that the vehicle correctly takes off in every other scenario including with no explicit NAV_TAKEOFF item present at all.

**Fixed-wing — genuine, unresolved compliance FAIL, not characterisation**: every single Tier 2 test fails identically — arms, mission starts, never reaches even 85% of a modest 20 m target within the 90 s timeout. Tier 1 (protocol acceptance/storage) is unaffected and passes identically to MC, so this is purely an execution-layer gap. Per the general testing philosophy, "the vehicle takes off" is the one behaviour NAV_TAKEOFF's XML text actually mandates — a stack that accepts the item and then never climbs fails that requirement outright, it isn't an "ignored param" case eligible for `xfail`. Not yet root-caused (needs a blind source read of PX4's fixed-wing launch-detection/runway-roll logic before concluding whether this is a SIH-config gap, a missing precondition, or a real regression) — tracked as an open item, not swept into a passing suite.

**VTOL**: blocked by a sandbox-level resource issue that killed even a single isolated Tier 2 test 5 times in a row, unrelated to two other real bugs found and fixed along the way (a Tier 2 log-accumulation bug, and PX4's own console-log growth) — see root `CLAUDE.md` item #10 for the full investigation.

### PX4 v1.17.0 MC — NACK-aware compatibility-error verification, 2026-09-14

Supersedes the "as it stood earlier" MC column above — this is the current 15-test suite (`test_takeoff_compat_respects_position` and `test_takeoff_obs_ascends_before_lateral_movement` now actually run; `xfail` replaced by a plain FAIL, per root `CLAUDE.md` rule 4's revision), and adds one more check per "possibly supported" param: does the stack NACK a non-sentinel value it doesn't honour, per root `CLAUDE.md` rule 4a? Full log: `logs/mission_nav_takeoff_tier2_px4_quadcopter_1.17.0-official_20260914_211414.log`.

**Result: 5 PASS, 3 FAIL, 7 NA.** All three FAILs are the "possibly supported" params (Yaw, Pitch, Lat/Lon) — and for every one of them, PX4 MC did **not** NACK the non-sentinel value it was sent (137° yaw, 10° pitch, a real lat/lon 100 m north of home all uploaded and accepted without complaint). Combined with Tier 2 showing none of the three has any effect at execution, this upgrades all three from a plain "not supported" to a confirmed **compatibility error** — the stack accepted a value it silently cannot act on, rather than rejecting it:

| Param | Tier 1 (uploaded non-sentinel value) | Tier 2 (execution effect) | Verdict |
|---|---|---|---|
| param4 (Yaw), 137° | ACCEPTED (not NACKed) | heading=28.4°, diff=108.6° from target — not honoured | **NOT SUPPORTED — COMPATIBILITY ERROR** |
| param1 (Pitch), 10° | ACCEPTED (not NACKed) | peak `\|pitch\|`=2.6° (need ≥5.0°) — not honoured | **NOT SUPPORTED — COMPATIBILITY ERROR** |
| param5/6 (Lat/Lon), 100 m north | ACCEPTED (not NACKed) | dist_from_target=100.0 m (vehicle stayed at home, dist_from_home=0.9 m) — not honoured | **NOT SUPPORTED — COMPATIBILITY ERROR** |

Everything else stayed as previously found: altitude (param7) SUPPORTED (settled 19.1 m vs commanded 20 m), the INT32_MAX location sentinel SUPPORTED, the pitch NaN sentinel ACCEPTED (no defined meaning to check against), Flags (param3) NOT TESTED (execution semantics not yet designed), Yaw's NaN sentinel NOT TESTABLE (mission protocol can't re-send mid-flight). Fixed-wing not re-run this pass (out of scope — see root `CLAUDE.md` item pending FW root-cause work); VTOL remains blocked (item #10).

### param4 (Yaw) — mission storage vs COMMAND_INT execution

Two distinct paths, with different behaviour. **Mission storage** (PX4): yaw is wrapped to [0°, 360°) rather than clamped or rejected — 90°→90°, 0°→0°, NaN→NaN, −90°→270°, 450°→90°; no pre-normalisation needed by a GCS. ArduPilot never stores param4 for NAV_TAKEOFF at all.

**Direct COMMAND_INT execution** (both stacks, confirmed in source): yaw is ignored outright. PX4's `navigator_main.cpp` unconditionally sets `rep->current.yaw = NAN` for `VEHICLE_CMD_NAV_TAKEOFF`, never reading `cmd.param4` (comment: "Don't set a yaw setpoint for takeoff, as Navigator doesn't handle the yaw reset"). ArduCopter's `handle_MAV_CMD_NAV_TAKEOFF` documents param4 as "(not supported)".

**Mission-item execution shares the same limitation on PX4** (source-confirmed this session, see `CLAUDE.md`): `mission_block.cpp`'s `NAV_CMD_TAKEOFF`/`NAV_CMD_VTOL_TAKEOFF` setpoint-conversion case carries the identical comment and unconditionally sets `sp->yaw = NAN` when not already flying — the same code path is shared by all vehicle types (the block's only vehicle-type check gates a different condition, `already_flying`, for rotary-wing). So despite yaw being correctly *stored* in the mission item (unlike the COMMAND_INT path, which never even reads it), PX4 discards it identically at the point of generating a flight setpoint — a structural finding that should reproduce on FW/VTOL too, not something specific to the multicopter tested here.

---

## Summary

`MAV_CMD_NAV_TAKEOFF` is supported everywhere at the protocol level, with real per-param gaps confirmed in source this session: PX4's mission-item message has no field to carry Pitch at all (structurally discarded on every vehicle type, not just a storage oversight) and never uses mission-stored Yaw at takeoff execution (`mission_block.cpp` unconditionally overrides it to NaN); ArduPilot's storage pipeline is generic and does carry Pitch (ArduPlane genuinely uses it; ArduCopter's executor simply doesn't read it), never stores Flags/Yaw, rejects NaN for any param but Yaw, and rejects the INT32_MAX location sentinel (a spec violation). See "Source comparison" in `CLAUDE.md` for the full writeup. The ArduCopter param1/location anomaly (⁵) remains unresolved — the source review rules out a code-level explanation, strengthening the stale-binary hypothesis.

**Execution (does it actually take off), 2026-09-14**: PX4 MC genuinely takes off in every tested scenario, including with no explicit NAV_TAKEOFF item at all. **PX4 fixed-wing does not** — it accepts the mission and arms, but never climbs, in all 11 Tier 2 tests, a real compliance FAIL against the one thing the spec actually requires, not yet root-caused. VTOL is untested, blocked by an environment issue unrelated to the flight stack (see `CLAUDE.md`).
