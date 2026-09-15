# MAV_CMD_CONDITION_GATE (cmd=4501) — Mission Protocol Conformance Tests

This directory contains Tier 1 (protocol acceptance) and Tier 2 (execution verification) tests for `MAV_CMD_CONDITION_GATE`.

See the root `CLAUDE.md` for the two-tier testing model and `CLAUDE.md` (this directory) for the full source-verification writeup, including a real bug found and fixed in this project's own raw-transport infrastructure while building these tests.

## Finding (read this first)

CONDITION_GATE is tagged `<wip/>` in `common.xml`, which makes MAVSDK's `mission_raw` plugin reject it client-side before it ever reaches the wire — this is the first command in the suite that needs a raw `mavlink_direct` transport (`tests/mission/conftest.py`'s `raw_upload_mission_items()`/`raw_download_mission_items()`) instead.

**PX4** accepts it as a mission item (since v1.11) and correctly implements the gate-crossing mechanism, but never actually stores params 1/2 (Geometry/UseAltitude) — they round-trip as `0.0` regardless of what was uploaded.
A full flight test confirms the mavlink-devguide PR #761 claims about the gate's behaviour directly from telemetry.

**ArduPilot** does not implement this command on any variant — absent from `AP_Mission`'s command-recognition switch (upstream issue: [ardupilot#13778](https://github.com/ArduPilot/ardupilot/issues/13778)).

## Command parameters (MAVLink spec)

| # | Label | Type | Notes |
|---|-------|------|-------|
| 1 | Geometry | float, minValue=0 | Only `0` ("orthogonal to path between previous and next waypoint") is defined; no `enum=` attribute despite this |
| 2 | UseAltitude | float, `enum=MAV_BOOL` | `0`=ignore altitude (2D gate), `1`=include altitude (3D gate); "Values not equal to 0 or 1 are invalid" |
| 3 | — | float | **Empty** |
| 4 | — | float | **Empty** |
| 5 | Latitude | int ×1e7 | Gate location |
| 6 | Longitude | int ×1e7 | Gate location |
| 7 | Altitude | float, m | Gate location |

`hasLocation="true" isDestination="true"` — but per the mavlink-devguide's own description (see cross-check table below) and this suite's flight evidence, the gate is explicitly **not** a route destination despite that attribute; see the "isDestination XML attribute" row in the cross-check table.

## Test files

| File | Tier | Description |
|------|------|-------------|
| `test_protocol.py` | Tier 1 | Baseline + 3 generic sentinel tests + 8 bespoke tests (Geometry, UseAltitude, the params-1/2-dropped finding, location, near-waypoint feasibility) |
| `test_flight.py` | Tier 2 | 1 comprehensive flight test: builds and flies a 6-item mission, verifies path-straightness and trigger-point telemetry (PX4 only — skips everywhere else since ArduPilot doesn't support the command and Tier 2 needs a real vehicle) |

## Running

```bash
# Paired (mock) — Tier 1 passes throughout (mock has no per-command allow-list
# or feasibility logic); Tier 2 skips (no real drone)
pytest tests/mission/condition_gate/ -v --log-cli-level=INFO

# PX4 SIH multicopter — Tier 1 + Tier 2
pytest tests/mission/condition_gate/ \
    --drone-address=udp://:14540 --vehicle-type=quadcopter --autopilot=px4 \
    --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \
    -v --log-cli-level=INFO

# ArduCopter SITL — Tier 1 only (baseline UNSUPPORTED; everything else skips)
pytest tests/mission/condition_gate/test_protocol.py \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
    --ardupilot-sitl=~/ardu_sitl/arducopter --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 \
    --vehicle-type=copter --autopilot=ardupilot -v --log-cli-level=INFO
```

**Prerequisite for PX4**: the SITL binary must be built from a checkout that includes the `mavlink_command_params.hpp` fix for cmd 4501 (mask `0x73`, not `0x00`) — see CLAUDE.md's "Source verification (PX4)" section. If Tier 1's baseline test reports `INVALID_PARAM5_X` instead of `ACCEPTED`, rebuild (`make px4_sitl_default` in the PX4-Autopilot checkout) before re-running.

---

## Tier 1 Results — Protocol acceptance

Tested: 2026-09-13. PX4 v1.18.0-beta (SIH, `sihsim_quadx`), ArduCopter V4.8.0-dev (70fe7125), mock.

| Test | PX4 MC | ArduCopter | Mock |
|------|--------|------------|------|
| Baseline: accepted as a mission item at all | ACCEPTED | UNSUPPORTED¹ | ACCEPTED |
| Undefined param3/4: sentinel (NaN) accepted | ACCEPTED | *(skipped)* | ACCEPTED |
| Undefined param3/4: non-sentinel value rejected | **ACCEPTED — correctly rejected** (`INVALID_PARAM3`/`4`) | *(skipped)* | XFAIL (mock doesn't validate) |
| Defined params 1/2/5/6/7: sentinel tolerated | ACCEPTED (all 5) | *(skipped)* | ACCEPTED (all 5) |
| Geometry (param1) = 0 | ACCEPTED | *(skipped)* | ACCEPTED |
| Geometry (param1) = 7 (undocumented) | ACCEPTED, observational | *(skipped)* | ACCEPTED, observational |
| UseAltitude (param2) = 0 / 1 | ACCEPTED (both) | *(skipped)* | ACCEPTED (both) |
| UseAltitude (param2) = 2 (invalid) | ACCEPTED (NOTE: spec says invalid) | *(skipped)* | ACCEPTED (NOTE) |
| **Params 1/2 round-trip with distinguishable values** | **Never stored — always zeroed regardless of upload** | *(skipped)* | PRESERVED (mock doesn't drop them) |
| Location (params 5/6/7) round-trip | PRESERVED | *(skipped)* | PRESERVED |
| Gate coincident with adjacent waypoint (0 m separation) | **correctly REJECTED** (feasibility check) | *(skipped)* | ACCEPTED (mock has no feasibility logic) |

¹ ArduCopter's baseline uses `param3=0.0, param4=0.0`, not the spec-correct NaN — the same ArduPilot baseline-probe pitfall already documented for [do_reposition](../do_reposition/README.md) and [nav_takeoff](../nav_takeoff/README.md): a NaN baseline gets rejected with `INVALID_PARAM3` (ArduPilot's generic `sanity_check_params()`, which doesn't special-case this unrecognised command) *before* the real "unsupported" finding is reached. See CLAUDE.md.

**Full logs**: `logs/mission_condition_gate_tier1_px4_quadcopter_1.18.0-beta_20260913_123542.log`, `logs/mission_condition_gate_tier1_ardupilot_copter_4.8.0-dev_20260913_124723.log`, `logs/mission_condition_gate_tier1_mock_mock_20260913_121721.log`.

### The params-1/2-drop finding, in detail

Uploading `param1=7.0` (Geometry — not the spec-documented `0`, chosen specifically so it's distinguishable from PX4's zero-initialised default) and `param2=1.0` (UseAltitude=MAV_BOOL_TRUE) downloads as `param1=0.0, param2=0.0` on PX4, every time.
Source-confirmed (see CLAUDE.md): PX4's mission upload-parse switch has a case for `CONDITION_GATE` that sets only `nav_cmd`, never copying these two params into the stored item; the download-format switch has no case for it at all.
This is consistent with, not a correction of, mavlink-devguide PR #761's "UseAltitude field ignored" claim — whether a value is dropped on storage or read-and-discarded at execution is not observable from a GCS, and either way the param has no effect.
Testing with `0.0` alone (the "valid and spec-correct" value) would be a **vacuous PASS** here — `0.0` is simultaneously a legitimate value and the always-zeroed default, so round-tripping to `0.0` proves nothing either way (see `test_geometry_only_value_zero_accepted`'s docstring, and the "Vacuous PASS" convention in `tests/mission/CLAUDE.md`).

---

## Tier 2 Results — Execution verification (PX4 only)

Tested: 2026-09-13, PX4 v1.18.0-beta (SIH, `sihsim_quadx`), `MPC_XY_CRUISE`=5.0 m/s default.

Mission: takeoff → wp1 (50 m north of home) → gate (25 m east of the direct line, projecting to the leg's midpoint at 120 m north) → `DO_CHANGE_SPEED` (2.0 m/s) → wp2 (190 m north) → RTL, uploaded via the raw transport and started with a raw `MAV_CMD_MISSION_START`.

| Assertion | Result (range across 5 runs) |
|-----------|--------|
| Max cross-track deviation from the direct wp1→wp2 line | **0.38–0.45 m** (tolerance 8.0 m; gate offset 25.0 m) |
| Closest approach to the gate's own coordinates | **24.97–25.01 m** (tolerance: must exceed 10.0 m) |
| North coordinate where groundspeed first drops below the cruise/reduced-speed midpoint | **116.2–116.5 m** (gate projects to 120.0 m; leg spans 50–190 m) |

All three assertions passed with wide margins on every run, consistently within ~1% of each other — no flakiness observed.

**Timing budget**: the sampling window is derived from the *measured* `MPC_XY_CRUISE` value plus a fixed, environment-overridable climb/transit allowance (`CONDITION_GATE_CLIMB_ALLOWANCE_S`, default 25 s) rather than a flat guess — see CLAUDE.md and the file's own module docstring for the reasoning. All 5 runs detected the trigger by t=48.6–48.7s of a ~105s budget (roughly half the budget unused); if a future run against a different/slower stack runs out of sampling time before reaching the relevant part of the mission, the test reports this distinctly ("LIKELY INSUFFICIENT SAMPLING TIME FOR THIS STACK") rather than as a behavioural failure.

**Tier 1 gates Tier 2**: the flight test itself attempts the mission upload first and skips cleanly (not errors) if the stack rejects it outright — confirmed against ArduCopter (`UNSUPPORTED`, 27 s to skip, never arms).

Full logs: `logs/mission_condition_gate_flight_px4_quadcopter_1.18.0-beta_20260913_140023.log` (PX4, latest run), `logs/mission_condition_gate_flight_ardupilot_copter_4.8.0-dev_20260913_140325.log` (ArduCopter, skip confirmation).

---

## mavlink-devguide PR #761 cross-check

[mavlink/mavlink-devguide#761](https://github.com/mavlink/mavlink-devguide/pull/761) ("Mission item - separate out to own doc", co-authored by Claude, open/not-yet-test-verified when this suite was written) documents CONDITION_GATE's behaviour. This is the "more precise implementation section" the PR author told a reviewer was coming.

| PR #761 claim | This suite's finding | Verdict |
|----------------|----------------------|---------|
| "marks an off-path location (not a destination)... the vehicle flies directly towards the next mission item" | Max cross-track deviation 0.38 m over a 25 m gate offset (Tier 2) | **CONFIRMED** |
| "The mission state machine is blocked on the gate mission item until the vehicle reaches the point on the path that is perpendicular to the gate location" | `DO_CHANGE_SPEED` triggers at 116.2–116.5 m across 5 runs, near the naive path-perpendicular projection (120.0 m) — but see the refinement below | **CONFIRMED, with a refinement worth feeding back** |
| — refinement: which line is "perpendicular"? | Source (`mission_block.cpp`'s `NAV_CMD_CONDITION_GATE` branch): the crossing test is `dot(vehicle − gate, normalize(next_real_waypoint − gate)) ≥ 0` — a plane through the *gate's own position*, perpendicular to the **gate→next-waypoint** vector, not to the inbound path. For an off-path gate these differ: in this test's geometry the true trigger plane works out to north≈111 m (not 120 m), and the observed ~116 m detection is consistent with that once ~5 m of deceleration lag is added back (see `CLAUDE.md` § blind-source-review finding) | **New nuance, not a compliance issue** — PR #761's prose/diagram don't distinguish "perpendicular to the path" from "perpendicular to gate→next-waypoint"; worth sharpening if revisited |
| "UseAltitude field ignored [by PX4]... geometry test is 2D" | The value never survives upload into the stored item at all — consistent with "ignored" (whether dropped on storage or read-and-discarded at execution makes no observable difference) (Tier 1) | **CONFIRMED** |
| "PX4: Supported from PX4v1.11" | Accepted as a mission item on PX4 v1.18.0-beta once the local `mavlink_command_params.hpp` mask fix is built in; rejected with `INVALID_PARAM5_X` on a stale binary predating that fix (Tier 1) | **CONFIRMED** (with the caveat that this specific checkout needed a rebuild — see CLAUDE.md) |
| "A gate within 5 cm of a neighboring waypoint is rejected as infeasible" | 0 m separation correctly rejected on PX4; mock accepts it (no feasibility logic) (Tier 1) | **CONFIRMED** |
| "ArduPilot: Not supported. See ardupilot#13778" | `MAV_MISSION_UNSUPPORTED` on ArduCopter, source-confirmed absent from `AP_Mission`'s command switch (Tier 1) | **CONFIRMED** |
| XML `isDestination="true"` | Contradicted by the PR's own prose ("not a destination") and by this suite's flight evidence (the vehicle never routes to the gate's coordinates) | **Spec attribute itself looks wrong** — worth raising as a `mavlink/mavlink` issue per root `CLAUDE.md`'s "MAVLink spec discrepancies" convention, independent of the devguide PR |

No claim in the PR was refuted or found inconclusive by this suite.
