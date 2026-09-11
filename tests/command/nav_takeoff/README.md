# MAV_CMD_NAV_TAKEOFF — Command Protocol (COMMAND_INT) Test Results

Tests the **command protocol** path (COMMAND_INT → COMMAND_ACK), distinct from the **mission protocol** path (MISSION_ITEM_INT upload → storage) covered by `tests/mission/nav_takeoff/`.

NAV_TAKEOFF has `hasLocation="true"`/`isDestination="true"` → COMMAND_INT is the correct message type (see `../CLAUDE.md` § COMMAND_INT vs COMMAND_LONG). NaN encoding and the general mission-vs-command param-handling difference are also covered there; NaN tests here skip in mock mode (the mock's ACCEPTED doesn't reflect real param handling).

## Lat/lon sentinel values

Coordinate fields aren't always meaningful lat/lon — they may carry sentinels:

| Sentinel | Message type | Meaning | PX4 behaviour |
|----------|-------------|---------|----------------|
| `x=INT32_MAX, y=INT32_MAX` | COMMAND_INT | "use current position" | MavlinkReceiver converts to NaN (`param5/6`, lines 611–614); navigator falls through to current position |
| `param5=NaN, param6=NaN` | COMMAND_LONG | "use current position" | `PX4_ISFINITE(NaN)=false` → falls through to current position |
| `param5≈INT32_MAX, param6≈INT32_MAX` | COMMAND_LONG | **Protocol error** | PX4 explicitly DENIES (`mavlink_receiver.cpp:499–505`) — treated as a miscoded COMMAND_INT |
| `param7=NaN` (altitude) | Both | "use system default" | `PX4_ISFINITE(NaN)=false` → uses `current_alt + MIS_TAKEOFF_ALT` |

PX4 correctly converts INT32_MAX→NaN for COMMAND_INT and passes NaN through for COMMAND_LONG; sending `float(INT32_MAX)` in COMMAND_LONG is treated as a protocol error, not the sentinel.

## Yaw ignored via COMMAND_INT

Both PX4 (`navigator_main.cpp:630`: `rep->current.yaw = NAN` unconditionally) and ArduPilot (`GCS_MAVLink_Copter.cpp:585`: "not supported"; ArduPlane reads only altitude) ignore param4 in the COMMAND_INT path — see `../CLAUDE.md` § Command vs mission protocol differences for the full comparison with the mission-protocol path (where PX4 *does* store and use it).

## Tier 1 (ACK) results

23 tests: `test_command_ack_received`, `test_command_supported`, `test_exactly_one_ack`, `test_frame_validation_survey`, `test_undefined_param_{sentinel_accepted,nonsentinel_rejected}[param2]`, and `test_defined_param_sentinel_tolerated[param{1,3,4,5,6,7}]` are inherited from `Tier1CommandTestBase` (`../CLAUDE.md`); the rest are this command's own bespoke tests. All COMMAND_INT except the two rows marked COMMAND_LONG.

### PX4 MC / FW / VTOL / Rover (1.18.0-beta, re-verified 2026-09-11)

NAV_TAKEOFF SUPPORTED on every vehicle type — PX4 doesn't gate by vehicle type (unlike ArduRover's UNSUPPORTED, below). Byte-identical results across all four vehicle types **except** `test_param1_pitch_ack_denied`, where **PX4 Rover alone actually DENIES** a non-NaN pitch (MC/FW/VTOL ignore it, same as before): 19 PASS/4 XFAIL (MC/FW/VTOL), 20 PASS/3 XFAIL (Rover). New finding from this run: PX4 genuinely validates the undefined param2 slot — a non-sentinel value is DENIED on all four vehicle types, unlike the mock's accept-everything default.

| Test | PX4 MC / FW / VTOL | PX4 Rover |
|------|---------------------|-----------|
| `test_command_ack_received` / `test_command_supported` | PASS — ACCEPTED | PASS — ACCEPTED |
| `test_exactly_one_ack` | PASS | PASS |
| `test_frame_validation_survey` | INCONCLUSIVE — all 22 frames ACKed | INCONCLUSIVE |
| `test_undefined_param_sentinel_accepted[param2]` | PASS — ACCEPTED | PASS — ACCEPTED |
| `test_undefined_param_nonsentinel_rejected[param2]` | PASS — DENIED | PASS — DENIED |
| `test_defined_param_sentinel_tolerated[param1,3,4,5,6,7]` | PASS ×6 — ACCEPTED | PASS ×6 — ACCEPTED |
| `test_param1_pitch_ack_denied` (param1=15°) | **XFAIL** — ACCEPTED; pitch ignored (spec violation) | **PASS** — DENIED |
| `test_param1_nan_ack_result` | PASS (obs) — ACCEPTED | PASS (obs) |
| `test_param4_yaw_ack_denied` (param4=90°) | **XFAIL** — ACCEPTED; yaw ignored | **XFAIL** — ACCEPTED |
| `test_param4_yaw_nan_ack` | PASS (obs) — ACCEPTED | PASS (obs) |
| `test_location_specific_ack` | PASS — ACCEPTED | PASS — ACCEPTED |
| `test_location_int32max_ack` | PASS (obs) — ACCEPTED | PASS (obs) |
| `test_location_out_of_range_latlon_ack` | **XFAIL** — ACCEPTED; PX4 doesn't validate lat/lon range (spec gap) | **XFAIL** — ACCEPTED |
| `test_nan_altitude_ack` | PASS (obs) — ACCEPTED | PASS (obs) |
| `test_wrong_frame_ack` | PASS (obs) — ACCEPTED | PASS (obs) |
| `test_latlon_nan_command_long_ack` (COMMAND_LONG) | PASS — ACCEPTED; navigator uses current position when `PX4_ISFINITE(param5)=false` | PASS |
| `test_latlon_int32max_command_long` (COMMAND_LONG) | **XFAIL** — DENIED; `mavlink_receiver.cpp:499` explicitly rejects the sentinel as a protocol error | **XFAIL** — DENIED |

### ArduCopter MC / ArduPlane FW / ArduPlane QP

**Stale** — last verified 2026-05-27 against the pre-migration 10-test suite (8 PASS, 2 XFAIL, 0 SKIP; SUPPORTED on all three; ArduPlane QP inferred identical to FW, not independently tested; ArduPlane ignores lat/lon/pitch/yaw entirely, reading only altitude, but still rejects out-of-range coordinates at the command-handler level). Not re-verified against the current 23-test suite in this environment: ArduCopter SITL here shows an intermittent-connection issue independent of the documented `is_armable` boot problem (Tier 1 doesn't arm) — the first probe got a real but COMMAND_INT/COMMAND_LONG-inconsistent ACK (`DENIED(2)` vs `FAILED(4)`), then every subsequent send got no ACK at all. Needs a healthy SITL instance to re-verify.

### ArduRover

**UNSUPPORTED** (ground vehicle, re-verified 2026-09-11) — baseline probe returns `MAV_RESULT_UNSUPPORTED(3)`; `_ensure_supported()` skips the remaining 22 tests.

### Mock (paired, re-verified 2026-09-11)

18 PASS, 2 SKIP, 3 XFAIL. Mock ACCEPTs everything by default except out-of-range lat/lon (outside ±900_000_000/±1_800_000_000, non-INT32_MAX) and the two COMMAND_LONG-only tests (SKIP — mock doesn't model NaN-lat/lon or INT32_MAX-as-float semantics for COMMAND_LONG).

| Test | Result |
|------|--------|
| `test_command_ack_received` / `test_command_supported` | PASS — ACCEPTED |
| `test_exactly_one_ack` | PASS |
| `test_frame_validation_survey` | INCONCLUSIVE — mock accepts all frames |
| `test_undefined_param_sentinel_accepted[param2]` | PASS — ACCEPTED |
| `test_undefined_param_nonsentinel_rejected[param2]` | **XFAIL** — mock accepts everything by default, no undefined-param validation |
| `test_defined_param_sentinel_tolerated[param1,3,4,5,6,7]` | PASS ×6 — ACCEPTED |
| `test_param1_pitch_ack_denied` | **XFAIL** — mock ignores pitch too |
| `test_param1_nan_ack_result` / `test_param4_yaw_nan_ack` / `test_nan_altitude_ack` | PASS (obs) — ACCEPTED |
| `test_param4_yaw_ack_denied` | **XFAIL** — yaw ignored |
| `test_location_specific_ack` | PASS — ACCEPTED |
| `test_location_int32max_ack` | PASS (obs) — ACCEPTED |
| `test_location_out_of_range_latlon_ack` | PASS — DENIED (mock validates range) |
| `test_wrong_frame_ack` | PASS (obs) — ACCEPTED |
| `test_latlon_nan_command_long_ack` / `test_latlon_int32max_command_long` | SKIP — requires real stack |

## Tier 2 flight tests (`test_flight.py`)

Arms, sends `NAV_TAKEOFF` via raw COMMAND_INT, observes telemetry. A two-stage gate runs first: (1) ACK probe — skip all if UNSUPPORTED; (2) execution probe — arm, send, wait ≤20s for climb >0.5m; skip all 17 execution tests if accepted-but-not-executed.

PX4 ignores COMMAND_INT's `frame` field and always treats `z` as absolute AMSL altitude — `_arm_and_send_takeoff()` converts the caller's relative altitude to absolute (home AMSL + relative) and forces frame=5 to match.

Per-stack summary blocks below (marked `TIER2_SUMMARY_START/END`) are auto-written by `_write_and_update_readme()` in `test_flight.py` on each run — do not hand-edit their content.

### PX4 MC (1.18.0)

<!-- TIER2_SUMMARY_START px4-quadcopter -->
**Last run:** 2026-06-03 12:54  **Firmware:** 1.18.0

**Preconditions:**
- Initial mode: no mode change required
- Command arms vehicle: False — must pre-arm
- Mode on NAV_TAKEOFF receipt: TAKEOFF (no change)

**Takeoff approach that worked:**
- COMMAND_INT with lat/lon
- Probe sequence: tier1: COMMAND_INT+lat/lon: ACCEPTED →CLIMBED

**Flight:**
- Climbs diagonally from ground — no vertical-first phase.
  Navigates toward specified lat/lon/alt.
  Holds at waypoint (HOLD mode).
  Yaw: param4=90° IGNORED (heading=4°).
  Ignored: yaw (param4), pitch (param1).


<!-- TIER2_SUMMARY_END px4-quadcopter -->
Command tests: 7 PASS, 3 XFAIL (yaw, pitch, out-of-range lat/lon). Flight tests (2026-06-02): 22 PASS, 3 FAIL, 1 SKIP, 3 XFAIL, 2 XPASS.

The 3 FAILs (`test_unarmed_takeoff`, `test_required_flight_mode`, `test_mode_after_takeoff`) hit at the end of the session after 14 consecutive arm-takeoff-RTL cycles — PX4 SIH degrades after many cycles; operational, not protocol failures. XPASS: `test_altitude_zero_behaviour` (safety minimum applied) and `test_position_zero_treated_as_current` (PX4 navigates to (0,0) — a spec-expected FAIL that passed instead).

`test_px4_mc_takeoff_comprehensive` (separate, target 200m north, param4=90°) confirms PX4 MC genuinely navigates toward the specified lat/lon (184m→106m→34m from target as altitude climbs from 2m→15m→26m) while still ignoring param4.

| Test | Param | Result | Observation |
|------|-------|--------|-------------|
| `test_altitude_nominal` | z=30m rel | **PASS** | Reached 25.5m (≥85%) |
| `test_altitude_higher` | z=50m rel | **PASS** | Reached 42.5m |
| `test_altitude_very_low` | z=0.5m rel | **PASS** | Reached 0.74m — safety minimum ~0.74m |
| `test_altitude_nan_uses_default` | z=NaN | **PASS** | Took off; reached 0.5m+ |
| `test_altitude_zero_behaviour` | z=0.0 abs | **XPASS** | Reached 0.26m — safety minimum applied |
| `test_yaw_*` (all 7 variants) | param4=0°..3600° | PASS (obs) | Heading stays ≈351–354° regardless — param4 ignored, uses pre-arm heading |
| `test_position_specific` | x/y=home | **PASS** | Vehicle arrives at home coords — confirms lat/lon IS the target |
| `test_position_int32max_stays_at_home` | x/y=INT32_MAX | **PASS** | → "use current position"; stays within 0.7m of home at 2.0m alt |
| `test_position_zero_treated_as_current` | x=0,y=0 | **FAIL** | (0,0) is a valid equatorial coordinate, not a sentinel — PX4 navigates there |
| `test_pitch_comparison_low_vs_high` | param1=5° vs 45° | PASS (obs) | Unsupported — both logged 0° peak pitch |
| `test_mode_after_takeoff` | — | PASS (obs) | Mode: Unknown — timed out after 16 prior cycles |

**Behaviour summary**: command accepted (PASS); altitude respected (≥85% reached, NaN→default ~0.5–0.7m, z=0→safety minimum ~0.26m); yaw/pitch not supported (ignored); **lat/lon IS used as the target destination** — vehicle climbs toward it simultaneously, not vertical-first; INT32_MAX→current position; mode transitions TAKEOFF→HOLD on arrival.

### PX4 FW (1.18.0)

<!-- TIER2_SUMMARY_START px4-fixed_wing -->
**Last run:** 2026-06-02 21:37  **Firmware:** 1.18.0

**Preconditions:**
- Initial mode: no mode change required
- Command arms vehicle: False — must pre-arm
- Mode on NAV_TAKEOFF receipt: HOLD → TAKEOFF

**Takeoff approach that worked:**
- none — see probe results
- Probe sequence: tier1: COMMAND_INT+lat/lon: ACCEPTED →no climb; tier2: COMMAND_LONG+alt-only: ACCEPTED →no climb; tier3: COMMAND_INT (unarmed): ACCEPTED →no climb

**Flight:**
- Did not become airborne — ground movement detected (altitude < 2 m, SIH FW runway roll).


<!-- TIER2_SUMMARY_END px4-fixed_wing -->

Command tests: 7 PASS, 3 XFAIL — same as PX4 MC. Flight tests: 8 PASS (7 command + 1 comprehensive observing ground roll), 20 SKIP, 3 XFAIL. `test_mc_takeoff_comprehensive` (vehicle_type=fixed_wing) confirms the SIH FW simulator performs a ground roll but never lifts off (altitude < 2m) — a simulator limitation, not a protocol issue.

### ArduCopter MC (4.8.0)

<!-- TIER2_SUMMARY_START ardupilot-quadcopter -->
**Last run:** 2026-06-02 21:26  **Firmware:** 4.8.0

**Preconditions:**
- Initial mode: GUIDED mode (confirmed)
- Command arms vehicle: False — must pre-arm
- Mode on NAV_TAKEOFF receipt: OFFBOARD (no change)

**Takeoff approach that worked:**
- COMMAND_INT with lat/lon
- Probe sequence: tier1: COMMAND_INT+lat/lon: ACCEPTED →CLIMBED

**Flight:**
- Climbs diagonally from ground — no vertical-first phase.
  Does not navigate toward specified lat/lon (lat/lon ignored).
  Yaw: param4=90° IGNORED (heading=0°).
  Ignored: yaw (param4), pitch (param1).


<!-- TIER2_SUMMARY_END ardupilot-quadcopter -->

**GUIDED mode required**: `has_user_takeoff(must_navigate=true)` — only GUIDED returns true (see `../CLAUDE.md` § ArduCopter mode restriction). The execution probe would normally arm→check-climb, but that leaves the vehicle in STABILIZE (always no-climb) and leaks a dangling `telemetry.health()` stream (§4a); `_set_executes_cache_for_known_modes` pre-sets `_nav_takeoff_executes=False` for ardupilot/quadcopter to bypass it.

Other execution notes: requires `_request_position_stream()` first (ArduCopter doesn't stream `GLOBAL_POSITION_INT` without an explicit `MAV_CMD_SET_MESSAGE_INTERVAL(511)`); reads only `packet.z` — lat/lon ("not supported") and yaw/pitch are ignored; the handler requires `frame==MAV_FRAME_GLOBAL_RELATIVE_ALT` (3).

Command tests: 8 PASS, 2 XFAIL (pitch, yaw — both ACCEPTED not DENIED). Flight tests: 11 PASS (10 command-mode-gated SKIPs + 1 comprehensive), 20 SKIP — the 17 standard flight tests skip (no-climb without GUIDED setup); `test_mc_takeoff_comprehensive` runs and PASSES via the tiered probe.

**Behaviour summary**: command accepted (PASS); altitude respected via z/param7; lat/lon and yaw/pitch not supported (ignored, vehicle climbs vertically at home); requires GUIDED mode; mode stays GUIDED/OFFBOARD throughout, no auto-transition.

### ArduPlane FW (4.8.0)

<!-- TIER2_SUMMARY_START ardupilot-fixed_wing -->
**Last run:** 2026-06-02 21:33  **Firmware:** 4.8.0

**Preconditions:**
- Initial mode: TAKEOFF mode (mode 13) — set via DO_SET_MODE before arming
- Command arms vehicle: False — arm() is called after TAKEOFF mode is set; COMMAND_INT NAV_TAKEOFF is NOT used (returns FAILED for non-QuadPlane fixed-wing)
- Mode on NAV_TAKEOFF receipt: N/A — takeoff is triggered by TAKEOFF mode + arm, not by the NAV_TAKEOFF command

**Flight:**
- The plane takes off in its initial direction, ignoring yaw, pitch, lat, and lon (source: ArduPlane do_takeoff() overwrites lat/lon with home±10 units).
  Pitch: TKOFF_PITCH_MIN=5°→peak=19.5°  TKOFF_PITCH_MIN=45°→peak=17.6° — peak pitch unchanged — TECS pitch dominates minimum (param1 ignored in practice).
  Acceptance on reaching TKOFF_ALT — then loiters within TAKEOFF mode.
  Mode transition: NOT automatic (test switches to GUIDED to observe post-takeoff position).
  Pre-arm requirements: TAKEOFF mode (mode 13) + arm.


<!-- TIER2_SUMMARY_END ardupilot-fixed_wing -->

**COMMAND_INT NAV_TAKEOFF is not the execution path** — ArduPlane FW takes off via `DO_SET_MODE TAKEOFF(13)` + arm (full-throttle takeoff controller), not via the command. `do_takeoff()` (`commands_logic.cpp`) overwrites x/y with `home.lat+10 / home.lng+10`; pitch is controlled by the `TKOFF_PITCH_MIN` param, not `param1` (only the mission path feeds that). `_ensure_nav_takeoff_supported` skips the 17 regular tests (ACCEPTED but no climb); `test_arduplane_guided_takeoff_to_target` and `test_mc_takeoff_comprehensive` both run and PASS.

Command tests: 8 PASS, 2 XFAIL. Flight tests: 12 PASS (8 command + 2 FW-specific), 19 SKIP.

### PX4 VTOL (1.18.0)

<!-- TIER2_SUMMARY_START px4-vtol -->
**Last run:** 2026-06-02  **Firmware:** 1.18.0

**Command accepted:** NAV_TAKEOFF (22) via COMMAND_INT returns ACCEPTED.
PX4 does not gate commands by vehicle type.

**Execution:** NAV_TAKEOFF executes on PX4 VTOL in the **MC hover phase** (same behaviour as PX4 MC for COMMAND_INT).
The vehicle climbs vertically using VTOL motors.
The VTOL-specific ``NAV_VTOL_TAKEOFF (84)`` command triggers the full VTOL sequence (MC hover → FW align → FW transition → FW climb); see `tests/command/baseline_takeoff/README.md`.

For NAV_TAKEOFF (22), the vehicle behaviour is identical to PX4 MC: diagonal climb to target lat/lon/alt, TAKEOFF → HOLD mode transition on arrival.
Yaw (param4) and pitch (param1) are ignored.
<!-- TIER2_SUMMARY_END px4-vtol -->

Command tests: 7 PASS, 3 XFAIL — identical to PX4 MC. Flight tests: 21 PASS, 3 FAIL (same late-session SITL degradation as PX4 MC), 2 SKIP (`test_mc_takeoff_comprehensive` excludes vtol; `test_arduplane_guided_takeoff_to_target` is ardupilot-only), 3 XFAIL, 2 XPASS.

Behaviour: identical to PX4 MC (altitude respected, lat/lon used as target, yaw/pitch ignored) — `NAV_VTOL_TAKEOFF(84)` is the preferred VTOL-specific command, not this one; see `baseline_takeoff/README.md`.

### PX4 Rover (1.18.0)

<!-- TIER2_SUMMARY_START px4-rover -->
**Last run:** 2026-06-02  **Firmware:** 1.18.0

**NAV_TAKEOFF does not execute on PX4 Rover.**

NAV_TAKEOFF (22) returns ACCEPTED (PX4 does not gate commands by vehicle type), but the rover cannot fly.
The execution probe detects no climb within 20 s and skips all 17 flight tests.

This contrasts with ArduRover where NAV_TAKEOFF returns UNSUPPORTED (3).
PX4's permissive command handling is a design choice but may be considered a protocol gap — a ground vehicle accepting a flight command without executing it or returning UNSUPPORTED is misleading.
<!-- TIER2_SUMMARY_END px4-rover -->

Command tests: 7 PASS, 3 XFAIL — ACCEPTED, same as PX4 MC. Flight tests: 19 SKIP (no climb) + 2 SKIP (comprehensive/arduplane excluded). Gap: command accepted but silently inert on a vehicle that can't fly — should arguably return UNSUPPORTED or FAILED.

### ArduPlane QP (4.8.0)

<!-- TIER2_SUMMARY_START ardupilot-quadplane -->
**Last run:** 2026-06-02  **Firmware:** 4.8.0

**NAV_TAKEOFF does not execute via COMMAND_INT without GUIDED mode.**

NAV_TAKEOFF (22) returns ACCEPTED, but ArduPlane QuadPlane requires GUIDED mode (custom_mode=15) before the command executes.
Without GUIDED mode, the handler in ArduPlane returns ACCEPTED but no takeoff occurs.
The correct sequence is documented in `tests/command/baseline_takeoff/README.md`: GUIDED (15) → arm → COMMAND_LONG NAV_TAKEOFF p7=altitude.

NAV_VTOL_TAKEOFF (84) is a mission-only command on ArduPlane QuadPlane (executed in AUTO mode); it cannot be sent as a direct COMMAND_INT.
<!-- TIER2_SUMMARY_END ardupilot-quadplane -->

Command tests: 10 PASS (incl. 2 XFAIL for pitch+yaw) — ACCEPTED. Flight tests: 19 SKIP (no climb without GUIDED setup) + 2 SKIP (comprehensive/arduplane-guided tests exclude quadplane).

### ArduRover (4.8.0)

<!-- TIER2_SUMMARY_START ardupilot-rover -->
**Last run:** 2026-06-02  **Firmware:** 4.8.0

**NAV_TAKEOFF is UNSUPPORTED on ArduRover.**  Result: `MAV_RESULT_UNSUPPORTED (3)`.

ArduRover is a ground vehicle; the command is explicitly rejected.
All 31 tests skip.
This is the correct behaviour per the MAVLink spec.
<!-- TIER2_SUMMARY_END ardupilot-rover -->

Command tests: 10 SKIP (survey-gated, UNSUPPORTED). Flight tests: 21 SKIP.

### Baseline tests (`baseline_takeoff/test_baseline.py`)

See `baseline_takeoff/README.md` for the full mode-restriction analysis (which modes accept NAV_TAKEOFF in code vs which actually execute autonomously).

| Test | Stack | Sequence | Result (2026-06-02) |
|------|-------|----------|---------------------|
| `test_px4_mc_takeoff_baseline` | PX4 MC | `action.arm()` → COMMAND_INT NAV_TAKEOFF (frame=5, z=abs) | **PASS** — reached 17.0m |
| `test_ardupilot_mc_takeoff_baseline` | ArduCopter MC | GUIDED → arm via COMMAND_LONG 400 → COMMAND_LONG NAV_TAKEOFF p7=alt | **PASS** — reached 17.1m |

## Spec gaps

Undefined/ambiguous behaviours for NAV_TAKEOFF via COMMAND_INT, all documented by `test_flight.py`:

- **Unsupported params silently ACCEPTed instead of DENIED** (general principle): the spec never states that a stack must DENY a non-NaN value for a param it doesn't support — but `NaN` is the universal "no preference" sentinel, so a non-NaN value expresses real intent, and silently discarding it violates that contract. Concretely: **param1** (pitch) and **param4** (yaw) are both ignored-but-ACCEPTED by every tested stack (PX4, ArduPilot) — tracked as xfail in `test_param1_pitch_ack_denied`/`test_param4_yaw_ack_denied`. ArduPlane also ignores lat/lon this way (`convert_MAV_CMD_NAV_TAKEOFF_to_COMMAND_INT` hardcodes x=0,y=0). Suggest: the spec should require `DENIED` for a non-NaN value on any param the stack can't honour, as a general command-protocol rule.
- **param1 (MinPitch) range** — no min/max defined; values outside `[0,90]°` are accepted everywhere. For fixed-wing specifically, no vehicle-type-aware range exists either — 90° is physically impossible but nothing rejects it. Suggest a defined range, e.g. fixed-wing `[0°,30°]`, MC/VTOL NaN-only (non-NaN → DENIED or ignored).
- **param4 (Yaw) range/normalisation** — no rule for negative values, wraparound (>360°), or very large values (e.g. 3600°); moot in practice since COMMAND_INT ignores param4 entirely on every tested stack, but the spec should still clarify.
- **param7 (Altitude)** — no minimum defined; `z=0` behaviour (safety minimum vs reject vs hover) and the NaN default are both implementation-defined.
- **param5/6 (Lat/Lon) = 0** — whether `(0,0)` is a valid coordinate or a "use current position" sentinel isn't specified; most stacks treat it as current position, undocumented.
- **param5/6 out-of-range** — no defined valid range or rejection requirement for geometrically impossible coordinates. ArduPilot correctly DENIES; PX4 ACCEPTs (bug, xfail in `test_location_out_of_range_latlon_ack`). Suggest requiring `DENIED` outside `[±900_000_000]`(lat)/`[±1_800_000_000]`(lon), excepting the `INT32_MAX` sentinel.
- **Position semantics** — the spec doesn't say whether x/y is the takeoff-from point or the arrival target. Empirically (PX4 MC): it's the **target** — the vehicle climbs toward it simultaneously rather than climbing vertically first.
- **Command completion** — no defined completion condition or expected post-takeoff mode. PX4 MC transitions to HOLD on arrival; ArduPlane's TAKEOFF mode internally loiters. Suggest: "reached commanded altitude within lat/lon tolerance" + transition to a station-keeping mode.

## Running

```bash
pytest tests/command/nav_takeoff/test_command.py -v --log-cli-level=INFO   # mock, tier 1

pytest tests/command/nav_takeoff/test_command.py \
    --drone-address=udp://:14540 -v --log-cli-level=INFO   # PX4, tier 1

pytest tests/command/nav_takeoff/test_flight.py \
    --drone-address=udp://:14540 --connection-timeout=60 \
    --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \
    --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO   # PX4, tier 2

pytest tests/command/nav_takeoff/test_flight.py \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
    --ardupilot-sitl=~/ardu_sitl/arducopter \
    --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 \
    --vehicle-type=quadcopter --autopilot=ardupilot -v --log-cli-level=INFO   # ArduCopter, tier 2
```

Other stacks/vehicle types: swap `--*-sitl`/`--*-model` and `--vehicle-type`/`--autopilot` per root `CLAUDE.md` § Running modes.
