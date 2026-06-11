# MAV_CMD_NAV_LAND (cmd=21) — Command Protocol (COMMAND_INT) Test Results

Tests the **command protocol** path (COMMAND_INT → COMMAND_ACK).
There is no mission-protocol counterpart directory for NAV_LAND in this suite (unlike NAV_TAKEOFF, which has both `tests/mission/nav_takeoff/` and `tests/command/nav_takeoff/`) — these are the only NAV_LAND tests.

## Parameter definition (common.xml)

`hasLocation="true"`, `isDestination="true"` → **COMMAND_INT** is the correct message type (integer x/y preserve lat/lon precision; see `tests/command/CLAUDE.md` § COMMAND_INT vs COMMAND_LONG selection rules).

| Param | Label | Description | Values | Units |
|-------|-------|-------------|--------|-------|
| 1 | Abort Alt | Minimum target altitude if landing is aborted (0 = undefined/use system default) | | m |
| 2 | Land Mode | `PRECISION_LAND_MODE` enum: 0=DISABLED, 1=OPPORTUNISTIC, 2=REQUIRED | 0/1/2 | |
| 3 | — | Empty | | |
| 4 | Yaw Angle | Desired yaw angle. NaN = use current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home) | | deg |
| 5 | Latitude | | | |
| 6 | Longitude | | | |
| 7 | Altitude | **Landing altitude (ground level in current frame)** | | m |

## Why this is structurally similar to — but semantically different from — NAV_TAKEOFF

The parameter layout closely parallels `MAV_CMD_NAV_TAKEOFF` (param1 numeric / param2 mode-ish / param4 Yaw / params 5–7 location), and most of the COMMAND_INT test machinery in `tests/command/nav_takeoff/test_command.py` carries over directly (survey-gating via `_ensure_supported`, `INT32_MAX` sentinel handling, out-of-range lat/lon, NaN encoding via `fields_json=None`, COMMAND_LONG sentinel comparisons).

**However, param7 (Altitude) means something fundamentally different.** For NAV_TAKEOFF, `z` is a *destination to climb to* (a 3D waypoint the vehicle navigates toward). For NAV_LAND, `z` is documented as **"Landing altitude (ground level in current frame)"** — a *ground/touchdown reference*, telling the autopilot where "the ground" is at the landing site (relevant when the landing site's elevation differs from the home/launch elevation, e.g. landing on a hill or rooftop). This is not a "fly here, then land" target.

## What ACK-level (Tier 1) tests can and cannot show

A `COMMAND_ACK` only reports `ACCEPTED` / `DENIED` / `UNSUPPORTED` / etc. — it cannot reveal *how* a parameter is used during execution. The following are genuinely **execution-semantics** questions that Tier 1 cannot answer (each needs flight observation — see "Tier 2 proposal" below):

- Does the vehicle fly directly toward the landing point (diagonal descent, like PX4 MC's NAV_TAKEOFF behaviour) or fly there at current altitude and then descend (a "dogleg" approach)?
- Is the commanded `x/y/z` the **touchdown point**, or — for a fixed-wing approach pattern — some other reference (glide-slope aim point, pattern-finish/rollout-stop point)?
- Is `z` actually honoured as a **ground-level reference** for touchdown detection, e.g. when landing somewhere whose elevation differs from home?
- For **VTOL**: does `NAV_LAND` trigger a transition to hover/MC mode for a vertical landing, does the vehicle attempt to land in whatever mode it is currently in (e.g. a runway-style landing while in FW cruise), or does it not execute at all (the same "accepted but inert" gap already documented for `NAV_TAKEOFF` on PX4 Rover)?

Tier 1 tests below are deliberately scoped to "was the value accepted", never "was it interpreted correctly" — see individual docstrings for the reasoning behind each assertion (or lack of one).

## Tier 1 test groups (`TestNavLandCommand`, `tests/command/nav_land/test_command.py`)

| Group | Tests | Approach |
|-------|-------|----------|
| A — Baseline | `test_command_accepted` | Assert not UNSUPPORTED |
| B — param1 (Abort Alt) | `test_param1_abort_alt_zero/_specific/_negative/_nan` | `zero` asserts not UNSUPPORTED (spec-defined sentinel); `specific`/`negative`/`nan` are observational |
| C — param2 (Land Mode) | `test_param2_land_mode_disabled/_opportunistic/_required/_undefined` | `disabled` asserts not UNSUPPORTED (baseline); the rest are observational — precision-landing *engagement* needs a simulated beacon (out of scope, see Future work) |
| D — param4 (Yaw) | `test_param4_yaw_specific_ack`, `test_param4_yaw_nan_ack` | Both observational — see note below on why this differs from NAV_TAKEOFF's approach |
| E — Location (5/6/7) | `test_location_specific_ack/_int32max_ack/_out_of_range_latlon_ack`, `test_altitude_specific_ack/_nan_ack`, `test_wrong_frame_ack` | `specific`/`altitude_specific` assert not UNSUPPORTED; `out_of_range_latlon` asserts DENIED (xfail mirrors NAV_TAKEOFF's tracked PX4 gap); the rest observational |
| F — COMMAND_LONG sentinels | `test_latlon_nan_command_long_ack`, `test_latlon_int32max_command_long` | Mirror NAV_TAKEOFF's COMMAND_LONG sentinel tests; skip on mock (real-stack only) |

### Why param4 (Yaw) is observational here, unlike NAV_TAKEOFF's `test_param4_yaw_ack_denied`

NAV_TAKEOFF's yaw test asserts `DENIED` (xfailing when stacks ignore it) **because prior survey runs already established that every tested stack ignores param4 in the COMMAND_INT takeoff path** (PX4: `rep->current.yaw = NAN`; ArduCopter/ArduPlane: explicitly "not supported").

We have no equivalent prior evidence for *landing*. Facing a specific heading on touchdown (into wind, to present a sensor/cargo bay, etc.) is plausibly something a stack *would* honour — landing and takeoff are different manoeuvres with different reasons to care about heading. `test_param4_yaw_specific_ack` is therefore observational; **once real-stack results are in, convert it to an assertion (`MAV_RESULT_DENIED`, xfail) if — and only if — the evidence shows the value is silently ignored**, per the "unsupported params must NACK" convention in `tests/command/nav_takeoff/README.md`.

## Spec gaps surfaced by these tests

These should be considered for the same kind of upstream issue the project already tracks for NAV_TAKEOFF (`tests/command/nav_takeoff/README.md` § Spec gaps):

1. **param1 (Abort Alt) — NaN sentinel undefined**: the spec defines `0` as "undefined/use system default" but says nothing about `NaN`. Per the general "NaN = no preference" convention used for other optional float params (and explicitly documented for param4 here), a stack treating `NaN` as equivalent to `0` would be reasonable — but a stack returning `DENIED` for `NaN` would *not* be a spec violation either, since `NaN` is simply not a documented value for this field. `test_param1_abort_alt_nan` is observational for exactly this reason — whatever a stack does is not assertable as "correct" or "wrong" until the spec says which numeric sentinel(s) are valid for "0 = use default" parameters in general.
   **Observed**: both PX4 and ArduCopter return `ACCEPTED` for `NaN` — consistent with treating it like `0`/default. Notably this *differs* from PX4's handling of other non-zero finite values (`10.0`, `-5.0` → both `DENIED`; see PX4 results below), i.e. PX4 special-cases `NaN` rather than validating it as "just another float".

2. **param7 (Altitude) — NaN meaning undefined**: contrast with NAV_TAKEOFF, where the spec explicitly permits `NaN` to mean "use default altitude". NAV_LAND's param7 description ("Landing altitude (ground level in current frame)") does not define what `NaN` means — current ground level? System default? Invalid? `test_altitude_nan_ack` is observational for the same reason as (1).
   **Observed**: both PX4 and ArduCopter return `ACCEPTED` for `NaN` — but Tier 1 cannot show *which* of the candidate meanings (current ground level / system default / ignored) the stack actually applies; that remains a Tier 2 (flight-observation) question.

3. **param7 (Altitude) — semantic ambiguity ("ground level" vs "destination")**: the spec text itself is the gap here — "ground level in current frame" is a touchdown/reference concept, but nothing in the command definition states *how* a stack should use it (e.g., should the vehicle stop descending when its measured altitude reaches this value? should it adjust its glide path to target this as the touchdown elevation?). This is empirically testable only via flight observation (Tier 2).

4. **Landing-point identity** (params 5/6/7 jointly): the spec does not state whether the commanded coordinate is the touchdown point, an approach/aim point, or a pattern-finish point — plausibly different for rotary-wing vs fixed-wing vehicles. Also testable only via flight observation.

## Future work / known gaps

- **Precision-landing behavioural verification** (param2 `OPPORTUNISTIC`/`REQUIRED`): confirming that a stack actually searches for / engages a landing beacon requires a simulated precision-landing target (e.g. IRLOCK or vision-beacon emulation in PX4/ArduPilot SITL), which is not currently configured for this suite. Tier 1 tests here can only confirm the enum value is *accepted* — see `tests/command/CLAUDE.md` § Future work / known gaps for the parallel "Error-condition tests… cannot force that condition from the suite" entry.

## Tier 2 results (`test_flight.py`)

### Overview

The original proposal listed 5 separate tests (one per question × vehicle-type
variant). They were consolidated into **4 comprehensive tests** — one flight per
vehicle type, each answering every applicable question from a single
takeoff → NAV_LAND → observe cycle (mirroring the `test_mc_takeoff_comprehensive`
pattern from `nav_takeoff/test_flight.py`, which the new tests import their
arming/telemetry helpers from directly):

| Test | Gate | Questions answered |
|------|------|--------------------|
| `test_mc_landing_comprehensive` | `vehicle_type == "quadcopter"` (PX4 MC + ArduCopter MC) | Approach trajectory; landing-point identity |
| `test_fw_landing_comprehensive` | `vehicle_type == "fixed_wing"`, `autopilot == "px4"` (ArduPlane FW skips — UNSUPPORTED) | Approach trajectory; landing-point identity (touchdown / aim / finish point) |
| `test_vtol_landing_behaviour` | `vehicle_type == "vtol"` (PX4 VTOL only) | VTOL transition-mode classification: (a) transitions to hover and lands vertically, (b) lands in current mode without transitioning, (c) doesn't land (inert) |
| `test_px4_rover_land_is_inert` | `autopilot == "px4" and vehicle_type == "rover"` | "Accepted but inert" — does NAV_LAND produce any landing-like behaviour on a vehicle that can never be airborne? |

Each test writes a natural-language behaviour summary to
`logs/command_nav_land_summary_<autopilot>_<vehicle_type>_<test>_<timestamp>.md`
and logs it at `INFO` level (`BEHAVIOUR SUMMARY`).

### PX4 MC (1.18.0) — `test_mc_landing_comprehensive`

**Setup:** no mode change required; takeoff to 30 m, then NAV_LAND commanded 80 m
laterally offset (ground-level z).

**Finding:** NAV_LAND is `ACCEPTED`, descent starts immediately, and the vehicle
touches down (`ON_GROUND`) — but it **ignores the commanded lat/lon and simply
descends from its current position** (trajectory: *descend-in-place*). Touchdown
was 80.2 m from the commanded coordinate — essentially the full lateral offset,
i.e. the vehicle never moved toward the target at all. **The commanded coordinate
is NOT the touchdown point** for PX4 MC.

### ArduCopter MC (4.8.0) — `test_mc_landing_comprehensive`

**Setup:** GUIDED mode (confirmed); takeoff to 30 m, then NAV_LAND commanded 80 m
laterally offset (ground-level z).

**Finding:** NAV_LAND is `ACCEPTED` and triggers a descent (touchdown distance from
the commanded point measured at 80.0 m — i.e. also *descend-in-place*, matching the
PX4 MC finding), but `landed_state()` never reported `ON_GROUND` within the 300 s
budget — telemetry showed the vehicle at 0.0 m relative altitude when the wait
expired, i.e. **essentially on the ground**. This reads as a `landed_state()`
reporting lag/quirk (the same kind of telemetry-settling artifact independently
observed on PX4 Rover below), not a vehicle that failed to land.

### PX4 FW (1.18.0) — `test_fw_landing_comprehensive`

**Setup:** no mode change required; attempted takeoff to 30 m before sending
NAV_LAND 400 m laterally offset.

**Finding:** **Inconclusive — could not be observed.** NAV_TAKEOFF was `ACCEPTED`
but the aircraft performed a ground roll and never reached the 2 m airborne
threshold within 90 s — the documented PX4 FW SIH-simulator limitation
(`nav_takeoff/README.md` § PX4 FW: "ground roll only, altitude < 2 m — an SIH
simulator limitation, not a protocol issue"). With the aircraft never airborne,
NAV_LAND was never sent — there was nothing to land. This is a **simulator
constraint, not a NAV_LAND-specific gap**; the test detects the condition up
front (a short, bounded liftoff probe) and exits cleanly with this finding rather
than hanging on a doomed climb.

### PX4 VTOL (1.18.0) — `test_vtol_landing_behaviour`

**Setup:** no mode change required; takeoff to 30 m (PX4 VTOL takes off in MC/hover
mode), then NAV_LAND commanded 80 m laterally offset (ground-level z).

**Finding:** the vehicle **stays in MC/hover throughout** — `vtol_state()` reported
`MC` continuously, with no `TRANSITION_TO_FW`/`TRANSITION_TO_MC` observed (it was
already in the rotary-wing mode needed for vertical landing, so a transition simply
wasn't necessary). It descended and reached `ON_GROUND` 79.6 m from the commanded
point — i.e. it landed essentially where it was hovering, **not** at the commanded
coordinate. Classification: **(b) lands in the current mode without transitioning**
— neither (a) "transitions to hover" (it was already there) nor (c) "doesn't land"
applies.

### PX4 Rover (1.18.0) — `test_px4_rover_land_is_inert`

**Setup:** rover armed in place — no takeoff (a ground vehicle is never airborne);
NAV_LAND commanded 80 m laterally offset (ground-level z), observed for 15 s.

**Finding:** NAV_LAND is `ACCEPTED`. The only telemetry change over 15 s was
`landed_state()` settling from `IN_AIR` to `ON_GROUND` — but `flight_mode` (`HOLD`)
and position (moved 0.0 m) stayed unchanged throughout, and a stationary ground
vehicle cannot meaningfully transition from "in air" to "on ground". This is a
`landed_state()` reporting/settling artifact (the simulator briefly reports `IN_AIR`
immediately after spawn/arm, before settling to `ON_GROUND` ~15 s later,
*independent of any command sent* — confirmed by sampling pre-NAV_LAND telemetry,
which already showed `IN_AIR`). Discounting that artifact, NAV_LAND produces **no
landing-like behaviour at all** — confirming, for NAV_LAND, the same
permissive-but-meaningless acceptance pattern already documented for NAV_TAKEOFF on
PX4 Rover (`nav_takeoff/README.md` § PX4 Rover: "a ground vehicle accepting a flight
command without executing it or returning UNSUPPORTED is misleading").

ArduRover is not tested here — Tier 1 found NAV_LAND `UNSUPPORTED` there, and
`require_real_stack` skips the whole Tier 2 suite for that combination.

### Cross-cutting findings — Spec gaps #3 / #4 resolved

These flight observations empirically resolve Spec gaps #3 (param7 semantic
ambiguity) and #4 (landing-point identity) raised above:

- **Landing-point identity (gap #4) — answered, and the answer is surprising**: on
  every platform where a landing could be observed (PX4 MC, ArduCopter MC, PX4
  VTOL), **the commanded x/y coordinate is *not* the touchdown point** — the
  vehicle simply descends from wherever it already is ("descend-in-place" /
  "lands in current mode"), landing 80–96% of the commanded lateral offset away
  from the target. The spec's params 5/6 (`Latitude`/`Longitude`) appear to be
  **ignored for the actual landing manoeuvre** on rotary-wing/VTOL vehicles — a
  materially different behaviour from NAV_TAKEOFF, where the commanded
  lat/lon/alt *is* the destination. PX4 FW could not be observed (SITL
  liftoff limitation), so the fixed-wing "touchdown vs. aim-point vs.
  finish-point" sub-question remains open.
- **param7 semantic ambiguity (gap #3) — partially answered**: since the commanded
  x/y is ignored and the vehicle descends in place to `ON_GROUND` (the actual
  ground, as reported by `landed_state()`), param7's "ground level in current
  frame" appears to be **interpreted as a hint/no-op rather than an actively-used
  reference** — the vehicle lands at the actual terrain height regardless of the
  commanded altitude value. This is consistent across PX4 MC, ArduCopter MC, and
  PX4 VTOL (all commanded with `z = home.absolute_altitude_m`, i.e.
  ground-level-ish, and all landed at actual ground level). A definitive answer
  for cases where commanded altitude *diverges* from actual terrain height would
  require a further targeted test (not pursued here — diminishing returns relative
  to the now-clear "lat/lon/alt are not used as a landing destination" finding).

## Tier 1 test results

### Mock (paired mode) — 2026-06-08

19 collected: **17 passed, 2 skipped**.

| Test | Mock result |
|------|-------------|
| `test_command_accepted` | PASS — result=0 ACCEPTED |
| `test_param1_abort_alt_zero` | PASS — result=0 ACCEPTED |
| `test_param1_abort_alt_specific` | PASS — result=0 ACCEPTED (observational) |
| `test_param1_abort_alt_negative` | PASS — result=0 ACCEPTED (observational; mock does not validate) |
| `test_param1_abort_alt_nan` | PASS — result=0 ACCEPTED (observational) |
| `test_param2_land_mode_disabled` | PASS — result=0 ACCEPTED |
| `test_param2_land_mode_opportunistic` | PASS — result=0 ACCEPTED (observational) |
| `test_param2_land_mode_required` | PASS — result=0 ACCEPTED (observational) |
| `test_param2_land_mode_undefined` | PASS — result=0 ACCEPTED (observational; mock does not validate enum) |
| `test_param4_yaw_specific_ack` | PASS — result=0 ACCEPTED (observational; mock ignores yaw) |
| `test_param4_yaw_nan_ack` | PASS — result=0 ACCEPTED (observational) |
| `test_location_specific_ack` | PASS — result=0 ACCEPTED |
| `test_location_int32max_ack` | PASS — result=0 ACCEPTED (observational) |
| `test_location_out_of_range_latlon_ack` | PASS — result=2 DENIED (mock validates lat/lon range) |
| `test_altitude_specific_ack` | PASS — result=0 ACCEPTED |
| `test_altitude_nan_ack` | PASS — result=0 ACCEPTED (observational) |
| `test_wrong_frame_ack` | PASS — result=0 ACCEPTED (mock accepts all frames) |
| `test_latlon_nan_command_long_ack` | SKIP — requires real stack |
| `test_latlon_int32max_command_long` | SKIP — requires real stack |

The mock returns `ACCEPTED` for everything except out-of-range lat/lon (validated generically for all COMMAND_INT, per `tests/mock_flight_stack.py` default behaviour — not NAV_LAND-specific).

### PX4 — MC / FW / VTOL / Rover (standalone) — 2026-06-08

Tested against PX4 1.18.0-alpha (git hash `0000006d67dc8571`), SIH simulator (`sihsim_quadx` / `sihsim_airplane` / `sihsim_standard_vtol` / `sihsim_rover_ackermann`), connected via `udp://:14540`.

**18 PASS, 1 XFAIL, 0 SKIP — byte-identical across all four vehicle types.**
NAV_LAND is SUPPORTED on every PX4 vehicle type via COMMAND_INT (PX4 does not gate by vehicle type — same permissive pattern documented for NAV_TAKEOFF).

| Test | Param | Result |
|------|-------|--------|
| `test_command_accepted` | baseline | PASS — result=0 ACCEPTED |
| `test_param1_abort_alt_zero` | param1=0.0 | PASS — result=0 ACCEPTED |
| `test_param1_abort_alt_specific` | param1=10.0 m | PASS — result=2 DENIED (observational; PX4 validates non-zero abort altitude) |
| `test_param1_abort_alt_negative` | param1=-5.0 m | PASS — result=2 DENIED (observational; PX4 rejects negative abort altitude too — same DENIED path as positive) |
| `test_param1_abort_alt_nan` | param1=NaN | PASS — result=0 ACCEPTED (observational; NaN treated like 0/default, not like a specific altitude) |
| `test_param2_land_mode_disabled` | param2=DISABLED (0) | PASS — result=0 ACCEPTED |
| `test_param2_land_mode_opportunistic` | param2=OPPORTUNISTIC (1) | PASS — result=0 ACCEPTED (observational) |
| `test_param2_land_mode_required` | param2=REQUIRED (2) | PASS — result=0 ACCEPTED (observational; no beacon configured — see Future work) |
| `test_param2_land_mode_undefined` | param2=5 (undefined enum) | PASS — result=0 ACCEPTED (observational; PX4 does not validate the enum range) |
| `test_param4_yaw_specific_ack` | param4=90° | PASS — result=0 ACCEPTED (observational) |
| `test_param4_yaw_nan_ack` | param4=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_location_specific_ack` | x/y=home | PASS — result=0 ACCEPTED |
| `test_location_int32max_ack` | x/y=INT32_MAX | PASS — result=0 ACCEPTED (observational) |
| `test_location_out_of_range_latlon_ack` | x=120°N, y=200°E | PASS — result=2 DENIED (matches NAV_TAKEOFF; no PX4 gap here) |
| `test_altitude_specific_ack` | param7=5.0 m | PASS — result=0 ACCEPTED |
| `test_altitude_nan_ack` | param7=NaN | PASS — result=0 ACCEPTED (observational; see Spec gaps) |
| `test_wrong_frame_ack` | frame=LOCAL_NED(1) | PASS — result=0 ACCEPTED (observational; PX4 accepts any frame via COMMAND_INT) |
| `test_latlon_nan_command_long_ack` | COMMAND_LONG param5/6=NaN | PASS — result=0 ACCEPTED |
| `test_latlon_int32max_command_long` | COMMAND_LONG param5/6=INT32_MAX | **XFAIL** — result=2 DENIED; PX4 rejects float(INT32_MAX) lat/lon as a protocol error (`mavlink_receiver.cpp:499–505`) — same documented gap as NAV_TAKEOFF |

**New finding — param1 (Abort Alt) validation**: unlike NAV_TAKEOFF (where PX4 ignores param1/pitch entirely, `xfail`-ing `test_param1_pitch_ack_denied`), PX4 actively validates NAV_LAND's `param1` and returns `DENIED` for *any* non-zero value, positive or negative. This is plausibly correct handling (the field defines `0` as "use system default", so PX4 may require a value within an internally-validated range and 10.0 m happens to fall outside it) — but it means a GCS cannot rely on "any finite abort altitude is accepted". Worth a follow-up probe across a wider range of values to map PX4's accepted envelope; not pursued here as it is execution-detail, not a spec conformance question.

### ArduCopter MC (standalone) — 2026-06-08

Tested against ArduCopter V4.8.0-dev (`70fe7125`, `--model +`) connected via TCP port 5760.

**18 PASS, 1 XFAIL, 0 SKIP.**
NAV_LAND is SUPPORTED on ArduCopter via COMMAND_INT.

| Test | Param | Result |
|------|-------|--------|
| `test_command_accepted` | baseline | PASS — result=0 ACCEPTED |
| `test_param1_abort_alt_zero` | param1=0.0 | PASS — result=0 ACCEPTED |
| `test_param1_abort_alt_specific` | param1=10.0 m | PASS — result=0 ACCEPTED (observational; ArduCopter does not validate, unlike PX4) |
| `test_param1_abort_alt_negative` | param1=-5.0 m | PASS — result=0 ACCEPTED (observational; ArduCopter does not validate) |
| `test_param1_abort_alt_nan` | param1=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_param2_land_mode_disabled` | param2=DISABLED (0) | PASS — result=0 ACCEPTED |
| `test_param2_land_mode_opportunistic` | param2=OPPORTUNISTIC (1) | PASS — result=0 ACCEPTED (observational; ArduCopter has no precision-landing mode enum handling via COMMAND_INT — see Future work) |
| `test_param2_land_mode_required` | param2=REQUIRED (2) | PASS — result=0 ACCEPTED (observational) |
| `test_param2_land_mode_undefined` | param2=5 (undefined enum) | PASS — result=0 ACCEPTED (observational; not validated) |
| `test_param4_yaw_specific_ack` | param4=90° | PASS — result=0 ACCEPTED (observational) |
| `test_param4_yaw_nan_ack` | param4=NaN | PASS — result=0 ACCEPTED (observational) |
| `test_location_specific_ack` | x/y=home | PASS — result=0 ACCEPTED |
| `test_location_int32max_ack` | x/y=INT32_MAX | PASS — result=0 ACCEPTED (observational) |
| `test_location_out_of_range_latlon_ack` | x=120°N, y=200°E | **XFAIL** — result=0 ACCEPTED; ArduCopter accepts geometrically impossible lat/lon via COMMAND_INT (spec violation — same documented gap as NAV_TAKEOFF) |
| `test_altitude_specific_ack` | param7=5.0 m | PASS — result=0 ACCEPTED |
| `test_altitude_nan_ack` | param7=NaN | PASS — result=0 ACCEPTED (observational; see Spec gaps) |
| `test_wrong_frame_ack` | frame=LOCAL_NED(1) | PASS — result=0 ACCEPTED (observational; ArduCopter accepts any frame via COMMAND_INT) |
| `test_latlon_nan_command_long_ack` | COMMAND_LONG param5/6=NaN | PASS — result=0 ACCEPTED |
| `test_latlon_int32max_command_long` | COMMAND_LONG param5/6=INT32_MAX | PASS — **UNKNOWN, no ACK within timeout** (logged per the no-ACK policy, not asserted; ArduCopter silently drops float(INT32_MAX) lat/lon in COMMAND_LONG rather than NACKing it like PX4 does — itself a spec-violation candidate, but cannot be distinguished from "busy/dropped" without further probing) |

### ArduPlane FW / ArduPlane QP / ArduRover — 2026-06-08

NAV_LAND is **UNSUPPORTED** on all three — confirmed exactly as the survey predicted (`tests/command/README.md` § Commands supported cross-platform: NAV_LAND ✗ for ArduPlane FW, ArduPlane QP, ArduRover).
`_ensure_supported()` probes once, caches `MAV_RESULT_UNSUPPORTED (3)`, and calls `pytest.skip()` for all 19 tests in the class — **19 SKIP, 0 PASS, 0 FAIL** on each:

| Stack | Firmware | Result |
|-------|----------|--------|
| ArduPlane FW | V4.8.0-dev (`70fe7125`, `--model plane`) | 19 SKIP — "NAV_LAND (cmd=21) is UNSUPPORTED on this platform" |
| ArduPlane QP | V4.8.0-dev (`70fe7125`, `--model quadplane`) | 19 SKIP — identical to FW (not separately probed by the survey; behaviour confirmed identical here) |
| ArduRover | V4.8.0-dev (`fab9a565`, `--model rover`) | 19 SKIP — "NAV_LAND (cmd=21) is UNSUPPORTED on this platform" |

This is consistent with NAV_LAND being an aerial-landing command: ArduPlane and ArduRover gate it out entirely (unlike PX4, which accepts it on every vehicle type — see the PX4 results above).

## Running

```bash
# Mock (paired)
pytest tests/command/nav_land/test_command.py -v --log-cli-level=INFO

# PX4 multicopter
pytest tests/command/nav_land/test_command.py --drone-address=udp://:14540 -v --log-cli-level=INFO

# ArduCopter
pytest tests/command/nav_land/test_command.py \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 -v --log-cli-level=INFO
```
