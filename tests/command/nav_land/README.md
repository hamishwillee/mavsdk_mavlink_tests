# MAV_CMD_NAV_LAND (cmd=21) — Command Protocol (COMMAND_INT) Test Results

Command protocol only (COMMAND_INT → COMMAND_ACK) — NAV_LAND has no mission-protocol test directory, unlike NAV_TAKEOFF.

## Parameter definition (common.xml)

`hasLocation="true"`, `isDestination="true"` → **COMMAND_INT** (see `tests/command/CLAUDE.md` § COMMAND_INT vs COMMAND_LONG selection rules).

| Param | Label | Description | Values | Units |
|-------|-------|-------------|--------|-------|
| 1 | Abort Alt | Minimum target altitude if landing is aborted (0 = undefined/use system default) | | m |
| 2 | Land Mode | `PRECISION_LAND_MODE`: 0=DISABLED, 1=OPPORTUNISTIC, 2=REQUIRED | 0/1/2 | |
| 3 | — | Empty | | |
| 4 | Yaw Angle | Desired yaw. NaN = use current system yaw heading mode | | deg |
| 5 | Latitude | | | |
| 6 | Longitude | | | |
| 7 | Altitude | **Landing altitude (ground level in current frame)** | | m |

Parameter layout mirrors NAV_TAKEOFF (param1 numeric / param2 mode-ish / param4 Yaw / params 5–7 location) and most of `nav_takeoff/test_command.py`'s test machinery carries over directly. The one structural difference: NAV_TAKEOFF's `z` is a climb destination; NAV_LAND's `z` is a ground/touchdown reference (relevant when the landing site's elevation differs from home — a hilltop or rooftop landing) — not a "fly here, then land" target.

## What ACK-level (Tier 1) tests can't show

A `COMMAND_ACK` only reports ACCEPTED/DENIED/UNSUPPORTED — it can't reveal how a parameter is used during execution. Needs flight observation (Tier 2, below):

- Direct/diagonal descent toward the landing point vs. fly-there-then-descend ("dogleg")?
- Is the commanded x/y/z the touchdown point, or (for fixed-wing) some other reference (glide-slope aim point, pattern-finish point)?
- Is `z` actually honoured as a ground-level reference for touchdown detection?
- VTOL: does NAV_LAND trigger a hover transition, land in the current mode, or not execute at all?

Tier 1 tests below are scoped to "was the value accepted", never "was it interpreted correctly".

## Tier 1 test groups (`TestNavLandCommand`, `test_command.py`)

| Group | Tests | Approach |
|-------|-------|----------|
| A — Baseline | `test_nav_land_command_accepted` | Assert not UNSUPPORTED |
| B — param1 (Abort Alt) | `_zero/_specific/_negative/_nan` | `zero` asserts not UNSUPPORTED (spec-defined sentinel); rest observational |
| C — param2 (Land Mode) | `_disabled/_opportunistic/_required/_undefined` | `disabled` asserts not UNSUPPORTED; rest observational — precision-landing *engagement* needs a simulated beacon (out of scope) |
| D — param4 (Yaw) | `_specific_ack`, `_nan_ack` | Both observational (see below) |
| E — Location (5/6/7) | `_specific_ack/_int32max_ack/_out_of_range_latlon_ack`, `_altitude_specific_ack/_nan_ack`, `_wrong_frame_ack` | `specific`/`altitude_specific` assert not UNSUPPORTED; `out_of_range_latlon` asserts DENIED (xfail, mirrors NAV_TAKEOFF's tracked PX4 gap); rest observational |
| F — COMMAND_LONG sentinels | `_latlon_nan_command_long_ack`, `_latlon_int32max_command_long` | Mirror NAV_TAKEOFF's; real-stack only |
| G — Message-type exclusivity | `test_nav_land_command_long_rejected` | Hand-rolled equivalent of `Tier1CommandTestBase.test_hasLocation_rejects_command_long` (this file predates that base class migration) — see `../CLAUDE.md` § Mandatory common tests, check 7 |

**Message-type exclusivity result (2026-09-17, against a PX4-Autopilot checkout at `~/github/PX4/PX4-Autopilot`)**: `test_nav_land_command_long_rejected` **XFAIL** — COMMAND_LONG for NAV_LAND returns `ACCEPTED(0)`, not the expected `MAV_RESULT_COMMAND_INT_ONLY(8)`. Contrast with `NAV_VTOL_TAKEOFF`, which genuinely enforces this on the same build (PX4 commit `83e7afba56`'s `command_is_int_only()` switch currently lists only that one command) — see `../nav_vtol_takeoff/CLAUDE.md`.

**param4 (Yaw) is observational here, unlike NAV_TAKEOFF's assertion**: NAV_TAKEOFF asserts DENIED because prior surveys established every stack ignores param4 there. No equivalent evidence exists for landing — a heading-on-touchdown preference is plausible enough to honour that it isn't assumed ignored. Convert to an assertion once real-stack evidence shows otherwise.

## Spec gaps

1. **param1 NaN sentinel undefined** — spec defines `0` as "use system default" but is silent on `NaN`; neither ACCEPTED nor DENIED for `NaN` would be a spec violation, so `test_nav_land_param1_abort_alt_nan` is observational. Observed: PX4 and ArduCopter both ACCEPT `NaN` — but PX4 DENIEs other non-zero finite values (`10.0`, `-5.0`), i.e. it special-cases `NaN` rather than validating it as an ordinary float.
2. **param7 NaN meaning undefined** — unlike NAV_TAKEOFF (NaN = "use default altitude"), NAV_LAND's "ground level in current frame" doesn't define NaN. `test_nav_land_altitude_nan_ack` is observational. Observed: both stacks ACCEPT `NaN`; which meaning is actually applied is a Tier 2 question.
3. **param7 semantic ambiguity** — "ground level in current frame" is a reference concept with no stated usage rule (stop descending at this altitude? adjust glide path to it?). Testable only via flight observation.
4. **Landing-point identity** (params 5/6/7) — spec doesn't state whether the commanded coordinate is the touchdown point, an approach/aim point, or a pattern-finish point. Also flight-observation only.

## Future work

Precision-landing engagement (param2 OPPORTUNISTIC/REQUIRED) needs a simulated beacon (IRLOCK/vision-beacon) not currently configured — Tier 1 can only confirm the enum value is *accepted*, not that landing behaviour changes.

## Tier 2 results (`test_flight.py`)

Four comprehensive tests (one per vehicle type, each a single takeoff → NAV_LAND → observe cycle, importing arming/telemetry helpers from `nav_takeoff/test_flight.py`) resolve spec gaps #3/#4 above:

| Test | Gate | Questions answered |
|------|------|--------------------|
| `test_mc_landing_comprehensive` | quadcopter (PX4 MC + ArduCopter MC) | Approach trajectory; landing-point identity |
| `test_fw_landing_comprehensive` | fixed_wing, PX4 (ArduPlane FW skips — UNSUPPORTED) | Approach trajectory; landing-point identity |
| `test_vtol_landing_behaviour` | vtol (PX4 VTOL only) | Transition classification: (a) hover then land, (b) land in current mode, (c) inert |
| `test_px4_rover_land_is_inert` | PX4 rover | Accepted-but-inert check |

Each writes a behaviour summary to `logs/command_nav_land_summary_<autopilot>_<vehicle_type>_<test>_<timestamp>.md`.

**Headline finding, all platforms where landing was observed**: the commanded coordinate is **not** the touchdown point — the vehicle descends in place from wherever it already is, landing 80–96% of the commanded lateral offset away from the target. Per-platform detail:

- **PX4 MC**: 30 m takeoff, NAV_LAND commanded 80 m offset. ACCEPTED, immediate descend-in-place, touchdown 80.2 m from the commanded point (essentially the full offset — no movement toward target).
- **ArduCopter MC** (GUIDED mode): same descend-in-place pattern, touchdown 80.0 m from target. `landed_state()` never reported `ON_GROUND` within 300 s despite the vehicle sitting at 0.0 m relative altitude — a telemetry-settling/reporting-lag quirk (also seen on PX4 Rover below), not a failed landing.
- **PX4 FW**: inconclusive. NAV_TAKEOFF ACCEPTED but the aircraft never left the ground within 90 s (documented SIH ground-roll-only limitation — see `nav_takeoff/README.md` § PX4 FW) so NAV_LAND was never sent. Simulator constraint, not a NAV_LAND gap; the test detects this up front and exits cleanly.
- **PX4 VTOL**: stayed in MC/hover throughout (`vtol_state()` = `MC`, no transition observed — already in the mode needed for vertical landing) and touched down 79.6 m from the commanded point. Classification: (b) lands in current mode without transitioning.
- **PX4 Rover**: ACCEPTED but no landing-like behaviour — `flight_mode` (HOLD) and position unchanged over 15 s. The only telemetry change was `landed_state()` settling `IN_AIR`→`ON_GROUND`, a spawn/arm artifact confirmed present *before* NAV_LAND was even sent (same pattern documented for NAV_TAKEOFF on PX4 Rover). Not tested on ArduRover — Tier 1 already found NAV_LAND UNSUPPORTED there.

param7 ("ground level in current frame") appears to be a hint/no-op rather than an actively-used reference — all three observed platforms landed at actual ground level regardless of the commanded altitude (all commanded near `home.absolute_altitude_m`). Confirming this for a commanded altitude that actually diverges from terrain height would need a further targeted test — not pursued, given the clear "lat/lon/alt aren't used as a destination" finding already answers the higher-value question. The fixed-wing touchdown/aim-point/finish-point sub-question remains open (PX4 FW couldn't be observed).

## Tier 1 test results

### Mock — 17 passed, 2 skipped (real-stack only)

Mock ACCEPTs everything except out-of-range lat/lon (generic COMMAND_INT validation in `mock_flight_stack.py`, not NAV_LAND-specific).

### PX4 — MC/FW/VTOL/Rover (standalone)

Tested against PX4 1.18.0-alpha (`0000006d67dc8571`), SIH. **18 PASS, 1 XFAIL, byte-identical across all four vehicle types.** NAV_LAND is SUPPORTED on every PX4 vehicle type (PX4 doesn't gate by vehicle type — same pattern as NAV_TAKEOFF).

| Test | Param | Result |
|------|-------|--------|
| `test_nav_land_command_accepted` | baseline | PASS — ACCEPTED |
| `test_nav_land_param1_abort_alt_zero` | 0.0 | PASS — ACCEPTED |
| `test_nav_land_param1_abort_alt_specific` | 10.0 m | PASS — DENIED (observational; PX4 validates non-zero abort alt) |
| `test_nav_land_param1_abort_alt_negative` | -5.0 m | PASS — DENIED (observational; negative also rejected) |
| `test_nav_land_param1_abort_alt_nan` | NaN | PASS — ACCEPTED (treated like 0/default) |
| `test_nav_land_param2_land_mode_disabled` | 0 | PASS — ACCEPTED |
| `test_nav_land_param2_land_mode_opportunistic` | 1 | PASS — ACCEPTED (observational; no beacon configured) |
| `test_nav_land_param2_land_mode_required` | 2 | PASS — ACCEPTED (observational) |
| `test_nav_land_param2_land_mode_undefined` | 5 | PASS — ACCEPTED (observational; enum range not validated) |
| `test_nav_land_param4_yaw_specific_ack` | 90° | PASS — ACCEPTED (observational) |
| `test_nav_land_param4_yaw_nan_ack` | NaN | PASS — ACCEPTED (observational) |
| `test_nav_land_location_specific_ack` | home | PASS — ACCEPTED |
| `test_nav_land_location_int32max_ack` | INT32_MAX | PASS — ACCEPTED (observational) |
| `test_nav_land_location_out_of_range_latlon_ack` | 120°N, 200°E | PASS — DENIED (matches NAV_TAKEOFF; no gap here) |
| `test_nav_land_altitude_specific_ack` | 5.0 m | PASS — ACCEPTED |
| `test_nav_land_altitude_nan_ack` | NaN | PASS — ACCEPTED (observational) |
| `test_nav_land_wrong_frame_ack` | LOCAL_NED(1) | PASS — ACCEPTED (PX4 accepts any frame) |
| `test_nav_land_latlon_nan_command_long_ack` | NaN | PASS — ACCEPTED |
| `test_nav_land_latlon_int32max_command_long` | INT32_MAX | **XFAIL** — DENIED; PX4 rejects `float(INT32_MAX)` as a protocol error (`mavlink_receiver.cpp:499–505`), same gap as NAV_TAKEOFF |

**param1 validation is new**: unlike NAV_TAKEOFF (which ignores param1/pitch entirely), PX4 DENIEs any non-zero NAV_LAND abort altitude. Plausibly correct (values are checked against an internal range), but a GCS can't assume any finite abort altitude is accepted.

### ArduCopter MC (standalone)

Tested against V4.8.0-dev (`70fe7125`, `--model +`). **18 PASS, 1 XFAIL.** SUPPORTED.

| Test | Param | Result |
|------|-------|--------|
| `test_nav_land_command_accepted` | baseline | PASS — ACCEPTED |
| `test_nav_land_param1_abort_alt_zero` | 0.0 | PASS — ACCEPTED |
| `test_nav_land_param1_abort_alt_specific` | 10.0 m | PASS — ACCEPTED (observational; no validation, unlike PX4) |
| `test_nav_land_param1_abort_alt_negative` | -5.0 m | PASS — ACCEPTED (observational; no validation) |
| `test_nav_land_param1_abort_alt_nan` | NaN | PASS — ACCEPTED (observational) |
| `test_nav_land_param2_land_mode_disabled` | 0 | PASS — ACCEPTED |
| `test_nav_land_param2_land_mode_opportunistic` | 1 | PASS — ACCEPTED (observational; no precision-land handling via COMMAND_INT) |
| `test_nav_land_param2_land_mode_required` | 2 | PASS — ACCEPTED (observational) |
| `test_nav_land_param2_land_mode_undefined` | 5 | PASS — ACCEPTED (observational; not validated) |
| `test_nav_land_param4_yaw_specific_ack` | 90° | PASS — ACCEPTED (observational) |
| `test_nav_land_param4_yaw_nan_ack` | NaN | PASS — ACCEPTED (observational) |
| `test_nav_land_location_specific_ack` | home | PASS — ACCEPTED |
| `test_nav_land_location_int32max_ack` | INT32_MAX | PASS — ACCEPTED (observational) |
| `test_nav_land_location_out_of_range_latlon_ack` | 120°N, 200°E | **XFAIL** — ACCEPTED; accepts geometrically impossible lat/lon (spec violation, same gap as NAV_TAKEOFF) |
| `test_nav_land_altitude_specific_ack` | 5.0 m | PASS — ACCEPTED |
| `test_nav_land_altitude_nan_ack` | NaN | PASS — ACCEPTED (observational) |
| `test_nav_land_wrong_frame_ack` | LOCAL_NED(1) | PASS — ACCEPTED (accepts any frame) |
| `test_nav_land_latlon_nan_command_long_ack` | NaN | PASS — ACCEPTED |
| `test_nav_land_latlon_int32max_command_long` | INT32_MAX | PASS — **UNKNOWN, no ACK** (logged per no-ACK policy, not asserted; ArduCopter silently drops this rather than NACKing like PX4 — itself a spec-violation candidate, but indistinguishable from "busy/dropped" without further probing) |

### ArduPlane FW / ArduPlane QP / ArduRover

NAV_LAND is **UNSUPPORTED** on all three, matching the survey prediction (`tests/command/README.md`). `_ensure_supported()` skips all 19 tests per class:

| Stack | Firmware | Result |
|-------|----------|--------|
| ArduPlane FW | V4.8.0-dev (`70fe7125`, `--model plane`) | 19 SKIP |
| ArduPlane QP | V4.8.0-dev (`70fe7125`, `--model quadplane`) | 19 SKIP |
| ArduRover | V4.8.0-dev (`fab9a565`, `--model rover`) | 19 SKIP |

Consistent with NAV_LAND being aerial-landing-specific: ArduPlane and ArduRover gate it out entirely; PX4 accepts it on every vehicle type.

## Running

```bash
pytest tests/command/nav_land/test_command.py -v --log-cli-level=INFO   # mock

pytest tests/command/nav_land/test_command.py \
    --drone-address=udp://:14540 -v --log-cli-level=INFO                # PX4 MC

pytest tests/command/nav_land/test_command.py \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
    -v --log-cli-level=INFO                                             # ArduCopter
```
