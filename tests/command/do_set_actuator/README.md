# MAV_CMD_DO_SET_ACTUATOR (cmd=187) — command protocol

Built specifically to verify [PX4-Autopilot PR #28723](https://github.com/PX4/PX4-Autopilot/pull/28723)
("fix(mavlink): fix COMMAND_INT/MISSION_ITEM_INT actuator scaling") — see
`CLAUDE.md` in this directory for the full bug/fix writeup and the
stale-binary pitfall hit while verifying it.

## The bug (PR #28723's own commit messages/diffs)

DO_SET_ACTUATOR's param5/param6 map to COMMAND_INT's `x`/`y` — genuinely
int32 wire fields — spec-scaled by 1e7 *regardless of frame*, even though
this command has no location semantics (`hasLocation="false"`). Before the
fix, `mavlink_receiver.cpp` decoded x/y using the *shared location-frame*
logic (1e4 for local/body frames, 1e7 for global/other) instead of always
1e7 for this command. Separately, the "not used" check required **both**
x AND y to equal INT32_MAX to treat either as ignored — spec-incorrect;
each of param5/param6 must independently accept its own sentinel. Both
fixed via a new shared helper, `mavlink_cmd_params::decode_scaled_int32_field()`.

## Command parameters (common.xml)

| Param | Label | Meaning |
|---|---|---|
| param1-4 | Actuator 1-4 | -1..1, NaN=ignore — plain floats, both message types |
| param5 | Actuator 5 | x in COMMAND_INT (×1e7, INT32_MAX=ignore); float in COMMAND_LONG |
| param6 | Actuator 6 | y in COMMAND_INT (×1e7, INT32_MAX=ignore); float in COMMAND_LONG |
| param7 | Index | z field — always float, minValue=0 |

Per `tests/command/CLAUDE.md`'s COMMAND_INT/COMMAND_LONG selection rule,
this command has no location and meaningful floats in params 5/6, so
**COMMAND_LONG is the primary message type** — unusual among this repo's
command tests, most of which default to COMMAND_INT.

## IMPORTANT — what Tier 1 can and cannot show

PX4's `Commander.cpp` answers DO_SET_ACTUATOR with an **unconditional
ACCEPTED**, no value/range validation of any kind. Tier 1 (ACK-only) tests
therefore **cannot distinguish correct from incorrect x/y scaling** — every
combination gets ACCEPTED regardless. The bespoke Tier 1 tests are kept as
protocol-level regression/documentation coverage only. **The actual PR fix
is conclusively verified in Tier 2** via real PWM output observation.

## IMPORTANT — what "supported: true" in the compat JSON actually means

`ACTUATOR_OUTPUT_STATUS` is PX4's own *commanded* output value, not sensed
physical position — there's no closed loop, and no physical actuator model
exists for this generic output function in SITL either way. Sufficient to
verify this PR (a pure decoding bug upstream of any physical actuator),
but NOT evidence of confirmed physical movement — see `test_flight.py`'s
module docstring and CLAUDE.md's dated section for the full reasoning.

## Tier 2 observability mechanism

`FunctionActuatorSet` copies `vehicle_command_s.param1-6` into up to 6
"Peripheral_via_Actuator_SetN" output-function slots when
`command == DO_SET_ACTUATOR && index (param7) == 0`. Assigning a spare PWM
output channel's `PWM_MAIN_FUNC<n>` parameter to the matching function enum
(e.g. 305 for Actuator 5) makes the decoded value observable as a real PWM
microsecond value via `ACTUATOR_OUTPUT_STATUS`
(PWM = 1000 + (value+1)×500). `sihsim_quadx` only assigns
`PWM_MAIN_FUNC1-4` to the four motors — but only channels up to **8** ever
actually activate in `ACTUATOR_OUTPUT_STATUS`; channels 9+ accept the
`PWM_MAIN_FUNC` parameter without error but never come alive (`pwm_out_sim`'s
simulated output count appears fixed at boot, not dynamically extended —
see CLAUDE.md's dated section). Confirmed empirically that the output only
reflects the commanded value once **armed** (no takeoff/flight needed —
DO_SET_ACTUATOR is instant).

**Portability**: `PWM_MAIN_FUNC<n>`/`Peripheral_via_Actuator_SetN` are
PX4-specific. Every test in this file first calls
`_ensure_actuator_output_observable()`, which skips the whole file with a
specific reason if a stack doesn't accept the parameter or doesn't stream
`ACTUATOR_OUTPUT_STATUS` for it — rather than every test failing with a
misleading "PWM wrong" that's actually "can't observe this stack this
way." Not yet exercised against a real non-PX4 stack (ArduCopter SITL is
currently blocked in this environment — root CLAUDE.md future-work #7).

## Running

```
# Mock
pytest tests/command/do_set_actuator/ -v --log-cli-level=INFO

# PX4 MC
pytest tests/command/do_set_actuator/ \
  --drone-address=udp://:14540 --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \
  --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO
```

## Results (PX4 MC, 1.18.0-beta, PR #28723 branch, rebuilt 2026-09-17)

All tests **PASS** against a correctly-rebuilt binary (see `CLAUDE.md`'s
stale-binary note):

| Test | Result | What it proves |
|---|---|---|
| `test_actuator_compat_command_int_scales_by_1e7` | PASS — PWM=1750 | COMMAND_INT x=5,000,000 (Actuator 5) decodes to 0.5, scaled by 1e7 (see docstring caveat: this one alone also passes on unpatched mainline — kept for documentation value, not discriminating power) |
| `test_actuator_compat_sentinel_independent_per_field` | PASS | x=INT32_MAX (Actuator 5) ignored independently while y=5,000,000 (Actuator 6) takes effect — the fix's per-field sentinel independence |
| `test_actuator_compat_local_frame_scales_same` | PASS — PWM=1750 | Same result under MAV_FRAME_LOCAL_NED (frame=1) — the exact frame the pre-fix bug mis-scaled via the 1e4 divisor |
| `test_actuator_info_command_long_matches_command_int` | PASS — PWM=1750 | COMMAND_LONG param5=0.5 produces the same result (was never buggy; equivalence coverage) |
| `test_actuator_compat_actuators1to4_reach_output` | PASS | Actuator 1-4 (params 1-4, plain floats, never affected by this PR) each independently reach their own correct PWM — full param coverage, closes the `"supported": null` gap for params 1-4 |

First verification attempt (pre-rebuild) failed exactly the two tests that
depend on `ea5734b460`'s per-field-independence refinement
(`sentinel_independent_per_field`, `local_frame_scales_same`) — the
`command_int_scales_by_1e7` test (which only needs the earlier
`0f029991b2` commit) passed even on the stale binary, correctly
pinpointing which of the two fix commits was missing.

## Negative control: verified against genuinely unpatched PX4 mainline

See CLAUDE.md's dated section for the full account. Built PX4 from the
commit immediately before either fix commit and re-ran the suite
unmodified: 5 of 33 tests (across both mission and command) correctly
failed, cleanly matching the two commits' real scope — with one honestly-
reported surprise: the simplest test (`command_int_scales_by_1e7`) passes
even pre-fix, meaning it alone wouldn't have caught the regression; the
edge-case tests are what actually discriminate.
