# DO_SET_ACTUATOR — command protocol notes

Built on `Tier1CommandTestBase`/`CommandSpec` (see `tests/command/CLAUDE.md` §
"Shared Tier 1 infrastructure") — the six mandatory checks are inherited;
this file's bespoke tests are `test_do_set_actuator_*` per root `CLAUDE.md`
rule 9.

## Why this directory exists

Added 2026-09-17 to verify [PX4-Autopilot PR #28723](https://github.com/PX4/PX4-Autopilot/pull/28723)
— see `README.md` for the full bug description. `Commander.cpp` answers
this command unconditionally `ACCEPTED` (no validation at all), so Tier 1
cannot distinguish a correct fix from the pre-fix bug; **Tier 2 (PWM
output observation) is the decisive verification** — see `test_flight.py`'s
module docstring for the observability mechanism (empirically confirmed,
not assumed from source alone, before committing to this test design).

## Stale-binary pitfall (root CLAUDE.md design decision 4c) — hit and resolved here

First verification run against this PR's branch produced 3 real-looking
Tier 2 failures. Root cause: the running SITL binary was built **before**
the second of the two fix commits (`ea5734b460`, which landed ~4 hours
after the binary's build timestamp) — confirmed via
`git log -1 --format='%ci' -- <file>` vs. `stat -c '%y' bin/px4`. A clean
rebuild resolved all 3 failures. Also needed `git submodule update --init
--recursive` for `src/drivers/gps/devices` and `src/modules/mavlink/mavlink`
first — both were separately out of sync and the initial rebuild attempt
failed with unrelated GPS-driver compile errors before that was fixed.
**Lesson reinforced**: always check binary-vs-source timestamps before
treating a Tier 2 failure as a genuine finding — this recurred multiple
times across different commands this session.

## What "supported" actually means here (2026-09-17, user-prompted)

`ACTUATOR_OUTPUT_STATUS` reports PX4's own *commanded* output value, not
sensed/measured physical position — there is no closed loop, and SITL has
no physical model for a generic "Peripheral_via_Actuator_Set" output
(unlike the four simulated motors). This is still the correct, sufficient
point to verify PR #28723 (a pure decoding bug upstream of any physical
actuator), but it is genuinely not "confirmed to move an actuator" — see
`test_flight.py`'s "IMPORTANT — what this can and cannot show" section and
the `notes` field on every `record_compat_json`/`record_compat_command_supported`
call, which now says so explicitly in the exported JSON rather than only
in this prose.

## Only channels 1-8 are usable in ACTUATOR_OUTPUT_STATUS (2026-09-17)

Confirmed empirically (direct probe against the running SITL instance):
`PWM_MAIN_FUNC9`/`PWM_MAIN_FUNC10` **parameters exist** (1-16 all do) and
accept an assignment without error, but the corresponding output channels
never set their `active` bit or report a value in `ACTUATOR_OUTPUT_STATUS`
— `pwm_out_sim`'s simulated output count appears fixed at boot from the
airframe's own `PWM_MAIN_FUNC1-4` config, not dynamically extended by a
later `param set`. This blocked the original design (Actuator 1-4 on
dedicated channels 7-10) — fixed by reusing channels 5-8 across two
independent tests instead (Actuator 5/6's own tests use 5/6 for one
purpose; `test_actuator_compat_actuators1to4_reach_output` reuses the same
physical channels 5-8 for Actuator 1-4 — safe, since every test
reconfigures `PWM_MAIN_FUNC<n>` fresh, no ordering dependency).

**A related, first-activation timing issue, also found and fixed**:
`test_actuator_compat_actuators1to4_reach_output` passed cleanly when run
in isolation but showed apparent cross-talk (channel 7 reading channel 5's
value, channel 8 reading channel 6's) when run after the file's other
tests in one session — channels 7/8's first-ever activation needed more
settle time than channels 5/6 (already warmed up by earlier tests) did.
Fixed with a longer settle window (3.5s vs. the file's usual 1.5s) and an
extra post-arm delay for this one test; re-verified clean against the full
file, multiple times.

## Portability: what happens on a stack without this observability mechanism (2026-09-17, user-prompted)

`PWM_MAIN_FUNC<n>`/`Peripheral_via_Actuator_SetN` are PX4-specific.
`_ensure_actuator_output_observable()` (`test_flight.py`) checks once per
session whether the stack accepts the parameter and actually streams
`ACTUATOR_OUTPUT_STATUS` for it, and **skips every test in the file with a
specific reason** if not — rather than every test failing with a
misleading `pwm=None`/`measured=900` that looks like "scaling is wrong"
but actually means "can't observe this stack this way at all." Not yet
run against a real non-PX4 stack to confirm the skip path fires correctly
in practice (ArduCopter SITL is blocked in this environment — see root
`CLAUDE.md` future-work item #7) — the logic is verified by code review
and by the fact that the *positive* path (PX4, working mechanism) is
proven working, but the skip path itself is currently unexercised.

## Negative control: verified against genuinely unpatched PX4 mainline (2026-09-17, user-prompted)

To confirm these tests actually discriminate (not just "always green"),
built PX4 from `651b8687b8` — the commit immediately before **either** fix
commit (via a separate `git worktree`, so the main checkout was never
disturbed) — and ran the full suite against it unmodified.

**Result: 5 of 33 tests correctly FAILED.** This run happened *before* the
channels 9/10 and channel-7/8-warm-up fixes below were found — the failing
5 were:
- `test_actuator_compat_sentinel_independent_per_field` (mission Tier 1 —
  round-trip) and its command-protocol namesake (Tier 2)
- `test_actuator_compat_local_frame_scales_same` (frame=1, the exact bug
  commit `0f029991b2`'s own message describes)
- `test_actuator_compat_mission_item_scales_by_1e7` (mission Tier 2)
- the *original* design of the all-actuators test (channels 7-10 for
  Actuator 1-4, before the channels-1-8-only fix below) — **but this
  failure is NOT evidence about the PR's fix**: param1-4 are plain floats
  in every message type, untouched by either fix commit (both only ever
  touch param5/param6 via x/y), so this test was never expected to
  discriminate pre/post-fix at all — it failed here purely because
  channels 9/10 never activate in `ACTUATOR_OUTPUT_STATUS` on this SITL
  build, a `pwm_out_sim` limitation unrelated to PX4 version. The
  corrected version of this test (channels 5-8, added after this negative-
  control run) has not itself been re-run against the unpatched baseline —
  the worktree was already removed by the time that redesign happened, and
  re-running wasn't warranted since this test was never meant to exercise
  the PR's fix in the first place.

**Interesting, honestly-reported nuance**: `test_actuator_compat_command_int_scales_by_1e7`
(the simplest, first-written test — frame=6, x=5,000,000 alone, no
sentinel/frame edge case) **passed even on the unpatched baseline**.
This means the basic happy-path test alone would NOT have caught the
regression — only the edge-case tests (per-field sentinel independence,
the specific local-frame bug, multi-actuator coverage) actually
discriminate. Not fully root-caused why frame=6 alone happens to scale
correctly pre-fix (plausibly `command_has_location()` returns true for
DO_SET_ACTUATOR in the old code despite the XML's `hasLocation="false"`,
routing frame=6 through the pre-existing generic "global frame → 1e7"
branch by coincidence) — worth revisiting if anyone digs further, but not
pursued here since the *practical* conclusion (the test suite as a whole
correctly discriminates, and no single test would have been sufficient
alone) is already clear and doesn't depend on resolving it. This is a
good validation of writing multiple edge-case tests rather than trusting
one happy-path test.

The original PX4 checkout (`~/github/PX4/PX4-Autopilot`, the PR branch,
already rebuilt with the fix) was untouched throughout — confirmed via
`git status`/binary timestamp before and after — and the temporary
worktree was removed afterward.

## Results

See `README.md` § "Results" — all four Tier 2 tests pass against a
correctly-rebuilt PX4 MC binary, with precise, quantitative PWM
confirmation (not just ACK-level "accepted") of both fix commits.
