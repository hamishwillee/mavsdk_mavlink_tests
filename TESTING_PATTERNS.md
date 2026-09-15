# Testing patterns — rollout tracker

A living checklist of structural/conceptual conventions developed for MAV_CMD
testing (mostly during the 2026-09-14/15 NAV_TAKEOFF work), separate from
`CLAUDE.md` so that file can stay terse narrative ("what and why") while this
one stays a blunt status table ("where is it actually adopted yet").

**How to use this file**:
- Whenever a new structural/conceptual pattern is adopted (not a one-off bug
  fix, not a finding about a specific stack's behaviour), add a row here and
  a corresponding rule/pattern entry in the relevant `CLAUDE.md` (root for
  cross-cutting patterns, `tests/mission/CLAUDE.md` / `tests/command/CLAUDE.md`
  for protocol-specific ones).
- Whenever a pattern gets rolled out to a directory that was previously
  "pending," flip its cell to ✓ in the same commit as the rollout.
- Treat every "pending" cell as a literal backlog item, not just a note.
- Periodically (start of a session touching this area, or when asked) scan
  for drift: a pattern marked ✓ somewhere may have silently regressed, or a
  new test file may have been added without picking up patterns it should
  have from day one.
- Keep entries terse. Full reasoning lives in `CLAUDE.md`; link to it, don't
  repeat it here.

## Directories tracked

**Mission** (`tests/mission/`): `nav_takeoff`, `do_reposition` (Tier 1 only —
rejected everywhere, nothing to fly), `condition_gate`.
**Command** (`tests/command/`): `nav_takeoff`, `nav_land`, `do_reposition`,
`do_set_mission_current`, `do_set_global_origin` (Tier 1 only — no Tier 2
flight semantics), `external_wind_estimate`, `baseline_takeoff` (bespoke,
predates the shared Tier 1 base).

## Tier 1 (protocol) patterns

| Pattern | Description | mission/nav_takeoff | mission/do_reposition | mission/condition_gate | command/nav_takeoff | command/nav_land | command/do_reposition | command/do_set_mission_current | command/do_set_global_origin | command/external_wind_estimate | command/baseline_takeoff |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Shared base class (`Tier1MissionTestBase`/`Tier1CommandTestBase`, `MissionItemSpec`/`CommandSpec`, `ParamSpec`) | One shared baseline/sentinel test suite instead of hand-rolled per-command Tier 1 tests — `tests/mission/CLAUDE.md` § Shared Tier 1 infrastructure | ✓ | ✓ | ✓ | ✓ | ✗ | ✗ | ✗ | ✗ | ✓ | ✗ (bespoke `test_baseline.py`) |
| Sentinel + non-sentinel probe for every param, both directions | For each param: does the sentinel round-trip untouched, and does a real non-sentinel value NACK (unsupported) or get accepted (possibly supported)? Baked into the shared base's generic tests where adopted (row above) | ✓ (via shared base) | ✓ (via shared base) | ✓ (via shared base) | ✓ (via shared base) | not audited | not audited | not audited | not audited | ✓ (via shared base) | not audited |
| Param limits: XML-stated explicit range AND type-inferred implicit range | `tests/mission/CLAUDE.md` § "Ranges for defined float params" — one value just outside an XML `minValue`/`maxValue`, or (no XML bounds) a value outside the *physically implied* range (e.g. yaw outside [0°,360°)) | ✓ (yaw/pitch edge-value tests) | n/a (command rejected outright) | not audited | not audited | not audited | not audited | not audited | not audited | not audited | not audited |

## Tier 2 (execution) patterns

| Pattern | Description | mission/nav_takeoff | mission/condition_gate | command/nav_takeoff | command/nav_land | command/do_reposition | command/do_set_mission_current | command/external_wind_estimate | command/baseline_takeoff |
|---|---|---|---|---|---|---|---|---|---|
| Naming: `test_<cmd>_<compat\|info\|obs>_<description>` (category as 3rd word, not trailing suffix) | Root `CLAUDE.md` rule 8. Lets `_compat`/`_info`/`_obs` tests visibly group in an alphabetised/`-v` test list | ✓ | ✗ (plain descriptive names) | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ |
| Explicit PASS/FAIL criterion in every detail line (`"PASS if X: <measured>"`, not just raw numbers) | So the Tier 2 log is self-explanatory without reading the test source | ✓ | partial | not audited | not audited | not audited | not audited | not audited | not audited |
| Compatibility summary (`record_tier2_param_verdict`) | Per-param SUPPORTED/NOT SUPPORTED/etc. rollup appended after the main results table | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ |
| NACK-aware compat verdict correlation (`_compat_verdict`: NACK / SUPPORTED / COMPATIBILITY ERROR) | Root `CLAUDE.md` rule 4a — an accepted-but-not-honoured value is a confirmed compatibility error, distinct from a legitimate NACK-based rejection | ✓ (yaw/pitch/position) | n/a | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ |
| NA-gating for dependent characterisation tests (cache the core probe, skip re-flying, report NA not FAIL) | Root `CLAUDE.md` rule 7 — an edge-value variant of an already-known-unsupported param shouldn't re-fly and re-fail the same fact | ✓ | n/a (no dependent edge-value tests) | not audited | not audited | not audited | not audited | not audited | not audited |
| Never gate "is it honoured" on `--vehicle-type` — pick one value whose *measured outcome* distinguishes support | Root `CLAUDE.md` rule 5 | ✓ | n/a | not audited | not audited | not audited | not audited | not audited | not audited |
| Shared Tier 2 auto-logging (`_tier2_auto_record`, `record_tier2_detail`) | Root `CLAUDE.md` Tier 2 pattern #7 — automatic PASS/FAIL/NA capture to a log file, two constants + one import to adopt | ✓ | ✗ | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ |
| Restart-instead-of-RTL cleanup (`restart_flight_stack`, passed into `_rtl_and_land`) | Root `CLAUDE.md` Tier 2 pattern #8 (2026-09-15) — RTL doesn't reliably land a vehicle (ArduPlane RTL_AUTOLAND defaults off; ArduPilot correctly refuses an in-air disarm), so a botched landing must restart the SITL, not force a disarm | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ |
| Order an ambiguous characterisation test last; cross-reference sibling tests' results for explanatory power | Root `CLAUDE.md` Tier 2 pattern #9 (2026-09-15) — e.g. `test_takeoff_info_implicit_from_waypoint` running last so it can say "TAKEOFF item required for this frame" instead of an ambiguous timeout | ✓ | n/a (not applicable to this command) | n/a | n/a | n/a | n/a | n/a | n/a |
| mavlink-compat-data JSON export (`record_compat_json`/`record_compat_command_supported` in `tests/flight_helpers.py`, rendered via `_render_compat_json` and appended to the Tier 2 log) | Matches the schema at github.com/hamishwillee/mavlink-compat-data (PR #9) so a stack's Tier 2 run produces a directly mergeable JSON fragment — per-param `supported`/`accept_nan_or_int32max`/`nacks_on_non_sentinel_value`/`notes`, frame-level `notes`, dev-build detection (`basis:"testing"` + upstream-merge warning comment for a `-dev`/`-beta`/`-rc`/`main` version string). A module opts into full-schema completeness by declaring `_COMPAT_JSON_ALL_PARAMS` (its command's full param-slot list) — every declared param then always renders, `"supported": null` ("not independently tested") for any whose test errored out before recording anything, instead of being silently absent (indistinguishable from "forgot to test it") | ✓ Validated 2026-09-15 against real PX4 MC (v1.17.0, `basis:"verified"`) and ArduPlane FW (4.8.0-dev, correctly downgraded to `basis:"testing"` with the dev-build warning) — both runs render valid, schema-shaped JSON including params 2/3 (protocol-only probes) and frame-level notes. `_COMPAT_JSON_ALL_PARAMS` added same day after PX4 fixed-wing's release run showed 1_Pitch/4_Yaw/7_Altitude silently missing (their tests all errored with TimeoutError before recording) — user caught it and specified the schema-correct fix (`null`, not a text note) | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ |
| Two-value altitude-tracking test pair (`_fly_to_altitude_target`, `test_takeoff_obs_altitude_tracks_low`/`test_takeoff_compat_altitude_tracks_high`) | Fly two widely-separated commanded values every run (no single-value test, no NA-gating); PASS if the measured outcome scales by at least a minimum margin between them (distinguishes "genuinely ignores the param" from "uses it as a target but subject to a separate, legitimate completion condition") | ✓ Validated 2026-09-15 against real hardware (PX4 MC, ArduPlane FW release, ArduPlane QuadPlane release) — all three correctly `TRACKS`. The original single-value + NA-gated design (dropped 2026-09-15) never actually exercised the comparison logic in 2/2 real runs, since the primary test kept passing on its own — this was the user's own call: "I think we should drop the original altitude check and always run both" | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ |
| Thin, stack-keyed setup layer for flight-stack-specific "how does this stack even become flyable" quirks (root `CLAUDE.md` Tier 2 pattern #11) | Special per-stack boot/arming requirements (e.g. missing INS calibration defaults) get isolated in one setup function keyed on vehicle model — never inlined into individual test bodies, so test logic stays identical across stacks | ✓ `tests/conftest.py`'s `_ardu_model_and_defaults()` (ArduPlane INS-cal defaults fix, 2026-09-15) | n/a | n/a | n/a | n/a | n/a | n/a | n/a |

## Infrastructure (applies harness-wide, not per-file)

| Pattern | Status |
|---|---|
| Multi-instance SITL (`--sitl-instance`, port-formula-derived, instance-scoped kill/restart logic) | ✓ Done, 2026-09-15 — `tests/conftest.py`, applies automatically to every test file (nothing per-file to adopt). See root `CLAUDE.md`'s CI section. |
| `restart_flight_stack` fixture (kill+restart SITL and its mavsdk_server bridge) | ✓ Built AND validated against real hardware, 2026-09-15 — fixed ArduPlane FW's near-total Tier 2 cascade (0-4 usable results before → all 15 tests completing with real data after). Available to every Tier 2 test via `tests/flight_helpers.py`'s `_rtl_and_land(system, restart_flight_stack)`. Per-file adoption tracked in the Tier 2 table above (currently nav_takeoff only). |
| Running the test *suite* across multiple instances in parallel (orchestration) | Partially validated, 2026-09-15: PX4 MC (instance 0) and ArduPlane FW (instance 1) run concurrently via two hand-launched pytest invocations, no interference. Still no driver script/xdist integration for automatic sharding — see root `CLAUDE.md`'s CI section, "Future work" subsection, for the two concrete next steps (shard-by-vehicle-type script; `pytest-xdist` worker-to-instance mapping). |
| `paired_drone_server`/`paired_gcs_server`/`GCS_MAVLINK_PORT` instance-offsetting (10×instance, same formula as the standalone ports) | ✓ Bug found AND fixed AND re-validated, 2026-09-15 — these fixtures start unconditionally every session (paired mode or not), so two concurrent standalone sessions on fixed ports collided (PX4 instance 0 errored at setup while ArduPlane instance 1, which grabbed the port first, ran fine). Fixed in `tests/conftest.py`; re-ran the same concurrent PX4 MC (instance 0) + ArduPlane FW (instance 1) combination afterward — both sessions completed a full run with no setup errors this time. |

## Process patterns (not technical, but part of the same discipline)

- **Roll out deliberately, not automatically.** A pattern proven on nav_takeoff doesn't silently propagate — each directory's row above gets flipped only when someone (a session, or an explicit ask) actually does the rollout and verifies it against real hardware where relevant.
- **Audit before rollout, don't assume.** Several cells above say "not audited" rather than a guessed ✓/✗ — check the actual file before rolling out a pattern that might already be partially present in a different shape.
- **Update this file in the same change that adopts/changes a pattern** — not as a separate cleanup pass later (it won't happen).
- **Keep `CLAUDE.md` terse.** When a pattern's rule text grows stale or redundant with this file's own description, trim the `CLAUDE.md` prose rather than letting both drift into saying the same thing at different lengths.
