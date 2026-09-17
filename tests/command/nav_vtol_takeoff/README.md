# MAV_CMD_NAV_VTOL_TAKEOFF — Command Protocol (COMMAND_INT) Test Results

Tests the **command protocol** path (COMMAND_INT/COMMAND_LONG → COMMAND_ACK) for `MAV_CMD_NAV_VTOL_TAKEOFF` (cmd=84) — the VTOL-specific sibling of `NAV_TAKEOFF` (cmd=22, see `../nav_takeoff/`). No Tier 2 (flight) test exists yet — see root `CLAUDE.md` future-work item #10: PX4 VTOL Tier 2 flight tests are currently blocked in this environment (arms and starts a mission but never leaves the ground; root cause still open).

Built on `Tier1CommandTestBase`/`CommandSpec` (`../conftest.py`), same framework as `nav_takeoff`, `do_set_actuator`, `external_wind_estimate`.

## PX4's real command-protocol param semantics — XML vs. source

common.xml marks param1/param3 "Empty" and param2 as the `VTOL_TRANSITION_HEADING` enum. Reading PX4's own standalone-command handler (`navigator_main.cpp`, `VEHICLE_CMD_NAV_VTOL_TAKEOFF` branch) shows real, source-confirmed meaning beyond the XML for most slots:

| Param | XML | PX4 command-protocol handler |
|-------|-----|-------------------------------|
| param1 | Empty | **Loiter Height** — height above takeoff altitude for the post-transition loiter (`_vtol_takeoff.setLoiterHeight(cmd.param1)`) |
| param2 | Transition Heading (enum 0-4) | Only ever compared for exact equality to `3.0` (`VTOL_TRANSITION_HEADING_SPECIFIED`); if equal, `setTransitionDirection(param4)` is called. The other four enum values are otherwise indistinguishable to this handler. |
| param3 | Empty | Genuinely unused — matches the XML |
| param4 | Yaw Angle | Transition bearing, but **only consulted when param2==3.0** |
| param5/6 | Latitude/Longitude | Post-transition loiter location (`setLoiterLocation`) |
| param7 | Altitude | **Transition Altitude** (absolute) — `setTransitionAltitudeAbsolute(cmd.param7)` |

This is an implementation-specific extension, not a spec violation — the XML doesn't forbid a stack from giving an "Empty" slot real meaning, it just doesn't define one (root `CLAUDE.md`'s general testing philosophy rule 3).

## Fix verified: PX4 commit `aad2f0f3` ("fix(mavlink): allow p1/p2 for standalone NAV_VTOL_TAKEOFF command")

PX4 has a separate MAVLink-boundary parameter mask (`src/modules/mavlink/mavlink_command_params.hpp`) that DENIES any non-"unset" value for a param outside a per-command allow-list, before the command ever reaches Navigator. Before this fix, cmd=84's command-path mask was `0x7C` — allowing only params 3/4/5/6/7 (param3, despite being genuinely unused!) and **denying any non-zero param1/param2**, even though Navigator's own handler reads both. Fixed same-day to `0x7B` — param1/param2/param4-7 now allowed, param3 (the one genuinely unused slot) correctly the only one still denied.

`test_param1_loiter_height_accepted` verifies this directly at the ACK level — the one place Tier 1 can observe it (a real param1 value flipping from DENIED to ACCEPTED):

```
NAV_VTOL_TAKEOFF | param1 (Loiter Height) = 20.0                | result=0   (ACCEPTED — fix confirmed)
NAV_VTOL_TAKEOFF | param3 (Empty) = non-sentinel                | result=2   (DENIED — correctly still the only rejected slot)
```

## Message-type exclusivity — a real, verified PX4 fix

`test_hasLocation_rejects_command_long` (inherited, check 7 — see `../CLAUDE.md` § Mandatory common tests) is a genuine **PASS** for this command: PX4 commit `83e7afba56` adds `MavlinkReceiver::command_is_int_only()`, which NACKs a COMMAND_LONG send of `NAV_VTOL_TAKEOFF` with `MAV_RESULT_COMMAND_INT_ONLY(8)`, confirmed independent of MAVSDK via a raw pymavlink probe. This is the *only* command in this test suite where that check currently passes — `NAV_TAKEOFF`/`NAV_LAND`/`DO_REPOSITION` all still `XFAIL` it on the same PX4 build. Full story, including a currently-open, not-yet-root-caused instability in a different, pre-existing test (`test_param1_loiter_height_accepted`) discovered during this same investigation: see `CLAUDE.md`.

## Tier 1 (ACK) results

28 tests: `test_command_ack_received`, `test_command_supported`, `test_exactly_one_ack`, `test_frame_validation_survey`, `test_undefined_param_{sentinel_accepted,nonsentinel_rejected}[param1,param3]`, `test_defined_param_sentinel_tolerated[param2,4,5,6,7]`, `test_hasLocation_rejects_command_long`, and `test_float_params5_6_rejects_command_int` are inherited from `Tier1CommandTestBase` (`../CLAUDE.md`); the rest are this command's own bespoke tests, mirroring `nav_takeoff/test_command.py`'s structure for the params the two commands share.

### PX4 VTOL (0.0.0-official, `sihsim_standard_vtol`, tested 2026-09-17 against a PX4-Autopilot checkout at `~/github/PX4/PX4-Autopilot`)

24 PASS, 1 SKIP (NA — `test_float_params5_6_rejects_command_int`, this command has no non-location float in param5/6), 3 XFAIL. `test_hasLocation_rejects_command_long`: **PASS** (see above). `test_param1_loiter_height_accepted`: intermittently **FAIL** later the same session against an unmodified running binary — see `CLAUDE.md`'s open finding; not reflected in the counts below, which are from the earlier, passing run.

| Test | Result | Notes |
|------|--------|-------|
| `test_command_ack_received` / `test_command_supported` | PASS — ACCEPTED | |
| `test_exactly_one_ack` | PASS | |
| `test_frame_validation_survey` | INCONCLUSIVE | all 22 frames ACKed |
| `test_undefined_param_sentinel_accepted[param1,param3]` | PASS ×2 — ACCEPTED | |
| `test_undefined_param_nonsentinel_rejected[param1]` | **XFAIL** — ACCEPTED | param1 is genuinely used as Loiter Height (source-confirmed); not a validation gap |
| `test_undefined_param_nonsentinel_rejected[param3]` | **PASS** — DENIED | PX4 genuinely validates the one truly-unused slot (mask `0x7B`) |
| `test_defined_param_sentinel_tolerated[param2,4,5,6,7]` | PASS ×5 — ACCEPTED | |
| `test_param2_transition_heading_values` (0-4) | PASS (obs) — ACCEPTED ×5 | |
| `test_param1_loiter_height_accepted` (param1=20.0) | **PASS**, then intermittently **FAIL** (DENIED) later the same session | verifies commit `aad2f0f3` — see `CLAUDE.md`'s open finding, not yet root-caused |
| `test_hasLocation_rejects_command_long` | **PASS** — COMMAND_INT_ONLY(8) | verifies commit `83e7afba56`; the only command in this suite where this check currently passes |
| `test_float_params5_6_rejects_command_int` | SKIP (NA) | this command has no non-location float in param5/6 |
| `test_param2_transition_heading_specified_uses_param4` (param2=3, param4=45°) | PASS (obs) — ACCEPTED | |
| `test_param2_transition_heading_out_of_range` (param2=5) | PASS (obs) — ACCEPTED | |
| `test_param4_yaw_ack` (param4=90°, param2=default) | PASS (obs) — ACCEPTED | yaw genuinely not consulted unless param2==SPECIFIED — not a spec gap |
| `test_param4_yaw_nan_ack` | PASS (obs) — ACCEPTED | |
| `test_location_specific_ack` | PASS — ACCEPTED | |
| `test_location_int32max_ack` | PASS (obs) — ACCEPTED | |
| `test_nan_altitude_ack` | PASS (obs) — ACCEPTED | |
| `test_location_out_of_range_latlon_ack` | **XFAIL** — ACCEPTED | PX4 doesn't validate lat/lon range — same known gap as `nav_takeoff` |
| `test_wrong_frame_ack` | PASS (obs) — ACCEPTED | |
| `test_latlon_nan_command_long_ack` | PASS — ACCEPTED | |
| `test_latlon_int32max_command_long` | **XFAIL** — DENIED | PX4 rejects `float(INT32_MAX)` in COMMAND_LONG param5/6 as a protocol error — same known gap as `nav_takeoff` |

Both XFAILs beyond the param1/param3 story are the identical, already-documented PX4 gaps found on `NAV_TAKEOFF` (`../nav_takeoff/README.md`) — consistent cross-command behaviour, not new findings.

Report: `reports/command_nav_vtol_takeoff_px4_vtol_*.log` / `.json` (mavlink-compat-data schema).

### Other stacks — not yet re-tested against this file

Per `../README.md`'s survey (footnote 1): ArduCopter maps `NAV_VTOL_TAKEOFF` onto its standard takeoff handler (ACCEPTED on any airframe); ArduPlane QuadPlane (the one real VTOL ArduPilot frame) rejects a direct COMMAND_INT — it only recognises this command as a mission-item executed during AUTO (see `../baseline_takeoff/README.md`). PX4 doesn't gate commands by vehicle type, so MC/FW/Rover are expected to match the VTOL results above, per the same pattern already confirmed for `NAV_TAKEOFF` — not independently re-verified against this specific test file.

## Mock (paired)

19 PASS, 2 SKIP (COMMAND_LONG-only NaN/INT32_MAX tests — mock doesn't model those semantics), 3 XFAIL (mock accepts everything by default, including the two undefined-param non-sentinel values and out-of-range lat/lon).

## Running

```bash
pytest tests/command/nav_vtol_takeoff/test_command.py -v --log-cli-level=INFO   # mock, tier 1

pytest tests/command/nav_vtol_takeoff/test_command.py \
    --drone-address=udp://:14540 --connection-timeout=60 \
    --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_standard_vtol \
    --vehicle-type=vtol --autopilot=px4 -v --log-cli-level=INFO   # PX4 VTOL, tier 1
```

Other stacks/vehicle types: swap `--*-sitl`/`--*-model` and `--vehicle-type`/`--autopilot` per root `CLAUDE.md` § Running modes.
