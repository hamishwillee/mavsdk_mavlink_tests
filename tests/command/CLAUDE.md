# Command protocol — implementation notes

See `tests/command/README.md` for survey result tables and per-stack test results.
Per-command deep-dives (full result tables, traces, flight findings) live in each command's own `README.md` — this file covers cross-cutting protocol mechanics and gotchas only.

## ACK uniqueness (`test_ack_uniqueness.py`)

Complements `test_survey.py` by asking "how many terminal ACKs?" instead of "what result?" — sends every MAV_CMD via COMMAND_INT and stays subscribed for a 1.5s window so a delayed duplicate ACK isn't missed. IN_PROGRESS ACKs are exempt; a duplicate terminal ACK is a failure (unlike the survey, this test is not purely observational).

Confirmed clean on real PX4 MC 1.18.0-beta: 145 commands with exactly one terminal ACK, 23 UNKNOWN, 0 duplicates. **Mock-only** duplicate: `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` (cmd=520) is ACK'd twice by the mavsdk_server binary itself (a legacy auto-responder — see root `CLAUDE.md` § Capability response interaction). Excluded from the mock-mode assertion via `_MOCK_ONLY_KNOWN_DUPLICATES`.

## COMMAND_INT vs COMMAND_LONG selection rules

For a command's per-parameter semantic tests, pick ONE primary message type:

1. **hasLocation/isDestination** (params 5/6 carry lat/lon): **COMMAND_INT** (integer ×1e7 preserves precision); fall back to COMMAND_LONG if COMMAND_INT is rejected.
2. **No location, non-integer floats in params 5/6** (speed, duration, camera ID, etc.): **COMMAND_LONG** — COMMAND_INT would force them through int32 `x`/`y`.
3. **Otherwise**: COMMAND_INT.

Examples: NAV_TAKEOFF/NAV_LAND → COMMAND_INT. DO_SET_MISSION_CURRENT/EXTERNAL_WIND_ESTIMATE → COMMAND_LONG.

This choice only affects a command's own per-parameter tests — the **mandatory common tests** below always send both message types regardless.

## Mandatory common tests (every command's `test_command.py`)

Six checks required for every command, in addition to its own per-parameter tests. Checks 1–5 go via both message types (`probe_dual()` in `conftest.py`, reduced with `effective_ack()`); report the result once, and only call out the two separately if they *disagree*.

1. **Always ACKs** — no response from either message type is a spec violation.
2. **Not UNSUPPORTED** — baseline (valid, meaningful params) must not ACK `UNSUPPORTED(3)`.
3. **Exactly one terminal ACK** — collect the full window (`probe_command_*_all_acks()`); more than one is a bug (own bug, or a stack-side race — see § EXTERNAL_WIND_ESTIMATE below).
4. **Undefined ("Empty") params accept their sentinel, reject anything else** — an accepted/rejected pair per param:
   - sentinel value (defined below) → not `UNSUPPORTED`/`DENIED`.
   - a real, non-sentinel value → `DENIED`.
5. **Defined (used) params tolerate their sentinel** — sending the sentinel on a *used* param must not `DENIED`, even where the spec text doesn't explicitly document it — treat this as the default expectation. Exempt a param only when the docs make the value mandatory with no fallback (e.g. DO_SET_GLOBAL_ORIGIN's lat/lon).
6. **Frame validation survey** — COMMAND_INT only. Send the baseline across all 22 `MAV_FRAME_CATALOGUE` values and check for `MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME` (9). Observational: seeing 9 for any frame is positive evidence of validation (PASS); seeing it for none is **INCONCLUSIVE**, never a failure.

**Sentinel rule**: `NaN` for a float slot, `INT32_MAX` for an int32 slot. `INT32_MAX` only ever applies to a param whose COMMAND_INT wire form is `x`/`y` (the only genuinely int32 fields either message type has); everywhere else — including `z`/param7 and param1–4 — the sentinel is `NaN` in both message types. Testing `INT32_MAX` on a non-`x`/`y` slot is a bug, not extra coverage.

**Naming**: see root `CLAUDE.md` rule 9 for the full convention (`test_protocol_` reserved for genuine protocol-mechanics tests; a bespoke per-command test is named after the command, `test_<command_dir_name>_<rest>`; a shared/inherited test is exempt from the command-name prefix). Concretely, checks 4/5 are pytest-parametrized methods on `Tier1CommandTestBase` (below) — `test_undefined_param_sentinel_accepted`, `test_undefined_param_nonsentinel_rejected`, `test_defined_param_sentinel_tolerated`, each with a `[param{N}]` id (e.g. `test_undefined_param_sentinel_accepted[param5]`) — no command-name prefix, since the class/`SPEC` already establishes which command; "defined"/"undefined" is the real, differentiating signal in these names, not filler. Each test's docstring first line is the single source of truth for its pass case — reused verbatim in the results log and any README table.

**Results report**: every `test_command.py` run writes `reports/command_<command>_<autopilot>_<vehicle>_<version>_<timestamp>.log` (test name, outcome, `MAV_RESULT`, pass case) plus a matching `.json` (mavlink-compat-data schema), always, regardless of pass/fail — combined with the same command's `test_flight.py` Tier 2 results if that ran in the same session too. `_check()`/`_record()`/`_write_tier1_log` in `conftest.py` are thin wrappers over the shared `tests/report.py` (root `CLAUDE.md`'s Tier 2 design pattern #7 has the full architecture — this replaced the old per-tier `logs/..._tier1_...log` convention on 2026-09-16).

### Shared Tier 1 infrastructure (`Tier1CommandTestBase`, `conftest.py`)

The six checks above are implemented once in `conftest.py`, not copy-pasted per command:

- **`CommandSpec`/`ParamSpec`** declare a command: `baseline` (valid `probe_dual()` kwargs) and `params` (one `ParamSpec` per slot 1–7 — `label`, `defined`, and optionally `sentinel_policy="deny_required"` for a mandatory-no-fallback field, or `reject_xfail_reason` for a documented per-command gap). The COMMAND_INT wire mapping (5→`x` int32, 6→`y` int32, 7→`z` float, 1–4→float) is derived purely from `slot` — it's a property of the message struct, not of whether the command uses that slot.
- **`Tier1CommandTestBase`** provides the four fixed-shape checks as real methods and the two count-varying ones parametrized per-command by a `pytest_generate_tests` hook reading `SPEC.undefined_params`/`SPEC.defined_params`. A command file needs only `SPEC = CommandSpec(...)`, `class TestXxxCommand(Tier1CommandTestBase): SPEC = SPEC`, and its own bespoke tests alongside — see `external_wind_estimate/test_command.py`.
- `__init_subclass__` gives each subclass fresh `_RESULTS`/`_DETAILS`/`_supported` — never move these onto the base class; multiple commands' test classes run in one session.
- Overriding an inherited method (e.g. to attach a command-specific xfail reason) is expected — subclass it like any Python method.

**Migration status**: `external_wind_estimate`, `nav_takeoff`, and `do_set_actuator` are fully migrated onto `Tier1CommandTestBase` — the six checks above are inherited, not written per-command. `nav_land`, `do_reposition`, `do_set_mission_current`, `do_set_global_origin` still hand-roll some or all of the six checks, but as of 2026-09-17 all four *also* report via the generic `_tier1_auto_record`/`_tier2_auto_record` fixtures (`tests/report.py`/`tests/flight_helpers.py`) — so every command in this directory produces a `reports/command_...{.log,.json}` pair regardless of migration status; "migrated" here is only about whether the six checks themselves are inherited vs. hand-written, not about reporting coverage (reporting coverage is already 100%).

## MAV_RESULT values

| Value | Name | Meaning |
|-------|------|---------|
| 0 | ACCEPTED | Command accepted, executing |
| 1 | TEMPORARILY_REJECTED | Temporarily rejected (stack busy); retry |
| 2 | DENIED | Refused (params or state issue) |
| 3 | UNSUPPORTED | Command not known to this stack |
| 4 | FAILED | Attempted but failed |
| 5 | IN_PROGRESS | Still executing; more ACKs will follow |
| 6 | CANCELLED | Was executing; now cancelled |
| 7 | COMMAND_LONG_ONLY | Must use COMMAND_LONG, not COMMAND_INT |
| 8 | COMMAND_INT_ONLY | Must use COMMAND_INT, not COMMAND_LONG |
| 9 | COMMAND_UNSUPPORTED_MAV_FRAME | A frame is required and the specified frame is not supported |
| 10 | NOT_IN_CONTROL | Source system is not in control of the target (`<wip/>` in common.xml) |

CANCELLED (6) is absent from pymavlink 2.4.49 — always use the MAVLink submodule XML as the authoritative source.

## Command protocol flow

1. GCS sends COMMAND_INT (or COMMAND_LONG).
2. Stack sends COMMAND_ACK with MAV_RESULT.
3. No ACK within timeout (default 5s): retransmit. COMMAND_INT retries are identical (no confirmation field); COMMAND_LONG increments `confirmation`.
4. After max retries with no ACK: `UNKNOWN`, not `UNSUPPORTED`.

## No-ACK policy

A missing ACK is a spec violation but ambiguous (busy/dropped, silently executing, or truly unsupported). **Policy**: treat "no ACK within timeout" as `UNKNOWN`, never `UNSUPPORTED`. Log at WARNING.

## NaN in float fields

Pass `None` (Python) in `fields_json` to encode NaN on the wire: `None` → JSON `null` → nlohmann/json decodes as IEEE-754 NaN → pymavlink re-encodes as `null` → Python receives `None`. Never use `float('nan')` — `json.dumps` produces non-standard `'NaN'`, which nlohmann/json rejects as `INVALID_FIELD`.

## Command vs mission protocol differences

Mission protocol (MISSION_ITEM_INT upload) *stores* parameters for later execution, possibly normalising them; command protocol (COMMAND_INT direct) uses them immediately, and a stack may ignore ones it doesn't act on in real time. Example — NAV_TAKEOFF param4 (Yaw): PX4 mission stores it, but COMMAND_INT forces yaw=NaN (`navigator_main.cpp:630`); ArduPilot mission doesn't store it at all, and COMMAND_INT ignores it (`GCS_MAVLink_Copter.cpp:585`, "not supported").

## Per-command gotchas

Full result tables and flight findings live in each command's own README — these are the cross-cutting notes worth surfacing here.

- **NAV_LAND (cmd=21)** — see `nav_land/README.md`. Structurally like NAV_TAKEOFF but `z`/param7 means "landing altitude (ground level)", not a climb destination — and param1's NaN behaviour is a spec gap (unlike NAV_TAKEOFF's explicit sentinel). Tier 2 headline: on every platform where landing was observed (PX4 MC/VTOL, ArduCopter MC), the commanded x/y is **not** the touchdown point — the vehicle descends in place from wherever it already is.
- **DO_SET_MISSION_CURRENT (cmd=224)** — see `do_set_mission_current/README.md`. COMMAND_LONG primary. Authoritative behaviour matrix (maintainer-provided): no mission uploaded → `FAILED` for any params, regardless of validity; with a mission, param1 (`-1`=keep current, valid index=set, out-of-range=`FAILED`, other=`DENIED`) and param2 (`0`/`1`=`ACCEPTED`, resets `DO_JUMP` counters when `1`, other=`DENIED`). Confirmed on PX4 MC: 18/18 Tier 1 tests match the matrix; Tier 2 confirms `param2=1` genuinely resets a `DO_JUMP` counter, but does **not** gate whether a completed mission can resume (that's purely positional in PX4 — see README for the full trace).
- **EXTERNAL_WIND_ESTIMATE (cmd=43004, development.xml)** — see `external_wind_estimate/README.md`. COMMAND_LONG primary. Two confirmed PX4 findings, both **fixed** on branch `fix_external_wind_estimate_mavlink` (commit `793d308c53`, plus follow-ups `b4a5854c62`/`73bcd5fb67`, re-verified 2026-09-17 at HEAD `ae61d09f9a`): (1) a Commander/EKF2 dual-ACK race — `EXTERNAL_WIND_ESTIMATE` was missing from Commander's "handled elsewhere" ignore-list, so it fell through to `UNSUPPORTED` racing EKF2's `ACCEPTED`; fixed by adding the missing case. (2) a DOC DISCREPANCY — `resetWindToExternalObservation()` is gated by landed state (still true post-fix, and still a real functional gap vs. the spec's in-flight use case), but the ACK previously claimed `ACCEPTED` unconditionally either way; it now correctly returns `TEMPORARILY_REJECTED` when airborne, so the silent ACK/behaviour mismatch itself is resolved even though PX4 still doesn't implement in-flight wind reset.

## MAVLink XML submodule

`mavlink/message_definitions/v1.0/common.xml` has 168 MAV_CMD entries (includes standard.xml → minimal.xml). Pymavlink 2.4.49 bundles only 155 and is missing `MAV_RESULT_CANCELLED`; always use the submodule XML for the survey and result lookups.

```bash
git submodule update --init mavlink   # one-time after cloning
pytest tests/command/ --mavlink-definitions-dir=/path/to/other/message_definitions/v1.0
```

## ArduCopter mode restriction for NAV_TAKEOFF

`GCS_MAVLink_Copter.cpp:578` checks `has_user_takeoff(must_navigate)`. With default param3=0 (`must_navigate=true`):

| Mode | Accepts NAV_TAKEOFF | Autonomous climb (no RC) |
|------|---------------------|--------------------------|
| STABILIZE, ALT_HOLD | ❌ | — |
| **GUIDED** | ✅ | ✅ `_AutoTakeoff::run()` |
| LOITER, POSHOLD | ✅ | ❌ needs RC throttle |

Only GUIDED uses the autonomous controller; LOITER/POSHOLD accept the command but read the RC throttle channel. All ArduPilot autotest calls precede `user_takeoff()` with `change_mode("GUIDED")`.

MAVSDK quirks (see `baseline_takeoff/README.md`): reports ArduCopter GUIDED (custom_mode=4) as `"OFFBOARD"`; ArduCopter/ArduPlane don't stream `GLOBAL_POSITION_INT` by default — send `MAV_CMD_SET_MESSAGE_INTERVAL(511, param1=33)` first.
