# Command Protocol Tests

This directory contains tests for the MAVLink **command protocol** — `COMMAND_INT` and `COMMAND_LONG` messages sent directly to the vehicle and acknowledged via `COMMAND_ACK`.

This is distinct from the **mission protocol** (MISSION_ITEM_INT upload) tested in `tests/mission/`.
The same MAV_CMD may behave differently in the two paths — see `CLAUDE.md § Command vs mission protocol differences`.

## Test structure

| File | Purpose |
|------|---------|
| `test_survey.py` | Probe all 168 MAV_CMD from common.xml; write support matrix to `logs/` |
| `test_protocol.py` | Protocol mechanics: ACK receipt, echo, ACCEPTED result, COMMAND_LONG, retry, IN_PROGRESS |
| `nav_takeoff/test_command.py` | `MAV_CMD_NAV_TAKEOFF` (cmd=22) via COMMAND_INT — ACK result tests |
| `nav_land/test_command.py` | `MAV_CMD_NAV_LAND` (cmd=21) via COMMAND_INT — ACK result tests |
| `do_set_mission_current/test_command.py` | `MAV_CMD_DO_SET_MISSION_CURRENT` (cmd=224) via COMMAND_LONG — ACK result tests |
| `do_set_mission_current/test_flight.py` | `MAV_CMD_DO_SET_MISSION_CURRENT` param2 — Tier 2 behavioural tests: does the reset flag actually reset a `DO_JUMP` repeat counter (PASS on PX4 MC), and does it make a completed mission restartable (RTL-ending mission: XFAIL; corrected Hold-ending mission + `MISSION_START` reactivation: PASS, but shows `param1` alone — not `param2` — gates resumption on PX4; see `do_set_mission_current/README.md`) |

## Running

```bash
# Paired (mock) — all tests pass; 3 mock-only protocol tests run
pytest tests/command/ -v --log-cli-level=INFO

# ArduCopter SITL (auto-managed)
pytest tests/command/ \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
    --ardupilot-sitl=~/ardu_sitl/arducopter \
    --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 \
    --vehicle-type=copter --autopilot=ardupilot \
    -v --log-cli-level=INFO

# ArduPlane fixed-wing SITL
pytest tests/command/ \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
    --ardupilot-sitl=~/ardu_sitl/arduplane \
    --vehicle-type=fixed_wing --autopilot=ardupilot \
    -v --log-cli-level=INFO

# ArduRover SITL
pytest tests/command/ \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
    --ardupilot-sitl=~/ardu_sitl/ardurover \
    --ardupilot-model=rover \
    --vehicle-type=rover --autopilot=ardupilot \
    -v --log-cli-level=INFO

# PX4 SIH multicopter
pytest tests/command/ \
    --drone-address=udp://:14540 --connection-timeout=60 \
    --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \
    --vehicle-type=quadcopter --autopilot=px4 \
    -v --log-cli-level=INFO

# PX4 SIH fixed-wing
pytest tests/command/ \
    --drone-address=udp://:14540 --connection-timeout=60 \
    --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_airplane \
    --vehicle-type=fixed_wing --autopilot=px4 \
    -v --log-cli-level=INFO

# PX4 SIH VTOL
pytest tests/command/ \
    --drone-address=udp://:14540 --connection-timeout=60 \
    --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_standard_vtol \
    --vehicle-type=vtol --autopilot=px4 \
    -v --log-cli-level=INFO

# PX4 SIH rover (ackermann)
pytest tests/command/ \
    --drone-address=udp://:14540 --connection-timeout=60 \
    --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_rover_ackermann \
    --vehicle-type=rover --autopilot=px4 \
    -v --log-cli-level=INFO
```

---

## Command support survey

Probes all 168 `MAV_CMD` entries in `common.xml` via `COMMAND_INT` and classifies the ACK result as SUPPORTED / UNSUPPORTED / UNKNOWN.
The survey always passes — results are observational and written to `logs/command_survey_<stack>_<timestamp>.log`.

**Classification:**
- `SUPPORTED` — any result except UNSUPPORTED (3): ACCEPTED, DENIED, FAILED, IN_PROGRESS, etc.
- `UNSUPPORTED` — ACK result = 3 (MAV_RESULT_UNSUPPORTED)
- `UNKNOWN` — no ACK within timeout (spec violation; may still be supported)

### Survey summary

| Metric | ArduCopter MC | ArduPlane FW | ArduPlane QP | ArduRover | PX4 MC | PX4 FW | PX4 VTOL | PX4 Rover | Mock |
|--------|---------------|--------------|--------------|-----------|--------|--------|----------|-----------|------|
| SUPPORTED | 61 | 12 | 12 | 49 | 36 | 37 | 38 | 35 | 168 |
| UNSUPPORTED | 106 | 28 | 29 | 118 | 105 | 105 | 105 | 106 | 0 |
| UNKNOWN | 1 | 128 | 127 | 1 | 27 | 26 | 25 | 27 | 0 |
| Log | `command_survey_ardupilot_copter_4.8.0-dev_*` | `command_survey_ardupilot_fixed_wing_4.8.0-dev_*` | `command_survey_ardupilot_quadplane_4.8.0-dev_*` | `command_survey_ardupilot_rover_4.8.0-dev_*` | `command_survey_px4_quadcopter_1.18.0-alpha_*` | `command_survey_px4_fixed_wing_1.18.0-alpha_*` | `command_survey_px4_vtol_1.18.0-alpha_*` | `command_survey_px4_rover_1.18.0-alpha_*` | `command_survey_mock_mock_*` |

**Note on UNKNOWN (ArduPlane FW/QP ≈ 127–128):** ArduPlane does not respond to most commands it considers irrelevant to its vehicle type.
A non-response is not the same as UNSUPPORTED — the stack may be executing the command silently.
ArduCopter explicitly ACKs most commands (only 1 UNKNOWN), giving a more useful matrix.
PX4 has 25–27 UNKNOWN (vehicle-type-dependent) — commands that are recognised but not via COMMAND_INT (likely COMMAND_LONG-only paths).

**Note on PX4 vehicle-type differences:** PX4 FW and VTOL report more SUPPORTED than MC because `DO_FIGURE_EIGHT` (35) — which MC gives no ACK — is SUPPORTED on FW and VTOL.
VTOL additionally supports `DO_VTOL_TRANSITION` (3000).
PX4 Rover does not support `DO_AUTOTUNE_ENABLE` (212) that MC supports (no flight-controller PID to tune).

**Note on ArduRover (SUPPORTED=49):** ArduRover supports a broader set of DO_ commands than ArduPlane (49 vs 12) because it has camera, relay, servo, and mission-management handlers.
However it does not support aerial commands (NAV_TAKEOFF, NAV_LAND, NAV_LOITER, etc.).
PX4 Rover, by contrast, returns ACCEPTED for NAV_TAKEOFF — PX4's rover controller does not gate commands based on vehicle type.

### Key command support by stack

**Key:** ✓ = SUPPORTED  ✗ = UNSUPPORTED  ? = UNKNOWN (no ACK within timeout)

These tables cover a representative selection of commands; the full 168-command survey results are in `logs/command_survey_*.log`.
See [Generating the tables](#generating-the-tables) for how to regenerate from survey logs.

#### Commands supported cross-platform (PX4 + ArduPilot)

Commands supported by PX4 MC **and** at least one ArduPilot vehicle type.

| CMD | ID | ArduCopter MC | ArduPlane FW | ArduPlane QP | ArduRover | PX4 MC | PX4 FW | PX4 VTOL | PX4 Rover | Mock |
|-----|----|---------------|--------------|--------------|-----------|--------|--------|----------|-----------|------|
| NAV_RETURN_TO_LAUNCH | 20 | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| NAV_LAND | 21 | ✓ | ✗ | ✗ | ✗ | ✓ | ✓ | ✓ | ✓ | ✓ |
| NAV_TAKEOFF | 22 | ✓ | ✓ | ✓ | ✗ | ✓ | ✓ | ✓ | ✓ | ✓ |
| NAV_VTOL_TAKEOFF | 84 | ✓¹ | ✗ | ✗ | ✗ | ✓ | ✓ | ✓ | ✓ | ✓ |
| DO_SET_MODE | 176 | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| DO_CHANGE_SPEED | 178 | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| DO_SET_HOME | 179 | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| DO_FLIGHTTERMINATION | 185 | ✓ | ✓ | ✓ | ✗ | ✓ | ✓ | ✓ | ✓ | ✓ |
| MISSION_START | 300 | ✓ | ? | ? | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| COMPONENT_ARM_DISARM | 400 | ✓ | ? | ? | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| REQUEST_MESSAGE | 512 | ✓ | ? | ? | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |

¹ ArduCopter maps `NAV_VTOL_TAKEOFF` to its standard takeoff, returning ACCEPTED even on a non-VTOL airframe.
ArduPlane QP (a true VTOL-capable vehicle) explicitly rejects it via COMMAND_INT.
NAV_LAND is only supported on aerial ArduPilot vehicles (ArduCopter); ArduPlane FW/QP and ArduRover return UNSUPPORTED.
All PX4 vehicle types (including rover) return ACCEPTED for NAV_TAKEOFF and NAV_VTOL_TAKEOFF — PX4 does not gate these on vehicle type.

ArduPlane FW/QP report ? for MISSION_START, ARM_DISARM, and REQUEST_MESSAGE — ArduPlane does not ACK commands it considers irrelevant to its vehicle type; they may execute silently.

#### Commands unsupported by PX4 and most ArduPilot stacks

Commands where PX4 returns UNSUPPORTED **and** the majority of ArduPilot vehicles also return UNSUPPORTED.

| CMD | ID | ArduCopter MC | ArduPlane FW | ArduPlane QP | ArduRover | PX4 MC | PX4 FW | PX4 VTOL | PX4 Rover | Mock |
|-----|----|---------------|--------------|--------------|-----------|--------|--------|----------|-----------|------|
| NAV_VTOL_LAND | 85 | ✓¹ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | ✓ |
| CONDITION_YAW | 115 | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | ✓ |

¹ ArduCopter maps `NAV_VTOL_LAND` to its standard land handler, returning ACCEPTED.
All other real stacks (including all PX4 vehicle types) return UNSUPPORTED.

#### Commands with partial or unclear cross-platform support

Commands not in either table above — supported by some ArduPilot vehicles but not PX4, or where ArduPlane FW/QP gave no ACK, or varying across PX4 vehicle types.

| CMD | ID | ArduCopter MC | ArduPlane FW | ArduPlane QP | ArduRover | PX4 MC | PX4 FW | PX4 VTOL | PX4 Rover | Mock |
|-----|----|---------------|--------------|--------------|-----------|--------|--------|----------|-----------|------|
| NAV_LOITER_UNLIM | 17 | ✓ | ✓ | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ | ✓ |
| DO_FIGURE_EIGHT | 35 | ✗ | ✗ | ✗ | ✗ | ? | ✓ | ✓ | ? | ✓ |
| DO_REPOSITION | 192 | ✓ | ? | ? | ✓ | ✗ | ✗ | ✗ | ✗ | ✓ |
| DO_FENCE_ENABLE | 207 | ✓ | ? | ? | ✓ | ✗ | ✗ | ✗ | ✗ | ✓ |
| DO_SET_MISSION_CURRENT | 224 | ✓ | ? | ? | ✓ | ✗³ | ✗ | ✗ | ✗ | ✓ |
| DO_VTOL_TRANSITION | 3000 | ✗ | ✗ | ✗ | ✗ | ? | ? | ✓ | ? | ✓ |

`DO_FIGURE_EIGHT` (35) is SUPPORTED on PX4 FW and VTOL but gives no ACK on MC and Rover (UNKNOWN).
`DO_VTOL_TRANSITION` (3000) is SUPPORTED only on PX4 VTOL; all other PX4 vehicle types give no ACK.
NAV_LOITER_UNLIM is supported by ArduCopter and ArduPlane FW/QP but not PX4 or ArduRover.
DO_REPOSITION, DO_FENCE_ENABLE, and DO_SET_MISSION_CURRENT are confirmed by ArduCopter and ArduRover; ArduPlane FW/QP gave no ACK (?); all PX4 vehicle types return UNSUPPORTED.

³ **Stale** — this row was tested 2026-05-27 against PX4 1.18.0-alpha. Live testing against PX4 1.18.0-beta shows PX4 MC actively processes DO_SET_MISSION_CURRENT (never UNSUPPORTED) and matches the full authoritative behaviour matrix exactly — see [`do_set_mission_current/README.md`](do_set_mission_current/README.md) for the complete deep-dive. The PX4 FW/VTOL/Rover ✗ entries for this row have not been re-verified and should not be trusted without a fresh survey run.

Tested: 2026-05-27.

### Generating the tables

The survey writes a per-command log (`logs/command_survey_<stack>_<timestamp>.log`) after each run.
`scripts/generate_command_tables.py` reads all survey logs in `logs/` and outputs updated markdown for the three tables above:

```bash
python scripts/generate_command_tables.py
```

Run the survey against each stack first, then run the script to regenerate the table content.

---

## Protocol conformance tests (`test_protocol.py`)

Tests the command protocol mechanics against `MAV_CMD_NAV_TAKEOFF (22)`.
Tests 1–5 run in both mock and standalone mode.
Tests 6–8 require the mock (drop/injection not possible on real stacks) — SKIP in standalone.

| Test | ArduCopter MC | ArduPlane FW | ArduPlane QP | ArduRover | PX4 MC | PX4 FW | PX4 VTOL | PX4 Rover | Mock |
|------|---------------|--------------|--------------|-----------|--------|--------|----------|-----------|------|
| test_command_int_ack_received | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS |
| test_command_int_ack_echoes_command_id | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS |
| test_command_int_ack_result_accepted | PASS | PASS | PASS | **FAIL**² | PASS | PASS | PASS | PASS | PASS |
| test_command_long_ack_received | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS |
| test_unsupported_command_returns_result | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS |
| test_command_long_confirmation_increments_on_retry | SKIP | SKIP | SKIP | SKIP | SKIP | SKIP | SKIP | SKIP | PASS |
| test_retry_recovers_from_dropped_ack | SKIP | SKIP | SKIP | SKIP | SKIP | SKIP | SKIP | SKIP | PASS |
| test_in_progress_then_accepted | SKIP | SKIP | SKIP | SKIP | SKIP | SKIP | SKIP | SKIP | PASS |

² ArduRover does not support `NAV_TAKEOFF` (it is a ground vehicle).
The test expects `result != UNSUPPORTED`, but rover returns `MAV_RESULT_UNSUPPORTED`.
PX4 Rover returns ACCEPTED for NAV_TAKEOFF — PX4 does not restrict commands by vehicle type.

Tested: 2026-05-27.
Logs: `logs/command_survey_ardupilot_copter_4.8.0-dev_20260526_211627.log`, `logs/command_survey_ardupilot_fixed_wing_4.8.0-dev_20260526_212118.log`, `logs/command_survey_ardupilot_quadplane_4.8.0-dev_20260526_212605.log`, `logs/command_survey_ardupilot_rover_4.8.0-dev_20260526_212652.log`, `logs/command_survey_px4_quadcopter_1.18.0-alpha_20260527_075001.log`, `logs/command_survey_px4_fixed_wing_1.18.0-alpha_20260527_075139.log`, `logs/command_survey_px4_vtol_1.18.0-alpha_20260527_075317.log`, `logs/command_survey_px4_rover_1.18.0-alpha_20260527_075459.log`.

---

See [`nav_takeoff/README.md`](nav_takeoff/README.md) for `MAV_CMD_NAV_TAKEOFF` COMMAND_INT test results and a comparison with the mission protocol path.

See [`nav_land/README.md`](nav_land/README.md) for `MAV_CMD_NAV_LAND` COMMAND_INT test results, including notes on why several parameters (notably the "ground level" altitude semantics and precision-landing flags) can only be tested observationally at the ACK level — and the Tier 2 (flight) results that resolve those execution-semantics questions empirically (headline finding: the commanded lat/lon/alt is **not** the touchdown point on PX4 MC/VTOL or ArduCopter MC — the vehicle descends in place from wherever it already is).

See [`do_set_mission_current/README.md`](do_set_mission_current/README.md) for `MAV_CMD_DO_SET_MISSION_CURRENT` COMMAND_LONG test results against an authoritative, maintainer-provided behaviour matrix (Mock, PX4 MC — 18/18 Tier 1 tests passing with zero deviation, ArduCopter MC blocked by a SITL environment issue) — plus the Tier 2 `test_flight.py` jump-counter reset test, which confirms `param2=1` genuinely resets a `DO_JUMP` repeat counter on PX4 MC, including mid-flight. A design outline (not yet implemented) remains for verifying that the command actually changes the current mission item via `MISSION_CURRENT`.
