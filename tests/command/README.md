# Command Protocol Tests

Tests for the MAVLink **command protocol** — `COMMAND_INT`/`COMMAND_LONG` sent directly to the vehicle, acknowledged via `COMMAND_ACK`. Distinct from the **mission protocol** (`MISSION_ITEM_INT` upload) in `tests/mission/` — the same MAV_CMD can behave differently in each; see `CLAUDE.md § Command vs mission protocol differences`.

## Test structure

| File | Purpose |
|------|---------|
| `test_survey.py` | Probe all 168 MAV_CMD; write support matrix to `logs/` |
| `test_ack_uniqueness.py` | Assert exactly one terminal COMMAND_ACK per command (catches double-ACK bugs) |
| `test_protocol.py` | Protocol mechanics: ACK receipt/echo/result, retry, IN_PROGRESS |
| `nav_takeoff/test_command.py` | NAV_TAKEOFF (22) via COMMAND_INT — ACK tests |
| `nav_vtol_takeoff/test_command.py` | NAV_VTOL_TAKEOFF (84) via COMMAND_INT — ACK tests; verifies PX4 commit `aad2f0f3` (param1/param2 mask fix). See `nav_vtol_takeoff/README.md` |
| `nav_land/test_command.py` | NAV_LAND (21) via COMMAND_INT — ACK tests |
| `do_set_mission_current/test_command.py` | DO_SET_MISSION_CURRENT (224) via COMMAND_LONG — ACK tests |
| `do_set_mission_current/test_flight.py` | DO_SET_MISSION_CURRENT — Tier 2: does param2 reset a `DO_JUMP` counter / make a completed mission restartable? See `do_set_mission_current/README.md` |
| `external_wind_estimate/test_command.py` | EXTERNAL_WIND_ESTIMATE (43004, development.xml) — mandatory common tests + per-parameter tests, both message types. See `external_wind_estimate/README.md` |
| `external_wind_estimate/test_flight.py` | EXTERNAL_WIND_ESTIMATE — Tier 2 ground-vs-air `WIND_COV` observation |

## Running

```bash
pytest tests/command/ -v --log-cli-level=INFO   # paired (mock)

pytest tests/command/ \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
    --ardupilot-sitl=~/ardu_sitl/arducopter \
    --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 \
    --vehicle-type=copter --autopilot=ardupilot -v --log-cli-level=INFO

pytest tests/command/ \
    --drone-address=udp://:14540 --connection-timeout=60 \
    --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \
    --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO
```

Other stacks/vehicle types: swap `--ardupilot-sitl`/`--px4-model` and `--vehicle-type`/`--autopilot` — see root `CLAUDE.md` § Running modes for the full matrix (ArduPlane FW/QP, ArduRover, PX4 FW/VTOL/Rover).

---

## Command support survey

Probes all 168 `MAV_CMD` twice — once via COMMAND_INT and once via COMMAND_LONG — and combines the two results (reported once when they agree; if one message type gets a real ACK and the other UNSUPPORTED/no ACK, the command is SUPPORTED via that type only, and the row notes the other type should NACK with `COMMAND_INT_ONLY`/`COMMAND_LONG_ONLY`). The table also has a raw `MAV_RESULT` column. Classified SUPPORTED (any non-UNSUPPORTED result) / UNSUPPORTED (`MAV_RESULT_UNSUPPORTED`) / UNKNOWN (no ACK — a spec violation, not necessarily unsupported). Always passes; observational. Written to `logs/command_survey_<stack>_<timestamp>.log`.

### Survey summary

| Metric | ArduCopter MC | ArduPlane FW | ArduPlane QP | ArduRover | PX4 MC | PX4 FW | PX4 VTOL | PX4 Rover | Mock |
|--------|---------------|--------------|--------------|-----------|--------|--------|----------|-----------|------|
| SUPPORTED | 61 | 12 | 12 | 49 | 36 | 37 | 38 | 35 | 168 |
| UNSUPPORTED | 106 | 28 | 29 | 118 | 105 | 105 | 105 | 106 | 0 |
| UNKNOWN | 1 | 128 | 127 | 1 | 27 | 26 | 25 | 27 | 0 |

Tested 2026-05-27. ArduPlane's high UNKNOWN count (~127) means it silently ignores most commands outside its vehicle type rather than ACKing UNSUPPORTED — a non-response isn't proof of non-support. PX4's 25–27 UNKNOWN are mostly camera/gimbal commands with no COMMAND_INT ACK path. ArduRover's DO_ coverage (49) is broader than ArduPlane's (12) — camera/relay/servo/mission-management handlers — but it lacks aerial commands (NAV_TAKEOFF, NAV_LAND, NAV_LOITER_UNLIM); PX4 Rover, by contrast, ACCEPTs NAV_TAKEOFF since PX4 doesn't gate commands by vehicle type.

### Key command support by stack

✓ SUPPORTED · ✗ UNSUPPORTED · ? UNKNOWN (no ACK). A representative subset — full 168-command results are in `logs/command_survey_*.log`; regenerate these tables with `python scripts/generate_command_tables.py` after running the survey against each stack.

#### Supported cross-platform (PX4 + at least one ArduPilot vehicle)

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

¹ ArduCopter maps NAV_VTOL_TAKEOFF to its standard takeoff (ACCEPTED on any airframe); ArduPlane QP (true VTOL) rejects it. All PX4 vehicle types ACCEPT both NAV_TAKEOFF and NAV_VTOL_TAKEOFF regardless of vehicle type. ArduPlane FW/QP's `?` on MISSION_START/ARM_DISARM/REQUEST_MESSAGE reflects the silent-ignore pattern noted above.

#### Unsupported by PX4 and most ArduPilot stacks

| CMD | ID | ArduCopter MC | ArduPlane FW | ArduPlane QP | ArduRover | PX4 MC | PX4 FW | PX4 VTOL | PX4 Rover | Mock |
|-----|----|---------------|--------------|--------------|-----------|--------|--------|----------|-----------|------|
| NAV_VTOL_LAND | 85 | ✓¹ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | ✓ |
| CONDITION_YAW | 115 | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | ✓ |

¹ ArduCopter maps NAV_VTOL_LAND to its standard land handler; every other real stack, PX4 included, returns UNSUPPORTED.

#### Partial or unclear cross-platform support

| CMD | ID | ArduCopter MC | ArduPlane FW | ArduPlane QP | ArduRover | PX4 MC | PX4 FW | PX4 VTOL | PX4 Rover | Mock |
|-----|----|---------------|--------------|--------------|-----------|--------|--------|----------|-----------|------|
| NAV_LOITER_UNLIM | 17 | ✓ | ✓ | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ | ✓ |
| DO_FIGURE_EIGHT | 35 | ✗ | ✗ | ✗ | ✗ | ? | ✓ | ✓ | ? | ✓ |
| DO_REPOSITION | 192 | ✓ | ? | ? | ✓ | ✗ | ✗ | ✗ | ✗ | ✓ |
| DO_FENCE_ENABLE | 207 | ✓ | ? | ? | ✓ | ✗ | ✗ | ✗ | ✗ | ✓ |
| DO_SET_MISSION_CURRENT | 224 | ✓ | ? | ? | ✓ | ✗³ | ✗ | ✗ | ✗ | ✓ |
| DO_VTOL_TRANSITION | 3000 | ✗ | ✗ | ✗ | ✗ | ? | ? | ✓ | ? | ✓ |

DO_FIGURE_EIGHT is SUPPORTED only on PX4 FW/VTOL; DO_VTOL_TRANSITION only on PX4 VTOL. DO_REPOSITION/DO_FENCE_ENABLE/DO_SET_MISSION_CURRENT: confirmed on ArduCopter/ArduRover, no ACK from ArduPlane FW/QP, UNSUPPORTED on all PX4.

³ **Stale**: tested 2026-05-27 against PX4 1.18.0-alpha. Live testing against 1.18.0-beta shows PX4 MC actively processes DO_SET_MISSION_CURRENT (never UNSUPPORTED) and matches the full authoritative behaviour matrix — see `do_set_mission_current/README.md`. The PX4 FW/VTOL/Rover ✗ entries here are unverified against the newer build.

---

## Protocol conformance (`test_protocol.py`)

Against `MAV_CMD_NAV_TAKEOFF`. Tests 1–5 run in both mock and standalone; tests 6–8 need the mock (drop/injection isn't possible on a real stack) and SKIP in standalone.

| Test | ArduCopter MC | ArduPlane FW/QP | ArduRover | PX4 (all) | Mock |
|------|---------------|------------------|-----------|------------|------|
| test_command_int_ack_received | PASS | PASS | PASS | PASS | PASS |
| test_command_int_ack_echoes_command_id | PASS | PASS | PASS | PASS | PASS |
| test_command_int_ack_result_accepted | PASS | PASS | **FAIL**² | PASS | PASS |
| test_command_long_ack_received | PASS | PASS | PASS | PASS | PASS |
| test_unsupported_command_returns_result | PASS | PASS | PASS | PASS | PASS |
| test_command_long_confirmation_increments_on_retry / test_retry_recovers_from_dropped_ack / test_in_progress_then_accepted | SKIP | SKIP | SKIP | SKIP | PASS |

² ArduRover doesn't support NAV_TAKEOFF (ground vehicle) — test expects `result != UNSUPPORTED`. PX4 Rover, by contrast, ACCEPTs it.

Tested 2026-05-27.

---

Per-command detail: [`nav_takeoff/README.md`](nav_takeoff/README.md) (COMMAND_INT vs mission-protocol comparison) · [`nav_vtol_takeoff/README.md`](nav_vtol_takeoff/README.md) (PX4's real param1/param2 semantics vs. the XML; verifies the `aad2f0f3` mask fix) · [`nav_land/README.md`](nav_land/README.md) (Tier 2: commanded lat/lon/alt is *not* the touchdown point on PX4 MC/VTOL or ArduCopter MC) · [`do_set_mission_current/README.md`](do_set_mission_current/README.md) (authoritative behaviour matrix, 18/18 on PX4 MC, `DO_JUMP` counter reset confirmed in flight).
