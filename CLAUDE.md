# CLAUDE.md — Implementation notes for Claude

This file records protocol behaviour, design decisions, and change-tracking notes for use in future Claude sessions.

Detailed notes for each test subtree live in their own CLAUDE.md files:
- `tests/mission/CLAUDE.md` — mission protocol, frame type tables, MAV_CMD methodology
- `tests/mission/nav_takeoff/CLAUDE.md` — NAV_TAKEOFF storage test results (all stacks)
- `tests/mission/do_reposition/CLAUDE.md` — DO_REPOSITION rejected as a mission item everywhere (UNSUPPORTED, spec-aligned); baseline-probe NaN pitfall
- `tests/command/CLAUDE.md` — command protocol, MAV_RESULT values, per-stack command notes

## Conventions

- **Test logs**: always write to the `logs/` directory at the repo root (e.g. `logs/test_arducopter_20260524.log`).
- **README synchronisation**: the root `README.md` and `tests/mission/README.md` must both be kept up to date whenever autopilot behaviour changes, new tests are added, or test result counts change.
  The subfolder README carries the detailed per-frame comparison tables; the root README carries the high-level summary.
  Update both together.
- **Test timeouts**: every test must be protected against hanging indefinitely.
  Use `asyncio.wait_for()` (or `asyncio.timeout()`) inside async helpers to guard individual MAVLink round-trips.
  The global `pytest-timeout` default (120 s in `pytest.ini`) covers the whole test function as a safety net.
  Override with `@pytest.mark.timeout(N)` (or a module-level `pytestmark`) only for tests that legitimately need more:
  - Protocol tests (mission upload/download, COMMAND_ACK): no override needed — completes well within 120 s.
  - Command protocol class-based tests (`TestCommandProtocol`, `TestNavTakeoffCommand`): `@pytest.mark.timeout(300)`.
  - Flight tests (`test_flight.py`): `pytestmark = pytest.mark.timeout(360)`.
  - Command survey (`test_survey.py`): `@pytest.mark.timeout(900)`.
    When adding a new test that could legitimately exceed 120 s, add the override and document the reason.
- **MAVLink spec discrepancies**: When observed flight-stack behaviour contradicts the official MAVLink documentation (mavlink.io), note the discrepancy explicitly in:
  1. The test log output (`log.warning` with "DOC DISCREPANCY:" prefix).
  2. The relevant README results table with a footnote.
  3. CLAUDE.md under the affected autopilot's behaviour section.
     Open an issue at https://github.com/mavlink/mavlink/issues if the spec is demonstrably wrong or ambiguous.
     Do not assume the stack is wrong — it may be that the spec lags the implementation.

## Project purpose

MAVLink protocol conformance tests, using MAVSDK-Python as the MAVLink transport.
The initial focus is the mission protocol (common.xml).

## Architecture

```
conftest.py              CLI options: --drone-address, --connection-timeout, --mavlink-definitions-dir, --vehicle-type, --autopilot,
                           --ardupilot-sitl, --ardupilot-model, --px4-sitl, --px4-model, --home-lat/lon/alt
tests/conftest.py        GCS (gcs_system) and drone (drone_system) fixtures; autopilot probe (_autopilot_header session fixture)
tests/mock_flight_stack.py  MockFlightStack — configurable MAVLink drone simulator
mavlink/                 Git submodule: https://github.com/mavlink/mavlink (authoritative XML)
tests/mission/           Mission protocol tests — see tests/mission/CLAUDE.md
  conftest.py            load_plan(), items_match(), clear_all_mission_types() helpers
  test_mission_client.py GCS-side tests using mission_raw plugin
  test_mission_server.py Drone-side tests using mission_raw_server plugin
  test_frame_types.py    MAV_FRAME support matrix (65 tests, stack-agnostic)
  nav_takeoff/           NAV_TAKEOFF mission-protocol tests — see nav_takeoff/CLAUDE.md
  plans/                 JSON plan files (MISSION_ITEM_INT fields)
tests/command/           Command protocol tests — see tests/command/CLAUDE.md
  conftest.py            send/receive helpers (probe_command_int/long), class-scoped fixtures
  test_survey.py         Probe all 168 MAV_CMD from common.xml; write support matrix to logs/
  test_protocol.py       Command protocol mechanics (ACK, retry, confirmation, in-progress)
  takeoff/               NAV_TAKEOFF via COMMAND_INT (test_command.py, README.md)
```

### MockFlightStack

`tests/mock_flight_stack.py` implements the full MAVLink mission protocol on the drone side via `mavlink_direct`, so all client tests can run against a local loopback mock without a real autopilot.

**Protocol handlers** (all via `drone_system.mavlink_direct`):

| Handler | Receives | Sends |
|---------|---------|-------|
| Upload | `MISSION_COUNT` → `MISSION_ITEM_INT` × n | `MISSION_REQUEST_INT` × n → `MISSION_ACK(SUCCESS)` |
| Download | `MISSION_REQUEST_LIST` | `MISSION_COUNT` → `MISSION_ITEM_INT` × n |
| Clear | `MISSION_CLEAR_ALL` | `MISSION_ACK(SUCCESS)` |
| Capability | `COMMAND_LONG(cmd=512, p1=148)` | `AUTOPILOT_VERSION(capabilities=…)` |
| Command INT | `COMMAND_INT` | `COMMAND_ACK(result, progress)` |
| Command LONG | `COMMAND_LONG` (non-capability) | `COMMAND_ACK(result, progress)` |

**Default behaviour**: accept all frames, store items exactly as received, serve unchanged on download, report `MAV_PROTOCOL_CAPABILITY_MISSION_INT` (bit 2 = 4).
Returns `MAV_RESULT_ACCEPTED (0)` for all COMMAND_INT and COMMAND_LONG by default, except that COMMAND_INT with lat/lon (x/y) values outside the valid range (±90°/±180° × 1e7) returns `MAV_RESULT_DENIED (2)`.
The INT32_MAX sentinel is always valid.

**Configurable parameters** (constructor kwargs):
- `capability_bits` — capabilities bitmask in AUTOPILOT_VERSION response
- `item_request_delay_s` — per-item delay before MISSION_REQUEST_INT (upload)
- `item_response_delay_s` — per-item delay before MISSION_ITEM_INT (download)
- `drop_responses` — `{msg_name: N}` to silently drop first N of that outgoing mission message
- `rejected_frames` — frame values to NACK with UNSUPPORTED_FRAME on upload
- `command_results` — `{cmd_id: MAV_RESULT}` to configure per-command ACK results
- `drop_command_acks` — `{cmd_id: N}` to drop first N COMMAND_ACKs for that command
- `command_in_progress` — `{cmd_id: [p0, p1, ...]}` to emit IN_PROGRESS ACKs before final ACK

**`received_commands`**: list of all received COMMAND_INT and non-capability COMMAND_LONG messages, each as a dict with fields `type`, `command`, `frame` (COMMAND_INT only), `confirmation` (COMMAND_LONG only), `param1`–`param4`, `x`, `y`, `z`.

**Capability response interaction**: the drone mavsdk_server binary also responds to AUTOPILOT_VERSION requests with its own bits (0x2000 = MAV_PROTOCOL_CAPABILITY_MAVLINK2).
`_get_autopilot_capabilities()` OR-combines all responses within a 0.3 s window.

All tests are async (pytest-asyncio, asyncio_mode=auto).

### Autopilot probe (`tests/conftest.py`)

Session-scoped `_autopilot_header` autouse fixture: calls `system.info.get_version()` (firmware version, git hash — reliable) and `system.info.get_product()` (vendor name, best-effort).
ArduPilot stores git hash as ASCII bytes in `flight_custom_version`; the probe ASCII-decodes these.
`suggest_log_filename()` derives log prefix from test path (e.g. `tests/mission/test_frame_types.py` → `mission_frame_types`).

**HEARTBEAT limitation**: MAVSDK intercepts HEARTBEAT internally — vehicle type cannot be auto-detected from HEARTBEAT.
Use `--vehicle-type` CLI option for correct log naming.

**MAVLink enum loading**: `_load_mavlink_enum()` parses the bundled XML submodule at import time.
Falls back to a hardcoded dict with `log.warning` when the submodule is absent.

## MAVSDK plugin choices

- `mission_raw` — GCS tests; maps 1:1 to MISSION_ITEM_INT fields; supports all three mission types.
- `mission_raw_server` — drone/server tests; mirrors MISSION_ITEM_INT on the server side.
- `mavlink_direct` — send/receive raw MAVLink messages.

The high-level `mission` plugin is NOT used — it abstracts away mission_type, geofence, and rally points, making protocol-level testing impossible.

## Running modes

SITL processes are managed automatically via `--ardupilot-sitl` / `--px4-sitl`.
`--ardupilot-model` overrides auto-detected model (default: `+` copter, `plane`, `rover`).
`--px4-model` sets PX4_SIM_MODEL (default: `sihsim_quadx`).
Current test result counts are in `README.md`.

| Mode | Command |
|------|---------|
| Paired (mock) | `pytest tests/` |
| PX4 multicopter | `pytest tests/ --drone-address=udp://:14540 --vehicle-type=quadcopter --autopilot=px4 --px4-sitl=~/github/PX4/PX4-Autopilot` |
| PX4 MC (flight only) | `pytest tests/command/nav_takeoff/test_flight.py --drone-address=udp://:14540 --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx --vehicle-type=quadcopter --autopilot=px4` |
| PX4 fixed-wing | `pytest tests/mission/nav_takeoff/test_protocol.py --drone-address=udp://:14540 --vehicle-type=fixed_wing --autopilot=px4 --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_airplane` |
| PX4 VTOL | `pytest tests/mission/nav_takeoff/test_protocol.py --drone-address=udp://:14540 --vehicle-type=vtol --autopilot=px4 --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_standard_vtol` |
| ArduCopter | `pytest tests/ --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 --ardupilot-sitl=~/ardu_sitl/arducopter --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 --vehicle-type=copter --autopilot=ardupilot` |
| ArduPlane fixed-wing | `pytest tests/ --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 --ardupilot-sitl=~/ardu_sitl/arduplane --vehicle-type=fixed_wing --autopilot=ardupilot` |
| ArduPlane QuadPlane | `pytest tests/ --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 --ardupilot-sitl=~/ardu_sitl/arduplane --ardupilot-model=quadplane --vehicle-type=quadplane --autopilot=ardupilot` |
| ArduRover | `pytest tests/command/ tests/mission/test_frame_types.py --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 --ardupilot-sitl=~/ardu_sitl/ardurover --ardupilot-model=rover --vehicle-type=rover --autopilot=ardupilot` |

**`--vehicle-type` and `--autopilot`** are used in log file naming only.
Always pass them explicitly for clean log filenames.

**IMPORTANT**: Do not run paired-mode tests while PX4 SITL is running — PX4 uses sysid=1 (same as the mock drone) and causes `_wait_for_connection` to hang indefinitely on port 14560.
Always `kill <px4_pid>` first.

**Standalone PX4 peer-caching**: Always start a fresh PX4 instance (kill and restart) before standalone tests — a cached peer on port 14560 causes `test_gcs_sends_mission_item_int` to hang.

Paired mode uses UDP loopback on port 14560 (deliberately not 14540, to avoid PX4's default SDK port):
- Drone: `udpout://127.0.0.1:14560` — GCS: `udpin://0.0.0.0:14560`

### ArduCopter SITL setup

See `SETUP.md` for download and startup commands.

### ArduCopter connection reliability pitfalls

**1. CLOSE-WAIT accumulation** — ArduCopter does not close its TCP socket on abrupt disconnect.
After killing mavsdk_server, port 5760 appears open but ArduCopter is in CLOSE-WAIT and won't heartbeat on new connections.
**Fix**: restart ArduCopter SITL.
Check with `ss -tlnp | grep 5760`.
Do **not** use `nc -z 127.0.0.1 5760` — that creates another CLOSE-WAIT.

**2. Missing / wrong parm file** — ArduCopter silently accepts the TCP connection then panics while loading defaults.
Correct path: `~/github/ArduPilot/ardupilot/Tools/autotest/default_params/copter.parm`.

## Design decisions

1. **JSON plan files, not QGC format** — maps directly to MISSION_ITEM_INT fields; QGC adds abstraction layers that hide raw values.

2. **mission_raw over mission** — `mission` only supports flight plans and abstracts away mission_type, geofence, and rally points.

3. **Function-scoped System objects, session-scoped processes** — `gcs_mavsdk_server` / `drone_mavsdk_server` are session-scoped; `gcs_system` / `drone_system` are function-scoped to avoid gRPC channel / event loop conflicts across tests.

   **Exception — frame-type tests**: 65 tests would exhaust gRPC resources (~35 open channels).
   Uses `@pytest_asyncio.fixture(scope="class", loop_scope="class")` + `@pytest.mark.asyncio(loop_scope="class")` to share one `System` per class (4 total).
   Teardown is in a `try/finally` block inside each test to avoid event-loop mismatch.

4. **TRANSFER_TIMEOUT_S = 30 s** — Conservative upper bound for a 4-item plan over loopback including gRPC overhead.

4a. **gRPC stream cancellation** — `asyncio.wait_for` on a gRPC `async for` loop blocks indefinitely after timeout (the gRPC stream doesn't yield to asyncio cancellation).
**Fix**: fire-and-forget `asyncio.create_task` for the subscription; await only a plain `asyncio.Event` (set by the task on success).
Cancel the task without awaiting it.
Implemented in `_wait_for_connection` and any helper that cancels a gRPC subscription task.

4b. **PX4 SIH SITL readiness** — Two pitfalls: (1) PX4 writes `"INFO  [mavlink]"` with two spaces — searching one-space form silently misses all iterations.
(2) The `pxh>` prompt refresh loop grows the log file at several GB/min after startup — read at most 64 KB.

5. **asyncio.timeout() over asyncio.wait_for()** — Python 3.11+ timeout context manager used where available.
   Replace with `asyncio.wait_for(..., timeout=t)` for 3.10 compatibility.

6. **GCS paired identity: sysid=255, compid=1** — MAVSDK only fires "System discovered" when it sees compid=1.
   Using compid=190 (MAV_COMP_ID_MISSIONPLANNER) means the drone's gRPC port (50052) never opens and `System.connect()` hangs indefinitely in `channel_ready()`.

7. **`collect_incoming_mission()` workaround** — MAVSDK-Python v3.15.x bug: `mission_raw_server.incoming_mission()` discards the plan when result=SUCCESS.
   Use `collect_incoming_mission()` in `tests/mission/conftest.py` (calls raw gRPC stub `SubscribeIncomingMission` directly) until fixed upstream.

## Future work / known gaps

1. **Error-condition tests** — Deliberately provoke non-zero MISSION_ACK error codes and verify the correct `MissionRawResult.Result` enum value is raised.
   `TestErrorHandling.test_int_messages_not_supported_is_raised` is skipped (cannot force that condition from the suite).

2. **Geofence roundtrip frame comparison** — PX4 converts geofence items from frame=0 to frame=5.
   The roundtrip test xfails; promote once `items_match()` supports frame-aware comparison.

3. **Frame-aware roundtrip comparison** — Extend `items_match()` to accept a coordinate-frame translation function.

4. **Heartbeat protocol tests** — Add `tests/heartbeat/` covering the MAVLink heartbeat service (component type, autopilot type, base mode, etc.).

5. **PX4 geofence relative-altitude frame z conversion** — `TestGeofenceFrames` fails for frames 3 and 6: PX4 accepts both but stores as frame=5 without converting z.
   Needs investigation: (a) is z conversion required by spec for geofence?
   (b) if so, this is a PX4 bug to report upstream.

## Change log

See `CHANGELOG.md` for full development history.
