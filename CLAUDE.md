# CLAUDE.md — Implementation notes for Claude

This file records protocol behaviour, design decisions, and change-tracking
notes for use in future Claude sessions.

Detailed notes for each test subtree live in their own CLAUDE.md files:
- `tests/mission/CLAUDE.md` — mission protocol, frame type tables, MAV_CMD methodology
- `tests/mission/nav_takeoff/CLAUDE.md` — NAV_TAKEOFF storage test results (all stacks)
- `tests/command/CLAUDE.md` — command protocol, MAV_RESULT values, per-stack command notes

## Conventions

- **Test logs**: always write to the `logs/` directory at the repo root (e.g. `logs/test_arducopter_20260524.log`).
- **README synchronisation**: the root `README.md` and `tests/mission/README.md` must both be kept up to date whenever autopilot behaviour changes, new tests are added, or test result counts change.  The subfolder README carries the detailed per-frame comparison tables; the root README carries the high-level summary.  Update both together.
- **Test timeouts**: every test must be protected against hanging indefinitely. Use `asyncio.wait_for()` (or `asyncio.timeout()`) inside async helpers to guard individual MAVLink round-trips. The global `pytest-timeout` default (120 s in `pytest.ini`) covers the whole test function as a safety net.  Override with `@pytest.mark.timeout(N)` (or a module-level `pytestmark`) only for tests that legitimately need more:
  - Protocol tests (mission upload/download, COMMAND_ACK): no override needed — completes well within 120 s.
  - Command protocol class-based tests (`TestCommandProtocol`, `TestNavTakeoffCommand`): `@pytest.mark.timeout(300)` — class-scoped fixture setup (up to 60 s connection) + 8–9 tests with 5 s ACK timeout each.
  - Flight tests (`test_flight.py`): `pytestmark = pytest.mark.timeout(360)` — arming (60 s) + takeoff (90 s) + RTL/land (120 s) + margin.
  - Command survey (`test_survey.py`): `@pytest.mark.timeout(900)` — 168 commands × up to 5 s each.
  When adding a new test that could legitimately exceed 120 s, add the override and document the reason.  A test that hangs without timing out is a bug — investigate whether an `asyncio.wait_for()` guard is missing inside the test or a helper it calls.
- **MAVLink spec discrepancies**: When observed flight-stack behaviour contradicts the official MAVLink documentation (mavlink.io), note the discrepancy explicitly in:
  1. The test log output (`log.warning` with "DOC DISCREPANCY:" prefix).
  2. The relevant README results table with a footnote.
  3. CLAUDE.md under the affected autopilot's behaviour section.
  Open an issue at https://github.com/mavlink/mavlink/issues if the spec is demonstrably wrong or ambiguous.  Do not assume the stack is wrong — it may be that the spec lags the implementation.

## Project purpose

MAVLink protocol conformance tests, using MAVSDK-Python as the MAVLink
transport.  The initial focus is the mission protocol (common.xml).

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

`tests/mock_flight_stack.py` implements the full MAVLink mission protocol on the
drone side via `mavlink_direct`, so all client tests can run against a local
loopback mock without a real autopilot.

**Protocol handlers** (all via `drone_system.mavlink_direct`):

| Handler | Receives | Sends |
|---------|---------|-------|
| Upload | `MISSION_COUNT` → `MISSION_ITEM_INT` × n | `MISSION_REQUEST_INT` × n → `MISSION_ACK(SUCCESS)` |
| Download | `MISSION_REQUEST_LIST` | `MISSION_COUNT` → `MISSION_ITEM_INT` × n |
| Clear | `MISSION_CLEAR_ALL` | `MISSION_ACK(SUCCESS)` |
| Capability | `COMMAND_LONG(cmd=512, p1=148)` | `AUTOPILOT_VERSION(capabilities=…)` |
| Command INT | `COMMAND_INT` | `COMMAND_ACK(result, progress)` |
| Command LONG | `COMMAND_LONG` (non-capability) | `COMMAND_ACK(result, progress)` |

**Default behaviour**: accept all frames, store items exactly as received, serve
unchanged on download, report `MAV_PROTOCOL_CAPABILITY_MISSION_INT` (bit 2 = 4).
Returns `MAV_RESULT_ACCEPTED (0)` for all COMMAND_INT and COMMAND_LONG by default.

**Configurable parameters** (constructor kwargs):
- `capability_bits` — capabilities bitmask in AUTOPILOT_VERSION response
- `item_request_delay_s` — per-item delay before MISSION_REQUEST_INT (upload)
- `item_response_delay_s` — per-item delay before MISSION_ITEM_INT (download)
- `drop_responses` — `{msg_name: N}` to silently drop first N of that outgoing mission message
- `rejected_frames` — frame values to NACK with UNSUPPORTED_FRAME on upload
- `command_results` — `{cmd_id: MAV_RESULT}` to configure per-command ACK results
- `drop_command_acks` — `{cmd_id: N}` to drop first N COMMAND_ACKs for that command
- `command_in_progress` — `{cmd_id: [p0, p1, ...]}` to emit IN_PROGRESS ACKs before final ACK

**`received_commands`**: list of all received COMMAND_INT and non-capability COMMAND_LONG
messages, each as a dict with fields `type`, `command`, `frame` (COMMAND_INT only),
`confirmation` (COMMAND_LONG only), `param1`–`param4`, `x`, `y`, `z`.

**Capability response interaction**: the drone mavsdk_server binary also responds
to AUTOPILOT_VERSION requests with its own bits (0x2000 = MAV_PROTOCOL_CAPABILITY_MAVLINK2).
`_get_autopilot_capabilities()` in the test file OR-combines all responses within a
0.3 s window so both the server's and the mock's bits are reflected in the result.

All tests are async (pytest-asyncio, asyncio_mode=auto).

### Autopilot probe (`tests/conftest.py`)

The session-scoped `_autopilot_header` autouse fixture runs once per session and logs a
flight stack identification block to the test output and log file.  Functions:

- `_probe_autopilot_async(grpc_port, timeout_s)` — creates a `System`, waits for connection,
  calls `system.info.get_version()` (reliable: firmware version, git hash) and
  `system.info.get_product()` (best-effort: vendor name).  ArduPilot stores git hash as ASCII
  bytes in `flight_custom_version`; the probe ASCII-decodes these bytes before display.
- `_format_autopilot_header(info, drone_address)` — formats the identification block.
- `suggest_log_filename(info, config)` — derives a log filename: prefix from test path
  (e.g. `tests/mission/test_frame_types.py` → `mission_frame_types`), autopilot and vehicle
  type from probe result or CLI overrides, firmware version, and timestamp.
- `_derive_log_prefix(config)` — strips `::NodeId` from `config.args`, extracts `.py` path,
  removes `tests/` prefix and `test_` prefix, joins with `_`.

**HEARTBEAT limitation**: MAVSDK's internal server handles HEARTBEAT messages before they
reach `mavlink_direct.message("HEARTBEAT")`, so the vehicle type cannot be auto-detected
from HEARTBEAT.  Use `--vehicle-type` CLI option to set it explicitly for correct log naming.

**MAVLink enum loading**: `_MAV_AUTOPILOT`, `_MAV_TYPE`, and `_MAV_FIRMWARE_TYPE` dicts are
populated at import time by `_load_mavlink_enum()`, which parses the bundled XML submodule
(`mavlink/message_definitions/v1.0/common.xml` and its includes).  If the submodule is absent,
a hardcoded fallback is used with a `log.warning`.  The XML is authoritative — loaded tables
have all current enum entries (e.g. 21 `MAV_AUTOPILOT`, 50 `MAV_TYPE` entries) rather than
the subset previously hardcoded.

## MAVSDK plugin choices

- `mission_raw` — used for GCS tests; maps 1:1 to MISSION_ITEM_INT fields.
  Supports all three mission types (mission_type field) and upload/download of
  flight plans, geofences, and rally points.
- `mission_raw_server` — used for drone/server tests; mirrors the same
  MISSION_ITEM_INT structure on the server side.  Methods: `incoming_mission`,
  `current_item_changed`, `set_current_item_complete`, `clear_all`.
- `mavlink_direct` — used to send/receive raw MAVLink messages (e.g.
  COMMAND_LONG for requesting AUTOPILOT_VERSION, or message sniffing during
  deprecated-message tests).

The high-level `mission` plugin is NOT used.  It abstracts away mission_type,
geofence, and rally points, making protocol-level testing impossible.

## Running modes

SITL processes can be managed automatically using `--ardupilot-sitl` (ArduPilot) or
`--px4-sitl` (PX4).  When supplied, the test suite starts and stops the SITL automatically.
The `--ardupilot-model` option overrides the auto-detected model (default: `+` for copter,
`plane` for plane, `rover` for rover).  `--px4-model` sets PX4_SIM_MODEL (default: `sihsim_quadx`).

| Mode | Command | Result (2026-05-27) |
|------|---------|-----------|
| Paired (mock) | `pytest tests/` | 120 passed, 25 skipped |
| Standalone (PX4 multicopter) | `pytest tests/ --drone-address=udp://:14540 --vehicle-type=quadcopter --autopilot=px4 --px4-sitl=~/github/PX4/PX4-Autopilot` (sihsim_quadx) | 84 passed, 4 failed, 1 skipped, 3 xfailed |
| Standalone (PX4 MC — flight tests) | `pytest tests/command/takeoff/test_flight.py --drone-address=udp://:14540 --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx --vehicle-type=quadcopter --autopilot=px4` | 15 passed, 2 xpassed |
| Standalone (PX4 fixed-wing) | `pytest tests/mission/nav_takeoff/test_protocol.py --drone-address=udp://:14540 --vehicle-type=fixed_wing --autopilot=px4 --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_airplane` | 16 passed, 2 failed |
| Standalone (PX4 VTOL) | `pytest tests/mission/nav_takeoff/test_protocol.py --drone-address=udp://:14540 --vehicle-type=vtol --autopilot=px4 --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_standard_vtol` | 16 passed, 2 failed |
| Standalone (ArduCopter) | `pytest tests/ --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 --ardupilot-sitl=~/ardu_sitl/arducopter --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 --vehicle-type=copter --autopilot=ardupilot` | 76 passed, 14 failed, 1 skipped, 1 xfailed |
| Standalone (ArduPlane fixed-wing) | `pytest tests/ --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 --ardupilot-sitl=~/ardu_sitl/arduplane --vehicle-type=fixed_wing --autopilot=ardupilot` (auto-detects model=plane) | 59 passed (frame_types), 13 passed (nav_takeoff), 15 passed + 3 skipped (command) |
| Standalone (ArduPlane QuadPlane) | `pytest tests/ --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 --ardupilot-sitl=~/ardu_sitl/arduplane --ardupilot-model=quadplane --vehicle-type=quadplane --autopilot=ardupilot` | 56 passed (frame_types), 13 passed (nav_takeoff), 15 passed + 3 skipped (command) |
| Standalone (ArduRover) | `pytest tests/command/ tests/mission/test_frame_types.py --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 --ardupilot-sitl=~/ardu_sitl/ardurover --ardupilot-model=rover --vehicle-type=rover --autopilot=ardupilot` | 67 passed, 13 failed, 3 skipped (4 command FAILs: NAV_TAKEOFF UNSUPPORTED; 9 frame FAILs: same altitude-category violations as ArduCopter) |
| Standalone (PX4 multicopter — command only) | `pytest tests/command/ tests/mission/test_frame_types.py --drone-address=udp://:14540 --connection-timeout=60 --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx --vehicle-type=quadcopter --autopilot=px4` | 15 passed + 3 skipped (command), 63 passed + 2 failed (frame_types: known geofence rel-alt bug) |

**`--vehicle-type` and `--autopilot` options**: these labels are used in log file naming only (e.g. `mission_frame_types_ardupilot_fixed_wing_4.8.0-dev_20260526.log`).  When omitted, the autopilot probe attempts auto-detection (firmware version and git hash are reliable; vehicle type often shows UNKNOWN because MAVSDK intercepts HEARTBEAT messages before they reach `mavlink_direct`).  Always pass these explicitly for clean log filenames.

**IMPORTANT**: Do not run paired-mode tests while PX4 SITL is running on the same machine.
PX4 uses sysid=1 (same as the mock drone) and can send traffic to port 14560 after it has
been a peer in a previous session.  This causes `_wait_for_connection` on the GCS System to
hang indefinitely.  Always `kill <px4_pid>` before running paired tests.

**Standalone PX4 peer-caching issue**: Running the full suite against PX4 while PX4 has
previously cached the paired loopback port 14560 as a peer causes `test_gcs_sends_mission_item_int`
to hang indefinitely.  **Fix**: always start a **fresh PX4 instance** (kill and restart) before
running standalone tests.  A freshly started PX4 has no peer cache.

Paired mode uses UDP loopback:
- Drone sends `udpout://127.0.0.1:14560`
- GCS binds `udpin://0.0.0.0:14560`
- Port 14560 is used deliberately (not 14540) to avoid PX4's default SDK port.
- In paired mode, `gcs_mavsdk_server` starts `drone_mavsdk_server` first so the peer is
  already connected when the GCS begins listening.

### ArduCopter SITL setup

```bash
# Download pre-built binary (one-time)
mkdir -p ~/ardu_sitl/sitl_working
curl -o ~/ardu_sitl/arducopter \
  https://firmware.ardupilot.org/Copter/latest/SITL_x86_64_linux_gnu/arducopter
chmod +x ~/ardu_sitl/arducopter

# Start SITL — must run from sitl_working; parm file path is exact (no shortcut)
cd ~/ardu_sitl/sitl_working
~/ardu_sitl/arducopter -S -I0 --model + \
  --home=37.6234,-122.0811,0,270 \
  --defaults ~/github/ArduPilot/ardupilot/Tools/autotest/default_params/copter.parm
```

### ArduCopter connection reliability pitfalls

Two issues that cause tests to hang indefinitely against ArduCopter SITL:

**1. CLOSE-WAIT accumulation** — ArduCopter does not close its TCP socket when a
client (mavsdk_server) disconnects abruptly.  After killing mavsdk_server, port
5760 appears open (`ss -tlnp`) but ArduCopter is stuck in CLOSE-WAIT and will not
send MAVLink heartbeats on a new connection.  **Fix**: restart ArduCopter SITL after
any abrupt kill of mavsdk_server or pytest.  Check readiness with `ss -tlnp | grep 5760`.
Do **not** use `nc -z 127.0.0.1 5760` — that creates yet another CLOSE-WAIT.

**2. Missing / wrong parm file** — ArduCopter silently accepts the TCP connection,
then panics while loading defaults and never sends a heartbeat.  The correct path
is `~/github/ArduPilot/ardupilot/Tools/autotest/default_params/copter.parm`.

## Design decisions

1. **JSON plan files, not QGC format** — The JSON format maps directly to
   MISSION_ITEM_INT fields, making each value traceable to the protocol spec.
   QGC format adds abstraction layers that hide the raw values.

2. **mission_raw over mission** — The `mission` plugin uses a high-level
   MissionItem abstraction (lat/lon/speed/camera_action etc.) and only
   supports flight plans.  `mission_raw` is needed for geofence, rally points,
   and direct protocol field access.

3. **Function-scoped System objects, session-scoped processes** —
   `gcs_mavsdk_server` / `drone_mavsdk_server` are session-scoped synchronous
   fixtures that start one `mavsdk_server` process per side for the whole
   session.  `gcs_system` / `drone_system` are function-scoped async fixtures
   that create a fresh `System` object per test in that test's own event loop.
   This avoids gRPC channel / event loop conflicts that arise from sharing
   channels across test functions.

   **Exception — frame-type tests**: `test_frame_types.py` runs 65 consecutive
   tests; creating 65 `System` objects exhausts mavsdk_server's gRPC resources
   (~35 open channels causes new connections to time out).  The frame tests use
   `@pytest_asyncio.fixture(scope="class", loop_scope="class")` to create ONE
   `System` per test class (4 total) and `@pytest.mark.asyncio(loop_scope="class")`
   to ensure all tests in the class share the same event loop.  Teardown is
   done in a `try/finally` block inside each test (not as a fixture) to avoid
   the event-loop mismatch that would occur if a function-scoped fixture tried
   to use the class-scoped System.

4. **TRANSFER_TIMEOUT_S = 30 s** — Conservative upper bound for a 4-item plan
   over loopback including MAVSDK gRPC overhead.  Reduce if tests become slow.

4a. **`_wait_for_connection` gRPC stream cancellation** — `system.core.connection_state()`
    is a gRPC streaming call.  Python's asyncio cancellation works by raising
    `CancelledError` at the next yield point.  For gRPC streams, that yield point
    may not arrive until the next connection-state change event from the server,
    which can be tens of seconds away.  Wrapping the `async for` loop in
    `asyncio.wait_for` does NOT reliably bound execution time, because after the
    timeout fires `wait_for` calls `await task` to wait for cancellation —
    which blocks indefinitely if the gRPC stream doesn't yield.

    **Fix**: run the gRPC subscription as a fire-and-forget `asyncio.create_task`.
    Only `await` a plain `asyncio.Event` (set by the background task when connected).
    `asyncio.wait_for(connected.wait(), timeout=N)` works correctly because `Event.wait()`
    is pure asyncio and cancels instantly.  After the timeout (or success), cancel the
    background task without awaiting it — the task will be cleaned up on its next I/O
    event.  This pattern is implemented in `_wait_for_connection` in `tests/conftest.py`.

4b. **PX4 SIH SITL readiness detection pitfalls** — Two bugs to avoid when detecting that PX4
    has started the mavlink module:
    (1) PX4 writes `"INFO  [mavlink]"` (two spaces between INFO and the bracket), not
    `"INFO [mavlink]"` (one space).  Searching for the one-space form silently misses
    all 45 readiness loop iterations, causing a `pytest.fail()` after 45 seconds.
    (2) PX4 enters a `pxh>` shell-prompt refresh loop immediately after startup that
    grows the log file at several GB per minute via repeated `[2K` (erase-line) escape
    sequences.  Calling `log_path.read_text()` on a multi-GB file is slow and wastes
    memory; reading only the first 64 KB is sufficient since the startup info appears
    in the first few KB.
    Fix: check `"INFO  [mavlink]"` (two spaces) and read at most 64 KB from the log file.

5. **asyncio.timeout() over asyncio.wait_for()** — Python 3.11+ timeout
   context manager is used where available.  If Python 3.10 compatibility is
   needed, replace `asyncio.timeout(t)` with `asyncio.wait_for(..., timeout=t)`.

6. **GCS paired identity: sysid=255, compid=1** — The GCS mavsdk_server MUST
   use `compid=1` (MAV_COMP_ID_AUTOPILOT1) in paired mode, not `compid=190`
   (MAV_COMP_ID_MISSIONPLANNER).  MAVSDK only fires "System discovered" — and
   starts its gRPC server — when it finds a component with compid=1.  With
   compid=190, the drone's gRPC port (50052) never opens and `System.connect()`
   hangs in `channel_ready()` indefinitely.  The sysid=255 keeps GCS and drone
   as distinct MAVLink systems; the compid=1 satisfies the drone's discovery
   requirement.

7. **`collect_incoming_mission()` workaround** — MAVSDK-Python v3.15.x has a
   bug in `mission_raw_server.incoming_mission()`: when the mavsdk_server binary
   sends the mission plan with result=SUCCESS (its actual behaviour), the Python
   generator returns immediately *without* yielding the plan.  The plan is present
   in the response but discarded by the early-return on SUCCESS.  The
   `collect_incoming_mission()` helper in `tests/mission/conftest.py` bypasses
   this by calling the raw gRPC stub (`SubscribeIncomingMission`) directly and
   extracting the plan from any non-error response.  If MAVSDK-Python is updated
   to fix this (server sends NEXT, not SUCCESS, when delivering the plan), this
   helper can be replaced with a direct `incoming_mission()` call.

## Future work / known gaps

These items were identified during development but deferred:

1. **Error-condition tests** — Tests that deliberately provoke error responses
   (MISSION_ACK with non-zero error codes) and verify the correct
   `MissionRawResult.Result` enum value is raised.  Currently only the happy
   path is tested; `TestErrorHandling.test_int_messages_not_supported_is_raised`
   is skipped because we cannot force that condition from the suite.

2. **Geofence roundtrip frame comparison** — PX4 converts geofence items from
   frame=0 to frame=5 on storage.  The roundtrip test currently xfails; could
   be promoted to a proper test once `items_match()` supports frame-aware comparison.

3. **Frame-aware roundtrip comparison** — Extend `items_match()` to accept a
   coordinate-frame translation function so that roundtrip tests can verify
   field values even when the autopilot changes the frame on storage.
   (`test_frame_types.py` documents which frames cause storage transforms but
   does not verify coordinate values for local/body frames.)

4. ~~**Download-only tests need prior upload**~~ — Fixed: each test now uploads
   what it needs and cleans up via an autouse teardown fixture.

5. **Heartbeat protocol tests** — Add `tests/heartbeat/` covering the
   MAVLink heartbeat service (component type, autopilot type, base mode, etc.).

6. **PX4 geofence relative-altitude frame z conversion** — `TestGeofenceFrames` currently
   fails for frame=3 (MAV_FRAME_GLOBAL_RELATIVE_ALT) and frame=6
   (MAV_FRAME_GLOBAL_RELATIVE_ALT_INT): PX4 accepts both but stores them as frame=5
   (MAV_FRAME_GLOBAL_INT, absolute) without converting the z value.  The test flags
   this as a protocol violation (relative→absolute change must adjust z).  Needs
   investigation: (a) is z conversion required by the MAVLink spec for geofence items,
   or is z always treated as MSL for geofence regardless of frame?  (b) if required,
   this is a PX4 bug to report upstream.  Left as hard FAILED until resolved.

## Change log

| Date | Change |
|------|--------|
| 2026-05-15 | Initial implementation: mission client/server tests, plans, README, CLAUDE.md |
| 2026-05-16 | Fix capability bit (1<<14→4), AUTOPILOT_VERSION request (cmd 520→512/148), clear_mission() API, frame-change xfail for roundtrip, geofence xfail for PROTOCOL_ERROR; add PX4 behaviour notes and future work list |
| 2026-05-17 | Fix paired-mode tests (6 tests now pass): (a) GCS compid=190→1 so drone discovers it and starts gRPC; (b) add collect_incoming_mission() workaround for MAVSDK-Python bug where SUCCESS result discards plan; (c) fix test_clear_all_received to use clear_mission() not upload_mission([]); (d) rewrite deprecated-message test to use raw stub |
| 2026-05-17 | Fix geofence tests: use PX4/pymavlink command values (5000=return, 5001=inclusion) not mavlink.io 5001-based values; add frame-change xfail for roundtrip (PX4 converts frame=0→5 on storage). All 8 standalone tests now pass. |
| 2026-05-18 | Move paired-mode GCS port from 14540→14560 so server tests can run alongside a live PX4 SITL without interference (PX4 sends heartbeats to 14540; using 14560 keeps the loopback session isolated). |
| 2026-05-18 | Add per-test autouse teardown to TestFlightMission/TestGeofence/TestRallyPoints: each test now clears all mission types after it runs via clear_all_mission_types() in mission conftest. Fix download tests (and test_clear_flight_mission) to upload explicitly before acting — no longer order-dependent. Fix CLAUDE.md: clear_mission() sends type=0 only (not 255); geofence/rally cleared via raw mavlink_direct MISSION_CLEAR_ALL. |
| 2026-05-18 | Add test_frame_types.py: 65-test MAV_FRAME support matrix (21 frames × 3 mission types + 2 MAV_FRAME_MISSION tests). Stack-agnostic — accepts any clean accept/reject outcome. Uses pytest_asyncio.fixture(loop_scope="class") + mark.asyncio(loop_scope="class") to share one System per class (avoids gRPC exhaustion from 65 connections). Documents PX4 frame support matrix in CLAUDE.md. |
| 2026-05-20 | Add MockFlightStack (tests/mock_flight_stack.py): full mavlink_direct implementation of upload/download/clear/capability protocols. Rewrote conftest.py gcs_system/mock_stack fixtures to be mode-aware (paired vs standalone). Add mock_stack_cls class-scoped fixture in test_frame_types.py. All 83 tests now run in paired mode: 82 pass, 1 skip. Fix _get_autopilot_capabilities to OR-combine responses within 0.3 s window (mavsdk_server and mock both respond). Add PX4 interference warning: do not run paired tests while PX4 SITL is running. |
| 2026-05-23 | Run full test suite against ArduCopter V4.8.0-dev SITL (pre-built binary): 79 passed, 2 failed, 1 skipped, 1 xfailed. Add ArduCopter-specific behaviour section to CLAUDE.md. New failures: test_clear_flight_mission (home waypoint retained after clear) and test_mission_frame_with_do_command (MAV_FRAME_MISSION param1 zeroed). Log saved: logs/test_arducopter_20260524_095950.log. |
| 2026-05-24 | Prove hypothesis: ArduCopter rejects single-item flight missions (frame=0) because it requires seq=0 to carry the home position. Confirmed by test_ardu_home_hypothesis.py: uploading [home@seq=0, waypoint@seq=1] is accepted with frame preserved. Add ardu_home_mission.json plan and test. Update CLAUDE.md: add SITL reliability pitfalls (CLOSE-WAIT, wrong parm file), correct startup command, update running modes table. Add --ardupilot-sitl fixture for automated SITL management. Add test_protocol_conformance.py: home-slot-not-required conformance test (FAIL for ArduCopter). Update test_frame_types.py: session probe + home-item prepend so flight frame tests work on all platforms. |
| 2026-05-24 | Run full test suite against all three stacks. Mock: 84 passed, 1 skipped. PX4: 79 passed, 2 failed, 1 skipped, 3 xfailed. ArduCopter (with home-slot prepend): 73 passed, 10 failed, 1 skipped, 1 xfailed — 8 new frame failures: frames 3, 6, 10, 11 for flight missions and geofence now correctly identified as altitude-category violations (previously hidden as REJECTED due to home-slot conflict). Document PX4 peer-caching hang (PX4 caches loopback port in RAM; must restart before standalone tests). Rewrite README.md with accurate test counts and autopilot behaviour tables. Update ArduCopter frame tables in CLAUDE.md. |
| 2026-05-24 | Add test_protocol.py (7 tests, Tier 1 protocol acceptance). NAV_TAKEOFF results across 6 stacks: PX4 (multicopter/FW/VTOL all identical) 5 pass/2 fail (param1/param3 zeroed, param4 preserved); ArduCopter/ArduPlane/QuadPlane all identical 3 pass/4 fail (param3/param4 zeroed, param2 NaN rejected = spec violation, param1 preserved). Download arduplane binary. Update unused-param testing convention to NaN-first three-probe pattern. Logs: test_px4_*_nav_takeoff_20260524.log, test_arduplane_nav_takeoff_20260524.log, test_quadplane_nav_takeoff_20260524.log. |
| 2026-05-24 | Add test_flight.py (2 tests, Tier 2 execution). test_takeoff_implicit_from_waypoint: uploads NAV_WAYPOINT with no explicit takeoff, asserts vehicle climbs to ≥85% of target altitude. test_takeoff_with_yaw: uploads NAV_TAKEOFF with param4=137°, asserts heading ≈137° after takeoff (expected PASS on PX4, FAIL on ArduPilot — param4 not stored). Both skip in mock/paired mode. Mock suite: 91 passed, 3 skipped. |
| 2026-05-25 | Reorganise NAV_TAKEOFF tests into tests/mission/nav_takeoff/ subpackage (test_protocol.py, test_flight.py). Add 5 Tier 1 edge-case tests: param4 yaw −90° (observational), 450° (observational), 0° (assert preserved); param1 pitch 89° (observational), −10° (observational). Add 4 conditional Tier 2 tests using inline-probe pattern: test_takeoff_with_negative_yaw, test_takeoff_with_overflow_yaw, test_takeoff_with_large_pitch, test_takeoff_with_negative_pitch — each probes storage first and skips if the value was normalised on storage (execution unambiguous), proceeding only when the raw edge-case value was stored. Document conditional Tier 2 pattern in CLAUDE.md. Mock suite: 96 passed, 7 skipped. |
| 2026-05-25 | Run all 12 Tier 1 tests (test_protocol.py) against all 6 vehicle/stack combinations. PX4 (all 3 types identical): 10 pass / 2 fail — yaw normalises −90°→270° and 450°→90° on storage; param1 zeroed for all values including 89° (PX4 does not store param1 at all). ArduPilot (all 3 types identical): 8 pass / 4 fail — negative pitch (−10°) stored as 65526° (uint16 underflow — storage bug); positive out-of-range pitch (89°) preserved raw; yaw edge cases (−90°, 450°) both become 0.0° (param4 not stored). Create tests/mission/nav_takeoff/README.md with per-test result tables. |
| 2026-05-25 | Gap-analysis against generic MAV_CMD methodology. Add 6 Tier 1 tests: test_protocol_location_current_position (INT32_MAX sentinel), test_protocol_location_nan_altitude (observational), test_protocol_param3_flags_zero, test_protocol_param3_flags_undefined_bits (observational), test_protocol_param1_nan (observational), test_protocol_param1_pitch_very_large (observational). Add 1 conditional Tier 2 test: test_takeoff_from_current_position. Refine CLAUDE.md methodology section: add implicit/explicit range testing, bitmask/enum protocols, location sentinel tests (INT32_MAX, NaN alt), observational test conventions, vacuous-PASS annotation. Mock suite: 102 passed, 8 skipped. Run all 18 Tier 1 tests against all 6 stacks: PX4 (all 3 types) 16 pass / 2 fail; ArduCopter 13 pass / 5 fail (new: INT32_MAX NACKed — spec violation); ArduPlane FW / QuadPlane 13 pass / 5 fail (same). Update nav_takeoff/README.md with full 18-test results table. Update root README.md spec-violations table and MAV_CMD summary table. |
| 2026-05-25 | Add `tests/command/` tree: command protocol (COMMAND_INT/COMMAND_LONG) test infrastructure parallel to existing mission protocol tests. Added `mavlink/` git submodule and `--mavlink-definitions-dir` CLI option. Extended MockFlightStack with COMMAND_INT/COMMAND_LONG handlers, MAV_RESULT constants (0–8), `received_commands` list, ACK-drop injection, and IN_PROGRESS sequence emission. Created `tests/command/conftest.py` (probe helpers using subscribe-before-send pattern to avoid race condition, retry helpers, XML command loader), `test_survey.py` (probes all 168 MAV_CMD from common.xml; writes support matrix to logs/), `test_protocol.py` (8 protocol conformance tests: ACK received, ACK echoes ID, ACCEPTED result, COMMAND_LONG ACK, confirmation increments, retry-after-dropped-ACK, IN_PROGRESS→ACCEPTED sequence, UNSUPPORTED result), `takeoff/test_command.py` (9 NAV_TAKEOFF COMMAND_INT tests), `takeoff/README.md`. Key findings: MAVSDK's `mavlink_direct.send_message` rejects NaN in COMMAND_INT float fields with INVALID_FIELD; subscribe-before-send pattern with 0.05 s settle is required. Mock suite: 119 passed, 8 skipped. |
| 2026-05-26 | Add autopilot probe infrastructure: session-scoped `_autopilot_header` autouse fixture in `tests/conftest.py` probes the connected stack using `system.info.get_version()` (firmware version, git hash — reliable) and `system.info.get_product()` (vendor name, best-effort). MAVSDK intercepts HEARTBEAT internally so `mavlink_direct.message("HEARTBEAT")` is unreliable for vehicle type — added `--vehicle-type` and `--autopilot` CLI overrides instead. ArduPilot stores git hash as ASCII bytes in `flight_custom_version`; ASCII-decoded before display. `suggest_log_filename()` auto-derives log prefix from test path (e.g. `tests/mission/test_frame_types.py` → `mission_frame_types`). Run `test_frame_types.py` against all available stacks: Mock (65/65), ArduCopter (56/65), ArduPlane fixed-wing (59/65), ArduPlane QuadPlane (56/65). Key finding: ArduPlane fixed-wing differs from ArduCopter/QuadPlane — frame 11 REJECTED (PASS) vs ACCEPTED/FAIL; frame 13 ACCEPTED/FAIL vs REJECTED/PASS; geofence rejects almost all frames. Add ArduPlane fixed-wing and QuadPlane sections to CLAUDE.md. |
| 2026-05-26 | Replace hardcoded `_MAV_AUTOPILOT`/`_MAV_TYPE`/`_MAV_FIRMWARE_TYPE` dicts in `tests/conftest.py` with `_load_mavlink_enum()` which parses the bundled XML submodule at import time (21/50 entries vs 5/16 hardcoded). Fallback to hardcoded values with `log.warning` when submodule absent. Add `pytest-timeout` to requirements and set global `timeout=120` in `pytest.ini`. Add `@pytest.mark.timeout(900)` to `TestCommandSurvey`, `pytestmark=timeout(360)` to flight tests. Document timeout policy in CLAUDE.md Conventions. |
| 2026-05-26 | Automate SITL lifecycle management: add `_manage_ardupilot_sitl` (handles arducopter/arduplane/ardurover with auto-model detection and defaults .parm lookup) and `_manage_px4_sitl` session fixtures. Add `--ardupilot-sitl`, `--ardupilot-model`, `--px4-sitl`, `--px4-model` CLI options. `gcs_mavsdk_server` explicitly depends on SITL fixtures to guarantee flight stack starts before GCS connects. Fix `_clear_stale_mavsdk_servers` to kill ALL mavsdk_server processes (not just port-scanning). Fix `command_survey` log filename to include autopilot/vehicle/version info. Fix `_wait_for_connection` gRPC cancellation bug: use fire-and-forget background task + asyncio.Event instead of wrapping gRPC stream in asyncio.wait_for (gRPC streams don't respond to asyncio cancellation). Add `@pytest.mark.timeout(300)` to `TestCommandProtocol` and `TestNavTakeoffCommand`. Create `tests/command/README.md` with survey results table and per-stack test results. |
| 2026-05-26 | Run command tests + frame tests for ArduPlane QP, ArduRover, and PX4 MC. Fix two bugs in `_manage_px4_sitl`: (1) wrong rootfs path — was `/tmp/px4_sitl_work` (no `etc/init.d-posix/` present), now `build/px4_sitl_default` (correct rootfs); (2) wrong search string for readiness detection — was `"INFO [mavlink]"` (one space) but PX4 writes `"INFO  [mavlink]"` (two spaces). Also fix `read_text()` on the rapidly-growing pxh> shell prompt log (grows at GB/min after startup) by reading only first 64 KB. Increase PX4 readiness loop from 45→60 iterations. All stacks now work with auto-managed SITL. Key results: ArduPlane QP 15p+3s (command), 56p+9f (frames); ArduRover 11p+4f+3s (command — 4 FAILs because NAV_TAKEOFF UNSUPPORTED on rover), 56p+9f (frames); PX4 MC 15p+3s (command), 63p+2f (frames — geofence rel-alt bug). Add ArduRover and PX4 command-protocol behaviour sections to CLAUDE.md. Update `tests/command/README.md` and `tests/command/takeoff/README.md` with complete comparative results. |
| 2026-05-26 | Replace pymavlink NaN bypass with `None` (JSON null). Discovered that `mavlink_direct.send_message` accepts `None` in float fields: Python `None` → `json.dumps(None)` → `"null"` → nlohmann/json decodes float null as IEEE-754 NaN → transmitted on wire → decoded back to `null` → Python `None`. Removed `nan_mavlink_conn` fixture, `--nan-mavlink-address` CLI option, `probe_command_int_nan()`, `pymavlink>=2.4.0` from requirements.txt, and `--out udp:127.0.0.1:14550` from ArduPilot SITL startup. The three NaN tests in `takeoff/test_command.py` now use `param=None` via the normal `probe_command_int()` path and pass in all modes. Paired mode: 119 passed / 8 skipped (the 3 NaN tests no longer skip). |
| 2026-05-27 | Split CLAUDE.md into per-subfolder files: tests/mission/CLAUDE.md (mission protocol, frame tables, MAV_CMD methodology), tests/mission/nav_takeoff/CLAUDE.md (NAV_TAKEOFF storage test results), tests/command/CLAUDE.md (command protocol, per-stack command notes). Root CLAUDE.md reduced from 1165 → ~420 lines. Also fixed stale ArduRover command survey counts (111→118 UNSUPPORTED, 8→1 UNKNOWN) and log reference (_140430 → _212652). |
| 2026-05-27 | Run command + frame tests for PX4 FW (sihsim_airplane), VTOL (sihsim_standard_vtol), and Rover (sihsim_rover_ackermann). All three: 78 passed, 2 failed, 3 skipped — same failures as PX4 MC (geofence relative-alt frames 3 and 6). Survey differences vs MC: FW adds DO_FIGURE_EIGHT (35) SUPPORTED; VTOL adds DO_FIGURE_EIGHT + DO_VTOL_TRANSITION (3000) SUPPORTED; Rover loses DO_AUTOTUNE_ENABLE (212) SUPPORTED. Key finding: PX4 Rover accepts NAV_TAKEOFF (result=0) unlike ArduRover which returns UNSUPPORTED — PX4 does not gate commands by vehicle type. Each run takes ~1m 43s. Logs: logs/command_frame_types_px4_{fixed_wing,vtol,rover}_1.18.0-alpha_20260527.log. |
| 2026-05-27 | Add `tests/command/takeoff/test_flight.py` (17 Tier 2 COMMAND_INT execution tests). Two-stage gate: ACK probe (MAVSDK subscription method), then execution probe (arm + send COMMAND_INT + wait ≤20 s for 0.5 m climb). Key finding: PX4 ignores the `frame` field in COMMAND_INT NAV_TAKEOFF and always treats `z` as absolute AMSL — `_arm_and_send_takeoff()` converts the caller's relative z to absolute (home.absolute_altitude_m + relative_z) and uses frame=5 (GLOBAL_INT). Fix: gRPC pitch_task cancellation uses fire-and-forget (no await) per CLAUDE.md §4a; test_mode_after_takeoff handles TimeoutError gracefully (purely informational). Results: PX4 MC 15 pass + 2 xpass (z=0 and x/y=0 spec gaps); PX4 FW 17 skip (requires runway, not vertical climb); ArduCopter MC 17 skip (requires GUIDED mode); ArduPlane FW 17 skip (same). Observational findings on PX4 MC: param4 (yaw) ignored in COMMAND_INT path (heading always ~351°); z=NaN → default alt (~0.5 m relative); x/y=0 → PX4 navigates toward equator (not "use current"); INT32_MAX → vehicle stays at home (0.7 m dist). Log: logs/command_takeoff_flight_px4_quadcopter_1.18.0-alpha_20260527_134513.log. |

When protocol behaviour changes (spec update, MAVSDK API change, or
autopilot-specific workaround is added), add a row to this table and update
the relevant sections above.
