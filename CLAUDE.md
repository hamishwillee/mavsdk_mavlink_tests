# CLAUDE.md — Implementation notes for Claude

This file records protocol behaviour, design decisions, and change-tracking
notes for use in future Claude sessions.

## Project purpose

MAVLink protocol conformance tests, using MAVSDK-Python as the MAVLink
transport.  The initial focus is the mission protocol (common.xml).

## Architecture

```
conftest.py              CLI options: --drone-address, --connection-timeout
tests/conftest.py        GCS (gcs_system) and drone (drone_system) fixtures
tests/mock_flight_stack.py  MockFlightStack — configurable MAVLink drone simulator
tests/mission/
  conftest.py            load_plan(), items_match(), clear_all_mission_types() helpers
  test_mission_client.py GCS-side tests using mission_raw plugin
  test_mission_server.py Drone-side tests using mission_raw_server plugin
  test_frame_types.py    MAV_FRAME support matrix (65 tests, stack-agnostic)
  plans/                 JSON plan files (MISSION_ITEM_INT fields)
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

**Default behaviour**: accept all frames, store items exactly as received, serve
unchanged on download, report `MAV_PROTOCOL_CAPABILITY_MISSION_INT` (bit 2 = 4).

**Configurable parameters** (constructor kwargs):
- `capability_bits` — capabilities bitmask in AUTOPILOT_VERSION response
- `item_request_delay_s` — per-item delay before MISSION_REQUEST_INT (upload)
- `item_response_delay_s` — per-item delay before MISSION_ITEM_INT (download)
- `drop_responses` — `{msg_name: N}` to silently drop first N of that outgoing message
- `rejected_frames` — frame values to NACK with UNSUPPORTED_FRAME on upload

**Capability response interaction**: the drone mavsdk_server binary also responds
to AUTOPILOT_VERSION requests with its own bits (0x2000 = MAV_PROTOCOL_CAPABILITY_MAVLINK2).
`_get_autopilot_capabilities()` in the test file OR-combines all responses within a
0.3 s window so both the server's and the mock's bits are reflected in the result.

All tests are async (pytest-asyncio, asyncio_mode=auto).

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

## Protocol behaviour (mission protocol)

### Timeouts

Per the MAVLink specification (https://mavlink.io/en/services/mission.html):

| Parameter | Value |
|-----------|-------|
| TIMEOUT_INITIAL_RESPONSE | 1500 ms |
| TIMEOUT_ITEM_RESPONSE    | 250 ms  |
| MAX_RETRIES              | 5       |

If these change in the spec, update:
1. The docstring in `test_mission_client.py` (module-level).
2. The README timeout table.
3. The `TRANSFER_TIMEOUT_S` constant in both test files.

### Capability check

`MAV_PROTOCOL_CAPABILITY_MISSION_INT = 4` (bit 2 of the
AUTOPILOT_VERSION.capabilities field).

The check in `TestCapability.test_mission_int_capability` uses
`mavlink_direct` to send `MAV_CMD_REQUEST_MESSAGE` (512) with
`param1=148.0` (AUTOPILOT_VERSION message ID), then reads the `capabilities`
field from the `AUTOPILOT_VERSION` response.

Note: `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` (520) is NOT used — PX4 does
not support command 520 and logs "command 520 unsupported".  PX4 also does not
passively broadcast AUTOPILOT_VERSION, so the request is always required.

If the autopilot does not set this bit, `mission_raw` returns
`MissionRawResult.Result.INT_MESSAGES_NOT_SUPPORTED`.

### Deprecated message handling

The original (pre-MAVLink 2) upload path used:
- MISSION_REQUEST (deprecated) instead of MISSION_REQUEST_INT
- MISSION_ITEM (deprecated) instead of MISSION_ITEM_INT

The spec requires modern GCS implementations to still handle MISSION_REQUEST
by completing the upload (responding with MISSION_ITEM_INT).  MAVSDK does this
transparently.

Tests:
- `TestDeprecatedMessageHandling.test_deprecated_request_yields_int_response`
  (client) — verifies that MISSION_ITEM_INT (not MISSION_ITEM) is used in
  the GCS response during a paired test.
- `TestDeprecatedRequestHandling.test_respond_with_deprecated_request`
  (server) — confirms that the upload completes even if the server triggers
  the deprecated path.

If MAVSDK removes this fallback, both tests will fail with
`INT_MESSAGES_NOT_SUPPORTED`; update the tests and this file accordingly.

### Mission types (MAV_MISSION_TYPE)

| Value | Name | MAVSDK method pair |
|-------|------|--------------------|
| 0 | MAV_MISSION_TYPE_MISSION | upload: `upload_mission()` / download: `download_mission()` |
| 1 | MAV_MISSION_TYPE_FENCE   | upload: `upload_geofence()` / download: `download_geofence()` |
| 2 | MAV_MISSION_TYPE_RALLY   | upload: `upload_rally_points()` / download: `download_rallypoints()` |

The rally pair has a deliberate spelling asymmetry in the MAVSDK API: upload is
`upload_rally_points` (underscore) but download is `download_rallypoints` (no
underscore). This is the MAVSDK API spelling, not a typo here.

### Clear mission

`mission_raw.clear_mission()` sends MISSION_CLEAR_ALL with **mission_type=0**
(flight missions only — empirically verified against PX4).  It does **not**
clear geofence (type=1) or rally points (type=2).  To clear all types, send
raw MISSION_CLEAR_ALL messages for types 1 and 2 via `mavlink_direct`; see
`clear_all_mission_types()` in `tests/mission/conftest.py`.

`upload_mission([])` / `upload_geofence([])` / `upload_rally_points([])` all
raise `NO_MISSION_AVAILABLE` — they are not equivalent to clearing and must
not be used to clear stored missions.

### Coordinate encoding (MISSION_ITEM_INT)

- `x` = latitude  × 1e7  (int32_t)
- `y` = longitude × 1e7  (int32_t)
- `z` = altitude in metres (float)
- `frame=5` (MAV_FRAME_GLOBAL_INT) — altitude absolute (AMSL); correct frame for MISSION_ITEM_INT with int32 lat/lon
- `frame=6` (MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) — altitude relative to takeoff; INT variant for MISSION_ITEM_INT
- `frame=0` (MAV_FRAME_GLOBAL) — nominally float lat/lon, but PX4 handles this in MISSION_ITEM_INT correctly (see below)
- `frame=3` (MAV_FRAME_GLOBAL_RELATIVE_ALT) — nominally float lat/lon; PX4 also handles this in MISSION_ITEM_INT

**Frame and int_mode in PX4:** PX4 tracks whether the exchange uses MISSION_ITEM_INT via an
internal `_int_mode` flag, set to `true` when `MISSION_ITEM_INT` is received (regardless of
frame).  On receive, if `_int_mode=true`, x/y are decoded as `int32×1e-7` even if the frame
field is 0 (GLOBAL) or 3 (GLOBAL_RELATIVE_ALT).  On send, frame is upgraded to the INT
variant (0→5, 3→6) in the response.  So frame=0 in uploaded items is accepted and stored
correctly, but PX4 always returns frame=5 on download.  Using frame=5/6 in upload items is
more spec-correct and also causes the roundtrip frame check to succeed.

### Float comparison

Autopilots (especially ArduPilot) round float parameters on storage.  The
`items_match()` helper in `tests/mission/conftest.py` uses `tol=1e-4`.  If
new tests encounter false failures due to rounding, increase tolerance and
update this note.

## Running modes

| Mode | Command | What runs |
|------|---------|-----------|
| Paired (mock) | `pytest tests/` | 82 tests pass, 1 skipped — no autopilot needed |
| Standalone (PX4) | `pytest tests/ --drone-address=udp://:14540` | Same 83 tests against real PX4 |

**IMPORTANT**: Do not run paired-mode tests while PX4 SITL is running on the same machine.
PX4 uses sysid=1 (same as the mock drone) and can send traffic to port 14560 after it has
been a peer in a previous session.  This causes `_wait_for_connection` on the GCS System to
hang indefinitely.  Always `kill <px4_pid>` before running paired tests.

Paired mode uses UDP loopback:
- Drone sends `udpout://127.0.0.1:14560`
- GCS binds `udpin://0.0.0.0:14560`
- Port 14560 is used deliberately (not 14540) to avoid PX4's default SDK port.
- In paired mode, `gcs_mavsdk_server` starts `drone_mavsdk_server` first so the peer is
  already connected when the GCS begins listening.

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

## Autopilot-specific behaviour (PX4)

Tested against PX4 mainline branch `mission_request_returns_int` with SIH
simulator (`PX4_SIM_MODEL=sihsim_quadx`), default SIH home
(47.397742°N, 8.545594°E).

### Coordinate frame conversion (flight missions)

PX4 accepts `MAV_FRAME_GLOBAL_RELATIVE_ALT` (frame=3) uploads but stores
waypoints in local NED (frame=6) internally.  On download, items are returned
with frame=6 and x≈0, y≈0 because the test coordinates are centred on the SIH
home position.  The roundtrip test detects this frame change and calls
`pytest.xfail()` rather than failing hard.

**Impact:** Field-by-field roundtrip comparison is not possible with PX4
without frame-aware coordinate conversion.

### Geofence command values (PX4 vs current MAVLink spec)

PX4 and pymavlink use **5000-based** fence command values, while the current
mavlink.io online spec lists **5001-based** values.  The PX4 bundled MAVLink
XML (`src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml`) defines:

| Command | PX4/pymavlink value | mavlink.io (current) |
|---------|--------------------|-----------------------|
| FENCE_RETURN_POINT | **5000** | 5001 |
| FENCE_POLYGON_VERTEX_INCLUSION | **5001** | 5002 |
| FENCE_POLYGON_VERTEX_EXCLUSION | **5002** | 5003 |

The `simple_geofence.json` plan and all geofence tests use the PX4/pymavlink
values (5000, 5001, …).  If you send the mavlink.io values (5001 for return
point), PX4 parses cmd=5001 as `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION`,
computes `vertex_count = param1 + 0.5 = 0`, and immediately rejects with
`MAV_MISSION_ERROR` ("Fence: too few vertices") — this was the root cause of
the earlier geofence upload failures.

### Geofence coordinate frame (PX4)

PX4 accepts geofence items with `frame=0` (MAV_FRAME_GLOBAL) but stores them
as `frame=5` (MAV_FRAME_GLOBAL_INT) internally.  On download, all items are
returned with frame=5; coordinates (x, y, z) are preserved unchanged.
The roundtrip test detects the frame change and calls `pytest.xfail()`.

**Impact:** Field-by-field roundtrip comparison is not possible with PX4
without frame-aware comparison, but upload/download of fence items works
correctly (coordinates are preserved).

### Rally points

PX4 accepts `upload_rally_points()` and stores rally items as WGS84 lat/lon
doubles (same as geofence items).  `download_rallypoints()` returns the stored
items with frame upgraded from 0 (MAV_FRAME_GLOBAL) to 5 (MAV_FRAME_GLOBAL_INT)
but coordinates (x, y, z) are preserved unchanged.  The roundtrip test detects
the frame change and calls `pytest.xfail()`.

Note: if PX4 is started with a stale `dataman` file from a previous session, the
dataman slot alternation can cause the download to read from the wrong slot, returning
x=0, y=0.  Starting with a clean `dataman` file (delete or reset between sessions)
ensures correct behaviour.  In the test suite the PX4 process is session-scoped so
the dataman state persists within a session; the `test_upload_rally_points` test runs
before `test_roundtrip_rally_points` and leaves the slot in the correct state.

### AUTOPILOT_VERSION request

PX4 does not respond to `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` (520) and
does not passively broadcast AUTOPILOT_VERSION.  Use
`MAV_CMD_REQUEST_MESSAGE` (512) with `param1=148.0` to request it.

### Frame type support (PX4)

Determined by `test_frame_types.py` against PX4 SIH.

**Flight missions (mission_type=0), Geofence (type=1), Rally (type=2):**

| Frame | Name                          | PX4 result                                  |
|-------|-------------------------------|---------------------------------------------|
| 0     | MAV_FRAME_GLOBAL              | ACCEPTED; downloaded as frame=5 (INT upgrade) |
| 1     | MAV_FRAME_LOCAL_NED           | REJECTED (UNSUPPORTED)                      |
| 3     | MAV_FRAME_GLOBAL_RELATIVE_ALT | ACCEPTED (flight: frame=6; fence/rally: frame=5) |
| 4     | MAV_FRAME_LOCAL_ENU [dep]     | REJECTED (UNSUPPORTED)                      |
| 5     | MAV_FRAME_GLOBAL_INT          | ACCEPTED; frame preserved on download        |
| 6     | MAV_FRAME_GLOBAL_RELATIVE_ALT_INT | ACCEPTED (flight: frame=6; fence/rally: frame=5) |
| 7–12  | LOCAL/BODY frames             | REJECTED (UNSUPPORTED)                      |
| 13–19 | RESERVED                      | REJECTED (UNSUPPORTED)                      |
| 20–21 | LOCAL_FRD / LOCAL_FLU         | REJECTED (UNSUPPORTED)                      |

**MAV_FRAME_MISSION (frame=2):**
- `DO_CHANGE_SPEED` (non-location cmd): **ACCEPTED**, param1 preserved unscaled.
- `NAV_WAYPOINT` (location cmd, misuse): **REJECTED** (PX4 correctly refuses MISSION frame for location commands).

Note: fence/rally download frame=5 instead of 6 for frames 3 and 6 — PX4's fence/rally
storage always uses GLOBAL_INT (frame=5) regardless of the relative-alt input frame.

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

When protocol behaviour changes (spec update, MAVSDK API change, or
autopilot-specific workaround is added), add a row to this table and update
the relevant sections above.
