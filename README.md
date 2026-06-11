# MAVLink Protocol Tests

Python-based protocol tests for the MAVLink common message set using [MAVSDK-Python](https://github.com/mavlink/MAVSDK-Python) as the transport library.

The test suite validates both the **client (GCS)** and **server (drone)** sides of each MAVLink service, and runs both sides against each other using a built-in mock drone — no external simulator required.

## Requirements

- Python 3.10+
- `mavsdk >= 2.0.0` (includes `mavsdk_server` binary)
- `pytest >= 8.0.0`
- `pytest-asyncio >= 0.23.0`

Install dependencies:

```bash
pip install -r requirements.txt
```

## Supported dialects

The default dialect is `common.xml`.
MAVSDK-Python includes common.xml support out of the box.
To load a custom dialect that includes common.xml, pass its XML to `mavlink_direct.load_custom_xml()` inside your test fixture.

## Running the tests

### Mock mode (no external drone needed)

```bash
pytest tests/
```

Starts two local `mavsdk_server` processes over loopback UDP and runs `MockFlightStack` as the drone-side handler.
All 214 tests run without any external simulator.
Expected result: **161 passed, 50 skipped, 3 xfailed** (the skips are Tier 2 execution tests and stack-specific probes that require a real flight stack).

### Against a real drone or simulator

```bash
# PX4 SITL (UDP port 14540 — start PX4 with PX4_SIM_MODEL=sihsim_quadx first)
pytest tests/ --drone-address=udp://:14540

# ArduCopter SITL (TCP port 5760 — requires --home-lat/lon for the SITL home position)
pytest tests/ \
  --drone-address=tcp://127.0.0.1:5760 \
  --connection-timeout=60 \
  --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0

# Let the suite start/stop ArduCopter SITL automatically
pytest tests/ \
  --drone-address=tcp://127.0.0.1:5760 \
  --connection-timeout=60 \
  --ardupilot-sitl=~/ardu_sitl/arducopter \
  --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0

# Serial connection
pytest tests/ --drone-address=serial:///dev/ttyUSB0:57600

# Increase connection timeout for slow links (default 30 s)
pytest tests/ --drone-address=udp://:14540 --connection-timeout=60
```

### Running a subset

```bash
# Frame-type support matrix only (21 frames × 3 mission types)
pytest tests/mission/test_frame_types.py -v

# Protocol conformance tests only
pytest tests/mission/test_protocol_conformance.py -v

# Client tests only
pytest tests/mission/test_mission_client.py --drone-address=udp://:14540

# Server tests only (always use mock, ignore --drone-address)
pytest tests/mission/test_mission_server.py

# NAV_TAKEOFF tests only (Tier 1 + Tier 2)
pytest tests/mission/nav_takeoff/ -v --log-cli-level=INFO
pytest tests/mission/nav_takeoff/ --drone-address=udp://:14540 -v --log-cli-level=INFO

# DO_REPOSITION tests only (Tier 1 — rejected as a mission item on every stack; see tests/mission/do_reposition/README.md)
pytest tests/mission/do_reposition/ -v --log-cli-level=INFO
```

### Verbose output with protocol logging

```bash
pytest tests/ -v --log-cli-level=INFO
```

## Mission plan files

Plans are stored as JSON in `tests/mission/plans/`.
Each file contains:

- `mission_type`: integer matching `MAV_MISSION_TYPE` (0=mission, 1=fence, 2=rally)
- `items`: list of MISSION_ITEM_INT fields

Fields with leading underscores (`_comment`, `_description`, etc.) are documentation-only and ignored by the loader.

| File | Contents |
|------|----------|
| `simple_mission.json` | 4-item flight plan: takeoff → 2 waypoints → RTL |
| `simple_geofence.json` | Fence return point + 4-vertex inclusion polygon |
| `simple_rally.json` | 2 rally (emergency landing) points |

Coordinates are centred on **47.3977 N, 8.5456 E** (Zurich, Switzerland) at low altitude; they are safe to use against any simulator.

## Protocol timeouts

These values are defined by the MAVLink mission protocol specification and applied internally by MAVSDK:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `TIMEOUT_INITIAL_RESPONSE` | 1500 ms | Wait for MISSION_COUNT or first MISSION_REQUEST_INT |
| `TIMEOUT_ITEM_RESPONSE` | 250 ms | Per-item wait between MISSION_REQUEST_INT / MISSION_ITEM_INT |
| `MAX_RETRIES` | 5 | Retransmissions before cancellation |

Worst-case transfer time for *N* items: `(N + 1) × MAX_RETRIES × max(TIMEOUT_INITIAL_RESPONSE, TIMEOUT_ITEM_RESPONSE)`.

## Project structure

```
.
├── conftest.py                    # CLI options (--drone-address, --connection-timeout, etc.)
├── pytest.ini                     # asyncio_mode=auto, markers
├── requirements.txt
├── README.md
├── CLAUDE.md                      # Implementation notes for Claude
└── tests/
    ├── conftest.py                # GCS/drone System fixtures; MockFlightStack integration
    ├── mock_flight_stack.py       # MAVLink drone simulator (upload/download/clear/capability)
    └── mission/
        ├── conftest.py            # Plan loaders, comparison helpers, home-slot detection
        ├── plans/
        │   ├── simple_mission.json
        │   ├── simple_geofence.json
        │   └── simple_rally.json
        ├── test_mission_client.py # GCS-side tests (upload, download, capability)
        ├── test_mission_server.py # Drone-side tests (receive, clear, paired)
        ├── test_frame_types.py    # MAV_FRAME support matrix (21 frames × 3 mission types)
        ├── test_protocol_conformance.py  # Normative spec-conformance tests
        ├── nav_takeoff/
        │   ├── test_protocol.py           # NAV_TAKEOFF param acceptance (Tier 1)
        │   └── test_flight.py             # NAV_TAKEOFF execution tests (Tier 2, requires real stack)
        └── do_reposition/
            └── test_protocol.py           # DO_REPOSITION mission-item acceptance (Tier 1 — UNSUPPORTED everywhere; no Tier 2, see README)
```

## Adding new tests

1. Create a new subdirectory under `tests/` (e.g. `tests/heartbeat/`).
2. Add `__init__.py` and a `conftest.py` for any service-specific fixtures.
3. Name client tests `test_<service>_client.py` and server tests `test_<service>_server.py`.
4. Update `CLAUDE.md` with any protocol behaviour notes specific to the new service.

## Known autopilot behaviour

Results below are from the current test suite run against each stack.

### PX4 (mainline, SIH simulator)

| Test group | Result |
|------------|--------|
| Capability | PASS |
| Flight mission upload/download | PASS |
| Flight mission roundtrip | XFAIL — PX4 converts frame on storage |
| Geofence upload/download | PASS |
| Geofence roundtrip | XFAIL — PX4 converts frame=0→5 on storage |
| Rally upload/download | PASS |
| Rally roundtrip | XFAIL — PX4 converts frame=0→5 on storage |
| Clear mission | PASS |
| Protocol conformance (no home slot required) | PASS |
| Frame support — flight/rally | All accepted frames preserve altitude category |
| Frame support — geofence | FAIL for frames 3 and 6 — PX4 stores all geofence items as GLOBAL_INT, losing relative-alt |
| NAV_TAKEOFF param acceptance | See MAV_CMD table below |
| DO_REPOSITION mission-item acceptance | UNSUPPORTED (rejected outright, spec-aligned) — see MAV_CMD table below |

**Accepted frames (flight and rally):** GLOBAL (0), GLOBAL_RELATIVE_ALT (3), GLOBAL_INT (5), GLOBAL_RELATIVE_ALT_INT (6); others REJECTED.
MAV_FRAME_MISSION accepted for DO commands only.
Geofence shares the same accepted set but frames 3 and 6 are stored incorrectly (see above).

### ArduCopter (V4.8.0-dev, SITL)

| Test group | Result |
|------------|--------|
| Capability | PASS |
| Flight mission upload/download | PASS |
| Flight mission roundtrip | XFAIL — ArduCopter converts all frames to GLOBAL on storage |
| Geofence upload/download | PASS |
| Geofence roundtrip | PASS — ArduCopter preserves frame |
| Rally upload/download | PASS |
| Rally roundtrip | PASS — ArduCopter preserves frame |
| Clear mission | FAIL — ArduCopter retains home waypoint after clear |
| Protocol conformance (no home slot required) | FAIL — ArduCopter requires home at seq=0 (spec violation) |
| MAV_FRAME_MISSION DO_CHANGE_SPEED param1 | FAIL — ArduCopter zeroes param1 (spec violation) |
| Frame support — flight/geofence | See frame table; frames 3/6/10/11 change altitude category → FAIL |
| Frame support — rally | All accepted frames preserve altitude category |
| NAV_TAKEOFF param acceptance | See MAV_CMD table below |
| DO_REPOSITION mission-item acceptance | UNSUPPORTED (rejected outright, spec-aligned) — see MAV_CMD table below |

**Home-slot requirement:** ArduCopter reserves seq=0 for the home position.
The test suite auto-detects this and prepends a home item (using `--home-lat`/`--home-lon`/`--home-alt`) so frame tests can probe real frame support rather than hitting the home-slot rejection.

### MAV_CMD protocol acceptance (Tier 1)

Round-trip evidence is asymmetric: a param that **is** preserved on download was stored correctly for that specific test value, but does not confirm the autopilot acts on it during execution.
A param that is **not** preserved was silently altered; the stack should have NACKed instead.

#### MAV_CMD_NAV_TAKEOFF (cmd=22)

Results confirmed across all tested vehicle types.
Within each firmware family the result is identical regardless of vehicle type (multicopter / fixed-wing / VTOL).

| Param | Label | PX4 (all vehicle types) | ArduCopter | ArduPlane / QuadPlane | Mock |
|-------|-------|-------------------------|------------|-----------------------|------|
| command accepted | — | PASS | PASS | PASS | PASS |
| param1 | Pitch | **zeroed** (not stored) | PRESERVED | PRESERVED | PRESERVED |
| param2 | unused/empty (NaN) | ACCEPTED | **FAIL: NaN rejected** (spec violation) | **FAIL: NaN rejected** (spec violation) | ACCEPTED |
| param3 | Flags (NAV_TAKEOFF_FLAGS) | **zeroed** (not stored) | **zeroed** (not stored) | **zeroed** (not stored) | PRESERVED |
| param4 | Yaw (specific) | PRESERVED | **zeroed** (not stored) | **zeroed** (not stored) | PRESERVED |
| param4 | Yaw (NaN = current heading) | PRESERVED | **zeroed** (not stored) | **zeroed** (not stored) | PRESERVED |
| params 5/6/7 | Lat/Lon/Alt | PRESERVED | PRESERVED | PRESERVED | PRESERVED |
| params 5/6 | Lat/Lon = INT32_MAX ("use current pos") | PRESERVED | **FAIL: NACKed** (spec violation) | **FAIL: NACKed** (spec violation) | PRESERVED |

**PX4 (multicopter, fixed-wing, VTOL — all identical):** Stores param4 (Yaw) and location; silently zeroes param1 (Pitch) and param3 (Flags).
Non-NaN values uploaded in param2 are also silently zeroed.
Correct behaviour: NACK if defined params cannot be stored faithfully.

**ArduCopter:** `AP_Mission::mavlink_int_to_mission_cmd` stores only `param1` for NAV_TAKEOFF; params 3 and 4 are discarded.
`sanity_check_params` disallows NaN for params 1–3 (`nan_mask = ~(1<<3)`), so param2=NaN is rejected with `MAV_MISSION_INVALID_PARAM2` despite being unused.
Workaround: use 0.0 for param2 in all other tests.
Home item at seq=0 required (see protocol conformance).

**ArduPlane / QuadPlane:** Identical storage behaviour to ArduCopter for NAV_TAKEOFF: stores param1, zeroes params 3 and 4, rejects NaN for param2.
No home-item requirement (ArduPlane does not reserve seq=0).

#### MAV_CMD_DO_REPOSITION (cmd=192)

Unlike NAV_TAKEOFF, this command is **rejected outright as a mission item** — `MAV_MISSION_UNSUPPORTED` → MAVSDK `UNSUPPORTED` — on **every** stack and vehicle/frame type tested.
This is **spec-aligned, not a violation**: the spec says outright "This command is intended for guided commands (for missions use MAV_CMD_NAV_WAYPOINT instead)".
Both stacks' source confirms the command is simply absent from their mission-item recognition switch (`mavlink_mission.cpp` for PX4, `AP_Mission::mavlink_int_to_mission_cmd()` for ArduPilot — see `tests/mission/do_reposition/README.md` for the full source-verification writeup).

| | PX4 (MC/FW/VTOL) | ArduCopter | ArduPlane FW / QuadPlane | Mock |
|---|------------------|------------|---------------------------|------|
| `test_protocol_command_accepted` (baseline) | NACKed: **UNSUPPORTED** | NACKed: **UNSUPPORTED** | NACKed: **UNSUPPORTED** | ACCEPTED |
| 21 param-level tests | all **SKIPPED** (command rejected — probing is moot) | all **SKIPPED** | all **SKIPPED** | all PASS |

Because the upload itself is rejected everywhere, *zero* params "passed" Tier 1 and **no Tier 2 flight test is possible or exists** — there is no mission containing a DO_REPOSITION item to fly.
The command's actual execution semantics (does the vehicle reposition at the commanded speed/location/yaw, does `CHANGE_MODE` switch to guided/hold mode, mode-dependent ACK behaviour, …) are properly exercised via **COMMAND_INT** in `tests/command/do_reposition/` — see that directory's README for results.
Result is identical and frame-independent across PX4 MC/FW/VTOL and across ArduCopter/ArduPlane FW/QuadPlane (neither stack's mission-command switch branches on vehicle type).

### ArduPlane / QuadPlane (V4.8.0-dev, SITL)

ArduPlane (fixed-wing) and QuadPlane (VTOL) use the same `arduplane` binary; results are identical for both vehicle types.

| Test group | Result |
|------------|--------|
| NAV_TAKEOFF param acceptance | See MAV_CMD table above |
| DO_REPOSITION mission-item acceptance | See MAV_CMD table above — UNSUPPORTED, identical to ArduCopter |

ArduPlane does **not** require a home item at seq=0 (unlike ArduCopter).
NAV_TAKEOFF storage behaviour mirrors ArduCopter: param1 (Pitch) preserved; params 3 and 4 zeroed; param2 NaN rejected (spec violation).

### Mock (MockFlightStack, no external drone)

161 of 214 tests pass in mock mode (50 skip — Tier 2/stack-specific probes that require a real flight stack; 3 xfail).
The mock accepts every command and frame, stores items exactly as received, and serves them unchanged on download.
Use it to verify protocol-level interactions without a real autopilot.

## Spec violation summary

These are confirmed deviations from the MAVLink mission protocol specification, identified by hard FAIL tests.
Each entry names the failing test and the spec clause violated.

### PX4

| Violation | Failing test | Notes |
|-----------|-------------|-------|
| Geofence items uploaded with `MAV_FRAME_GLOBAL_RELATIVE_ALT` or `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` are stored as `MAV_FRAME_GLOBAL_INT`, changing the altitude reference without adjusting `z` | `TestGeofenceFrames::test_frame[3-global-rel-alt]`, `test_frame[6-global-rel-alt-int]` | PX4 bug: `altitude_is_relative` not stored in `mission_fence_point_s`; always re-encoded as absolute on download.  Flight and rally are unaffected. |
| `MAV_CMD_NAV_TAKEOFF`: `param1` (Pitch) silently zeroed on download | `TestNavTakeoff::test_protocol_param1_pitch_preserved` | PX4 does not store the Pitch angle for NAV_TAKEOFF; it should NACK if a defined param cannot be stored faithfully. |
| `MAV_CMD_NAV_TAKEOFF`: `param3` (Flags / `NAV_TAKEOFF_FLAGS`) silently zeroed on download | `TestNavTakeoff::test_protocol_param3_flags_preserved` | PX4 does not store the flags bitmask for NAV_TAKEOFF; same issue as param1. |

### ArduCopter

| Violation | Failing test | Notes |
|-----------|-------------|-------|
| Flight mission upload with items starting at seq=0 (no home item) is rejected | `TestMissionSlotSemantics::test_seq0_item_accepted_without_home` | The MAVLink mission protocol does not require a home item at seq=0.  ArduCopter treats seq=0 as reserved for the home position and returns `TOO_MANY_MISSION_ITEMS`. |
| `clear_mission()` does not produce an empty mission — home waypoint at seq=0 is retained | `TestFlightMission::test_clear_flight_mission` | After `MISSION_CLEAR_ALL`, ArduCopter returns a 1-item list containing the home waypoint.  The spec requires an empty list. |
| Flight mission items uploaded with `MAV_FRAME_GLOBAL_RELATIVE_ALT`, `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`, `MAV_FRAME_GLOBAL_TERRAIN_ALT`, or `MAV_FRAME_GLOBAL_TERRAIN_ALT_INT` are stored as `MAV_FRAME_GLOBAL`, changing altitude reference without adjusting `z` | `TestFlightMissionFrames::test_frame[3-global-rel-alt]`, `test_frame[6-global-rel-alt-int]`, `test_frame[10-terrain-alt]`, `test_frame[11-terrain-alt-int]` | ArduCopter normalises all flight mission items to `MAV_FRAME_GLOBAL` on storage.  Only INT-encoding changes within the same altitude category are permitted. |
| Geofence items uploaded with `MAV_FRAME_GLOBAL_RELATIVE_ALT`, `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`, `MAV_FRAME_GLOBAL_TERRAIN_ALT`, or `MAV_FRAME_GLOBAL_TERRAIN_ALT_INT` are stored as `MAV_FRAME_GLOBAL`, changing altitude reference without adjusting `z` | `TestGeofenceFrames::test_frame[3-global-rel-alt]`, `test_frame[6-global-rel-alt-int]`, `test_frame[10-terrain-alt]`, `test_frame[11-terrain-alt-int]` | Same normalisation as flight missions.  `MAV_FRAME_GLOBAL` and `MAV_FRAME_GLOBAL_INT` are unaffected (same altitude category). |

#### MAV_CMD specific issues

| Violation | Failing test | Notes |
|-----------|-------------|-------|
| `MAV_CMD_DO_CHANGE_SPEED` with `MAV_FRAME_MISSION`: `param1` (Speed Type) is silently zeroed on storage instead of NACKing | `TestMissionFrame::test_mission_frame_with_do_command` | ArduCopter accepts the upload but corrupts `param1` (Speed Type).  The correct behaviour is to NACK with a mission error if the command cannot be stored faithfully.  PX4 preserves the value correctly. |
| `MAV_CMD_NAV_TAKEOFF`: `param2` (unused/empty) rejects `NaN` with `MAV_MISSION_INVALID_PARAM2` | `TestNavTakeoff::test_protocol_param2_unused` | The MAVLink spec marks param2 as "empty"; unused float params must accept NaN.  ArduCopter's `sanity_check_params` applies `nan_mask = ~(1<<3)`, forbidding NaN for params 1–3.  Workaround: use `0.0` for param2. |
| `MAV_CMD_NAV_TAKEOFF`: params 3, 4 silently zeroed on download (not stored) — ArduCopter | `TestNavTakeoff::test_protocol_param3_flags_preserved`, `test_protocol_param4_yaw_specific`, `test_protocol_param4_yaw_nan` | `AP_Mission::mavlink_int_to_mission_cmd` stores only `param1`; correct behaviour is to NACK if defined params cannot be stored faithfully. |
| `MAV_CMD_NAV_TAKEOFF`: `INT32_MAX` lat/lon ("use current position" sentinel) NACKed — ArduCopter | `TestNavTakeoff::test_protocol_location_current_position` | The MAVLink spec requires `hasLocation` commands to accept `INT32_MAX` as "use current position".  ArduCopter rejects the upload with `MAV_MISSION_INVALID_PARAM5_6`. |
| `MAV_CMD_NAV_TAKEOFF`: `param2` (unused/empty) rejects `NaN` — ArduPlane / QuadPlane | `TestNavTakeoff::test_protocol_param2_unused` | Same `sanity_check_params` rejection as ArduCopter; applies to both plane and quadplane vehicle types. |
| `MAV_CMD_NAV_TAKEOFF`: params 3, 4 silently zeroed on download (not stored) — ArduPlane / QuadPlane | `TestNavTakeoff::test_protocol_param3_flags_preserved`, `test_protocol_param4_yaw_specific`, `test_protocol_param4_yaw_nan` | Same storage pattern as ArduCopter: only `param1` stored; yaw and flags discarded. |
| `MAV_CMD_NAV_TAKEOFF`: `INT32_MAX` lat/lon ("use current position" sentinel) NACKed — ArduPlane / QuadPlane | `TestNavTakeoff::test_protocol_location_current_position` | Same rejection as ArduCopter; applies to both plane and quadplane vehicle types. |
