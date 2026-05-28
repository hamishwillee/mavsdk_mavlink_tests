# Mission protocol tests

Tests covering the MAVLink mission protocol (upload, download, clear, capability
check) using `mission_raw` (GCS side) and `mission_raw_server` (drone side).

## Running

```bash
# Paired mode — no autopilot needed
pytest tests/

# Standalone — PX4 SITL
pytest tests/ --drone-address=udp://:14540

# Standalone — ArduCopter SITL (pre-built binary on TCP port 5760)
pytest tests/ \
  --drone-address=tcp://127.0.0.1:5760 \
  --connection-timeout=60 \
  --ardupilot-sitl=~/ardu_sitl/arducopter \
  --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0
```

## Specification clarifications

### Acceptable frame changes

When a message is sent as `MISSION_ITEM_INT`, a flight stack may change a frame
to its INT-encoding equivalent without altering mission behaviour. These are the
only acceptable frame changes on download:

| Uploaded | Downloaded | Acceptable? |
|----------|------------|-------------|
| GLOBAL (0) | GLOBAL_INT (5) | Yes — same absolute altitude, INT encoding |
| GLOBAL_INT (5) | GLOBAL (0) | Yes — same absolute altitude |
| GLOBAL_RELATIVE_ALT (3) | GLOBAL_RELATIVE_ALT_INT (6) | Yes — same relative altitude, INT encoding |
| GLOBAL_RELATIVE_ALT_INT (6) | GLOBAL_RELATIVE_ALT (3) | Yes — same relative altitude |
| GLOBAL_TERRAIN_ALT (10) | GLOBAL_TERRAIN_ALT_INT (11) | Yes — same terrain altitude, INT encoding |
| GLOBAL_TERRAIN_ALT_INT (11) | GLOBAL_TERRAIN_ALT (10) | Yes — same terrain altitude |
| Any relative/terrain | Any absolute | **No** — changes what altitude the autopilot flies |
| Any absolute | Any relative/terrain | **No** — changes what altitude the autopilot flies |

Any cross-category change is a **test FAIL**.

## Autopilot comparison

Differences observed between PX4 and ArduCopter via the test suite.

PX4 tested against mainline branch `mission_request_returns_int` with SIH simulator.
ArduCopter tested against V4.8.0-dev (70fe7125) pre-built SITL.

### Connection

| Behaviour | PX4 | ArduCopter |
|-----------|-----|------------|
| SITL connection | UDP `:14540` | TCP `127.0.0.1:5760` |
| AUTOPILOT_VERSION request | `MAV_CMD_REQUEST_MESSAGE` (512), param1=148 | `MAV_CMD_REQUEST_MESSAGE` (512), param1=148 |
| `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` (520) | Ignored | Not tested |

### Mission: Frame test (flight plans, mission_type=0)

Each row tests a single `MAV_FRAME_*` value. The test uploads one `NAV_WAYPOINT`
item with that frame as `MISSION_ITEM_INT`, then downloads the mission and
compares the returned frame and data.

Column meanings:
- **Rejected (reason)** — the drone refused the upload with the indicated error; the item was never stored.
- **Accepted (preserved)** — upload accepted; downloaded frame matches uploaded frame.
- **Accepted (→ X, INT upgrade/downgrade)** — upload accepted; downloaded frame differs only by INT-encoding within the same altitude category. Altitude semantics are unchanged — this is acceptable.
- **FAIL — stored as X** — upload accepted but the downloaded frame has a different altitude category. The drone will fly a different altitude than specified.

| Frame | PX4 | ArduCopter |
|-------|-----|------------|
| `MAV_FRAME_GLOBAL` | Accepted (→ `GLOBAL_INT`, acceptable INT upgrade) | Accepted (preserved — with home-slot prepend; see note) |
| `MAV_FRAME_LOCAL_NED` | Rejected | Rejected |
| `MAV_FRAME_GLOBAL_RELATIVE_ALT` | Accepted (→ `GLOBAL_RELATIVE_ALT_INT`, acceptable INT upgrade) | **FAIL** — stored as `GLOBAL`, relative-alt lost |
| `MAV_FRAME_LOCAL_ENU` (deprecated) | Rejected | Rejected |
| `MAV_FRAME_GLOBAL_INT` | Accepted (preserved) | Accepted (→ `GLOBAL`, acceptable INT downgrade) |
| `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` | Accepted (preserved) | **FAIL** — stored as `GLOBAL`, relative-alt lost |
| `MAV_FRAME_LOCAL_OFFSET_NED`, `MAV_FRAME_BODY_NED`, `MAV_FRAME_BODY_OFFSET_NED`, `MAV_FRAME_BODY_FRD`, `MAV_FRAME_LOCAL_FRD`, `MAV_FRAME_LOCAL_FLU` | Rejected | Rejected |
| `MAV_FRAME_GLOBAL_TERRAIN_ALT` | Rejected | **FAIL** — stored as `GLOBAL`, terrain info lost |
| `MAV_FRAME_GLOBAL_TERRAIN_ALT_INT` | Rejected | **FAIL** — stored as `GLOBAL`, terrain info lost |
| `MAV_FRAME_RESERVED_13` through `MAV_FRAME_RESERVED_19` | Rejected | Rejected |

Notes:
- PX4 terrain frame support (`MAV_FRAME_GLOBAL_TERRAIN_ALT` / `MAV_FRAME_GLOBAL_TERRAIN_ALT_INT`): **No** — hard whitelist in `mavlink_mission.cpp:1412–1415` covers only `MAV_FRAME_GLOBAL`, `MAV_FRAME_GLOBAL_RELATIVE_ALT`, `MAV_FRAME_GLOBAL_INT`, and `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`.
- ArduCopter stores all accepted flight mission frames that are not in the `{GLOBAL, GLOBAL_INT}` category as `MAV_FRAME_GLOBAL`, discarding the altitude reference.
- ArduCopter home-slot requirement: ArduCopter reserves seq=0 for the home position and rejects any single-item upload that does not provide it. The test suite auto-detects this and prepends a home item at seq=0 when `--home-lat/lon/alt` are supplied. Without the prepend, all frames would be rejected with `TOO_MANY_MISSION_ITEMS` before the frame is even checked.

### Geofence: Frame test (mission_type=1)

Each row tests a single `MAV_FRAME_*` value. The test uploads one fence return-point
item with that frame as `MISSION_ITEM_INT`, then downloads the geofence and
compares the returned frame and data. Column meanings are the same as the mission table above.

| Frame | PX4 | ArduCopter |
|-------|-----|------------|
| `MAV_FRAME_GLOBAL` | Accepted (→ `GLOBAL_INT`, acceptable INT upgrade) | Accepted (preserved) |
| `MAV_FRAME_LOCAL_NED` | Rejected | Rejected |
| `MAV_FRAME_GLOBAL_RELATIVE_ALT` | **FAIL** — stored as `GLOBAL_INT`, relative-alt lost | **FAIL** — stored as `GLOBAL`, relative-alt lost |
| `MAV_FRAME_LOCAL_ENU` (deprecated) | Rejected | Rejected |
| `MAV_FRAME_GLOBAL_INT` | Accepted (preserved) | Accepted (→ `GLOBAL`, acceptable INT downgrade) |
| `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` | **FAIL** — stored as `GLOBAL_INT`, relative-alt lost | **FAIL** — stored as `GLOBAL`, relative-alt lost |
| `MAV_FRAME_LOCAL_OFFSET_NED`, `MAV_FRAME_BODY_NED`, `MAV_FRAME_BODY_OFFSET_NED`, `MAV_FRAME_BODY_FRD`, `MAV_FRAME_LOCAL_FRD`, `MAV_FRAME_LOCAL_FLU` | Rejected | Rejected |
| `MAV_FRAME_GLOBAL_TERRAIN_ALT` | Rejected | **FAIL** — stored as `GLOBAL`, terrain info lost |
| `MAV_FRAME_GLOBAL_TERRAIN_ALT_INT` | Rejected | **FAIL** — stored as `GLOBAL`, terrain info lost |
| `MAV_FRAME_RESERVED_13` through `MAV_FRAME_RESERVED_19` | Rejected | Rejected |

Notes:
- PX4 source (`mavlink_mission.cpp:1201–1221`): the fence point `frame` field is stored, but
  `altitude_is_relative` is not stored in `mission_fence_point_s`. On re-encoding
  (`format_mavlink_mission_item:1772–1787`), `altitude_is_relative` defaults to `false`, so all
  geofence items are returned as `MAV_FRAME_GLOBAL_INT` regardless of the uploaded frame. This is a bug.
- ArduCopter stores all accepted geofence items as `MAV_FRAME_GLOBAL`, discarding altitude type.

### Rally points: Frame test (mission_type=2)

Each row tests a single `MAV_FRAME_*` value. The test uploads one rally-point
item with that frame as `MISSION_ITEM_INT`, then downloads the rally points and
compares the returned frame and data. Column meanings are the same as the mission table above.

| Frame | PX4 | ArduCopter |
|-------|-----|------------|
| `MAV_FRAME_GLOBAL` | Accepted (→ `GLOBAL_INT`, acceptable INT upgrade) | Accepted (preserved) |
| `MAV_FRAME_LOCAL_NED` | Rejected | Rejected |
| `MAV_FRAME_GLOBAL_RELATIVE_ALT` | Accepted (→ `GLOBAL_RELATIVE_ALT_INT`, acceptable INT upgrade) | Accepted (preserved) |
| `MAV_FRAME_LOCAL_ENU` (deprecated) | Rejected | Rejected |
| `MAV_FRAME_GLOBAL_INT` | Accepted (preserved) | Accepted (→ `GLOBAL`, acceptable INT downgrade) |
| `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` | Accepted (preserved) | Accepted (→ `GLOBAL_RELATIVE_ALT`, acceptable INT downgrade) |
| `MAV_FRAME_LOCAL_OFFSET_NED`, `MAV_FRAME_BODY_NED`, `MAV_FRAME_BODY_OFFSET_NED`, `MAV_FRAME_BODY_FRD`, `MAV_FRAME_LOCAL_FRD`, `MAV_FRAME_LOCAL_FLU` | Rejected | Rejected |
| `MAV_FRAME_GLOBAL_TERRAIN_ALT` | Rejected | Accepted (preserved) |
| `MAV_FRAME_GLOBAL_TERRAIN_ALT_INT` | Rejected | Accepted (→ `GLOBAL_TERRAIN_ALT`, acceptable INT downgrade) |
| `MAV_FRAME_RESERVED_13` through `MAV_FRAME_RESERVED_19` | Rejected | Rejected |

Notes:
- PX4 rally: correctly preserves the altitude category for all accepted frames. Unlike geofence, PX4 rally storage does retain `altitude_is_relative`, so frames 3 and 6 are returned with their relative-alt category intact (as `GLOBAL_RELATIVE_ALT_INT`).
- ArduCopter rally: correctly preserves frame category for all accepted frames (unique among its three mission types — flight and geofence both lose altitude reference on storage).

### Other protocol differences

| Behaviour | PX4 | ArduCopter |
|-----------|-----|------------|
| `MAV_FRAME_MISSION` + `DO_CHANGE_SPEED` | Accepted, param1 preserved | Accepted, param1 zeroed (protocol violation) |
| `MAV_FRAME_MISSION` + `NAV_WAYPOINT` (misuse) | Rejected | Accepted, raw integer x/y/z stored |
| `clear_mission()` result | Empty | Home waypoint (seq=0) retained |
| Geofence command values | 5000-based (RETURN_POINT=5000, INCLUSION=5001) | 5000-based (same) |
| Home slot required for flight missions | No | Yes — seq=0 must carry home position (spec violation) |
| Test suite result | 79 passed, 2 failed, 1 skipped, 3 xfailed | 73 passed, 10 failed, 1 skipped, 1 xfailed |
