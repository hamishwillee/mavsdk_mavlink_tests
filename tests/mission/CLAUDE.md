# Mission protocol — implementation notes

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

`MAV_PROTOCOL_CAPABILITY_MISSION_INT = 4` (bit 2 of the AUTOPILOT_VERSION.capabilities field).

The check in `TestCapability.test_mission_int_capability` uses `mavlink_direct` to send `MAV_CMD_REQUEST_MESSAGE` (512) with `param1=148.0` (AUTOPILOT_VERSION message ID), then reads the `capabilities` field from the `AUTOPILOT_VERSION` response.

Note: `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` (520) is NOT used — PX4 does not support command 520 and logs "command 520 unsupported".
PX4 also does not passively broadcast AUTOPILOT_VERSION, so the request is always required.

If the autopilot does not set this bit, `mission_raw` returns `MissionRawResult.Result.INT_MESSAGES_NOT_SUPPORTED`.

### Deprecated message handling

The original (pre-MAVLink 2) upload path used:
- MISSION_REQUEST (deprecated) instead of MISSION_REQUEST_INT
- MISSION_ITEM (deprecated) instead of MISSION_ITEM_INT

The spec requires modern GCS implementations to still handle MISSION_REQUEST by completing the upload (responding with MISSION_ITEM_INT).
MAVSDK does this transparently.

Tests:
- `TestDeprecatedMessageHandling.test_deprecated_request_yields_int_response` (client) — verifies that MISSION_ITEM_INT (not MISSION_ITEM) is used in the GCS response during a paired test.
- `TestDeprecatedRequestHandling.test_respond_with_deprecated_request` (server) — confirms that the upload completes even if the server triggers the deprecated path.

If MAVSDK removes this fallback, both tests will fail with `INT_MESSAGES_NOT_SUPPORTED`; update the tests and this file accordingly.

### Mission types (MAV_MISSION_TYPE)

| Value | Name | MAVSDK method pair |
|-------|------|--------------------|
| 0 | MAV_MISSION_TYPE_MISSION | upload: `upload_mission()` / download: `download_mission()` |
| 1 | MAV_MISSION_TYPE_FENCE   | upload: `upload_geofence()` / download: `download_geofence()` |
| 2 | MAV_MISSION_TYPE_RALLY   | upload: `upload_rally_points()` / download: `download_rallypoints()` |

The rally pair has a deliberate spelling asymmetry in the MAVSDK API: upload is `upload_rally_points` (underscore) but download is `download_rallypoints` (no underscore).
This is the MAVSDK API spelling, not a typo here.

### Clear mission

`mission_raw.clear_mission()` sends MISSION_CLEAR_ALL with **mission_type=0** (flight missions only — empirically verified against PX4).
It does **not** clear geofence (type=1) or rally points (type=2).
To clear all types, send raw MISSION_CLEAR_ALL messages for types 1 and 2 via `mavlink_direct`; see `clear_all_mission_types()` in `tests/mission/conftest.py`.

`upload_mission([])` / `upload_geofence([])` / `upload_rally_points([])` all raise `NO_MISSION_AVAILABLE` — they are not equivalent to clearing and must not be used to clear stored missions.

### Coordinate encoding (MISSION_ITEM_INT)

- `x` = latitude  × 1e7  (int32_t)
- `y` = longitude × 1e7  (int32_t)
- `z` = altitude in metres (float)
- `frame=5` (MAV_FRAME_GLOBAL_INT) — altitude absolute (AMSL); correct frame for MISSION_ITEM_INT with int32 lat/lon
- `frame=6` (MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) — altitude relative to takeoff; INT variant for MISSION_ITEM_INT
- `frame=0` (MAV_FRAME_GLOBAL) — nominally float lat/lon, but PX4 handles this in MISSION_ITEM_INT correctly (see below)
- `frame=3` (MAV_FRAME_GLOBAL_RELATIVE_ALT) — nominally float lat/lon; PX4 also handles this in MISSION_ITEM_INT

**Frame and int_mode in PX4:** PX4 tracks whether the exchange uses MISSION_ITEM_INT via an internal `_int_mode` flag, set to `true` when `MISSION_ITEM_INT` is received (regardless of frame).
On receive, if `_int_mode=true`, x/y are decoded as `int32×1e-7` even if the frame field is 0 (GLOBAL) or 3 (GLOBAL_RELATIVE_ALT).
On send, frame is upgraded to the INT variant (0→5, 3→6) in the response.
So frame=0 in uploaded items is accepted and stored correctly, but PX4 always returns frame=5 on download.
Using frame=5/6 in upload items is more spec-correct and also causes the roundtrip frame check to succeed.

### Float comparison

Autopilots (especially ArduPilot) round float parameters on storage.
The `items_match()` helper in `tests/mission/conftest.py` uses `tol=1e-4`.
If new tests encounter false failures due to rounding, increase tolerance and update this note.

---

## MAV_CMD support testing

### Two-tier testing model

**Tier 1 — Protocol acceptance** (implemented in `tests/mission/test_cmd_*.py`)

Tests whether the mission protocol accepts a command and stores its parameters faithfully.
Does *not* prove the autopilot executes the command.

Results are recorded per **(autopilot, vehicle type)** pair — a command that works on a multicopter may be unsupported or behave differently on fixed-wing or VTOL.
Current coverage: multicopter only (ArduCopter SITL, PX4 SIH quadrotor).
When fixed-wing/VTOL stacks are added, results go in separate columns in the README.

| Result | Meaning |
|--------|---------|
| **Not accepted** | Upload rejected (`MissionRawError`) for the baseline valid item |
| **Accepted (fully)** | Upload accepted; all defined params round-trip correctly — does not confirm execution |
| **Accepted (partially — Param Label)** | Upload accepted; listed defined params are corrupted or zeroed on download |

A **NOTE** is appended when an unused param silently accepts a non-NaN value instead of NACKing.

Round-trip testing is asymmetric evidence:
- Param **not** preserved → stack silently altered it; it should have NACKed if the value is unsupported.
  Whether another value for the same param would be accepted is not determined by the single test.
- Param **is** preserved → the specific test value was stored correctly; does not confirm the param is actually used at execution time.

**Tier 2 — Execution verification** (implemented for NAV_TAKEOFF in `tests/mission/test_flight.py`; future work for other commands — requires active mission + telemetry)

All commands need telemetry evidence to confirm execution.
Two complementary signals:
1. **Command-specific telemetry** — expected state change observed (altitude after TAKEOFF, etc.).
2. **STATUSTEXT** (MAVLink msg 253) — autopilots often emit a human-readable reason when a command cannot be executed or a parameter is rejected; monitor alongside telemetry even when the primary check is state-based.

Results per (autopilot, vehicle type): **Supported (fully)** or **Supported (partially)**.

### Parameter conventions in test items (MISSION_ITEM_INT)

| Param position | Default / unused value |
|---------------|------------------------|
| Float params 1, 2, 4, 7 | `float('nan')` (spec-correct) |
| Integer location params 5, 6 | `INT32_MAX = 0x7FFF_FFFF` |

The baseline item must use these for every param that is not under test.

**Note on stack compatibility:** Some stacks (ArduCopter) reject NaN for float params that are unused in a specific command, even though the MAVLink spec requires unused params to accept NaN.
When this happens the command's test file documents the workaround (typically `0.0`) in the `_<cmd>_item()` helper, and `test_protocol_param<N>_unused` records the NaN rejection as a spec violation.

### Testing each param category

**Defined float params** — upload with a specific valid test value; assert round-trip within 1e-4.
Test spec-documented special values (e.g., `NaN` = "use current heading") as separate sub-tests.
Also test `NaN` itself (some stacks reject NaN for defined params via sanity_check_params — document whether accepted/altered/rejected; no hard assertion since defined params may legitimately require a value).

**Ranges for defined float params:**
- **Explicit range** (XML `minValue`/`maxValue` present): test one value just below `minValue` and one just above `maxValue`; expect NACK or clamping; NACK is preferred.
- **Implicit range** (no explicit XML bounds but physical meaning implies limits): determine the natural range from the label/units (e.g. yaw in degrees → [0°, 360°), or [−360°, 360°] if negative direction is meaningful; pitch for takeoff → [0°, 90°]).
  Test: a value just above the implicit maximum; a value just below the implicit minimum (if negative is ambiguous); a value well above the maximum (e.g. twice the max — characterises whether clamping or wrap occurs).
  **Observational tests** (no assertion) are appropriate when any outcome is protocol-valid; the goal is to characterise clamp/reject/wrap/zero behaviour.

**Unused params** (spec marks param as "empty" / "reserved") — three-probe pattern:
1. **NaN probe** (spec-correct): upload with `float('nan')`.
   - Accepted → `log.info` "ACCEPTED — NaN preserved as spec requires".
     No failure.
   - Rejected → `log.warning` "FAIL: NaN rejected — spec violation"; set `nan_rejected = True`.
2. **0.0 retry** (only when NaN was rejected): re-upload with `0.0`.
   - Provides diagnostic evidence that the command itself is accepted, just not with NaN.
3. **1.0 probe** (non-zero non-NaN — should ideally be NACKed): always run.
   - `MissionRawError` → `log.info` "correctly NACKed".
   - Accepted, value preserved → `log.warning` "NOTE: silently accepted and preserved".
   - Accepted, value altered → `log.warning` "NOTE: silently altered to X".
4. At the end: `pytest.fail()` if `nan_rejected` (the test as a whole FAILs when NaN was rejected).

**Bitmask params** (XML `bitmask="true"`):
- Test value=0 (no bits set): must always be accepted and round-trip as 0.
- Test each defined bit value individually (one test per bit).
- Test all defined bits combined simultaneously if ≥2 bits are defined (single test with all defined bits OR'd together).
- Test at least one undefined bit value (e.g. the next power of 2 after the highest defined bit).
  Expect NACK or silent discard; NACK is preferred.
  Use an **observational test** (no assertion).

**Enum params** (XML `enum` attribute present):
- Test each defined enum value individually.
- Test one invalid enum value (e.g. a value not in the enum).
  Expect NACK.

**Location params** (`hasLocation="true"` or `isDestination="true"` in the XML):
- `INT32_MAX = 0x7FFF_FFFF` for integer lat/lon fields (params 5, 6) means "use current position".
  Test whether this is accepted and the sentinel is preserved; assert if spec-mandated, otherwise document as observational.
- `NaN` for float altitude (param 7 / z field) means "use current/default altitude".
  Test whether this is accepted; use an observational test (no assertion) since NAV commands may legitimately require an explicit altitude.
- A **zero-value PASS is ambiguous**: uploading `x=0, y=0, z=0` and getting back `0, 0, 0` does not prove storage — the stack may have returned its zero-initialised default.
  Always use non-zero, non-default test values for location assertions.

**Observational tests** (no assertion):
- Use when any outcome is protocol-valid (edge-case values outside the primary use range).
- Upload the item, download it, log the stored value in context.
  The test always PASSes.
- In the log, distinguish outcomes: PRESERVED (raw), NORMALISED (canonical form), ZEROED (matches zero-default — ambiguous if param may not be stored at all), ALTERED (other value), NACKed (upload rejected).
- **Vacuous PASS**: annotate when a PASS on a zero-valued probe is trivially derived from the param not being stored at all.
  Example: a stack that never stores param4 (yaw) will also return 0.0 for `param4=0.0`, which is indistinguishable from "north correctly stored".
  Mark these explicitly: "PASS (vacuous — param4 not stored; 0.0 is the zero-initialised default)".

### Home-slot prepend

For ArduCopter (requires home at seq=0), use `home_item_for_mission` from `tests/mission/conftest.py`.
When not None, prepend it as seq=0 and renumber the probe item to seq=1.
Always find the probe item by `seq` on download, not by list index.

### Conditional Tier 2 pattern

For edge-case params (negative yaw, overflow yaw, negative pitch, etc.) Tier 1 alone is **ambiguous**: if the value is stored raw, the autopilot deferred interpretation to execution time and a Tier 2 test is required.
If already normalised, execution is unambiguous.

A Tier 2 test begins with an **inline Tier 1 probe** (upload, download, read stored value) and skips based on the outcome:

| Stored value | Meaning | Tier 2 action |
|---|---|---|
| `N` (normalised) | Stack committed to canonical form | **Skip** — execution unambiguous |
| `V` (raw) | Stack deferred normalisation | **Proceed** — verify execution |
| Other value | Altered (zeroed etc.) | **Skip** — documented by Tier 1 |
| NACKed | Upload rejected | **Skip** — captured by Tier 1 |

`_probe_takeoff_item(system, home_item, **overrides)` in `test_flight.py` performs the upload–download–clear cycle without flying.

### File and class naming

**Tier 1 (protocol acceptance):**
- Directory: `tests/mission/<lower_snake_name>/`
- File: `test_protocol.py`
- Class: `Test<CamelCaseName>` with class-scoped fixtures (same pattern as `test_frame_types.py`).
- Log format: `_FMT = "%-14s | %-40s | %s"` with fields `(CMD_NAME, param_label, outcome)`.

**Tier 2 (execution verification):**
- File: `test_flight.py` (in the same command subdirectory)
- Standalone async functions (no class) with function-scoped `gcs_system` fixture.
- `autouse` fixture `require_real_stack` skips all tests when `--drone-address` is absent.
- Cleanup (RTL + land + `clear_all_mission_types`) always in `finally` block.
- Use `system.telemetry.home()` to get the vehicle's live home position rather than assuming SIH defaults — ensures test works against any simulator home location.
- `_build_mission(home_item_for_mission, *probes)` handles ArduCopter home-slot prepend.
- For edge-case params: use the **conditional Tier 2 pattern** (inline probe + skip decision) described in the section above.

---

## Autopilot-specific behaviour (PX4) — mission protocol

Tested against PX4 mainline branch `mission_request_returns_int` with SIH simulator (`PX4_SIM_MODEL=sihsim_quadx`), default SIH home (47.397742°N, 8.545594°E).

### Coordinate frame conversion (flight missions)

PX4 accepts `MAV_FRAME_GLOBAL_RELATIVE_ALT` (frame=3) uploads but stores waypoints in local NED (frame=6) internally.
On download, items are returned with frame=6 and x≈0, y≈0 because the test coordinates are centred on the SIH home position.
The roundtrip test detects this frame change and calls `pytest.xfail()` rather than failing hard.

**Impact:** Field-by-field roundtrip comparison is not possible with PX4 without frame-aware coordinate conversion.

### Geofence command values (PX4 vs current MAVLink spec)

PX4 and pymavlink use **5000-based** fence command values, while the current mavlink.io online spec lists **5001-based** values.
The PX4 bundled MAVLink XML (`src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml`) defines:

| Command | PX4/pymavlink value | mavlink.io (current) |
|---------|--------------------|-----------------------|
| FENCE_RETURN_POINT | **5000** | 5001 |
| FENCE_POLYGON_VERTEX_INCLUSION | **5001** | 5002 |
| FENCE_POLYGON_VERTEX_EXCLUSION | **5002** | 5003 |

The `simple_geofence.json` plan and all geofence tests use the PX4/pymavlink values (5000, 5001, …).
If you send the mavlink.io values (5001 for return point), PX4 parses cmd=5001 as `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION`, computes `vertex_count = param1 + 0.5 = 0`, and immediately rejects with `MAV_MISSION_ERROR` ("Fence: too few vertices") — this was the root cause of the earlier geofence upload failures.

### Geofence coordinate frame (PX4)

PX4 accepts geofence items with `frame=0` (MAV_FRAME_GLOBAL) but stores them as `frame=5` (MAV_FRAME_GLOBAL_INT) internally.
On download, all items are returned with frame=5; coordinates (x, y, z) are preserved unchanged.
The roundtrip test detects the frame change and calls `pytest.xfail()`.

**Impact:** Field-by-field roundtrip comparison is not possible with PX4 without frame-aware comparison, but upload/download of fence items works correctly (coordinates are preserved).

### Rally points

PX4 accepts `upload_rally_points()` and stores rally items correctly.
`download_rallypoints()` returns items with frame upgraded from 0 (MAV_FRAME_GLOBAL) to 5 (MAV_FRAME_GLOBAL_INT) but coordinates (x, y, z) are preserved unchanged.
The roundtrip test detects the frame change (0→5) and calls `pytest.xfail()`.
Unlike geofence, PX4 rally correctly preserves the altitude category for frames 3 and 6 — both are returned as GLOBAL_RELATIVE_ALT_INT (frame=6), not GLOBAL_INT.

Note: if PX4 is started with a stale `dataman` file from a previous session, the dataman slot alternation can cause the download to read from the wrong slot, returning x=0, y=0.
Starting with a clean `dataman` file (delete or reset between sessions) ensures correct behaviour.
In the test suite the PX4 process is session-scoped so the dataman state persists within a session; the `test_upload_rally_points` test runs before `test_roundtrip_rally_points` and leaves the slot in the correct state.

### Frame type support (PX4)

Determined by `test_frame_types.py` against PX4 SIH.

**Flight missions (mission_type=0) and Rally (type=2):**

| Frame | Name                          | PX4 result                                  |
|-------|-------------------------------|---------------------------------------------|
| 0     | MAV_FRAME_GLOBAL              | ACCEPTED; downloaded as frame=5 (INT upgrade) |
| 1     | MAV_FRAME_LOCAL_NED           | REJECTED (UNSUPPORTED)                      |
| 3     | MAV_FRAME_GLOBAL_RELATIVE_ALT | ACCEPTED; downloaded as frame=6 (INT upgrade, relative-alt preserved) |
| 4     | MAV_FRAME_LOCAL_ENU [dep]     | REJECTED (UNSUPPORTED)                      |
| 5     | MAV_FRAME_GLOBAL_INT          | ACCEPTED; frame preserved on download        |
| 6     | MAV_FRAME_GLOBAL_RELATIVE_ALT_INT | ACCEPTED; frame preserved on download    |
| 7–12  | LOCAL/BODY frames             | REJECTED (UNSUPPORTED)                      |
| 13–19 | RESERVED                      | REJECTED (UNSUPPORTED)                      |
| 20–21 | LOCAL_FRD / LOCAL_FLU         | REJECTED (UNSUPPORTED)                      |

**Geofence (type=1):** Same as above except frames 3 and 6 **FAIL** — PX4 stores all geofence items as `GLOBAL_INT` (frame=5) regardless of the uploaded frame, losing the altitude reference.
See "Geofence coordinate frame (PX4)" section.
Frames 10/11 also REJECTED for geofence (same as flight/rally).

**MAV_FRAME_MISSION (frame=2):**
- `DO_CHANGE_SPEED` (non-location cmd): **ACCEPTED**, param1 preserved unscaled.
- `NAV_WAYPOINT` (location cmd, misuse): **REJECTED** (PX4 correctly refuses MISSION frame for location commands).

Note: the geofence `altitude_is_relative` bug (frames 3/6 → frame=5) does **not** affect flight missions or rally points — PX4 rally storage correctly retains relative-alt and returns frame=6 for both.

---

## Autopilot-specific behaviour (ArduCopter) — mission protocol

Tested against ArduCopter V4.8.0-dev (70fe7125) pre-built SITL (`firmware.ardupilot.org/Copter/latest/SITL_x86_64_linux_gnu/arducopter`), connected via TCP port 5760.
Result: **73 passed, 10 failed, 1 skipped, 1 xfailed** (log: `logs/test_arducopter_20260524_151722.log`).

### clear_mission() retains home waypoint

`clear_mission()` sends `MISSION_CLEAR_ALL(mission_type=0)`.
PX4 returns an empty list on the subsequent download.
ArduCopter retains a home waypoint (seq=0) and returns a 1-item list.
This causes `test_clear_flight_mission` to **fail** because the test asserts an empty download after clear.

The home waypoint is implicitly managed by ArduCopter and cannot be cleared via the mission protocol.
To fix this test for ArduCopter, the assertion would need to allow a single home-waypoint item to remain after clear.

### MAV_FRAME_MISSION with DO_CHANGE_SPEED — param1 corrupted

ArduCopter accepts a `MAV_FRAME_MISSION` + `DO_CHANGE_SPEED` (cmd=178) item but zeroes out `param1` on storage (PX4 preserves it).
Test `test_mission_frame_with_do_command` **fails** because it asserts `param1 ≈ 5.0` after roundtrip but gets `0.0`.

This is an ArduCopter-specific behaviour: MAV_FRAME_MISSION items appear to have non-float parameters discarded.

### Flight mission roundtrip

Flight mission roundtrip xfails (same as PX4): ArduCopter stores items as `MAV_FRAME_GLOBAL` (frame=0) regardless of the uploaded frame (3, 5, 6), and the z value is zeroed (all coordinates become 0.0 because test coords are relative to the home position).

### Geofence and rally roundtrip

Unlike PX4, ArduCopter **preserves** the uploaded frame on geofence and rally roundtrips — both `test_roundtrip_geofence` and `test_roundtrip_rally_points` **pass** (not xfail).

### Frame type support (ArduCopter)

Determined by `test_frame_types.py` against ArduCopter V4.8.0-dev.

**Flight missions (mission_type=0):** *(home-slot prepend active — requires `--home-lat/lon/alt`)*

| Frame | Name                          | ArduCopter result                                                        |
|-------|-------------------------------|--------------------------------------------------------------------------|
| 0     | MAV_FRAME_GLOBAL              | ACCEPTED, frame preserved (z=0.100) — with home-slot prepend            |
| 1     | MAV_FRAME_LOCAL_NED           | REJECTED (UNSUPPORTED)                                                   |
| 3     | MAV_FRAME_GLOBAL_RELATIVE_ALT | ACCEPTED, stored as MAV_FRAME_GLOBAL — altitude category change → FAIL  |
| 4     | MAV_FRAME_LOCAL_ENU [dep]     | REJECTED (UNSUPPORTED)                                                   |
| 5     | MAV_FRAME_GLOBAL_INT          | ACCEPTED, INT-encoded as MAV_FRAME_GLOBAL (same category {0,5}) → PASS  |
| 6     | MAV_FRAME_GLOBAL_RELATIVE_ALT_INT | ACCEPTED, stored as MAV_FRAME_GLOBAL — altitude category change → FAIL |
| 7–12  | LOCAL/BODY frames             | REJECTED (UNSUPPORTED)                                                   |
| 10    | MAV_FRAME_GLOBAL_TERRAIN_ALT  | ACCEPTED, stored as MAV_FRAME_GLOBAL — altitude category change → FAIL  |
| 11    | MAV_FRAME_GLOBAL_TERRAIN_ALT_INT | ACCEPTED, stored as MAV_FRAME_GLOBAL — altitude category change → FAIL |
| 13–19 | RESERVED                      | REJECTED (UNSUPPORTED)                                                   |
| 20–21 | LOCAL_FRD / LOCAL_FLU         | REJECTED (UNSUPPORTED)                                                   |

**Geofence (mission_type=1):**

| Frame | ArduCopter result                                                         |
|-------|---------------------------------------------------------------------------|
| 0     | ACCEPTED, frame preserved (z=0.000)                                       |
| 3     | ACCEPTED, stored as MAV_FRAME_GLOBAL — altitude category change → FAIL    |
| 5     | ACCEPTED, INT-encoded as MAV_FRAME_GLOBAL (same category {0,5}) → PASS   |
| 6     | ACCEPTED, stored as MAV_FRAME_GLOBAL — altitude category change → FAIL    |
| 10    | ACCEPTED, stored as MAV_FRAME_GLOBAL — altitude category change → FAIL    |
| 11    | ACCEPTED, stored as MAV_FRAME_GLOBAL — altitude category change → FAIL    |
| others | REJECTED (UNSUPPORTED)                                                   |

**Rally points (mission_type=2):**

| Frame | ArduCopter result                                                  |
|-------|--------------------------------------------------------------------|
| 0     | ACCEPTED, frame preserved (z=10.000)                               |
| 3     | ACCEPTED, frame preserved (z=10.000)                               |
| 5     | ACCEPTED, stored as MAV_FRAME_GLOBAL (z=10.000)                    |
| 6     | ACCEPTED, stored as MAV_FRAME_GLOBAL_RELATIVE_ALT (z=10.000)       |
| 10    | ACCEPTED, frame preserved (z=10.000)                               |
| 11    | ACCEPTED, stored as MAV_FRAME_GLOBAL_TERRAIN_ALT (z=10.000)        |
| others | REJECTED (UNSUPPORTED)                                            |

**MAV_FRAME_MISSION (frame=2):**
- `DO_CHANGE_SPEED` (non-location cmd): **ACCEPTED but param1 zeroed** (protocol violation — see above).
- `NAV_WAYPOINT` (location cmd, misuse): **ACCEPTED** with coordinates stored unscaled (unlike PX4 which rejects this).

**Key differences vs PX4:**
- ArduCopter accepts `MAV_FRAME_GLOBAL_TERRAIN_ALT` (10) and `MAV_FRAME_GLOBAL_TERRAIN_ALT_INT` (11) for all mission types; PX4 does not.
- ArduCopter's geofence/rally canonical storage frame is `GLOBAL` (frame=0); PX4 uses `GLOBAL_INT` (frame=5).
- ArduCopter requires a home item at seq=0 for flight mission uploads (spec violation — see `test_protocol_conformance.py`); the frame test suite auto-prepends one when `--home-lat/lon/alt` is supplied.
- ArduCopter converts frames 3, 6, 10, 11 to `GLOBAL` (frame=0) on storage for flight missions and geofence — an altitude reference change that is a protocol violation; PX4 preserves these frames correctly.
- Rally point frame=6 (`GLOBAL_RELATIVE_ALT_INT`) is stored as `GLOBAL_RELATIVE_ALT` (frame=3) on ArduCopter; PX4 stores as `GLOBAL_INT` (frame=5).

---

## Autopilot-specific behaviour (ArduPlane fixed-wing) — mission protocol

Tested against ArduPlane V4.8.0-dev (70fe7125, `--model plane`) connected via TCP port 5760.
Log: `logs/mission_frame_types_ardupilot_fixed_wing_4.8.0-dev_20260526.log`.
Result: **59 passed, 6 failed**.

### Frame type support (ArduPlane fixed-wing)

**Flight missions (mission_type=0):** ArduPlane does **not** require a home item at seq=0.

| Frame | Name                              | ArduPlane FW result                                                      |
|-------|-----------------------------------|--------------------------------------------------------------------------|
| 0     | MAV_FRAME_GLOBAL                  | ACCEPTED, frame preserved (z=0.000)                                      |
| 1     | MAV_FRAME_LOCAL_NED               | REJECTED                                                                 |
| 3     | MAV_FRAME_GLOBAL_RELATIVE_ALT     | ACCEPTED, stored as MAV_FRAME_GLOBAL — altitude category change → **FAIL** |
| 4     | MAV_FRAME_LOCAL_ENU [dep]         | REJECTED                                                                 |
| 5     | MAV_FRAME_GLOBAL_INT              | ACCEPTED, INT-encoded as MAV_FRAME_GLOBAL (same category) → PASS        |
| 6     | MAV_FRAME_GLOBAL_RELATIVE_ALT_INT | ACCEPTED, stored as MAV_FRAME_GLOBAL — altitude category change → **FAIL** |
| 7–12  | LOCAL/BODY frames                 | REJECTED                                                                 |
| 10    | MAV_FRAME_GLOBAL_TERRAIN_ALT      | ACCEPTED, stored as MAV_FRAME_GLOBAL — altitude category change → **FAIL** |
| 11    | MAV_FRAME_GLOBAL_TERRAIN_ALT_INT  | REJECTED → PASS (contrast with ArduCopter where 11 is ACCEPTED/FAIL)    |
| 12    | MAV_FRAME_BODY_FRD                | REJECTED                                                                 |
| 13    | MAV_FRAME_RESERVED_13             | ACCEPTED, stored as MAV_FRAME_GLOBAL — altitude category change → **FAIL** |
| 14–21 | RESERVED / LOCAL                 | REJECTED                                                                 |

**Geofence (mission_type=1):** ArduPlane fixed-wing rejects almost all geofence frames.

| Frame | ArduPlane FW result                                                         |
|-------|-----------------------------------------------------------------------------|
| 0     | REJECTED (contrast: ArduCopter ACCEPTS frame=0 for geofence)                |
| 3     | REJECTED                                                                    |
| 5     | REJECTED                                                                    |
| 6     | ACCEPTED but misidentified as GLOBAL_RELATIVE_ALT → altitude category change → **FAIL** |
| 10    | REJECTED                                                                    |
| 11    | REJECTED                                                                    |
| others | REJECTED                                                                   |

**Rally points (mission_type=2):**

| Frame | ArduPlane FW result                                                  |
|-------|----------------------------------------------------------------------|
| 0     | REJECTED (contrast: ArduCopter ACCEPTS frame=0 for rally)            |
| 3     | ACCEPTED, frame preserved (z=10.000)                                 |
| 5     | ACCEPTED, INT-encoded as MAV_FRAME_GLOBAL (z=10.000)                 |
| 6     | ACCEPTED, INT-encoded as MAV_FRAME_GLOBAL_RELATIVE_ALT (z=10.000)   |
| 10    | ACCEPTED, frame preserved (z=10.000)                                 |
| 11    | ACCEPTED, INT-encoded as MAV_FRAME_GLOBAL_TERRAIN_ALT (z=10.000)    |
| others | REJECTED                                                             |

**MAV_FRAME_MISSION (frame=2):**
- `DO_CHANGE_SPEED` (non-location cmd): **ACCEPTED but param1 zeroed** (same violation as ArduCopter) → **FAIL**.
- `NAV_WAYPOINT` (location cmd, misuse): **ACCEPTED** with coordinates stored unscaled.

**Key differences vs ArduCopter:**
- Frame 11 (`GLOBAL_TERRAIN_ALT_INT`) is **REJECTED** on ArduPlane FW but **ACCEPTED** (and fails) on ArduCopter/QuadPlane.
- Frame 13 (`RESERVED_13`) is **ACCEPTED-and-altered** on ArduPlane FW but **REJECTED** on ArduCopter/QuadPlane.
- Geofence is almost entirely rejected on ArduPlane FW (only frame=6 is accepted, and it fails); ArduCopter accepts frames 0, 3, 5, 6, 10, 11.
- Rally frame=0 is **REJECTED** on ArduPlane FW; ACCEPTED on ArduCopter.
- Total failures: 6 (vs 9 for ArduCopter/QuadPlane).

---

## Autopilot-specific behaviour (ArduPlane QuadPlane) — mission protocol

Tested against ArduPlane V4.8.0-dev (70fe7125, `--model quadplane`) connected via TCP port 5760.
Log: `logs/mission_frame_types_ardupilot_quadplane_4.8.0-dev_20260526.log`.
Result: **56 passed, 9 failed**.

**Frame type results are identical to ArduCopter** — same 9 failures (frames 3, 6, 10, 11 for flight missions and geofence; MAV_FRAME_MISSION DO_CHANGE_SPEED param1 zeroed).
Rally results also match ArduCopter exactly.

QuadPlane requires a home item at seq=0 for flight missions, same as ArduCopter.

See "Frame type support (ArduCopter)" for the full tables.

---

## Autopilot-specific behaviour (ArduRover) — mission protocol

Tested against ArduRover V4.8.0-dev (70fe7125, `--model rover`) connected via TCP port 5760.
Result: **56 passed, 9 failed** (frame_types).

**Frame type results are identical to ArduCopter** — same 9 failures (frames 3, 6, 10, 11 for flight missions and geofence; MAV_FRAME_MISSION DO_CHANGE_SPEED param1 zeroed).

ArduRover does **not** require a home item at seq=0 for flight missions.

See "Frame type support (ArduCopter)" for the full tables.
