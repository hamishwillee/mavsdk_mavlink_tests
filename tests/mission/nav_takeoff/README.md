# NAV_TAKEOFF (cmd=22) — Protocol Conformance Tests

This directory contains Tier 1 (protocol acceptance) and Tier 2 (execution verification) tests for `MAV_CMD_NAV_TAKEOFF`.

See the root `CLAUDE.md` for the two-tier testing model, parameter conventions, and the NaN-first three-probe pattern for unused params.

## Command parameters (MAVLink spec)

| # | Label | Type | Notes |
|---|-------|------|-------|
| 1 | Pitch | float, deg | Minimum pitch for fixed-wing; ignored by multicopters |
| 2 | — | float | **Unused** — spec requires NaN; some stacks reject NaN (see below) |
| 3 | Flags | float | `NAV_TAKEOFF_FLAGS` bitmask; bit 0 = HORIZONTAL_POSITION_NOT_REQUIRED |
| 4 | Yaw | float, deg | NaN = use current heading |
| 5 | Latitude | int ×1e7 | INT32_MAX = use current position |
| 6 | Longitude | int ×1e7 | INT32_MAX = use current position |
| 7 | Altitude | float, m | Target altitude AMSL |

## Test files

| File | Tier | Description |
|------|------|-------------|
| `test_protocol.py` | Tier 1 | 18 round-trip tests: upload item, download, assert param preserved |
| `test_flight.py` | Tier 2 | 7 execution tests: arm + fly + assert telemetry (skip in mock mode) |

## Running

```bash
# Paired (mock) — protocol tests pass, flight tests skip
pytest tests/mission/nav_takeoff/ -v --log-cli-level=INFO

# PX4 SIH multicopter
pytest tests/mission/nav_takeoff/test_protocol.py \
    --drone-address=udp://:14540 -v --log-cli-level=INFO

# ArduCopter SITL
pytest tests/mission/nav_takeoff/test_protocol.py \
    --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
    --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 \
    -v --log-cli-level=INFO
```

---

## Tier 1 Results — Protocol acceptance

Tested: 2026-05-25.
Results updated after adding 6 new tests.
Logs: `logs/nav_takeoff_px4_mc_20260525b.log`, `logs/nav_takeoff_px4_fw_20260525b.log`, `logs/nav_takeoff_px4_vtol_20260525b.log`, `logs/nav_takeoff_arducopter_20260525b.log`, `logs/nav_takeoff_arduplane_20260525b.log`, `logs/nav_takeoff_quadplane_20260525b.log`.

**Key: PASS = test passed &nbsp; FAIL = test failed &nbsp; NOTE = advisory (test passes, behavior noted)**

### test_protocol_command_accepted

Upload baseline NAV_TAKEOFF item; assert no `MissionRawError`.

| PX4 MC | PX4 FW | PX4 VTOL | ArduCopter MC | ArduPlane FW | ArduPlane QP |
|--------|--------|----------|---------------|--------------|--------------|
| PASS   | PASS   | PASS     | PASS          | PASS         | PASS         |

### test_protocol_param1_pitch_preserved

Upload `param1=15.0`; assert `downloaded param1 ≈ 15.0`.

| PX4 MC | PX4 FW | PX4 VTOL | ArduCopter MC | ArduPlane FW | ArduPlane QP |
|--------|--------|----------|---------------|--------------|--------------|
| **FAIL** (zeroed → 0.0) | **FAIL** (zeroed → 0.0) | **FAIL** (zeroed → 0.0) | PASS | PASS | PASS |

PX4 silently zeroes param1 on storage across all vehicle types.
ArduPilot stores param1 correctly (`mavlink_int_to_mission_cmd` copies `packet.param1`).

### test_protocol_param2_unused

NaN-first three-probe pattern for the unused param2.

| PX4 MC | PX4 FW | PX4 VTOL | ArduCopter MC | ArduPlane FW | ArduPlane QP |
|--------|--------|----------|---------------|--------------|--------------|
| PASS   | PASS   | PASS     | **FAIL**      | **FAIL**     | **FAIL**     |

- **PX4**: accepts NaN (spec-correct); non-NaN value 1.0 silently altered to 0.0 (NOTE).
- **ArduPilot** (all variants): rejects NaN with `INVALID_ARGUMENT` — spec violation.
  The `sanity_check_params` mask for NAV_TAKEOFF disallows NaN in param2.
  Workaround: use `0.0`.
  Non-NaN value 1.0 silently altered to 0.0 (NOTE).

### test_protocol_param3_flags_preserved

Upload `param3=1.0` (NAV_TAKEOFF_FLAGS bit 0); assert `downloaded param3 ≈ 1.0`.

| PX4 MC | PX4 FW | PX4 VTOL | ArduCopter MC | ArduPlane FW | ArduPlane QP |
|--------|--------|----------|---------------|--------------|--------------|
| **FAIL** (zeroed → 0.0) | **FAIL** | **FAIL** | **FAIL** (zeroed → 0.0) | **FAIL** | **FAIL** |

No stack tested stores param3.
PX4 zeroes it; ArduPilot never reads or stores it.
`NAV_TAKEOFF_FLAGS` (added in MAVLink 2.0) is not yet implemented in either stack.

### test_protocol_param4_yaw_specific

Upload `param4=90.0`; assert `downloaded param4 ≈ 90.0`.

| PX4 MC | PX4 FW | PX4 VTOL | ArduCopter MC | ArduPlane FW | ArduPlane QP |
|--------|--------|----------|---------------|--------------|--------------|
| PASS   | PASS   | PASS     | **FAIL** (zeroed → 0.0) | **FAIL** | **FAIL** |

PX4 stores param4 (Yaw) correctly.
ArduPilot's NAV_TAKEOFF handler does not store param4.

### test_protocol_param4_yaw_nan

Upload `param4=NaN`; assert downloaded param4 is NaN.

| PX4 MC | PX4 FW | PX4 VTOL | ArduCopter MC | ArduPlane FW | ArduPlane QP |
|--------|--------|----------|---------------|--------------|--------------|
| PASS   | PASS   | PASS     | **FAIL** (→ 0.0) | **FAIL** | **FAIL** |

ArduPilot does not store param4; NaN (use-current-heading) comes back as 0.0.

### test_protocol_location_preserved

Upload distinct lat/lon/alt; assert x, y, z round-trip exactly.

| PX4 MC | PX4 FW | PX4 VTOL | ArduCopter MC | ArduPlane FW | ArduPlane QP |
|--------|--------|----------|---------------|--------------|--------------|
| PASS   | PASS   | PASS     | PASS          | PASS         | PASS         |

All stacks store the location fields faithfully.

### test_protocol_location_current_position

Upload `x=INT32_MAX, y=INT32_MAX` (0x7FFF_FFFF — "use current position" sentinel); assert round-trip.

| PX4 MC | PX4 FW | PX4 VTOL | ArduCopter MC | ArduPlane FW | ArduPlane QP |
|--------|--------|----------|---------------|--------------|--------------|
| PASS   | PASS   | PASS     | **FAIL** (NACKed — INVALID_ARGUMENT) | **FAIL** | **FAIL** |

PX4 accepts and stores INT32_MAX for lat/lon — the "take off from current position" sentinel is supported.
ArduPilot rejects INT32_MAX with `INVALID_ARGUMENT` for all three vehicle types; this is a spec violation for a command marked `hasLocation="true"` and `isDestination="true"`.

### test_protocol_location_nan_altitude

Upload `z=NaN` (altitude field); assert accepted (observational — any outcome is valid for this field).

| PX4 MC | PX4 FW | PX4 VTOL | ArduCopter MC | ArduPlane FW | ArduPlane QP |
|--------|--------|----------|---------------|--------------|--------------|
| PASS (NaN preserved — accepted) | PASS | PASS | PASS (NaN NACKed — INVALID_ARGUMENT) | PASS | PASS |

PX4 accepts NaN altitude (stores it as NaN — "use default altitude").
ArduPilot rejects NaN altitude with `INVALID_ARGUMENT`.
Both behaviours are protocol-valid for an altitude that is arguably required for takeoff.
Test is observational — no hard assertion either way.

### test_protocol_param3_flags_zero

Upload `param3=0.0` (no flags); assert `downloaded param3 ≈ 0.0`.

| PX4 MC | PX4 FW | PX4 VTOL | ArduCopter MC | ArduPlane FW | ArduPlane QP |
|--------|--------|----------|---------------|--------------|--------------|
| PASS   | PASS   | PASS     | PASS          | PASS         | PASS         |

All stacks accept param3=0.0.
Note: this PASS is trivially satisfied for PX4 (which zeroes param3 regardless of the uploaded value) and ArduPilot (which discards param3 entirely).
The zero download is indistinguishable from "param not stored at all".

### test_protocol_param3_flags_undefined_bits

Upload `param3=2.0` (bit 1 — undefined in `NAV_TAKEOFF_FLAGS`); observe outcome (observational).

| Stack | Outcome | Observation |
|-------|---------|-------------|
| PX4 (all) | NOTE: undefined bit silently altered to 0.0 | PX4 zeroes param3 regardless (same as all param3 values) |
| ArduPilot (all) | NOTE: undefined bit silently altered to 0.0 | ArduPilot discards param3; 0.0 is returned (not stored) |

No stack NACKs the undefined bit value.
Both silently discard it (producing 0.0), which is the same behaviour as for the defined bit (param3 is not stored by any tested stack).
A NACK would be preferred per the methodology for undefined enum/bitmask values.

### test_protocol_param1_nan

Upload `param1=NaN` ("no minimum pitch constraint"); observe outcome (observational).

| Stack | Outcome | Observation |
|-------|---------|-------------|
| PX4 (all) | ALTERED to 0.0 | NaN normalised to 0.0 (not rejected; param1 not stored anyway) |
| ArduPilot (all) | NaN rejected — INVALID_ARGUMENT | `sanity_check_params` rejects NaN for param1 (defined param) |

ArduPilot rejects NaN for param1 via `sanity_check_params` — the mask for NAV_TAKEOFF requires a numeric pitch value.
This is consistent with the param1 NaN rejection pattern already observed for param2.
PX4 normalises NaN to 0.0 without rejection (param1 is not stored anyway).

### test_protocol_param1_pitch_very_large

Upload `param1=180.0` (above implicit 90° maximum); observe outcome (observational).

| Stack | Stored value | Observation |
|-------|-------------|-------------|
| PX4 (all) | **0.0°** | Trivially zeroed — PX4 does not store param1 at all. Not informative beyond `test_protocol_param1_pitch_preserved`. |
| ArduPilot (all) | **180.0°** | Preserved raw — no upper-bound enforcement. ArduPilot stores the raw value without clamping or rejection at 90°. |

---

### Edge-case tests (observational — always PASS, behaviour noted)

#### test_protocol_param4_yaw_negative — upload `param4=−90.0`

| Stack | Stored value | Observation |
|-------|-------------|-------------|
| PX4 (all) | **270.0°** | Normalised to [0, 360) — execution unambiguous |
| ArduPilot (all) | **0.0°** | Altered/zeroed (param4 not stored) |

#### test_protocol_param4_yaw_overflow — upload `param4=450.0`

| Stack | Stored value | Observation |
|-------|-------------|-------------|
| PX4 (all) | **90.0°** | Normalised modulo 360° — execution unambiguous |
| ArduPilot (all) | **0.0°** | Altered/zeroed (param4 not stored) |

#### test_protocol_param4_yaw_zero — upload `param4=0.0`

| PX4 MC | PX4 FW | PX4 VTOL | ArduCopter MC | ArduPlane FW | ArduPlane QP |
|--------|--------|----------|---------------|--------------|--------------|
| PASS (0.0° stored) | PASS | PASS | PASS (0.0° returned — **vacuous PASS**: ArduPilot does not store param4; the 0.0 is the zero-initialised default, not a stored north heading) | PASS (vacuous) | PASS (vacuous) |

The test specifically checks that 0.0° is not aliased to NaN (which would be a spec violation).
For ArduPilot the PASS is vacuous — param4 is not stored at all, so 0.0° (north) is indistinguishable from the stack's "not set" state.

#### test_protocol_param1_pitch_large — upload `param1=89.0`

| Stack | Stored value | Observation |
|-------|-------------|-------------|
| PX4 (all) | **0.0°** | Trivially zeroed — PX4 does not store param1 at all (same result as 15°). This test is not informative for PX4; it confirms nothing beyond what `test_protocol_param1_pitch_preserved` already showed. |
| ArduPilot (all) | **89.0°** | Preserved raw — no clamping applied. ArduPilot accepts the full pitch range without rejection. |

#### test_protocol_param1_pitch_negative — upload `param1=−10.0`

| Stack | Stored value | Observation |
|-------|-------------|-------------|
| PX4 (all) | **0.0°** | Trivially zeroed — PX4 does not store param1 at all (same result as 15°). Not informative beyond `test_protocol_param1_pitch_preserved`. |
| ArduPilot (all) | **65526.0°** | **uint16 underflow bug**: −10 stored in a `uint16_t` field → 65536 − 10 = 65526. ArduPilot does not validate the sign of param1 before storage. If NAV_TAKEOFF were executed with this stored value, the pitch target would be 65526° which is nonsensical. |

---

## Result summary table

`✓` = protocol stores/accepts value correctly &nbsp; `✗` = value not stored, corrupted, or NACKed &nbsp; `~` = observational (no assertion)

| Test | PX4 MC | PX4 FW | PX4 VTOL | ArduCopter MC | ArduPlane FW | ArduPlane QP |
|------|:------:|:------:|:--------:|:-------------:|:------------:|:------------:|
| Command accepted | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| param1 (Pitch) 15° | ✗ zeroed | ✗ zeroed | ✗ zeroed | ✓ | ✓ | ✓ |
| param2 (unused) NaN | ✓ NaN ok | ✓ NaN ok | ✓ NaN ok | ✗ NaN rejected¹ | ✗ NaN rejected¹ | ✗ NaN rejected¹ |
| param3 (Flags) 1.0 | ✗ zeroed | ✗ zeroed | ✗ zeroed | ✗ zeroed | ✗ zeroed | ✗ zeroed |
| param4 (Yaw) 90° | ✓ | ✓ | ✓ | ✗ zeroed | ✗ zeroed | ✗ zeroed |
| param4 (Yaw) NaN | ✓ NaN preserved | ✓ | ✓ | ✗ → 0.0 | ✗ → 0.0 | ✗ → 0.0 |
| params 5/6/7 (Lat/Lon/Alt) | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| params 5/6 (Lat/Lon) INT32_MAX | ✓ preserved | ✓ | ✓ | ✗ NACKed² | ✗ NACKed² | ✗ NACKed² |
| param7 (Alt) NaN | ~ NaN ok | ~ | ~ | ~ NACKed | ~ | ~ |
| param3 (Flags) zero (0.0) | ✓³ | ✓³ | ✓³ | ✓³ | ✓³ | ✓³ |
| param3 (Flags) undefined bit (2.0) | ~ zeroed | ~ | ~ | ~ zeroed | ~ | ~ |
| param1 (Pitch) NaN | ~ → 0.0 | ~ | ~ | ~ NACKed | ~ NACKed | ~ NACKed |
| param1 (Pitch) 180° | ~ zeroed | ~ | ~ | ~ preserved | ~ | ~ |

¹ ArduPilot's `sanity_check_params` for NAV_TAKEOFF explicitly disallows NaN in param2, which is a spec violation.
The MAVLink spec requires unused params to accept NaN.
Workaround: use 0.0.

² ArduPilot rejects INT32_MAX for lat/lon with `INVALID_ARGUMENT`.
The MAVLink spec for commands with `hasLocation="true"` and `isDestination="true"` requires INT32_MAX to be accepted as the "use current position" sentinel.
This is a spec violation.

³ Trivially satisfied: PX4 zeroes param3 regardless of upload value; ArduPilot discards it entirely.
A zero PASS here is indistinguishable from "param not stored at all".

**PX4 results are identical across multicopter, fixed-wing, and VTOL** — PX4 uses the same mission storage code regardless of vehicle type.
Only params 4 (Yaw) and 5/6/7 (Location) are stored; params 1, 2, 3 are discarded.
PX4 additionally accepts the INT32_MAX location sentinel and NaN altitude.

**ArduPilot results are identical across ArduCopter, ArduPlane, and QuadPlane** — the same `mavlink_int_to_mission_cmd` handler is used.
Only param1 (Pitch) and 5/6/7 (Location) are stored; params 3 and 4 are discarded.
ArduPilot rejects NaN for param1 and param2 (defined and undefined params alike), and rejects INT32_MAX for the location sentinel — all spec violations.

### param4 (Yaw) — mission storage vs COMMAND_INT execution

There are two distinct paths where param4 yaw matters: **mission protocol** (MISSION_ITEM_INT upload/download via `mission_raw`) and **direct execution** via COMMAND_INT.
They behave differently.

#### Mission storage — PX4 wraps yaw to [0°, 360°)

PX4 **wraps** yaw to [0°, 360°) on mission storage rather than clamping or rejecting out-of-range values:

| Uploaded | Stored | Interpretation |
|----------|--------|----------------|
| 90° (in range) | 90° | preserved as-is |
| 0° (north) | 0° | preserved as-is |
| NaN (use current heading) | NaN | sentinel preserved |
| −90° (negative) | 270° | wrapped: −90 + 360 |
| 450° (> 360°) | 90° | wrapped: 450 mod 360 |

**Implication for mission GCS**: pre-normalisation is not required before sending NAV_TAKEOFF mission items to PX4 — any value is accepted and PX4 will wrap it correctly via modular arithmetic.
There is no evidence of clamping (e.g. a value beyond ±360° being pinned to ±360°); the behaviour is pure wrap-around.

ArduPilot does not store param4 (Yaw) at all in the mission path.
Source confirmation (`AP_Mission.cpp`, `mavlink_int_to_mission_cmd`):

```cpp
case MAV_CMD_NAV_TAKEOFF:                           // MAV ID: 22
    cmd.p1 = packet.param1;                         // minimum pitch (plane only)
    break;
```

Only `param1`/`cmd.p1` is copied in both the upload path (storing) and the download path (serialising back to MISSION_ITEM_INT).
param4 is absent from both; it is silently discarded.

#### COMMAND_INT direct execution — both PX4 and ArduPilot ignore yaw

When NAV_TAKEOFF is sent as a direct COMMAND_INT (not via mission upload), **both stacks ignore param4 yaw**:

**PX4** (`navigator_main.cpp`, `VEHICLE_CMD_NAV_TAKEOFF` handler):
```cpp
// Don't set a yaw setpoint for takeoff, as Navigator doesn't handle the yaw reset.
// The yaw setpoint generation is handled by FlightTaskAuto.
rep->current.yaw = NAN;
```
`cmd.param4` is not read.
Yaw is unconditionally set to NaN (use current heading), regardless of what param4 contains.
Note: this is a different code path from mission execution — the normalisation seen in mission storage does not apply here.

**ArduCopter** (`GCS_MAVLink_Copter.cpp`, `handle_MAV_CMD_NAV_TAKEOFF`):
```cpp
// param4 : yaw angle   (not supported)
```
param4 is explicitly documented as unsupported and never read.
Only param3 (flags) and altitude are used.

**ArduPlane** (`GCS_MAVLink_Plane.cpp`, `handle_command_MAV_CMD_NAV_TAKEOFF`): reads only altitude; param4 is never referenced.

**Summary**: GCS implementations cannot use COMMAND_INT to set a takeoff yaw heading on any tested stack.
For PX4 missions, yaw is stored and used during mission execution; COMMAND_INT bypasses this and always uses current heading.

---

## Tier 2 Results — Execution verification

See `test_flight.py`.
Tests require `--drone-address`; they are skipped in paired/mock mode.

> Tier 2 tests have not been run as part of generating this README.
> Run them manually against a SITL with a real drone address to obtain execution results.
> Expected outcomes based on Tier 1 storage evidence:
>
> - **PX4**: `test_takeoff_with_yaw` expected PASS (param4 stored → heading should be followed).
>   `test_takeoff_with_negative_yaw` and `test_takeoff_with_overflow_yaw` will skip (PX4 normalises
>   these on storage, so execution behaviour is unambiguous).
> - **ArduPilot**: `test_takeoff_with_yaw` expected FAIL (param4 not stored → heading not followed).
>   Yaw edge-case tests skip (param4 altered on storage for all values).
