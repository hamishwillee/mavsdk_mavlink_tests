# NAV_TAKEOFF — mission protocol notes

See `README.md` for per-test results tables (all stacks side-by-side).

## MAV_CMD_NAV_TAKEOFF (cmd=22) — PX4 storage

PX4 stores **param4 (Yaw)** and the location fields (x, y, z) for NAV_TAKEOFF but silently
zeroes **param1 (Pitch)** and **param3 (Flags)** on download.  Non-NaN values uploaded in
param2 (unused) are also zeroed.  NaN for param2 is accepted.

**Results are identical across all PX4 vehicle types** (multicopter, fixed-wing, VTOL standard) —
PX4 uses the same mission storage code regardless of vehicle type.

Test results (multicopter log: `logs/nav_takeoff_px4_mc_20260525b.log`;
fixed-wing: `logs/nav_takeoff_px4_fw_20260525b.log`;
VTOL: `logs/nav_takeoff_px4_vtol_20260525b.log`):

- `test_protocol_command_accepted` — PASS
- `test_protocol_param1_pitch_preserved` — **FAIL** (param1 zeroed on storage; 89° also zeroed)
- `test_protocol_param2_unused` — PASS (NaN accepted; 1.0 silently altered to 0.0 — NOTE)
- `test_protocol_param3_flags_preserved` — **FAIL** (param3 zeroed on storage)
- `test_protocol_param4_yaw_specific` — PASS (param4=90.0 preserved)
- `test_protocol_param4_yaw_nan` — PASS (param4=NaN preserved)
- `test_protocol_location_preserved` — PASS
- `test_protocol_location_current_position` — PASS (INT32_MAX preserved — "use current position" accepted)
- `test_protocol_location_nan_altitude` — PASS (observational: NaN altitude **preserved** — accepted as "use default")
- `test_protocol_param3_flags_zero` — PASS (param3=0.0 trivially preserved — param3 not stored)
- `test_protocol_param3_flags_undefined_bits` — PASS (observational: undefined bit 2.0 silently altered to 0.0 — NACK preferred)
- `test_protocol_param1_nan` — PASS (observational: NaN param1 **altered to 0.0** — normalised, not rejected)
- `test_protocol_param1_pitch_very_large` — PASS (observational: 180° **zeroed** — param1 not stored at all)
- `test_protocol_param4_yaw_negative` — PASS (−90° **normalised to 270°** on storage — execution unambiguous)
- `test_protocol_param4_yaw_overflow` — PASS (450° **normalised to 90°** on storage — execution unambiguous)
- `test_protocol_param4_yaw_zero` — PASS (0° preserved)
- `test_protocol_param1_pitch_large` — PASS (89° **zeroed** — consistent with all other param1 values; PX4 does not store param1)
- `test_protocol_param1_pitch_negative` — PASS (−10° **zeroed** — consistent with all other param1 values)

## MAV_CMD_NAV_TAKEOFF (cmd=22) — ArduCopter storage

ArduCopter's `mavlink_int_to_mission_cmd` for NAV_TAKEOFF only stores `cmd.p1 = packet.param1`.
Params 2, 3, 4 are **not stored** and come back as 0.0 on download.

`sanity_check_params` uses `nan_mask = ~(1 << 3)` for NAV_TAKEOFF, meaning **only param4 may be
NaN**; params 1, 2, 3 must be non-NaN or the upload is rejected with `MAV_MISSION_INVALID_PARAM2`
(MAVSDK raises `MissionRaw::Result::InvalidArgument`).

This creates a spec violation for param2 (unused/empty): the MAVLink spec requires unused params
to accept NaN, but ArduCopter explicitly disallows it.  Workaround: use `0.0` for param2 in all
non-param2 tests; `test_protocol_param2_unused` records the NaN rejection as a FAIL.

Test results (ArduCopter, multicopter; log: `logs/nav_takeoff_arducopter_20260525b.log`):
- `test_protocol_command_accepted` — PASS
- `test_protocol_param1_pitch_preserved` — PASS (param1 stored correctly)
- `test_protocol_param2_unused` — **FAIL** (NaN rejected — spec violation; 0.0 accepted as workaround)
- `test_protocol_param3_flags_preserved` — **FAIL** (param3 not stored; zeroed on download)
- `test_protocol_param4_yaw_specific` — **FAIL** (param4 not stored; zeroed on download)
- `test_protocol_param4_yaw_nan` — **FAIL** (param4 not stored; 0.0 returned instead of NaN)
- `test_protocol_location_preserved` — PASS (location fields stored correctly)
- `test_protocol_location_current_position` — **FAIL** (INT32_MAX NACKed — `INVALID_ARGUMENT`; spec violation: `hasLocation` commands must accept the "use current position" sentinel)
- `test_protocol_location_nan_altitude` — PASS (observational: NaN NACKed — `INVALID_ARGUMENT`; sanity_check_params rejects NaN altitude)
- `test_protocol_param3_flags_zero` — PASS (vacuous — param3 discarded; 0.0 is the zero-initialised default)
- `test_protocol_param3_flags_undefined_bits` — PASS (observational: undefined bit 2.0 silently altered to 0.0 — NACK preferred)
- `test_protocol_param1_nan` — PASS (observational: NaN rejected `INVALID_ARGUMENT` — sanity_check_params mask requires non-NaN param1)
- `test_protocol_param1_pitch_very_large` — PASS (observational: 180° **preserved raw** — no upper bound enforced)
- `test_protocol_param4_yaw_negative` — PASS (−90° **altered to 0.0°** — param4 not stored; all non-zero yaw values become 0.0)
- `test_protocol_param4_yaw_overflow` — PASS (450° **altered to 0.0°** — same reason)
- `test_protocol_param4_yaw_zero` — PASS (0.0° preserved — indistinguishable from "not stored")
- `test_protocol_param1_pitch_large` — PASS (89° **preserved raw** — no clamping; ArduPilot stores param1 as-is)
- `test_protocol_param1_pitch_negative` — PASS (−10° **→ 65526.0°** — uint16 underflow: stored as unsigned 16-bit value; 65536 − 10 = 65526)

**Note on negative pitch**: ArduPilot stores param1 in a `uint16_t` field internally.  Negative
float values are converted to uint16 by truncating/wrapping, producing 65526 for −10.  This is
a storage bug — the field should either reject negative pitch or store it correctly.  If NAV_TAKEOFF
is ever executed with this stored value, the pitch target would be 65526° which is nonsensical.

## MAV_CMD_NAV_TAKEOFF (cmd=22) — ArduPlane / QuadPlane storage

**Storage behaviour is identical to ArduCopter**: only `param1` is stored; params 3 and 4 are
discarded; NaN for param2 is rejected with `MAV_MISSION_INVALID_PARAM2`.

ArduPlane does **not** require a home item at seq=0 (no `home_item_for_mission` prepend needed).

Test results apply to both ArduPlane (fixed-wing, `--model plane`) and QuadPlane (VTOL,
`--model quadplane`; logs: `logs/nav_takeoff_arduplane_20260525b.log`,
`logs/nav_takeoff_quadplane_20260525b.log`):

**QuadPlane SITL note**: use `--model quadplane` with ROMFS defaults only — specifying
`--defaults quadplane.parm` causes SITL to crash after the first TCP connection closes.

- `test_protocol_command_accepted` — PASS
- `test_protocol_param1_pitch_preserved` — PASS
- `test_protocol_param2_unused` — **FAIL** (NaN rejected — spec violation; 0.0 accepted as workaround)
- `test_protocol_param3_flags_preserved` — **FAIL** (param3 not stored; zeroed on download)
- `test_protocol_param4_yaw_specific` — **FAIL** (param4 not stored; zeroed on download)
- `test_protocol_param4_yaw_nan` — **FAIL** (param4 not stored; 0.0 returned instead of NaN)
- `test_protocol_location_preserved` — PASS
- `test_protocol_location_current_position` — **FAIL** (INT32_MAX NACKed — `INVALID_ARGUMENT`; spec violation: same as ArduCopter)
- `test_protocol_location_nan_altitude` — PASS (observational: NaN NACKed — `INVALID_ARGUMENT`)
- `test_protocol_param3_flags_zero` — PASS (vacuous — param3 discarded; 0.0 is the zero-initialised default)
- `test_protocol_param3_flags_undefined_bits` — PASS (observational: undefined bit 2.0 silently altered to 0.0)
- `test_protocol_param1_nan` — PASS (observational: NaN rejected `INVALID_ARGUMENT` — sanity_check_params)
- `test_protocol_param1_pitch_very_large` — PASS (observational: 180° **preserved raw** — no upper bound enforced)
- `test_protocol_param4_yaw_negative` — PASS (−90° altered to 0.0° — same as ArduCopter)
- `test_protocol_param4_yaw_overflow` — PASS (450° altered to 0.0° — same as ArduCopter)
- `test_protocol_param4_yaw_zero` — PASS (0.0° preserved)
- `test_protocol_param1_pitch_large` — PASS (89° preserved raw)
- `test_protocol_param1_pitch_negative` — PASS (−10° → 65526.0° — same uint16 underflow as ArduCopter)
