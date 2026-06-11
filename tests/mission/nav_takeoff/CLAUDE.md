# NAV_TAKEOFF — mission protocol notes

See `README.md` for per-test results tables (all stacks side-by-side).

## Parameter storage summary (all stacks)

| Parameter | PX4 (all types) | ArduCopter MC | ArduPlane FW / QuadPlane |
|-----------|-----------------|---------------|--------------------------|
| param1 — Pitch | zeroed (not stored) | stored raw; no bounds enforced | stored raw; no bounds enforced |
| param2 — unused | NaN ok; non-NaN silently zeroed | **NaN REJECTED** — spec violation; use 0.0 | same as ArduCopter |
| param3 — Flags | zeroed (not stored) | zeroed (not stored) | zeroed (not stored) |
| param4 — Yaw | stored; −90°→270°, 450°→90° (normalised) | zeroed (not stored) | zeroed (not stored) |
| x/y — location | stored | stored | stored |
| z — altitude | stored; NaN preserved as "use default" | stored; NaN **REJECTED** (sanity_check) | stored; NaN **REJECTED** (sanity_check) |
| INT32_MAX sentinel | accepted ("use current position") | **NACKED** — spec violation | **NACKED** — spec violation |

**PX4**: identical behaviour across multicopter, fixed-wing, and VTOL (same mission storage code).

**ArduPilot**: `mavlink_int_to_mission_cmd` for NAV_TAKEOFF stores only `cmd.p1 = packet.param1`.
`sanity_check_params` uses `nan_mask = ~(1 << 3)` — only param4 may be NaN; params 1/2/3 must be non-NaN or the upload is rejected with `MAV_MISSION_INVALID_PARAM2` (MAVSDK raises `InvalidArgument`).

**ArduCopter/ArduPlane/QuadPlane**: identical storage behaviour.
ArduPlane does **not** require a home item at seq=0 (no `home_item_for_mission` prepend needed).

## ArduPilot-specific storage notes

- **Negative pitch underflow**: param1=−10° is stored as 65526° — uint16 underflow: 65536 − 10.
  This is a storage bug (nonsensical if executed).
- **INT32_MAX NACKed**: both ArduCopter and ArduPlane reject the "use current position" sentinel with `INVALID_ARGUMENT` — a spec violation for `hasLocation` commands.
- **param2 NaN rejected**: spec requires unused params to accept NaN; ArduPilot explicitly disallows it via `sanity_check_params`.
- **Positive out-of-range pitch preserved raw**: 89° and 180° stored as-is — no bounds checking on param1.

## Log references

- PX4 MC: `logs/nav_takeoff_px4_mc_20260525b.log`
- PX4 FW: `logs/nav_takeoff_px4_fw_20260525b.log`
- PX4 VTOL: `logs/nav_takeoff_px4_vtol_20260525b.log`
- ArduCopter: `logs/nav_takeoff_arducopter_20260525b.log`
- ArduPlane FW: `logs/nav_takeoff_arduplane_20260525b.log`
- QuadPlane: `logs/nav_takeoff_quadplane_20260525b.log`

**QuadPlane SITL note**: use `--model quadplane` with ROMFS defaults only — specifying `--defaults quadplane.parm` causes SITL to crash after the first TCP connection closes.
