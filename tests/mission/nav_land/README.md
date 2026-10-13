# NAV_LAND (cmd=21) — mission item tests

Tier 1 (protocol acceptance) and Tier 2 (execution) tests for `MAV_CMD_NAV_LAND` ("Land at location.") as a mission item.
The command-protocol (COMMAND_INT) tests are in `tests/command/nav_land/`; `NAV_VTOL_LAND` is in `tests/mission/nav_vtol_land/` and `tests/command/nav_vtol_land/`.
Design notes, source predictions and findings for all four directories: `CLAUDE.md` in this directory.

## Parameters (common.xml)

| # | Label | Notes |
|---|-------|-------|
| 1 | Abort Alt (m) | Minimum target altitude if landing is aborted (0 = use system default) |
| 2 | Land Mode | `PRECISION_LAND_MODE`: 0 DISABLED, 1 OPPORTUNISTIC, 2 REQUIRED |
| 3 | Empty | |
| 4 | Yaw Angle (deg) | NaN = use the current yaw heading mode |
| 5/6 | Latitude/Longitude | |
| 7 | Altitude (m) | Landing altitude (ground level in current frame) |

## Test files

| File | Tier | What |
|------|------|------|
| `test_protocol.py` | 1 | `Tier1MissionTestBase` (baseline, sentinel pairs, frame survey) + Abort Alt storage, every Land Mode value, yaw storage/NaN/edge values, location round-trip, NaN altitude |
| `test_flight.py` | 2 | lands; lat/lon and yaw (two flights: target N with 135°, target E with 315°); how the landing is flown; a wrong ground altitude (15 m); untestable params recorded with reasons |

The flight routine is shared with the other three landing directories (`tests/land_helpers.py`).
A VTOL (HEARTBEAT type) takes off with NAV_VTOL_TAKEOFF and reaches the landing item in forward flight, 250 m targets; anything else takes off with NAV_TAKEOFF, 60 m targets.

## Running

```bash
pytest tests/mission/nav_land/ -v --log-cli-level=INFO     # mock: Tier 1 only, Tier 2 skips
pytest tests/mission/nav_land/test_protocol.py tests/mission/nav_land/test_flight.py --drone-address=udp://:14540 \
    --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO
# PX4 VTOL: --px4-model=sihsim_standard_vtol --vehicle-type=vtol
```

`~/px4_sitl_images/latest` is PX4's `px4io/px4-sitl:latest` container build, unpacked by `scripts/fetch_px4_sitl_image.py` (`SETUP.md`).
Tier 2 takes ~4 min on the multicopter and ~12 min on the VTOL.

## Results — 2026-10-13, PX4 SIH

Two builds: PX4's `px4io/px4-sitl:latest` container (v1.18.0-beta1-660-g61907962c9) and PX4 PR #28764 "honor yaw in LAND and VTOL_LAND" (`17ed9c8c42`, built from source). They differ only in the Yaw row.

Tier 1 is identical on both vehicles and both builds: **18 PASS**. Every param round-trips (Abort Alt 25 m, all three Land Modes, yaw 90°, the location); a real param3 is NACKed; Land Mode 5 (outside the enum) is stored as sent; yaw −90°/450° is wrapped to 270°/90°; NaN altitude is stored as NaN. Accepted in frames 0, 3, 5, 6.

| Tier 2 | PX4 multicopter (`sihsim_quadx`) | PX4 VTOL (`sihsim_standard_vtol`) |
|---|---|---|
| lands | PASS — touchdown ~47 s after the item became current | PASS — reached in fixed-wing flight, back-transition ~22 m from the target at 30 m, touchdown in MC mode after ~105 s |
| 5/6 Lat/Lon | SUPPORTED — 0.0 m from both 60 m targets | SUPPORTED — 0.0 / 0.1 m from both 250 m targets |
| 4 Yaw Angle — container | **FAIL (compat)** — 135° → 0.0°, 315° → 90.1° (the approach bearings) | **FAIL (compat)** — 135° → 26.1°, 315° → 62.6° |
| 4 Yaw Angle — PR #28764 | **SUPPORTED** — 135.1°, 315.1° | **SUPPORTED** — 135.0°, 314.9° |
| 7 Altitude | untestable (flat terrain); z = 15 m still lands on the ground | untestable; z = 15 m still lands on the ground |
| 1 Abort Alt, 2 Land Mode | untestable (no aborted landing, no beacon) | same |
| Param coverage | COMPLETE | COMPLETE |

- **Yaw is stored but never used on the container build.** `mission_base.cpp::handleLanding`: "ignore yaw for landing items" (`_mission_item.yaw = NAN`). Accepted and not honoured — a compatibility error (root `CLAUDE.md` rule 4).
- **PR #28764 fixes it.** The same test passes on both vehicles, within 0.1° for both values; position, Tier 1 and everything else are unchanged, and the landing takes ~3 s longer (the vehicle aligns over the point before descending).
- **The approach is a dogleg.** The multicopter flies to the point at its current altitude and then descends (0 m from the target at half altitude).
- **On a VTOL the item is flown as NAV_VTOL_LAND** (`force_vtol()`): the results match `tests/mission/nav_vtol_land/`'s.
