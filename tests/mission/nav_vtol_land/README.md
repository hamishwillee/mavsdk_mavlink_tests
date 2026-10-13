# NAV_VTOL_LAND (cmd=85) — mission item tests

Tier 1 (protocol acceptance) and Tier 2 (execution) tests for `MAV_CMD_NAV_VTOL_LAND` ("Land using VTOL mode") as a mission item.
The command-protocol (COMMAND_INT) tests are in `tests/command/nav_vtol_land/`.
Design notes, source predictions and findings (shared with NAV_LAND): `tests/mission/nav_land/CLAUDE.md`.

## Parameters (common.xml)

| # | Label | Notes |
|---|-------|-------|
| 1 | Land Options | `NAV_VTOL_LAND_OPTIONS`: 0 DEFAULT, 1 FW_SPIRAL_APPROACH, 2 FW_APPROACH |
| 2 | Empty | |
| 3 | Approach Altitude (m) | Same reference as the Altitude field; NaN if unspecified |
| 4 | Yaw (deg) | NaN = use the current yaw heading mode |
| 5/6 | Latitude/Longitude | |
| 7 | Ground Altitude (m) | NaN = use system default landing altitude |

## Test files

| File | Tier | What |
|------|------|------|
| `test_protocol.py` | 1 | `Tier1MissionTestBase` (baseline, sentinel pairs, frame survey) + every Land Options value, Approach Altitude storage and NaN, yaw storage/NaN/edge values, location round-trip, NaN ground altitude |
| `test_flight.py` | 2 | lands in VTOL mode; lat/lon, yaw and approach altitude (two flights: target N, 135°, 50 m; target E, 315°, 70 m); how the landing is flown; a wrong ground altitude (15 m); untestable params recorded with reasons |

The flight routine is shared with the other three landing directories (`tests/land_helpers.py`).
A VTOL (HEARTBEAT type) takes off with NAV_VTOL_TAKEOFF and reaches the landing item in forward flight — the case "land using VTOL mode" is about.

## Running

```bash
pytest tests/mission/nav_vtol_land/ -v --log-cli-level=INFO     # mock: Tier 1 only, Tier 2 skips
pytest tests/mission/nav_vtol_land/test_protocol.py tests/mission/nav_vtol_land/test_flight.py --drone-address=udp://:14540 \
    --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_standard_vtol --vehicle-type=vtol --autopilot=px4 -v --log-cli-level=INFO
# PX4 multicopter: --px4-model=sihsim_quadx --vehicle-type=quadcopter
```

`~/px4_sitl_images/latest` is PX4's `px4io/px4-sitl:latest` container build, unpacked by `scripts/fetch_px4_sitl_image.py` (`SETUP.md`).
Tier 2 takes ~12 min on the VTOL; on the multicopter nothing flies (below).

## Results — 2026-10-13, PX4 SIH

Two builds: PX4's `px4io/px4-sitl:latest` container (v1.18.0-beta1-660-g61907962c9) and PX4 PR #28764 "honor yaw in LAND and VTOL_LAND" (`17ed9c8c42`, built from source). They differ only in the Yaw row.

Tier 1 is identical on both vehicles and both builds: **12 PASS, 2 FAIL (compat)**. Land Options 1 and 2 (and 3, outside the enum) are NACKed; a real param2 is NACKed; yaw and the location round-trip; NaN ground altitude is stored as NaN. **Approach Altitude is accepted but not stored** — 40 m and NaN both come back as 0.0 (both FAIL). Accepted in frames 0, 3, 5, 6.

| Tier 2 | PX4 VTOL (`sihsim_standard_vtol`) | PX4 multicopter (`sihsim_quadx`) |
|---|---|---|
| lands in VTOL mode | PASS — reached in fixed-wing flight, back-transition 22–24 m from the target at 30 m, touchdown in MC mode after ~105 s | NA — upload accepted, `start_mission` DENIED |
| 5/6 Lat/Lon | SUPPORTED — 0.1 / 0.0 m from both 250 m targets | REJECTED |
| 4 Yaw — container | **FAIL (compat)** — 135° → 26.2°, 315° → 62.7° | REJECTED |
| 4 Yaw — PR #28764 | **SUPPORTED** — 135.0°, 315.0° | REJECTED |
| 3 Approach Altitude | **FAIL (compat)** — landing started at 30 m and peaked at ~32 m for both 50 and 70 m (both builds) | REJECTED |
| 1 Land Options | not supported — non-default values NACKed at upload | same |
| 7 Ground Altitude | untestable (flat terrain); z = 15 m still lands on the ground | — |
| Param coverage | COMPLETE | COMPLETE |

- **Yaw is stored but never used on the container build** — the same `handleLanding` "ignore yaw for landing items" as NAV_LAND. **PR #28764 fixes it**: the same test passes, both values exact to 0.1°, with the back-transition, touchdown point and every other result unchanged.
- **Approach Altitude is neither stored nor used.** The mission param mask allows it ("QGC sets it; unused by mission_block"), so it is accepted, dropped, and the approach is flown at whatever altitude the vehicle is already at. Accepted and not honoured — a compatibility error twice over (Tier 1 and Tier 2).
- **A multicopter won't fly it.** PX4's feasibility check rejects VTOL items on a non-VTOL (`FeasibilityChecker::checkItemsFitToVehicleType`), so the mission is refused at start; recorded as not supported on that frame.
