# NAV_VTOL_TAKEOFF (cmd=84) — mission item tests

Tier 1 (protocol acceptance) and Tier 2 (execution) tests for
`MAV_CMD_NAV_VTOL_TAKEOFF` as a mission item. The command-protocol
(COMMAND_INT) tests are in `tests/command/nav_vtol_takeoff/`. Design notes,
source predictions and full per-stack findings: `CLAUDE.md` in this directory.

## Parameters (common.xml)

| # | Label | Notes |
|---|-------|-------|
| 1 | Empty | |
| 2 | Transition Heading | `VTOL_TRANSITION_HEADING`: 0 VEHICLE_DEFAULT, 1 NEXT_WAYPOINT, 2 TAKEOFF, 3 SPECIFIED (use param4), 4 ANY |
| 3 | Empty | |
| 4 | Yaw Angle (deg) | NaN = use the current yaw heading mode |
| 5/6 | Latitude/Longitude | INT32_MAX = current position |
| 7 | Altitude (m) | |

## Test files

| File | Tier | What |
|------|------|------|
| `test_protocol.py` | 1 | `Tier1MissionTestBase` (baseline, sentinel pairs, frame survey) + every Transition Heading value, yaw storage/NaN/0°/edge values, location round-trip and INT32_MAX |
| `test_flight.py` | 2 | takes off and transitions; altitude (30/50 m); lat/lon (400 m N/E); yaw with SPECIFIED (135°/225°); Transition Heading (SPECIFIED, NEXT_WAYPOINT ×2, TAKEOFF); what param7 is used as; characterisation of INT32_MAX and of yaw without SPECIFIED |

## Running

```bash
pytest tests/mission/nav_vtol_takeoff/ -v --log-cli-level=INFO     # mock: Tier 1 only, Tier 2 skips
pytest tests/mission/nav_vtol_takeoff/ --drone-address=udp://:14540 --px4-sitl=~/github/px4/PX4-Autopilot \
    --px4-model=gz_standard_vtol --vehicle-type=vtol --autopilot=px4 -v --log-cli-level=INFO
pytest tests/mission/nav_vtol_takeoff/ --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \
    --ardupilot-sitl=~/github/ardupilot/ardupilot/build/sitl/bin/arduplane --ardupilot-model=quadplane \
    --vehicle-type=quadplane --autopilot=ardupilot -v --log-cli-level=INFO
```

PX4 VTOL needs Gazebo (`gz_standard_vtol`); SIH VTOL never leaves the ground.
Allow ~15 min (PX4) to ~25 min (ArduPlane) for Tier 2.

## Results — 2026-10-01

| Param | PX4 VTOL (Gazebo) | ArduPlane QuadPlane | PX4 multicopter | ArduCopter |
|---|---|---|---|---|
| takes off + transitions in the item | PASS | FAIL (compat) — transitions on the next item | NA — refuses to start (not a VTOL) | NA — not a VTOL; plain takeoff |
| 7 Altitude | SUPPORTED | SUPPORTED | REJECTED | SUPPORTED |
| 5/6 Lat/Lon | SUPPORTED | FAIL (compat) — climbs in place | REJECTED | FAIL (compat) — climbs in place |
| 4 Yaw Angle | REJECTED (SPECIFIED NACKed) | FAIL (compat) — not stored | REJECTED | NA |
| 2 Transition Heading | REJECTED (non-default values NACKed) | FAIL (compat) — not stored | REJECTED | NA |
| param7 used as | transition and final altitude | final altitude only | — | — |

Versions: PX4 `main` (binary `3fe7e7af3`, then re-run on a fresh `7cb65787b3` worktree with identical results); ArduPilot master `31d9b842cb` (4.8.0-dev). PX4 v1.17.0: Tier 1 differs (no param validation, so values are silently dropped). Its Gazebo VTOL quad-chutes too often for Tier 2 verdicts (see `CLAUDE.md`).

Findings worth reporting upstream:
- **PX4: INT32_MAX lat/lon is flown as a real coordinate.** Accepted and
  round-trips, but the vehicle flies toward latitude 214.7°: 2.3 km from
  home after 180 s, with the item never completing.
- **ArduPilot: params 1–4 aren't stored** for this item. Transition Heading
  and Yaw are silently dropped, and a NaN Yaw becomes 0°.
- **Spec:** param7 is a transition altitude on one stack and a final
  altitude on the other (and PX4's command and mission paths differ too).
  "Altitude" needs defining.
