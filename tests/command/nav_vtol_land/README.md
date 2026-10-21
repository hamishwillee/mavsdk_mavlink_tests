# NAV_VTOL_LAND (cmd=85) — command protocol (COMMAND_INT) tests

Tier 1 (ACK) and Tier 2 (execution) tests for `MAV_CMD_NAV_VTOL_LAND` ("Land using VTOL mode") sent as a command.
The mission-item tests are in `tests/mission/nav_vtol_land/`.
Design notes, source predictions and findings (shared with NAV_LAND): `tests/mission/nav_land/CLAUDE.md`.

## Parameters (common.xml)

`hasLocation="true"`, `isDestination="true"` → **COMMAND_INT**.

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
| `test_command.py` | 1 | `Tier1CommandTestBase` (the seven mandatory common checks) + every Land Options value, Approach Altitude, Yaw, location (real, INT32_MAX, out of range), Ground Altitude, wrong frame |
| `test_flight.py` | 2 | lands in VTOL mode; lat/lon, yaw and approach altitude (two flights: target N, 135°, 50 m; target E, 315°, 70 m); how the landing is flown; a wrong ground altitude (15 m) |

Tier 2 uses the flight routine shared by the four landing directories (`tests/land_helpers.py`): a VTOL is flown into forward flight with a short mission and the command is sent there; a NACK of the combined values is attributed per param.
Support is learned from one disarmed probe; if the command isn't UNSUPPORTED the stack is restarted before flying.

## Running

```bash
pytest tests/command/nav_vtol_land/ -v --log-cli-level=INFO     # mock: Tier 1 only, Tier 2 skips
pytest tests/command/nav_vtol_land/test_command.py --drone-address=udp://:14540 \
    --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_standard_vtol --vehicle-type=vtol --autopilot=px4 -v --log-cli-level=INFO
```

Run `test_command.py` and `test_flight.py` as separate sessions (Tier 1's disarmed probes shouldn't precede a flight).

## Results — 2026-10-13, PX4 SIH, multicopter and VTOL

**UNSUPPORTED** on both vehicles, on both builds tested: `px4io/px4-sitl:latest` (v1.18.0-beta1-660-g61907962c9) and PR #28764 (`17ed9c8c42`).
Tier 1: the baseline ACKs `UNSUPPORTED(3)` via COMMAND_INT and COMMAND_LONG (1 PASS — it does ACK), the other 21 checks skip. Tier 2: all 7 skip. Param coverage: COMPLETE (not applicable — command not supported).

PX4 implements NAV_VTOL_LAND only in the mission executor: `Commander::handle_command()` has no case for it, so it falls to the default `UNSUPPORTED`, and `mavlink_receiver.cpp`'s `command_has_location()` lists it under "Not supported by PX4 as COMMAND_INT (mission items or unimplemented)". A spec-aligned answer — the command is simply not offered — but `mavlink_command_params.hpp` still carries a command param mask for it (`{ 85, 0x7C, 0x7F }`), which nothing reaches.

The Tier 2 file has therefore not flown against a stack that accepts the command. The same flight code is exercised by `tests/command/nav_land/test_flight.py` (identical routine, different command id).
