# LOCAL_POSITION_NED (message id 32)

[LOCAL_POSITION_NED](https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED): the vehicle's filtered position and velocity in the local NED frame.
Listed as "ready to move" to standard.xml in the 2026-11-11 common→standard audit, with nothing needed first.

```bash
pytest tests/messages_telemetry/local_position_ned/ --log-cli-level=INFO \
  --drone-address=udp://:14540 --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx \
  --vehicle-type=quadcopter --autopilot=px4          # or any ArduPilot row from the root CLAUDE.md
```

About 60 s per vehicle. Nothing flies.

## What the XML defines

- **Message**: "The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)".
- **`x`/`y`/`z`** (m), **`vx`/`vy`/`vz`** (m/s), **`time_boot_ms`**. No invalid values, no min/max.

Silent on: the origin of the frame (GPS_GLOBAL_ORIGIN "publishes the GPS coordinates of the vehicle local origin", but this message doesn't refer to it), and what to send, if anything, while the estimate is invalid.

## Tests

| Test | Kind | Checks |
|---|---|---|
| `obs_default_rate`, `compat_set_interval_*`, `compat_request_message` | obs / compat | Generic emission tests (`../emission.py`) |
| `compat_boot_ms_tracks_real_time` | compat | `time_boot_ms` never decreases or sticks; advances at host rate ±5%, or with the sender's own boot clock |
| `info_position_matches_global` | info | x/y/z = GLOBAL_POSITION_INT's position relative to GPS_GLOBAL_ORIGIN (±1 m) |
| `info_velocity_at_rest` | info | vx/vy/vz finite, < 0.5 m/s, equal to GLOBAL_POSITION_INT's (±0.3 m/s) |

Field verdicts: a vehicle standing at its origin reads x/y ≈ 0 whichever way round the axes are, so a position axis is recorded supported only when it's ≥ 5 m from the origin, and untestable otherwise. Velocities need movement: untestable. The mock puts its origin 40 m S / 30 m W, so the strong path runs there.

## Results (2026-11-12)

PX4: `px4io/px4-sitl:latest` (1.18.0-beta, SIH, built 2026-09-11). ArduPilot: 4.8.0-dev SITL. 8/8 on every vehicle, no compatibility errors.

| | PX4 MC / FW / VTOL / tailsitter | ArduCopter | ArduPlane / QuadPlane / Rover |
|---|---|---|---|
| Unrequested (this link) | 29 Hz (`onboard` mode: 30 Hz)¹ | **not sent** (MESSAGE_INTERVAL −1) | 1.00 Hz |
| SET_MESSAGE_INTERVAL 2/10 Hz, −1, 0 | all honoured | all honoured | all honoured |
| REQUEST_MESSAGE | one message, ~57 ms | one message, ~56 ms | one message, ~57 ms |
| vs GLOBAL_POSITION_INT relative to GPS_GLOBAL_ORIGIN (worst) | ≤ 0.02 m | ≤ 0.01 m | ≤ 0.01 m |
| Position at rest | within 0.05 m of the origin | within 0.02 m | within 0.06 m |
| Speed at rest; vs GLOBAL_POSITION_INT | ≤ 0.06 m/s; ≤ 0.012 m/s | ≤ 0.03 m/s; ≤ 0.01 m/s | ≤ 0.04 m/s; ≤ 0.011 m/s |

¹ PX4's default depends on the link's MAVLink mode (below).

Both stacks set their origin where the vehicle boots, so x/y/z are ~0 and their axes are recorded **untestable** (the check can't tell a swapped axis at the origin). The mock (origin 40 m S / 30 m W) exercises the strong path.

## How the stacks implement it (source, 2026-11-12)

PX4 `36cedabb02` (a board fork of main; every stream file cited is identical in upstream main `e192036345`, 2026-11-11; line numbers are at `36cedabb02`). ArduPilot `ce9c2d01a4`.

| | PX4 (`streams/LOCAL_POSITION_NED.hpp`) | ArduPilot (`GCS_Common.cpp:3044-3065`) |
|---|---|---|
| Source | `vehicle_local_position` | `AP_AHRS::get_relative_position_NED_origin()`, velocity NED |
| Origin | EKF origin (`ref_lat/lon/alt`) — what GPS_GLOBAL_ORIGIN sends | EKF origin — what GPS_GLOBAL_ORIGIN sends |
| Estimate invalid | **sent anyway** (no `xy_valid`/`z_valid` check) | **not sent** unless position NE, D and velocity are valid (:3049-3052) |
| `time_boot_ms` | the estimate's own timestamp (:67) | `millis()` at send |
| Default rate | per link mode (`mavlink_main.cpp`): normal 1 Hz, onboard/config/onboard-low-bandwidth 30 Hz, low-bandwidth 1 Hz | `SRn_POSITION` group: Copter 0, Plane/Rover 1 Hz |

Every result matches this. Not exercised: PX4 sending with an invalid estimate (needs a lost position).

## Definition review

The stacks are compatible: both send the EKF position relative to the same origin GPS_GLOBAL_ORIGIN reports, in NED, and a consumer needs no autopilot-specific code. Suggested XML change:

1. **State the origin**: "… relative to the local origin (see GPS_GLOBAL_ORIGIN)". Both stacks already do this; the test above shows the two messages and GLOBAL_POSITION_INT agree to centimetres. Proposed upstream with the other local-frame clarifications (HOME_POSITION).

Not recommended: an invalid value or a validity rule. PX4 sends while the estimate is flagged invalid and ArduPilot stops; neither sends NaN, and there's no field for a validity flag — consumers that care use ESTIMATOR_STATUS.
