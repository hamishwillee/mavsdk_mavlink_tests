# SCALED_IMU, SCALED_IMU2, SCALED_IMU3 (message ids 26, 116, 129)

[SCALED_IMU](https://mavlink.io/en/messages/common.html#SCALED_IMU) / [2](https://mavlink.io/en/messages/common.html#SCALED_IMU2) / [3](https://mavlink.io/en/messages/common.html#SCALED_IMU3): the first, second and third IMU's accelerometer, gyro and magnetometer readings, scaled to integer units. Identical fields; one test module per message (one report each), sharing `imu_checks.py`.
Listed as "ready to move" to standard.xml in the 2026-11-11 common→standard audit, with nothing needed first.

```bash
pytest tests/messages_telemetry/scaled_imu/ --log-cli-level=INFO \
  --drone-address=udp://:14540 --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx \
  --vehicle-type=quadcopter --autopilot=px4          # or any ArduPilot row from the root CLAUDE.md
```

About 3 × 60 s per vehicle. Nothing flies.

## What the XML defines

- **Message**: "The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units" (2/3: "secondary"/"3rd 9DOF sensor setup").
- **`xacc`/`yacc`/`zacc`** (int16, mG), **`xgyro`/`ygyro`/`zgyro`** (int16, mrad/s, "Angular speed around X axis"), **`xmag`/`ymag`/`zmag`** (int16, mgauss), **`time_boot_ms`**.
- **`temperature`** (extension, cdegC): "0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C)." `invalid="0"` on SCALED_IMU2 and SCALED_IMU3, **but not on SCALED_IMU**.

Silent on: the frame (body or sensor), whether values are calibrated, what to send for a sensor an instance doesn't have (e.g. a magnetometer-only instance), and saturation.

## Tests

| Test | Kind | Checks |
|---|---|---|
| `obs_default_rate`, `compat_set_interval_*`, `compat_request_message` | obs / compat | Generic emission tests (`../emission.py`) |
| `compat_boot_ms_tracks_real_time` | compat | `time_boot_ms` never decreases or sticks; advances at host rate ±5%, or with the sender's own boot clock |
| `compat_accel_is_gravity` | compat | At rest the accelerometer reads specific force: (0, 0, −1 g) rotated into the body frame by ATTITUDE's roll/pitch, i.e. x = g·sinθ, y = −g·sinφ·cosθ, z = −g·cosφ·cosθ (±60 mG per axis, \|a\| ±5%) |
| `info_gyro_at_rest` | info | < 50 mrad/s on every axis |
| `info_mag_is_earth_field` | info | Rotated to the earth frame with ATTITUDE: 150–700 mgauss, inclination 30–80° down in the northern hemisphere, declination within ±25° |
| `compat_temperature` | compat | 0 (not provided) or −40..85 °C |

Field verdicts:
- An axis is recorded supported only when its expected value is large enough to show a swapped or flipped axis (≥ 100 mG, ≥ 100 mgauss); a level vehicle's x/y acceleration is ~0 either way (untestable). Gyro axes need rotation (untestable).
- An instance whose accel, gyro or mag is exactly 0 on every axis in every message is "not provided" — the XML has no invalid value for those fields — and recorded not supported with that note, not as a wrong value.
- A message that's never sent (no such IMU) gives every field "untestable: not sent".

<!-- RESULTS -->
