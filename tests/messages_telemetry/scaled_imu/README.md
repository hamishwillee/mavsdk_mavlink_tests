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

## Results (2026-11-12)

PX4: `px4io/px4-sitl:latest` (1.18.0-beta, SIH, built 2026-09-11). ArduPilot: 4.8.0-dev SITL. No field value contradicts the definition on any vehicle.

| | PX4 (all four SIH models) | ArduCopter | ArduPlane / QuadPlane / Rover |
|---|---|---|---|
| Instances sent | **SCALED_IMU only** (one IMU + mag) | SCALED_IMU, IMU2; IMU3 **mag only** | same as Copter |
| Unrequested | none of the three (`onboard` mode streams them only on USB/config links) | none | SCALED_IMU none; IMU2 and IMU3 1 Hz (`SRn_RAW_SENS`) |
| SCALED_IMU2/3 SET_MESSAGE_INTERVAL | **ACCEPTED, never sent — compatibility error** | honoured | honoured |
| SCALED_IMU2/3 REQUEST_MESSAGE | DENIED (a legitimate NACK) | one message | one message |
| SCALED_IMU (and present instances): rates, request | honoured; one message ~56 ms | honoured; one message | honoured; one message |
| Accel at rest | −999..−1000 mG on z, x/y within 3 mG of ATTITUDE's expectation | −1001 mG on z, x/y ≤ 1 mG | same |
| Gyro at rest | ≤ 27 mrad/s (SIH noise) | ≤ 4 mrad/s | ≤ 4 mrad/s |
| Mag (NED) | 479–484 mgauss, inclination +63°, declination +3–4° | 481 mgauss, +63°, +3–4° | 481–498, +63–66°, +3–4° |
| Mag at the SITL heading | yaw ~0°: x ≈ +215 (north) | yaw ~271°: y ≈ +217 (north is to the right) | same |
| temperature | 15.00 °C (SIH fixed) | IMU1 36 °C, IMU2 37.5 °C; **IMU3 0** (no IMU, "not provided") | same |

Two measurements back up the frame: with PX4 facing north and ArduPilot facing west, both magnetometers give the same NED field (declination ~4°, inclination ~63°), and both accelerometers give −1 g on z. So acc and mag are body frame FRD on both stacks in SITL (the XML doesn't say).

**ArduPilot SCALED_IMU3 is a magnetometer-only message**: acc, gyro and temperature exactly 0 in every message. The XML gives acc/gyro/mag no "not provided" value, so a consumer can't tell this from a level, still, perfectly calibrated IMU at 0 g — except that 0 g at rest is impossible.

## How the stacks implement it (source, 2026-11-12)

PX4 `36cedabb02` (a board fork of main; every stream file cited is identical in upstream main `e192036345`, 2026-11-11; line numbers are at `36cedabb02`). ArduPilot `ce9c2d01a4`.

| | PX4 (`streams/SCALED_IMU*.hpp`) | ArduPilot (`GCS_Common.cpp` `send_scaled_imu`, :2339-2382) |
|---|---|---|
| Instance | `vehicle_imu` / `sensor_mag` instance 0/1/2 | INS accel/gyro instance and compass (by **priority**) 0/1/2; sent if *any* of the three exists |
| Default rate | all three 25 Hz, **config (USB) mode only** (`mavlink_main.cpp:2053-2055`) | SCALED_IMU in no group (RAW_IMU carries instance 0); IMU2/IMU3 in `SRn_RAW_SENS`: Copter 0, Plane/Rover 1 Hz |
| acc / gyro | integrated delta-velocity/-angle, calibrated, body FRD | `get_accel(i)`/`get_gyro(i)`, calibrated, board rotation applied |
| mag | `sensor_mag`: **uncalibrated, no board rotation** (driver rotation only) | `compass.get_field(i)`: **calibrated, body frame** |
| Missing sensor | 0 | 0 |
| temperature | accel, else gyro, else mag; **0 → 1** (:126-134) | accel instance only; **no 0 → 1 mapping** |
| SET_MESSAGE_INTERVAL for an absent instance | **ACCEPTED** (any known stream, `mavlink_receiver.cpp:2617-2662`), never sent | n/a in SITL (every instance has a compass) |
| REQUEST_MESSAGE for an absent instance | DENIED (`mavlink_receiver.cpp:941-983`) | ACCEPTED |
| time_boot_ms | sample timestamp | `millis()` at send |

Consumers: Mission Planner stores raw acc/gyro/mag and temperature/100 (`CurrentState.cs:3782-3843`); MAVProxy's `magsetfield` treats SCALED_IMU2/3 mag as uncalibrated (subtracting `COMPASS_OFS2/3`), which is wrong for ArduPilot's already-corrected values (inferred). QGC doesn't read these messages.

## Definition review

Proposed upstream in [mavlink/mavlink#2675](https://github.com/mavlink/mavlink/pull/2675) (draft): items 1–2 below, with SCALED_PRESSURE's wording fix.

Compatible for what SITL shows. The differences are things the XML leaves open, and PX4's mag being uncalibrated/sensor-frame on hardware is the one a consumer would actually notice (in SIH, sensor = body). Suggested XML changes:

1. **SCALED_IMU `temperature`: add `invalid="0"`**, as SCALED_IMU2/3 already have. The description is identical in all three; only the attribute is missing.
2. **acc/gyro: say "body frame (FRD)"** — both stacks send calibrated body-frame values. Not for mag: PX4 sends raw, uncalibrated sensor-frame mag (`PX4Magnetometer.cpp:96-101`), ArduPilot calibrated body-frame. Stating one would make the other non-conformant; worth an issue to decide which (calibrated body frame is what the name "scaled" and RAW_IMU's existence suggest).

Not proposed now:
- An invalid value for absent acc/gyro/mag (ArduPilot's mag-only SCALED_IMU3). INT16_MAX would be the natural choice, but both stacks send 0 today.
- ArduPilot's missing 0 → 1 temperature mapping: only matters at exactly 0.00 °C; an ArduPilot fix, not an XML one.
