# SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 (message ids 29, 137, 143)

[SCALED_PRESSURE](https://mavlink.io/en/messages/common.html#SCALED_PRESSURE) / [2](https://mavlink.io/en/messages/common.html#SCALED_PRESSURE2) / [3](https://mavlink.io/en/messages/common.html#SCALED_PRESSURE3): the first, second and third barometer, with a differential (airspeed) pressure. Identical fields; one test module per message, sharing `pressure_checks.py`.
Listed as "ready to move" to standard.xml in the 2026-11-11 common→standard audit, with nothing needed first.

```bash
pytest tests/messages_telemetry/scaled_pressure/ --log-cli-level=INFO \
  --drone-address=udp://:14540 --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx \
  --vehicle-type=quadcopter --autopilot=px4          # or any ArduPilot row from the root CLAUDE.md
```

About 3 × 55 s per vehicle. Nothing flies.

## What the XML defines

- **Message**: "The pressure readings for the typical setup of one absolute and differential pressure sensor" (2/3: "Barometer readings for 2nd/3rd barometer").
- **`press_abs`** (float, hPa), **`press_diff`** (float, hPa; "Differential pressure 1" on SCALED_PRESSURE, "Differential pressure" on 2/3), **`time_boot_ms`**.
- **`temperature`** (int16, cdegC): "Absolute pressure temperature". No invalid value.
- **`temperature_press_diff`** (extension, cdegC, `invalid="0"`): "Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC."

Silent on: what press_diff holds with no differential-pressure sensor, and what `temperature` holds without a barometer temperature.

## Tests

| Test | Kind | Checks |
|---|---|---|
| `obs_default_rate`, `compat_set_interval_*`, `compat_request_message` | obs / compat | Generic emission tests (`../emission.py`) |
| `compat_boot_ms_tracks_real_time` | compat | `time_boot_ms` never decreases or sticks; advances at host rate ±5%, or with the sender's own boot clock |
| `compat_press_abs_matches_altitude` | compat | The ISA pressure at GLOBAL_POSITION_INT's MSL altitude (±10 hPa, ~80 m): Pa or kPa would be 100× off |
| `info_press_diff_at_rest` | info | Finite and ≤ 1 hPa (~13 m/s) at rest with no wind; always exactly 0 is characterised as "no sensor" |
| `compat_temperatures` | compat | `temperature` and `temperature_press_diff`: 0 (characterised) or −40..85 °C |

press_diff needs airflow to be shown right: recorded untestable. A message that's never sent (no such barometer) gives every field "untestable: not sent".

## Results (2026-11-12)

PX4: `px4io/px4-sitl:latest` (1.18.0-beta, SIH, built 2026-09-11). ArduPilot: 4.8.0-dev SITL. No field value contradicts the definition on any vehicle.

| | PX4 (all four SIH models) | ArduCopter | ArduPlane / QuadPlane | Rover |
|---|---|---|---|---|
| Instances sent | **SCALED_PRESSURE only** | SCALED_PRESSURE, PRESSURE2 | same | same |
| Unrequested | SCALED_PRESSURE 0.98 Hz (`onboard`: 1 Hz); 2/3 none | none | 1/2 at 1 Hz (`SRn_RAW_SENS`) | 1/2 at 1 Hz |
| Absent instance (PX4: 2, 3; ArduPilot: 3) SET_MESSAGE_INTERVAL | **ACCEPTED, never sent — compatibility error** | **same** | **same** | **same** |
| Absent instance REQUEST_MESSAGE | DENIED (a legitimate NACK) | **ACCEPTED, nothing sent — compatibility error** | **same** | **same** |
| `press_abs` vs ISA at the MSL altitude | 955.79 hPa at 489 m (ISA 955.82) | 1013.24 hPa at 0 m (ISA 1013.24) | same | same |
| `press_diff` at rest | always 0 (no sensor in SIH) | always 0 (no airspeed sensor) | **0.000–0.012 hPa** (analog airspeed, SCALED_PRESSURE only); PRESSURE2 0 | always 0 |
| `temperature` | 11.82 °C | 35.0 °C | 35.0 °C | 35.0 °C |
| `temperature_press_diff` | always 0 | always 0 | always 0 (analog sensor has no temperature) | always 0 |

So the barometer is exactly as defined on both stacks (ISA from the SITL's altitude, to 0.03 hPa). The absent-instance failures are request handling, recorded against SET_MESSAGE_INTERVAL/REQUEST_MESSAGE, not against the message's fields.

## How the stacks implement it (source, 2026-11-12)

PX4 `36cedabb02` (a board fork of main; every stream file cited is identical in upstream main `e192036345`, 2026-11-11; line numbers are at `36cedabb02`). ArduPilot `ce9c2d01a4`.

| | PX4 (`streams/SCALED_PRESSURE*.hpp`) | ArduPilot (`GCS_Common.cpp:2388-2447`) |
|---|---|---|
| Instance | `sensor_baro` + `differential_pressure` instance 0/1/2 | baro instance i, airspeed instance i; sent if either exists |
| Default rate | SCALED_PRESSURE 1 Hz in normal/onboard/osd/config; PRESSURE2 1 Hz config only; PRESSURE3 none | all three in `SRn_RAW_SENS`: Copter 0, Plane/Rover 1 Hz |
| press_abs | Pa × 0.01 | Pa × 0.01 |
| press_diff | Pa × 0.01; **0 without a differential-pressure topic** | airspeed differential pressure if `ARSPDn_TYPE` ≠ NONE (health not checked); else 0 |
| temperature | round(°C × 100); no 0 → 1 mapping; NaN would be cast | °C × 100, truncated; no 0 → 1 mapping |
| temperature_press_diff | round(°C × 100); **no 0 → 1 mapping**; NaN = "unknown" in the topic, cast to int16 | **0 → 1**; 0 if the sensor has no temperature |
| Absent instance | SET ACCEPTED (any known stream); REQUEST DENIED | SET ACCEPTED; REQUEST ACCEPTED, nothing sent |

Consumers: QGC shows only `temperature / 100` from all three (`VehicleTemperatureFactGroup.cc:63-91`), with no 0 check. Mission Planner stores `temperature` and `temperature_press_diff` **raw, in cdegC**, in properties HIGH_LATENCY fills in °C (`CurrentState.cs:3272-3292`, :2490). MAVProxy uses `press_abs` in hPa for QNH.

## Definition review

Compatible: both stacks send the same quantities in the defined units, and the same 0 for an absent differential-pressure sensor. Suggested XML change:

1. **SCALED_PRESSURE `press_diff`: "Differential pressure 1" → "Differential pressure"**, as SCALED_PRESSURE2/3 say.

Not proposed:
- An invalid value for `press_diff` (both send 0 with no sensor, and 0 is a real reading in still air) or `temperature` (both send 0 with no baro temperature). Adding one now would make both stacks non-conformant.
- PX4's missing 0 → 1 mapping on `temperature_press_diff`: a PX4 fix, and only visible at exactly 0.00 °C or with a NaN-reporting sensor.
