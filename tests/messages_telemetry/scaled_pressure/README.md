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

<!-- RESULTS -->
