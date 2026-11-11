# FLIGHT_INFORMATION (message id 264)

[FLIGHT_INFORMATION](https://mavlink.io/en/messages/common.html#FLIGHT_INFORMATION): arming, takeoff and landing times (since boot) and a flight number.
Listed as "ready to move" to standard.xml in the 2026-11-11 common→standard audit, with nothing needed first.

```bash
pytest tests/messages_telemetry/flight_information/ --log-cli-level=INFO \
  --drone-address=udp://:14540 --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx \
  --vehicle-type=quadcopter --autopilot=px4          # or any ArduPilot row from the root CLAUDE.md
```

About 60 s per vehicle (plus up to 90 s for ArduPilot to become armable). The vehicle is **armed once, without taking off**, and disarmed.

## What the XML defines

- **Message**: "time since boot for arm, takeoff, and land, and a flight number. Takeoff and landing values reset to zero on arm. This can be requested using MAV_CMD_REQUEST_MESSAGE. Note, some fields are misnamed - timestamps are from boot (not UTC) and the flight_uuid is a sequence number."
- **`time_boot_ms`**.
- **`arming_time_utc`** (µs since boot, `invalid="0"`): "Set to 0 on boot. Set value on arming."
- **`takeoff_time_utc`** (µs since boot, `invalid="0"`): "Set to 0 at boot and on arming."
- **`flight_uuid`** (`invalid="0"`): "Flight number."
- **`landing_time`** (extension, **ms** since boot, `invalid="0"`): "Set to 0 at boot and on arming."

Silent on: what happens at disarm, and when flight_uuid changes (per arming, takeoff, or flight) and whether it persists across reboots.

## Tests

| Test | Kind | Checks |
|---|---|---|
| `obs_default_rate`, `compat_set_interval_*`, `compat_request_message` | obs / compat | Generic emission tests (`../emission.py`) |
| `compat_boot_ms_tracks_real_time` | compat | `time_boot_ms` never decreases or sticks; advances at host rate ±5%, or with the sender's own boot clock |
| `compat_times_in_the_past` | compat | Disarmed: every time is 0 or not later than `time_boot_ms` |
| `obs_flight_uuid` | obs | flight_uuid while disarmed |
| `compat_arming_time_set_on_arm` | compat | Arm (no takeoff): `arming_time_utc` is the arming time (between the last pre-arm `time_boot_ms` and the armed message's). Then disarm, and characterise what each field and flight_uuid do |

takeoff_time_utc and landing_time need a flight: recorded untestable. NA in mock mode (the mock can't arm).

<!-- RESULTS -->
