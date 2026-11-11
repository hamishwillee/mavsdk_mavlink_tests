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

<!-- RESULTS -->
