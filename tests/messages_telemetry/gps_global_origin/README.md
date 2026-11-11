# GPS_GLOBAL_ORIGIN (message id 49)

[GPS_GLOBAL_ORIGIN](https://mavlink.io/en/messages/common.html#GPS_GLOBAL_ORIGIN): the global coordinates of the local frame's origin.
Listed as "ready to move" to standard.xml in the 2026-11-11 common→standard audit, with nothing needed first.

```bash
pytest tests/messages_telemetry/gps_global_origin/ --log-cli-level=INFO \
  --drone-address=udp://:14540 --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx \
  --vehicle-type=quadcopter --autopilot=px4          # or any ArduPilot row from the root CLAUDE.md
```

About 60 s per vehicle. Nothing flies.

## What the XML defines

- **Message**: "Publishes the GPS coordinates of the vehicle local origin (0,0,0) position. Emitted whenever a new GPS-Local position mapping is requested or set - e.g. following SET_GPS_GLOBAL_ORIGIN message."
- **`latitude`/`longitude`** (degE7, WGS84), **`altitude`** (mm, MSL). No invalid values.
- **`time_usec`** (extension): "UNIX Epoch time or time since system boot".

"Emitted whenever … set" describes an event, not a stream; the XML doesn't say whether it is also streamed, or whether SET_MESSAGE_INTERVAL applies.

## Tests

| Test | Kind | Checks |
|---|---|---|
| `obs_default_rate`, `compat_set_interval_*`, `compat_request_message` | obs / compat | Generic emission tests (`../emission.py`) |
| `info_origin_plus_local_is_global` | info | origin + LOCAL_POSITION_NED = GLOBAL_POSITION_INT's position (±1 m) |
| `info_constant_at_rest` | info | The origin doesn't move while the vehicle sits still |
| `obs_time_usec` | obs | UNIX or boot time; send time or set time. Fails only if neither is plausible |

The first check is absolute: any unit, axis or altitude-reference error in the origin moves the result by far more than 1 m, so lat/lon/alt are recorded supported when it passes. A stack that sends this message only on change gives the field tests too few samples from a rate window; the window then asks for three copies with REQUEST_MESSAGE (the detail says so).

<!-- RESULTS -->
