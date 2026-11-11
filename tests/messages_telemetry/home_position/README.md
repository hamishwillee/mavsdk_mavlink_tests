# HOME_POSITION (message id 242)

[HOME_POSITION](https://mavlink.io/en/messages/common.html#HOME_POSITION): the position the vehicle returns to, in global and local coordinates, with the landing surface's orientation and an approach vector.
Listed as "ready to move" to standard.xml in the 2026-11-11 common→standard audit, after an ArduPilot fork description sync.

```bash
pytest tests/messages_telemetry/home_position/ --log-cli-level=INFO \
  --drone-address=udp://:14540 --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx \
  --vehicle-type=quadcopter --autopilot=px4          # or any ArduPilot row from the root CLAUDE.md
```

About 70 s per vehicle. Nothing flies.

## What the XML defines

- **Message**: home "must be set automatically by the system during the takeoff, and may also be explicitly set using MAV_CMD_DO_SET_HOME"; q "encodes the orientation of the surface"; "can be requested by sending the MAV_CMD_REQUEST_MESSAGE with param1=242".
- **`latitude`/`longitude`** (degE7, WGS84), **`altitude`** (mm, MSL).
- **`x`/`y`/`z`** (m): "Local X/Y/Z position of this position in the local coordinate frame (NED)". No invalid value.
- **`q`** (`float[4]`, `invalid="[NaN]"`): world-to-surface-normal and heading of the takeoff position; "All fields should be set to NaN if an accurate quaternion for both heading and surface slope cannot be supplied."
- **`approach_x/y/z`** (m): the end of the approach vector, with per-vehicle-type guidance. No invalid value.
- **`time_usec`** (extension): "UNIX Epoch time or time since system boot", told apart by magnitude.

Silent on: the local frame's origin, what x/y/z hold without a local frame, how to say approach_* isn't provided, and whether time_usec is when home was set or when the message was sent.

## Tests

| Test | Kind | Checks |
|---|---|---|
| `obs_default_rate`, `compat_set_interval_*`, `compat_request_message` | obs / compat | Generic emission tests (`../emission.py`). On ArduPilot the harness requests HOME_POSITION at 1 Hz, so the unrequested rate is measured after `SET_MESSAGE_INTERVAL 0` |
| `info_global_is_vehicle_position` | info | The vehicle hasn't moved since boot, so home's lat/lon/alt are within 5 m of GLOBAL_POSITION_INT, and its `relative_alt` ("above home") equals its `alt` minus home's altitude (±0.5 m) |
| `info_local_matches_origin` | info | x/y/z = home relative to GPS_GLOBAL_ORIGIN (±1 m); characterises NaN/0 |
| `compat_q` | compat | All NaN (not supplied) or a unit quaternion; characterises which, and its angles |
| `obs_approach` | obs | What approach_x/y/z hold |
| `obs_time_usec` | obs | UNIX or boot time; send time or set time. Fails only if neither is plausible |

The relative_alt cross-check is what pins `altitude` down: it's in mm and MSL only if GLOBAL_POSITION_INT's alt minus it is the height above home. x/y/z are recorded supported only when home is ≥ 5 m from the origin (otherwise ~0 either way: untestable).

<!-- RESULTS -->
