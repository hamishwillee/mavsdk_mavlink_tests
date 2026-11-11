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

## Results (2026-11-12)

PX4: `px4io/px4-sitl:latest` (1.18.0-beta, SIH, built 2026-09-11). ArduPilot: 4.8.0-dev SITL. 10/10 on every vehicle, no compatibility errors.

| | PX4 MC / FW / VTOL / tailsitter | ArduCopter / Rover | ArduPlane / QuadPlane |
|---|---|---|---|
| Unrequested | 0.49 Hz (`onboard` mode: 0.5 Hz) | **not sent** (MESSAGE_INTERVAL −1) | every ~6 s (Plane re-sets home while disarmed; MESSAGE_INTERVAL −1) |
| SET_MESSAGE_INTERVAL 2/10 Hz, −1, 0 | all honoured | all honoured | all honoured (−1 leaves the event copies) |
| REQUEST_MESSAGE | one message, ~56 ms | one message, ~57 ms | one message, ~57 ms |
| Home vs stationary vehicle | ≤ 0.09 m horizontal, ≤ 0.01 m vertical | ≤ 0.02 m, ≤ 0.08 m | ≤ 0.01 m, ≤ 0.13 m |
| `relative_alt` vs `alt` − home altitude | ≤ 0.001 m | ≤ 0.009 m | ≤ 0.009 m |
| x/y/z vs GPS_GLOBAL_ORIGIN | ≤ 0.01 m (home ~0.05 m from the origin) | 0.00 m (home at the origin) | 0.00 m |
| `q` | **unit quaternion**: the vehicle's attitude when home was set (roll/pitch ~0, yaw = heading) | **all NaN** | all NaN |
| `approach_x/y/z` | always 0 | always 0 | always 0 |
| `time_usec` | boot time **when home was set**, constant | boot time **at send** | boot time at send |

x/y/z are recorded untestable (home is at the origin on every SITL). approach_* are recorded untestable: always 0, and with no invalid value "not provided" can't be told from "at the origin".

## How the stacks implement it (source, 2026-11-12)

PX4 `36cedabb02` (a board fork of main; every stream file cited is identical in upstream main `e192036345`, 2026-11-11; line numbers are at `36cedabb02`). ArduPilot `ce9c2d01a4`.

| | PX4 (`streams/HOME_POSITION.hpp:60-98`) | ArduPilot (`GCS_Common.cpp:3104-3134`) |
|---|---|---|
| When sent | 0.5 Hz in normal/onboard/config/osd/low-bandwidth modes (0.1 Hz minimal); only once home is valid (:67), else REQUEST_MESSAGE DENIED | Not streamed: sent on every home set (`AP_AHRS.cpp:1998`, to all channels), MAV_CMD_GET_HOME_POSITION, or request. Plane re-sets home every 5 s while disarmed (`ArduPlane/Plane.cpp:395-403`) |
| Home set | at arming (landed) and takeoff (`commander/Commander.cpp:737-741`) | Copter: when the EKF has a position (`ArduCopter/commands.cpp:4-20`), and at arming |
| altitude | AMSL (`msg/HomePosition.msg:9`) | AMSL (`AP_AHRS.cpp:1978`) |
| x/y/z | home relative to the EKF origin; **0 when the local position is invalid** (:74-76) | home relative to the EKF origin; **NaN** without an origin (:3112-3119) |
| q | full vehicle attitude when home was set (`commander/HomePosition.cpp:181-188`); NaN if the attitude wasn't valid | always NaN (:3122) |
| approach_* | 0 (:86-88) | 0 (:3132) |
| time_usec | when home was set, µs since boot (:90) | `micros64()` at send (:3133) |

Consumers: QGC uses only latitude/longitude/altitude (`Vehicle.cc:1237-1251`) and requests HOME_POSITION from ArduPilot at 1 Hz itself.

## Definition review

Proposed upstream in [mavlink/mavlink#2677](https://github.com/mavlink/mavlink/pull/2677) (draft): items 1–2 below, with LOCAL_POSITION_NED's.

The global fields are compatible and exactly as defined: home is in WGS84 degE7 with an AMSL altitude in mm, consistent with GLOBAL_POSITION_INT's `relative_alt` on every vehicle. Suggested XML changes:

1. **x/y/z: state the frame's origin and an invalid value.** Both stacks give home relative to the EKF origin (GPS_GLOBAL_ORIGIN). Without a local frame ArduPilot sends NaN and PX4 0; 0 is also a valid position, so suggest `invalid="NaN"` with "NaN if there is no local origin". PX4 would need its 0 changed to NaN to conform — a small change, and no consumer found reads x/y/z.
2. **"set automatically … during the takeoff"**: both stacks set home earlier (PX4 at arming; ArduCopter as soon as it has a position, and again at arming; ArduPlane continuously while disarmed). Suggest "set automatically by the system (e.g. at arming or takeoff)".

Not proposed (no evidence to choose a convention, or no consumer):
- **q**: PX4 sends the vehicle's attitude at home-set time, which is the surface heading and slope only for a vehicle standing on that surface; ArduPilot sends NaN. Both are within the definition.
- **approach_***: both always send 0, which the XML can't distinguish from a real point at the origin. An invalid value (NaN) would be the fix, but would make both stacks non-conformant for a field no one uses.
- **time_usec**: PX4 sends when home was set, ArduPilot the send time; the XML allows either.
