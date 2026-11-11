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

## Results (2026-11-12)

PX4: `px4io/px4-sitl:latest` (1.18.0-beta, SIH, built 2026-09-11). ArduPilot: 4.8.0-dev SITL. 9/9 on every vehicle, no compatibility errors.

| | PX4 MC / FW / VTOL / tailsitter | ArduCopter / Plane / QuadPlane / Rover |
|---|---|---|
| Unrequested | **not sent** (MESSAGE_INTERVAL −1) | **not sent** (MESSAGE_INTERVAL −1) |
| SET_MESSAGE_INTERVAL 2/10 Hz, −1, 0 | all honoured | all honoured |
| REQUEST_MESSAGE | one message, ~56 ms | one message, ~56 ms |
| `time_boot_ms` rate vs host | ×0.977–0.980 (SIH lockstep) | ×0.9999–1.0004 |
| Disarmed, never armed | all times 0 | all times 0 |
| Armed (no takeoff) | `arming_time_utc` = the arming time; takeoff 0 | `arming_time_utc` = the arming time; takeoff 0 |
| After disarm | **arming and takeoff times back to 0** | **arming and takeoff times back to 0** |
| `flight_uuid` | a counter that **increments at every disarm** and persists across boots (10, 11, … here) | **always 0** |

## How the stacks implement it (source, 2026-11-12)

PX4 `36cedabb02` (a board fork of main; every stream file cited is identical in upstream main `e192036345`, 2026-11-11; line numbers are at `36cedabb02`). ArduPilot `ce9c2d01a4`.

| | PX4 (`streams/FLIGHT_INFORMATION.hpp`) | ArduPilot (`GCS_Common.cpp:6476-6523`) |
|---|---|---|
| When sent | on request or interval only | on request or interval only |
| arming_time_utc | `armed_time`, set at arming (`commander/Commander.cpp:743`), **0 after disarm** (:784) | `arm_time_us()`: last arming time, **0 when disarmed** (`AP_Arming.h:112`) |
| takeoff_time_utc | set when the land detector sees a takeoff (:2363); 0 after disarm (:787) | computed inside the send function from `landed_state()` changes seen at send time (:6486-6503), so it depends on when the message is sent; always 0 on Rover |
| flight_uuid | `COM_FLIGHT_UUID`, incremented at every disarm, with or without a takeoff (:804-806); the first flight is **0** | always 0 (:6513) |
| landing_time | never set (0) | 0, "not supported" (:6521) |

Every result matches this. Not exercised: takeoff and landing times (need a flight). From source, ArduPilot's takeoff time is only right if the message happens to be sent while the vehicle takes off — an ArduPilot implementation issue, not a definition one.

## Definition review

The stacks are compatible for what's testable: `arming_time_utc` is µs since boot at arming on both, and every unset value is 0. Suggested XML changes:

1. **Say what disarm does to `arming_time_utc`.** Both stacks set it back to 0 at disarm (tested on every vehicle), which the XML doesn't mention ("Set to 0 on boot. Set value on arming."). Proposed: "Set to 0 on boot and on disarm. Set value on arming." A consumer can then read a non-zero arming time as "armed now". (PX4 also zeroes `takeoff_time_utc` at disarm; ArduPilot's is computed per message, so that isn't proposed.)
2. **flight_uuid**: "Flight number" doesn't say what to send without support. PX4 counts disarms, persisted across reboots, starting at 0 — the invalid value, so PX4's first flight reads as "not supported"; ArduPilot always sends 0. Proposed: "Flight number (e.g. incremented for each flight), or 0 if not supported." PX4 starting at 0 is a minor PX4 issue.
