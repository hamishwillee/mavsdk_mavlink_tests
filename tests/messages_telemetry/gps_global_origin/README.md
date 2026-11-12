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

## Results (2026-11-12)

PX4: `px4io/px4-sitl:latest` (1.18.0-beta, SIH, built 2026-09-11). ArduPilot: 4.8.0-dev SITL.

| | PX4 MC / FW / VTOL / tailsitter | ArduCopter / Rover | ArduPlane / QuadPlane |
|---|---|---|---|
| Unrequested | **not sent** (sent once, when the origin is set) — but MESSAGE_INTERVAL says 1 000 000 (1 Hz) | **not sent** (MESSAGE_INTERVAL −1) | every ~6 s: Plane re-sets home while disarmed, which re-sends HOME_POSITION and GPS_GLOBAL_ORIGIN (MESSAGE_INTERVAL −1) |
| SET_MESSAGE_INTERVAL 2/10 Hz | **ACCEPTED, nothing sent — compatibility error** | honoured | honoured |
| SET −1, 0 | NA (can't make it stream) | honoured | honoured (−1 leaves the event copies) |
| REQUEST_MESSAGE | one message, ~56 ms | one message, ~57–61 ms | one message, ~53–63 ms |
| origin + LOCAL_POSITION_NED vs GLOBAL_POSITION_INT | ≤ 0.02 m | ≤ 0.01 m | ≤ 0.01 m |
| Constant at rest | yes | yes | yes |
| `time_usec` | boot time **when the origin was set** (0.6 s), constant | boot time **at send** | boot time at send |

The PX4 failure is the generic SET_MESSAGE_INTERVAL check (rule 4a): the request is accepted and never honoured, because the stream only sends when the origin changes.

## Flight tests (2026-11-12)

The shared flight (`../flight.py`; `pytest tests/messages_telemetry -m flight …`, ~3–6 min per vehicle including a SITL restart) flies north, east, south and west at 30–50 m, lands away from home, and arms once on the ground there. Flown 2026-11-12 on the same builds as the results above: PX4 SIH multicopter, fixed-wing, standard VTOL and tailsitter; ArduCopter, ArduPlane, QuadPlane, Rover (Rover drives the route).

| Test | Kind | Checks |
|---|---|---|
| `obs_changes_in_flight` | obs | Whether the origin changes from arming to the end of the flight (the XML allows it: "emitted whenever a new GPS-Local position mapping is requested or set") |

Constant on all eight vehicles through the flight, the landing 40–448 m away and the re-arm there (25–95 copies each). LOCAL_POSITION_NED's flight test uses the origin current at each sample, so it would have tolerated a change.

## How the stacks implement it (source, 2026-11-12)

PX4 `36cedabb02` (a board fork of main; every stream file cited is identical in upstream main `e192036345`, 2026-11-11; line numbers are at `36cedabb02`). ArduPilot `ce9c2d01a4`.

| | PX4 (`streams/GPS_GLOBAL_ORIGIN.hpp`) | ArduPilot (`GCS_Common.cpp:3136-3148`) |
|---|---|---|
| When sent | Listed at 1 Hz in most link modes (0.1 Hz low-bandwidth; not in osd/minimal), but **sends only the first time and when the origin moves** by > 1e-7° or 1 mm (:80-92). REQUEST_MESSAGE forces a send (:59-62) | Not streamed. Sent when the origin is first set (`AP_AHRS.cpp:513`), with every home set (`:1999`), after SET_GPS_GLOBAL_ORIGIN/DO_SET_GLOBAL_ORIGIN (`GCS_Common.cpp:3958`), on GET_HOME_POSITION, and on request |
| No origin | not sent; REQUEST_MESSAGE DENIED (:82) | not sent (:3139); REQUEST_MESSAGE still ACCEPTED |
| altitude | `ref_alt` (AMSL) | origin altitude (AMSL) |
| time_usec | time the origin was set, µs since boot (:96) | `micros64()` at send |
| Setting it | SET_GPS_GLOBAL_ORIGIN → EKF2 accepts at any time | only if there's no origin yet; otherwise FAILED |

Every result matches this.

## Definition review

The stacks are compatible in content: both publish the EKF origin, in AMSL, and the test shows origin + local = global to centimetres on every vehicle. Two places where a consumer could be surprised, both about the "on change" nature of the message:

1. **Emission**: "Emitted whenever a new GPS-Local position mapping is requested or set" is right for both, but PX4 also reports a stream interval (1 Hz) for it while sending nothing, and accepts a rate it doesn't honour. That's a PX4 issue (MESSAGE_INTERVAL and SET_MESSAGE_INTERVAL for an on-change stream), not an XML one. Worth adding "It can also be requested with MAV_CMD_REQUEST_MESSAGE", which both support.
2. **`time_usec`**: PX4 sends when the origin was set, ArduPilot the send time. The XML doesn't say which; for an origin, "time the origin was set" is the useful value. Not proposed now (no consumer was found that reads it).

Not recommended: invalid values. Neither stack sends the message without an origin.
