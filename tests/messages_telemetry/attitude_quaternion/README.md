# ATTITUDE_QUATERNION (message id 31)

[ATTITUDE_QUATERNION](https://mavlink.io/en/messages/common.html#ATTITUDE_QUATERNION): the vehicle attitude as a quaternion, plus body rates and a display-offset rotation.
Listed as "ready to move" to standard.xml in the 2026-11-11 common→standard audit, with nothing needed first.

```bash
pytest tests/messages_telemetry/attitude_quaternion/ --log-cli-level=INFO \
  --drone-address=udp://:14540 --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx \
  --vehicle-type=quadcopter --autopilot=px4          # or any ArduPilot row from the root CLAUDE.md
```

About 60 s per vehicle. Nothing flies.

## What the XML defines

- **Message**: "The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0)."
- **`time_boot_ms`** (ms): "Timestamp (time since system boot)".
- **`q1`..`q4`**: w, x, y, z. No invalid value.
- **`rollspeed`, `pitchspeed`, `yawspeed`** (rad/s): "Roll/Pitch/Yaw angular speed", the same wording as ATTITUDE. No invalid value.
- **`repr_offset_q`** (extension, `float[4]`, `invalid="[0]"`): a rotation to apply "for user display". `[0,0,0,0]` "if field not supported". The XML's example is a tailsitter: `[1,0,0,0]` in hover, `[0.7071,0,0.7071,0]` in fixed-wing.

Silent on: the reference frame the attitude is relative to (NED is implied, not stated), whether the rates are body-frame, what `time_boot_ms` timestamps (the estimate or the send), and what to send when the attitude is unknown.

## Tests

| Test | Kind | Checks |
|---|---|---|
| `obs_default_rate`, `compat_set_interval_*`, `compat_request_message` | obs / compat | Generic emission tests (`../emission.py`) |
| `compat_boot_ms_tracks_real_time` | compat | `time_boot_ms` never decreases or sticks; advances at host rate ±5% |
| `compat_q_is_unit` | compat | q1..q4 finite and \|q\| = 1 ± 1e-3: a quaternion that isn't a rotation contradicts "attitude … expressed as quaternion" |
| `info_q_matches_attitude` | info | q as ZYX Euler angles matches ATTITUDE (±1°), and its yaw GLOBAL_POSITION_INT's `hdg` (±2°) |
| `info_rates_at_rest` | info | Rates finite, < 0.2 rad/s at rest, and equal to ATTITUDE's (±0.05 rad/s, gyro noise) |
| `compat_repr_offset_q` | compat | `[0,0,0,0]` or a unit quaternion; characterises which |

The Euler comparison catches swapped components at any attitude (≥90° error in an offline check). A conjugated (sign-flipped) quaternion shows up as twice the yaw, so it is only caught when the yaw is away from 0/180°. Each run's detail says how strong its check was.

Field verdicts:
- **Quaternion**: recorded as supported only when the Euler comparison passes.
- **Rates**: **untestable** at rest (needs rotation; shown by the flight tests below). At rest the test only shows them finite, small and identical to ATTITUDE's, which says nothing about units or axes.
- **`repr_offset_q`**: when only `[0,0,0,0]` arrives, it's untestable. PX4 sends that for "no offset" too (below), so it isn't evidence of "not supported".

## Results (2026-11-11)

PX4: `px4io/px4-sitl:latest` (1.18.0-beta, SIH, built 2026-09-11). ArduPilot: 4.8.0-dev (`70fe7125`) SITL. 10/10 on every vehicle, no compatibility errors.

| | PX4 MC / FW / VTOL / tailsitter (xvert) | ArduCopter / Plane / QuadPlane / Rover |
|---|---|---|
| Unrequested (this link) | 49 Hz (`onboard` mode: 50 Hz)¹ | **not sent** (MESSAGE_INTERVAL −1) |
| SET_MESSAGE_INTERVAL 2/10 Hz, −1, 0 | all honoured | all honoured |
| REQUEST_MESSAGE | one message, ~57 ms | one message, 65–89 ms |
| `time_boot_ms` rate vs host | ×0.979–0.981 (SIH lockstep, see SYSTEM_TIME) | ×0.9999–1.0004 |
| \|q\| − 1 (worst) | ≤1.2e-7 | ≤4e-8 |
| vs ATTITUDE (worst r/p/y) | ≤0.01° | 0.00° |
| vs GLOBAL_POSITION_INT.hdg | ≤0.04° | 0.01° |
| Yaw at rest (check strength) | 0.4–1.5° (order only) | Plane/QuadPlane ~270° (**sign + order**); Copter 2.3°, Rover 0.7° (order only) |
| Rates at rest | ≤0.02 rad/s, gyro noise; ATTITUDE's within 0.018 | ≤0.004 rad/s; identical to ATTITUDE's |
| `repr_offset_q` | `[0,0,0,0]` on the ground, xvert included (it's in hover mode) | `[0,0,0,0]` |

¹ PX4's default depends on the link's MAVLink mode (below).

## Flight tests (2026-11-12)

The shared flight (`../flight.py`; `pytest tests/messages_telemetry -m flight …`, ~3–6 min per vehicle including a SITL restart) flies north, east, south and west at 30–50 m, lands away from home, and arms once on the ground there. Flown 2026-11-12 on the same builds as the results above: PX4 SIH multicopter, fixed-wing, standard VTOL and tailsitter; ArduCopter, ArduPlane, QuadPlane, Rover (Rover drives the route).

| Test | Kind | Checks |
|---|---|---|
| `info_q_matches_attitude_in_flight` | info | q as ZYX Euler angles matches ATTITUDE (±1°, ATTITUDE interpolated to each sample's `time_boot_ms`) on every heading flown; supported once the headings cover ≥ 90° |
| `info_rates_match_attitude_in_flight` | info | rollspeed/pitchspeed/yawspeed equal ATTITUDE's (±0.05 rad/s or 10%) where \|rate\| ≥ 0.15 rad/s; supported per axis with ≥ 5 such samples |
| `compat_repr_offset_q_in_flight` | compat | `[0,0,0,0]` or a unit quaternion throughout; characterises what's sent in forward flight |

| | PX4 MC / FW / VTOL / tailsitter | ArduCopter / Plane / QuadPlane / Rover |
|---|---|---|
| q vs ATTITUDE | 100% within 1° (worst ≤ 0.1°), headings covering 273–358° | 100% within 1° (worst ≤ 0.2°), headings covering 353–355° |
| Rates vs ATTITUDE | 96–100% on every axis, 47–364 samples per axis | 100% on every axis, 26–176 samples per axis (Rover: yaw only) |
| repr_offset_q | MC, FW, standard VTOL: always `[0,0,0,0]`. **Tailsitter: `[0.707, 0, 0.707, 0]` (a 90° pitch offset) for most of forward flight**, `[0,0,0,0]` in hover | always `[0,0,0,0]`, QuadPlane in forward flight too |

All three fields' axes and signs are now shown right on every vehicle: the quaternion, the rates (recorded supported), and the tailsitter's display offset (repr_offset_q supported on PX4 tailsitter). The tailsitter's worst Euler error is 43° roll/yaw on a few samples at the gimbal-lock pitch of the transition (ZYX Euler angles aren't defined at ±90° pitch), which doesn't affect the 1° check's 100%.

## How the stacks implement it (source, 2026-11-11)

PX4 `cd0e647af5` (main). ArduPilot `ce9c2d01a4`.

| | PX4 (`streams/ATTITUDE_QUATERNION.hpp`) | ArduPilot (`GCS_Common.cpp` `send_attitude_quaternion()`) |
|---|---|---|
| Source | `vehicle_attitude.q` (the estimator's attitude); ATTITUDE is derived from the same quaternion | `AP_AHRS::get_quaternion()`; ATTITUDE from the same AHRS (`get_roll/pitch/yaw_rad`) |
| `time_boot_ms` | the attitude estimate's own timestamp | `AP_HAL::millis()` at send |
| Rates | `vehicle_angular_velocity` (body frame), as ATTITUDE | `AP_AHRS::get_gyro()` (body frame), as ATTITUDE |
| `repr_offset_q` | 90° pitch for a tailsitter in fixed-wing mode; otherwise **`[0,0,0,0]`**, "to save bandwidth" instead of `[1,0,0,0]` | always `[0,0,0,0]` ("unused, but probably should correspond to the AHRS view?") |
| Attitude unknown | sent only when `vehicle_attitude` updates | not sent if `get_quaternion()` fails |
| Default rate | per link mode (`mavlink_main.cpp`): normal 10 Hz, onboard/USB-config 50 Hz, onboard-low-bandwidth 20 Hz, low-bandwidth 4 Hz | not in any `SRn_*` stream group: only on request |

Every result matches this. Not exercised here:
- PX4's tailsitter fixed-wing offset. It needs an `sihsim_xvert` flight with a transition; that's the one "is it honoured" test left for `repr_offset_q`.
- ArduPlane's ATTITUDE override, which subtracts `PTCH_TRIM_DEG` from pitch and uses the VTOL view on tailsitters (`GCS_MAVLink_Plane.cpp`). With the default trim of 0 and a non-tailsitter QuadPlane, the two messages agree. Otherwise ATTITUDE would no longer match ATTITUDE_QUATERNION, as the audit notes. That deviation belongs to ATTITUDE's own tests.

## Definition review

Proposed upstream in [mavlink/mavlink#2673](https://github.com/mavlink/mavlink/pull/2673) (draft): items 1–3 below.

The stacks are compatible. Both send the same quaternion convention: it matches each stack's own ATTITUDE, and on ArduPlane/QuadPlane at a 270° heading. Both send body-frame gyro rates, and `[0,0,0,0]` whenever no offset applies. A consumer needs no autopilot-specific code. Suggested XML changes, all clarifications of what both stacks already do:

1. **`repr_offset_q`: say that `[0,0,0,0]` also means "no offset".** PX4 supports the field but sends `[0,0,0,0]`, not the XML's `[1,0,0,0]`, whenever no offset applies; ArduPilot always sends `[0,0,0,0]`.
   A consumer can't tell "not supported" from "supported, no offset", and doesn't need to: both mean "don't rotate". Suggested text: "… send [0, 0, 0, 0] if there is no offset or the field is not supported". The tailsitter example would then use `[0, 0, 0, 0]` (or "no offset") in hover mode.
2. **State the reference frame.** The description gives the body axes (X-front, Y-right, Z-down) but not what the attitude is relative to.
   PX4 documents its quaternion as "rotation from the FRD body frame to the NED earth frame", Hamilton convention (`msg/versioned/VehicleAttitude.msg`). ArduPilot's gives the same ZYX Euler angles as its ATTITUDE, including at a 270° heading, so it uses the same convention.
   Suggested: "… expressed as a quaternion (Hamilton convention) rotating from the FRD body frame to the NED earth frame".
3. **Rates: say "body frame".** "Roll angular speed" doesn't say about which axes. Both stacks send body-frame gyro rates, and ATTITUDE has the same gap.
4. **`time_boot_ms`: optional.** PX4 sends the estimate's timestamp and ArduPilot the send time. At rest they're within a few ms, and the field's wording permits either. Worth saying "timestamp of the attitude estimate" only if a consumer needs the distinction; not recommended for this move.
5. **Not recommended**:
   - An `invalid` value for q1..q4. Neither stack sends an unknown attitude; both suppress the message instead. HOME_POSITION's "all NaN if unknown" is precedent if one is ever wanted.
   - A unit-norm requirement. Both send normalised quaternions, and "quaternion" for an attitude already implies it.
