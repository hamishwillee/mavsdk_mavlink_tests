# GPS_RAW_INT, GPS2_RAW (message ids 24, 124)

[GPS_RAW_INT](https://mavlink.io/en/messages/common.html#GPS_RAW_INT) / [GPS2_RAW](https://mavlink.io/en/messages/common.html#GPS2_RAW): the first and second GPS receiver's raw output (not the estimate). The same fields, with GPS2_RAW adding `dgps_numch`/`dgps_age` and ordering its extensions differently; one test module per message, sharing `gps_checks.py`.
Listed as "ready to move" to standard.xml in the 2026-11-11 common→standard audit, moving GPS_FIX_TYPE with them.

```bash
pytest tests/messages_telemetry/gps_raw_int/ --log-cli-level=INFO \
  --drone-address=udp://:14540 --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx \
  --vehicle-type=quadcopter --autopilot=px4          # or any ArduPilot row from the root CLAUDE.md
```

About 2 × 60 s per vehicle, plus two SITL restarts for GPS2_RAW. Nothing flies.

**GPS2_RAW needs a second receiver.** Neither SITL has one by default, so `test_gps2_raw.py` adds one for that module only, keyed on the vehicle's HEARTBEAT autopilot, restarts SITL, and puts the parameters back after its last test (both stacks persist parameters):
- ArduPilot: `GPS2_TYPE 1`, `SIM_GPS2_ENABLE 1` (as `Tools/autotest/default_params/copter-gps-for-yaw.parm`).
- PX4: `SENS_GNSS1_OFFX 1` (a `sensor_gps_sim` option to publish a second receiver 1 m north) — only on builds that have the parameter; the `px4-sitl` image used here doesn't.

`GPS2_SIM_SETUP=0` tests the SITL as it is.

## What the XML defines

| Field | Definition | Invalid / unknown |
|---|---|---|
| `time_usec` | "UNIX Epoch time or time since system boot" | — |
| `fix_type` | GPS_FIX_TYPE | (NO_GPS, NO_FIX) |
| `lat`/`lon` | degE7. GPS_RAW_INT: "WGS84, EGM96 ellipsoid"; GPS2_RAW: "WGS84" | — |
| `alt` | mm, MSL | — |
| `eph`/`epv` | HDOP/VDOP ×100 (GPS_RAW_INT has `multiplier="1E-2"`, GPS2_RAW doesn't) | UINT16_MAX |
| `vel` | cm/s ground speed | UINT16_MAX |
| `cog` | cdeg, 0.0..359.99° | UINT16_MAX |
| `satellites_visible` | | UINT8_MAX |
| `alt_ellipsoid` (ext) | mm "above WGS84, EGM96 ellipsoid" | none |
| `h_acc`, `v_acc`, `vel_acc`, `hdg_acc` (ext) | mm, mm, mm/s, degE5 | none |
| `yaw` (ext) | cdeg from north; 36000 for north | 0 "does not provide yaw"; UINT16_MAX "configured … unable to provide it" |
| `dgps_numch`, `dgps_age` (GPS2_RAW) | count, ms | none |

EGM96 is a geoid model, not an ellipsoid, so "WGS84, EGM96 ellipsoid" (lat/lon, alt_ellipsoid) is self-contradictory.

## Tests

| Test | Kind | Checks |
|---|---|---|
| `obs_default_rate`, `compat_set_interval_*`, `compat_request_message` | obs / compat | Generic emission tests (`../emission.py`) |
| `obs_time_usec` | obs | UNIX or boot time, send or fix time. Fails only if neither is plausible |
| `info_fix_and_position` | info | fix_type is a GPS_FIX_TYPE value (a compatibility error if not); with a 3D fix lat/lon/alt are within 10 m of GLOBAL_POSITION_INT |
| `compat_ranges_and_sentinels` | compat | cog 0..35999 or UINT16_MAX; yaw 0, 1..36000 or UINT16_MAX; eph/epv not 0 (a DOP can't be 0); characterises vel and satellites_visible |
| `obs_extensions` | obs | alt_ellipsoid − alt (plausible geoid separation −110..+90 m), and the accuracy fields |
| `obs_dgps` (GPS2_RAW) | obs | dgps_numch, dgps_age |

vel/cog need movement: untestable at rest. yaw/hdg_acc need a yaw-capable receiver: untestable while 0.

## Results (2026-11-12)

PX4: `px4io/px4-sitl:latest` (1.18.0-beta, SIH, built 2026-09-11). ArduPilot: 4.8.0-dev SITL. No field value contradicts the definition on any vehicle.

**GPS2_RAW**: tested on all four ArduPilot vehicles with the second simulated receiver. On PX4 the image's build has no `SENS_GNSS1_OFFX` (setting it returns DOES_NOT_EXIST), so there's no second receiver and every GPS2_RAW field is untestable ("not sent").

| | PX4 (all four SIH models) | ArduCopter | ArduPlane / QuadPlane / Rover |
|---|---|---|---|
| GPS_RAW_INT unrequested | 7.7 Hz: every sample (`onboard`: "unlimited"), but **MESSAGE_INTERVAL −1** | not sent | 1 Hz (`SRn_EXT_STAT`) |
| GPS_RAW_INT SET 2/10 Hz | 2 Hz honoured; 10 Hz **capped at 7.8 Hz** (SIH's GPS is 8 Hz) | honoured | honoured |
| GPS2_RAW unrequested / SET / REQUEST | not sent / **ACCEPTED, never sent — compatibility error** / DENIED | not sent / honoured / one message | 1 Hz / honoured / one message |
| `time_usec` | boot time at send (SIH has no UTC) | boot time at send (last fix) | same |
| `fix_type` | 3D | **RTK_FIXED** (SITL default `SIM_GPS1_FIXTYPE`) | RTK_FIXED |
| lat/lon/alt vs GLOBAL_POSITION_INT | ≤ 0.13 m / ≤ 0.10 m | ≤ 0.01 / 0.01 m | ≤ 0.02 / 0.15 m |
| `eph` / `epv` | 70 / 110 | 121 / 200 | 121 / 200 |
| `vel` at rest | 0–7 cm/s (noise) | 0 | 0 |
| `cog` at rest | random 0–359° (from noisy velocity) | 0 | Plane 90°, QuadPlane 270° (the heading), Rover 0 |
| `satellites_visible` | 10 | 10 | 10 |
| `alt_ellipsoid` − `alt` | 0 (SIH sets them equal) | 0 | 0; **Rover: alt_ellipsoid always 0** |
| `h_acc` / `v_acc` / `vel_acc` | 900 / 1780 / 400 | 300 / 300 / **0** | same |
| `hdg_acc` / `yaw` | 0 / 0 (no yaw source) | 0 / 0 | 0 / 0 |
| GPS2_RAW `dgps_numch` / `dgps_age` | — | 0 / 0 | 0 / 0 |

`vel`, `cog` are recorded untestable (need movement); `yaw`, `hdg_acc` untestable (no yaw-capable receiver); `dgps_*` untestable (no DGPS).

## How the stacks implement it (source, 2026-11-12)

PX4 `36cedabb02` (a board fork of main; every stream file cited is identical in upstream main `e192036345`, 2026-11-11; line numbers are at `36cedabb02`). ArduPilot `ce9c2d01a4`.

| | PX4 (`streams/GPS_RAW_INT.hpp`, `GPS2_RAW.hpp`) | ArduPilot (`AP_GPS.cpp:1375-1455`) |
|---|---|---|
| Which receiver | GPS_RAW_INT: the *preferred* receiver (`sensors_status_gnss.order == 0`); GPS2_RAW: order 1 | GPS_RAW_INT: instance 0; GPS2_RAW: instance 1 (not primary/secondary) |
| Default rate | normal 5 Hz (GPS2_RAW 1 Hz); onboard/config/onboard-low-bandwidth: every sample ("unlimited", which MESSAGE_INTERVAL reports as −1) | `SRn_EXT_STAT`: Copter 0, Plane/Rover 1 Hz |
| No receiver / no data | not sent; after > 1 s without a sample, a 1 Hz placeholder with NO_GPS and every sentinel (:160-176) | not sent if the GPS type is 0; otherwise always, including with no fix |
| time_usec | receiver UTC if known, else boot time | boot time of the last fix |
| eph/epv | HDOP/VDOP × 100; SIH with no fix sends 10000, not UINT16_MAX | HDOP/VDOP; UINT16_MAX when unknown |
| vel / cog unknown | never UINT16_MAX; cog unchecked | never UINT16_MAX |
| satellites_visible | **satellites used** | satellites used (`num_sats`); never UINT8_MAX |
| alt_ellipsoid | receiver ellipsoid height | alt − undulation; **0 if the undulation is unknown** |
| h_acc / v_acc / vel_acc | receiver accuracies × 1000 | **0 when the receiver doesn't provide them** |
| hdg_acc | only with a heading, else 0 | always 0 (TODO) |
| yaw | heading if fresh; 0 → 36000. GPS_RAW_INT: 0 until a heading is first seen, then UINT16_MAX when it's lost. **GPS2_RAW: 0 when it's lost** | 0 not configured; UINT16_MAX configured but unavailable; 0 → 36000 |
| fix_type | uORB value passed through; uORB "extrapolated" (8) goes out as PPP | the same enum as GPS_FIX_TYPE (static_assert) |

Consumers: QGC treats eph/epv UINT16_MAX as unknown, **h_acc/v_acc 0 as unknown**, cog ≥ 36000 as unknown, yaw 0 or > 36000 as unknown (`VehicleGPSFactGroup.cc:110-147`). Mission Planner skips UINT16_MAX eph/vel/cog and UINT8_MAX satellites, uses eph as HDOP, and has no sentinel handling for GPS2_RAW or for yaw. MAVProxy reads yaw 65535 as "configured but invalid" and doesn't check cog for UINT16_MAX.

## Definition review

Compatible: both stacks send the same quantities in the same units; every sentinel they send is the defined one; QGC and Mission Planner decode both correctly. Suggested XML changes:

1. **lat/lon/alt_ellipsoid: drop "EGM96 ellipsoid".** EGM96 is a geoid model; the ellipsoid is WGS84. GPS_RAW_INT's lat/lon say "WGS84, EGM96 ellipsoid" and both messages' alt_ellipsoid "above WGS84, EGM96 ellipsoid"; GPS2_RAW's lat/lon already say just "WGS84". Suggest "Latitude (WGS84)" and "Altitude (above WGS84 ellipsoid)".
2. **h_acc, v_acc, vel_acc, hdg_acc: `invalid="0"` ("0 if unknown")**. Both stacks already send 0 when the receiver doesn't provide an accuracy, QGC already treats h_acc/v_acc 0 as unknown, and a zero uncertainty can't be a real value.
3. **Align the two messages' descriptions**: GPS2_RAW's eph/epv get GPS_RAW_INT's `multiplier="1E-2"`, and GPS_RAW_INT's cog drops "in degrees * 100" (the `units` attribute already says cdeg), matching GPS2_RAW. (The extension order differs, which can't change.)

Not proposed:
- `satellites_visible` → "used": both stacks send satellites used, but renaming the meaning of a field is a bigger change than this move needs; worth an issue.
- `vel`/`cog` "UINT16_MAX if unknown": neither stack ever sends it while a receiver reports, and both send a course when stationary. That's within the definition (the course is "known", just meaningless at rest).
- `alt_ellipsoid` invalid value: ArduPilot sends 0 when the undulation is unknown, but 0 m is a real ellipsoid height.
- PX4's GPS2_RAW yaw 0 vs GPS_RAW_INT UINT16_MAX for a lost heading, and its uORB "extrapolated" fix sent as PPP: PX4 fixes.
