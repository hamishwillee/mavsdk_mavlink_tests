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
- PX4: `SENS_GNSS1_OFFX 1` (`sensor_gps_sim` then publishes a second receiver 1 m north) — only on builds that have the parameter.

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

<!-- RESULTS -->
