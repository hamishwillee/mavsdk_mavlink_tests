# tests/messages_telemetry/ — notes for Claude

Tests of messages a flight stack (or other component) **emits** to provide information: telemetry, and replies to requests for information or state.
A message that belongs to a service (mission, parameter, FTP, …) is generally tested with that service instead.
Started 2026-11-11 with the "ready to move" set of the common→standard audit (`common-to-standard-report.pdf`, 11 Nov 2026), beginning with SYSTEM_TIME.

One directory per message (`system_time/`), each with its own `README.md` (test design, results, source findings, definition review).
A family of messages with identical fields shares a directory and a checks module, with one thin test module per message (one report each): `scaled_imu/` (SCALED_IMU/2/3, `imu_checks.py`), `scaled_pressure/` (SCALED_PRESSURE/2/3, `pressure_checks.py`), `gps_raw_int/` (GPS_RAW_INT, GPS2_RAW, `gps_checks.py`).

## What a message test asks

Derived from the message's XML, the same way MAV_CMD tests are (root CLAUDE.md "General testing philosophy"):

- **Emission** — is it streamed unrequested (characterise: the XML never says), does `MAV_CMD_SET_MESSAGE_INTERVAL` work (two rates, rule 4c; `-1`; `0`), does `MAV_CMD_REQUEST_MESSAGE` give exactly one, does `MESSAGE_INTERVAL` agree with the measured rate.
  An accepted request that isn't honoured is a compatibility error (rule 4a); a NACK is a legitimate PASS.
- **Fields** — for each field: with a defined invalid value, record whether it's used; with none, check values are what the system would plausibly report (not stuck at 0 for a dynamic value, right units, right clock); with min/max/increment, check values stay inside.
  A value that contradicts the field's description is a FAIL + compatibility error; anything the XML leaves open is characterised (`obs`).
- **Vehicle types** — a message test doesn't fly, so it runs unchanged on every vehicle type, with no `--vehicle-type` branching. Run all of them.
- **Then** read both stacks' source to explain the results (Tier 2 pattern #5), and review the definition against what the implementations do.

## Framework (`conftest.py`, `emission.py`)

- **`emission.py`**: the five emission tests and two shared field checks. Every message module creates an `Emission(name, id, window_extra=(...))`, calls `EMISSION.define_tests(globals(), "<msg>")` to define the five emission tests as `test_<msg>_<category>_...`, and calls the field-check methods from its own tests:
  - the emission tests: `default_rate`, `set_interval_honoured`, `set_interval_disable`, `set_interval_default`, `request_message`;
  - `sample_window()`: one 10 s window at 10 Hz, with `window_extra` messages alongside, read by every field test (rule 7);
  - `boot_ms_tracks_real_time()`.

  The unrequested rate is cached per message, measured before anything changes it. For a message the harness itself requests from ArduPilot at the start of every test (`message_watcher.ARDUPILOT_REQUESTED_RATES_HZ`: HOME_POSITION, EXTENDED_SYS_STATE, ATTITUDE, GLOBAL_POSITION_INT, …) it is measured after `SET_MESSAGE_INTERVAL 0`, and the detail says so. `request_message` always stops the stream first for the same reason.

  `sample_window()` requests each `window_extra` message at 5 Hz for the window (ArduPilot sends GPS_GLOBAL_ORIGIN only on request, LOCAL_POSITION_NED only if a GCS asks), falls back to one REQUEST_MESSAGE if none arrives, and puts them back to 0 afterwards. If the message under test itself arrives fewer than 3 times (PX4 sends GPS_GLOBAL_ORIGIN only when it changes) it asks for 3 copies with REQUEST_MESSAGE, and the detail says so.

- `MessageSampler` (receive times + sender ids), `measure_rate`/`rate_hz` (`(n-1)/span` — counting n in a fixed window overcounts by one), `set_message_interval`, `request_message`, `get_message_interval` (REQUEST_MESSAGE(244)), `compat_verdict`.
- Commands go to the vehicle's own sysid/compid from its HEARTBEAT (`vehicle_target`), not a hardcoded 1, so non-zero `--sitl-instance` works. `vehicle_heartbeat()` also gives its MAV_TYPE and autopilot — which is what decides "is this a VTOL" (EXTENDED_SYS_STATE) or which stack-keyed setup applies, never `--vehicle-type`.
- Field helpers: `ned_from_origin()` (a global position relative to a GPS_GLOBAL_ORIGIN), `classify_time_usec()` (the "UNIX or boot" `time_usec` fields: which, plausible, send time or set time), `skip_untestable()` (record fields untestable and skip as NA).
- **Strength of an at-rest check.** A vehicle sitting level at its local origin reads ~0 on x/y, on level-axis acceleration, and on a mag axis near 0 — whichever way round the axes are. Those checks record a field `supported` only when its expected value is large enough to show an axis or sign error (5 m, 100 mG, 100 mgauss), and `untestable` otherwise. The mock deliberately puts its origin 40 m S / 30 m W of the vehicle and tilts it, so the strong path runs there.
- **GPS2_RAW**: neither SITL has a second receiver by default; `gps_raw_int/test_gps2_raw.py`'s autouse `_second_gps` gives one for that module only (ArduPilot `GPS2_TYPE=1`, `SIM_GPS2_ENABLE=1`; PX4 `SENS_GNSS1_OFFX=1`), restarting SITL, and restores the parameters after the module's last test (both stacks persist parameters across SITL runs). `GPS2_SIM_SETUP=0` skips it.
- **FLIGHT_INFORMATION** arms the vehicle once (no takeoff) to check `arming_time_utc`, then disarms.
- **Reporting**: `_message_auto_record` (autouse) writes every test into `reports/message_<name>_...` via `tests/report.py`'s `"message"` protocol. A module declares `_MSG_NAME`, `_MSG_ID` and its field list (`report.declare_params(report.MESSAGE_PROTOCOL, name, [fields])`).
  `record_field_fact()` gives each field a verdict (or an `untestable` reason), so the report's **Field coverage** works like Param coverage. `record_message_fact()` records whether the message is sent at all. `record_verdict()` adds request-handling verdicts to the Compatibility summary. `record_detail()` sets the row's text.
  The JSON is mavlink-compat-data's message shape (`render_message_json_doc`): `compatibility.<stack>.<variant>` plus per-field statements, with the run's vehicle frame as the variant.
  Validated 2026-11-11 by splicing PX4 and ArduPilot output into a scratch copy of the repo and running its `scripts/validate.py`, plus a negative control (a field variant without an implemented parent is rejected).
- **Mock**: `MockFlightStack` streams SYSTEM_TIME (1 Hz), ATTITUDE and ATTITUDE_QUATERNION (4 Hz, one fixed non-trivial attitude `MOCK_ATTITUDE_RAD`), and has every other message tested here available on request or SET_MESSAGE_INTERVAL (default 0, so other mock suites see no new traffic), from one consistent state: `MOCK_POSITION`, an origin offset `MOCK_ORIGIN_OFFSET_NED`, an Earth field `MOCK_MAG_FIELD`, ISA pressure. SCALED_IMU3/SCALED_PRESSURE3 are absent ("no such sensor"). It honours SET_MESSAGE_INTERVAL/REQUEST_MESSAGE and answers REQUEST_MESSAGE(MESSAGE_INTERVAL). Extend `_message_fields()` there when adding a message.
  Mock reports are only written with `--write-mock-reports`.

## Pitfalls found here

- **MAVSDK-Python 4.0.6 segfault on unsubscribe** (2026-11-11). Every binding's `unsubscribe_*()` drops the ctypes callback thunk before the native unsubscribe, so a callback in flight executes freed memory. Cancelling a 100 Hz ATTITUDE subscription on PX4 SIH crashed the whole pytest process.
  `tests/mavsdk_compat.py` patches every plugin's `_subscriptions` to keep popped entries alive. Not yet reported upstream.
- **PX4's default rate depends on the link's MAVLink mode.** Port 14540 is `-m onboard`, which streams SYSTEM_TIME at 1 Hz; `normal` mode (the usual GCS link) doesn't stream it at all.
  So "default rate" results are per link, not per stack.
- **An all-zero "not supported" value can also be a stack's "nothing to report"**. PX4 sends `ATTITUDE_QUATERNION.repr_offset_q = [0,0,0,0]` for no offset, "to save bandwidth", not the XML's `[1,0,0,0]`. So seeing only the "not supported" value isn't evidence the field is unsupported: record it as untestable, and take the per-stack verdict from source.
- **A near-identity attitude hides sign errors.** Converting a quaternion to Euler angles and comparing it against ATTITUDE catches swapped components at any attitude, but a conjugated (sign-flipped) quaternion only when the attitude is away from 0/180°. The harness passes a 270° heading in ArduPilot's `--home` (`tests/conftest.py`), but only ArduPlane and QuadPlane started there (2026-11-11). ArduCopter and Rover read ~1–2°, as did every PX4 SIH model, so the check is strong only on the plane types. Each run's detail line says which. The mock uses 2°/−3°/30°.
- **A NaN field arrives as `None`**: `mavlink_direct`'s `fields_json` encodes NaN as `null`. A check for an invalid-NaN value (HOME_POSITION.q) must accept `None`; `finite()` treats it as non-finite.
- **MAVSDK's `mavlink_direct` can't send NaN**: the fields are JSON, and a NaN field fails `send_message()` with INVALID_FIELD. The mock can't send "unknown" for a float whose invalid value is NaN (HOME_POSITION.q); its stream loop logs the failure and the message silently never arrives, so the tests go NA rather than fail. Check the log for "Stream loop error" when a mock message test is unexpectedly NA.
- **A message sent only on change** (PX4's GPS_GLOBAL_ORIGIN; ArduPilot's HOME_POSITION and GPS_GLOBAL_ORIGIN when not requested) gets too few samples from a rate window — `sample_window()` falls back to REQUEST_MESSAGE. An accepted SET_MESSAGE_INTERVAL that such a stream then ignores is still rule 4a's compatibility error.
- **Absent sensor instances** (SCALED_IMU3, SCALED_PRESSURE2/3, GPS2_RAW on most SITLs): every field test records "not sent" as untestable; the emission tests show how each stack answers requests for something it can't send. An instance carrying only *some* sensors (ArduPilot's SCALED_IMU3: magnetometer only) sends exactly 0 for the rest — the checks read "every axis exactly 0 in every message" as not provided.
- **PX4 SITL has two clocks**: `hrt_absolute_time()` is the lockstep simulation time; `CLOCK_REALTIME` is the host's. A field taken from each drifts apart at the simulation's real-time error (~2% on SIH).
