# tests/messages_telemetry/ — notes for Claude

Tests of messages a flight stack (or other component) **emits** to provide information: telemetry, and replies to requests for information or state.
A message that belongs to a service (mission, parameter, FTP, …) is generally tested with that service instead.
Started 2026-11-11 with the "ready to move" set of the common→standard audit (`common-to-standard-report.pdf`, 11 Nov 2026), beginning with SYSTEM_TIME.

One directory per message (`system_time/`), each with its own `README.md` (test design, results, source findings, definition review).

## What a message test asks

Derived from the message's XML, the same way MAV_CMD tests are (root CLAUDE.md "General testing philosophy"):

- **Emission** — is it streamed unrequested (characterise: the XML never says), does `MAV_CMD_SET_MESSAGE_INTERVAL` work (two rates, rule 4c; `-1`; `0`), does `MAV_CMD_REQUEST_MESSAGE` give exactly one, does `MESSAGE_INTERVAL` agree with the measured rate.
  An accepted request that isn't honoured is a compatibility error (rule 4a); a NACK is a legitimate PASS.
- **Fields** — for each field: with a defined invalid value, record whether it's used; with none, check values are what the system would plausibly report (not stuck at 0 for a dynamic value, right units, right clock); with min/max/increment, check values stay inside.
  A value that contradicts the field's description is a FAIL + compatibility error; anything the XML leaves open is characterised (`obs`).
- **Vehicle types** — a message test doesn't fly, so it runs unchanged on every vehicle type, with no `--vehicle-type` branching. Run all of them.
- **Then** read both stacks' source to explain the results (Tier 2 pattern #5), and review the definition against what the implementations do.

## Framework (`conftest.py`, `emission.py`)

- **`emission.py`**: the five emission tests and two shared field checks. Every message module creates an `Emission(name, id, window_extra=(...))` and calls its methods from its own `test_<msg>_...` functions:
  - the emission tests: `default_rate`, `set_interval_honoured`, `set_interval_disable`, `set_interval_default`, `request_message`;
  - `sample_window()`: one 10 s window at 10 Hz, with `window_extra` messages alongside, read by every field test (rule 7);
  - `boot_ms_tracks_real_time()`.

  The unrequested rate is cached per message, measured before anything changes it.

- `MessageSampler` (receive times + sender ids), `measure_rate`/`rate_hz` (`(n-1)/span` — counting n in a fixed window overcounts by one), `set_message_interval`, `request_message`, `get_message_interval` (REQUEST_MESSAGE(244)), `compat_verdict`.
- Commands go to the vehicle's own sysid/compid from its HEARTBEAT (`vehicle_target`), not a hardcoded 1, so non-zero `--sitl-instance` works.
- **Reporting**: `_message_auto_record` (autouse) writes every test into `reports/message_<name>_...` via `tests/report.py`'s `"message"` protocol. A module declares `_MSG_NAME`, `_MSG_ID` and its field list (`report.declare_params(report.MESSAGE_PROTOCOL, name, [fields])`).
  `record_field_fact()` gives each field a verdict (or an `untestable` reason), so the report's **Field coverage** works like Param coverage. `record_message_fact()` records whether the message is sent at all. `record_verdict()` adds request-handling verdicts to the Compatibility summary. `record_detail()` sets the row's text.
  The JSON is mavlink-compat-data's message shape (`render_message_json_doc`): `compatibility.<stack>.<variant>` plus per-field statements, with the run's vehicle frame as the variant.
  Validated 2026-11-11 by splicing PX4 and ArduPilot output into a scratch copy of the repo and running its `scripts/validate.py`, plus a negative control (a field variant without an implemented parent is rejected).
- **Mock**: `MockFlightStack` streams SYSTEM_TIME (1 Hz), ATTITUDE and ATTITUDE_QUATERNION (4 Hz, one fixed non-trivial attitude `MOCK_ATTITUDE_RAD`), honours SET_MESSAGE_INTERVAL/REQUEST_MESSAGE for them, and answers REQUEST_MESSAGE(MESSAGE_INTERVAL). Extend `_message_fields()` there when adding a message.

## Pitfalls found here

- **MAVSDK-Python 4.0.6 segfault on unsubscribe** (2026-11-11). Every binding's `unsubscribe_*()` drops the ctypes callback thunk before the native unsubscribe, so a callback in flight executes freed memory. Cancelling a 100 Hz ATTITUDE subscription on PX4 SIH crashed the whole pytest process.
  `tests/mavsdk_compat.py` patches every plugin's `_subscriptions` to keep popped entries alive. Not yet reported upstream.
- **PX4's default rate depends on the link's MAVLink mode.** Port 14540 is `-m onboard`, which streams SYSTEM_TIME at 1 Hz; `normal` mode (the usual GCS link) doesn't stream it at all.
  So "default rate" results are per link, not per stack.
- **An all-zero "not supported" value can also be a stack's "nothing to report"**. PX4 sends `ATTITUDE_QUATERNION.repr_offset_q = [0,0,0,0]` for no offset, "to save bandwidth", not the XML's `[1,0,0,0]`. So seeing only the "not supported" value isn't evidence the field is unsupported: record it as untestable, and take the per-stack verdict from source.
- **A near-identity attitude hides sign errors.** Converting a quaternion to Euler angles and comparing it against ATTITUDE catches swapped components at any attitude, but a conjugated (sign-flipped) quaternion only when the attitude is away from 0/180°. The harness passes a 270° heading in ArduPilot's `--home` (`tests/conftest.py`), but only ArduPlane and QuadPlane started there (2026-11-11). ArduCopter and Rover read ~1–2°, as did every PX4 SIH model, so the check is strong only on the plane types. Each run's detail line says which. The mock uses 2°/−3°/30°.
- **PX4 SITL has two clocks**: `hrt_absolute_time()` is the lockstep simulation time; `CLOCK_REALTIME` is the host's. A field taken from each drifts apart at the simulation's real-time error (~2% on SIH).
