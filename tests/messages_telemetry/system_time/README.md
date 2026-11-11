# SYSTEM_TIME (message id 2)

[SYSTEM_TIME](https://mavlink.io/en/messages/common.html#SYSTEM_TIME): "the time of the sender's master clock".
Listed as "ready to move" to standard.xml in the 2026-11-11 common→standard audit, conditional on adding `invalid="0"` to `time_unix_usec`.

```bash
pytest tests/messages_telemetry/system_time/ --log-cli-level=INFO \
  --drone-address=udp://:14540 --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx \
  --vehicle-type=quadcopter --autopilot=px4          # or any ArduPilot row from the root CLAUDE.md
```

About 70 s per vehicle, including one SITL restart. Nothing flies.

## What the XML defines

| | `time_unix_usec` | `time_boot_ms` |
|---|---|---|
| Type, units | uint64, µs | uint32, ms |
| Description | "Timestamp (UNIX epoch time)" | "Timestamp (time since system boot)" |
| invalid / min / max / increment | none | none |

The message description says it can be sent by any component, and a component with a less reliable clock "can choose to" adopt a more recent time it receives.
The XML says nothing about default emission, rate, what to send before the clock is known, or whether the time can jump.

## Tests

| Test | Kind | Checks |
|---|---|---|
| `obs_default_rate` | obs | Unrequested rate and senders over 10 s; whether `MESSAGE_INTERVAL` (REQUEST_MESSAGE 244) agrees |
| `compat_set_interval_honoured` | compat | SET_MESSAGE_INTERVAL at 2 Hz and 10 Hz: NACK, or measured rate within ±20% of each |
| `compat_set_interval_disable` | compat | `-1`: NACK, or no message for 3 s (stream enabled first) |
| `compat_set_interval_default` | compat | `0` after a change: NACK, or back to the unrequested rate |
| `compat_request_message` | compat | REQUEST_MESSAGE(2) with the stream off: NACK, or exactly one message |
| `compat_boot_ms_tracks_real_time` | compat | `time_boot_ms` never decreases or sticks; advances at host rate ±5% |
| `info_boot_ms_matches_other_messages` | info | ATTITUDE / GLOBAL_POSITION_INT `time_boot_ms` agree with SYSTEM_TIME's (±500 ms) |
| `compat_unix_is_epoch` | compat | Every non-zero `time_unix_usec` is after 2020 and within ±5 s of the host clock (`SYSTEM_TIME_UNIX_TOLERANCE_S`) |
| `obs_unix_vs_boot_ms_drift` | obs | Whether `time_unix_usec/1000 − time_boot_ms` stays constant (one clock or two) |
| `info_boot_ms_resets_on_reboot` | info | After a SITL restart, `time_boot_ms` ≤ time since the old process was killed |
| `obs_unix_after_boot` | obs | What `time_unix_usec` is between boot and the clock becoming known |

The field tests share one 10 s window at 10 Hz, and the two reboot tests share one restart (rule 7).

## Results (2026-11-11)

PX4: `px4io/px4-sitl:latest` (1.18.0-beta, SIH, built 2026-09-11). ArduPilot: 4.8.0-dev (`70fe7125`) SITL.

| | PX4 MC / FW / VTOL | ArduCopter | ArduPlane / QuadPlane / Rover |
|---|---|---|---|
| Unrequested (this link) | 0.98 Hz¹ | **not sent** (MESSAGE_INTERVAL −1) | 1.00 Hz |
| SET_MESSAGE_INTERVAL 2/10 Hz, −1, 0 | all honoured | all honoured | all honoured |
| REQUEST_MESSAGE | one message, ~56 ms | one message, ~65 ms | one message, 56–61 ms |
| `time_boot_ms` rate vs host | ×0.978–0.981² | ×0.9999 | ×0.9996–1.0000 |
| Other messages' `time_boot_ms` | within 10 ms | within 1 ms | within 5 ms |
| `time_unix_usec` vs host clock | 0 ms | −1.8 s³ | −1.2 to −1.5 s³ |
| `unix/1000 − boot_ms` | drifts ~20 ms/s² | constant (0.5 ms spread) | constant (1 ms spread) |
| After restart | valid from the first message (boot 7.1–7.3 s) | **0** until boot ≈10.1 s | **0** until boot ≈9.2–9.8 s |

¹ PX4's default depends on the link's MAVLink mode (below). The test link (14540) is `onboard` mode, which streams 1 Hz.
² PX4 SITL only: `time_boot_ms` is lockstep simulation time, which runs ~2% slow on SIH, while `time_unix_usec` is the host's real-time clock. Not a finding about PX4 on hardware.
³ Constant within a run but different per run: the SITL's simulated-GPS time, not a property of ArduPilot on hardware.

No compatibility errors on any of the seven vehicles. Both stacks support the message, both fields, and every request mechanism. Within each stack every vehicle type behaves the same, except ArduCopter's default rate (by design: `SRn_EXTRA3` defaults to 0 on Copter).

## How the stacks implement it (source, 2026-11-11)

PX4 `cd0e647af5` (main; the image above predates it, with no SYSTEM_TIME changes since September). ArduPilot `ce9c2d01a4`.

| | PX4 | ArduPilot |
|---|---|---|
| Sender | `streams/SYSTEM_TIME.hpp`: `time_boot_ms = hrt_absolute_time()/1000`, `time_unix_usec` = `CLOCK_REALTIME` | `GCS_Common.cpp` `send_system_time()`: `time_unix_usec` = `AP::rtc().get_utc_usec()` (`micros64() + rtc_shift`), `time_boot_ms = AP_HAL::millis()` |
| Clock unknown | **Not sent** if `time_unix_usec` ≤ 2001-01-01; REQUEST_MESSAGE then ACKs DENIED (`handle_request_message_command`) | **Sent with `time_unix_usec = 0`** until the RTC has a source (GPS in SITL) |
| Default rate | Per link mode (`mavlink_main.cpp`): onboard/OSD/USB-config 1 Hz, onboard-low-bandwidth 2 Hz, low-bandwidth 0.5 Hz; **normal** (the usual GCS link) and minimal: not streamed | `SRn_EXTRA3` group: Copter/Blimp 0, Plane/Rover/Tracker 1 Hz, Sub 3 Hz |
| SET_MESSAGE_INTERVAL | −1 stop, 0 default, >0 rate; non-zero param3/4/7 rejected | −1 stop, other negatives DENIED, 0 default, ms resolution capped at 60 s; non-zero param3 DENIED |
| MESSAGE_INTERVAL reply | stream's interval; −1 if no stream | −1 when not streaming |
| Receiving SYSTEM_TIME | `mavlink_timesync.cpp`: sets the clock if the received time is > 2009-02-13 and > 60 s ahead, and `SYS_TIME_SRC` allows MAVLink. `time_boot_ms` ignored | `handle_system_time_message`: `AP_RTC::set_utc_usec(…, SOURCE_MAVLINK_SYSTEM_TIME)`: ignored if before 2022-01-01, below the current source's priority (GPS wins), or not in `RTC_TYPES`. `time_boot_ms` ignored |

Every test result matches this reading. The one result not traced to source is ArduPilot SITL's 1.2–1.8 s lag behind the host clock (constant within a run). The RTC is set from the simulated GPS, so that's the likely source, but it isn't confirmed. It is irrelevant on hardware.

**Consumers** (the audit report, plus QGC checked here):
- ExpressLRS and mLRS treat `time_unix_usec = 0` as unknown.
- QGC doesn't read SYSTEM_TIME. It sends one to the vehicle on connect (`Vehicle::_sendQGCTimeToVehicle`) with `time_boot_ms = 0` ("Not necessary").
- Mission Planner sends one with `time_boot_ms = 0` too (`FlightData.cs`). On receipt it turns `time_unix_usec = 0` into 1970-01-01 (`CurrentState.cs`), so it doesn't treat 0 as unknown.
- MAVProxy's `system_time` module sends `time_boot_ms` as its own uptime. The first message goes out right after the module loads, so it can carry 0.
- ArduPilot's routing notes (`MAVLink_routing.cpp`) tell GCSes to detect a sender reboot by a *decrease* in SYSTEM_TIME `time_boot_ms`.

## Definition review

Proposed upstream in [mavlink/mavlink#2671](https://github.com/mavlink/mavlink/pull/2671) (draft): items 1–3 below.

The two stacks are compatible, and a consumer needs no autopilot-specific code, provided it treats both "0" and "not sent" as "time unknown". Suggested XML changes:

1. **`time_unix_usec`: `invalid="0"`.**
   - ArduPilot sends 0 when it doesn't know the time; PX4 suppresses the message instead. Both are compatible with an explicit 0.
   - Every consumer that reads the field already treats 0 as unknown.
   - Both receivers already ignore 0 (and anything old), so a GCS sending 0 is harmless.
   - NaN isn't possible: the field is an integer.
2. **`time_boot_ms`: `invalid="0"`**, for a sender with no boot time (e.g. a GCS).
   - QGC already sends 0 for "not provided", and both stacks ignore the field on receipt.
   - No real sender emits 0 in practice. PX4 doesn't send until its clock is valid, and ArduPilot's first message in these runs was ≥1.8 s after boot.
   - It also lets a consumer that detects reboots by a decrease in `time_boot_ms` (item 3) skip a GCS's 0, instead of reading it as a reboot.
3. **Description**: worth adding two things.
   - A decrease in `time_boot_ms` from the same sender means it rebooted (ArduPilot's routing documentation already depends on this). A 0 (invalid) doesn't count.
   - `time_boot_ms` wraps after ~49.7 days.
4. **Not recommended**:
   - A `minValue` on `time_unix_usec`. Both stacks apply a floor on receipt (2009 and 2022), and PX4 one on send (2001), but the floors differ and they're plausibility checks, not part of the format. `invalid="0"` covers the case that matters.
   - A required default rate. Both stacks configure it per link or per vehicle, and a GCS can request it.
