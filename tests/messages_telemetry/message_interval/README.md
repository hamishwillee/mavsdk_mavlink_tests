# MESSAGE_INTERVAL (message id 244)

[MESSAGE_INTERVAL](https://mavlink.io/en/messages/common.html#MESSAGE_INTERVAL): the reply to REQUEST_MESSAGE(244, param2=id), giving the interval at which a message is sent.
Listed as "ready to move" to standard.xml in the 2026-11-11 common→standard audit, together with MAV_CMD_SET_MESSAGE_INTERVAL.

```bash
pytest tests/messages_telemetry/message_interval/ --log-cli-level=INFO \
  --drone-address=udp://:14540 --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx \
  --vehicle-type=quadcopter --autopilot=px4          # or any ArduPilot row from the root CLAUDE.md
```

About 50 s per vehicle. Nothing flies.

## What the XML defines

- **Message**: "The interval between messages for a particular MAVLink message ID. This message is sent in response to the MAV_CMD_REQUEST_MESSAGE command with param1=244 (this message) and param2=message_id".
- **`message_id`**: "The ID of the requested MAVLink message".
- **`interval_us`**: "A value of -1 indicates this stream is disabled, 0 indicates it is not available, > 0 indicates the interval at which it is sent."
- **MAV_CMD_SET_MESSAGE_INTERVAL** param2: "The interval between two messages. -1: disable. 0: request default rate (which may be zero)."

Silent on: what "not available" covers (a message the system doesn't know, can't send now, or isn't streaming), how to express "sent at every update", and what to report for a message sent only on events or on request.

## Tests

The generic emission tests don't apply (it's a reply). Every message tested in this tree also exercises it through `emission.default_rate`, which compares its own measured rate against MESSAGE_INTERVAL.

| Test | Kind | Checks |
|---|---|---|
| `compat_reports_set_rate` | compat | After SET_MESSAGE_INTERVAL(SYSTEM_TIME) at 2 Hz and 5 Hz (rule 4c), interval_us reports each (±2%) for the requested message_id. A NACKed SET or REQUEST is a PASS (rule 4a) |
| `compat_reports_disabled` | compat | After SET -1, interval_us is -1 |
| `obs_reports_default` | obs | After SET 0: interval_us against the measured rate |
| `compat_matches_what_arrives` | compat | For 15 messages (including absent sensors and a second GPS), listened to for 10 s: interval_us > 0 and within ±30% of the measured rate for each that arrives; not > 0 (up to a 5 s interval) for one that never arrives |
| `compat_unknown_message` | compat | For message id 65000 (in no dialect): NACKed, or interval_us 0 ("not available") |
| `obs_unsolicited` | obs | Whether MESSAGE_INTERVAL is sent unrequested, e.g. after a SET |

On ArduPilot several of the 15 messages are requested by the harness itself at the start of the test; that doesn't matter here, since the check is interval_us against what actually arrives.

## Results (2026-11-12)

PX4: `px4io/px4-sitl:latest` (1.18.0-beta, SIH, built 2026-09-11). ArduPilot: 4.8.0-dev SITL.

| | PX4 (all four SIH models) | ArduCopter | ArduPlane / QuadPlane / Rover |
|---|---|---|---|
| After SET 2 Hz / 5 Hz | 500000 / 200000 | 500000 / 200000 | 500000 / 200000 |
| After SET −1 | −1 | −1 | −1 |
| After SET 0 (SYSTEM_TIME at its default) | 1000000 (1 Hz, measured 0.98) | −1 (not streamed by default) | 1000000 (measured 1.00) |
| Unknown message (65000) | **ACCEPTED, −1 — compatibility error** ("not available" is 0) | FAILED, and a reply with 0 | FAILED, and a reply with 0 |
| interval vs what arrives (15 messages) | **GPS_RAW_INT arrives at 7.8 Hz but −1 — compatibility error** (an "unlimited" stream); every other one agrees | all agree | **GPS2_RAW and SCALED_PRESSURE3 never arrive but 1000000 — compatibility error** (stream-group rate for a sensor that isn't there) |
| Unsolicited MESSAGE_INTERVAL | none (also none after a SET) | none | none |

From the other messages' `default_rate` tests (the same check, per message):
- PX4 reports 1000000 for **GPS_GLOBAL_ORIGIN**, which it sends only when the origin changes (never, at rest).
- ArduPlane/QuadPlane report −1 for HOME_POSITION and GPS_GLOBAL_ORIGIN while sending them every ~6 s (home re-set while disarmed). That's an event, not a stream; −1 is reasonable.

## How the stacks implement it (source, 2026-11-12)

PX4 `36cedabb02` (a board fork of main; every file cited is identical in upstream main `e192036345` for this code, 2026-11-11). ArduPilot `ce9c2d01a4`.

| Case | PX4 (`mavlink_receiver.cpp`) | ArduPilot (`GCS_Common.cpp`) |
|---|---|---|
| Streamed message | the stream's configured interval (:2666-2680); **an "unlimited" stream reports −1** | `interval_ms × 1000` from its deferred bucket (:3323-3341) |
| Not streamed | −1 (:2668) | −1, ACCEPTED (:3364-3367) |
| Unknown id | **−1, ACCEPTED** (:777-778) | **0, FAILED** (:3357-3361) |
| In a stream table/group but unsendable (no such sensor) | the table's interval (e.g. 1000000 on a normal link), while nothing is sent | the group's interval, while nothing is sent |
| After REQUEST_MESSAGE on a link that doesn't stream it | stream added with interval 0 (:968), so later queries report **0** | unchanged |
| Unsolicited reply after SET | no | no |
| SET param2 | < 0 stop; 0 mode default; > 0 rate (applied before the ACK) | −1 stop, other negatives DENIED; 0 group default; < 1 ms → 1 ms; > 60 s clamped |
| SET rejected | FAILED for unknown id, HEARTBEAT, param3/4 ≠ 0, **param7 ≠ 0** | DENIED for param3 ≠ 0, unknown id; param7 ignored |

Every result matches this.

Consumers: QGC stores `1e6 / interval` for > 0 and passes −1/0 through (`MessageIntervalManager.cc:46`); MAVSDK and MAVProxy use 0 = default and −1 = stop when *setting*. No consumer was found that distinguishes −1 from 0 in a reply.

## Definition review

Proposed upstream in [mavlink/mavlink#2676](https://github.com/mavlink/mavlink/pull/2676) (draft): item 1 below.

The basic case is compatible: both stacks report exactly the interval set, −1 after a disable, and agree on every message that streams normally. A consumer can rely on "> 0: sent at this interval" only for messages it has seen arrive, because of the cases below. Suggested XML changes:

1. **Say what 0 and −1 cover.** "0 indicates it is not available" is the case PX4 gets wrong (−1 for a message id it doesn't know). Suggest: "-1: this message is supported but not currently streamed (disabled). 0: this message is not supported (e.g. an unknown message ID, or one the system cannot send). > 0: the interval at which it is sent." ArduPilot already does this for unknown ids; PX4's −1 is a PX4 fix.
2. **"> 0 indicates the interval at which it is sent"**: both stacks report a configured interval for a stream that sends nothing (PX4 GPS_GLOBAL_ORIGIN; ArduPilot a second GPS/barometer the vehicle doesn't have). That's an implementation issue on both, against a clear definition; no XML change, but worth reporting to both.

Not proposed:
- A value for "sent at every update" (PX4's unlimited streams report −1, which reads as "disabled"). A real gap, but it needs a convention (e.g. a tiny positive interval); worth an issue rather than a guess.
- PX4 rejecting SET_MESSAGE_INTERVAL param7 (response target) 1 and 2, which the XML defines: a PX4 issue.
