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

<!-- RESULTS -->
