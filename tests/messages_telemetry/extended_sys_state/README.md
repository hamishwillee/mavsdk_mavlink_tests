# EXTENDED_SYS_STATE (message id 245)

[EXTENDED_SYS_STATE](https://mavlink.io/en/messages/common.html#EXTENDED_SYS_STATE): the VTOL state and the landed state.
Listed as "ready to move" to standard.xml in the 2026-11-11 common→standard audit, moving MAV_LANDED_STATE and MAV_VTOL_STATE with it.

```bash
pytest tests/messages_telemetry/extended_sys_state/ --log-cli-level=INFO \
  --drone-address=udp://:14540 --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx \
  --vehicle-type=quadcopter --autopilot=px4          # or any ArduPilot row from the root CLAUDE.md
```

About 55 s per vehicle. Nothing flies.

## What the XML defines

- **`vtol_state`** (MAV_VTOL_STATE: UNDEFINED, TRANSITION_TO_FW, TRANSITION_TO_MC, MC, FW): "The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration."
- **`landed_state`** (MAV_LANDED_STATE: UNDEFINED, ON_GROUND, IN_AIR, TAKEOFF, LANDING): "Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown."

## Tests

| Test | Kind | Checks |
|---|---|---|
| `obs_default_rate`, `compat_set_interval_*`, `compat_request_message` | obs / compat | Generic emission tests (`../emission.py`). On ArduPilot the harness requests EXTENDED_SYS_STATE at 2 Hz, so the unrequested rate is measured after `SET_MESSAGE_INTERVAL 0` |
| `compat_landed_state_on_ground` | compat | An enum value; ON_GROUND (or UNDEFINED) for a disarmed vehicle on the ground |
| `compat_vtol_state_matches_type` | compat | Against the vehicle's own HEARTBEAT type (never `--vehicle-type`): a MAV_TYPE_VTOL_* type sends a VTOL state; a multirotor or ground/surface type can't be "in VTOL configuration", so sends UNDEFINED. Any other type (fixed-wing) is characterised only, because ArduPilot reports a standard QuadPlane as MAV_TYPE_FIXED_WING |

Field verdicts: an UNDEFINED vtol_state from a non-VTOL is right, but is also what a stack that never fills the field sends, so it's recorded untestable; only a VTOL state shows the field is supported. An always-UNDEFINED landed_state is recorded not supported.

<!-- RESULTS -->
