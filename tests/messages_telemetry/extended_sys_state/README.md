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

## Results (2026-11-12)

PX4: `px4io/px4-sitl:latest` (1.18.0-beta, SIH, built 2026-09-11). ArduPilot: 4.8.0-dev SITL.

| | PX4 MC / FW | PX4 VTOL / tailsitter | ArduCopter | ArduPlane | QuadPlane | Rover |
|---|---|---|---|---|---|---|
| HEARTBEAT type | 2 / 1 | 22 (VTOL_FIXEDROTOR) / 19 (VTOL_TAILSITTER_DUOROTOR) | 2 (quadrotor) | 1 | **1 (fixed-wing)** | 10 |
| Unrequested | 4.9 Hz (`onboard`: 5 Hz) | 4.9 Hz | **not sent** | not sent | not sent | not sent |
| SET_MESSAGE_INTERVAL 2/10 Hz, −1, 0 | honoured | honoured | honoured | honoured | honoured | honoured |
| REQUEST_MESSAGE | one, ~57 ms | one | one, ~56 ms | one | one | one |
| `landed_state` (disarmed) | ON_GROUND | ON_GROUND | ON_GROUND | ON_GROUND | ON_GROUND | **UNDEFINED** |
| `vtol_state` | UNDEFINED | MC | **MC — compatibility error** | UNDEFINED | **FW** | UNDEFINED |

On ArduPilot the harness requests EXTENDED_SYS_STATE at 2 Hz; with that undone (`SET_MESSAGE_INTERVAL 0`) it isn't sent.

**ArduCopter sends `vtol_state` MC** for a quadrotor. A multirotor can't be "in VTOL configuration", and the XML says UNDEFINED then; a consumer reading MC as "VTOL in multicopter mode" would treat every ArduCopter as a VTOL.

**ArduPilot QuadPlane reports HEARTBEAT type FIXED_WING** (`Q_MAV_TYPE` 0) and sends a VTOL state (FW on the ground in its default mode). That's right for EXTENDED_SYS_STATE, but means a consumer can't tell a QuadPlane is a VTOL from HEARTBEAT; vtol_state is the only sign. QGC reads vtol_state only for vehicles whose type is a VTOL (`Vehicle.cc:1065-1090`), so it ignores QuadPlane's.

## Flight tests (2026-11-12)

The shared flight (`../flight.py`; `pytest tests/messages_telemetry -m flight …`, ~3–6 min per vehicle including a SITL restart) flies north, east, south and west at 30–50 m, lands away from home, and arms once on the ground there. Flown 2026-11-12 on the same builds as the results above: PX4 SIH multicopter, fixed-wing, standard VTOL and tailsitter; ArduCopter, ArduPlane, QuadPlane, Rover (Rover drives the route).

| Test | Kind | Checks |
|---|---|---|
| `compat_landed_state_in_flight` | compat | Never ON_GROUND while ≥ 5 m above home (GLOBAL_POSITION_INT); never IN_AIR/TAKEOFF/LANDING while disarmed on the ground |
| `obs_landed_state_transitions` | obs | Which states appear from arming to touchdown, and when |
| `compat_vtol_state_in_flight` | compat | A VTOL (its HEARTBEAT type, or a fixed-wing type reporting a VTOL state at rest): while flying fast (ground speed ≥ 9 m/s for 10 s — above both stacks' 5 m/s default multicopter mission speed) never MC or UNDEFINED, and FW at some point; on the ground MC or FW. NA for anything else |

| | PX4 (all four) | ArduCopter / Plane / QuadPlane | Rover |
|---|---|---|---|
| landed_state | ON_GROUND → TAKEOFF (at or just after arming) → IN_AIR (3–16 s after leaving the ground) → LANDING (in the landing item) → ON_GROUND | the same sequence | NA (never leaves the ground) |
| vtol_state, VTOL | standard VTOL: MC → TRANSITION_TO_FW → FW (178 of 178 fast samples) → MC; tailsitter adds TRANSITION_TO_MC | QuadPlane: TRANSITION_TO_FW 5, FW 72, TRANSITION_TO_MC 5 while fast; **FW on the ground** outside a Q mode, MC in one | — |

Every vehicle uses all four landed states, in order, so landed_state is recorded supported on all seven that fly; vtol_state is supported on the three VTOLs. No compatibility error.

Not covered by the XML, and the two stacks differ: what vtol_state means on the ground. PX4 VTOLs report MC; QuadPlane reports FW in a fixed-wing mode (e.g. MANUAL, which it boots in) and MC in a Q mode. Both are defensible ("in fixed-wing state" can describe the mode, not the motion). Worth noting in a MAV_VTOL_STATE clarification if one is ever made; not proposed. PX4's standard VTOL went from FW straight to MC at 5 Hz: its back-transition is shorter than one EXTENDED_SYS_STATE interval.

## How the stacks implement it (source, 2026-11-12)

PX4 `36cedabb02` (a board fork of main; every stream file cited is identical in upstream main `e192036345`, 2026-11-11; line numbers are at `36cedabb02`). ArduPilot `ce9c2d01a4`.

| | PX4 (`streams/EXTENDED_SYS_STATE.hpp`) | ArduPilot (`GCS_Common.cpp:6063-6066`) |
|---|---|---|
| When sent | normal 1 Hz, onboard 5 Hz, config 2, osd 1, onboard-low-bandwidth 1, low-bandwidth 0.5, minimal 0.1 | in no stream group: only by interval or request (QGC requests it at 1 Hz, `APMFirmwarePlugin.cc:440-442`) |
| vtol_state | UNDEFINED unless `is_vtol` (:61); MC/FW/transitions from `vehicle_status` (:80-93) | Copter: **always MC** (`ArduCopter/GCS_MAVLink_Copter.h:50`). Plane without quadplane: UNDEFINED. QuadPlane: `quadplane.cpp:4659-4683`; tailsitter `tailsitter.cpp:1041-1058` |
| landed_state | from the land detector; **ON_GROUND before it first publishes** (:62), so never UNDEFINED. Rover: `!armed` (`RoverLandDetector.cpp:80`) | Copter from `land_complete`; Plane from `is_flying()`; **Rover: UNDEFINED** (no override of `GCS.h:521-522`) |

Every result matches this.

## Definition review

Compatible except for ArduCopter's vtol_state, which is an ArduPilot bug against a clear definition (report it: `GCS_MAVLink_Copter.h:50` should return UNDEFINED). No XML change needed; the enums need to move with the message (MAV_VTOL_STATE, MAV_LANDED_STATE), as the audit says.

Not proposed: PX4's ON_GROUND before its land detector is ready is a plausible default for a vehicle that boots on the ground, and a consumer can't be misled by it at boot; ArduPilot Rover's UNDEFINED is allowed ("if landed state is unknown").
