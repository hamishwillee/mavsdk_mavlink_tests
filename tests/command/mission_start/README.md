# MAV_CMD_MISSION_START (300) — command protocol tests

"start running a mission" ([common.xml](https://mavlink.io/en/messages/common.html#MAV_CMD_MISSION_START)).

| Param | Label | XML |
|---|---|---|
| 1 | First Item | the first mission item to run (`minValue=0`, `increment=1`) |
| 2 | Last Item | the last mission item to run — after it is run, the mission ends (`minValue=0`, `increment=1`) |
| 3–7 | — | omitted (Empty) |

No location and no float in params 5/6, so COMMAND_INT is the primary message type; the mandatory common checks send both.

## Files

| File | What it tests |
|---|---|
| `test_command.py` | Tier 1 (ACK). `TestMissionStartNoMission`: refused when no mission is loaded. `TestMissionStartCommand` (mission loaded): the seven mandatory common checks (`Tier1CommandTestBase`) plus per-param ACK tests |
| `test_flight.py` | Tier 2 (execution, real stack only). Does the mission start; are First Item / Last Item honoured (two values each); does it arm a disarmed vehicle |

The command is stateful, which shapes both files:

- **Without a mission, PX4 DENIES every send regardless of params** — so every param check runs with a mission loaded (`_mission_loaded_cls`), or "a real value in an Empty param is rejected" would pass for the wrong reason.
- **On PX4 an accepted MISSION_START arms the vehicle and flies the mission.** Tier 1 therefore flies: the vehicle takes off on the first accepted probe and keeps re-starting the mission while the class runs. The class fixture waits until the vehicle is armable first (otherwise PX4 answers `TEMPORARILY_REJECTED`) and brings it home with RTL afterwards, restarting the stack if it doesn't land.

The mission (both tiers): `NAV_TAKEOFF` at home to 15 m, then waypoints 40 m N, E, S and W of home. From home or any waypoint, the path to another waypoint passes nowhere near a third, so the waypoint the vehicle reaches identifies the item the mission is on.

## Running

```bash
pytest tests/command/mission_start/ -v --log-cli-level=INFO          # mock (Tier 1 only; Tier 2 skips)

pytest tests/command/mission_start/ \
    --drone-address=udp://:14540 --connection-timeout=60 \
    --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx \
    --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO
```

Tunables (environment): `MISSION_START_WAYPOINT_SPACING_M` (40), `MISSION_START_REACH_RADIUS_M` (8), `MISSION_START_REACH_TIMEOUT_S` (90), `MISSION_START_LAST_ITEM_HOLD_S` (30).

## Source review (before running)

**PX4 main** — `Commander.cpp` (`VEHICLE_CMD_MISSION_START`), `navigator_main.cpp`:

- no valid mission → `DENIED`;
- param1 must be finite, ≥ −1 and < the item count, else `DENIED`. −1 is PX4's own "start from the current item" (Navigator only sets the index for param1 ≥ 0) — the XML's minimum is 0;
- then: switch to AUTO_MISSION **and arm**; either refused → `TEMPORARILY_REJECTED`;
- param2 is never read;
- cmd 300 has no `mavlink_command_params.hpp` entry, so params 3–7 aren't validated.

**ArduPilot master** — `GCS_MAVLink_{Copter,Plane,Rover}.cpp`:

- any non-zero param1 or param2 → `DENIED` ("first-item/last item not supported"); NaN counts as non-zero;
- otherwise switch to AUTO — Copter/Rover `ACCEPTED` or `FAILED`, Plane always `ACCEPTED`. It does not arm.
- params 3–7 aren't read.

## Results

### PX4 SIH multicopter — `px4io/px4-sitl:latest` (1.18.0-beta), 2026-10-22

Every result matches the source review. 20 passed, 9 failed (all compatibility errors), 2 NA. Param coverage COMPLETE.

**Tier 1**

| Check | Result | |
|---|---|---|
| No mission loaded | PASS | `DENIED` |
| Mandatory common checks 1–3 | PASS | one `ACCEPTED` per send, both message types |
| Frame validation survey | INCONCLUSIVE | all 22 frames accepted (frame is irrelevant to this command) |
| Empty params 3–7: sentinel accepted | PASS | |
| Empty params 3–7: real value rejected | **FAIL** (compat) ×5 | `ACCEPTED` — not validated (no `mavlink_command_params.hpp` entry) |
| param1 NaN not denied | **FAIL** (compat) | `DENIED` (`PX4_ISFINITE` check) — see § Spec gaps |
| param2 NaN not denied | PASS | `ACCEPTED` (never read) |
| param1 / param2 = a valid item | ACK recorded | `ACCEPTED` / `ACCEPTED` |
| param1 = −1 rejected (minValue 0) | **FAIL** (compat) | `ACCEPTED` — PX4's "start from the current item" |
| param2 = −1 rejected (minValue 0) | **FAIL** (compat) | `ACCEPTED` — never read |
| param1 past the last item rejected | PASS | `DENIED` |
| param2 past the end / param1 = 1.5 / param2 < param1 | observed | all `ACCEPTED` |

**Tier 2**

| Test | Result | Measured |
|---|---|---|
| `compat_starts_mission` (0/0, armed) | PASS | took off on item 0 and **continued to the first waypoint** — param2 = 0 runs the whole mission |
| `compat_first_item_honoured` (E, S) | PASS — **SUPPORTED** | first waypoint reached was E (MISSION_CURRENT `[2]`), then S (`[3]`) |
| `compat_last_item_honoured` (N, E) | **FAIL** (compat) — **NOT SUPPORTED** | ran past the last item every time: param2 = N reached E and S; param2 = E reached S and W (MISSION_CURRENT `[0..4]` both times) |
| `obs_arms_when_disarmed` | observed | **armed and took off** within 10 s of MISSION_START |

Each flight's RTL took longer than the 30 s restart window (RTL climbs to its return altitude first), so the harness restarted PX4 after every flight. That's correct, just slow: about 10 minutes for both tiers.

### ArduPilot

Not yet run. ArduCopter SITL still stalls at "Waiting for internal clock bits to be set" in this environment (root `CLAUDE.md` future-work #7), and this session's run never connected. Source predicts: no mission → `FAILED` (Copter/Rover) or `ACCEPTED` (Plane); any non-zero param1/param2, or NaN in either, → `DENIED`. That makes both param verdicts **REJECTED (NACKed)**, a legitimate "not supported" under rule 4a, and fails "param1/param2 NaN not denied". MISSION_START doesn't arm on ArduPilot.

## Spec gaps

Things the XML could usefully tighten. Both stacks agree on each one (root `CLAUDE.md` rule 3):

1. **No "run to the end" value for Last Item.** Taken literally, param2 = 0 means "end after item 0". Yet 0/0 is what GCSs send to run the whole mission, the only value ArduPilot accepts, and (above) what PX4 treats as the whole mission. The XML should say what 0 (or NaN) means.
2. **No sentinel for First Item.** Both stacks DENY NaN in param1, so the mandatory "defined param tolerates its sentinel" check fails on both. Either NaN should be defined (e.g. "first item" or "current item"), or param1 should be documented as mandatory, which would exempt it (`sentinel_policy="deny_required"`).
3. **"Start from the current item."** PX4 gives param1 = −1 that meaning, below the XML's `minValue=0`, and this repo's own `do_set_mission_current` Tier 2 relies on it. DO_SET_MISSION_CURRENT already defines −1 as "keep the current item", so MISSION_START could adopt the same value.
4. **No mission / no such item.** The XML doesn't say what a start with nothing to run should return. PX4 DENIES, ArduCopter FAILs, ArduPlane ACCEPTs. The tests assert only "not accepted".
