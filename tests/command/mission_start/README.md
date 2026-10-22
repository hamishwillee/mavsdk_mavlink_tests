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

The mission (both tiers): `NAV_TAKEOFF` to 30 m, waypoints A, B, C 200 m apart in a straight line along the vehicle's heading, then the shared approach waypoint + `NAV_LAND` (`mission_landing_items()`) further along the same line.

Tier 2 reads which item is running from the protocol itself — `MISSION_CURRENT.seq` (the current item) and `MISSION_ITEM_REACHED.seq` (an item completed) — not from where the vehicle is. The layout identifies nothing; it only has to be flyable by any vehicle type:

- **Everything on the line the vehicle takes off along**, including the takeoff's own position (100 m out) — no turns.
- **The first waypoint beyond a fixed-wing's climb-out.** At 60 m, the plane flew over it while still climbing and stalled turning back.
- **A landing**, because PX4 fixed-wing/VTOL refuse a mission without one (`MIS_TKO_LAND_REQ`), which would make every MISSION_START `DENIED`. It's never reached; each flight ends with RTL (or a stack restart).

What each check reads:

- **First Item**: honoured if `MISSION_CURRENT` goes to that item within 10 s of the ACK without first showing an earlier waypoint. No flying to it needed.
- **Last Item**: the vehicle must actually run the item, so it flies to it (A, then B). Honoured if, once it's reached, `MISSION_CURRENT` hasn't moved on 5 s later — a stack that moves on reports the next item immediately. No verdict if the vehicle came back down after takeoff before then.

## Running

```bash
pytest tests/command/mission_start/ -v --log-cli-level=INFO          # mock (Tier 1 only; Tier 2 skips)

pytest tests/command/mission_start/ \
    --drone-address=udp://:14540 --connection-timeout=60 \
    --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx \
    --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO
```

Fixed-wing: `--px4-model=sihsim_airplane --vehicle-type=fixed_wing`.

Tunables (environment): `MISSION_START_WAYPOINT_SPACING_M` (200), `MISSION_START_REACH_TIMEOUT_S` (180), `MISSION_START_SELECT_TIMEOUT_S` (10), `MISSION_START_END_HOLD_S` (5).

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

### PX4 SIH — `px4io/px4-sitl:latest` (1.18.0-beta, `61907962c`), 2026-10-22

Multicopter (`sihsim_quadx`) and fixed-wing (`sihsim_airplane`). Tier 1 is identical on both and matches the source review: 20 passed, 9 failed (all compatibility errors), 2 NA each.

**Tier 1** (both vehicle types)

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

| Test | Multicopter | Fixed-wing |
|---|---|---|
| `compat_starts_mission` (0/0, armed) | PASS — took off, `MISSION_CURRENT` 0 → 1: param2 = 0 runs the whole mission | PASS — took off to 32 m, `MISSION_CURRENT` 0 → 1 |
| `compat_first_item_honoured` (B, C) | PASS — **SUPPORTED**: `MISSION_CURRENT` went straight to 2, then 3 | PASS — **SUPPORTED**: same |
| `compat_last_item_honoured` (A, B) | **FAIL** (compat) — **NOT SUPPORTED**: after item 1 the mission moved on to 2; after item 2, to 3 | inconclusive (plain FAIL, no verdict) — the plane crashed after takeoff, see below |
| `obs_arms_when_disarmed` | **armed and took off** | **armed and took off** |

Fixed-wing findings from PX4's own flight logs (not MISSION_START compliance, but they shaped the tests):

- **PX4 SIH fixed-wing on this build stalls as soon as a mission takeoff completes**, and falls to the ground. At the takeoff → waypoint switch the thrust setpoint goes NaN for one sample, airspeed collapses from 18 to 8 m/s within a second, and the longitudinal controller then holds the throttle near zero while stalled. Reproduced with the same mission started through MAVSDK (`mission_raw.start_mission()`, no MISSION_START), so it isn't caused by this command. A crashed plane keeps "driving" along the line on the ground, and the mission logic carries on past the items, so the Last Item test gives no verdict once the vehicle has come back down (rule 10).
- **A fixed-wing started at a waypoint from the ground banks towards it on the runway** (roll 25–42° within a second of arming) and is left tipped over: "Preflight Fail: Attitude failure (roll)", unarmable. The First Item result stands — `MISSION_CURRENT` showed the item — and the flight cleanup restarts the stack after any start that never got airborne.
- **With the NAV_TAKEOFF position at home, the climb-out turned back through ~200°** at up to 52° of bank, towards that position. The mission now puts the takeoff position, and every waypoint, ahead on the line the vehicle faces.

Each airborne flight ends in a stack restart (RTL doesn't land within 30 s on either vehicle type). About 10 minutes per vehicle type for both tiers.

### ArduPilot

Not yet run. An attempt on `--sitl-instance=1` never connected, and the command helpers address system ID 1 anyway, so the next attempt should use instance 0 (root `CLAUDE.md` § Caveat: raw command/mission helpers). Source predicts: no mission → `FAILED` (Copter/Rover) or `ACCEPTED` (Plane); any non-zero param1/param2, or NaN in either, → `DENIED`. That makes both param verdicts **REJECTED (NACKed)**, a legitimate "not supported" under rule 4a, and fails "param1/param2 NaN not denied". MISSION_START doesn't arm on ArduPilot.

## Spec gaps

Things the XML could usefully tighten. Both stacks agree on each one (root `CLAUDE.md` rule 3):

1. **No "run to the end" value for Last Item.** Taken literally, param2 = 0 means "end after item 0". Yet 0/0 is what GCSs send to run the whole mission, the only value ArduPilot accepts, and (above) what PX4 treats as the whole mission. The XML should say what 0 (or NaN) means.
2. **No sentinel for First Item.** Both stacks DENY NaN in param1, so the mandatory "defined param tolerates its sentinel" check fails on both. Either NaN should be defined (e.g. "first item" or "current item"), or param1 should be documented as mandatory, which would exempt it (`sentinel_policy="deny_required"`).
3. **"Start from the current item."** PX4 gives param1 = −1 that meaning, below the XML's `minValue=0`, and this repo's own `do_set_mission_current` Tier 2 relies on it. DO_SET_MISSION_CURRENT already defines −1 as "keep the current item", so MISSION_START could adopt the same value.
4. **No mission / no such item.** The XML doesn't say what a start with nothing to run should return. PX4 DENIES, ArduCopter FAILs, ArduPlane ACCEPTs. The tests assert only "not accepted".
