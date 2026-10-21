# NAV_LAND and NAV_VTOL_LAND — notes for all four directories

`tests/mission/nav_land/`, `tests/mission/nav_vtol_land/`, `tests/command/nav_land/`, `tests/command/nav_vtol_land/`.
Result tables are in each directory's README; this file holds the design, the source predictions and what was learned.

## Design

- **One flight routine per protocol, shared** — `tests/land_helpers.py`: `fly_mission()`, `fly_command()`, a `LandingWatch` telemetry sampler and the verdict helpers. Each `test_flight.py` is a thin set of tests over it, with its own cache and report key.
- **Two flights carry every "is it honoured" param** (rules 4c, 7): target due N of the start with yaw 135° (and, for NAV_VTOL_LAND, approach altitude 50 m), then due E with 315° (70 m). Position and yaw are judged at touchdown (rule 4b); approach altitude as the highest altitude reached once the landing starts — both values are above the 30 m the landing starts from, so honouring one shows as a climb.
- **NACK attribution.** Several params ride on one landing, so a NACK of the combined set is attributed: mission — each value is uploaded on its own (no flight); command — each is sent on its own in the air, then the landing is flown with the accepted ones. A `start_mission()` refusal counts as the stack declining the mission (rule 4a) and is recorded as "not supported" at command level.
- **Touchdown** = `ON_GROUND` or disarmed, after having been airborne. Cleanup waits for the stack's auto-disarm — `_rtl_and_land()` on a landed, still-armed vehicle would take it off again.
- **Getting airborne is setup, keyed on the HEARTBEAT type** (pattern #11). A VTOL reaches the landing in forward flight: by mission, NAV_VTOL_TAKEOFF + a transit waypoint 500 m SW; by command, a short get-airborne mission (`_forward_flight_via_mission`) with the landing command sent on its outbound leg. Anything else takes off and lands from a hover. Targets: 60 m (hover), 250 m (forward flight).
- **Yaw values avoid every heading a landing ends on by itself**: approach bearings 0°/90°, the VTOL's outbound leg 225° and inbound legs ~25–65°. The first run used 225°; a commanded VTOL landing touched down at 223° and that value "passed" alone — the second value is what caught it (rule 4c doing its job), and 225° was replaced by 315°.
- **Targets are measured from where the vehicle sits now**, not from one fixed home: the second flight starts where the first landed, and with a fixed home its 'E' target lay at 135° from the new start — the other flight's yaw value.
- **Untestable, with reasons (rule 10d)**: ground Altitude (flat simulated terrain — a flight with z = 15 m records what a wrong ground level does), Abort Alt (no aborted landing), Land Mode (no beacon), Land Options (DEFAULT is autopilot-defined, so no fixed approach shape). A NACK for a real value is still recorded as the verdict.

## Expected from source vs observed — PX4 v1.18.0-beta1-660-g61907962c9

Blind read first (pattern #5); every prediction held.

| | Source | Observed |
|---|---|---|
| NAV_LAND command | `Commander`: switches to AUTO_LAND, "Landing at current position" — no param is read. Mask `{ 21, 0x7B, 0x7B }` allows params 1, 2, 4–7 | Lands in place; lat/lon and yaw accepted and ignored (**compat errors**); a real param3 DENIED. A VTOL in forward flight back-transitions at once and lands where it is |
| NAV_VTOL_LAND command | No handler outside the mission executor → `Commander`'s default | `UNSUPPORTED` on multicopter and VTOL |
| NAV_LAND mission item | `handleLanding`: fly to the point at the current altitude, then descend; "ignore yaw for landing items"; on a VTOL replaced by NAV_VTOL_LAND (`force_vtol()`) | Lat/lon honoured (≤ 0.1 m); yaw stored but ignored (**compat error**); dogleg approach |
| NAV_VTOL_LAND mission item | As above, with a back-transition at the point; mission mask `0x7C` (params 3–7): Land Options NACKed, Approach Altitude allowed but neither stored nor used | VTOL: lat/lon honoured, back-transition ~22 m out at 30 m, lands in MC; yaw ignored, Approach Altitude dropped to 0 at upload and ignored in flight (**compat errors**). Multicopter: upload accepted, mission refused at start (`FeasibilityChecker::checkItemsFitToVehicleType`) |

Version notes against `tests/command/nav_land/README.md`'s older PX4 runs: Abort Alt 10 / −5 m is now ACCEPTED (was DENIED), and an out-of-range lat/lon is now ACCEPTED (was DENIED).

## PX4 PR #28764 ("fix(navigator): honor yaw in LAND and VTOL_LAND") — verified 2026-10-13

Built `17ed9c8c42` (the PR head — 17 unrelated commits plus the fix on top of the container's commit, none touching the navigator) and ran all eight files on both vehicles. A/B against the container build:

| Yaw check | container `61907962c9` | PR `17ed9c8c42` |
|---|---|---|
| mission NAV_LAND, multicopter | FAIL — 135° → 0.0°, 315° → 90.1° | **PASS — 135.1°, 315.1°** |
| mission NAV_LAND, VTOL | FAIL — 26.1°, 62.6° | **PASS — 135.0°, 314.9°** |
| mission NAV_VTOL_LAND, VTOL | FAIL — 26.2°, 62.7° | **PASS — 135.0°, 315.0°** |
| mission NAV_VTOL_LAND, multicopter | not flown (mission refused at start) | same |
| command NAV_LAND, multicopter | FAIL — 359.5°, 359.4° | FAIL — 0.7°, 0.1° (unchanged) |
| command NAV_LAND, VTOL | FAIL — 223.0°, 223.6° (the heading it was flying) | FAIL — 223.8°, 224.0° (unchanged) |
| command NAV_VTOL_LAND | UNSUPPORTED | UNSUPPORTED |

- **The fix works for mission items and the tests verify it**: the same test flips from FAIL (compat) to PASS on every vehicle/command combination that flies, with both values within 0.1° and nothing else changing (touchdown still ≤ 0.1 m from the target; the landing takes ~3 s longer for the align step).
- **It does not change the command.** The PR only touches the mission executor (`mission_base.cpp`); a commanded NAV_LAND goes through `Commander` to AUTO_LAND and still ignores yaw (and lat/lon). If "yaw respected" is meant to cover the command too, that needs a separate change.
- Approach Altitude and every Tier 1 result are identical on both builds.

## Environment lessons

- **PX4's SIH container image runs without Docker**: `scripts/fetch_px4_sitl_image.py` unpacks `px4io/px4-sitl` and `--px4-sitl` points at it (`SETUP.md`). The SIH `standard_vtol` in this build flies full missions (takeoff, transition, back-transition, landing) and a commanded takeoff — root `CLAUDE.md` future-work #10's "never leaves the ground" doesn't reproduce when the mission ends in a landing.
- **Command tests don't work on a non-zero `--sitl-instance`.** `tests/command/conftest.py`'s raw senders target system ID 1 (`_DRONE_SYSID`; also `_RAW_DRONE_SYSID` in the mission conftest and `request_message_rate()`), and PX4 `-i N` is system ID N+1 — so every raw COMMAND_INT is silently ignored: no ACK, and a commanded takeoff that "arms and never climbs" (the arm goes through MAVSDK, which addresses the vehicle correctly). First misread here as a SIH VTOL takeoff problem. Mission tests through `mission_raw` are fine on any instance. Run command files on instance 0 until the helpers take the vehicle's own system ID.
- **Don't run mock mode while a standalone session is running** — the paired-mode fixture kills instance-0 PX4 (it did, mid-flight).
- Tier 1 before Tier 2 for the mission files, as separate sessions or in that order on the command line: Tier 1's "accepted as a mission item" would otherwise overwrite Tier 2's "refused at start → not supported" in the merged report.
