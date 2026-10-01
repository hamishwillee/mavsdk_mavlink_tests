# NAV_VTOL_TAKEOFF — mission item notes

Added 2026-10-01. Built on `Tier1MissionTestBase` (`test_protocol.py`) plus a
Tier 2 flight file (`test_flight.py`) using every current convention from day
one: rule 8/9 naming, rule 4a/4b/4c verdicts (two values per "is it
honoured" check, enum values per value), rule 7 cached flights, rule 10
param coverage, Tier 2 pattern #12 message watcher. Command-protocol sibling:
`tests/command/nav_vtol_takeoff/`.

## What each param's verdict is based on

| Param | Tier 1 | Tier 2 "is it honoured" |
|---|---|---|
| 1, 3 Empty | inherited sentinel pair | not applicable (`compat_empty_params` records the sentinel facts) |
| 2 Transition Heading (enum) | every value 0–4 stored-or-NACKed, plus 5 | SPECIFIED (from the yaw flights), NEXT_WAYPOINT ×2 directions, TAKEOFF — heading at transition start. VEHICLE_DEFAULT/ANY have no fixed target: Tier 1 only |
| 4 Yaw Angle | 90° stored (with SPECIFIED, retried with VEHICLE_DEFAULT if NACKed), NaN kept, 0° not aliased, −90°/450° characterised | 135°/225° with param2 = SPECIFIED — heading at transition start |
| 5/6 Lat/Lon | round-trip, INT32_MAX kept | targets 400 m N/E — closest approach while the item is current |
| 7 Altitude | round-trip, NaN characterised | 30/50 m — altitude when the item completes |
| command level | — | took off and began the transition *during the takeoff item* (the XML: "takeoff … and transition to forward flight") |

Why the transition heading, not the vehicle's heading at some fixed time:
the command's own text is "transition to forward flight with specified
heading", and VTOL_TRANSITION_HEADING is literally "Direction of VTOL
transition". The heading is read when `vtol_state` first reports
TRANSITION_TO_FW/FW — PX4 aligns before transitioning, so that's the heading
the transition starts on.

Whether a vehicle is a VTOL comes from its own `EXTENDED_SYS_STATE.vtol_state`
(UNDEFINED for anything else), never `--vehicle-type` — the command-level
test reports NA for a non-VTOL, since the XML says such vehicles should
ignore the command.

## Blind source predictions (written before the first run, Tier 2 pattern #5)

**PX4 (binary built from `3fe7e7af3`; source read at `b0cc996eee`)**
- `mavlink_mission.cpp` stores only param4 for cmd 84 (`wrap_2pi`), and
  `mavlink_command_params.hpp` `{ 84, 0x78, 0x7C }` (mission mask 0x78 =
  params 4–7) → a non-zero param1/2/3 should be NACKed.
  **Confirmed at Tier 1**: param1/param3 = 1.0 and every Transition Heading
  value except VEHICLE_DEFAULT → `INVALID_ARGUMENT`; yaw stored and wrapped
  (−90° → 270°, 450° → 90°); NaN yaw and INT32_MAX lat/lon kept.
- `mission.cpp`: climbs in hover to the item altitude, sets the yaw setpoint
  to the bearing *toward the item's lat/lon* (param4 is overwritten with NaN
  first), transitions, then flies to the lat/lon as a fixed-wing waypoint at
  the item altitude. Expect Altitude and Lat/Lon honoured; Yaw ignored —
  but moot, since SPECIFIED is NACKed (REJECTED is a PASS).

**ArduPlane QuadPlane (master `31d9b842cb`)**
- `AP_Mission.cpp` stores no param1–4 for this command, and its NaN check
  allows NaN only in param4. Expect param2/param4 non-default values to come
  back as 0 (accepted but not stored → compatibility error) and NaN in
  param1/2/3 NACKed.
- `quadplane.cpp::do_vtol_takeoff`: "we always use the current location in XY
  for takeoff" — lat/lon deliberately zeroed; altitude relative to the
  current height. The item completes in VTOL mode at altitude; the
  transition happens on the following item. Expect Altitude honoured,
  Lat/Lon a compatibility error, and the command-level test to FAIL because
  the transition isn't part of the command.

## Results

### Cross-stack summary, 2026-10-01

PX4 VTOL column: binary `3fe7e7af3` (morning) and re-confirmed on a fresh
worktree of `main` `7cb65787b3` (`~/github/px4/PX4-Autopilot-main`, built for
this) — identical verdicts, Tier 2 8 PASS / 2 NA, coverage COMPLETE, the
INT32_MAX fly-away reproduced (2343 m).

| Param | PX4 VTOL (Gazebo) | ArduPlane QuadPlane | PX4 multicopter (SIH) | ArduCopter |
|---|---|---|---|---|
| command level (takes off + transitions in the item) | SUPPORTED | **FAIL** — completes the item in hover, transitions on the next item | NA — refuses to start the mission (DENIED); not a VTOL | NA — not a VTOL; flies it as a plain takeoff |
| 7 Altitude | SUPPORTED (29.6 / 48.3 m) | SUPPORTED (30.1 / 50.3 m) | REJECTED (start refused) | SUPPORTED (30.0 / 50.0 m) |
| 5/6 Lat/Lon | SUPPORTED (27 / 29 m) | **FAIL** — climbs in place (400 m off) | REJECTED (start refused) | **FAIL** — climbs in place |
| 4 Yaw Angle | REJECTED (SPECIFIED NACKed) | **FAIL** — not stored; transition on the ground heading | REJECTED | NA (no transition) |
| 2 Transition Heading | REJECTED (all but VEHICLE_DEFAULT NACKed) | **FAIL** — not stored; always transitions straight ahead | REJECTED | NA (no transition) |
| param7 is used as | **both** transition and final altitude | **final altitude only** (transition is on the next item) | — | — |

**Altitude meaning (`test_vtol_takeoff_obs_altitude_meaning`).** For the
mission item, both stacks that fly it end the item at the commanded
altitude; only PX4 also transitions there (QuadPlane hovers to it and
transitions later). For the standalone command PX4 transitions at param7
and then climbs to VTO_LOITER_ALT (see `tests/command/nav_vtol_takeoff/`).
So param7 means different things on the two stacks, and on PX4 it means
different things for the command and the mission item — a candidate for a
spec clarification ("Altitude" — transition altitude, or where the takeoff
ends?).

**Non-VTOLs.** PX4 multicopter accepts the upload but refuses to start the
mission ("Switching to Mission is currently not available") — the XML's
"should be ignored", done at start rather than at upload. ArduCopter flies
it as an ordinary takeoff (altitude honoured, lat/lon ignored). VTOL-ness is
read from the HEARTBEAT type: ArduCopter reports `vtol_state` MC
(`GCS_MAVLink_Copter.h`), which made the first version of these tests treat
it as a VTOL.

**ArduPilot Tier 1 (QuadPlane and ArduCopter identical)**: params 1–4 are
never stored — every Transition Heading value comes back 0, yaw 90° comes
back 0, yaw NaN comes back 0 (the sentinel's meaning lost), −90°/450° come
back 0. NaN is NACKed in params 1/2/3/7 (only param4 may be NaN), a real
value in Empty params 1/3 is accepted, and INT32_MAX lat/lon is NACKed.
All compatibility errors except the NaN-in-defined-param ones, which carry
the documented ArduPilot nan_mask reason.

### PX4 v1.17.0 (tagged release, `~/github/px4/PX4-Autopilot-v1.17`), 2026-10-01

**Tier 1 — v1.17 has no param validation for this item** (the `mavlink_command_params.hpp`
mask table is newer): real values in Empty params 1/3 are accepted (compatibility
error; `main` NACKs them), and every non-default Transition Heading value is
accepted and stored as 0 (compatibility error; `main` NACKs them). Yaw is stored
and wrapped (90° kept, −90° → 270°, 450° → 90°), NaN yaw/altitude and INT32_MAX
lat/lon kept. So the `main` mask work turned v1.17's silent drops into proper
NACKs — the same verdict change mavlink-compat-data should show by version.

**Tier 2 — mostly inconclusive: the v1.17 Gazebo `standard_vtol` keeps aborting
its transition.** Each time: "No airspeed sensor detected. Switch to
non-airspeed" → "Airspeed sensor healthy, start using again" → "Quad-chute
triggered" → "VTOL fixed-wing system failure detected" (latched — the vehicle
stays unarmable until a restart). Six confirmed quad-chutes (PX4's own
"Quad-chute triggered", 5 of 6 mission-item transitions and 1 command
transition) — versus none on either `main`-era binary. (A first version of
the abort check also stopped watching too early and mis-flagged one `main`
flight; it now waits for FW before judging.) One clean flight: 30 m → item
complete at 27.7 m after a transition at 29.4 m. The harness now classifies an
aborted transition as inconclusive (not a compatibility error) and restarts the
stack after one (`_require_airborne`, `_fly`'s cleanup). A simulator problem,
not NAV_VTOL_TAKEOFF behaviour.

### Environment note — PX4 checkout changed mid-run (2026-10-01 11:32)

Gazebo loads models/worlds from the PX4 *source* tree at every boot, so a
checkout change takes effect without a rebuild. At 11:31–11:34 the checkout
was moved to upstream `main` (pulled) and then to another branch, while the
binary stayed the 06:53 build. Every Gazebo boot after that (first at the
11:39 restart) produced no GLOBAL_POSITION_INT at all — HOME_POSITION
arrived, the EKF never got a global position (the message watcher reported
it every time; clearing the saved parameters didn't help; PX4 SIH and
ArduPilot were unaffected). PX4 VTOL results above are all from before
11:32. If a Gazebo run suddenly never gets position, check
`git -C <PX4> reflog` against the binary's mtime first.

### PX4 VTOL detail (binary reports git `3fe7e7af3`, Gazebo `gz_standard_vtol`, runs before 11:32)

Tier 1 21/21 PASS; Tier 2 7 PASS / 2 NA, 0 compatibility errors; param coverage
COMPLETE. Every source prediction held.

| Param | Verdict | Evidence |
|---|---|---|
| command level | SUPPORTED | took off and began the transition during the takeoff item (29.4 m / 49.3 m) |
| 7 Altitude | SUPPORTED | 30 → 29.6 m, 50 → 48.3 m at item completion |
| 5/6 Lat/Lon | SUPPORTED | 400 m N/E targets: closest approach 27.1 / 29.2 m; transition started facing the target (3.8° / 95.7°) |
| 4 Yaw Angle | REJECTED (NACKed) | param2 = SPECIFIED is NACKed, so no yaw can be applied; with VEHICLE_DEFAULT a yaw of 135° is stored but the transition went 245.7° (characterisation, rule 3) |
| 2 Transition Heading | REJECTED (NACKed) | every value except VEHICLE_DEFAULT NACKed at upload (`INVALID_ARGUMENT`) |
| 1, 3 Empty | n/a | a real value is NACKed |

**PX4 finding — INT32_MAX lat/lon is flown as a real coordinate.**
`mavlink_mission.cpp:1487` converts a position item's x/y with `x * 1e-7` and
no sentinel check, so INT32_MAX becomes latitude 214.7°. The upload is
accepted and the value round-trips (×1e-7 then ×1e7 gives INT32_MAX back),
so Tier 1 passes — but in flight the vehicle transitions toward that
impossible point and keeps going: the takeoff item never completed, and it
was 2297 m from home after 180 s (`test_vtol_takeoff_obs_from_current_position`).
Safety-relevant (a fly-away on a value the spec reserves). Every PX4
position mission item goes through the same conversion; whether each one
then flies toward it depends on how it uses the lat/lon — not tested here.
Characterisation test, so it doesn't fail; worth a PX4 issue.

**Transition heading with the takeoff point overhead is arbitrary on PX4.**
PX4 faces "the bearing to the takeoff point"; when that point is the
vehicle's own position the bearing is noise — 325°, 228°, 159°, 246° in four
flights. Not a compliance question (VEHICLE_DEFAULT has no fixed target).

Tuning data (pattern #2): position closest approach 27–29 m (tolerance 50 m,
could tighten to ~35 m once another stack has been seen); whole Tier 2 file
~12 min.
