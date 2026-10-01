# NAV_VTOL_TAKEOFF — command protocol notes

Built on `Tier1CommandTestBase`/`CommandSpec` (see `tests/command/CLAUDE.md` §
"Shared Tier 1 infrastructure") — the seven mandatory checks are inherited;
this file's bespoke tests are `test_param{1,2,4}_*`/`test_location_*`/etc.
per root `CLAUDE.md` rule 9.

## Message-type exclusivity (check 7) — a real, verified PX4 fix, 2026-09-17

`SPEC.has_location=True` drives the inherited `test_hasLocation_rejects_command_long`
(see `tests/command/CLAUDE.md` § Mandatory common tests, check 7). This is
the one instance in this repo where that check is a genuine **PASS**, not
the `XFAIL` seen everywhere else (`NAV_TAKEOFF`, `NAV_LAND`, `DO_REPOSITION`
— see their own CLAUDE.md/README entries).

**Source-verified mechanism**: PX4 commit `83e7afba56` ("fix(mavlink):
reject standalone NAV_VTOL_TAKEOFF sent as COMMAND_LONG", branch
`hamishwillee/reject-vtol-takeoff-command-long`, checked out at
`~/github/PX4/PX4-Autopilot`) adds `MavlinkReceiver::command_is_int_only()`
— a small switch statement (currently listing only cmd=84) checked in
`handle_message_command_both()` before any param validation: if the
incoming message was `COMMAND_LONG` and the command is in that switch, PX4
immediately ACKs `MAV_RESULT_COMMAND_INT_ONLY(8)` and returns, never
reaching Commander/Navigator at all. The commit's own rationale (from its
message): `MAV_CMD_NAV_VTOL_TAKEOFF` carries a lat/lon loiter position, but
`COMMAND_LONG` has no `MAV_FRAME` field, so PX4 has no way to know whether
those coordinates are global degrees or a local frame — `COMMAND_INT`
carries an explicit frame and higher-resolution scaled int32 lat/lon, so
the command now requires it.

**Confirmed independent of MAVSDK**: verified twice — once through this
repo's own test suite (`probe_command_long_all_acks`/`effective_ack`), and
once via a raw `pymavlink` connection straight to the PX4 SITL UDP port
(`udpin:0.0.0.0:14540`), bypassing `mavsdk_server` entirely. Both show the
identical, deterministic result — `MAV_RESULT_COMMAND_INT_ONLY(8)`
regardless of the actual param5/6 values sent (tested with real degrees,
`0.0`, `NaN`, and the original raw-scaled test value; all four gave `8`).
This rules out a MAVSDK-side interception and a value-dependent heuristic
— the rejection is unconditional for this one command, on the message
type alone.

**Narrow scope, confirmed by source, not assumed**: `command_is_int_only()`'s
switch currently has exactly one case (`MAV_CMD_NAV_VTOL_TAKEOFF`). Every
other `hasLocation="true"` command in this repo's test suite
(`NAV_TAKEOFF`, `NAV_LAND`, `DO_REPOSITION`) still `XFAIL`s
`test_hasLocation_rejects_command_long` against the very same PX4 build —
this is a real, currently uneven rollout of the exclusivity rule across
commands, not evidence that the rule itself is unenforced everywhere.

**Companion fix, same investigation, different problem**: `aad2f0f3`
("fix(mavlink): allow p1/p2 for standalone NAV_VTOL_TAKEOFF command",
branch `hamishwillee/fix-vtol-takeoff-param-mask`) is a separate, earlier
fix to `mavlink_command_params.hpp`'s param-mask table (`{ 84, 0x78, 0x7B }`,
was `{ 84, 0x78, 0x7C }`) — unrelated to message-type exclusivity, but
found and fixed in the same investigation. See `test_nav_vtol_takeoff_param1_loiter_height_accepted`'s
docstring and the finding below.

## Open finding, not yet root-caused (2026-09-17)

`test_nav_vtol_takeoff_param1_loiter_height_accepted` (param1=20.0 via COMMAND_INT, verifying
the `aad2f0f3` fix above) passed cleanly (`ACCEPTED`) the first two times
it was run against PX4 VTOL SITL this session. Re-run later the same
session — against the same running binary, confirmed by an unchanged file
mtime throughout — it started returning `DENIED(2)` instead, reproduced
both through the test suite and through a direct raw-pymavlink probe
(ruling out a MAVSDK/harness artifact). `param1=0.0` still gets `ACCEPTED`
on the same probe, so this isn't a wholesale regression of the fix, just a
changed outcome for this one non-zero value. Not root-caused: the binary
itself never changed, so the most likely explanation is persisted SITL
state (parameters or the `dataman` file) in the shared
`build/px4_sitl_default` working directory being altered by some other
process between the two test runs — not confirmed. Flagging for whoever
next investigates this command rather than asserting a cause that wasn't
actually verified.

## Checkout hygiene note (2026-09-17)

Partway through this investigation, the PX4-Autopilot checkout's working
branch was found switched from `hamishwillee/reject-vtol-takeoff-command-long`
to an unrelated branch (`board/gpilot-p1`, fetched from a third-party PR,
`#28681` by `gokhan-iha`), with a further commit (`a956a8f75a`,
"docs(docs): Subedit") on top — none of this done by the test session
investigating this command. Both fix commits above are safe (pushed to
`origin` before the switch, confirmed via `git reflog`), but anyone
re-verifying this command against that same checkout should first run
`git status`/`git log -1` to confirm which commit is actually checked out
— it may no longer be either fix branch.

## Tier 2 (`test_flight.py`, added 2026-10-01)

Same design as `tests/mission/nav_vtol_takeoff/` (read its CLAUDE.md): two
values per "is it honoured" param, judged where the vehicle *settles*
(rule 4b — mean over the last 30 s of a 150 s watch, so a fixed-wing's
loiter centre counts), headings at the start of the transition, VTOL-ness
from the HEARTBEAT type, an aborted transition (quad-chute) inconclusive and
followed by a restart.

| Param | PX4 `main` VTOL (Gazebo, `7cb65787b`) | ArduPlane QuadPlane | PX4 multicopter (`main`) | ArduCopter |
|---|---|---|---|---|
| command level | SUPPORTED — takes off and transitions | UNSUPPORTED (mission-only) — all flight tests skip | NA (not a VTOL) — ACCEPTED and flown as a multicopter takeoff | NA — DENIED (armed outside GUIDED) |
| 7 Altitude | **FAIL (compat)** — transitions at 29.1 / 49.4 m, settles at ~80 m (VTO_LOITER_ALT) | — | to param7 (30.0 / 50.0 m) | REJECTED |
| 5/6 Lat/Lon | SUPPORTED — loiter centre 5–8 m from both 400 m targets | — | **FAIL (compat)** — climbs in place | REJECTED |
| 4 Yaw (with SPECIFIED) | SUPPORTED — transition 131° / 222° for 135° / 225° | — | NA | NA |
| 2 Transition Heading | **FAIL (compat)** — SPECIFIED honoured, TAKEOFF accepted but ignored (transition 180° toward the loiter point, not the 91° ground heading) | — | NA | NA |
| param7 used as | **transition altitude only** | — | NA | NA |

Tier 1 on `7cb65787b`: param1 accepted (compatibility error — see the
loiter-height note), param3 correctly DENIED, COMMAND_LONG accepted for this
hasLocation command (compatibility error; branch commit `83e7afba56` isn't
on `main`). So the `aad2f0f3` mask fix (param1/param2 allowed) has landed
upstream, which is why SPECIFIED now works.

**PX4: param7 is the transition altitude, not where the takeoff ends.**
`navigator_main.cpp` passes param7 to `setTransitionAltitudeAbsolute()`;
after the transition `vtol_takeoff.cpp` loiters at home + `VTO_LOITER_ALT`
(default 80 m). A compatibility error under rule 4b, but the XML only says
"Altitude" — and PX4's *mission item* uses param7 as both transition and
final altitude. A candidate spec clarification.

**PX4: param1 ("loiter height") is dead.** `setLoiterHeight(cmd.param1)`
stores `_loiter_height`, which nothing reads; param1 = 40 still settled at
80.2 m. (Corrects this file's 2026-09-17 notes and `test_command.py`.)

**PX4: INT32_MAX lat/lon is flown as a coordinate.** Accepted; the vehicle
transitions toward latitude 214.7° — settled centre 1689 m from home at the
end of the watch. Same fly-away as the mission item.

**PX4 bug: a below-ground NAV_VTOL_TAKEOFF sent while disarmed makes the next
valid one a no-op.** Isolated by A/B on a fresh SIH boot each time
(`scripts/px4_disarmed_vtol_takeoff_ab.py`, 2 runs per variant): a disarmed
NAV_VTOL_TAKEOFF with a real position but z = 30 in frame 6 — which PX4 reads
as 30 m AMSL, ~460 m below the vehicle, since it ignores the COMMAND_INT
frame for z — is ACCEPTED; the next armed, valid NAV_VTOL_TAKEOFF is also
ACCEPTED, but the vehicle stays in HOLD and never climbs. No disarmed send,
a valid disarmed send, or a disarmed send with INT32_MAX lat/lon and a valid
altitude: the takeoff climbs normally. This caused every "first flight of the
session doesn't climb" seen here — after the (now removed) disarmed support
probe, or after `test_command.py`'s disarmed probes in the same session. Run
`test_flight.py` in its own session (or after a restart) for clean data.

**ArduPilot bug: COMMAND_LONG with an out-of-range lat/lon crashes SITL.**
`GCS_Common.cpp` `convert_COMMAND_LONG_loc_param()` (master `31d9b842cb`)
does `return param * 1e7;` into an `int32_t` with no range check (NaN is
handled). param5/6 = `float(INT32_MAX)` — `test_nav_vtol_takeoff_latlon_int32max_command_long`
— overflows; SITL traps the FP exception and aborts
(`ERROR: Floating point exception - aborting`), taking every later test with
it. Reproduced in isolation; a control with real coordinates gets a normal
ACK (`COMMAND_INT_ONLY`). Generic to every location command sent as
COMMAND_LONG; on hardware it's undefined behaviour. Worth an ArduPilot issue.

**PX4 v1.17.0** (tagged release): Tier 1 — param1/param3 real values and
COMMAND_LONG accepted (no mask table, no int-only rule), all compatibility
errors `main` has since fixed. Tier 2 mostly inconclusive: the v1.17 Gazebo
`standard_vtol` quad-chutes during transitions (airspeed sensor dropout; see
the mission CLAUDE.md). The one clean 50 m flight matched `main`
(transition 49.4 m, settled 80.2 m). PX4 multicopter on v1.17: both takeoffs
climbed (30.1 / 50.9 m).

**Environment lessons from these runs**: see the mission CLAUDE.md (PX4
checkout changed mid-run; fresh-boot position delay; quad-chutes).
