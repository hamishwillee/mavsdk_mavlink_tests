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
found and fixed in the same investigation. See `test_param1_loiter_height_accepted`'s
docstring and the finding below.

## Open finding, not yet root-caused (2026-09-17)

`test_param1_loiter_height_accepted` (param1=20.0 via COMMAND_INT, verifying
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
