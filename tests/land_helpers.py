"""
Shared Tier 2 scaffolding for the landing commands — MAV_CMD_NAV_LAND (21) and
MAV_CMD_NAV_VTOL_LAND (85), as a command and as a mission item.

The two commands share params 4-7 (Yaw, Latitude, Longitude, ground Altitude) and
the same question — where and how does the vehicle touch down — so the four
test_flight.py modules (tests/command/nav_land, tests/command/nav_vtol_land,
tests/mission/nav_land, tests/mission/nav_vtol_land) share one flight routine
per protocol and one set of verdict helpers. Each module keeps its own tests,
caches and report keys.

Conventions (root CLAUDE.md):
- Every "is it honoured" param flies two values, combined into one verdict
  (rule 4c): NACKed = REJECTED (PASS), honoured = SUPPORTED (PASS), accepted but
  not honoured = compatibility error (rule 4/4a).
- Values are judged where the vehicle ends up (rule 4b): the touchdown point and
  the heading at touchdown.
- Several params ride on the same two flights (rule 7): flight A lands 'N' of
  the start with the first yaw, flight B 'E' with the second. A NACK of the combined
  value set is attributed per param (see fly_command()/fly_mission()) so one
  param's rejection can't be recorded against another.
- Nothing is gated on --vehicle-type (rule 5). How to get airborne IS keyed on the
  vehicle's own HEARTBEAT type (Tier 2 pattern #11 — setup, not a verdict): a VTOL
  is put into forward flight first, because a landing from a hover says nothing
  about "land using VTOL mode".
"""

import asyncio
import json
import logging
import math
import os

import pytest
from mavsdk.plugins.mission_raw import MissionItem, MissionRawError
from mavsdk.plugins.telemetry import LandedState, VtolState

from tests import report
from tests.command.conftest import probe_command_int
from tests.flight_helpers import (
    AIRBORNE_THRESHOLD_M,
    MAV_AUTOPILOT_ARDUPILOTMEGA,
    _arm_and_send_takeoff,
    _dist_m,
    _get_heading,
    _get_home_position,
    _get_position,
    _offset_lat_lon,
    _rtl_and_land,
    _wait_armable,
    _wait_for_climb_complete,
    record_compat_json,
    record_tier2_detail,
    record_tier2_param_verdict,
    request_message_rate,
    vehicle_is_vtol,
)

log = logging.getLogger(__name__)

NAN = float("nan")
INT32_MAX = 0x7FFF_FFFF

NAV_WAYPOINT, NAV_LAND, NAV_TAKEOFF, NAV_VTOL_TAKEOFF, NAV_VTOL_LAND = 16, 21, 22, 84, 85

TAKEOFF_ALT_M = 30.0  # relative; the altitude every landing starts from

# Landing targets, two directions (rule 4c). A hover start lands HOVER_TARGET_M
# away; a forward-flight start needs room to turn and slow down.
TARGET_BEARINGS = (("N", 0.0), ("E", 90.0))
HOVER_TARGET_M = 60.0
FORWARD_TARGET_M = 250.0
TOUCHDOWN_TOLERANCE_M = float(os.environ.get("LAND_TOUCHDOWN_TOLERANCE_M", "10.0"))

# Yaw at touchdown, two values (rule 4c) — away from every heading a landing ends
# on by itself: the N/E approach bearings (0°, 90°), the VTOL's outbound leg
# (TRANSIT_BEARING_DEG, 225° — a commanded landing there touched down at 223°, which
# a 225° yaw value would have matched by coincidence, 2026-10-13) and its inbound
# legs (~25-65°).
YAW_VALUES_DEG = (135.0, 315.0)
YAW_TOLERANCE_DEG = float(os.environ.get("LAND_YAW_TOLERANCE_DEG", "20.0"))

# Approach altitude (NAV_VTOL_LAND param3), two values, both above TAKEOFF_ALT_M so
# honouring one shows as a climb that a plain descent can't produce.
APPROACH_ALT_VALUES_M = (50.0, 70.0)
ALT_TOLERANCE_MIN_M = 5.0
ALT_TOLERANCE_FRAC = 0.2

# Forward-flight start (VTOL): the mission's transit waypoint / where a commanded
# landing is sent from.
TRANSIT_OFFSET_M = 500.0
TRANSIT_BEARING_DEG = 225.0  # south-west — away from both landing targets
FORWARD_FLIGHT_S = 15.0      # fixed-wing time before a landing is commanded

TRANSFER_TIMEOUT_S = 30.0
CLIMB_TIMEOUT_S = float(os.environ.get("LAND_CLIMB_TIMEOUT_S", "120.0"))
TRANSITION_TIMEOUT_S = float(os.environ.get("LAND_TRANSITION_TIMEOUT_S", "60.0"))
# From "landing commanded / mission started" to touchdown.
TOUCHDOWN_TIMEOUT_S = float(os.environ.get("LAND_TOUCHDOWN_TIMEOUT_S", "300.0"))
DISARM_WAIT_S = 15.0
# A fresh (re)boot can send HOME_POSITION well before GLOBAL_POSITION_INT (pattern #12).
POSITION_READY_TIMEOUT_S = float(os.environ.get("LAND_POSITION_READY_TIMEOUT_S", "120.0"))

_ACCEPTED = (0, 5)  # MAV_RESULT_ACCEPTED, IN_PROGRESS


def angle_diff(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def target_from(home, distance_m: float, bearing_deg: float) -> tuple[float, float]:
    n = distance_m * math.cos(math.radians(bearing_deg))
    e = distance_m * math.sin(math.radians(bearing_deg))
    return _offset_lat_lon(home.latitude_deg, home.longitude_deg, n, e)


# ---------------------------------------------------------------------------
# Telemetry watch — one per flight
# ---------------------------------------------------------------------------

class LandingWatch:
    """
    Samples position, heading, landed state, VTOL state and armed state for one
    flight. `begin_landing()` marks the moment the landing under test starts;
    everything in `result()` after that is measured from it.

    Touchdown = LandedState.ON_GROUND, or disarmed, after the vehicle has been
    airborne. (ArduCopter has been seen sitting at 0 m without reporting
    ON_GROUND — tests/command/nav_land/README.md — so the disarm counts too.)
    """

    def __init__(self, system, target: tuple[float, float] | None = None):
        self.system = system
        self.target = target
        self.lat = self.lon = None
        self.alt = 0.0
        self.heading = None
        self.max_alt = 0.0
        self.vtol_state: str | None = None
        self.vtol_states: list[str] = []      # distinct, in order seen
        self.seq: int | None = None           # MISSION_CURRENT, when watched
        self.landing_started: float | None = None
        self.start: dict | None = None        # snapshot at begin_landing()
        self.peak_alt_landing = 0.0           # highest altitude after begin_landing()
        self.back_transition: dict | None = None
        self.touchdown: dict | None = None
        self.track: list[tuple[float, float]] = []  # (alt, dist to target) after begin_landing()
        self.touched_down = asyncio.Event()
        self.in_fw = asyncio.Event()
        self.disarmed = asyncio.Event()
        self._tasks: list[asyncio.Task] = []
        self._on_seq = None

    # -- lifecycle ----------------------------------------------------------

    def start_watching(self, *, mission_current: bool = False) -> None:
        feeds = [self._positions, self._headings, self._landed, self._vtol, self._armed]
        if mission_current:
            feeds.append(self._mission_current)
        self._tasks = [asyncio.create_task(f()) for f in feeds]

    def stop(self) -> None:
        for t in self._tasks:
            t.cancel()

    def begin_landing(self) -> None:
        if self.landing_started is not None:
            return
        self.landing_started = asyncio.get_running_loop().time()
        self.peak_alt_landing = self.alt
        self.start = {"alt": self.alt, "lat": self.lat, "lon": self.lon, "heading": self.heading,
                      "vtol_state": self.vtol_state, "dist": self._dist_to_target()}

    def _dist_to_target(self) -> float | None:
        if self.target is None or self.lat is None:
            return None
        return _dist_m(self.lat, self.lon, *self.target)

    def _touch(self) -> None:
        if self.touchdown is None and self.max_alt >= AIRBORNE_THRESHOLD_M and self.lat is not None:
            self.touchdown = {"lat": self.lat, "lon": self.lon, "alt": self.alt, "heading": self.heading,
                              "vtol_state": self.vtol_state, "dist": self._dist_to_target(),
                              "t": asyncio.get_running_loop().time()}
            self.touched_down.set()

    # -- feeds --------------------------------------------------------------

    async def _positions(self) -> None:
        async for pos in self.system.telemetry.position():
            self.lat, self.lon, self.alt = pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m
            self.max_alt = max(self.max_alt, self.alt)
            if self.landing_started is not None and self.touchdown is None:
                self.peak_alt_landing = max(self.peak_alt_landing, self.alt)
                d = self._dist_to_target()
                if d is not None:
                    self.track.append((self.alt, d))

    async def _headings(self) -> None:
        async for hdg in self.system.telemetry.heading():
            self.heading = hdg.heading_deg

    async def _landed(self) -> None:
        async for state in self.system.telemetry.landed_state():
            if state == LandedState.ON_GROUND:
                self._touch()

    async def _armed(self) -> None:
        async for armed in self.system.telemetry.armed():
            if not armed and self.max_alt >= AIRBORNE_THRESHOLD_M:
                self._touch()
                self.disarmed.set()

    async def _vtol(self) -> None:
        async for vs in self.system.telemetry.vtol_state():
            was_fw = self.vtol_state in ("FW", "TRANSITION_TO_FW")
            self.vtol_state = vs.name
            if not self.vtol_states or self.vtol_states[-1] != vs.name:
                self.vtol_states.append(vs.name)
            if vs == VtolState.FW:
                self.in_fw.set()
            # EXTENDED_SYS_STATE is slow (~1 Hz), so TRANSITION_TO_MC can be missed
            # entirely: leaving fixed-wing flight for MC counts as the back-transition too.
            if (self.landing_started is not None and self.back_transition is None and was_fw
                    and vs in (VtolState.TRANSITION_TO_MC, VtolState.MC)):
                self.back_transition = {"alt": self.alt, "dist": self._dist_to_target()}

    async def _mission_current(self) -> None:
        async for msg in self.system.mavlink_direct.message("MISSION_CURRENT"):
            self.seq = int(json.loads(msg.fields_json)["seq"])
            if self._on_seq is not None:
                self._on_seq(self.seq)

    # -- result -------------------------------------------------------------

    def result(self) -> dict:
        td = self.touchdown
        half = None
        if self.track and self.start is not None and self.start["alt"] > 1.0:
            mid = min(self.track, key=lambda s: abs(s[0] - self.start["alt"] * 0.5))
            half = mid[1]
        return {
            "nacked": False, "reason": None, "max_alt": self.max_alt,
            "landing_started": self.landing_started is not None, "start": self.start,
            "landed": td is not None and self.landing_started is not None,
            "touchdown": td, "dist": None if td is None else td["dist"],
            "heading": None if td is None else td["heading"],
            "peak_alt_landing": self.peak_alt_landing, "dist_at_half_alt": half,
            "back_transition": self.back_transition, "vtol_states": list(self.vtol_states),
            "landing_s": None if td is None or self.landing_started is None else td["t"] - self.landing_started,
        }


async def _cleanup(system, watch: LandingWatch, restart) -> None:
    """
    Leave the vehicle on the ground and disarmed. After a touchdown, wait for the
    stack's own auto-disarm rather than sending RTL: _rtl_and_land() on a landed,
    still-armed vehicle would take it off again.
    """
    try:
        if watch.touchdown is not None:
            try:
                await asyncio.wait_for(watch.disarmed.wait(), DISARM_WAIT_S)
            except asyncio.TimeoutError:
                try:
                    await system.action.disarm()
                except Exception as exc:  # noqa: BLE001 — best effort; _rtl_and_land decides next
                    log.warning("disarm after touchdown failed: %r", exc)
    finally:
        watch.stop()
    await _rtl_and_land(system, restart)


# ---------------------------------------------------------------------------
# Command flights
# ---------------------------------------------------------------------------

def _land_fields(cmd_id: int, groups: dict[str, dict], base: dict) -> dict:
    """COMMAND_INT kwargs: the baseline (land here, default everything) plus each value group."""
    kw = dict(command=cmd_id, frame=6, param1=0.0, param2=0.0, param3=0.0, param4=None,
              x=base["x"], y=base["y"], z=0.0)  # frame 6, z = 0: ground level relative to home
    for g in groups.values():
        kw.update(g)
    return kw


async def _forward_flight_via_mission(system, watch: "LandingWatch", home) -> None:
    """
    Get a VTOL into forward flight for a commanded landing, using a mission:
    NAV_VTOL_TAKEOFF, two waypoints out along TRANSIT_BEARING_DEG, and a landing
    back at home (stacks refuse a mission with no landing). Returns once the
    vehicle has been in fixed-wing flight for FORWARD_FLIGHT_S, still on the
    outbound leg; the landing command under test then takes it out of the mission.

    A mission rather than NAV_TAKEOFF + a transition command because it gives the
    landing a defined starting state on every stack: a straight outbound leg on a
    known heading (TRANSIT_BEARING_DEG), not wherever a hold-mode loiter happens
    to be pointing when the command arrives.
    Setup only (Tier 2 pattern #11) — nothing here is the command under test.
    """
    hx, hy = int(home.latitude_deg * 1e7), int(home.longitude_deg * 1e7)
    t1 = target_from(home, TRANSIT_OFFSET_M, TRANSIT_BEARING_DEG)
    t2 = target_from(home, 4 * TRANSIT_OFFSET_M, TRANSIT_BEARING_DEG)
    body = [_item(NAV_VTOL_TAKEOFF, int(t1[0] * 1e7), int(t1[1] * 1e7), TAKEOFF_ALT_M),
            _item(NAV_WAYPOINT, int(t1[0] * 1e7), int(t1[1] * 1e7), TAKEOFF_ALT_M),
            _item(NAV_WAYPOINT, int(t2[0] * 1e7), int(t2[1] * 1e7), TAKEOFF_ALT_M),
            _item(NAV_VTOL_LAND, hx, hy, 0.0)]
    if getattr(system, "autopilot_type", None) == MAV_AUTOPILOT_ARDUPILOTMEGA:
        body.insert(0, _item(NAV_WAYPOINT, hx, hy, 0.0, frame=5))  # ArduPilot's home slot
    first = 1 if len(body) == 5 else 0
    items = [MissionItem(seq=i, frame=it.frame, command=it.command, current=1 if i == first else 0, autocontinue=1,
                         param1=it.param1, param2=it.param2, param3=it.param3, param4=it.param4,
                         x=it.x, y=it.y, z=it.z, mission_type=0) for i, it in enumerate(body)]
    reason = await _upload(system, items)
    if reason is not None:
        pytest.fail(f"Could not upload the get-airborne mission ({reason}) — inconclusive")
    await _wait_armable(system)
    await system.action.arm()
    try:
        await system.mission_raw.start_mission()
        await asyncio.wait_for(watch.in_fw.wait(), CLIMB_TIMEOUT_S + TRANSITION_TIMEOUT_S)
    except Exception as exc:  # noqa: BLE001 — any failure here means the precondition wasn't met
        pytest.fail(f"Could not reach forward flight before the landing ({exc!r}; "
                    f"vtol states {watch.vtol_states}, max alt {watch.max_alt:.1f} m) — inconclusive")
    await asyncio.sleep(FORWARD_FLIGHT_S)


async def fly_command(system, restart, cmd_id: int, *, groups: dict[str, dict] | None = None,
                      target: tuple[float, float] | None = None, forward: bool | None = None) -> dict:
    """
    Take off (a VTOL: fly a short mission into forward flight —
    _forward_flight_via_mission), send the landing command with every value group in `groups` (e.g. {"position": {x, y},
    "yaw": {param4}}), and watch until touchdown.

    If the combined send is NACKed, each group is sent on its own to find which
    one the stack rejects, and the landing is then flown with the accepted groups
    only. `rejected` in the result names the NACKed groups; `nacked` is True only
    when even the bare command (no groups) is refused. A group sent on its own
    that is ACCEPTED starts a landing, which the next send replaces within a
    second — the landing measured is the last accepted send.

    `forward`: start from forward flight (True), a hover (False), or decide from
    the vehicle's HEARTBEAT type (None — a VTOL starts in forward flight).

    Result: LandingWatch.result() plus `result` (final COMMAND_ACK), `rejected`,
    `sent` (groups actually flown), `forward` (whether it really was in FW).
    """
    groups = dict(groups or {})
    home = await _get_home_position(system)
    await _get_position(system, timeout_s=POSITION_READY_TIMEOUT_S)
    if forward is None:
        forward = bool(await vehicle_is_vtol(system))
    watch = LandingWatch(system, target)
    watch.start_watching()
    try:
        if forward:
            await _forward_flight_via_mission(system, watch, home)
        else:
            await _arm_and_send_takeoff(system, z=TAKEOFF_ALT_M, param4=None)
            await _wait_for_climb_complete(system, TAKEOFF_ALT_M, timeout_s=CLIMB_TIMEOUT_S)
        here = {"x": int(watch.lat * 1e7), "y": int(watch.lon * 1e7)}

        async def _send(names) -> int | None:
            ack = await probe_command_int(system, **_land_fields(cmd_id, {n: groups[n] for n in names}, here))
            return None if ack is None else int(ack["result"])

        watch.begin_landing()
        sent = list(groups)
        result = await _send(sent)
        rejected: list[str] = []
        if result is None:
            pytest.fail("No COMMAND_ACK for the landing command — cannot classify")
        if result not in _ACCEPTED and groups:
            singles = {n: await _send([n]) for n in groups}
            rejected = [n for n, r in singles.items() if r not in _ACCEPTED]
            sent = [n for n in groups if n not in rejected]
            result = await _send(sent)
            log.info("landing command: combined send NACKed; per group %s; flying with %s (result=%s)",
                     singles, sent, result)
        if result not in _ACCEPTED:
            out = watch.result()
            out.update(nacked=True, reason=f"result={result}", result=result, rejected=rejected, sent=[],
                       forward=forward)
            return out
        try:
            await asyncio.wait_for(watch.touched_down.wait(), TOUCHDOWN_TIMEOUT_S)
        except asyncio.TimeoutError:
            log.warning("landing command: no touchdown within %.0f s", TOUCHDOWN_TIMEOUT_S)
        out = watch.result()
        out.update(result=result, rejected=rejected, sent=sent, forward=forward,
                   home=(home.latitude_deg, home.longitude_deg))
        log.info("landing flight (command %d): %s", cmd_id, {k: v for k, v in out.items() if k != "touchdown"})
        return out
    finally:
        await _cleanup(system, watch, restart)
        if forward:
            from tests.mission.conftest import clear_all_mission_types
            await clear_all_mission_types(system)  # the get-airborne mission
            # Back to Hold: once landed and disarmed PX4 returns to Mission mode, and with
            # the mission now cleared it is not armable there ("no valid mission") — the
            # next test that arms without uploading a mission timed out in _wait_armable
            # (2026-10-13). Skipped after a restart (nothing to undo) or while still armed.
            try:
                if watch.disarmed.is_set():
                    await system.action.hold()
            except Exception as exc:  # noqa: BLE001 — best effort
                log.warning("could not return to Hold after the get-airborne mission: %r", exc)


# ---------------------------------------------------------------------------
# Mission flights
# ---------------------------------------------------------------------------

def _item(command: int, x: int, y: int, z: float, **kw) -> MissionItem:
    fields = dict(seq=0, frame=6, command=command, current=0, autocontinue=1,
                  param1=0.0, param2=0.0, param3=0.0, param4=NAN, x=x, y=y, z=z, mission_type=0)
    fields.update(kw)
    return MissionItem(**fields)


def build_landing_mission(home_item, home, land_cmd_id: int, land_kw: dict, *, vtol: bool) -> tuple[list, int]:
    """
    [home slot (ArduPilot)], takeoff, [transit waypoint (VTOL)], the landing item.
    Returns (items, seq of the landing item). A VTOL takes off with
    NAV_VTOL_TAKEOFF and flies to a transit waypoint so it reaches the landing
    item in forward flight; anything else takes off with NAV_TAKEOFF and goes
    straight to the landing item.
    """
    hx, hy = int(home.latitude_deg * 1e7), int(home.longitude_deg * 1e7)
    body = []
    if vtol:
        t_lat, t_lon = target_from(home, TRANSIT_OFFSET_M, TRANSIT_BEARING_DEG)
        tx, ty = int(t_lat * 1e7), int(t_lon * 1e7)
        body.append(_item(NAV_VTOL_TAKEOFF, tx, ty, TAKEOFF_ALT_M))
        body.append(_item(NAV_WAYPOINT, tx, ty, TAKEOFF_ALT_M))
    else:
        body.append(_item(NAV_TAKEOFF, hx, hy, TAKEOFF_ALT_M))
    land = dict(x=hx, y=hy, z=0.0)
    land.update(land_kw)
    body.append(_item(land_cmd_id, **land))
    items = []
    if home_item is not None:
        items.append(home_item)
    for i, it in enumerate(body):
        items.append(MissionItem(
            seq=len(items), frame=it.frame, command=it.command, current=1 if i == 0 else 0,
            autocontinue=1, param1=it.param1, param2=it.param2, param3=it.param3, param4=it.param4,
            x=it.x, y=it.y, z=it.z, mission_type=0))
    return items, len(items) - 1


async def _upload(system, items) -> str | None:
    """Upload; None if accepted, else the NACK reason."""
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await system.mission_raw.upload_mission(items)
        return None
    except MissionRawError as exc:
        return str(exc).split(":")[0].strip()


async def fly_mission(system, home_item, restart, land_cmd_id: int, *, groups: dict[str, dict] | None = None,
                      target: tuple[float, float] | None = None) -> dict:
    """
    Upload [takeoff, (transit), landing item] with every value group in `groups`
    set on the landing item, fly it, and watch until touchdown.

    If the upload is NACKed, each group is uploaded on its own (no flight) to
    find which the stack rejects, and the mission is flown with the accepted
    groups only — `rejected` names the others. `nacked` is True only when the
    bare landing item is refused, or start_mission() is (a refusal to start is
    the stack declining the mission, rule 4a).

    Result: LandingWatch.result() plus `rejected`, `sent`, `forward` (in forward
    flight when the landing item became current), `item_reached` (it became
    current at all).
    """
    from tests.mission.conftest import clear_all_mission_types

    groups = dict(groups or {})
    home = await _get_home_position(system)
    await _get_heading(system, timeout_s=POSITION_READY_TIMEOUT_S)
    vtol = bool(await vehicle_is_vtol(system))

    def _mission(names):
        kw: dict = {}
        for n in names:
            kw.update(groups[n])
        return build_landing_mission(home_item, home, land_cmd_id, kw, vtol=vtol)

    sent, rejected = list(groups), []
    items, land_seq = _mission(sent)
    reason = await _upload(system, items)
    if reason is not None and groups:
        singles = {}
        for n in groups:
            await clear_all_mission_types(system)
            singles[n] = await _upload(system, _mission([n])[0])
        rejected = [n for n, r in singles.items() if r is not None]
        sent = [n for n in groups if n not in rejected]
        await clear_all_mission_types(system)
        items, land_seq = _mission(sent)
        reason = await _upload(system, items)
        log.info("landing mission: combined upload NACKed; per group %s; flying with %s (%s)",
                 singles, sent, reason or "accepted")
    if reason is not None:
        await clear_all_mission_types(system)
        return {"nacked": True, "reason": reason, "rejected": rejected, "sent": []}

    watch = LandingWatch(system, target)
    state = {"reached": False}

    def _on_seq(seq: int) -> None:
        if seq == land_seq and watch.max_alt >= AIRBORNE_THRESHOLD_M and not state["reached"]:
            state["reached"] = True
            watch.begin_landing()

    watch._on_seq = _on_seq
    watch.start_watching(mission_current=True)
    try:
        await request_message_rate(system, 42, 10.0)  # MISSION_CURRENT
        await _wait_armable(system)
        await system.action.arm()
        try:
            await system.mission_raw.start_mission()
        except MissionRawError as exc:
            return {"nacked": True, "reason": f"start_mission refused ({exc.result.name})",
                    "rejected": rejected, "sent": []}
        try:
            await asyncio.wait_for(watch.touched_down.wait(), TOUCHDOWN_TIMEOUT_S)
        except asyncio.TimeoutError:
            log.warning("landing mission: no touchdown within %.0f s (MISSION_CURRENT seq %s, landing item %d)",
                        TOUCHDOWN_TIMEOUT_S, watch.seq, land_seq)
        out = watch.result()
        out.update(rejected=rejected, sent=sent, item_reached=state["reached"],
                   forward=bool(watch.start and watch.start["vtol_state"] == "FW"),
                   home=(home.latitude_deg, home.longitude_deg))
        log.info("landing flight (mission item %d): %s", land_cmd_id,
                 {k: v for k, v in out.items() if k != "touchdown"})
        return out
    finally:
        await _cleanup(system, watch, restart)
        await clear_all_mission_types(system)


# ---------------------------------------------------------------------------
# The two shared flights: position + yaw (+ any extra per-flight groups)
# ---------------------------------------------------------------------------

async def fly_pair(fly, system, *, extra: tuple[dict[str, dict], dict[str, dict]] = ({}, {})) -> list[dict]:
    """
    Two flights — target 'N' with YAW_VALUES_DEG[0], target 'E' with [1] — through
    `fly(groups=..., target=...)`, each target measured from where that flight
    starts. `extra` adds further value groups to each flight (e.g. NAV_VTOL_LAND's
    approach altitude). Each returned flight has
    `label`, `target`, `yaw` added.
    """
    distance = FORWARD_TARGET_M if await vehicle_is_vtol(system) else HOVER_TARGET_M
    flights = []
    for (label, bearing), yaw, more in zip(TARGET_BEARINGS, YAW_VALUES_DEG, extra):
        # Measured from where the vehicle sits now, not from one fixed home: the second
        # flight starts where the first one landed, and the targets must stay due N /
        # due E of the start so the approach bearing (0° / 90°) can never coincide with
        # a commanded yaw (seen 2026-10-13: an 'E of the original home' target lay at
        # 135° from the new start — exactly the other flight's yaw).
        here = await _get_position(system, timeout_s=POSITION_READY_TIMEOUT_S)
        t_lat, t_lon = target_from(here, distance, bearing)
        groups = {"position": {"x": int(t_lat * 1e7), "y": int(t_lon * 1e7)}, "yaw": {"param4": yaw}}
        groups.update(more)
        r = await fly(groups=groups, target=(t_lat, t_lon))
        r.update(label=f"{distance:.0f} m {label}", target=(t_lat, t_lon), yaw=yaw)
        flights.append(r)
    return flights


# ---------------------------------------------------------------------------
# Verdicts (rule 4a)
# ---------------------------------------------------------------------------

def combine(values: list[dict]) -> dict:
    """One verdict from several values of the same param (rule 4c)."""
    return {"ok": all(v["ok"] for v in values), "nacked": all(v["nacked"] for v in values),
            "detail": "; ".join(v["detail"] for v in values)}


def verdict_text(v: dict) -> str:
    if v["nacked"]:
        return "REJECTED (NACKed)"
    return "SUPPORTED" if v["ok"] else "NOT SUPPORTED — COMPATIBILITY ERROR"


def record_param(request, label: str, keys: tuple[str, ...], v: dict, criterion: str) -> None:
    record_tier2_detail(request, f"{criterion}: {v['detail']}")
    record_tier2_param_verdict(request, label, verdict_text(v))
    for key in keys:
        record_compat_json(request, key, supported=v["ok"],
                           nacks_on_non_sentinel_value=None if v["ok"] else v["nacked"])
    if not (v["ok"] or v["nacked"]):
        report.compat_fail(f"{label} accepted but not honoured: {v['detail']} (root CLAUDE.md rule 4)")


def require_landed(r: dict, what: str) -> None:
    """A verdict needs a landing that happened (rule 10): otherwise the run proves nothing about `what`."""
    if r["max_alt"] < AIRBORNE_THRESHOLD_M:
        pytest.fail(f"Vehicle never got airborne — {what} inconclusive ({_brief(r)})")
    if not r.get("landing_started"):
        pytest.fail(f"The landing never started (the landing item was never reached) — {what} inconclusive ({_brief(r)})")
    if not r["landed"]:
        pytest.fail(f"No touchdown within {TOUCHDOWN_TIMEOUT_S:.0f} s — {what} inconclusive; likely insufficient "
                    f"time for this stack rather than a behavioural finding ({_brief(r)})")


def _brief(r: dict) -> dict:
    return {k: r.get(k) for k in ("max_alt", "landing_started", "landed", "dist", "heading", "vtol_states", "start")}


def group_value(r: dict, group: str, tag: str) -> dict | None:
    """The REJECTED verdict value for `group` on flight `r`, or None if it was flown."""
    if r["nacked"]:
        return {"nacked": True, "ok": False, "detail": f"{tag}: REJECTED — landing refused ({r['reason']})"}
    if group in r["rejected"]:
        return {"nacked": True, "ok": False, "detail": f"{tag}: REJECTED (NACKed on its own)"}
    return None


def position_verdict(flights: list[dict]) -> dict:
    values = []
    for r in flights:
        v = group_value(r, "position", r["label"])
        if v is None:
            require_landed(r, "lat/lon")
            v = {"nacked": False, "ok": r["dist"] <= TOUCHDOWN_TOLERANCE_M,
                 "detail": f"{r['label']}: touched down {r['dist']:.1f} m from the target "
                           f"(landing started {r['start']['dist']:.0f} m from it)"}
        values.append(v)
    return combine(values)


def yaw_verdict(flights: list[dict]) -> dict:
    values = []
    for r in flights:
        tag = f"{r['yaw']:.0f}°"
        v = group_value(r, "yaw", tag)
        if v is None:
            require_landed(r, "yaw")
            diff = angle_diff(r["heading"], r["yaw"])
            v = {"nacked": False, "ok": diff <= YAW_TOLERANCE_DEG,
                 "detail": f"{tag}: heading at touchdown {r['heading']:.1f}° (diff {diff:.1f}°)"}
        values.append(v)
    return combine(values)


def approach_altitude_verdict(flights: list[dict]) -> dict:
    """Approach altitude judged as the highest altitude reached once the landing started."""
    values = []
    for r in flights:
        alt = r["approach_alt"]
        tag = f"{alt:.0f} m"
        v = group_value(r, "approach", tag)
        if v is None:
            require_landed(r, "approach altitude")
            tol = max(ALT_TOLERANCE_MIN_M, ALT_TOLERANCE_FRAC * alt)
            v = {"nacked": False, "ok": abs(r["peak_alt_landing"] - alt) <= tol,
                 "detail": f"{tag}: landing started at {r['start']['alt']:.1f} m, peaked at "
                           f"{r['peak_alt_landing']:.1f} m (±{tol:.0f} m)"}
        values.append(v)
    return combine(values)


def describe_landing(r: dict) -> str:
    """One line on how a landing went — for characterisation details."""
    if r["nacked"]:
        return f"refused ({r['reason']})"
    if not r.get("landing_started"):
        return f"landing never started (max alt {r['max_alt']:.1f} m)"
    s = r["start"]
    parts = [f"started at {s['alt']:.1f} m" + (f", {s['dist']:.0f} m from the target" if s["dist"] is not None else "")
             + (f", vtol_state {s['vtol_state']}" if s["vtol_state"] not in (None, "UNDEFINED") else "")]
    if r["back_transition"] is not None:
        bt = r["back_transition"]
        parts.append(f"back-transition at {bt['alt']:.1f} m"
                     + (f", {bt['dist']:.0f} m from the target" if bt["dist"] is not None else ""))
    if r["dist_at_half_alt"] is not None:
        parts.append(f"{r['dist_at_half_alt']:.0f} m from the target at half altitude")
    if r["landed"]:
        td = r["touchdown"]
        parts.append(f"touched down after {r['landing_s']:.0f} s"
                     + (f", {td['dist']:.1f} m from the target" if td["dist"] is not None else "")
                     + f", heading {td['heading']:.0f}°"
                     + (f", vtol_state {td['vtol_state']}" if td["vtol_state"] not in (None, "UNDEFINED") else ""))
    else:
        parts.append(f"no touchdown within {TOUCHDOWN_TIMEOUT_S:.0f} s")
    return "; ".join(parts)


# ---------------------------------------------------------------------------
# No-flight probes and facts shared by the four modules
# ---------------------------------------------------------------------------

async def mission_probe_nacked(system, home_item, land_cmd_id: int, **land_kw) -> bool:
    """Upload a landing mission with `land_kw` on the landing item, without flying; True if NACKed."""
    from tests.mission.conftest import clear_all_mission_types

    home = await _get_home_position(system)
    items, _ = build_landing_mission(home_item, home, land_cmd_id, land_kw, vtol=bool(await vehicle_is_vtol(system)))
    try:
        return await _upload(system, items) is not None
    finally:
        await clear_all_mission_types(system)


GROUND_ALT_UNTESTABLE = "Flat simulated terrain — touchdown is at the ground whatever value is sent"
YAW_SENTINEL_NOT_TESTABLE = "NOT TESTABLE (\"current yaw heading mode\" has no fixed target)"


def lands_verdict(flights: list[dict]) -> tuple[bool | None, str]:
    """
    Command level, from the shared flights: (None, why) when every landing was
    refused (nothing to judge), else (every accepted landing touched down, detail).
    """
    flown = [r for r in flights if not r["nacked"]]
    if not flown:
        return None, "; ".join(f"{r['label']}: refused ({r['reason']})" for r in flights)
    for r in flown:
        if r["max_alt"] < AIRBORNE_THRESHOLD_M or not r.get("landing_started"):
            pytest.fail(f"The landing under test never started — inconclusive ({_brief(r)})")
    return all(r["landed"] for r in flown), "; ".join(f"{r['label']}: {describe_landing(r)}" for r in flown)


START_REFUSED_NOTE = "Accepted at upload, but a mission containing it is refused at start"


def refused_at_start(flights: list[dict]) -> bool:
    """Every flight's mission uploaded fine and was then refused by start_mission()."""
    return all(r["nacked"] and str(r["reason"]).startswith("start_mission refused") for r in flights)
