"""
Camera commands with a target camera ID — Tier 2: does a mission item's camera ID
decide where the autopilot re-emits the command?

The main question: when a mission item sets the target camera ID to a MAVLink
camera component (7-255), is the command re-emitted with target_component = that
id? common.xml defines this ("resend it as a command if it is intended for a
MAVLink camera (param1 = 7 - 255), setting the command's target_component"), so
it's a compatibility check (`compat`), with two ids (101 and 200, rule 4c —
neither is the 100 fallback). Secondary (`info`, PX4-documented, not spec):
with the id unset (0 or NaN) camera commands go to MAV_COMP_ID_CAMERA (100) and
trigger commands to MAV_COMP_ID_ALL (0); and the IMAGE_STOP_CAPTURE PX4 sends
when a mission is paused goes to the camera that was started.

Method: the autopilot re-emits mission camera commands as COMMAND_LONG on every
MAVLink link (PX4: the COMMAND_LONG stream, `streams/COMMAND_LONG.hpp`), so the
GCS sees each one with its real target_component. Each case is one flight of one
mission — takeoff, every camera item with that case's id, a waypoint — and each
command's first matching COMMAND_LONG is read from the capture (items carry
distinctive values so they aren't confused with commands the autopilot sends on
its own). Items are upload-probed one by one first: one the stack NACKs is left
out of the flight and reported REJECTED (rule 4a), rather than failing the whole
mission upload. Flights are cached per case (rule 7) and shared by every test.

Not tested (methodology notes):
- **Camera discovery.** "Unset" falls back to component 100 on PX4. In principle
  an autopilot should instead look for a connected camera (a component whose
  HEARTBEAT type is MAV_TYPE_CAMERA) and use that; 100 is a common convention,
  not the spec. Validating discovery needs a fake camera component on the link —
  e.g. a second MAVSDK endpoint configured as a camera (MAVSDK's camera server)
  at a non-100 component id — and a check that unset-id commands go to it.
- **ids 1-6** ("cameras attached to the autopilot"): the spec says execute
  locally, not resend; PX4 resends them to component 1-6. Not covered here.

Each test reports under its own command (`_tier2_key_for` below), so this one
module feeds eleven command reports.

Running (PX4 SIH multicopter is enough — the vehicle only has to fly a mission)::

    pytest tests/mission/camera_target_id/test_flight.py --drone-address=udp://:14540 \\
        --px4-sitl=~/github/px4/PX4-Autopilot-camera --px4-model=sihsim_quadx \\
        --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO
"""

import asyncio
import json
import logging
import math

import pytest
from mavsdk.plugins.mission_raw import MissionItem, MissionRawError

from tests import report
from ..conftest import clear_all_mission_types
from .camera_commands import BY_NAME, CAMERA_COMMANDS, MAV_FRAME_MISSION, item_params, matches, xml_params
from tests.flight_helpers import (
    _get_home_position,
    _north_of,
    _rtl_and_land,
    _tier2_auto_record,  # noqa: F401 — autouse: records every test's outcome
    _wait_armable,
    record_compat_json,
    record_tier2_detail,
    record_tier2_param_verdict,
    require_real_stack,  # noqa: F401 — real-stack skip gate
)

log = logging.getLogger(__name__)

pytestmark = pytest.mark.timeout(900)

TRANSFER_TIMEOUT_S = 30.0
TAKEOFF_ALT_M = 10.0
WATCH_TIMEOUT_S = 120.0  # takeoff + every DO item (with spacing) + reaching the waypoint
# A NAV_DELAY between camera items: PX4's COMMAND_LONG stream reads at most
# ORB_QUEUE_LENGTH (8) vehicle_commands per cycle, so with 12 camera items back to
# back the first 4 re-emitted commands are lost (2026-10-01 — measured: consecutive
# delivers only the last 8, spaced delivers all 12). Spacing keeps the routing test
# about routing; test_camera_info_consecutive_items_all_reemitted measures the loss.
ITEM_SPACING_S = 1.0
SETTLE_S = 3.0           # keep capturing briefly after the waypoint is reached
NAN = float("nan")

SET_IDS = (101.0, 200.0)     # MAVLink camera component ids (7-255), neither is 100
UNSET_IDS = (0.0, NAN)       # "not set"
PAUSE_IDS = (101.0, 200.0, 0.0)

_CMD_IDS = {c.spec_name: c.cmd_id for c in CAMERA_COMMANDS}


def _tier2_key_for(node):
    """Report each test under its own command (tests/flight_helpers.py's hook)."""
    callspec = getattr(node, "callspec", None)
    if callspec is not None and "cmd_name" in callspec.params:
        name = BY_NAME[callspec.params["cmd_name"]].spec_name
    elif node.originalname == "test_camera_info_pause_stops_started_camera":
        name = "IMAGE_STOP_CAPTURE"
    else:  # the consecutive-items test: reported with the first command it loses
        name = "IMAGE_START_CAPTURE"
    return "mission", name, _CMD_IDS[name]


# Every command reported here — declare them so each report lists its params.
for _c in CAMERA_COMMANDS:
    report.declare_command("mission", _c.spec_name, _c.cmd_id)
    report.declare_params("mission", _c.spec_name, [f"{p.slot}_{p.label}" for p in xml_params(_c.spec_name)])


# ---------------------------------------------------------------------------
# Missions
# ---------------------------------------------------------------------------

def _item(seq: int, command: int, frame: int, current: int = 0, **kw) -> MissionItem:
    return MissionItem(seq=seq, frame=frame, command=command, current=current, autocontinue=1,
                       mission_type=0, **kw)


def _delay_item(seq: int, seconds: float = ITEM_SPACING_S) -> MissionItem:
    """NAV_DELAY — keeps a camera item clear of the burst of commands PX4 emits around takeoff."""
    return _item(seq, 93, MAV_FRAME_MISSION, param1=seconds, param2=-1.0, param3=-1.0, param4=-1.0,
                 x=0x7FFF_FFFF, y=0x7FFF_FFFF, z=NAN)


def _camera_item(seq: int, cmd, camera_id: float) -> MissionItem:
    return _item(seq, cmd.cmd_id, MAV_FRAME_MISSION, **item_params(cmd, camera_id))


def _build(home_item, home, camera_items: list, waypoint_north_m: float = 30.0,
           spacing_s: float = 0.0) -> tuple[list, int]:
    """[home slot (ArduPilot)], takeoff, the camera items (a NAV_DELAY of spacing_s after each, if > 0),
    a waypoint. Returns (items, waypoint seq)."""
    items = []
    if home_item is not None:
        items.append(MissionItem(seq=0, frame=home_item.frame, command=home_item.command, current=0,
                                 autocontinue=1, param1=home_item.param1, param2=home_item.param2,
                                 param3=home_item.param3, param4=home_item.param4, x=home_item.x,
                                 y=home_item.y, z=home_item.z, mission_type=0))
    lat, lon = int(home.latitude_deg * 1e7), int(home.longitude_deg * 1e7)
    common = dict(param1=0.0, param2=0.0, param3=0.0, param4=NAN)
    items.append(_item(len(items), 22, 6, current=1, x=lat, y=lon, z=TAKEOFF_ALT_M, **common))  # NAV_TAKEOFF
    for build in camera_items:
        items.append(build(len(items)))
        if spacing_s > 0:
            items.append(_delay_item(len(items), spacing_s))
    wp_seq = len(items)
    items.append(_item(wp_seq, 16, 6, x=_north_of(home.latitude_deg, waypoint_north_m), y=lon,
                       z=TAKEOFF_ALT_M, **common))  # NAV_WAYPOINT
    return items, wp_seq


async def _upload(system, items) -> str | None:
    """Upload; None if accepted, else the NACK reason."""
    try:
        async with asyncio.timeout(TRANSFER_TIMEOUT_S):
            await system.mission_raw.upload_mission(items)
        return None
    except MissionRawError as exc:
        return str(exc).split(":")[0].strip()


# ---------------------------------------------------------------------------
# One flight per camera id: every accepted camera item, capture what's re-emitted
# ---------------------------------------------------------------------------

_flights: dict[str, dict] = {}  # case key -> {"nacked": {name: reason}, "targets": {name: target or None}}


def _key(camera_id: float) -> str:
    return "nan" if math.isnan(camera_id) else f"{camera_id:.0f}"


async def _capture_during(system, wp_seq: int, start) -> list[dict]:
    """Run `start()` (arm + start mission) while capturing every COMMAND_LONG until the waypoint is reached."""
    captured: list[dict] = []
    reached = asyncio.Event()

    async def _commands():
        async for msg in system.mavlink_direct.message("COMMAND_LONG"):
            captured.append(json.loads(msg.fields_json))

    async def _progress():
        async for p in system.mission_raw.mission_progress():
            if p.current >= wp_seq:
                reached.set()

    tasks = [asyncio.create_task(_commands()), asyncio.create_task(_progress())]
    try:
        await start()
        try:
            await asyncio.wait_for(reached.wait(), WATCH_TIMEOUT_S)
        except asyncio.TimeoutError:
            log.warning("Camera flight: waypoint not reached within %.0fs — judging what was captured", WATCH_TIMEOUT_S)
        await asyncio.sleep(SETTLE_S)
    finally:
        for t in tasks:
            t.cancel()
    return captured


async def _fly_case(gcs_system, home_item, restart_flight_stack, camera_id: float,
                   spacing_s: float = ITEM_SPACING_S) -> dict:
    key = _key(camera_id) + ("" if spacing_s else "-consecutive")
    if key in _flights:
        return _flights[key]
    home = await _get_home_position(gcs_system)

    # Probe each item on its own, so one NACKed item doesn't sink the whole mission.
    nacked = {}
    for cmd in CAMERA_COMMANDS:
        items, _ = _build(home_item, home, [lambda s, c=cmd: _camera_item(s, c, camera_id)])
        reason = await _upload(gcs_system, items)
        await clear_all_mission_types(gcs_system)
        if reason is not None:
            nacked[cmd.name] = reason
    flown = [c for c in CAMERA_COMMANDS if c.name not in nacked]

    targets: dict[str, int | None] = {}
    if flown:
        items, wp_seq = _build(home_item, home, [lambda s, c=c: _camera_item(s, c, camera_id) for c in flown],
                               spacing_s=spacing_s)
        reason = await _upload(gcs_system, items)
        if reason is not None:
            pytest.fail(f"Mission of individually-accepted camera items NACKed ({reason}) — can't fly case id={key}")

        async def _start():
            await _wait_armable(gcs_system)
            await gcs_system.action.arm()
            await gcs_system.mission_raw.start_mission()
        try:
            captured = await _capture_during(gcs_system, wp_seq, _start)
        finally:
            await _rtl_and_land(gcs_system, restart_flight_stack)
            await clear_all_mission_types(gcs_system)
        for cmd in flown:
            hit = next((m for m in captured if matches(cmd, m)), None)
            targets[cmd.name] = None if hit is None else int(hit["target_component"])
        log.info("Camera flight id=%s: targets %s; NACKed %s", key, targets, nacked)
    _flights[key] = {"nacked": nacked, "targets": targets}
    return _flights[key]


def _value(flight: dict, cmd, camera_id: float, expected: int) -> dict:
    """One id value's result for one command: NACKed / honoured / not, with a detail string."""
    label = f"id {_key(camera_id)}"
    if cmd.name in flight["nacked"]:
        return {"nacked": True, "ok": False, "detail": f"{label}: REJECTED ({flight['nacked'][cmd.name]})"}
    got = flight["targets"].get(cmd.name)
    if got is None:
        return {"nacked": False, "ok": False, "detail": f"{label}: not re-emitted at all"}
    return {"nacked": False, "ok": got == expected, "detail": f"{label}: target_component {got} (expected {expected})"}


def _id_param_key(cmd) -> str:
    p = xml_params(cmd.spec_name)[cmd.id_slot - 1]
    return f"{p.slot}_{p.label}"


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

_NAMES = [c.name for c in CAMERA_COMMANDS]


@pytest.mark.parametrize("cmd_name", _NAMES)
async def test_camera_compat_target_id_respected(gcs_system, home_item_for_mission, restart_flight_stack, request, cmd_name):
    """A set target camera ID (101, 200) is the re-emitted command's target_component — or the item is NACKed."""
    cmd = BY_NAME[cmd_name]
    values = [_value(await _fly_case(gcs_system, home_item_for_mission, restart_flight_stack, i), cmd, i, int(i))
              for i in SET_IDS]
    ok = all(v["ok"] for v in values)
    nacked = all(v["nacked"] for v in values)
    detail = "; ".join(v["detail"] for v in values)
    record_tier2_detail(request, f"PASS if each set id is NACKed or used as the target component: {detail}")
    verdict = "REJECTED (NACKed)" if nacked else "SUPPORTED" if ok else "NOT SUPPORTED — COMPATIBILITY ERROR"
    label = f"param{cmd.id_slot} (target camera ID)" + (" — single-shot capture" if cmd.xml_name else "")
    record_tier2_param_verdict(request, label, verdict)
    if not cmd.xml_name:  # the single-shot case reports a verdict but leaves the param's JSON to the main case
        record_compat_json(request, _id_param_key(cmd), supported=ok, nacks_on_non_sentinel_value=None if ok else nacked)
    if not (ok or nacked):
        report.compat_fail(f"Target camera ID accepted but not used as the target component: {detail}")


@pytest.mark.parametrize("cmd_name", _NAMES)
async def test_camera_info_unset_id_fallback(gcs_system, home_item_for_mission, restart_flight_stack, request, cmd_name):
    """(Information) With the id unset (0, NaN) the command goes to PX4's documented fallback — 100, or 0 for the trigger commands."""
    cmd = BY_NAME[cmd_name]
    values = [_value(await _fly_case(gcs_system, home_item_for_mission, restart_flight_stack, i), cmd, i, cmd.fallback)
              for i in UNSET_IDS]
    detail = "; ".join(v["detail"] for v in values)
    record_tier2_detail(request, f"(Information) PASS if each unset id is NACKed or sent to {cmd.fallback}: {detail}")
    assert all(v["ok"] or v["nacked"] for v in values), (
        f"Unset camera ID not sent to the documented fallback component {cmd.fallback}: {detail}")


async def _pause_case(gcs_system, home_item, restart_flight_stack, camera_id: float) -> dict:
    """Start multi-image capture for camera_id, pause the mission, return the IMAGE_STOP_CAPTURE target."""
    home = await _get_home_position(gcs_system)
    start = BY_NAME["IMAGE_START_CAPTURE"]
    # The delay first: right after takeoff PX4 emits commands of its own, and a camera item
    # there can lose its re-emitted command to the 8-deep queue (seen on unpatched main).
    items, wp_seq = _build(home_item, home, [_delay_item, lambda s: _camera_item(s, start, camera_id)],
                           waypoint_north_m=400.0)
    reason = await _upload(gcs_system, items)
    if reason is not None:
        return {"nacked": True, "ok": False, "detail": f"id {_key(camera_id)}: REJECTED ({reason})"}
    captured: list[dict] = []
    started = asyncio.Event()

    async def _commands():
        async for msg in gcs_system.mavlink_direct.message("COMMAND_LONG"):
            fields = json.loads(msg.fields_json)
            captured.append(fields)
            if matches(start, fields):
                started.set()

    task = asyncio.create_task(_commands())
    try:
        await _wait_armable(gcs_system)
        await gcs_system.action.arm()
        await gcs_system.mission_raw.start_mission()
        try:
            await asyncio.wait_for(started.wait(), WATCH_TIMEOUT_S)
        except asyncio.TimeoutError:
            return {"nacked": False, "ok": False,
                    "detail": f"id {_key(camera_id)}: IMAGE_START_CAPTURE never re-emitted — pause not tested"}
        await asyncio.sleep(2.0)
        captured.clear()
        await gcs_system.mission_raw.pause_mission()
        await asyncio.sleep(5.0)
    finally:
        task.cancel()
        await _rtl_and_land(gcs_system, restart_flight_stack)
        await clear_all_mission_types(gcs_system)
    expected = int(camera_id) if camera_id else start.fallback
    stop = next((int(m["target_component"]) for m in captured if int(m.get("command", -1)) == 2001), None)
    if stop is None:
        return {"nacked": False, "ok": False, "detail": f"id {_key(camera_id)}: no IMAGE_STOP_CAPTURE after the pause"}
    return {"nacked": False, "ok": stop == expected,
            "detail": f"id {_key(camera_id)}: stop sent to {stop} (expected {expected})"}


async def test_camera_info_pause_stops_started_camera(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """(Information) Pausing a mission that is capturing images sends IMAGE_STOP_CAPTURE to the camera that was started."""
    values = [await _pause_case(gcs_system, home_item_for_mission, restart_flight_stack, i) for i in PAUSE_IDS]
    detail = "; ".join(v["detail"] for v in values)
    record_tier2_detail(request, f"(Information) PASS if the automatic stop goes to the started camera (or 100 if unset): {detail}")
    assert all(v["ok"] or v["nacked"] for v in values), f"Automatic IMAGE_STOP_CAPTURE didn't target the started camera: {detail}"


async def test_camera_info_consecutive_items_all_reemitted(gcs_system, home_item_for_mission, restart_flight_stack, request):
    """(Information) Twelve camera items back to back (no spacing) are all re-emitted, none lost."""
    spaced = await _fly_case(gcs_system, home_item_for_mission, restart_flight_stack, SET_IDS[0])
    burst = await _fly_case(gcs_system, home_item_for_mission, restart_flight_stack, SET_IDS[0], spacing_s=0.0)
    sent = [n for n in _NAMES if n not in burst["nacked"]]
    lost = [n for n in sent if burst["targets"].get(n) is None and spaced["targets"].get(n) is not None]
    detail = (f"{len(sent) - len(lost)} of {len(sent)} re-emitted back to back"
              + (f"; lost: {', '.join(lost)} (all re-emitted when spaced {ITEM_SPACING_S:.0f} s apart)" if lost else ""))
    record_tier2_detail(request, f"(Information) PASS if every consecutive camera item is re-emitted: {detail}")
    assert not lost, f"Back-to-back camera items lost their re-emitted commands: {detail}"
