"""
One shared flight for the message tests that need movement — every
`test_flight*.py` in this tree (see this directory's CLAUDE.md).

Many field checks can't be made on a vehicle sitting at its origin: a position
or velocity axis reads ~0 whichever way round it is, a gyro reads ~0, a GPS
course is undefined, a takeoff time is never set. So the vehicle flies one
mission, every message is recorded throughout, and each message's flight tests
read what they need from that one recording (root CLAUDE.md rule 7: one flight,
many reads). The flight runs when the first flight test asks for it.

The mission, relative to home, with s = the leg length:

    takeoff to 30 m → (s N, 0, 30 m) → (s N, s E, 50 m) → (0, s E, 30 m) → land at (0, s W)

(a fixed-wing flies the first waypoint at 50 m — see build_mission())

so the vehicle moves north, east, south and west (both signs of every
horizontal position and velocity axis), climbs and descends (vz both ways, and
two cruise altitudes for the pressure check — rule 4c), turns (body rates), and
faces several headings (magnetometer, quaternion signs). It lands away from
home and the origin, and is then armed and disarmed once on the ground there,
so a home position reset at arming sits away from the local origin
(HOME_POSITION x/y/z).

How to fly it is setup keyed on the vehicle's own HEARTBEAT type (Tier 2
pattern #11), never on `--vehicle-type` (rule 5):
- multirotor: NAV_TAKEOFF at home, s = 100 m, NAV_LAND;
- VTOL (a VTOL HEARTBEAT type, or a fixed-wing type that reports a VTOL state
  at rest — ArduPilot QuadPlane sends type FIXED_WING): NAV_VTOL_TAKEOFF,
  s = 400 m flown in forward flight, NAV_VTOL_LAND;
- ground vehicle (rover, boat): waypoints only, s = 40 m, ending at the "land" point;
- anything else (fixed-wing): NAV_TAKEOFF at home with a 15° minimum pitch (as
  tests/mission/nav_takeoff), s = 400 m, NAV_LAND (stacks refuse a
  fixed-wing mission without a landing — tests/mission/CLAUDE.md).

Recorded at 10 Hz (ATTITUDE 20 Hz; GPS, VFR_HUD, EXTENDED_SYS_STATE 5 Hz): every message
in `RATES_HZ`. HOME_POSITION, GPS_GLOBAL_ORIGIN and FLIGHT_INFORMATION are
also requested every 2 s (ArduPilot sends the first two only on request or
change, and nothing streams FLIGHT_INFORMATION by default).

Afterwards the flight stack is always restarted (restart_flight_stack), so
the ground tests that follow in the session start from a vehicle at rest at
its origin, as they assume.
"""

from __future__ import annotations

import asyncio
import bisect
import json
import logging
import math
import os
import pathlib
import time
from dataclasses import dataclass, field

import pytest
from mavsdk.plugins.mission_raw import MissionItem, MissionRawError

from tests.flight_helpers import (
    MAV_AUTOPILOT_ARDUPILOTMEGA,
    _VTOL_MAV_TYPES,
    _get_heading,
    _get_home_position,
    _offset_lat_lon,
    _rtl_and_land,
    _wait_armable,
)
from tests.messages_telemetry.conftest import (
    MSG_IDS,
    MessageSampler,
    Sample,
    record_field_fact,
    request_message,
    set_message_interval,
    vehicle_heartbeat,
)
from tests.messages_telemetry.emission import autopilot_only

log = logging.getLogger(__name__)

RATES_HZ = {
    "ATTITUDE": 20, "ATTITUDE_QUATERNION": 10, "LOCAL_POSITION_NED": 10, "GLOBAL_POSITION_INT": 10,
    "SCALED_IMU": 10, "SCALED_IMU2": 10, "SCALED_IMU3": 10,
    "SCALED_PRESSURE": 10, "SCALED_PRESSURE2": 10, "SCALED_PRESSURE3": 10,
    "GPS_RAW_INT": 5, "GPS2_RAW": 5, "VFR_HUD": 5, "EXTENDED_SYS_STATE": 5, "MISSION_CURRENT": 2,
}
POLLED = ("HOME_POSITION", "GPS_GLOBAL_ORIGIN", "FLIGHT_INFORMATION")
POLL_S = 2.0
NAMES = (*RATES_HZ, *POLLED, "HEARTBEAT")

TAKEOFF_ALT_M = 30.0
HIGH_ALT_M = 50.0
LEG_M = {"multirotor": 100.0, "vtol": 400.0, "fixed_wing": 400.0, "ground": 40.0}
FLIGHT_TIMEOUT_S = float(os.environ.get("MESSAGE_FLIGHT_TIMEOUT_S", "600"))
TEST_TIMEOUT_S = FLIGHT_TIMEOUT_S + 300  # pytest-timeout for a flight test: the first one flies (and restarts SITL)
# Every flight module: `pytestmark = F.PYTESTMARK`. The `flight` marker is what puts a test's row in
# the report's Tier 2 table (conftest._message_auto_record), and selects them: -m flight / -m "not flight".
PYTESTMARK = [pytest.mark.timeout(TEST_TIMEOUT_S), pytest.mark.flight]
# Replay a saved recording (logs/message_flight_*.json) instead of flying — for re-tuning checks.
REPLAY = os.environ.get("MESSAGE_FLIGHT_REPLAY")
ARMABLE_TIMEOUT_S = 120.0
AIRBORNE_M = 2.0        # relative altitude that counts as off the ground
ON_GROUND_M = 1.0       # ... and back on it
TOUCHDOWN_AFTER_M = 10.0  # a touchdown only counts after the vehicle has been this high
LANDED_HOLD_S = 10.0    # on the ground and still this long after the landing item: disarm if the stack hasn't
ARRIVED_M = 3.0         # ground vehicle: this close to the last waypoint and stopped = done
REARM_S = 6.0           # armed this long on the ground after landing (home reset)

MULTIROTOR_TYPES = {2, 3, 4, 13, 14, 15, 29, 35}  # quad, coax, heli, hexa, octo, tri, dodeca, decarotor
GROUND_TYPES = {10, 11, 12}                        # rover, boat, submarine
MAV_MODE_FLAG_SAFETY_ARMED = 128

NAV_WAYPOINT, NAV_LAND, NAV_TAKEOFF, NAV_VTOL_TAKEOFF, NAV_VTOL_LAND = 16, 21, 22, 84, 85
GRAVITY = 9.80665


# ---------------------------------------------------------------------------
# The recording
# ---------------------------------------------------------------------------

class BootIndex:
    """Samples of one message ordered by time_boot_ms, for matching messages on the sender's own clock."""

    def __init__(self, samples: list[Sample]):
        self.samples = sorted((s for s in samples if "time_boot_ms" in s.fields), key=lambda s: s.fields["time_boot_ms"])
        self.keys = [s.fields["time_boot_ms"] for s in self.samples]

    def at(self, boot_ms: float, max_dt_ms: float = 60.0) -> Sample | None:
        i = bisect.bisect_left(self.keys, boot_ms)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(self.keys) and abs(self.keys[j] - boot_ms) <= max_dt_ms:
                if best is None or abs(self.keys[j] - boot_ms) < abs(best.fields["time_boot_ms"] - boot_ms):
                    best = self.samples[j]
        return best

    def interp(self, boot_ms: float, names: tuple[str, ...], angles: tuple[str, ...] = (),
               max_gap_ms: float = 250.0) -> dict | None:
        """
        `names` linearly interpolated to `boot_ms` between the two samples around it
        (`angles`, in radians, interpolated the short way round). None if there's no
        sample on both sides within `max_gap_ms` of each other. Streams at the same
        rate are often sent with a fixed phase offset, so a nearest-sample match can
        be up to half a period off every time.
        """
        i = bisect.bisect_left(self.keys, boot_ms)
        if i < len(self.keys) and self.keys[i] == boot_ms:
            return {n: self.samples[i].fields[n] for n in names}
        if i == 0 or i >= len(self.keys) or self.keys[i] - self.keys[i - 1] > max_gap_ms:
            return None
        a, b = self.samples[i - 1].fields, self.samples[i].fields
        w = (boot_ms - self.keys[i - 1]) / (self.keys[i] - self.keys[i - 1])
        out = {}
        for n in names:
            va, vb = a[n], b[n]
            if va is None or vb is None:
                return None
            if n in angles:
                d = (vb - va + math.pi) % (2 * math.pi) - math.pi
                out[n] = va + w * d
            else:
                out[n] = va + w * (vb - va)
        return out


@dataclass
class Flight:
    kind: str                       # multirotor | vtol | fixed_wing | ground
    mav_type: int | None
    autopilot: int | None
    leg_m: float
    home: tuple[float, float, float]  # lat, lon (deg), AMSL (m) at the start
    land_point: tuple[float, float]
    land_seq: int
    samples: dict[str, list[Sample]] = field(default_factory=dict)
    events: dict[str, float] = field(default_factory=dict)  # name -> host monotonic time
    problem: str | None = None      # why the flight didn't complete, if it didn't
    started: bool = False           # the mission started (arming and start_mission accepted)

    # -- access ---------------------------------------------------------------

    @property
    def excluded(self) -> tuple[float, float] | None:
        """
        From an early ground contact until airborne again (PX4 SIH fixed-wing: stalled into the
        ground after its takeoff climb, then slid along it under throttle before taking off again).
        Neither flying nor at rest, so every check leaves it out.
        """
        first = self.t("first_touchdown")
        if first is None:
            return None
        again = next((g.host_mono for g in self.samples.get("GLOBAL_POSITION_INT") or []
                      if g.host_mono > first and g.fields["relative_alt"] / 1000.0 >= AIRBORNE_M), None)
        return None if again is None else (first, again)

    def s(self, name: str, t0: float | None = None, t1: float | None = None) -> list[Sample]:
        out = self.samples.get(name) or []
        gap = self.excluded
        if gap is not None:
            out = [x for x in out if not gap[0] <= x.host_mono <= gap[1]]
        if t0 is not None:
            out = [x for x in out if x.host_mono >= t0]
        if t1 is not None:
            out = [x for x in out if x.host_mono <= t1]
        return out

    def boot(self, name: str) -> BootIndex:
        return BootIndex(self.s(name))

    def t(self, event: str) -> float | None:
        return self.events.get(event)

    def boot_ms_at(self, host_mono: float) -> float | None:
        """The vehicle's time_boot_ms at a host time (from the nearest GLOBAL_POSITION_INT)."""
        gpi = self.s("GLOBAL_POSITION_INT")
        if not gpi:
            return None
        g = min(gpi, key=lambda x: abs(x.host_mono - host_mono))
        return g.fields["time_boot_ms"] + (host_mono - g.host_mono) * 1000.0

    @property
    def flying(self) -> tuple[float, float] | None:
        """(airborne, touchdown) host times, or None if it never left the ground."""
        a = self.t("airborne")
        if a is None:
            return None
        return a, self.t("touchdown") or self.t("end")

    @property
    def moving(self) -> tuple[float, float] | None:
        """From arming to the end of the mission (touchdown, or arrival for a ground vehicle)."""
        a = self.t("armed")
        if a is None:
            return None
        return a, self.t("touchdown") or self.t("arrived") or self.t("end")

    @property
    def label(self) -> str:
        """The vehicle kind for a detail line, plus anything unusual about the flight."""
        first, air = self.t("first_touchdown"), self.t("airborne")
        if first is not None and air is not None:
            gap = self.excluded
            left = "" if gap is None else f" ({gap[1] - gap[0]:.0f} s on the ground left out)"
            return f"{self.kind}; hit the ground {first - air:.0f} s after takeoff and took off again{left}"
        return self.kind

    def describe(self) -> str:
        rel = {k: f"{v - self.events['start']:.0f}s" for k, v in self.events.items() if "start" in self.events}
        return f"{self.kind} (HEARTBEAT type {self.mav_type}), legs {self.leg_m:.0f} m, events {rel}" + (
            f"; problem: {self.problem}" if self.problem else "")

    # -- saving -----------------------------------------------------------------

    def save(self, path) -> None:
        doc = {k: v for k, v in self.__dict__.items() if k != "samples"}
        doc["samples"] = {n: [[x.host_mono, x.host_wall, x.sysid, x.compid, x.fields] for x in v]
                          for n, v in self.samples.items()}
        path.write_text(json.dumps(doc), encoding="utf-8")

    @classmethod
    def load(cls, path) -> "Flight":
        doc = json.loads(open(path, encoding="utf-8").read())
        samples = {n: [Sample(*x) for x in v] for n, v in doc.pop("samples").items()}
        doc["home"], doc["land_point"] = tuple(doc["home"]), tuple(doc["land_point"])
        return cls(samples=samples, **doc)

    # -- preconditions --------------------------------------------------------

    def need_moving(self, request, fields) -> tuple[float, float]:
        if self.moving is None or not self.started:
            _inconclusive(request, fields, self, "the vehicle never started its mission")
        return self.moving

    def need_flying(self, request, fields) -> tuple[float, float]:
        if self.flying is None:
            if self.kind == "ground":
                for f in fields:
                    record_field_fact(request, f, untestable="a ground vehicle: never leaves the ground")
                pytest.skip(f"NA: a ground vehicle (HEARTBEAT type {self.mav_type}) never leaves the ground")
            _inconclusive(request, fields, self, "the vehicle never got airborne")
        return self.flying

    def need_touchdown(self, request, fields) -> float:
        self.need_flying(request, fields)
        if self.t("touchdown") is None:
            _inconclusive(request, fields, self, "the vehicle never landed")
        return self.t("touchdown")


def _inconclusive(request, fields, flight: Flight, what: str) -> None:
    for f in fields:
        record_field_fact(request, f, untestable=f"flight incomplete: {what}")
    pytest.fail(f"Inconclusive — {what}: {flight.describe()}")


# ---------------------------------------------------------------------------
# Flying it
# ---------------------------------------------------------------------------

_FLIGHT: Flight | None = None
_FLIGHT_ERROR: str | None = None


async def get_flight(system, restart) -> Flight:
    """The session's flight, flown on first use (and then the flight stack restarted)."""
    global _FLIGHT, _FLIGHT_ERROR
    if _FLIGHT is None and _FLIGHT_ERROR is None and REPLAY:
        _FLIGHT = Flight.load(REPLAY)
        refine_events(_FLIGHT)
        log.warning("message flight: REPLAYING %s, not flying", REPLAY)
    if _FLIGHT is None and _FLIGHT_ERROR is None:
        try:
            _FLIGHT = await _fly(system, restart)
            _save(_FLIGHT)
            log.info("message flight: %s; samples %s", _FLIGHT.describe(),
                     {k: len(v) for k, v in _FLIGHT.samples.items()})
        except Exception as exc:  # noqa: BLE001 — every later flight test reports the same cause
            log.exception("message flight failed")
            _FLIGHT_ERROR = repr(exc)
    if _FLIGHT is None:
        pytest.fail(f"Inconclusive — the shared message flight failed: {_FLIGHT_ERROR}")
    return _FLIGHT


def _save(flight: Flight) -> None:
    """Keep the recording in logs/ (gitignored) for MESSAGE_FLIGHT_REPLAY."""
    try:
        logs = pathlib.Path(__file__).resolve().parents[2] / "logs"
        logs.mkdir(exist_ok=True)
        stack = {12: "px4", 3: "ardupilot"}.get(flight.autopilot, f"autopilot{flight.autopilot}")
        path = logs / f"message_flight_{stack}_{flight.kind}_{time.strftime('%Y%m%d_%H%M%S')}.json"
        flight.save(path)
        log.info("message flight recording: %s", path)
    except Exception as exc:  # noqa: BLE001 — losing the copy loses nothing the tests need
        log.warning("could not save the message flight recording: %r", exc)


def _kind(mav_type: int | None, vtol_state_at_rest: int | None) -> str:
    if mav_type in _VTOL_MAV_TYPES or (mav_type == 1 and vtol_state_at_rest in (3, 4)):
        return "vtol"
    if mav_type in MULTIROTOR_TYPES:
        return "multirotor"
    if mav_type in GROUND_TYPES:
        return "ground"
    return "fixed_wing"


def _item(seq: int, command: int, latlon: tuple[int, int], z: float, *, frame: int = 6, current: int = 0,
          param1: float = 0.0) -> MissionItem:
    return MissionItem(seq=seq, frame=frame, command=command, current=current, autocontinue=1,
                       param1=param1, param2=0.0, param3=0.0, param4=float("nan"),
                       x=latlon[0], y=latlon[1], z=z, mission_type=0)


def build_mission(kind: str, home, leg_m: float, ardupilot: bool) -> tuple[list[MissionItem], int, tuple[float, float]]:
    """(items, seq of the final item, land point lat/lon) for the route in this module's docstring."""
    def ll(n: float, e: float) -> tuple[int, int]:
        lat, lon = _offset_lat_lon(home.latitude_deg, home.longitude_deg, n, e)
        return int(lat * 1e7), int(lon * 1e7)

    s = leg_m
    ground = kind == "ground"
    # A fixed-wing climbs on to 50 m at the first waypoint: levelling off at 30 m straight after the
    # takeoff climb (18 m/s, full throttle), PX4 SIH fixed-wing cut the throttle, lost 12 m/s in a second,
    # stalled and flew into the ground, solo and under load alike (2026-11-12).
    first_alt = HIGH_ALT_M if kind == "fixed_wing" else TAKEOFF_ALT_M
    route = [(ll(s, 0), 0.0 if ground else first_alt),
             (ll(s, s), 0.0 if ground else HIGH_ALT_M),
             (ll(0, s), 0.0 if ground else TAKEOFF_ALT_M)]
    land = ll(0, -s)
    # (command, position, altitude, param1). NAV_TAKEOFF at home with a 15° minimum pitch, as
    # tests/mission/nav_takeoff flies it: with the takeoff position at the first waypoint, PX4 SIH
    # fixed-wing pitched 28° down at 27 m right after the takeoff item and hit the ground (2026-11-12).
    body: list[tuple[int, tuple[int, int], float, float]] = []
    if kind == "vtol":
        body.append((NAV_VTOL_TAKEOFF, route[0][0], TAKEOFF_ALT_M, 0.0))
    elif kind == "multirotor":
        body.append((NAV_TAKEOFF, ll(0, 0), TAKEOFF_ALT_M, 0.0))
    elif kind == "fixed_wing":
        body.append((NAV_TAKEOFF, ll(0, 0), TAKEOFF_ALT_M, 15.0))
    body += [(NAV_WAYPOINT, p, z, 0.0) for p, z in route]
    body.append(({"vtol": NAV_VTOL_LAND, "ground": NAV_WAYPOINT}.get(kind, NAV_LAND), land, 0.0, 0.0))
    items: list[MissionItem] = []
    if ardupilot:  # ArduPilot's mission item 0 is home
        items.append(_item(0, NAV_WAYPOINT, ll(0, 0), 0.0, frame=0))
    first = len(items)
    for i, (cmd, p, z, param1) in enumerate(body):
        items.append(_item(first + i, cmd, p, z, current=1 if i == 0 else 0, param1=param1))
    return items, len(items) - 1, (land[0] / 1e7, land[1] / 1e7)


async def _poll(system, stop: asyncio.Event) -> None:
    while not stop.is_set():
        for name in POLLED:
            try:
                await request_message(system, MSG_IDS[name])
            except Exception as exc:  # noqa: BLE001 — a missed poll is just a gap in the data
                log.debug("poll %s failed: %r", name, exc)
            await asyncio.sleep(0.2)
        try:
            await asyncio.wait_for(stop.wait(), POLL_S)
        except asyncio.TimeoutError:
            pass


def _gpi_state(sample: Sample) -> tuple[float, float, float]:
    f = sample.fields
    return f["relative_alt"] / 1000.0, math.hypot(f["vx"], f["vy"]) / 100.0, f["vz"] / 100.0


async def _fly(system, restart) -> Flight:
    from tests.mission.conftest import clear_all_mission_types

    hb = await vehicle_heartbeat(system)
    target = (hb["sysid"], hb["compid"])
    home = await _get_home_position(system)
    await _get_heading(system, timeout_s=ARMABLE_TIMEOUT_S)  # position ready (root CLAUDE.md pattern #12)

    sampler = MessageSampler(system, *NAMES)
    await sampler.__aenter__()
    stop_poll = asyncio.Event()
    quiet = os.environ.get("MESSAGE_FLIGHT_QUIET") == "1"  # debugging: no rate requests or polling
    poller = None if quiet else asyncio.create_task(_poll(system, stop_poll))
    flight: Flight | None = None
    try:
        for name, hz in ({} if quiet else RATES_HZ).items():
            await set_message_interval(system, MSG_IDS[name], 1e6 / hz)
        await asyncio.sleep(2.0)
        ess = autopilot_only(sampler.samples("EXTENDED_SYS_STATE"), target)
        vtol_state = int(ess[-1].fields["vtol_state"]) if ess else None
        kind = _kind(hb.get("type"), vtol_state)
        leg = LEG_M[kind]
        ardupilot = hb.get("autopilot") == MAV_AUTOPILOT_ARDUPILOTMEGA
        items, last_seq, land_point = build_mission(kind, home, leg, ardupilot)
        flight = Flight(kind, hb.get("type"), hb.get("autopilot"), leg,
                        (home.latitude_deg, home.longitude_deg, home.absolute_altitude_m),
                        land_point, last_seq)
        ev = flight.events
        ev["start"] = time.monotonic()
        log.info("message flight: %s, legs %.0f m, %d items, vtol_state at rest %s", kind, leg, len(items), vtol_state)

        try:
            await asyncio.wait_for(system.mission_raw.upload_mission(items), 30)
        except MissionRawError as exc:
            flight.problem = f"mission upload refused: {exc}"
            return flight
        await _wait_armable(system, ARMABLE_TIMEOUT_S)
        await system.action.arm()
        try:
            await system.mission_raw.start_mission()
        except MissionRawError as exc:
            flight.problem = f"start_mission refused ({exc.result.name})"
            return flight
        flight.started = True
        await _watch(system, sampler, target, flight)
        if flight.t("disarmed") is not None or flight.t("arrived") is not None:
            await _rearm(system, sampler, target, flight)
        return flight
    finally:
        stop_poll.set()
        if poller is not None:
            poller.cancel()
        await sampler.__aexit__(None, None, None)
        if flight is not None:
            flight.samples = {n: autopilot_only(sampler.samples(n), target) for n in NAMES}
            flight.events.setdefault("end", time.monotonic())
            refine_events(flight)
        try:
            await _rtl_and_land(system, restart)  # returns at once if disarmed
            await clear_all_mission_types(system)
        except Exception as exc:  # noqa: BLE001 — the restart below cleans up anyway
            log.warning("message flight cleanup: %r", exc)
        # Leave the vehicle at rest at its origin for the ground tests (module docstring).
        try:
            restart()
        except Exception as exc:  # noqa: BLE001
            log.warning("restart after the message flight failed: %r", exc)


def refine_events(flight: Flight) -> None:
    """
    Arming and disarming times from the recorded HEARTBEATs (the first one showing
    the new state), not from when the 0.5 s watch loop noticed: a state reported
    just after arming must not count as "before arming".
    """
    hbs = [h for h in flight.s("HEARTBEAT") if h.fields.get("autopilot") != 8]
    ev = flight.events
    order = [("armed", True, ev.get("start")), ("disarmed", False, "armed"),
             ("rearmed", True, "disarmed"), ("redisarmed", False, "rearmed")]
    for name, state, after in order:
        if name not in ev:
            continue
        t_after = ev.get(after) if isinstance(after, str) else after
        if t_after is None:
            continue
        hit = next((h.host_mono for h in hbs if h.host_mono >= t_after
                    and bool(int(h.fields["base_mode"]) & MAV_MODE_FLAG_SAFETY_ARMED) == state), None)
        if hit is not None and hit <= ev[name] + 1.0:
            ev[name] = hit
    # The landing is the last time it came down, not the first: PX4 SIH fixed-wing stalls into the
    # ground straight after its takeoff climb, then takes off again and flies the route (2026-11-12).
    air, end = ev.get("airborne"), ev.get("disarmed") or ev.get("end")
    if air is not None and "touchdown" in ev and end is not None:
        gpi = flight.s("GLOBAL_POSITION_INT", air, end)
        up = [g.host_mono for g in gpi if g.fields["relative_alt"] / 1000.0 >= ON_GROUND_M]
        final = next((g.host_mono for g in gpi if up and g.host_mono > max(up)), None)
        if final is not None and final - ev["touchdown"] > LANDED_HOLD_S:
            ev.setdefault("first_touchdown", ev["touchdown"])
            ev["touchdown"] = final


def _armed(sampler, target) -> bool | None:
    hbs = [h for h in autopilot_only(sampler.samples("HEARTBEAT"), target) if h.fields.get("autopilot") != 8]
    return None if not hbs else bool(int(hbs[-1].fields["base_mode"]) & MAV_MODE_FLAG_SAFETY_ARMED)


async def _watch(system, sampler, target, flight: Flight) -> None:
    """Follow the mission until touchdown and disarm (or, for a ground vehicle, arrival), recording events."""
    ev = flight.events
    deadline = time.monotonic() + FLIGHT_TIMEOUT_S
    landed_since = None
    max_alt = 0.0
    seq = None
    while time.monotonic() < deadline:
        await asyncio.sleep(0.5)
        now = time.monotonic()
        armed = _armed(sampler, target)
        if armed and "armed" not in ev:
            ev["armed"] = now
        gpi = autopilot_only(sampler.samples("GLOBAL_POSITION_INT"), target)
        mc = autopilot_only(sampler.samples("MISSION_CURRENT"), target)
        if mc and mc[-1].fields["seq"] != seq:
            seq = int(mc[-1].fields["seq"])
            ev[f"seq{seq}"] = mc[-1].host_mono
        if not gpi:
            continue
        rel_alt, speed, vz = _gpi_state(gpi[-1])
        max_alt = max(max_alt, rel_alt)
        if "airborne" not in ev and rel_alt >= AIRBORNE_M and "armed" in ev:
            first = next(g for g in gpi if g.host_mono >= ev["armed"] and _gpi_state(g)[0] >= AIRBORNE_M)
            ev["airborne"] = first.host_mono
        if flight.kind == "ground":
            if seq == flight.land_seq and _dist(gpi[-1], flight.land_point) <= ARRIVED_M and speed < 0.3:
                ev["arrived"] = now
                await asyncio.sleep(2.0)
                await _disarm(system, flight)
                return
            continue
        if "airborne" in ev and "touchdown" not in ev and rel_alt < ON_GROUND_M and max_alt >= TOUCHDOWN_AFTER_M:
            last_up = max(g.host_mono for g in gpi if _gpi_state(g)[0] >= ON_GROUND_M)
            ev["touchdown"] = next(g.host_mono for g in gpi if g.host_mono > last_up)
        if "touchdown" in ev:
            if armed is False:
                ev["disarmed"] = now
                return
            still = rel_alt < ON_GROUND_M and speed < 0.5 and abs(vz) < 0.3
            landed_since = (landed_since or now) if still else None
            if landed_since and now - landed_since >= LANDED_HOLD_S:
                log.info("message flight: still armed %.0f s after touchdown — disarming", now - ev["touchdown"])
                await _disarm(system, flight)
                return
    flight.problem = (f"mission not complete within {FLIGHT_TIMEOUT_S:.0f} s (MISSION_CURRENT seq {seq}, "
                      f"last item {flight.land_seq}, max altitude {max_alt:.1f} m)")


def _dist(gpi: Sample, point: tuple[float, float]) -> float:
    lat, lon = gpi.fields["lat"] / 1e7, gpi.fields["lon"] / 1e7
    n = math.radians(lat - point[0]) * 6_378_137.0
    e = math.radians(lon - point[1]) * 6_378_137.0 * math.cos(math.radians(lat))
    return math.hypot(n, e)


async def _disarm(system, flight: Flight) -> None:
    try:
        await system.action.disarm()
        flight.events["disarmed"] = time.monotonic()
    except Exception as exc:  # noqa: BLE001
        log.warning("disarm after the mission failed: %r", exc)


async def _rearm(system, sampler, target, flight: Flight) -> None:
    """
    Arm and disarm once on the ground where the vehicle ended up, out of the
    mission (Hold first — re-arming in a mission mode could start it again),
    so a stack that sets home at arming moves it away from the origin.
    """
    await asyncio.sleep(3.0)
    try:
        await system.action.hold()
    except Exception as exc:  # noqa: BLE001
        log.info("hold before re-arm: %r", exc)
    try:
        await _wait_armable(system, 60.0)
        await system.action.arm()
        flight.events["rearmed"] = time.monotonic()
        await asyncio.sleep(REARM_S)
    except Exception as exc:  # noqa: BLE001 — a refusal only loses the home-reset checks
        log.warning("re-arm after landing failed: %r", exc)
        return
    finally:
        if _armed(sampler, target):
            try:
                await system.action.disarm()
            except Exception as exc:  # noqa: BLE001
                log.warning("disarm after re-arm failed: %r", exc)
    flight.events["redisarmed"] = time.monotonic()
    await asyncio.sleep(2 * POLL_S + 1.0)  # a fresh copy of each polled message


# ---------------------------------------------------------------------------
# Shared analysis helpers
# ---------------------------------------------------------------------------

def body_to_ned(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    """Rotation matrix R (body → NED) for ZYX Euler angles in radians."""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return [[cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
            [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
            [-sp, sr * cp, cr * cp]]


def mat_vec(m, v, transpose: bool = False) -> list[float]:
    if transpose:
        return [sum(m[j][i] * v[j] for j in range(3)) for i in range(3)]
    return [sum(m[i][j] * v[j] for j in range(3)) for i in range(3)]


def angle_diff(a: float, b: float) -> float:
    """a − b in degrees, wrapped to −180..180."""
    return (a - b + 180.0) % 360.0 - 180.0


def heading_spread(headings_deg: list[float]) -> float:
    """Largest gap-complement of a set of headings: how much of the circle they cover (0..360)."""
    if len(headings_deg) < 2:
        return 0.0
    h = sorted(x % 360.0 for x in headings_deg)
    gaps = [b - a for a, b in zip(h, h[1:])] + [h[0] + 360.0 - h[-1]]
    return 360.0 - max(gaps)


def fraction(flags: list[bool]) -> float:
    return sum(flags) / len(flags) if flags else 0.0
