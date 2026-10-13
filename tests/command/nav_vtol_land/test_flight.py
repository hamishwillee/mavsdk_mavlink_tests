"""
MAV_CMD_NAV_VTOL_LAND (cmd=85) via COMMAND_INT — Tier 2 execution tests.

"Land using VTOL mode"

Takes off, puts a VTOL into forward flight, sends NAV_VTOL_LAND as a COMMAND_INT
and checks each parameter against telemetry. Requires a real flight stack
(--drone-address); every test is skipped in paired/mock mode. The flight routine
and verdict helpers are shared with the other three landing modules —
tests/land_helpers.py (read its docstring for the conventions).

- Test names are `test_vtol_land_<compat|obs>_<what>` (root CLAUDE.md rule 8).
- Latitude/Longitude, Yaw and Approach Altitude share two flights (rule 7): due N
  of the start with yaw 135° and approach altitude 50 m, due E with 315° and 70 m
  (rule 4c). If the combined send is NACKed, each value is sent on its own to
  find which one the stack rejects.
- Whether the command is supported at all is learned from one disarmed probe. A
  landing command sent to a vehicle on the ground can change its flight mode, so
  when the probe isn't UNSUPPORTED the flight stack is restarted before any
  flight (see tests/command/nav_vtol_takeoff/CLAUDE.md for what a disarmed probe
  did there).

Expected from source (blind read, 2026-10-13 — Tier 2 pattern #5)
-----------------------------------------------------------------
PX4 v1.18.0-beta1-660: UNSUPPORTED as a command on every vehicle type — only the
mission executor implements NAV_VTOL_LAND (see test_command.py's docstring). So
every test here should skip on PX4; tests/mission/nav_vtol_land/ is where PX4's
behaviour is measured.

Running
-------
PX4 SIH VTOL (the px4io/px4-sitl container build — see README.md)::

    pytest tests/command/nav_vtol_land/test_flight.py --drone-address=udp://:14540 \\
        --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_standard_vtol \\
        --vehicle-type=vtol --autopilot=px4 -v --log-cli-level=INFO
"""

import logging

import pytest

from tests import land_helpers as lh
from tests import report
from tests.command.conftest import _FMT, probe_command_int
from .test_command import SPEC as _SPEC, LAND_OPTIONS
from tests.flight_helpers import (
    _get_home_position,
    _tier2_auto_record,  # noqa: F401 — autouse: records every test's outcome for the Tier 2 log
    record_compat_command_supported,
    record_compat_json,
    record_tier2_detail,
    record_tier2_param_verdict,
    require_real_stack,  # noqa: F401 — registers the real-stack skip gate for this module
)
from tests.mock_flight_stack import MAV_RESULT_UNSUPPORTED

log = logging.getLogger(__name__)

# The shared pair is two flights, each up to TOUCHDOWN_TIMEOUT_S plus takeoff/cleanup.
pytestmark = pytest.mark.timeout(1200)

_CMD = "NAV_VTOL_LAND"
_CMD_ID = 85
_CMD_NAME = _CMD  # the report key — must match test_command.py's SPEC.name
assert _SPEC.name == _CMD_NAME

_supported: bool | None = None
_pair: list[dict] | None = None  # cached shared flights (rule 7)
_restart = None


@pytest.fixture(autouse=True)
def _bind_restart_flight_stack(restart_flight_stack):
    """Every cleanup restarts the stack if the vehicle doesn't land (root CLAUDE.md Tier 2 pattern #8)."""
    global _restart
    _restart = restart_flight_stack
    yield
    _restart = None


async def _ensure_supported(system) -> None:
    """One disarmed probe per session; skip every flight test when the command is UNSUPPORTED."""
    global _supported
    if _supported is None:
        home = await _get_home_position(system)
        ack = await probe_command_int(system, command=_CMD_ID, frame=6, param1=0.0, param2=0.0, param3=None,
                                      param4=None, x=int(home.latitude_deg * 1e7), y=int(home.longitude_deg * 1e7), z=0.0)
        if ack is None:
            pytest.fail(f"No COMMAND_ACK for {_CMD} — cannot tell whether it is supported")
        _supported = int(ack["result"]) != MAV_RESULT_UNSUPPORTED
        log.info(_FMT, _CMD, "disarmed support probe", f"result={int(ack['result'])}")
        report.record_command_fact("command", _CMD_NAME, supported=_supported)
        if _supported and _restart is not None:
            _restart()  # the probe may have put the grounded vehicle into a landing mode
    if not _supported:
        pytest.skip(f"{_CMD} (cmd={_CMD_ID}) is UNSUPPORTED via COMMAND_INT on this stack — flight tests not run")


async def _flights(system) -> list[dict]:
    global _pair
    if _pair is None:
        async def _fly(**kw):
            return await lh.fly_command(system, _restart, _CMD_ID, **kw)
        _pair = await lh.fly_pair(_fly, system,
                                  extra=tuple({"approach": {"param3": a}} for a in lh.APPROACH_ALT_VALUES_M))
        for r, alt in zip(_pair, lh.APPROACH_ALT_VALUES_M):
            r["approach_alt"] = alt
    return _pair


async def test_vtol_land_compat_lands_in_vtol_mode(gcs_system, request):
    """
    The XML's one requirement: the vehicle lands, in VTOL (hover) mode.

    PASS if every accepted NAV_VTOL_LAND ends in a touchdown and — for a landing
    commanded in forward flight — the vehicle is back in multicopter mode when it
    touches down. A vehicle that lands from a hover (not a VTOL) can only show the
    first half; the detail says which.
    """
    await _ensure_supported(gcs_system)
    flights = await _flights(gcs_system)
    ok, detail = lh.lands_verdict(flights)
    record_tier2_detail(request, f"PASS if the vehicle touches down in VTOL (hover) mode: {detail}")
    if ok is None:
        pytest.skip(f"NA: every NAV_VTOL_LAND was NACKed in flight — {detail}")
    if not ok:
        record_compat_command_supported(request, False, notes="Accepted, but the vehicle does not land")
        report.compat_fail(f"NAV_VTOL_LAND accepted but the vehicle did not touch down: {detail}")
    if any(not r["nacked"] and r["touchdown"]["vtol_state"] in ("FW", "TRANSITION_TO_FW") for r in flights):
        record_compat_command_supported(request, True, notes="Lands, but not in VTOL (hover) mode")
        report.compat_fail(f"NAV_VTOL_LAND touched down while still in fixed-wing mode: {detail}")
    record_compat_command_supported(request, True)


async def test_vtol_land_compat_position_honoured(gcs_system, request):
    """params 5/6 = points due N and due E of the start — "is lat/lon honoured": NACKed, or the vehicle touches down there."""
    await _ensure_supported(gcs_system)
    lh.record_param(request, "param5/6 (Lat/Lon)", ("5_Latitude", "6_Longitude"), lh.position_verdict(await _flights(gcs_system)),
                    f"PASS if each target is NACKed or the touchdown is within {lh.TOUCHDOWN_TOLERANCE_M:.0f} m of it")


async def test_vtol_land_compat_yaw_honoured(gcs_system, request):
    """param4 = 135° and 315° — "is yaw honoured": NACKed, or the vehicle touches down on that heading."""
    await _ensure_supported(gcs_system)
    lh.record_param(request, "param4 (Yaw)", ("4_Yaw",), lh.yaw_verdict(await _flights(gcs_system)),
                    f"PASS if each yaw is NACKed or the heading at touchdown is within ±{lh.YAW_TOLERANCE_DEG:.0f}° of it")


async def test_vtol_land_compat_approach_altitude_honoured(gcs_system, request):
    """param3 = 50 m and 70 m — "is the approach altitude honoured": NACKed, or the vehicle climbs to it before descending."""
    await _ensure_supported(gcs_system)
    lh.record_param(request, "param3 (Approach Altitude)", ("3_Approach Altitude",),
                    lh.approach_altitude_verdict(await _flights(gcs_system)),
                    "PASS if each approach altitude is NACKed or is the highest altitude reached during the landing")


async def test_vtol_land_obs_approach(gcs_system, request):
    """Characterisation: how the landing is flown — where it starts, where the back-transition happens, where it touches down."""
    await _ensure_supported(gcs_system)
    record_tier2_detail(request, "Observational — " + " | ".join(
        f"{r['label']}: {lh.describe_landing(r)}" for r in await _flights(gcs_system)))


async def test_vtol_land_obs_ground_altitude(gcs_system, request):
    """
    param7 = 15 m above home ("ground level") — characterisation: does the descent end there or on the ground?

    param7 is the ground level, not a destination, so on flat simulated terrain
    there is nothing to compare a touchdown against — recorded as untestable
    (rule 10d); this records what a deliberately wrong ground level does.
    """
    await _ensure_supported(gcs_system)
    report.record_compat_fact("command", _CMD_NAME, "7_Ground Altitude", untestable=lh.GROUND_ALT_UNTESTABLE)
    r = await lh.fly_command(gcs_system, _restart, _CMD_ID, groups={"altitude": {"z": 15.0}})
    record_tier2_detail(request, f"Observational: z = 15 m — {lh.describe_landing(r)}"
                        + ("" if r["nacked"] or not r["landed"] else f"; touchdown altitude {r['touchdown']['alt']:.1f} m"))


async def test_vtol_land_compat_untestable_params(gcs_system, request):
    """
    params 1 (Land Options), 2 (Empty) and Yaw = NaN — no execution verdict possible; records why, without flying.

    Land Options: DEFAULT is "default autopilot landing behaviour", so there is no
    fixed approach shape to tell the other two values apart from. Tier 1
    (test_command.py) records the ACK for each value.
    """
    await _ensure_supported(gcs_system)
    report.record_compat_fact("command", _CMD_NAME, "1_Land Options",
                              untestable="DEFAULT is autopilot-defined — no fixed approach shape to compare against")
    record_compat_json(request, "2_Empty", supported="not-applicable")
    record_tier2_param_verdict(request, "Yaw sentinel (NaN)", lh.YAW_SENTINEL_NOT_TESTABLE)
    pytest.skip("NA: Land Options and the Yaw sentinel can't be judged from a landing — see docstring")
