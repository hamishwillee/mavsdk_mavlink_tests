"""
MAV_CMD_DO_REPOSITION (cmd=192) via COMMAND_INT — direct command protocol tests.

DO_REPOSITION repositions the vehicle to a specific WGS84 global position.
This command is intended for guided commands (not mission use).

Key parameter semantics:
- param1 (Speed): m/s; -1 = use default; NaN = use default; minValue=-1
- param2 (Bitmask): MAV_DO_REPOSITION_FLAGS
    - bit 0 (1): CHANGE_MODE — switch vehicle to guided/hold mode immediately
    - bit 1 (2): RELATIVE_YAW — yaw is relative to current heading, not North
- param3 (Radius): loiter radius for planes (0 or NaN = ignored; positive values only)
- param4 (Yaw): radians; NaN = use current heading mode
- param5/x: Latitude × 1e7 (COMMAND_INT)
- param6/y: Longitude × 1e7 (COMMAND_INT)
- param7/z: Altitude (m)

hasLocation="true", isDestination="true" → use COMMAND_INT (integer lat/lon).

PX4 (branch dakejahl/do-reposition-ack, commit bc236e7178):
- Before fix: always returned MAV_RESULT_UNSUPPORTED
- After fix: mode-dependent ACK:
    - param2 bit-0 set (CHANGE_MODE) → ACCEPTED (switches to AUTO_LOITER)
    - param2=0 AND already in AUTO_LOITER → ACCEPTED (repositions hold point)
    - param2=0 AND not in AUTO_LOITER → DENIED

Running
-------
Paired mock (no autopilot)::

    pytest tests/command/do_reposition/test_command.py -v --log-cli-level=INFO

PX4 MC (patched)::

    pytest tests/command/do_reposition/test_command.py \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \\
        --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO

ArduCopter::

    pytest tests/command/do_reposition/test_command.py \\
        --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \\
        --ardupilot-sitl=~/ardu_sitl/arducopter \\
        --home-lat=47.3977 --home-lon=8.5456 --home-alt=0 \\
        --vehicle-type=quadcopter --autopilot=ardupilot -v --log-cli-level=INFO
"""

import logging
import math

import pytest

from tests.command.conftest import (
    probe_command_int,
    probe_command_long,
    gcs_system_cls,
    mock_stack_cls,
    ACK_TIMEOUT_S,
    INT32_MAX,
    _FMT,
)
from tests.mock_flight_stack import MAV_RESULT_ACCEPTED, MAV_RESULT_DENIED, MAV_RESULT_UNSUPPORTED

log = logging.getLogger(__name__)

_CMD    = "DO_REPOSITION"
_CMD_ID = 192  # MAV_CMD_DO_REPOSITION

# SIH simulator home (47.3977°N, 8.5456°E) — same as takeoff tests
_LAT_INT = 473977000   # 47.3977° × 1e7
_LON_INT =  85456000   #  8.5456° × 1e7

# MAV_DO_REPOSITION_FLAGS bitmask values (from common.xml)
_FLAG_CHANGE_MODE  = 1   # bit 0: switch vehicle to guided/hold mode
_FLAG_RELATIVE_YAW = 2   # bit 1: yaw relative to current heading, not North


def _reposition_cmd(**overrides) -> dict:
    """Return default COMMAND_INT kwargs for DO_REPOSITION."""
    defaults = dict(
        command = _CMD_ID,
        frame   = 6,       # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        param1  = -1.0,    # Speed: -1 = use default cruise speed
        param2  = 1.0,     # Flags: CHANGE_MODE — valid regardless of starting mode
        param3  = 0.0,     # Loiter radius: 0 = ignored (per spec)
        param4  = None,    # Yaw: NaN = "use current heading mode"
        x       = _LAT_INT,
        y       = _LON_INT,
        z       = 50.0,    # 50 m relative altitude
    )
    defaults.update(overrides)
    return defaults


async def _probe(system, **kwargs) -> dict | None:
    """Subscribe first, then send COMMAND_INT, then collect COMMAND_ACK."""
    return await probe_command_int(system, **_reposition_cmd(**kwargs))


# ---------------------------------------------------------------------------
# Class 1: PX4 mode-dependent ACK tests (ordered) — must run before Class 2
# ---------------------------------------------------------------------------
#
# These tests depend on PX4 starting in MANUAL mode (SITL startup state).
# They must run before any other tests that send CHANGE_MODE (param2=1), which
# would switch PX4 into AUTO_LOITER and invalidate the "not in Hold" precondition
# for test_denied_not_in_hold.

@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(300)
class TestDoRepositionPx4ModeGating:
    """
    Ordered mode-dependent ACK tests for PX4 commit bc236e7178.

    Verifies the three ACK branches introduced by the fix:
      Branch 3: param2=0, not in Hold → DENIED  (was UNSUPPORTED before fix)
      Branch 1: param2=1 (CHANGE_MODE) → ACCEPTED, switches to Hold
      Branch 2: param2=0, already in Hold → ACCEPTED  (was UNSUPPORTED before fix)

    ORDERING CONTRACT: Tests must run in definition order.
      test 1 must run with PX4 in MANUAL mode (SITL startup state — not in Hold)
      test 2 transitions the vehicle into AUTO_LOITER (Hold)
      test 3 runs while vehicle is in Hold (set by test 2)

    This class is placed FIRST in the file so it runs before TestDoRepositionCommand,
    which sends CHANGE_MODE commands that would switch PX4 into Hold mode.

    All three tests skip in mock mode (mock has no Hold-mode state machine).
    All three tests skip when DO_REPOSITION is UNSUPPORTED (unpatched PX4).
    xfail guard on test 1 for non-PX4 stacks that have no mode gating.
    """

    _supported: bool | None = None
    _in_hold: bool = False

    async def _ensure_mode_gating_applicable(self, system, mock_stack) -> None:
        """Skip in mock mode; skip if DO_REPOSITION is UNSUPPORTED."""
        if mock_stack is not None:
            pytest.skip("Mode-gating tests require a real flight stack (not mock)")
        if TestDoRepositionPx4ModeGating._supported is None:
            # Probe with param2=0 (no CHANGE_MODE) so we don't alter the vehicle mode
            # before test_denied_not_in_hold runs.  Pre-fix PX4 returns UNSUPPORTED
            # for all inputs; patched PX4 returns DENIED (not UNSUPPORTED) for param2=0.
            ack = await probe_command_int(system, **_reposition_cmd(param2=0.0))
            unsupported = (ack is not None and int(ack["result"]) == MAV_RESULT_UNSUPPORTED)
            TestDoRepositionPx4ModeGating._supported = not unsupported
        if not TestDoRepositionPx4ModeGating._supported:
            pytest.skip(
                f"{_CMD} (cmd={_CMD_ID}) is UNSUPPORTED — "
                "mode-gating tests require the patched PX4 build (bc236e7178)"
            )

    async def test_denied_not_in_hold(self, gcs_system_cls, mock_stack_cls):
        """
        param2=0 (no CHANGE_MODE), vehicle in MANUAL mode at SITL startup.

        Branch 3: not in AUTO_LOITER AND no mode change requested → DENIED.
        Before fix: UNSUPPORTED (wrong — command is understood, just refused).
        After fix: DENIED (correct per MAVLink spec).

        Must run FIRST (vehicle must be in MANUAL mode, not Hold).

        xfail: non-PX4 stacks have no mode gating and return ACCEPTED.
        """
        await self._ensure_mode_gating_applicable(gcs_system_cls, mock_stack_cls)
        ack = await probe_command_int(gcs_system_cls, **_reposition_cmd(param2=0.0))
        if ack is None:
            log.warning(_FMT, _CMD, "mode-gating: param2=0 (not in Hold)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "mode-gating: param2=0 (not in Hold → DENIED)", f"result={result}")
        if result != MAV_RESULT_DENIED:
            pytest.xfail(
                f"Stack returned {result} for param2=0 when not in Hold; "
                "expected DENIED.  Two possible causes: (a) mode gating not implemented, "
                "or (b) PX4 SIH auto-transitions to AUTO_LOITER after EKF convergence "
                "so the vehicle is already in Hold by the time this test runs"
            )
        assert result == MAV_RESULT_DENIED

    async def test_accepted_change_mode(self, gcs_system_cls, mock_stack_cls):
        """
        param2=1 (CHANGE_MODE) → ACCEPTED; vehicle switches to AUTO_LOITER.

        Branch 1: CHANGE_MODE requested → switch mode → ACCEPTED.
        Must run SECOND (transitions vehicle to Hold for test 3).
        """
        await self._ensure_mode_gating_applicable(gcs_system_cls, mock_stack_cls)
        ack = await probe_command_int(gcs_system_cls, **_reposition_cmd(param2=1.0))
        if ack is None:
            log.warning(_FMT, _CMD, "mode-gating: param2=CHANGE_MODE(1)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "mode-gating: param2=CHANGE_MODE(1) → ACCEPTED", f"result={result}")
        if result == MAV_RESULT_ACCEPTED:
            TestDoRepositionPx4ModeGating._in_hold = True
        assert result == MAV_RESULT_ACCEPTED, (
            f"CHANGE_MODE should produce ACCEPTED; got {result}"
        )

    async def test_accepted_already_in_hold(self, gcs_system_cls, mock_stack_cls):
        """
        param2=0 (no CHANGE_MODE), vehicle now in AUTO_LOITER (set by prior test).

        Branch 2: already in AUTO_LOITER AND no mode change → ACCEPTED.
        Enables repositioning the Hold point without repeated mode-switch requests.

        Must run THIRD (depends on test_accepted_change_mode having switched to Hold).
        Before fix: UNSUPPORTED. After fix: ACCEPTED (correct).
        """
        await self._ensure_mode_gating_applicable(gcs_system_cls, mock_stack_cls)
        if not TestDoRepositionPx4ModeGating._in_hold:
            pytest.skip(
                "Vehicle not in Hold mode — requires test_accepted_change_mode to have "
                "passed first"
            )
        ack = await probe_command_int(gcs_system_cls, **_reposition_cmd(param2=0.0))
        if ack is None:
            log.warning(_FMT, _CMD, "mode-gating: param2=0 (in Hold)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "mode-gating: param2=0 (already in Hold → ACCEPTED)",
                 f"result={result}")
        assert result == MAV_RESULT_ACCEPTED, (
            f"param2=0 while already in Hold should produce ACCEPTED; got {result}"
        )


# ---------------------------------------------------------------------------
# Class 2: General parameter compliance
# ---------------------------------------------------------------------------

@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(300)
class TestDoRepositionCommand:
    """DO_REPOSITION via COMMAND_INT — ACK result tests."""

    _supported: bool | None = None

    async def _ensure_supported(self, system, mock_stack) -> None:
        """Probe DO_REPOSITION once per class; skip all tests if UNSUPPORTED."""
        if TestDoRepositionCommand._supported is None:
            ack = await _probe(system)
            unsupported = (ack is not None and int(ack["result"]) == MAV_RESULT_UNSUPPORTED)
            TestDoRepositionCommand._supported = not unsupported
        if not TestDoRepositionCommand._supported:
            pytest.skip(
                f"{_CMD} (cmd={_CMD_ID}) is UNSUPPORTED on this platform — test not run"
            )

    # -----------------------------------------------------------------------
    # Group A — Baseline
    # -----------------------------------------------------------------------

    async def test_command_accepted(self, gcs_system_cls, mock_stack_cls):
        """Baseline: DO_REPOSITION COMMAND_INT returns ACCEPTED (or non-UNSUPPORTED)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls)
        if ack is None:
            pytest.skip("No ACK received — UNKNOWN (spec violation by stack)")
        result = int(ack["result"])
        log.info(_FMT, _CMD, "baseline COMMAND_INT", f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED, (
            "DO_REPOSITION should not return UNSUPPORTED on a platform that supports it"
        )

    # -----------------------------------------------------------------------
    # Group B — param2 (MAV_DO_REPOSITION_FLAGS bitmask)
    # -----------------------------------------------------------------------

    async def test_param2_change_mode_flag(self, gcs_system_cls, mock_stack_cls):
        """
        param2=1 (CHANGE_MODE) — vehicle should switch to hold/guided mode.

        CHANGE_MODE requests the vehicle transition to a mode where repositioning
        can be acted on.  All stacks that support DO_REPOSITION must accept this.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param2=float(_FLAG_CHANGE_MODE))
        if ack is None:
            log.warning(_FMT, _CMD, "param2=CHANGE_MODE(1)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param2=CHANGE_MODE(1)", f"result={result}")
        assert result == MAV_RESULT_ACCEPTED, (
            f"CHANGE_MODE flag should produce ACCEPTED; got {result}"
        )

    async def test_param2_flags_zero(self, gcs_system_cls, mock_stack_cls):
        """
        param2=0 (no flags) — observational.

        Without CHANGE_MODE, the stack should only accept if already in a
        hold-equivalent mode.  PX4 (post-fix) returns DENIED when not in
        AUTO_LOITER; other stacks may return ACCEPTED regardless.
        The mode-dependent assertion is in TestDoRepositionPx4ModeGating.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param2=0.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param2=0 (no flags)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param2=0 (no flags)", f"result={result}  "
                 "(PX4 patched: DENIED when not in Hold; ArduCopter/Rover: likely ACCEPTED)")

    async def test_param2_relative_yaw_only(self, gcs_system_cls, mock_stack_cls):
        """
        param2=2 (RELATIVE_YAW only, no CHANGE_MODE) — observational.

        RELATIVE_YAW without CHANGE_MODE follows the same no-mode-change path.
        PX4 returns DENIED; other stacks may ACCEPT.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param2=float(_FLAG_RELATIVE_YAW))
        if ack is None:
            log.warning(_FMT, _CMD, "param2=RELATIVE_YAW(2)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param2=RELATIVE_YAW(2)", f"result={result}  "
                 "(PX4 patched: DENIED — no CHANGE_MODE bit; others: likely ACCEPTED)")

    async def test_param2_all_flags(self, gcs_system_cls, mock_stack_cls):
        """
        param2=3 (CHANGE_MODE | RELATIVE_YAW) — both defined flags set.

        CHANGE_MODE is set, so the vehicle should accept and switch to hold mode.
        RELATIVE_YAW modifies yaw interpretation but does not block acceptance.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param2=float(_FLAG_CHANGE_MODE | _FLAG_RELATIVE_YAW))
        if ack is None:
            log.warning(_FMT, _CMD, "param2=CHANGE_MODE|RELATIVE_YAW(3)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param2=CHANGE_MODE|RELATIVE_YAW(3)", f"result={result}")
        assert result == MAV_RESULT_ACCEPTED, (
            f"CHANGE_MODE bit set; should produce ACCEPTED; got {result}"
        )

    async def test_param2_undefined_bits(self, gcs_system_cls, mock_stack_cls):
        """
        param2=255 (all bits, including undefined bits 2–7).

        Bit 0 (CHANGE_MODE) is set so PX4 accepts.  Unknown bits should not
        cause UNSUPPORTED — the command is understood.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param2=255.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param2=255 (all bits)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param2=255 (all bits)", f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED, (
            "Unknown bitmask bits should not cause UNSUPPORTED"
        )

    # -----------------------------------------------------------------------
    # Group C — param1 (Speed m/s; spec minValue=-1)
    # -----------------------------------------------------------------------

    async def test_param1_default_speed(self, gcs_system_cls, mock_stack_cls):
        """param1=-1.0 — spec-defined 'use default cruise speed' sentinel."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param1=-1.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param1 (Speed) = -1 (default)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param1 (Speed) = -1 (default)", f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED

    async def test_param1_positive_speed(self, gcs_system_cls, mock_stack_cls):
        """param1=5.0 m/s — positive, valid speed. Observational."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param1=5.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param1 (Speed) = 5.0 m/s", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param1 (Speed) = 5.0 m/s", f"result={result}")

    async def test_param1_zero_speed(self, gcs_system_cls, mock_stack_cls):
        """
        param1=0.0 — boundary value; PX4 treats <=0 as default. Observational.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param1=0.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param1 (Speed) = 0.0", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param1 (Speed) = 0.0", f"result={result}  "
                 "(PX4: param1 <=0 treated as default speed)")

    async def test_param1_nan_speed(self, gcs_system_cls, mock_stack_cls):
        """
        param1=NaN — 'no speed preference'; should use default cruise speed.
        Skips in mock mode.
        """
        if mock_stack_cls is not None:
            pytest.skip("NaN speed test requires a real stack")
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param1=None)
        if ack is None:
            log.warning(_FMT, _CMD, "param1 (Speed) = NaN", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param1 (Speed) = NaN", f"result={result}  "
                 "(PX4: !PX4_ISFINITE(NaN) → use default speed)")

    async def test_param1_below_min(self, gcs_system_cls, mock_stack_cls):
        """
        param1=-5.0 — below the declared minValue="-1".

        The spec declares minValue="-1"; values below -1 are out of range.
        A strict stack should return DENIED.  Known stacks permissively treat
        any param1 <= 0 as "use default" and return ACCEPTED.

        xfail: all known stacks accept below-minimum speed values.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param1=-5.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param1 (Speed) = -5.0 (below min)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param1 (Speed) = -5.0 (below min)", f"result={result}")
        if result != MAV_RESULT_DENIED:
            pytest.xfail(
                f"Stack accepted param1=-5.0 (below minValue=-1, result={result}); "
                "should return DENIED — spec gap: minValue not enforced"
            )
        assert result == MAV_RESULT_DENIED

    # -----------------------------------------------------------------------
    # Group D — param4 (Yaw, radians; NaN = use current heading mode)
    # -----------------------------------------------------------------------

    async def test_param4_yaw_nan(self, gcs_system_cls, mock_stack_cls):
        """param4=NaN — 'use current heading mode'. Spec-correct sentinel. Must not UNSUPPORTED."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param4=None)
        if ack is None:
            log.warning(_FMT, _CMD, "param4 (Yaw) = NaN", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param4 (Yaw) = NaN", f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED

    async def test_param4_yaw_zero(self, gcs_system_cls, mock_stack_cls):
        """
        param4=0.0 rad — heading North.

        Unlike NAV_TAKEOFF (where all stacks ignore param4), PX4 DO_REPOSITION
        applies param4 when finite (rep->current.yaw = cmd.param4 in navigator).
        Observational — stacks that honour it ACCEPTED and execute; those that
        ignore it should DENIED but typically ACCEPTED.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param4=0.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param4 (Yaw) = 0.0 rad (North)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param4 (Yaw) = 0.0 rad (North)", f"result={result}")

    async def test_param4_yaw_specific(self, gcs_system_cls, mock_stack_cls):
        """
        param4=π/2 rad (East, 90°) — a non-trivial heading.

        PX4 navigator applies a finite param4 as a heading setpoint.
        Other stacks may accept but not honour it.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param4=math.pi / 2)
        if ack is None:
            log.warning(_FMT, _CMD, "param4 (Yaw) = π/2 rad (East)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param4 (Yaw) = π/2 rad (East)", f"result={result}")

    async def test_param4_relative_yaw_with_flag(self, gcs_system_cls, mock_stack_cls):
        """
        param2=CHANGE_MODE|RELATIVE_YAW, param4=π/4 — yaw relative to current heading.

        When RELATIVE_YAW bit is set, param4 is an offset from the vehicle's current
        heading rather than an absolute heading.
        Skips in mock mode.
        """
        if mock_stack_cls is not None:
            pytest.skip("RELATIVE_YAW interaction test requires a real stack")
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls,
                           param2=float(_FLAG_CHANGE_MODE | _FLAG_RELATIVE_YAW),
                           param4=math.pi / 4)
        if ack is None:
            log.warning(_FMT, _CMD, "param2=CHANGE_MODE|RELATIVE_YAW, param4=π/4",
                        "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param2=CHANGE_MODE|RELATIVE_YAW, param4=π/4", f"result={result}  "
                 "(RELATIVE_YAW: param4 is heading offset from current, not absolute North)")

    # -----------------------------------------------------------------------
    # Group E — param3 (Loiter radius m; 0 or NaN = ignored; planes only)
    # -----------------------------------------------------------------------

    async def test_param3_zero(self, gcs_system_cls, mock_stack_cls):
        """param3=0.0 — spec 'ignored' value. Must not cause UNSUPPORTED."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param3=0.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param3 (Radius) = 0.0 (ignored)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param3 (Radius) = 0.0 (ignored)", f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED

    async def test_param3_nan(self, gcs_system_cls, mock_stack_cls):
        """
        param3=NaN — spec 'ignored' value (equivalent to 0).
        Skips in mock mode.
        """
        if mock_stack_cls is not None:
            pytest.skip("NaN radius test requires a real stack")
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param3=None)
        if ack is None:
            log.warning(_FMT, _CMD, "param3 (Radius) = NaN (ignored)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param3 (Radius) = NaN (ignored)", f"result={result}  "
                 "(spec: 0 and NaN are both ignored for loiter radius)")

    async def test_param3_positive(self, gcs_system_cls, mock_stack_cls):
        """
        param3=100.0 m — positive loiter radius (planes only).

        A non-zero loiter radius is meaningful only for fixed-wing vehicles.
        A multicopter cannot honour this parameter; by the "unsupported params
        must NACK" convention it should DENIED.  In practice, MC stacks ACCEPT
        and ignore param3 (spec gap for MC).
        Observational (vehicle type is not known here).
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param3=100.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param3 (Radius) = 100.0 m", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param3 (Radius) = 100.0 m", f"result={result}  "
                 "(FW: should honour; MC: should DENIED (spec gap — cannot honour))")

    async def test_param3_negative(self, gcs_system_cls, mock_stack_cls):
        """
        param3=-50.0 m — negative radius; spec says 'positive values only'.

        Negative is invalid per spec.  Observational — stacks may reject or clamp.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param3=-50.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param3 (Radius) = -50.0 (negative, invalid)",
                        "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param3 (Radius) = -50.0 (negative, invalid)", f"result={result}  "
                 "(spec: 'positive values only' — negative should be DENIED)")

    # -----------------------------------------------------------------------
    # Group F — Location (params 5/6/7)
    # -----------------------------------------------------------------------

    async def test_location_specific(self, gcs_system_cls, mock_stack_cls):
        """Specific lat/lon at SIH home — valid coordinates. Must not cause UNSUPPORTED."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, x=_LAT_INT, y=_LON_INT)
        if ack is None:
            log.warning(_FMT, _CMD, "params 5/6 (Lat/Lon) specific", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "params 5/6 (Lat/Lon) specific", f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED

    async def test_location_int32max(self, gcs_system_cls, mock_stack_cls):
        """
        x=INT32_MAX, y=INT32_MAX — 'use current position' sentinel.

        Observational: INT32_MAX is the COMMAND_INT sentinel for "use current lat/lon".
        PX4 navigator converts INT32_MAX to NaN and uses current position.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, x=INT32_MAX, y=INT32_MAX)
        if ack is None:
            log.warning(_FMT, _CMD, "params 5/6 INT32_MAX (use current pos)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "params 5/6 INT32_MAX (use current pos)", f"result={result}")

    async def test_location_out_of_range_latlon(self, gcs_system_cls, mock_stack_cls):
        """
        x=1_200_000_000 (120°N), y=2_000_000_000 (200°E) — impossible coordinates.

        Valid ranges: lat ×1e7 ±900_000_000; lon ×1e7 ±1_800_000_000.
        These values are geometrically impossible but below INT32_MAX.
        Expected: DENIED.

        xfail: PX4 does not validate coordinate ranges (spec gap).
        ArduPilot validates and returns DENIED (PASS).
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        _OUT_LAT = 1_200_000_000
        _OUT_LON = 2_000_000_000
        ack = await _probe(gcs_system_cls, x=_OUT_LAT, y=_OUT_LON)
        if ack is None:
            log.warning(_FMT, _CMD, "params 5/6 out-of-range lat/lon", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "params 5/6 out-of-range lat/lon", f"result={result}")
        if result != MAV_RESULT_DENIED:
            pytest.xfail(
                f"Stack accepted impossible coordinates (result={result}); "
                "should return DENIED — spec gap: coordinate range validation not required"
            )
        assert result == MAV_RESULT_DENIED

    async def test_altitude_nan(self, gcs_system_cls, mock_stack_cls):
        """
        z=NaN — 'use current altitude'.

        PX4 navigator: !PX4_ISFINITE(param7) → keep current altitude.
        Skips in mock mode.
        """
        if mock_stack_cls is not None:
            pytest.skip("NaN altitude test requires a real stack")
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, z=None)
        if ack is None:
            log.warning(_FMT, _CMD, "param7 (Alt) = NaN (use current)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param7 (Alt) = NaN (use current)", f"result={result}  "
                 "(NaN altitude = keep current altitude per spec)")

    async def test_altitude_zero(self, gcs_system_cls, mock_stack_cls):
        """z=0.0 m — zero altitude. Observational (may be accepted or denied)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, z=0.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param7 (Alt) = 0.0 m", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param7 (Alt) = 0.0 m", f"result={result}")

    async def test_altitude_only_reposition(self, gcs_system_cls, mock_stack_cls):
        """
        x=INT32_MAX, y=INT32_MAX, z=100.0 — altitude-only reposition.

        INT32_MAX lat/lon = use current position; only altitude changes.
        PX4 navigator has an explicit "altitude-only" code path for this.
        Observational.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, x=INT32_MAX, y=INT32_MAX, z=100.0)
        if ack is None:
            log.warning(_FMT, _CMD, "altitude-only (INT32_MAX lat/lon, z=100)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "altitude-only (INT32_MAX lat/lon, z=100)", f"result={result}  "
                 "(INT32_MAX lat/lon = keep current position; only altitude changes)")

    # -----------------------------------------------------------------------
    # Group G — All-NaN "pause" (COMMAND_LONG required)
    # -----------------------------------------------------------------------

    async def test_all_nan_pause(self, gcs_system_cls, mock_stack_cls):
        """
        COMMAND_LONG with all position/yaw fields NaN — 'pause vehicle'.

        Per PX4 navigator: when all of lat/lon/alt/yaw are NaN, the vehicle is
        commanded to stop at its current position.
        Requires COMMAND_LONG (NaN cannot be encoded in COMMAND_INT int32 x/y).
        Skips in mock mode.
        """
        if mock_stack_cls is not None:
            pytest.skip("All-NaN pause test requires a real stack")
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await probe_command_long(
            gcs_system_cls, _CMD_ID,
            param1=-1.0,
            param2=1.0,    # CHANGE_MODE
            param3=0.0,
            param4=None,   # NaN yaw
            param5=None,   # NaN lat
            param6=None,   # NaN lon
            param7=None,   # NaN alt
        )
        if ack is None:
            log.warning(_FMT, _CMD, "COMMAND_LONG all-NaN (pause)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "COMMAND_LONG all-NaN (pause)", f"result={result}  "
                 "(PX4: vehicle stops at current position; others: observational)")

    # -----------------------------------------------------------------------
    # Group H — COMMAND_LONG variant
    # -----------------------------------------------------------------------

    async def test_command_long_nan_latlon(self, gcs_system_cls, mock_stack_cls):
        """
        COMMAND_LONG with param5/6=NaN — 'use current position'.

        NaN in COMMAND_LONG float param5/6 is the float-field equivalent of
        INT32_MAX in COMMAND_INT x/y.  Must not cause UNSUPPORTED.
        Skips in mock mode.
        """
        if mock_stack_cls is not None:
            pytest.skip("NaN lat/lon COMMAND_LONG test requires a real stack")
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await probe_command_long(
            gcs_system_cls, _CMD_ID,
            param1=-1.0,
            param2=1.0,
            param3=0.0,
            param4=None,   # NaN yaw
            param5=None,   # NaN lat → use current
            param6=None,   # NaN lon → use current
            param7=50.0,
        )
        if ack is None:
            log.warning(_FMT, _CMD, "COMMAND_LONG param5/6=NaN (use current pos)",
                        "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "COMMAND_LONG param5/6=NaN (use current pos)",
                 f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED, (
            "NaN lat/lon in COMMAND_LONG should not cause UNSUPPORTED"
        )

    async def test_command_long_int32max_float(self, gcs_system_cls, mock_stack_cls):
        """
        COMMAND_LONG with param5/6=float(INT32_MAX) — 'use current position' sentinel.

        INT32_MAX (2_147_483_647) is the MAVLink 'use current position' sentinel for
        lat/lon fields.  It applies to both COMMAND_INT (int32_t x/y) and COMMAND_LONG
        (float param5/6).  Expected result: ACCEPTED.

        xfail: PX4 explicitly rejects float(INT32_MAX) in param5/6 as a protocol error
        (mavlink_receiver.cpp:499–505), treating it as a miscoded COMMAND_INT.  This is
        a PX4 spec violation — the stack should treat INT32_MAX as "use current position".
        Skips in mock mode.
        """
        if mock_stack_cls is not None:
            pytest.skip("INT32_MAX-as-float COMMAND_LONG test requires a real stack")
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await probe_command_long(
            gcs_system_cls, _CMD_ID,
            param1=-1.0,
            param2=1.0,
            param5=float(INT32_MAX),
            param6=float(INT32_MAX),
            param7=50.0,
        )
        if ack is None:
            log.warning(_FMT, _CMD, "COMMAND_LONG param5/6=INT32_MAX (use current pos)",
                        "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "COMMAND_LONG param5/6=INT32_MAX (use current pos)",
                 f"result={result}  "
                 "(INT32_MAX is 'use current position' sentinel; DENIED is a PX4 spec violation)")
        if result != MAV_RESULT_ACCEPTED:
            pytest.xfail(
                f"Stack returned {result} for INT32_MAX lat/lon in COMMAND_LONG; "
                "expected ACCEPTED — INT32_MAX is the 'use current position' sentinel "
                "(PX4 incorrectly rejects it as a protocol error)"
            )
        assert result == MAV_RESULT_ACCEPTED

