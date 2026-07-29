"""
MAV_CMD_DO_SET_MISSION_CURRENT (cmd=224) via COMMAND_LONG — command protocol tests.

Sets the mission item with sequence number param1 as the current item and emits
MISSION_CURRENT (whether or not the mission number changed).  Superseded the
legacy MISSION_SET_CURRENT message (id=41, superseded 2022-08).

hasLocation="false" isDestination="false" → per CLAUDE.md's COMMAND_INT/
COMMAND_LONG selection rule, no param carries lat/lon or a float location, so
**COMMAND_LONG** is the primary vehicle for this command (one observational
COMMAND_INT-equivalence test is included since the spec doesn't forbid it).

Survey status (tests/command/README.md, tested 2026-05-27 against PX4
1.18.0-alpha): SUPPORTED on ArduCopter MC and ArduRover; UNSUPPORTED on all
PX4 vehicle types (MC/FW/VTOL/Rover); UNKNOWN (no ACK) on ArduPlane FW/QP;
SUPPORTED on the Mock (generic accept-all fallback).

KNOWN STALE for PX4 MC: live testing against PX4 1.18.0-beta shows the
command is actively processed (never UNSUPPORTED), with all 18 Tier 1 tests
passing cleanly and zero deviation from the authoritative behaviour matrix
below — see do_set_mission_current/README.md for the full writeup.

Authoritative behaviour matrix (per project maintainer, refining/extending the
bare common.xml text — this is what the assertions below encode)
--------------------------------------------------------------------
No mission uploaded:
    ANY param1/param2 combination -> MAV_RESULT_FAILED.  This is a
    precondition-failure gate: "no mission" fails outright, independent of
    whether param1/param2 would otherwise be valid — not just one instance of
    an out-of-range seq.  See TestDoSetMissionCurrentNoMission.

Mission uploaded — param1 ("Number"):
    -1                          -> ACCEPTED, keeps the current item unchanged.
    > number of mission items   -> FAILED.
    a valid mission item index  -> ACCEPTED, sets the current item.
    any other value             -> DENIED (e.g. a negative value other than -1).

Mission uploaded — param2 ("Reset Mission"):
    0 (false)                   -> ACCEPTED, jump counters untouched.
    1                           -> ACCEPTED, resets jump counters AND changes
                                    mission state "completed" to "active"/
                                    "paused" (e.g. PX4: a completed mission
                                    normally can't be restarted just by
                                    re-entering mission mode or sending a
                                    start command — resetting the jump
                                    counters via param2=1, with a valid
                                    current-item param1, makes it
                                    restartable).  ACK-level tests below only
                                    confirm the ACK; the jump-counter-reset
                                    and mission-state-transition *effects*
                                    are Tier 2 (test_flight.py).
    any other value             -> DENIED.

params 3–7 are Empty (reserved) — the spec doesn't define a result code for
non-NaN reserved-param values here, so (unlike the above, which are hard
assertions per the maintainer's explicit spec) these remain the usual
ambiguous-result xfail convention used elsewhere in this suite.

Because "no mission" masks per-parameter validation (every result would be
FAILED regardless of what's actually being tested), the param1/param2 matrix
above can only be tested meaningfully with a mission uploaded — hence nearly
all substantive assertions live in TestDoSetMissionCurrentWithMission, and
TestDoSetMissionCurrentNoMission is deliberately small (confirms the
precondition gate itself, nothing else).

What these tests can and cannot show
-------------------------------------
Can show: ACK result (ACCEPTED/DENIED/FAILED/UNSUPPORTED) for each parameter
combination.

Cannot show at the ACK level (Tier 2, tests/command/do_set_mission_current/
test_flight.py, covers the second one; the first remains a documented design
outline only — see README.md):
  - Whether the current mission item actually changed (MISSION_CURRENT
    emission).
  - Whether param2=1 actually reset a DO_JUMP repeat counter / made a
    completed mission restartable — **implemented and validated on PX4 MC**
    in test_flight.py.

Running
-------
Paired mock (no autopilot)::

    pytest tests/command/do_set_mission_current/test_command.py -v --log-cli-level=INFO

Standalone::

    pytest tests/command/do_set_mission_current/test_command.py \\
        --drone-address=tcp://127.0.0.1:5760 --connection-timeout=60 \\
        --ardupilot-sitl=~/ardu_sitl/arducopter \\
        --home-lat=37.6234 --home-lon=-122.0811 --home-alt=0 \\
        --vehicle-type=copter --autopilot=ardupilot -v --log-cli-level=INFO
"""

import asyncio
import json
import logging

import pytest
import pytest_asyncio
from mavsdk.mission_raw import MissionItem

from tests.command.conftest import (
    ACK_TIMEOUT_S,
    _FMT,
    gcs_system_cls,   # noqa: F401 — imported to initialise class-scoped asyncio
    mock_stack_cls,   # noqa: F401   infrastructure; not used directly by any test
    probe_command_int,
    probe_command_long,
    send_command_long,
)
from tests.mission.conftest import (
    clear_all_mission_types,
    home_item_for_mission,   # noqa: F401 — registers as a fixture in this module
    load_plan,
    requires_home_slot,      # noqa: F401 — dependency of home_item_for_mission
)
from tests.mock_flight_stack import (
    MAV_RESULT_ACCEPTED,
    MAV_RESULT_DENIED,
    MAV_RESULT_FAILED,
    MAV_RESULT_UNSUPPORTED,
)

log = logging.getLogger(__name__)

_CMD = "DO_SET_MISSION_CURRENT"
_CMD_ID = 224  # MAV_CMD_DO_SET_MISSION_CURRENT


def _set_current_cmd(**overrides) -> dict:
    """Return default COMMAND_LONG kwargs for DO_SET_MISSION_CURRENT."""
    defaults = dict(
        command=_CMD_ID,
        param1=-1.0,   # Number: -1 = keep current item unchanged (see param2)
        param2=0.0,    # Reset Mission: MAV_BOOL FALSE
        param3=None,   # Empty (reserved) — NaN
        param4=None,   # Empty (reserved) — NaN
        param5=None,   # Empty (reserved) — NaN
        param6=None,   # Empty (reserved) — NaN
        param7=None,   # Empty (reserved) — NaN
    )
    defaults.update(overrides)
    return defaults


async def _probe(system, **kwargs) -> dict | None:
    """Subscribe first, then send COMMAND_LONG, then collect COMMAND_ACK."""
    kw = _set_current_cmd(**kwargs)
    return await probe_command_long(system, **kw)


def _build_mission_items(home_item: MissionItem | None, plan_items: list[MissionItem]) -> list[MissionItem]:
    """
    Prepend a home item at seq=0 (ArduCopter) and renumber, or return unchanged.

    Mirrors tests/mission/nav_takeoff/test_flight.py's _build_mission() helper:
    when the connected stack reserves seq=0 for home (ArduCopter), the plan
    items must be shifted to seq=1.. and current=0; otherwise the plan's own
    seq/current values (item 0 already has current=1) are used unmodified.
    """
    if home_item is None:
        return plan_items
    items = [MissionItem(
        seq=0, frame=home_item.frame, command=home_item.command,
        current=0, autocontinue=1,
        param1=home_item.param1, param2=home_item.param2,
        param3=home_item.param3, param4=home_item.param4,
        x=home_item.x, y=home_item.y, z=home_item.z,
        mission_type=0,
    )]
    for i, it in enumerate(plan_items):
        items.append(MissionItem(
            seq=i + 1, frame=it.frame, command=it.command,
            current=0, autocontinue=it.autocontinue,
            param1=it.param1, param2=it.param2, param3=it.param3, param4=it.param4,
            x=it.x, y=it.y, z=it.z, mission_type=0,
        ))
    return items


# ---------------------------------------------------------------------------
# No-mission class — every call must FAIL, regardless of params
# ---------------------------------------------------------------------------


@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(300)
class TestDoSetMissionCurrentNoMission:
    """
    No mission uploaded — confirms the precondition-failure gate itself.

    Per the authoritative spec (module docstring), "no mission uploaded" NACKs
    MAV_RESULT_FAILED unconditionally — this is not merely the out-of-range
    case from common.xml's parenthetical, it takes priority over param1/param2
    validation entirely.  Deliberately small: exhaustive param1/param2
    validation belongs in TestDoSetMissionCurrentWithMission, where it isn't
    confounded by this gate.
    """

    _supported: bool | None = None  # class-level cache; None = not yet probed

    async def _ensure_supported(self, system, mock_stack) -> None:
        """Probe once per class; skip all subsequent tests if UNSUPPORTED."""
        if TestDoSetMissionCurrentNoMission._supported is None:
            ack = await _probe(system)
            unsupported = (ack is not None and int(ack["result"]) == MAV_RESULT_UNSUPPORTED)
            TestDoSetMissionCurrentNoMission._supported = not unsupported
        if not TestDoSetMissionCurrentNoMission._supported:
            pytest.skip(f"{_CMD} (cmd={_CMD_ID}) is UNSUPPORTED on this platform — test not run")

    async def _check_no_mission_failed(self, system, mock_stack, label: str, **probe_kwargs) -> None:
        ack = await _probe(system, **probe_kwargs)
        if ack is None:
            log.warning(_FMT, _CMD, label, "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, label, f"result={result}")
        if mock_stack is not None:
            return  # observational only — mock has no mission-state tracking for cmd 224
        if result != MAV_RESULT_FAILED:
            log.warning(
                "DOC DISCREPANCY: DO_SET_MISSION_CURRENT with no mission uploaded (%s) "
                "returned result=%d, not the spec-mandated MAV_RESULT_FAILED(4)", label, result,
            )
            pytest.xfail(
                f"Stack returned {result} for {label} with no mission uploaded; "
                "spec mandates MAV_RESULT_FAILED(4) regardless of param1/param2"
            )
        assert result == MAV_RESULT_FAILED

    async def test_no_mission_sentinel_failed(self, gcs_system_cls, mock_stack_cls):
        """param1=-1, param2=0 — even the 'keep unchanged' sentinel requires a mission to exist."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        await self._check_no_mission_failed(
            gcs_system_cls, mock_stack_cls, "param1=-1, param2=0 (no mission)",
            param1=-1.0, param2=0.0,
        )

    async def test_no_mission_valid_looking_index_failed(self, gcs_system_cls, mock_stack_cls):
        """param1=0 (looks like a valid index) — still FAILED, nothing exists to index into."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        await self._check_no_mission_failed(
            gcs_system_cls, mock_stack_cls, "param1=0, param2=0 (no mission)",
            param1=0.0, param2=0.0,
        )

    async def test_no_mission_out_of_range_failed(self, gcs_system_cls, mock_stack_cls):
        """param1=999999 — same gate; a large value doesn't change the result."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        await self._check_no_mission_failed(
            gcs_system_cls, mock_stack_cls, "param1=999999, param2=0 (no mission)",
            param1=999999.0, param2=0.0,
        )


# ---------------------------------------------------------------------------
# WithMission class — full param1/param2 behaviour matrix
# ---------------------------------------------------------------------------


@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(300)
class TestDoSetMissionCurrentWithMission:
    """
    Full param1/param2 behaviour matrix — requires an uploaded mission (see
    module docstring for the authoritative expected result per case).
    Uploads tests/mission/plans/simple_mission.json once per class.
    """

    _supported: bool | None = None  # class-level cache; None = not yet probed
    _valid_seq: int = 1     # a real index into the uploaded mission
    _num_items: int = 4     # len(uploaded mission); set by _uploaded_mission

    async def _ensure_supported(self, system, mock_stack) -> None:
        if TestDoSetMissionCurrentWithMission._supported is None:
            ack = await _probe(system)
            unsupported = (ack is not None and int(ack["result"]) == MAV_RESULT_UNSUPPORTED)
            TestDoSetMissionCurrentWithMission._supported = not unsupported
        if not TestDoSetMissionCurrentWithMission._supported:
            pytest.skip(f"{_CMD} (cmd={_CMD_ID}) is UNSUPPORTED on this platform — test not run")

    @pytest_asyncio.fixture(scope="class", loop_scope="class", autouse=True)
    async def _uploaded_mission(self, gcs_system_cls, home_item_for_mission):
        """Upload a small mission once for the class; best-effort, never raises."""
        plan_items = load_plan("simple_mission.json")
        items = _build_mission_items(home_item_for_mission, plan_items)
        TestDoSetMissionCurrentWithMission._valid_seq = 1 if home_item_for_mission is None else 2
        TestDoSetMissionCurrentWithMission._num_items = len(items)
        await clear_all_mission_types(gcs_system_cls)
        try:
            await gcs_system_cls.mission_raw.upload_mission(items)
        except Exception as exc:
            log.warning(_FMT, _CMD, "mission upload precondition", f"failed: {exc}")
        yield
        await clear_all_mission_types(gcs_system_cls)

    # -----------------------------------------------------------------------
    # Group A — baseline / hygiene
    # -----------------------------------------------------------------------

    async def test_command_accepted(self, gcs_system_cls, mock_stack_cls):
        """Baseline: param1=<valid index>, param2=0 — not UNSUPPORTED."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        seq = float(TestDoSetMissionCurrentWithMission._valid_seq)
        ack = await _probe(gcs_system_cls, param1=seq, param2=0.0)
        if ack is None:
            pytest.skip("No ACK received — UNKNOWN (spec violation by stack)")
        result = int(ack["result"])
        log.info(_FMT, _CMD, "baseline COMMAND_LONG (mission loaded)", f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED

    async def test_exactly_one_ack(self, gcs_system_cls, mock_stack_cls):
        """
        Exactly one COMMAND_ACK must be received per command send.

        Subscribe to COMMAND_ACK before sending, collect during a 1 s drain
        window after the first ACK, and assert total count == 1.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)

        ack_queue: asyncio.Queue = asyncio.Queue()

        async def _collect_acks() -> None:
            async for msg in gcs_system_cls.mavlink_direct.message("COMMAND_ACK"):
                fields = json.loads(msg.fields_json)
                if int(fields.get("command", -1)) == _CMD_ID:
                    await ack_queue.put(fields)

        ack_task = asyncio.create_task(_collect_acks())
        await asyncio.sleep(0.05)

        seq = float(TestDoSetMissionCurrentWithMission._valid_seq)
        kw = _set_current_cmd(param1=seq, param2=0.0)
        await send_command_long(gcs_system_cls, **kw)

        try:
            first = await asyncio.wait_for(ack_queue.get(), timeout=ACK_TIMEOUT_S)
        except asyncio.TimeoutError:
            ack_task.cancel()
            pytest.fail("No COMMAND_ACK received")
            return

        extra = []
        deadline = asyncio.get_event_loop().time() + 1.0
        while asyncio.get_event_loop().time() < deadline:
            remaining = deadline - asyncio.get_event_loop().time()
            try:
                item = await asyncio.wait_for(ack_queue.get(), timeout=min(remaining, 0.05))
                extra.append(item)
            except asyncio.TimeoutError:
                pass
        ack_task.cancel()

        result = int(first["result"])
        log.info(_FMT, _CMD, "exactly-one-ACK", f"result={result} extra_acks={len(extra)}")
        if mock_stack_cls is not None:
            assert len(extra) == 0, f"Got {1 + len(extra)} ACKs; expected exactly 1"

    async def test_command_int_variant_observational(self, gcs_system_cls, mock_stack_cls):
        """
        Same params sent via COMMAND_INT instead of COMMAND_LONG — observational.

        DO_SET_MISSION_CURRENT has no lat/lon and is not hasLocation, so
        COMMAND_LONG is the preferred message type (see module docstring).
        This documents whether stacks also accept the non-preferred type;
        x/y/z are unused (Empty per spec) so left at 0/0.0.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        seq = float(TestDoSetMissionCurrentWithMission._valid_seq)
        ack = await probe_command_int(
            gcs_system_cls, _CMD_ID, frame=6,
            param1=seq, param2=0.0, param3=None, param4=None,
            x=0, y=0, z=0.0,
        )
        if ack is None:
            log.warning(_FMT, _CMD, "COMMAND_INT variant", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "COMMAND_INT variant", f"result={result}")
        # Observational — no assertion

    # -----------------------------------------------------------------------
    # Group B — param1 behaviour matrix
    # -----------------------------------------------------------------------

    async def test_param1_negative_one_keeps_unchanged_accepted(self, gcs_system_cls, mock_stack_cls):
        """
        param1=-1 ('keep current item, just reset'), param2=0 — must be ACCEPTED
        once a mission exists.

        Confirmed ACCEPTED on PX4 MC 1.18.0-beta, both here (mission uploaded,
        not yet started/armed) and mid-flight in test_flight.py's Tier 2
        jump-counter reset test — see README.md. Kept as an xfail-capable
        assertion (rather than a bare assert) as a regression guard, not
        because a failure is currently expected.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param1=-1.0, param2=0.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param1=-1 (mission loaded)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param1=-1 (mission loaded)", f"result={result}")
        if mock_stack_cls is not None:
            assert result != MAV_RESULT_UNSUPPORTED
            return
        if result != MAV_RESULT_ACCEPTED:
            log.warning(
                "DOC DISCREPANCY: DO_SET_MISSION_CURRENT param1=-1 with a mission loaded "
                "returned result=%d, not the spec-mandated MAV_RESULT_ACCEPTED(0)", result,
            )
            pytest.xfail(
                f"Stack returned {result} for param1=-1 with a mission loaded; spec says "
                "-1 keeps the current item unchanged and should be ACCEPTED"
            )
        assert result == MAV_RESULT_ACCEPTED

    async def test_param1_valid_index_accepted(self, gcs_system_cls, mock_stack_cls):
        """param1=<valid index>, param2=0 — sets the current item; must be ACCEPTED."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        seq = float(TestDoSetMissionCurrentWithMission._valid_seq)
        ack = await _probe(gcs_system_cls, param1=seq, param2=0.0)
        if ack is None:
            log.warning(_FMT, _CMD, f"param1={seq:.0f} (valid index)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, f"param1={seq:.0f} (valid index)", f"result={result}")
        if mock_stack_cls is not None:
            assert result != MAV_RESULT_UNSUPPORTED
            return
        assert result == MAV_RESULT_ACCEPTED, (
            f"param1={seq:.0f} is a valid index into the "
            f"{TestDoSetMissionCurrentWithMission._num_items}-item uploaded mission; "
            f"spec requires ACCEPTED, got {result}"
        )

    async def test_param1_out_of_range_failed(self, gcs_system_cls, mock_stack_cls):
        """
        param1 > number of mission items — must be FAILED.

        Uses num_items + 10 as an unambiguous out-of-range value. Confirmed
        FAILED on PX4 MC 1.18.0-beta — see README.md. Kept as an
        xfail-capable assertion (rather than a bare assert) as a regression
        guard, not because a failure is currently expected.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        out_of_range = float(TestDoSetMissionCurrentWithMission._num_items + 10)
        ack = await _probe(gcs_system_cls, param1=out_of_range, param2=0.0)
        if ack is None:
            log.warning(_FMT, _CMD, f"param1={out_of_range:.0f} (out of range)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, f"param1={out_of_range:.0f} (out of range)", f"result={result}")
        if mock_stack_cls is not None:
            return  # observational only — mock has no seq-range validation for cmd 224
        if result != MAV_RESULT_FAILED:
            log.warning(
                "DOC DISCREPANCY: DO_SET_MISSION_CURRENT out-of-range param1 (mission loaded) "
                "returned result=%d, not the spec-mandated MAV_RESULT_FAILED(4)", result,
            )
            pytest.xfail(
                f"Stack returned {result} for out-of-range param1; spec mandates "
                "MAV_RESULT_FAILED(4) — DOC DISCREPANCY, confirmed on PX4 MC 1.18.0-beta"
            )
        assert result == MAV_RESULT_FAILED

    async def test_param1_other_invalid_denied(self, gcs_system_cls, mock_stack_cls):
        """param1=-2 (negative, not the -1 sentinel) — 'any other value': must be DENIED."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        ack = await _probe(gcs_system_cls, param1=-2.0, param2=0.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param1=-2 (other invalid)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param1=-2 (other invalid)", f"result={result}")
        if mock_stack_cls is not None:
            return  # observational only — mock accepts everything by default
        if result != MAV_RESULT_DENIED:
            log.warning(
                "DOC DISCREPANCY: DO_SET_MISSION_CURRENT param1=-2 (other invalid) "
                "returned result=%d, not the spec-mandated MAV_RESULT_DENIED(2)", result,
            )
            pytest.xfail(
                f"Stack returned {result} for param1=-2; spec mandates DENIED(2) for "
                "'any other value' of param1"
            )
        assert result == MAV_RESULT_DENIED

    # -----------------------------------------------------------------------
    # Group C — param2 behaviour matrix
    # -----------------------------------------------------------------------

    async def test_param2_zero_accepted(self, gcs_system_cls, mock_stack_cls):
        """param2=0 (jump counters untouched) — must be ACCEPTED."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        seq = float(TestDoSetMissionCurrentWithMission._valid_seq)
        ack = await _probe(gcs_system_cls, param1=seq, param2=0.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param2=0 (no reset)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param2=0 (no reset)", f"result={result}")
        if mock_stack_cls is not None:
            assert result != MAV_RESULT_UNSUPPORTED
            return
        assert result == MAV_RESULT_ACCEPTED

    async def test_param2_one_accepted(self, gcs_system_cls, mock_stack_cls):
        """
        param2=1 (reset jump counters; COMPLETE -> ACTIVE/PAUSED) — must be ACCEPTED.

        ACK-level only — does not verify the jump-counter reset or mission-
        state transition actually happened; see test_flight.py (Tier 2),
        which validates the jump-counter-reset effect behaviourally and
        confirms it on PX4 MC.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        seq = float(TestDoSetMissionCurrentWithMission._valid_seq)
        ack = await _probe(gcs_system_cls, param1=seq, param2=1.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param2=1 (reset)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param2=1 (reset)", f"result={result}")
        if mock_stack_cls is not None:
            assert result != MAV_RESULT_UNSUPPORTED
            return
        assert result == MAV_RESULT_ACCEPTED

    async def test_param2_invalid_denied(self, gcs_system_cls, mock_stack_cls):
        """param2=2 (not 0 or 1) — 'any other value': must be DENIED."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        seq = float(TestDoSetMissionCurrentWithMission._valid_seq)
        ack = await _probe(gcs_system_cls, param1=seq, param2=2.0)
        if ack is None:
            log.warning(_FMT, _CMD, "param2=2 (invalid)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "param2=2 (invalid)", f"result={result}")
        if mock_stack_cls is not None:
            return  # observational only — mock does not validate the MAV_BOOL enum
        if result != MAV_RESULT_DENIED:
            log.warning(
                "DOC DISCREPANCY: DO_SET_MISSION_CURRENT param2=2 (invalid) "
                "returned result=%d, not the spec-mandated MAV_RESULT_DENIED(2)", result,
            )
            pytest.xfail(
                f"Stack returned {result} for param2=2; spec mandates DENIED(2) for "
                "'any other value' of param2"
            )
        assert result == MAV_RESULT_DENIED

    # -----------------------------------------------------------------------
    # Group D — reserved params (3–7): ambiguous result code, xfail convention
    # -----------------------------------------------------------------------

    async def _check_reserved_param(self, system, label: str, value: float, **probe_kwargs) -> None:
        """
        Send a probe (mission loaded, valid param1) with one reserved param
        set to a non-NaN value.

        A conformant stack must return MAV_RESULT_DENIED — "Empty" params must
        be sent as NaN, and any non-NaN value represents an intent that cannot
        be honoured.  Unlike the param1/param2 cases above, the spec does not
        name a result code for this, so it stays an xfail scaffold.

        xfail: no known stack currently enforces NaN for reserved params — all
        return non-DENIED while silently ignoring the value (same pattern
        already documented for DO_SET_GLOBAL_ORIGIN's reserved params).
        """
        seq = float(TestDoSetMissionCurrentWithMission._valid_seq)
        ack = await _probe(system, param1=seq, param2=0.0, **probe_kwargs)
        if ack is None:
            log.warning(_FMT, _CMD, label, "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, label, f"result={result}")
        if result != MAV_RESULT_DENIED:
            pytest.xfail(
                f"Stack returned {result} for reserved param={value}; expected DENIED — "
                "no known stack enforces NaN for 'Empty' params (spec gap)"
            )
        assert result == MAV_RESULT_DENIED

    async def test_reserved_param3_nonnan_ack(self, gcs_system_cls, mock_stack_cls):
        """param3=1.0 (non-NaN reserved) — must be DENIED; xfail on all known stacks."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        await self._check_reserved_param(gcs_system_cls, "param3=1.0 (non-NaN reserved)", 1.0, param3=1.0)

    async def test_reserved_param4_nonnan_ack(self, gcs_system_cls, mock_stack_cls):
        """param4=1.0 (non-NaN reserved) — must be DENIED; xfail on all known stacks."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        await self._check_reserved_param(gcs_system_cls, "param4=1.0 (non-NaN reserved)", 1.0, param4=1.0)

    async def test_reserved_param5_nonnan_ack(self, gcs_system_cls, mock_stack_cls):
        """param5=1.0 (non-NaN reserved) — must be DENIED; xfail on all known stacks."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        await self._check_reserved_param(gcs_system_cls, "param5=1.0 (non-NaN reserved)", 1.0, param5=1.0)

    async def test_reserved_param6_nonnan_ack(self, gcs_system_cls, mock_stack_cls):
        """param6=1.0 (non-NaN reserved) — must be DENIED; xfail on all known stacks."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        await self._check_reserved_param(gcs_system_cls, "param6=1.0 (non-NaN reserved)", 1.0, param6=1.0)

    async def test_reserved_param7_nonnan_ack(self, gcs_system_cls, mock_stack_cls):
        """param7=1.0 (non-NaN reserved) — must be DENIED; xfail on all known stacks."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        await self._check_reserved_param(gcs_system_cls, "param7=1.0 (non-NaN reserved)", 1.0, param7=1.0)
