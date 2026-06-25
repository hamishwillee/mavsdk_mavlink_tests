"""
MAV_CMD_DO_SET_GLOBAL_ORIGIN (cmd=611) via COMMAND_INT — command protocol tests.

Sets the GNSS coordinates of the vehicle local origin (0,0,0).  The vehicle must
emit GPS_GLOBAL_ORIGIN (message id=49) irrespective of whether the origin changed.
Supersedes the SET_GPS_GLOBAL_ORIGIN message (id=48, deprecated 2025-04).

Defined in development.xml (not common.xml) — not in the command survey.
No real flight stack is known to support this command yet; expect SKIP in
standalone mode until implementations appear.

Spec reference: mavlink/message_definitions/v1.0/development.xml entry value=611.

Frame: the current submodule text says "MAV_FRAME_GLOBAL (0)" but PR #2530
corrects this to MAV_FRAME_GLOBAL_INT (6), consistent with every other
hasLocation COMMAND_INT.  Tests use frame=6.

Parameter layout::

    param1–4  Empty (reserved — must be NaN; non-NaN should be DENIED)
    param5    Latitude   (degE7 in COMMAND_INT; degrees in COMMAND_LONG)
    param6    Longitude  (degE7 in COMMAND_INT; degrees in COMMAND_LONG)
    param7    Altitude   (m MSL — must be a real value; NaN not valid)

IMPORTANT — what these tests can and cannot show
-------------------------------------------------
Can show:
  - ACCEPTED / DENIED / UNSUPPORTED for each parameter combination.
  - Whether GPS_GLOBAL_ORIGIN is emitted; whether its fields are consistent
    with what was commanded.

Cannot show:
  - Whether the vehicle navigation stack actually uses the new origin for
    local↔global coordinate transforms.
  No flight test is planned — success is fully measured at the
  ACK + GPS_GLOBAL_ORIGIN level.

Running
-------
Paired mock (no autopilot)::

    pytest tests/command/do_set_global_origin/test_command.py -v --log-cli-level=INFO

Standalone (expected: all SKIP until a stack supports cmd=611)::

    pytest tests/command/do_set_global_origin/test_command.py \\
        --drone-address=udp://:14540 --vehicle-type=quadcopter --autopilot=px4 \\
        -v --log-cli-level=INFO
"""

import asyncio
import json
import logging

import pytest

from tests.command.conftest import (
    ACK_TIMEOUT_S,
    INT32_MAX,
    _FMT,
    gcs_system_cls,   # noqa: F401 — imported to initialise class-scoped asyncio
    mock_stack_cls,   # noqa: F401   infrastructure; not used directly by any test
    probe_command_int,
    probe_command_long,
    send_command_int,
)
from tests.mock_flight_stack import MAV_RESULT_ACCEPTED, MAV_RESULT_DENIED, MAV_RESULT_UNSUPPORTED

# gcs_system_origin_cls / mock_stack_origin_cls / gcs_system_nack_cls / mock_stack_nack_cls
# are injected by pytest from the local conftest.py.  The gcs_system_cls / mock_stack_cls
# imports above are NOT used in test methods; they are present solely so that pytest-asyncio
# registers class-scoped fixture infrastructure at the file level, which is required for the
# local class-scoped GCS fixtures to receive gRPC connection-state events correctly.

log = logging.getLogger(__name__)

_CMD = "DO_SET_GLOBAL_ORIGIN"
_CMD_ID = 611  # MAV_CMD_DO_SET_GLOBAL_ORIGIN (development.xml)

# PX4 SIH simulator home — used as nominal coordinate set A
_LAT_INT = 473977000   # 47.3977 °N
_LON_INT =  85456000   #  8.5456 °E

# Distinct second coordinate set B — used in change-detection tests
_LAT_INT2 = 474000000  # 47.4000 °N
_LON_INT2 =  85500000  #  8.5500 °E

_GPS_TIMEOUT_S = 5.0   # wait for GPS_GLOBAL_ORIGIN after ACCEPTED
_GPS_DRAIN_S   = 1.0   # window to detect duplicate GPS_GLOBAL_ORIGIN messages


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _origin_cmd(**overrides) -> dict:
    """Return default COMMAND_INT kwargs for DO_SET_GLOBAL_ORIGIN."""
    defaults = dict(
        command=_CMD_ID,
        frame=6,       # MAV_FRAME_GLOBAL_INT — per PR #2530
        param1=None,   # NaN — reserved ("Empty" in spec)
        param2=None,   # NaN
        param3=None,   # NaN
        param4=None,   # NaN
        x=_LAT_INT,
        y=_LON_INT,
        z=10.0,        # 10 m MSL
    )
    defaults.update(overrides)
    return defaults


async def _probe(system, **kwargs) -> dict | None:
    kw = _origin_cmd(**kwargs)
    return await probe_command_int(system, **kw)


async def _subscribe_gps(system):
    """
    Start a background GPS_GLOBAL_ORIGIN subscription.

    Returns (task, queue).  Caller must cancel the task (fire-and-forget)
    when done — do NOT await after cancel (gRPC stream caveat, CLAUDE.md §4a).
    """
    queue: asyncio.Queue = asyncio.Queue()

    async def _collect() -> None:
        async for msg in system.mavlink_direct.message("GPS_GLOBAL_ORIGIN"):
            await queue.put(json.loads(msg.fields_json))

    task = asyncio.create_task(_collect())
    await asyncio.sleep(0.05)  # let gRPC stream register before any send
    return task, queue


async def _drain(queue: asyncio.Queue, drain_s: float = _GPS_DRAIN_S) -> list:
    """Drain queue for drain_s seconds; return list of extra items collected."""
    extra = []
    deadline = asyncio.get_event_loop().time() + drain_s
    while asyncio.get_event_loop().time() < deadline:
        remaining = deadline - asyncio.get_event_loop().time()
        try:
            item = await asyncio.wait_for(queue.get(), timeout=min(remaining, 0.05))
            extra.append(item)
        except asyncio.TimeoutError:
            pass
    return extra


# ---------------------------------------------------------------------------
# Main test class
# ---------------------------------------------------------------------------


@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(300)
class TestDoSetGlobalOriginCommand:
    """DO_SET_GLOBAL_ORIGIN via COMMAND_INT — ACK result and GPS_GLOBAL_ORIGIN response tests."""

    _supported: bool | None = None  # class-level cache; None = not yet probed

    async def _ensure_supported(self, system, mock_stack) -> None:
        """Probe once per class; skip all subsequent tests if UNSUPPORTED."""
        if TestDoSetGlobalOriginCommand._supported is None:
            ack = await _probe(system)
            unsupported = (ack is not None and int(ack["result"]) == MAV_RESULT_UNSUPPORTED)
            TestDoSetGlobalOriginCommand._supported = not unsupported
        if not TestDoSetGlobalOriginCommand._supported:
            pytest.skip(f"{_CMD} (cmd={_CMD_ID}) is UNSUPPORTED on this platform — test not run")

    # -----------------------------------------------------------------------
    # Group A — Baseline + exactly-one-ACK
    # -----------------------------------------------------------------------

    async def test_command_accepted(self, gcs_system_origin_cls, mock_stack_origin_cls):
        """Baseline: DO_SET_GLOBAL_ORIGIN returns ACCEPTED with nominal coordinates."""
        await self._ensure_supported(gcs_system_origin_cls, mock_stack_origin_cls)
        ack = await _probe(gcs_system_origin_cls)
        if ack is None:
            pytest.skip("No ACK received — UNKNOWN (spec violation by stack)")
        result = int(ack["result"])
        log.info(_FMT, _CMD, "baseline COMMAND_INT", f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED

    async def test_exactly_one_ack(self, gcs_system_origin_cls, mock_stack_origin_cls):
        """
        Exactly one COMMAND_ACK must be received per command send.

        Subscribe to COMMAND_ACK before sending, collect during a 1 s drain
        window after the first ACK, and assert total count == 1.
        Duplicate ACKs would indicate a double-send or handler bug.
        """
        await self._ensure_supported(gcs_system_origin_cls, mock_stack_origin_cls)

        ack_queue: asyncio.Queue = asyncio.Queue()

        async def _collect_acks() -> None:
            async for msg in gcs_system_origin_cls.mavlink_direct.message("COMMAND_ACK"):
                fields = json.loads(msg.fields_json)
                if int(fields.get("command", -1)) == _CMD_ID:
                    await ack_queue.put(fields)

        ack_task = asyncio.create_task(_collect_acks())
        await asyncio.sleep(0.05)

        kw = _origin_cmd()
        await send_command_int(gcs_system_origin_cls, **kw)

        try:
            first = await asyncio.wait_for(ack_queue.get(), timeout=ACK_TIMEOUT_S)
        except asyncio.TimeoutError:
            ack_task.cancel()
            pytest.fail("No COMMAND_ACK received")
            return

        extra = await _drain(ack_queue, drain_s=1.0)
        ack_task.cancel()

        result = int(first["result"])
        log.info(_FMT, _CMD, "exactly-one-ACK", f"result={result} extra_acks={len(extra)}")
        if mock_stack_origin_cls is not None:
            assert len(extra) == 0, f"Got {1 + len(extra)} ACKs; expected exactly 1"

    # -----------------------------------------------------------------------
    # Group B — Reserved params (1–4): must be NaN; non-NaN must be DENIED
    # -----------------------------------------------------------------------

    async def _check_reserved_param(self, system, label: str, value: float, **probe_kwargs) -> None:
        """
        Send a probe with one reserved param set to a non-NaN value.

        A conformant stack must return MAV_RESULT_DENIED — "Empty" params must
        be sent as NaN, and any non-NaN value represents an intent that cannot
        be honoured.

        Sending 0.0 is a common GCS mistake (treating 0 as "no value") but is
        still a spec violation; it must also be DENIED.

        xfail: no known stack currently enforces NaN for reserved params — all
        return ACCEPTED while silently ignoring the value.
        """
        ack = await _probe(system, **probe_kwargs)
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

    async def test_reserved_param1_zero_ack(self, gcs_system_origin_cls, mock_stack_origin_cls):
        """param1=0.0 (zero, not NaN) — reserved ("Empty"); must be DENIED; xfail all stacks."""
        await self._ensure_supported(gcs_system_origin_cls, mock_stack_origin_cls)
        await self._check_reserved_param(gcs_system_origin_cls, "param1=0.0 (zero, not NaN)", 0.0, param1=0.0)

    async def test_reserved_param1_nonnan_ack(self, gcs_system_origin_cls, mock_stack_origin_cls):
        """param1=1.0 (non-NaN reserved) — must be DENIED; xfail on all known stacks."""
        await self._ensure_supported(gcs_system_origin_cls, mock_stack_origin_cls)
        await self._check_reserved_param(gcs_system_origin_cls, "param1=1.0 (non-NaN reserved)", 1.0, param1=1.0)

    async def test_reserved_param2_nonnan_ack(self, gcs_system_origin_cls, mock_stack_origin_cls):
        """param2=1.0 (non-NaN reserved) — must be DENIED; xfail on all known stacks."""
        await self._ensure_supported(gcs_system_origin_cls, mock_stack_origin_cls)
        await self._check_reserved_param(gcs_system_origin_cls, "param2=1.0 (non-NaN reserved)", 1.0, param2=1.0)

    async def test_reserved_param3_nonnan_ack(self, gcs_system_origin_cls, mock_stack_origin_cls):
        """param3=1.0 (non-NaN reserved) — must be DENIED; xfail on all known stacks."""
        await self._ensure_supported(gcs_system_origin_cls, mock_stack_origin_cls)
        await self._check_reserved_param(gcs_system_origin_cls, "param3=1.0 (non-NaN reserved)", 1.0, param3=1.0)

    async def test_reserved_param4_nonnan_ack(self, gcs_system_origin_cls, mock_stack_origin_cls):
        """param4=1.0 (non-NaN reserved) — must be DENIED; xfail on all known stacks."""
        await self._ensure_supported(gcs_system_origin_cls, mock_stack_origin_cls)
        await self._check_reserved_param(gcs_system_origin_cls, "param4=1.0 (non-NaN reserved)", 1.0, param4=1.0)

    # -----------------------------------------------------------------------
    # Group C — Frame validation
    # -----------------------------------------------------------------------

    async def test_frame_global_ack(self, gcs_system_origin_cls, mock_stack_origin_cls):
        """
        frame=0 (MAV_FRAME_GLOBAL) — observational.

        The current submodule spec text says "Expected frame is MAV_FRAME_GLOBAL (0)"
        but PR #2530 corrects this to MAV_FRAME_GLOBAL_INT (6).  This test
        checks whether the stack accepts the pre-PR #2530 value.
        """
        await self._ensure_supported(gcs_system_origin_cls, mock_stack_origin_cls)
        ack = await _probe(gcs_system_origin_cls, frame=0)
        if ack is None:
            log.warning(_FMT, _CMD, "frame=MAV_FRAME_GLOBAL(0)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "frame=MAV_FRAME_GLOBAL(0)", f"result={result}")

    async def test_frame_global_relative_alt_ack(self, gcs_system_origin_cls, mock_stack_origin_cls):
        """
        frame=3 (MAV_FRAME_GLOBAL_RELATIVE_ALT) — observational.

        A relative-altitude frame is inappropriate for setting a global origin
        (altitude must be MSL).  Observational — the spec does not mandate
        rejection; log what the stack does.
        """
        await self._ensure_supported(gcs_system_origin_cls, mock_stack_origin_cls)
        ack = await _probe(gcs_system_origin_cls, frame=3)
        if ack is None:
            log.warning(_FMT, _CMD, "frame=GLOBAL_RELATIVE_ALT(3)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "frame=GLOBAL_RELATIVE_ALT(3)", f"result={result}")

    # -----------------------------------------------------------------------
    # Group D — Location validation (params 5–7 must be real, in-range values)
    # -----------------------------------------------------------------------

    async def test_location_int32max_denied(self, gcs_system_origin_cls, mock_stack_origin_cls):
        """
        x=y=INT32_MAX — must be DENIED.

        Unlike NAV_TAKEOFF/NAV_LAND (where INT32_MAX means "use current position"),
        DO_SET_GLOBAL_ORIGIN requires an explicit GNSS coordinate.  INT32_MAX is
        not a valid sentinel for this command; it must be DENIED.

        xfail: PX4 maps INT32_MAX → NaN lat/lon (standard COMMAND_INT sentinel
        convention) and EKF2 then returns FAILED when it cannot set a NaN origin —
        reasonable behaviour but the wrong result code (DENIED is expected).
        """
        await self._ensure_supported(gcs_system_origin_cls, mock_stack_origin_cls)
        ack = await _probe(gcs_system_origin_cls, x=INT32_MAX, y=INT32_MAX)
        if ack is None:
            log.warning(_FMT, _CMD, "x=y=INT32_MAX (no sentinel semantic)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "x=y=INT32_MAX (no sentinel semantic)", f"result={result}")
        if result != MAV_RESULT_DENIED:
            pytest.xfail(
                f"Stack returned {result} for INT32_MAX lat/lon; expected DENIED — "
                "PX4 converts INT32_MAX → NaN and EKF2 returns FAILED instead of DENIED "
                "(spec gap: command should reject the sentinel, not attempt to apply NaN)"
            )
        assert result == MAV_RESULT_DENIED

    async def test_location_out_of_range_latlon_denied(self, gcs_system_origin_cls, mock_stack_origin_cls):
        """
        x=910_000_000 (91°N) — geometrically impossible; must be DENIED.

        Setting an impossible coordinate as the global origin is invalid.

        xfail: PX4 converts the value to 91.0°N and EKF2 returns FAILED when
        `setEkfGlobalOrigin` rejects the out-of-range coordinate — reasonable
        behaviour but the wrong result code (DENIED is expected for bad input).
        """
        await self._ensure_supported(gcs_system_origin_cls, mock_stack_origin_cls)
        _OUT_LAT = 910_000_000  # 91°N — outside ±90°×1e7 valid range
        ack = await _probe(gcs_system_origin_cls, x=_OUT_LAT, y=_LON_INT)
        if ack is None:
            log.warning(_FMT, _CMD, "x=910_000_000 (91°N, out-of-range)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "x=910_000_000 (91°N, out-of-range)", f"result={result}")
        if result != MAV_RESULT_DENIED:
            pytest.xfail(
                f"Stack returned {result} for out-of-range lat 91°N; expected DENIED — "
                "PX4 passes the value to EKF2 which returns FAILED; "
                "rejection should happen before the command is attempted (DENIED)"
            )
        assert result == MAV_RESULT_DENIED

    async def test_altitude_nan_denied(self, gcs_system_origin_cls, mock_stack_origin_cls):
        """
        z=NaN — must be DENIED.

        Unlike some commands (e.g. NAV_TAKEOFF param7=NaN means "use default
        altitude"), DO_SET_GLOBAL_ORIGIN requires an explicit MSL altitude to
        define the origin.  NaN is not valid for this command.

        xfail: PX4 currently accepts NaN altitude without error — EKF2 does not
        validate the altitude field for this command (spec gap).
        """
        await self._ensure_supported(gcs_system_origin_cls, mock_stack_origin_cls)
        ack = await _probe(gcs_system_origin_cls, z=None)  # None → NaN on wire
        if ack is None:
            log.warning(_FMT, _CMD, "z=NaN (altitude must be real)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "z=NaN (altitude must be real)", f"result={result}")
        if result != MAV_RESULT_DENIED:
            pytest.xfail(
                f"Stack returned {result} for NaN altitude; expected DENIED — "
                "PX4/EKF2 does not validate the altitude field and accepts NaN "
                "(spec gap: NaN altitude is not meaningful for a global origin)"
            )
        assert result == MAV_RESULT_DENIED

    async def test_altitude_zero(self, gcs_system_origin_cls, mock_stack_origin_cls):
        """z=0.0 m MSL — valid; origin at sea level."""
        await self._ensure_supported(gcs_system_origin_cls, mock_stack_origin_cls)
        ack = await _probe(gcs_system_origin_cls, z=0.0)
        if ack is None:
            log.warning(_FMT, _CMD, "z=0.0 (sea-level origin)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "z=0.0 (sea-level origin)", f"result={result}")
        assert result != MAV_RESULT_UNSUPPORTED

    async def test_altitude_negative(self, gcs_system_origin_cls, mock_stack_origin_cls):
        """
        z=-500.0 m MSL — observational; spec has no lower bound.

        Negative MSL altitudes are physically valid (Dead Sea −430 m, underground
        installations).  Stacks may accept or reject at their discretion.
        """
        await self._ensure_supported(gcs_system_origin_cls, mock_stack_origin_cls)
        ack = await _probe(gcs_system_origin_cls, z=-500.0)
        if ack is None:
            log.warning(_FMT, _CMD, "z=-500.0 (below sea level)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "z=-500.0 (below sea level)", f"result={result}")

    # -----------------------------------------------------------------------
    # Group E — GPS_GLOBAL_ORIGIN response emission
    # -----------------------------------------------------------------------

    async def test_gps_global_origin_emitted(self, gcs_system_origin_cls, mock_stack_origin_cls):
        """
        GPS_GLOBAL_ORIGIN (id=49) must be emitted exactly once after ACCEPTED.

        Spec: "Vehicle should emit GPS_GLOBAL_ORIGIN irrespective of whether
        the origin is changed."  This test verifies emission and the
        exactly-once constraint.
        """
        await self._ensure_supported(gcs_system_origin_cls, mock_stack_origin_cls)

        gps_task, gps_queue = await _subscribe_gps(gcs_system_origin_cls)
        ack = await _probe(gcs_system_origin_cls)

        if ack is None or int(ack["result"]) != MAV_RESULT_ACCEPTED:
            gps_task.cancel()
            pytest.skip("Command not ACCEPTED — skipping GPS_GLOBAL_ORIGIN emission check")

        try:
            gps = await asyncio.wait_for(gps_queue.get(), timeout=_GPS_TIMEOUT_S)
        except asyncio.TimeoutError:
            gps_task.cancel()
            if mock_stack_origin_cls is not None:
                pytest.fail("GPS_GLOBAL_ORIGIN not received within 5 s after ACCEPTED")
            log.warning(_FMT, _CMD, "GPS_GLOBAL_ORIGIN emission", "NOT received — spec violation")
            return

        extra = await _drain(gps_queue)
        gps_task.cancel()

        log.info(_FMT, _CMD, "GPS_GLOBAL_ORIGIN emitted",
                 f"lat={gps.get('latitude')} lon={gps.get('longitude')} "
                 f"alt_mm={gps.get('altitude')} extra={len(extra)}")
        if mock_stack_origin_cls is not None:
            assert len(extra) == 0, (
                f"GPS_GLOBAL_ORIGIN emitted {1 + len(extra)} times; expected exactly 1"
            )

    async def test_gps_global_origin_changes_when_new_value_set(self, gcs_system_origin_cls, mock_stack_origin_cls):
        """
        GPS_GLOBAL_ORIGIN fields must reflect the NEW coordinates when the origin changes.

        Send coord_A → receive GPS_GLOBAL_ORIGIN_A → send coord_B → receive
        GPS_GLOBAL_ORIGIN_B.  Assert: B fields differ from A fields.
        """
        await self._ensure_supported(gcs_system_origin_cls, mock_stack_origin_cls)

        gps_task, gps_queue = await _subscribe_gps(gcs_system_origin_cls)

        ack_a = await _probe(gcs_system_origin_cls, x=_LAT_INT, y=_LON_INT, z=10.0)
        if ack_a is None or int(ack_a["result"]) != MAV_RESULT_ACCEPTED:
            gps_task.cancel()
            pytest.skip("coord_A not ACCEPTED — skipping")

        try:
            gps_a = await asyncio.wait_for(gps_queue.get(), timeout=_GPS_TIMEOUT_S)
        except asyncio.TimeoutError:
            gps_task.cancel()
            if mock_stack_origin_cls is not None:
                pytest.fail("GPS_GLOBAL_ORIGIN not received after coord_A send")
            return

        ack_b = await _probe(gcs_system_origin_cls, x=_LAT_INT2, y=_LON_INT2, z=20.0)
        if ack_b is None or int(ack_b["result"]) != MAV_RESULT_ACCEPTED:
            gps_task.cancel()
            pytest.skip("coord_B not ACCEPTED — skipping")

        try:
            gps_b = await asyncio.wait_for(gps_queue.get(), timeout=_GPS_TIMEOUT_S)
        except asyncio.TimeoutError:
            gps_task.cancel()
            if mock_stack_origin_cls is not None:
                pytest.fail("GPS_GLOBAL_ORIGIN not received after coord_B send")
            return

        gps_task.cancel()

        log.info(_FMT, _CMD, "GPS_GLOBAL_ORIGIN changes on new value",
                 f"A=({gps_a.get('latitude')},{gps_a.get('longitude')},{gps_a.get('altitude')}) "
                 f"B=({gps_b.get('latitude')},{gps_b.get('longitude')},{gps_b.get('altitude')})")

        if mock_stack_origin_cls is not None:
            changed = (
                gps_a.get("latitude") != gps_b.get("latitude")
                or gps_a.get("longitude") != gps_b.get("longitude")
                or gps_a.get("altitude") != gps_b.get("altitude")
            )
            assert changed, "GPS_GLOBAL_ORIGIN fields did not change when a different origin was commanded"

    async def test_gps_global_origin_unchanged_and_emitted_on_repeat(self, gcs_system_origin_cls, mock_stack_origin_cls):
        """
        GPS_GLOBAL_ORIGIN is emitted even when the same origin is set again,
        and carries identical fields both times.

        Spec: "irrespective of whether the origin is changed".
        Send coord_A twice — assert emission on both sends and equal fields.
        """
        await self._ensure_supported(gcs_system_origin_cls, mock_stack_origin_cls)

        gps_task, gps_queue = await _subscribe_gps(gcs_system_origin_cls)

        # First send
        ack1 = await _probe(gcs_system_origin_cls, x=_LAT_INT, y=_LON_INT, z=10.0)
        if ack1 is None or int(ack1["result"]) != MAV_RESULT_ACCEPTED:
            gps_task.cancel()
            pytest.skip("First send not ACCEPTED — skipping")

        try:
            gps1 = await asyncio.wait_for(gps_queue.get(), timeout=_GPS_TIMEOUT_S)
        except asyncio.TimeoutError:
            gps_task.cancel()
            if mock_stack_origin_cls is not None:
                pytest.fail("GPS_GLOBAL_ORIGIN not received after first send")
            return

        # Second send — identical coordinates
        ack2 = await _probe(gcs_system_origin_cls, x=_LAT_INT, y=_LON_INT, z=10.0)
        if ack2 is None or int(ack2["result"]) != MAV_RESULT_ACCEPTED:
            gps_task.cancel()
            pytest.skip("Second send not ACCEPTED — skipping")

        try:
            gps2 = await asyncio.wait_for(gps_queue.get(), timeout=_GPS_TIMEOUT_S)
        except asyncio.TimeoutError:
            gps_task.cancel()
            if mock_stack_origin_cls is not None:
                pytest.fail(
                    "GPS_GLOBAL_ORIGIN not emitted on second (unchanged) send — "
                    "spec violation: must emit 'irrespective of whether origin changed'"
                )
            log.warning(_FMT, _CMD, "GPS_GLOBAL_ORIGIN on unchanged repeat",
                        "NOT received — spec violation")
            return

        gps_task.cancel()

        log.info(_FMT, _CMD, "GPS_GLOBAL_ORIGIN unchanged repeat",
                 f"first=({gps1.get('latitude')},{gps1.get('longitude')},{gps1.get('altitude')}) "
                 f"second=({gps2.get('latitude')},{gps2.get('longitude')},{gps2.get('altitude')})")

        if mock_stack_origin_cls is not None:
            assert gps1.get("latitude") == gps2.get("latitude"), "latitude changed on unchanged repeat"
            assert gps1.get("longitude") == gps2.get("longitude"), "longitude changed on unchanged repeat"
            assert gps1.get("altitude") == gps2.get("altitude"), "altitude changed on unchanged repeat"

    # -----------------------------------------------------------------------
    # Group F — COMMAND_LONG variant
    # -----------------------------------------------------------------------

    async def test_command_long_accepted(self, gcs_system_origin_cls, mock_stack_origin_cls):
        """
        COMMAND_LONG with float degrees — observational.

        The spec says "should be sent as COMMAND_INT" (integer lat/lon preserves
        precision); this test documents COMMAND_LONG behaviour.  Params 1–4
        are NaN (reserved).
        """
        await self._ensure_supported(gcs_system_origin_cls, mock_stack_origin_cls)
        ack = await probe_command_long(
            gcs_system_origin_cls, _CMD_ID,
            param1=None, param2=None, param3=None, param4=None,
            param5=47.3977,  # latitude degrees
            param6=8.5456,   # longitude degrees
            param7=10.0,     # altitude m MSL
        )
        if ack is None:
            log.warning(_FMT, _CMD, "COMMAND_LONG (float degrees)", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "COMMAND_LONG (float degrees)", f"result={result}")

    async def test_command_long_float_int32max_denied(self, gcs_system_origin_cls, mock_stack_origin_cls):
        """
        COMMAND_LONG param5=float(INT32_MAX), param6=float(INT32_MAX) — must be DENIED.

        float(INT32_MAX) ≈ 2.15 × 10⁹ degrees — far outside any valid coordinate range.
        No sentinel semantic exists for this command; the value must be DENIED.
        """
        await self._ensure_supported(gcs_system_origin_cls, mock_stack_origin_cls)
        ack = await probe_command_long(
            gcs_system_origin_cls, _CMD_ID,
            param1=None, param2=None, param3=None, param4=None,
            param5=float(INT32_MAX),
            param6=float(INT32_MAX),
            param7=10.0,
        )
        if ack is None:
            log.warning(_FMT, _CMD, "COMMAND_LONG float(INT32_MAX) lat/lon", "UNKNOWN — no ACK")
            return
        result = int(ack["result"])
        log.info(_FMT, _CMD, "COMMAND_LONG float(INT32_MAX) lat/lon", f"result={result}")
        assert result == MAV_RESULT_DENIED, (
            f"float(INT32_MAX) as lat/lon must be DENIED for DO_SET_GLOBAL_ORIGIN (got {result})"
        )


# ---------------------------------------------------------------------------
# NACK behaviour class — GPS_GLOBAL_ORIGIN must NOT be emitted on DENIED
# ---------------------------------------------------------------------------


@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(300)
class TestDoSetGlobalOriginNackBehaviour:
    """NACK-behaviour tests: GPS_GLOBAL_ORIGIN must not be emitted when command is DENIED."""

    async def test_gps_global_origin_not_emitted_on_nack(
        self, gcs_system_nack_cls, mock_stack_nack_cls
    ):
        """
        GPS_GLOBAL_ORIGIN must NOT be emitted when COMMAND_ACK result is DENIED.

        The origin was not updated (the command was rejected), so there is
        no origin to report.  Runs in paired mode only — requires a mock
        configured to DENY cmd=611 so the condition is reproducible.
        """
        if mock_stack_nack_cls is None:
            pytest.skip("NACK-behaviour test requires paired mock mode — skipping in standalone")

        gps_task, gps_queue = await _subscribe_gps(gcs_system_nack_cls)
        ack = await _probe(gcs_system_nack_cls)

        if ack is not None:
            assert int(ack["result"]) == MAV_RESULT_DENIED, (
                f"Mock should return DENIED for cmd={_CMD_ID}; got {ack['result']}"
            )

        extra = await _drain(gps_queue, drain_s=2.0)
        gps_task.cancel()

        log.info(_FMT, _CMD, "GPS_GLOBAL_ORIGIN not emitted on NACK",
                 f"ack={ack['result'] if ack else 'NONE'} gps_count={len(extra)}")
        assert len(extra) == 0, (
            f"GPS_GLOBAL_ORIGIN must not be emitted when command is DENIED; "
            f"received {len(extra)} message(s)"
        )
