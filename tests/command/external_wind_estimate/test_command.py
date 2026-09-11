"""
MAV_CMD_EXTERNAL_WIND_ESTIMATE (cmd=43004) — Tier 1 ACK tests.

Sets an external estimate of wind speed/direction on the EKF wind estimator —
intended to extend GPS-denied dead-reckoning time by giving the estimator a
head start on the wind vector.  Defined in development.xml (not common.xml,
so not covered by test_survey.py).

Spec reference: mavlink/message_definitions/v1.0/development.xml entry
value=43004.

Parameter layout (development.xml)::

    param1  Wind speed            (m/s, minValue=0)              Horizontal wind speed.           DEFINED (float)
    param2  Wind speed accuracy   (m/s)                          1-sigma accuracy; NaN if unknown.  DEFINED (float)
    param3  Direction             (deg, minValue=0, maxValue=360) Azimuth wind blows FROM.          DEFINED (float)
    param4  Direction accuracy    (deg)                          1-sigma accuracy; NaN if unknown.  DEFINED (float)
    param5  Empty                 (no MAVLink definition) -- COMMAND_INT's `x`  -- UNDEFINED (int32 in COMMAND_INT, float in COMMAND_LONG)
    param6  Empty                 (no MAVLink definition) -- COMMAND_INT's `y`  -- UNDEFINED (int32 in COMMAND_INT, float in COMMAND_LONG)
    param7  Empty                 (no MAVLink definition) -- COMMAND_INT's `z`  -- UNDEFINED (float in BOTH message types -- never int32)

PX4 implementation (src/modules/ekf2/EKF2.cpp, VEHICLE_CMD_EXTERNAL_WIND_ESTIMATE
handler) confirms this layout exactly: it reads only param1-4 and unconditionally
returns ACCEPTED (or UNSUPPORTED if CONFIG_EKF2_WIND is compiled out — not the
case for any SITL board) — params 5-7/x/y/z are never read, and params 1-4 are
never range- or NaN-checked before being forwarded to
Ekf::resetWindToExternalObservation() (src/modules/ekf2/EKF/wind.cpp).

Reporting convention (CLAUDE.md § Mandatory common tests)
-----------------------------------------------------------
Every test's docstring FIRST LINE is a one-sentence statement of its pass
case (e.g. "Accepted when param6/y is sent as its own sentinel (undefined
param)."). This is the single source of truth for:
  - the test's own name (defined/undefined + accepted/rejected/not_denied,
    per the naming convention below),
  - the auto-generated Tier 1 results log (see _check()/_write_tier1_log
    below — written to logs/ on every run, mirroring test_survey.py /
    test_ack_uniqueness.py),
  - the "Pass case" column in external_wind_estimate/README.md's results
    table (kept in sync manually with these docstrings).

Naming convention: a test name states (a) which param, (b) whether that
param is DEFINED (has a MAVLink meaning -- param1-4 here) or UNDEFINED (no
MAVLink meaning -- param5-7 here), and (c) the pass case in one word
(accepted / rejected / not_denied). E.g. `test_param1_defined_nan_not_denied`,
`test_param6_undefined_int32max_accepted`.

INT32_MAX only ever matters for param5/param6 (COMMAND_INT's `x`/`y` — the
only genuinely int32 wire fields either message type has). It is NEVER
tested for param7 (COMMAND_INT's `z` is a float, same as COMMAND_LONG's
param7) or for any of param1-4 (also always floats) — sending
`float(INT32_MAX)` into a float field is just an arbitrary non-NaN float,
already covered by the general "non-sentinel" case, not a distinct
condition worth its own test. (An earlier version of this file had separate
`test_param{5,6,7}_int32max_denied` tests that conflated these two things —
including one for param7, which is a bug: param7 can never be int32 in
either message type. Fixed here.)

Per CLAUDE.md § Mandatory common tests, EVERY test in this file sends the
command via COMMAND_INT, then COMMAND_LONG (`probe_dual()` in
tests/command/conftest.py), and reports one merged result — unless the two
message types disagree, in which case both are logged explicitly as a
finding.  This is a non-location command with meaningful floats, so
COMMAND_LONG is its per-parameter-test "primary" type per CLAUDE.md's
selection rule, but the common checks (and, for simplicity/consistency in
this file, every test) run both regardless.

Since this command has no location, `x`/`y`/`z` (COMMAND_INT's param5/6/7
equivalents) carry no meaning at all — they default to INT32_MAX/INT32_MAX/NaN
(the general "not specified" sentinel for an int32 field / a float field
respectively), matching COMMAND_LONG's param5/6/7 defaulting to NaN.

CONFIRMED PX4 BUG — dual-ACK race (source-traced AND empirically reproduced;
see test_exactly_one_ack): Commander::handle_command() (src/modules/commander/
Commander.cpp) has an explicit switch-case list of commands it deliberately
does NOT answer because "commands ... handled by other parts of the system".
That list includes VEHICLE_CMD_EXTERNAL_ATTITUDE_ESTIMATE,
VEHICLE_CMD_EXTERNAL_POSITION_ESTIMATE and VEHICLE_CMD_ESTIMATOR_SENSOR_ENABLE
— EKF2's other three vehicle_command handlers added in the same family of
work — but VEHICLE_CMD_EXTERNAL_WIND_ESTIMATE is missing from it. Commander
therefore falls through to its `default:` case and answers UNSUPPORTED(3) for
every EXTERNAL_WIND_ESTIMATE send (via either message type — both funnel into
the same internal vehicle_command_s topic), racing against EKF2's own
unconditional ACCEPTED(0). Which ACK reaches the GCS's listener first is
non-deterministic (observed flipping between runs, and between COMMAND_INT vs
COMMAND_LONG sends within the same run, against PX4 MC 1.18.0-beta-dev, HEAD
2026-09-09). To avoid every assertion in this file being flaky on that
account, effective_ack() (conftest.py) collects ALL acks in a short window per
send and prefers a non-UNSUPPORTED one when present — this tests the
command's real behaviour (as EKF2 implements it) rather than which of the two
racing PX4 modules happened to answer first. The race itself is asserted
explicitly (xfail) in test_exactly_one_ack.

IMPORTANT — what these tests can and cannot show
-------------------------------------------------
ACK-level tests can only observe ACCEPTED / DENIED / UNSUPPORTED.  They CANNOT
show whether the EKF wind state actually changed — nor, critically, whether it
changes differently depending on whether the vehicle is on the ground or
airborne.  PX4's resetWindToExternalObservation() is gated by
`if (!_control_status.flags.in_air)` — the reset is applied ONLY when landed,
and silently does nothing while airborne, even though the ACK is unconditionally
ACCEPTED in both cases and the spec's own description explicitly describes an
in-flight ("operating at altitude") use case.  This ground/air behavioural
difference is Tier 2 — see test_flight.py and README.md § "Ground vs air".

Running
-------
Paired mock (no autopilot)::

    pytest tests/command/external_wind_estimate/test_command.py -v --log-cli-level=INFO

PX4 SIH multicopter (head revision)::

    pytest tests/command/external_wind_estimate/test_command.py \\
        --drone-address=udp://:14540 --connection-timeout=60 \\
        --px4-sitl=~/github/PX4/PX4-Autopilot --px4-model=sihsim_quadx \\
        --vehicle-type=quadcopter --autopilot=px4 -v --log-cli-level=INFO

Every run (any mode) writes a Tier 1 results table to
logs/command_external_wind_estimate_tier1_<autopilot>_<vehicle>_<version>_<timestamp>.log
— see _write_tier1_log() below.
"""

import logging

import pytest

from tests.command.conftest import (
    CommandSpec,
    INT32_MAX,
    ParamSpec,
    Tier1CommandTestBase,
    _check,
    gcs_system_cls,  # noqa: F401 — see do_set_global_origin/test_command.py comment
    mock_stack_cls,  # noqa: F401
)
from tests.mock_flight_stack import MAV_RESULT_DENIED, MAV_RESULT_UNSUPPORTED

log = logging.getLogger(__name__)

_CMD = "EXTERNAL_WIND_ESTIMATE"
_CMD_ID = 43004  # MAV_CMD_EXTERNAL_WIND_ESTIMATE (development.xml)

_PX4_NEVER_READS = (
    "PX4's EXTERNAL_WIND_ESTIMATE handler never reads param{slot}/{wire} at all "
    "(confirmed by source inspection, EKF2.cpp) — no known stack validates params "
    "with no MAVLink definition (spec gap)"
)

SPEC = CommandSpec(
    cmd_id=_CMD_ID,
    name=_CMD,
    baseline=dict(
        param1=8.0,     # Wind speed: 8 m/s
        param2=None,    # Wind speed accuracy: NaN = unknown (spec-valid sentinel)
        param3=90.0,    # Direction: wind blowing FROM east
        param4=None,    # Direction accuracy: NaN = unknown (spec-valid sentinel)
        long5=None, long6=None, long7=None,
        int_x=INT32_MAX, int_y=INT32_MAX, int_z=None,
    ),
    params=[
        ParamSpec(1, "Wind speed", defined=True),
        ParamSpec(2, "Wind speed accuracy", defined=True),
        ParamSpec(3, "Direction", defined=True),
        ParamSpec(4, "Direction accuracy", defined=True),
        ParamSpec(5, "Empty", defined=False,
                  reject_xfail_reason=_PX4_NEVER_READS.format(slot=5, wire="x")),
        ParamSpec(6, "Empty", defined=False,
                  reject_xfail_reason=_PX4_NEVER_READS.format(slot=6, wire="y")),
        ParamSpec(7, "Empty", defined=False,
                  reject_xfail_reason=_PX4_NEVER_READS.format(slot=7, wire="z")),
    ],
)


async def _probe(system, **overrides) -> tuple[dict | None, dict | None]:
    """
    probe_dual() with this command's baseline defaults applied — a
    module-level convenience mirroring Tier1CommandTestBase._probe(), for
    test_flight.py (Tier 2) to reuse without instantiating the test class.
    """
    from tests.command.conftest import _ACK_WINDOW_S, probe_dual
    kw = dict(SPEC.baseline)
    kw.update(overrides)
    return await probe_dual(system, SPEC.cmd_id, window_s=_ACK_WINDOW_S, **kw)


def _reduce(label: str, int_ack: dict | None, long_ack: dict | None) -> int | None:
    """Merge a probe_dual() result pair — see conftest._reduce_dual()'s docstring."""
    from tests.command.conftest import _reduce_dual
    return _reduce_dual(SPEC.name, label, int_ack, long_ack)


@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(300)
class TestExternalWindEstimateCommand(Tier1CommandTestBase):
    """
    EXTERNAL_WIND_ESTIMATE — ACK result tests, sent via both COMMAND_INT and
    COMMAND_LONG. Groups A/B/C (the six mandatory common checks) are inherited
    from Tier1CommandTestBase (tests/command/conftest.py) — see its docstring.
    Groups D-G below are this command's own bespoke per-parameter tests.
    """

    SPEC = SPEC

    async def test_exactly_one_ack(self, gcs_system_cls, mock_stack_cls, request):
        """
        Exactly one terminal COMMAND_ACK per send, via each message type.

        CONFIRMED PX4 BUG (see module docstring): Commander.cpp is missing
        EXTERNAL_WIND_ESTIMATE from its "handled elsewhere" ignore-list (its
        siblings EXTERNAL_ATTITUDE_ESTIMATE / EXTERNAL_POSITION_ESTIMATE /
        ESTIMATOR_SENSOR_ENABLE ARE listed there) — Commander's `default:`
        case answers UNSUPPORTED for every send, racing EKF2's ACCEPTED.
        xfail on real PX4 (reproducible — observed on every run against PX4 MC
        1.18.0-beta-dev, HEAD 2026-09-09); hard assertion on mock
        (MockFlightStack has only one command handler — no such race).

        Overrides Tier1CommandTestBase's generic version to attach this
        command's own documented root cause to the xfail reason.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        from tests.command.conftest import (
            _ACK_WINDOW_S, _record, probe_command_int_all_acks, probe_command_long_all_acks,
        )
        description = "Exactly one terminal COMMAND_ACK per send, via each message type"
        kw = self.SPEC.baseline
        int_acks = await probe_command_int_all_acks(
            gcs_system_cls, _CMD_ID, window_s=_ACK_WINDOW_S,
            param1=kw["param1"], param2=kw["param2"], param3=kw["param3"], param4=kw["param4"],
            x=kw["int_x"], y=kw["int_y"], z=kw["int_z"],
        )
        long_acks = await probe_command_long_all_acks(
            gcs_system_cls, _CMD_ID, window_s=_ACK_WINDOW_S,
            param1=kw["param1"], param2=kw["param2"], param3=kw["param3"], param4=kw["param4"],
            param5=kw["long5"], param6=kw["long6"], param7=kw["long7"],
        )

        # Gather both counts BEFORE deciding pass/fail/xfail — pytest.xfail()
        # raises immediately, so deciding inside a loop over message types
        # would skip checking whichever type comes second.
        counts: dict[str, tuple[int, list[int]]] = {}
        for label, acks in (("COMMAND_INT", int_acks), ("COMMAND_LONG", long_acks)):
            assert len(acks) >= 1, f"No COMMAND_ACK received via {label}"
            results = [int(a["result"]) for a in acks]
            log.info("%-14s | %-44s | %s", _CMD, f"ACK count ({label})", f"n={len(acks)} results={results}")
            counts[label] = (len(acks), results)

        offenders = {label: rs for label, (n, rs) in counts.items() if n > 1}
        if offenders:
            if mock_stack_cls is not None:
                _record(type(self), request, "FAIL", description, None)
                pytest.fail(f"Got more than one ACK per send in mock mode: {offenders}")
            _record(type(self), request, "XFAIL", description, None)
            pytest.xfail(
                f"Got more than one COMMAND_ACK for a single send ({offenders}) — confirmed "
                "PX4 bug: Commander::handle_command() is missing VEHICLE_CMD_EXTERNAL_WIND_ESTIMATE "
                "from its 'handled elsewhere' ignore-list (Commander.cpp), unlike its EKF2 "
                "siblings EXTERNAL_ATTITUDE_ESTIMATE/EXTERNAL_POSITION_ESTIMATE/"
                "ESTIMATOR_SENSOR_ENABLE — Commander's default case answers UNSUPPORTED, racing "
                "EKF2's ACCEPTED (EKF2.cpp); which one the GCS sees first is non-deterministic"
            )
        for n, _ in counts.values():
            assert n == 1
        _record(type(self), request, "PASS", description, None)

    # -----------------------------------------------------------------------
    # Group D — param1 (Wind speed, m/s, minValue=0) semantic values
    # -----------------------------------------------------------------------

    async def test_param1_nominal(self, gcs_system_cls, mock_stack_cls, request):
        """Not UNSUPPORTED for param1 (Wind speed) = 8.0 m/s (nominal value)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await self._probe(gcs_system_cls, param1=8.0)
        result = self._reduce("param1 (Wind speed) = 8.0 m/s", int_ack, long_ack)
        _check(type(self), request, "Not UNSUPPORTED for param1 (Wind speed) = 8.0 m/s (nominal value)",
               result, expect=lambda r: r != MAV_RESULT_UNSUPPORTED)

    async def test_param1_zero_boundary(self, gcs_system_cls, mock_stack_cls, request):
        """Not UNSUPPORTED for param1 (Wind speed) = 0.0 m/s (minValue boundary, no wind)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await self._probe(gcs_system_cls, param1=0.0)
        result = self._reduce("param1 (Wind speed) = 0.0 m/s (min boundary)", int_ack, long_ack)
        _check(type(self), request, "Not UNSUPPORTED for param1 (Wind speed) = 0.0 m/s (minValue boundary, no wind)",
               result, expect=lambda r: r != MAV_RESULT_UNSUPPORTED)

    async def test_param1_negative_denied(self, gcs_system_cls, mock_stack_cls, request):
        """
        Denied for param1 (Wind speed) = -1.0 m/s (below minValue=0).

        xfail: PX4 clamps via `math::max(wind_speed, 0.0f)` in
        Ekf::resetWindToExternalObservation() rather than rejecting the command
        (src/modules/ekf2/EKF/wind.cpp) — reasonable defensive behaviour, but
        the wrong result code (an out-of-range input should be DENIED, not
        silently clamped and ACCEPTED).
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await self._probe(gcs_system_cls, param1=-1.0)
        result = self._reduce("param1 (Wind speed) = -1.0 m/s (invalid)", int_ack, long_ack)
        _check(
            type(self), request, "Denied for param1 (Wind speed) = -1.0 m/s (below minValue=0)",
            result, expect=lambda r: r == MAV_RESULT_DENIED,
            xfail_reason=(f"Stack returned {result} for param1=-1.0 (below minValue=0); expected DENIED — "
                          "PX4 clamps negative wind speed to 0 instead of rejecting it (wind.cpp)"),
        )

    # -----------------------------------------------------------------------
    # Group E — param2 (Wind speed accuracy, m/s) semantic values
    # -----------------------------------------------------------------------

    async def test_param2_specific(self, gcs_system_cls, mock_stack_cls, request):
        """Not UNSUPPORTED for param2 (Wind speed accuracy) = 1.5 m/s (specific 1-sigma estimate)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await self._probe(gcs_system_cls, param2=1.5)
        result = self._reduce("param2 (Wind speed accuracy) = 1.5 m/s", int_ack, long_ack)
        _check(type(self), request, "Not UNSUPPORTED for param2 (Wind speed accuracy) = 1.5 m/s (specific 1-sigma estimate)",
               result, expect=lambda r: r != MAV_RESULT_UNSUPPORTED)

    async def test_param2_negative_denied(self, gcs_system_cls, mock_stack_cls, request):
        """
        Denied for param2 (Wind speed accuracy) = -1.0 (negative accuracy is physically meaningless).

        xfail: PX4 squares the value unconditionally
        (`wind_speed_var = sq(wind_speed_accuracy)`) without validating sign —
        a negative accuracy is silently accepted (spec gap).
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await self._probe(gcs_system_cls, param2=-1.0)
        result = self._reduce("param2 (Wind speed accuracy) = -1.0 (invalid)", int_ack, long_ack)
        _check(
            type(self), request, "Denied for param2 (Wind speed accuracy) = -1.0 (negative accuracy is physically meaningless)",
            result, expect=lambda r: r == MAV_RESULT_DENIED,
            xfail_reason=(f"Stack returned {result} for negative param2 accuracy; expected DENIED — "
                          "PX4 squares the value unconditionally without sign validation (wind.cpp)"),
        )

    # -----------------------------------------------------------------------
    # Group F — param3 (Direction, deg, 0-360) semantic values
    # -----------------------------------------------------------------------

    async def test_param3_nominal(self, gcs_system_cls, mock_stack_cls, request):
        """Not UNSUPPORTED for param3 (Direction) = 90.0 deg (nominal, wind from east)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await self._probe(gcs_system_cls, param3=90.0)
        result = self._reduce("param3 (Direction) = 90.0 deg", int_ack, long_ack)
        _check(type(self), request, "Not UNSUPPORTED for param3 (Direction) = 90.0 deg (nominal, wind from east)",
               result, expect=lambda r: r != MAV_RESULT_UNSUPPORTED)

    async def test_param3_zero_boundary(self, gcs_system_cls, mock_stack_cls, request):
        """Not UNSUPPORTED for param3 (Direction) = 0.0 deg (minValue boundary, wind from true north)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await self._probe(gcs_system_cls, param3=0.0)
        result = self._reduce("param3 (Direction) = 0.0 deg (min boundary)", int_ack, long_ack)
        _check(type(self), request, "Not UNSUPPORTED for param3 (Direction) = 0.0 deg (minValue boundary, wind from true north)",
               result, expect=lambda r: r != MAV_RESULT_UNSUPPORTED)

    async def test_param3_max_boundary(self, gcs_system_cls, mock_stack_cls, request):
        """Not UNSUPPORTED for param3 (Direction) = 360.0 deg (maxValue boundary)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await self._probe(gcs_system_cls, param3=360.0)
        result = self._reduce("param3 (Direction) = 360.0 deg (max boundary)", int_ack, long_ack)
        _check(type(self), request, "Not UNSUPPORTED for param3 (Direction) = 360.0 deg (maxValue boundary)",
               result, expect=lambda r: r != MAV_RESULT_UNSUPPORTED)

    async def test_param3_negative_denied(self, gcs_system_cls, mock_stack_cls, request):
        """
        Denied for param3 (Direction) = -10.0 deg (below minValue=0).

        xfail: PX4 wraps the value unconditionally via `wrap_pi()` (EKF2.cpp)
        with no range check — an out-of-range azimuth is silently accepted.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await self._probe(gcs_system_cls, param3=-10.0)
        result = self._reduce("param3 (Direction) = -10.0 deg (invalid)", int_ack, long_ack)
        _check(
            type(self), request, "Denied for param3 (Direction) = -10.0 deg (below minValue=0)",
            result, expect=lambda r: r == MAV_RESULT_DENIED,
            xfail_reason=(f"Stack returned {result} for param3=-10.0 (below minValue=0); expected DENIED — "
                          "PX4 wraps the azimuth unconditionally with no range check (EKF2.cpp)"),
        )

    async def test_param3_over_max_denied(self, gcs_system_cls, mock_stack_cls, request):
        """
        Denied for param3 (Direction) = 370.0 deg (above maxValue=360).

        xfail: same reason as test_param3_negative_denied — PX4 wraps
        unconditionally with no range check.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await self._probe(gcs_system_cls, param3=370.0)
        result = self._reduce("param3 (Direction) = 370.0 deg (invalid)", int_ack, long_ack)
        _check(
            type(self), request, "Denied for param3 (Direction) = 370.0 deg (above maxValue=360)",
            result, expect=lambda r: r == MAV_RESULT_DENIED,
            xfail_reason=(f"Stack returned {result} for param3=370.0 (above maxValue=360); expected DENIED — "
                          "PX4 wraps the azimuth unconditionally with no range check (EKF2.cpp)"),
        )

    # -----------------------------------------------------------------------
    # Group G — param4 (Direction accuracy, deg) semantic values
    # -----------------------------------------------------------------------

    async def test_param4_specific(self, gcs_system_cls, mock_stack_cls, request):
        """Not UNSUPPORTED for param4 (Direction accuracy) = 5.0 deg (specific 1-sigma estimate)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await self._probe(gcs_system_cls, param4=5.0)
        result = self._reduce("param4 (Direction accuracy) = 5.0 deg", int_ack, long_ack)
        _check(type(self), request, "Not UNSUPPORTED for param4 (Direction accuracy) = 5.0 deg (specific 1-sigma estimate)",
               result, expect=lambda r: r != MAV_RESULT_UNSUPPORTED)

    async def test_param4_negative_denied(self, gcs_system_cls, mock_stack_cls, request):
        """
        Denied for param4 (Direction accuracy) = -5.0 (negative accuracy is physically meaningless).

        xfail: PX4 does not validate sign before converting to radians and
        squaring (EKF2.cpp / wind.cpp).
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await self._probe(gcs_system_cls, param4=-5.0)
        result = self._reduce("param4 (Direction accuracy) = -5.0 (invalid)", int_ack, long_ack)
        _check(
            type(self), request,
            "Denied for param4 (Direction accuracy) = -5.0 (negative accuracy is physically meaningless)",
            result, expect=lambda r: r == MAV_RESULT_DENIED,
            xfail_reason=(f"Stack returned {result} for negative param4 accuracy; expected DENIED — "
                          "PX4 does not validate sign before use (wind.cpp)"),
        )
