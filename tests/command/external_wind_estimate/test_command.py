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
import time
from pathlib import Path

import pytest

from tests.command.conftest import (
    ACK_TIMEOUT_S,
    INT32_MAX,
    MAV_FRAME_CATALOGUE,
    _FMT,
    effective_ack,
    gcs_system_cls,  # noqa: F401 — see do_set_global_origin/test_command.py comment
    mock_stack_cls,  # noqa: F401
    probe_command_int_all_acks,
    probe_dual,
)
from tests.conftest import _format_autopilot_header
from tests.mock_flight_stack import (
    MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME,
    MAV_RESULT_DENIED,
    MAV_RESULT_UNSUPPORTED,
)

log = logging.getLogger(__name__)

_CMD = "EXTERNAL_WIND_ESTIMATE"
_CMD_ID = 43004  # MAV_CMD_EXTERNAL_WIND_ESTIMATE (development.xml)

# Window for collecting ALL acks per send, not just the first — needed to see
# past PX4's confirmed Commander/EKF2 dual-ACK race (see module docstring).
# Matches the window used by test_ack_uniqueness.py for the same reason.
_ACK_WINDOW_S = 1.5

# A real-looking coordinate, used as the "non-sentinel" probe value for the
# undefined int32 x/y fields (SIH home lat/lon — arbitrary but plausible).
_REAL_INT = 473977000

# Defaults for the undefined slots (x/y/z ~ param5/6/7): the "not specified"
# sentinel for each field type — INT32_MAX for int32 (x/y), NaN for float (z,
# and COMMAND_LONG's param5/6/7).
_UNDEFINED_DEFAULTS = dict(long5=None, long6=None, long7=None,
                           int_x=INT32_MAX, int_y=INT32_MAX, int_z=None)


def _wind_kwargs(**overrides) -> dict:
    """Default probe_dual() kwargs for a baseline, valid EXTERNAL_WIND_ESTIMATE."""
    defaults = dict(
        param1=8.0,     # Wind speed: 8 m/s
        param2=None,    # Wind speed accuracy: NaN = unknown (spec-valid sentinel)
        param3=90.0,    # Direction: wind blowing FROM east
        param4=None,    # Direction accuracy: NaN = unknown (spec-valid sentinel)
    )
    defaults.update(_UNDEFINED_DEFAULTS)
    defaults.update(overrides)
    return defaults


async def _probe(system, **overrides) -> tuple[dict | None, dict | None]:
    """probe_dual() with this command's baseline defaults applied."""
    return await probe_dual(system, _CMD_ID, window_s=_ACK_WINDOW_S, **_wind_kwargs(**overrides))


def _reduce(label: str, int_ack: dict | None, long_ack: dict | None) -> int | None:
    """
    Merge a probe_dual() result pair into a single reported outcome.

    Logs once if COMMAND_INT and COMMAND_LONG agree; logs both explicitly
    (WARNING) if they disagree — a real protocol inconsistency, not just
    noise, per CLAUDE.md § Mandatory common tests. Returns None if EITHER
    message type got no ACK at all (a partial UNKNOWN is itself worth
    surfacing rather than silently resolved to "whichever answered").
    """
    int_result = int(int_ack["result"]) if int_ack is not None else None
    long_result = int(long_ack["result"]) if long_ack is not None else None
    if int_result is None or long_result is None:
        log.warning(_FMT, _CMD, label,
                    f"UNKNOWN on at least one message type: COMMAND_INT={int_result} COMMAND_LONG={long_result}")
        return None
    if int_result == long_result:
        log.info(_FMT, _CMD, label, f"result={long_result} (COMMAND_INT and COMMAND_LONG agree)")
        return long_result
    log.warning(_FMT, _CMD, label,
                f"INCONSISTENT: COMMAND_INT={int_result} COMMAND_LONG={long_result}")
    return long_result  # this command's per-parameter-test primary type (see module docstring)


# ---------------------------------------------------------------------------
# Tier 1 results table — recorded by _check(), written to logs/ at class end.
# ---------------------------------------------------------------------------

_RESULTS: list[tuple[str, str, str, "int | None"]] = []  # (test name, outcome, description, result)
_DETAILS: list[str] = []  # supplementary multi-line blocks (e.g. a full per-frame breakdown)


def _record(request, outcome: str, description: str, result: int | None) -> None:
    _RESULTS.append((request.node.name, outcome, description, result))


def _record_detail(text: str) -> None:
    """
    Attach a supplementary multi-line block to the Tier 1 log, printed as an
    appendix after the main results table. Use for a test whose full findings
    don't fit the table's one-line-per-test format (e.g. a per-frame survey).
    """
    _DETAILS.append(text)


def _check(request, description: str, result: int | None, *, expect, xfail_reason: str | None = None) -> None:
    """
    Record this test's outcome into the shared Tier 1 results table, then
    perform the actual pytest assertion/xfail.

    `description` is this test's one-line pass case (should match its
    docstring's first line — see module docstring "Reporting convention").
    `expect`: predicate(result:int) -> bool, only called when result is not
    None. `xfail_reason`: if given and the predicate fails, xfail with this
    reason (a known, documented stack gap) instead of hard-failing.
    """
    if result is None:
        _record(request, "UNKNOWN", description, None)
        return  # ambiguous no-ACK on at least one message type; already logged by _reduce()
    condition = expect(result)
    if condition:
        _record(request, "PASS", description, result)
        return
    if xfail_reason:
        _record(request, "XFAIL", description, result)
        pytest.xfail(xfail_reason)
    _record(request, "FAIL", description, result)
    pytest.fail(description)


def _safe(s: str) -> str:
    return s.replace("/", "_").replace(" ", "_").replace("\\", "_")


@pytest.fixture(scope="class", autouse=True)
def _write_tier1_log(request):
    """
    Write the accumulated Tier 1 results table to logs/ once, after every
    test in the class has run — mirrors test_survey.py / test_ack_uniqueness.py,
    which always write regardless of pass/fail.
    """
    yield
    if not _RESULTS:
        return  # nothing ran (e.g. filtered with -k) -- nothing to write

    info = getattr(request.config, "_autopilot_info", {})
    drone_address = request.config.getoption("--drone-address")
    header = _format_autopilot_header(info, drone_address)

    counts: dict[str, int] = {}
    lines = [
        f"Tier 1 results: MAV_CMD_{_CMD} (cmd={_CMD_ID})",
        "=" * 100,
        f"{'Test':<45} {'Outcome':<12} {'Result':<7} Pass case",
        "-" * 100,
    ]
    for name, outcome, description, result in _RESULTS:
        counts[outcome] = counts.get(outcome, 0) + 1
        result_str = "-" if result is None else str(result)
        lines.append(f"{name:<45} {outcome:<12} {result_str:<7} {description}")
    lines.append("-" * 100)
    lines.append(
        f"Total: {len(_RESULTS)}  " + "  ".join(f"{k}={v}" for k, v in sorted(counts.items()))
    )
    if _DETAILS:
        lines.append("")
        lines.append("Supplementary detail")
        lines.append("=" * 100)
        for block in _DETAILS:
            lines.append(block)
    table = "\n".join(lines)
    log.info("\n%s", table)

    ap = _safe(info.get("autopilot", "unknown").lower().replace("ardupilotmega", "ardupilot"))
    vt = _safe(info.get("vehicle_type", "unknown").lower())
    ver_raw = info.get("firmware_version", "")
    ver = f"_{_safe(ver_raw)}" if ver_raw and ver_raw != "N/A" else ""
    timestamp = time.strftime("%Y%m%d_%H%M%S")

    logs_dir = Path("logs")
    logs_dir.mkdir(exist_ok=True)
    log_path = logs_dir / f"command_external_wind_estimate_tier1_{ap}_{vt}{ver}_{timestamp}.log"
    log_path.write_text(header + "\n\n" + table + "\n")
    log.info(_FMT, _CMD, "Tier 1 results log written", str(log_path))


@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(300)
class TestExternalWindEstimateCommand:
    """EXTERNAL_WIND_ESTIMATE — ACK result tests, sent via both COMMAND_INT and COMMAND_LONG."""

    _supported: bool | None = None  # class-level cache; None = not yet probed

    async def _ensure_supported(self, system, mock_stack) -> None:
        """Probe once per class; skip all subsequent tests if UNSUPPORTED."""
        if TestExternalWindEstimateCommand._supported is None:
            int_ack, long_ack = await _probe(system)
            result = _reduce("support probe", int_ack, long_ack)
            TestExternalWindEstimateCommand._supported = (result != MAV_RESULT_UNSUPPORTED)
        if not TestExternalWindEstimateCommand._supported:
            pytest.skip(f"{_CMD} (cmd={_CMD_ID}) is UNSUPPORTED on this platform — test not run")

    # -----------------------------------------------------------------------
    # Group A — mandatory common tests (CLAUDE.md § Mandatory common tests)
    # -----------------------------------------------------------------------

    async def test_command_ack_received(self, gcs_system_cls, mock_stack_cls, request):
        """ACKs (via both COMMAND_INT and COMMAND_LONG) for a baseline, valid send."""
        int_ack, long_ack = await _probe(gcs_system_cls)
        description = "ACKs (via both COMMAND_INT and COMMAND_LONG) for a baseline, valid send"
        if int_ack is None or long_ack is None:
            _record(request, "FAIL", description, None)
            assert int_ack is not None, (
                f"No COMMAND_ACK received via COMMAND_INT within {ACK_TIMEOUT_S:.1f}s — "
                "every command must be acknowledged per spec"
            )
            assert long_ack is not None, (
                f"No COMMAND_ACK received via COMMAND_LONG within {ACK_TIMEOUT_S:.1f}s — "
                "every command must be acknowledged per spec"
            )
        _record(request, "PASS", description, int(long_ack["result"]))
        log.info(_FMT, _CMD, "ACK received", f"COMMAND_INT={int(int_ack['result'])} "
                 f"COMMAND_LONG={int(long_ack['result'])}")

    async def test_command_supported(self, gcs_system_cls, mock_stack_cls, request):
        """Not UNSUPPORTED for a baseline, valid send."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await _probe(gcs_system_cls)
        result = _reduce("baseline", int_ack, long_ack)
        _check(request, "Not UNSUPPORTED for a baseline, valid send", result,
               expect=lambda r: r != MAV_RESULT_UNSUPPORTED)

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
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        description = "Exactly one terminal COMMAND_ACK per send, via each message type"

        from tests.command.conftest import probe_command_int_all_acks, probe_command_long_all_acks
        kw = _wind_kwargs()
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
            log.info(_FMT, _CMD, f"ACK count ({label})", f"n={len(acks)} results={results}")
            counts[label] = (len(acks), results)

        offenders = {label: rs for label, (n, rs) in counts.items() if n > 1}
        if offenders:
            if mock_stack_cls is not None:
                _record(request, "FAIL", description, None)
                pytest.fail(f"Got more than one ACK per send in mock mode: {offenders}")
            _record(request, "XFAIL", description, None)
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
        _record(request, "PASS", description, None)

    async def test_frame_validation_survey(self, gcs_system_cls, mock_stack_cls, request):
        """
        Frame validation survey: does the stack ever return
        MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME(9) for any MAV_FRAME value?

        COMMAND_INT only — COMMAND_LONG has no `frame` field, so there is no
        COMMAND_LONG counterpart to this test. Sends the baseline command
        across all 22 canonical MAV_FRAME values (MAV_FRAME_CATALOGUE,
        tests/command/conftest.py — the same catalogue
        tests/mission/test_frame_types.py uses) and checks whether ANY
        response is UNSUPPORTED_MAV_FRAME(9), which would confirm the stack
        validates the frame field for this command.

        This is purely observational/informative, not a spec-conformance
        check with a single right answer — there is no assertion. If
        UNSUPPORTED_MAV_FRAME(9) is seen for at least one frame, that is
        positive evidence of frame validation (recorded PASS). If it is never
        seen, that does NOT mean the stack ignores frame — only that none of
        the tested frames happened to trigger a rejection; recorded
        INCONCLUSIVE, not FAIL.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        description = "Frame validation survey: any MAV_FRAME value returns MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME(9)"

        kw = _wind_kwargs()
        per_frame: list[tuple[int, str, int | None]] = []
        for frame_id, frame_name in MAV_FRAME_CATALOGUE:
            acks = await probe_command_int_all_acks(
                gcs_system_cls, _CMD_ID, frame=frame_id, window_s=_ACK_WINDOW_S,
                param1=kw["param1"], param2=kw["param2"], param3=kw["param3"], param4=kw["param4"],
                x=kw["int_x"], y=kw["int_y"], z=kw["int_z"],
            )
            ack = effective_ack(acks)
            result = int(ack["result"]) if ack is not None else None
            per_frame.append((frame_id, frame_name, result))

        all_acked = all(result is not None for _, _, result in per_frame)
        hits = [(fid, fname) for fid, fname, result in per_frame
                if result == MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME]

        # Report: full per-frame breakdown if any frame got no ACK at all;
        # otherwise just a terse "all ACKed" note (still stating the primary
        # finding either way).
        if not all_acked:
            table_lines = [
                f"  frame={fid:>2} ({fname}): {'UNKNOWN (no ACK)' if result is None else result}"
                for fid, fname, result in per_frame
            ]
            block = (
                f"Frame validation survey ({_CMD}, cmd={_CMD_ID}) — full breakdown "
                f"(not all frames ACKed):\n" + "\n".join(table_lines)
            )
        else:
            block = (
                f"Frame validation survey ({_CMD}, cmd={_CMD_ID}): all {len(per_frame)} frames ACKed."
            )
        if hits:
            hit_desc = ", ".join(f"frame={fid} ({fname})" for fid, fname in hits)
            block += f"\nUNSUPPORTED_MAV_FRAME(9) returned for: {hit_desc} — frame validation confirmed."
        else:
            block += (
                "\nNo frame returned UNSUPPORTED_MAV_FRAME(9) — inconclusive; this does NOT mean "
                "frame is unvalidated, only that none of the tested frames triggered a rejection."
            )
        _record_detail(block)
        log.info(_FMT, _CMD, "frame validation survey", block)

        if hits:
            _record(request, "PASS", description, MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME)
        else:
            _record(request, "INCONCLUSIVE", description, None)

    # -----------------------------------------------------------------------
    # Group B — undefined params (5/6/7 ~ COMMAND_INT's x/y/z): sentinel
    # accepted, non-sentinel rejected. param5/param6 have a genuine int32
    # form (x/y); param7 (z) never does — it is a float in BOTH message
    # types, so INT32_MAX is never tested for it (see module docstring).
    # -----------------------------------------------------------------------

    async def test_param5_undefined_int32max_accepted(self, gcs_system_cls, mock_stack_cls, request):
        """Accepted when param5/x is sent as its own sentinel (undefined param)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await _probe(gcs_system_cls)  # baseline already uses the sentinel
        result = _reduce("param5/x = sentinel (INT32_MAX / NaN)", int_ack, long_ack)
        _check(request, "Accepted when param5/x is sent as its own sentinel (undefined param)",
               result, expect=lambda r: r not in (MAV_RESULT_UNSUPPORTED, MAV_RESULT_DENIED))

    async def test_param5_undefined_nonsentinel_rejected(self, gcs_system_cls, mock_stack_cls, request):
        """
        Rejected when param5/x is sent a real (non-sentinel) value (undefined param).

        xfail: PX4's EXTERNAL_WIND_ESTIMATE handler never reads param5/x at
        all (confirmed by source inspection, EKF2.cpp) — any value there is
        silently ignored and the command is still ACCEPTED.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await _probe(gcs_system_cls, long5=1.0, int_x=_REAL_INT)
        result = _reduce("param5/x = non-sentinel", int_ack, long_ack)
        _check(
            request, "Rejected when param5/x is sent a real (non-sentinel) value (undefined param)",
            result, expect=lambda r: r == MAV_RESULT_DENIED,
            xfail_reason=(f"Stack returned {result} for undefined param5/x; expected DENIED — "
                          "no known stack validates params with no MAVLink definition (spec gap; "
                          "PX4 never reads param5/x for this command)"),
        )

    async def test_param6_undefined_int32max_accepted(self, gcs_system_cls, mock_stack_cls, request):
        """Accepted when param6/y is sent as its own sentinel (undefined param)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await _probe(gcs_system_cls)
        result = _reduce("param6/y = sentinel (INT32_MAX / NaN)", int_ack, long_ack)
        _check(request, "Accepted when param6/y is sent as its own sentinel (undefined param)",
               result, expect=lambda r: r not in (MAV_RESULT_UNSUPPORTED, MAV_RESULT_DENIED))

    async def test_param6_undefined_nonsentinel_rejected(self, gcs_system_cls, mock_stack_cls, request):
        """
        Rejected when param6/y is sent a real (non-sentinel) value (undefined param).

        xfail: PX4's EXTERNAL_WIND_ESTIMATE handler never reads param6/y at
        all (confirmed by source inspection, EKF2.cpp).
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await _probe(gcs_system_cls, long6=1.0, int_y=_REAL_INT)
        result = _reduce("param6/y = non-sentinel", int_ack, long_ack)
        _check(
            request, "Rejected when param6/y is sent a real (non-sentinel) value (undefined param)",
            result, expect=lambda r: r == MAV_RESULT_DENIED,
            xfail_reason=(f"Stack returned {result} for undefined param6/y; expected DENIED — "
                          "no known stack validates params with no MAVLink definition (spec gap; "
                          "PX4 never reads param6/y for this command)"),
        )

    async def test_param7_undefined_nan_accepted(self, gcs_system_cls, mock_stack_cls, request):
        """Accepted when param7/z is sent as NaN (undefined param; never int32 in either message type)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await _probe(gcs_system_cls)  # baseline already uses NaN
        result = _reduce("param7/z = NaN", int_ack, long_ack)
        _check(request, "Accepted when param7/z is sent as NaN (undefined param; never int32 in either message type)",
               result, expect=lambda r: r not in (MAV_RESULT_UNSUPPORTED, MAV_RESULT_DENIED))

    async def test_param7_undefined_nonsentinel_rejected(self, gcs_system_cls, mock_stack_cls, request):
        """
        Rejected when param7/z is sent a real (non-NaN) value (undefined param).

        xfail: PX4's EXTERNAL_WIND_ESTIMATE handler never reads param7/z at
        all (confirmed by source inspection, EKF2.cpp).
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await _probe(gcs_system_cls, long7=1.0, int_z=1.0)
        result = _reduce("param7/z = 1.0 (non-sentinel)", int_ack, long_ack)
        _check(
            request, "Rejected when param7/z is sent a real (non-NaN) value (undefined param)",
            result, expect=lambda r: r == MAV_RESULT_DENIED,
            xfail_reason=(f"Stack returned {result} for undefined param7/z; expected DENIED — "
                          "no known stack validates params with no MAVLink definition (spec gap; "
                          "PX4 never reads param7/z for this command)"),
        )

    # -----------------------------------------------------------------------
    # Group C — defined (used) params 1-4 tolerate their NaN sentinel.
    # Generalises the documented cases (param2/param4, "NaN if unknown") to
    # the general MAVLink "value not specified" convention, per CLAUDE.md
    # § Mandatory common tests item 5 — param1/param3's spec text is silent
    # on NaN, but the convention is treated as the default expectation for
    # any used param. This command has no used int32 param (no location), so
    # no INT32_MAX-defined-param case applies here.
    # -----------------------------------------------------------------------

    async def test_param1_defined_nan_not_denied(self, gcs_system_cls, mock_stack_cls, request):
        """Not denied when param1 (Wind speed, defined) is sent as NaN."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await _probe(gcs_system_cls, param1=None)
        result = _reduce("param1 (defined) = NaN", int_ack, long_ack)
        _check(request, "Not denied when param1 (Wind speed, defined) is sent as NaN",
               result, expect=lambda r: r != MAV_RESULT_DENIED)

    async def test_param2_defined_nan_not_denied(self, gcs_system_cls, mock_stack_cls, request):
        """Not denied when param2 (Wind speed accuracy, defined) is sent as NaN — spec-documented 'unknown' sentinel."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await _probe(gcs_system_cls, param2=None)
        result = _reduce("param2 (defined) = NaN", int_ack, long_ack)
        _check(request, "Not denied when param2 (Wind speed accuracy, defined) is sent as NaN "
                        "— spec-documented 'unknown' sentinel",
               result, expect=lambda r: r != MAV_RESULT_DENIED)

    async def test_param3_defined_nan_not_denied(self, gcs_system_cls, mock_stack_cls, request):
        """Not denied when param3 (Direction, defined) is sent as NaN."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await _probe(gcs_system_cls, param3=None)
        result = _reduce("param3 (defined) = NaN", int_ack, long_ack)
        _check(request, "Not denied when param3 (Direction, defined) is sent as NaN",
               result, expect=lambda r: r != MAV_RESULT_DENIED)

    async def test_param4_defined_nan_not_denied(self, gcs_system_cls, mock_stack_cls, request):
        """Not denied when param4 (Direction accuracy, defined) is sent as NaN — spec-documented 'unknown' sentinel."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await _probe(gcs_system_cls, param4=None)
        result = _reduce("param4 (defined) = NaN", int_ack, long_ack)
        _check(request, "Not denied when param4 (Direction accuracy, defined) is sent as NaN "
                        "— spec-documented 'unknown' sentinel",
               result, expect=lambda r: r != MAV_RESULT_DENIED)

    # -----------------------------------------------------------------------
    # Group D — param1 (Wind speed, m/s, minValue=0) semantic values
    # -----------------------------------------------------------------------

    async def test_param1_nominal(self, gcs_system_cls, mock_stack_cls, request):
        """Not UNSUPPORTED for param1 (Wind speed) = 8.0 m/s (nominal value)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await _probe(gcs_system_cls, param1=8.0)
        result = _reduce("param1 (Wind speed) = 8.0 m/s", int_ack, long_ack)
        _check(request, "Not UNSUPPORTED for param1 (Wind speed) = 8.0 m/s (nominal value)",
               result, expect=lambda r: r != MAV_RESULT_UNSUPPORTED)

    async def test_param1_zero_boundary(self, gcs_system_cls, mock_stack_cls, request):
        """Not UNSUPPORTED for param1 (Wind speed) = 0.0 m/s (minValue boundary, no wind)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await _probe(gcs_system_cls, param1=0.0)
        result = _reduce("param1 (Wind speed) = 0.0 m/s (min boundary)", int_ack, long_ack)
        _check(request, "Not UNSUPPORTED for param1 (Wind speed) = 0.0 m/s (minValue boundary, no wind)",
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
        int_ack, long_ack = await _probe(gcs_system_cls, param1=-1.0)
        result = _reduce("param1 (Wind speed) = -1.0 m/s (invalid)", int_ack, long_ack)
        _check(
            request, "Denied for param1 (Wind speed) = -1.0 m/s (below minValue=0)",
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
        int_ack, long_ack = await _probe(gcs_system_cls, param2=1.5)
        result = _reduce("param2 (Wind speed accuracy) = 1.5 m/s", int_ack, long_ack)
        _check(request, "Not UNSUPPORTED for param2 (Wind speed accuracy) = 1.5 m/s (specific 1-sigma estimate)",
               result, expect=lambda r: r != MAV_RESULT_UNSUPPORTED)

    async def test_param2_negative_denied(self, gcs_system_cls, mock_stack_cls, request):
        """
        Denied for param2 (Wind speed accuracy) = -1.0 (negative accuracy is physically meaningless).

        xfail: PX4 squares the value unconditionally
        (`wind_speed_var = sq(wind_speed_accuracy)`) without validating sign —
        a negative accuracy is silently accepted (spec gap).
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await _probe(gcs_system_cls, param2=-1.0)
        result = _reduce("param2 (Wind speed accuracy) = -1.0 (invalid)", int_ack, long_ack)
        _check(
            request, "Denied for param2 (Wind speed accuracy) = -1.0 (negative accuracy is physically meaningless)",
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
        int_ack, long_ack = await _probe(gcs_system_cls, param3=90.0)
        result = _reduce("param3 (Direction) = 90.0 deg", int_ack, long_ack)
        _check(request, "Not UNSUPPORTED for param3 (Direction) = 90.0 deg (nominal, wind from east)",
               result, expect=lambda r: r != MAV_RESULT_UNSUPPORTED)

    async def test_param3_zero_boundary(self, gcs_system_cls, mock_stack_cls, request):
        """Not UNSUPPORTED for param3 (Direction) = 0.0 deg (minValue boundary, wind from true north)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await _probe(gcs_system_cls, param3=0.0)
        result = _reduce("param3 (Direction) = 0.0 deg (min boundary)", int_ack, long_ack)
        _check(request, "Not UNSUPPORTED for param3 (Direction) = 0.0 deg (minValue boundary, wind from true north)",
               result, expect=lambda r: r != MAV_RESULT_UNSUPPORTED)

    async def test_param3_max_boundary(self, gcs_system_cls, mock_stack_cls, request):
        """Not UNSUPPORTED for param3 (Direction) = 360.0 deg (maxValue boundary)."""
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await _probe(gcs_system_cls, param3=360.0)
        result = _reduce("param3 (Direction) = 360.0 deg (max boundary)", int_ack, long_ack)
        _check(request, "Not UNSUPPORTED for param3 (Direction) = 360.0 deg (maxValue boundary)",
               result, expect=lambda r: r != MAV_RESULT_UNSUPPORTED)

    async def test_param3_negative_denied(self, gcs_system_cls, mock_stack_cls, request):
        """
        Denied for param3 (Direction) = -10.0 deg (below minValue=0).

        xfail: PX4 wraps the value unconditionally via `wrap_pi()` (EKF2.cpp)
        with no range check — an out-of-range azimuth is silently accepted.
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await _probe(gcs_system_cls, param3=-10.0)
        result = _reduce("param3 (Direction) = -10.0 deg (invalid)", int_ack, long_ack)
        _check(
            request, "Denied for param3 (Direction) = -10.0 deg (below minValue=0)",
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
        int_ack, long_ack = await _probe(gcs_system_cls, param3=370.0)
        result = _reduce("param3 (Direction) = 370.0 deg (invalid)", int_ack, long_ack)
        _check(
            request, "Denied for param3 (Direction) = 370.0 deg (above maxValue=360)",
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
        int_ack, long_ack = await _probe(gcs_system_cls, param4=5.0)
        result = _reduce("param4 (Direction accuracy) = 5.0 deg", int_ack, long_ack)
        _check(request, "Not UNSUPPORTED for param4 (Direction accuracy) = 5.0 deg (specific 1-sigma estimate)",
               result, expect=lambda r: r != MAV_RESULT_UNSUPPORTED)

    async def test_param4_negative_denied(self, gcs_system_cls, mock_stack_cls, request):
        """
        Denied for param4 (Direction accuracy) = -5.0 (negative accuracy is physically meaningless).

        xfail: PX4 does not validate sign before converting to radians and
        squaring (EKF2.cpp / wind.cpp).
        """
        await self._ensure_supported(gcs_system_cls, mock_stack_cls)
        int_ack, long_ack = await _probe(gcs_system_cls, param4=-5.0)
        result = _reduce("param4 (Direction accuracy) = -5.0 (invalid)", int_ack, long_ack)
        _check(
            request,
            "Denied for param4 (Direction accuracy) = -5.0 (negative accuracy is physically meaningless)",
            result, expect=lambda r: r == MAV_RESULT_DENIED,
            xfail_reason=(f"Stack returned {result} for negative param4 accuracy; expected DENIED — "
                          "PX4 does not validate sign before use (wind.cpp)"),
        )
