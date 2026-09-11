"""
COMMAND_ACK uniqueness — confirms every MAV_CMD produces exactly one terminal
COMMAND_ACK per COMMAND_INT sent, catching double-ACK protocol bugs.

Purpose
-------
test_survey.py asks "what result did we get?" using probe_command_int(), which
returns as soon as the *first* matching ACK arrives and then cancels its
subscription — it cannot see a second ACK arriving shortly after.  This module
asks a different question: "how many terminal ACKs did we get?", by staying
subscribed for a fixed window after each send and collecting all of them.

A command may legitimately emit any number of IN_PROGRESS (5) ACKs before its
one terminal result (ACCEPTED/DENIED/UNSUPPORTED/FAILED/etc — see
tests/command/CLAUDE.md § MAV_RESULT values).  Only the terminal ACK is
required to be sent exactly once per the MAVLink spec; receiving it twice (or
receiving two different terminal results) is a stack-side protocol bug.  A
missing terminal ACK (UNKNOWN, per the no-ACK policy in conftest.py) is a
separate, already-tracked condition and is NOT treated as a failure here —
this test only fails on *duplicate* terminal ACKs.

Classification per command
---------------------------
  0 terminal ACKs   → UNKNOWN, logged, not a failure (tracked by test_survey.py)
  1 terminal ACK    → OK
  2+ terminal ACKs  → FAIL — violation recorded and asserted at the end

Running
-------
Against the mock (also exercises the duplicate-ACK detection path — see
TestAckUniquenessDetection below)::

    pytest tests/command/test_ack_uniqueness.py -v --log-cli-level=INFO

Against a real flight stack::

    pytest tests/command/test_ack_uniqueness.py --drone-address=udp://:14540 -v --log-cli-level=INFO
"""

import asyncio
import logging
import time
from pathlib import Path

import pytest
import pytest_asyncio
from mavsdk import System

from .conftest import (
    _load_commands,
    probe_command_int_all_acks,
    INT32_MAX,
)
from tests.conftest import DRONE_GRPC_PORT, _wait_for_connection, _format_autopilot_header
from tests.mock_flight_stack import MockFlightStack, MAV_RESULT_IN_PROGRESS

log = logging.getLogger(__name__)

# Time to keep listening for COMMAND_ACK after each send.  Deliberately a bit
# longer than test_survey.py's 2.0s single-ACK timeout, since this test must
# wait out the full window on every command regardless of when the first ACK
# arrives (it cannot early-exit without risking missing a delayed duplicate).
_ACK_WINDOW_S = 1.5

# Known MAVSDK mavsdk_server quirk (paired/mock mode only): the drone-side
# mavsdk_server binary auto-ACKs the deprecated MAV_CMD_REQUEST_AUTOPILOT_
# CAPABILITIES (520) itself, independently of and in addition to whatever the
# drone-side script (MockFlightStack) sends — confirmed by inspecting the raw
# wire traffic: two byte-identical COMMAND_ACK(cmd=520, result=0) messages
# arrive at the GCS even though MockFlightStack.received_commands shows only
# one incoming COMMAND_INT was ever delivered to its handler.  This mirrors
# the already-documented AUTOPILOT_VERSION dual-response behaviour (CLAUDE.md
# § Capability response interaction) — mavsdk_server has built-in legacy
# capability-request handling that our mock's generic pass-through logic
# cannot see or suppress.  This is a MAVSDK test-harness artifact specific to
# the paired-mock architecture, not a flight-stack protocol bug, so it is
# excluded from the mock-mode assertion (it does not apply in standalone mode
# against a real stack, where the vehicle is the sole source of ACKs).
_MOCK_ONLY_KNOWN_DUPLICATES = {520}  # MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES


def _is_terminal(ack: dict) -> bool:
    return int(ack.get("result", -1)) != MAV_RESULT_IN_PROGRESS


@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(900)  # 168 commands × 1.5s fixed window each + connection overhead
class TestCommandAckUniqueness:
    """For every MAV_CMD, confirm exactly one terminal COMMAND_ACK is received."""

    async def test_exactly_one_terminal_ack_per_command(self, gcs_system_cls, mock_stack_cls, request):
        """
        Send every MAV_CMD via COMMAND_INT and assert no command produces more
        than one terminal COMMAND_ACK.

        Always writes the full per-command table to logs/ (like test_survey.py),
        but — unlike the survey — this test actually fails if any command
        violates the "exactly one terminal ACK" invariant.
        """
        definitions_dir = Path(request.config.getoption("--mavlink-definitions-dir"))
        commands = _load_commands(definitions_dir)
        if not commands:
            pytest.skip(f"No MAV_CMD entries found in {definitions_dir}/common.xml — "
                        "run: git submodule update --init mavlink")

        log.info("Checking ACK uniqueness for %d MAV_CMD entries from %s", len(commands), definitions_dir)

        results: dict[int, tuple[str, int, int]] = {}  # cmd_id -> (name, n_in_progress, n_terminal)
        violations: list[str] = []
        known_exceptions: set[int] = set()

        for cmd_id, cmd_name in sorted(commands.items()):
            acks = await probe_command_int_all_acks(
                gcs_system_cls,
                command=cmd_id,
                frame=6,          # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                window_s=_ACK_WINDOW_S,
                param1=0.0,
                param2=0.0,
                param3=0.0,
                param4=0.0,
                x=INT32_MAX,      # "use current position" sentinel for lat
                y=INT32_MAX,      # "use current position" sentinel for lon
                z=0.0,
            )
            terminal = [a for a in acks if _is_terminal(a)]
            in_progress = [a for a in acks if not _is_terminal(a)]
            results[cmd_id] = (cmd_name, len(in_progress), len(terminal))

            if len(terminal) > 1:
                result_values = [int(a["result"]) for a in terminal]
                if mock_stack_cls is not None and cmd_id in _MOCK_ONLY_KNOWN_DUPLICATES:
                    known_exceptions.add(cmd_id)
                    log.info(
                        "%s (cmd=%d): %d terminal ACKs (results=%s) — known mavsdk_server "
                        "auto-responder quirk in mock mode, not a flight-stack bug; excluded "
                        "from assertion (see _MOCK_ONLY_KNOWN_DUPLICATES)",
                        cmd_name, cmd_id, len(terminal), result_values,
                    )
                else:
                    msg = (
                        f"{cmd_name} (cmd={cmd_id}): {len(terminal)} terminal ACKs received "
                        f"(results={result_values}) — expected at most 1"
                    )
                    violations.append(msg)
                    log.error("DUPLICATE ACK: %s", msg)
            else:
                log.info(
                    "%-45s (cmd=%4d) → %d terminal, %d IN_PROGRESS",
                    cmd_name, cmd_id, len(terminal), len(in_progress),
                )

        info = getattr(request.config, "_autopilot_info", {})
        drone_address = request.config.getoption("--drone-address")
        _log_uniqueness_table(results, violations, known_exceptions, info, drone_address)

        assert not violations, (
            f"{len(violations)} command(s) received more than one terminal COMMAND_ACK "
            f"for a single send:\n" + "\n".join(violations)
        )


def _log_uniqueness_table(
    results: dict[int, tuple[str, int, int]],
    violations: list[str],
    known_exceptions: set[int],
    info: dict,
    drone_address: str | None,
) -> None:
    """Log and write the ACK-uniqueness results table, prefixed with the stack probe header."""
    table_lines = [
        "COMMAND_ACK uniqueness check results",
        "=" * 88,
        f"{'CMD ID':>6}  {'Terminal':<9}  {'InProg':<7}  {'Status':<10}  {'Command Name'}",
        "-" * 88,
    ]
    n_ok = n_unknown = n_violation = n_known = 0
    for cmd_id, (cmd_name, n_in_progress, n_terminal) in sorted(results.items()):
        if n_terminal == 0:
            status = "UNKNOWN"
            n_unknown += 1
        elif n_terminal == 1:
            status = "OK"
            n_ok += 1
        elif cmd_id in known_exceptions:
            status = "KNOWN"
            n_known += 1
        else:
            status = "DUPLICATE"
            n_violation += 1
        table_lines.append(f"{cmd_id:>6}  {n_terminal:<9}  {n_in_progress:<7}  {status:<10}  {cmd_name}")

    table_lines.append("-" * 88)
    table_lines.append(
        f"Total: {len(results)}  OK={n_ok}  UNKNOWN(no ACK)={n_unknown}  "
        f"KNOWN(mock quirk)={n_known}  DUPLICATE={n_violation}"
    )
    if violations:
        table_lines.append("")
        table_lines.append("Violations:")
        table_lines.extend(f"  - {v}" for v in violations)

    table = "\n".join(table_lines)
    log.info("\n%s", table)

    def _safe(s: str) -> str:
        return s.replace("/", "_").replace(" ", "_").replace("\\", "_")

    ap = _safe(info.get("autopilot", "unknown").lower().replace("ardupilotmega", "ardupilot"))
    vt = _safe(info.get("vehicle_type", "unknown").lower())
    ver_raw = info.get("firmware_version", "")
    ver = f"_{_safe(ver_raw)}" if ver_raw and ver_raw != "N/A" else ""
    timestamp = time.strftime("%Y%m%d_%H%M%S")

    logs_dir = Path("logs")
    logs_dir.mkdir(exist_ok=True)
    log_path = logs_dir / f"command_ack_uniqueness_{ap}_{vt}{ver}_{timestamp}.log"

    header = _format_autopilot_header(info, drone_address)
    log_path.write_text(header + "\n\n" + table + "\n")
    log.info("ACK uniqueness table written to %s", log_path)


# ---------------------------------------------------------------------------
# Mock-only: validate the detection helper itself
# ---------------------------------------------------------------------------

# Arbitrary MAV_CMD IDs unused elsewhere in common.xml, reserved for this test.
_NORMAL_CMD_ID = 9001        # baseline: single ACCEPTED ACK, no duplication
_DUPLICATE_CMD_ID = 9002     # mock configured to send the terminal ACK twice
_IN_PROGRESS_CMD_ID = 9003   # mock configured to emit IN_PROGRESS then one terminal ACK


@pytest_asyncio.fixture(scope="class", loop_scope="class")
async def mock_ack_uniqueness(request) -> MockFlightStack | None:
    """
    Mock configured to inject a double-ACK bug for _DUPLICATE_CMD_ID.

    Proves that probe_command_int_all_acks() — the helper the main survey-style
    test above relies on — actually surfaces a duplicate terminal ACK, and that
    IN_PROGRESS ACKs are correctly excluded from the terminal count.  In
    standalone mode (real flight stack), fault injection isn't possible, so
    this fixture yields None and the dependent tests skip.
    """
    drone_address = request.config.getoption("--drone-address")
    if drone_address is not None:
        yield None
        return

    system = System(mavsdk_server_address="localhost", port=DRONE_GRPC_PORT)
    await system.connect()

    stack = MockFlightStack(
        duplicate_command_acks={_DUPLICATE_CMD_ID: 1},
        command_in_progress={_IN_PROGRESS_CMD_ID: [10, 50, 90]},
    )
    task = asyncio.create_task(stack.run(system))
    await asyncio.sleep(0.5)
    yield stack
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass


@pytest_asyncio.fixture(scope="class", loop_scope="class")
async def gcs_system_ack_uniqueness(gcs_mavsdk_server, mock_ack_uniqueness, request) -> System:
    """Class-scoped GCS System for the mock-validation tests."""
    timeout_s = int(request.config.getoption("--connection-timeout"))
    system = System(mavsdk_server_address="localhost", port=gcs_mavsdk_server)
    await system.connect()
    await _wait_for_connection(system, timeout_s)
    if request.config.getoption("--drone-address") is not None:
        await asyncio.sleep(3.0)
    yield system


@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(60)
class TestAckUniquenessDetection:
    """Mock-only: validates that the uniqueness-check machinery works correctly."""

    async def test_single_ack_is_not_flagged(self, gcs_system_ack_uniqueness, mock_ack_uniqueness):
        """A normal command (no fault injection) yields exactly one terminal ACK."""
        if mock_ack_uniqueness is None:
            pytest.skip("Fault injection requires mock")
        acks = await probe_command_int_all_acks(
            gcs_system_ack_uniqueness, command=_NORMAL_CMD_ID, window_s=1.0,
        )
        terminal = [a for a in acks if _is_terminal(a)]
        assert len(terminal) == 1, f"Expected exactly 1 terminal ACK, got {len(terminal)}: {terminal}"

    async def test_in_progress_acks_do_not_count_as_duplicates(
        self, gcs_system_ack_uniqueness, mock_ack_uniqueness
    ):
        """3 IN_PROGRESS ACKs + 1 terminal ACK must classify as 1 terminal, not 4."""
        if mock_ack_uniqueness is None:
            pytest.skip("Fault injection requires mock")
        acks = await probe_command_int_all_acks(
            gcs_system_ack_uniqueness, command=_IN_PROGRESS_CMD_ID, window_s=1.0,
        )
        terminal = [a for a in acks if _is_terminal(a)]
        in_progress = [a for a in acks if not _is_terminal(a)]
        assert len(in_progress) == 3, f"Expected 3 IN_PROGRESS ACKs, got {len(in_progress)}: {in_progress}"
        assert len(terminal) == 1, f"Expected exactly 1 terminal ACK, got {len(terminal)}: {terminal}"

    async def test_duplicate_terminal_ack_is_detected(self, gcs_system_ack_uniqueness, mock_ack_uniqueness):
        """
        The mock is configured with duplicate_command_acks={_DUPLICATE_CMD_ID: 1}
        — a genuine double-ACK bug injection.  This confirms
        probe_command_int_all_acks() actually surfaces it (terminal count == 2),
        i.e. that TestCommandAckUniqueness.test_exactly_one_terminal_ack_per_command
        would correctly fail against a real stack exhibiting this bug.
        """
        if mock_ack_uniqueness is None:
            pytest.skip("Fault injection requires mock")
        acks = await probe_command_int_all_acks(
            gcs_system_ack_uniqueness, command=_DUPLICATE_CMD_ID, window_s=1.0,
        )
        terminal = [a for a in acks if _is_terminal(a)]
        assert len(terminal) == 2, (
            f"Mock injected a duplicate terminal ACK for cmd={_DUPLICATE_CMD_ID}; "
            f"expected the detection helper to see 2 terminal ACKs, got {len(terminal)}: {terminal}"
        )
        log.info(
            "Duplicate-ACK injection correctly detected: cmd=%d produced %d terminal ACKs",
            _DUPLICATE_CMD_ID, len(terminal),
        )
