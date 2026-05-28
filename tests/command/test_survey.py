"""
MAV_CMD support survey — probes every command in common.xml via COMMAND_INT
and records the ACK result (or absence thereof) in a support matrix table.

Purpose
-------
Produces a per-flight-stack support matrix: SUPPORTED / UNSUPPORTED / UNKNOWN.
This table is the precondition for skipping detailed tests: if a command is
UNSUPPORTED on a given stack, all its per-command tests can be skipped.

This test always PASSes — results are observational and written to logs/.

Classification
--------------
  ACCEPTED (0)               → SUPPORTED
  TEMPORARILY_REJECTED (1)   → SUPPORTED (recognised, stack busy)
  DENIED (2)                 → SUPPORTED (recognised, refused params/state)
  UNSUPPORTED (3)            → UNSUPPORTED
  FAILED (4)                 → SUPPORTED (recognised, execution failed)
  IN_PROGRESS (5)            → SUPPORTED (executing)
  CANCELLED (6)              → SUPPORTED (was executing)
  COMMAND_LONG_ONLY (7)      → SUPPORTED (stack requires COMMAND_LONG)
  COMMAND_INT_ONLY (8)       → SUPPORTED (stack requires COMMAND_INT)
  Any other value            → UNKNOWN
  No ACK within timeout      → UNKNOWN (spec violation; may still be supported)

XML source
----------
Parsed from ``--mavlink-definitions-dir`` (default: mavlink/message_definitions/v1.0/).
Uses the MAVLink git submodule (common.xml → standard.xml → minimal.xml).

Running
-------
Against the mock (all commands return ACCEPTED by default)::

    pytest tests/command/test_survey.py -v --log-cli-level=INFO

Against a real flight stack::

    pytest tests/command/test_survey.py --drone-address=udp://:14540 -v --log-cli-level=INFO
"""

import asyncio
import logging
import time
from pathlib import Path

import pytest

from .conftest import (
    _load_commands,
    probe_command_int,
    INT32_MAX,
)
from tests.conftest import _format_autopilot_header
from tests.mock_flight_stack import (
    MAV_RESULT_ACCEPTED,
    MAV_RESULT_TEMPORARILY_REJECTED,
    MAV_RESULT_DENIED,
    MAV_RESULT_UNSUPPORTED,
    MAV_RESULT_FAILED,
    MAV_RESULT_IN_PROGRESS,
    MAV_RESULT_CANCELLED,
    MAV_RESULT_COMMAND_LONG_ONLY,
    MAV_RESULT_COMMAND_INT_ONLY,
)

log = logging.getLogger(__name__)

_SURVEY_ACK_TIMEOUT_S = 2.0  # short timeout per probe — survey only

_SUPPORTED_RESULTS = frozenset({
    MAV_RESULT_ACCEPTED,
    MAV_RESULT_TEMPORARILY_REJECTED,
    MAV_RESULT_DENIED,
    MAV_RESULT_FAILED,
    MAV_RESULT_IN_PROGRESS,
    MAV_RESULT_CANCELLED,
    MAV_RESULT_COMMAND_LONG_ONLY,
    MAV_RESULT_COMMAND_INT_ONLY,
})


def _classify(ack: dict | None) -> str:
    if ack is None:
        return "UNKNOWN"
    result = int(ack.get("result", -1))
    if result == MAV_RESULT_UNSUPPORTED:
        return "UNSUPPORTED"
    if result in _SUPPORTED_RESULTS:
        return "SUPPORTED"
    return f"UNKNOWN(result={result})"


@pytest.mark.asyncio(loop_scope="class")
@pytest.mark.timeout(900)  # 168 commands × up to 5 s ACK timeout each
class TestCommandSurvey:
    """Probe all MAV_CMD from common.xml and record the ACK result."""

    async def test_survey_all_commands(self, gcs_system_cls, mock_stack_cls, request):
        """
        Probe every MAV_CMD with COMMAND_INT and build a support matrix table.

        Always passes — the goal is to record which commands are supported,
        unsupported, or unknown for the current flight stack.
        """
        definitions_dir = Path(request.config.getoption("--mavlink-definitions-dir"))
        commands = _load_commands(definitions_dir)
        if not commands:
            pytest.skip(f"No MAV_CMD entries found in {definitions_dir}/common.xml — "
                        "run: git submodule update --init mavlink")

        log.info("Surveying %d MAV_CMD entries from %s", len(commands), definitions_dir)

        results: dict[int, tuple[str, str]] = {}
        for cmd_id, cmd_name in sorted(commands.items()):
            try:
                ack = await probe_command_int(
                    gcs_system_cls,
                    command=cmd_id,
                    frame=6,       # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                    param1=0.0,
                    param2=0.0,
                    param3=0.0,
                    param4=0.0,
                    x=INT32_MAX,   # "use current position" sentinel for lat
                    y=INT32_MAX,   # "use current position" sentinel for lon
                    z=0.0,
                    timeout_s=_SURVEY_ACK_TIMEOUT_S,
                )
                classification = _classify(ack)
                ack_result = int(ack["result"]) if ack is not None else None
            except Exception as exc:
                classification = f"ERROR({exc})"
                ack_result = None

            results[cmd_id] = (cmd_name, classification)
            log.info(
                "%-45s (cmd=%4d) → %s%s",
                cmd_name, cmd_id, classification,
                f" [result={ack_result}]" if ack_result is not None and classification not in ("SUPPORTED", "UNSUPPORTED") else "",
            )

        info = getattr(request.config, "_autopilot_info", {})
        drone_address = request.config.getoption("--drone-address")
        _log_survey_table(results, info, drone_address)

    # Note: no assertions — this test always passes regardless of results


def _log_survey_table(
    results: dict[int, tuple[str, str]],
    info: dict,
    drone_address: str | None,
) -> None:
    """Log and write the survey results table, prefixed with the stack probe header."""
    table_lines = [
        "Command support survey results",
        "=" * 72,
        f"{'CMD ID':>6}  {'Result':<12}  {'Command Name'}",
        "-" * 72,
    ]
    counts = {"SUPPORTED": 0, "UNSUPPORTED": 0, "UNKNOWN": 0}
    for cmd_id, (cmd_name, classification) in sorted(results.items()):
        table_lines.append(f"{cmd_id:>6}  {classification:<12}  {cmd_name}")
        key = "UNKNOWN" if classification not in ("SUPPORTED", "UNSUPPORTED") else classification
        counts[key] += 1

    table_lines.append("-" * 72)
    table_lines.append(
        f"Total: {len(results)}  "
        f"SUPPORTED={counts['SUPPORTED']}  "
        f"UNSUPPORTED={counts['UNSUPPORTED']}  "
        f"UNKNOWN={counts['UNKNOWN']}"
    )

    table = "\n".join(table_lines)
    log.info("\n%s", table)

    # Build log filename: command_survey_<autopilot>_<vehicle>_<version>_<timestamp>.log
    def _safe(s: str) -> str:
        return s.replace("/", "_").replace(" ", "_").replace("\\", "_")

    ap = _safe(info.get("autopilot", "unknown").lower().replace("ardupilotmega", "ardupilot"))
    vt = _safe(info.get("vehicle_type", "unknown").lower())
    ver_raw = info.get("firmware_version", "")
    ver = f"_{_safe(ver_raw)}" if ver_raw and ver_raw != "N/A" else ""
    timestamp = time.strftime("%Y%m%d_%H%M%S")

    logs_dir = Path("logs")
    logs_dir.mkdir(exist_ok=True)
    log_path = logs_dir / f"command_survey_{ap}_{vt}{ver}_{timestamp}.log"

    header = _format_autopilot_header(info, drone_address)
    log_path.write_text(header + "\n\n" + table + "\n")
    log.info("Survey table written to %s", log_path)
