#!/usr/bin/env python3
"""
Generate the three-table "Key command support by stack" markdown from survey logs.

Reads all logs/command_survey_*.log files and produces markdown suitable for
pasting into tests/command/README.md § "Key command support by stack".

Usage
-----
    python scripts/generate_command_tables.py [--logs-dir logs/] [--commands CMD_ID,...]

Examples
--------
    # Use all survey logs found in logs/:
    python scripts/generate_command_tables.py

    # Restrict to a custom set of command IDs:
    python scripts/generate_command_tables.py --commands 17,20,21,22,84,85,115,176,178,179,185,192,207,224,300,400,512
"""

import argparse
import re
import sys
from pathlib import Path

# Default set of "key" commands to include in the tables.
# Override with --commands.
_DEFAULT_COMMANDS = [17, 20, 21, 22, 84, 85, 115, 176, 178, 179, 185, 192, 207, 224, 300, 400, 512]

# Regex to parse a result line from the survey log:
#     22  SUPPORTED     MAV_CMD_NAV_TAKEOFF
_LINE_RE = re.compile(r"^\s*(\d+)\s+(SUPPORTED|UNSUPPORTED|UNKNOWN\S*|ERROR\S*)\s+(\S+)")

# Friendly short names (strip MAV_CMD_ prefix for table brevity)
def _short(name: str) -> str:
    return name.removeprefix("MAV_CMD_")


def _stack_label(log_path: Path) -> str:
    """Derive a short stack label from the filename.

    Filename pattern: command_survey_<autopilot>_<vehicle>_<version>_<timestamp>.log
    """
    stem = log_path.stem  # e.g. command_survey_ardupilot_copter_4.8.0-dev_20260526_131342
    parts = stem.split("_")
    # parts[0]="command", parts[1]="survey", parts[2]=autopilot, parts[3]=vehicle, ...
    if len(parts) < 4:
        return stem
    ap = parts[2]
    vehicle = parts[3]
    label_map = {
        ("ardupilot", "copter"): "ArduCopter MC",
        ("ardupilot", "fixed"): "ArduPlane FW",
        ("ardupilot", "quadplane"): "ArduPlane QP",
        ("ardupilot", "rover"): "ArduRover",
        ("px4", "quadcopter"): "PX4 MC",
        ("mock", "mock"): "Mock",
    }
    return label_map.get((ap, vehicle), f"{ap}_{vehicle}")


def _parse_log(log_path: Path) -> dict[int, tuple[str, str]]:
    """Return {cmd_id: (cmd_name, classification)} parsed from a survey log."""
    results: dict[int, tuple[str, str]] = {}
    for line in log_path.read_text(errors="replace").splitlines():
        m = _LINE_RE.match(line)
        if m:
            cmd_id = int(m.group(1))
            classification = m.group(2)
            cmd_name = m.group(3)
            # Normalise unknown variants to "UNKNOWN"
            if not classification.startswith(("SUPPORTED", "UNSUPPORTED")):
                classification = "UNKNOWN"
            results[cmd_id] = (cmd_name, classification)
    return results


def _symbol(classification: str) -> str:
    if classification == "SUPPORTED":
        return "✓"
    if classification == "UNSUPPORTED":
        return "✗"
    return "?"


def _render_table(
    cmd_ids: list[int],
    stack_labels: list[str],
    cmd_names: dict[int, str],
    data: dict[str, dict[int, str]],
) -> str:
    col_widths = [max(len(label), 14) for label in stack_labels]
    header_cells = ["CMD", "ID"] + stack_labels
    sep_cells = ["---", "----"] + ["-" * w for w in col_widths]
    lines = [
        "| " + " | ".join(header_cells) + " |",
        "| " + " | ".join(sep_cells) + " |",
    ]
    for cmd_id in cmd_ids:
        name = _short(cmd_names.get(cmd_id, f"CMD_{cmd_id}"))
        row = [name, str(cmd_id)]
        for label in stack_labels:
            cls = data.get(label, {}).get(cmd_id, "?")
            row.append(_symbol(cls))
        lines.append("| " + " | ".join(row) + " |")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--logs-dir", default="logs", help="Directory containing command_survey_*.log files"
    )
    parser.add_argument(
        "--commands",
        help="Comma-separated list of MAV_CMD IDs to include (default: the 17 key commands)",
    )
    args = parser.parse_args(argv)

    logs_dir = Path(args.logs_dir)
    if not logs_dir.exists():
        sys.exit(f"ERROR: logs directory not found: {logs_dir}")

    # Collect and parse survey logs, keeping only the most recent per stack label.
    log_files = sorted(logs_dir.glob("command_survey_*.log"))
    if not log_files:
        sys.exit(f"ERROR: no command_survey_*.log files found in {logs_dir}")

    # Parse all logs; keep the latest per stack label.
    latest: dict[str, tuple[Path, dict[int, tuple[str, str]]]] = {}
    for lf in log_files:
        label = _stack_label(lf)
        if label not in latest or lf.name > latest[label][0].name:
            latest[label] = (lf, _parse_log(lf))

    # Desired column order (non-mock stacks first if present)
    preferred_order = ["Mock", "ArduCopter MC", "ArduPlane FW", "ArduPlane QP", "ArduRover", "PX4 MC"]
    stack_labels = [l for l in preferred_order if l in latest]
    stack_labels += sorted(s for s in latest if s not in preferred_order)

    if args.commands:
        cmd_ids = [int(c.strip()) for c in args.commands.split(",")]
    else:
        cmd_ids = list(_DEFAULT_COMMANDS)

    # Flatten: {label: {cmd_id: classification}}
    data: dict[str, dict[int, str]] = {}
    cmd_names: dict[int, str] = {}
    for label, (_, results) in latest.items():
        data[label] = {cid: cls for cid, (_, cls) in results.items()}
        for cid, (name, _) in results.items():
            cmd_names.setdefault(cid, name)

    # Categorise commands into three groups.
    #
    # Table 1 (cross-platform): PX4 SUPPORTED AND at least one ArduPilot vehicle SUPPORTED.
    # Table 2 (broadly unsupported): PX4 UNSUPPORTED AND ≥3 real stacks UNSUPPORTED.
    # Table 3 (partial/unclear): everything else.
    real_labels = [l for l in stack_labels if l != "Mock"]
    ardu_labels = [l for l in real_labels if l != "PX4 MC"]

    supported_ids: list[int] = []
    unsupported_ids: list[int] = []
    unknown_ids: list[int] = []

    for cmd_id in cmd_ids:
        px4_supported = data.get("PX4 MC", {}).get(cmd_id) == "SUPPORTED"
        ardu_any = any(data.get(l, {}).get(cmd_id) == "SUPPORTED" for l in ardu_labels)
        u_count = sum(1 for l in real_labels if data.get(l, {}).get(cmd_id) == "UNSUPPORTED")
        majority = len(real_labels) // 2 + 1

        if px4_supported and ardu_any:
            supported_ids.append(cmd_id)
        elif not px4_supported and u_count >= majority:
            unsupported_ids.append(cmd_id)
        else:
            unknown_ids.append(cmd_id)

    # Render output
    out: list[str] = []
    out.append("### Key command support by stack")
    out.append("")
    out.append("**Key:** ✓ = SUPPORTED  ✗ = UNSUPPORTED  ? = UNKNOWN (no ACK within timeout)")
    out.append("")

    out.append("#### Commands supported cross-platform (PX4 + ArduPilot)")
    out.append("")
    out.append(_render_table(supported_ids, stack_labels, cmd_names, data))
    out.append("")

    out.append("#### Commands unsupported by PX4 and most ArduPilot stacks")
    out.append("")
    out.append(_render_table(unsupported_ids, stack_labels, cmd_names, data))
    out.append("")

    out.append("#### Commands with partial or unclear cross-platform support")
    out.append("")
    out.append(_render_table(unknown_ids, stack_labels, cmd_names, data))
    out.append("")

    out.append(f"Sources: {', '.join(lf.name for lf, _ in latest.values())}")

    print("\n".join(out))


if __name__ == "__main__":
    main()
