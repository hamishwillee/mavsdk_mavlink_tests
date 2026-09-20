#!/usr/bin/env python3
"""
Build a Supported / Unsupported command list per platform from survey logs
(logs/command_survey_*.log, written by tests/command/test_survey.py).

Unsupported: both COMMAND_INT and COMMAND_LONG NACKed UNSUPPORTED.
Supported:   at least one of them did not NACK UNSUPPORTED.  A note is added
             when either message type returned UNSUPPORTED, COMMAND_LONG_ONLY or
             COMMAND_INT_ONLY, or when one message type got no ACK at all; a literal
             UNSUPPORTED also gets a FAIL note giving the NACK the stack should
             have sent.  Other differences (e.g. IN_PROGRESS vs TEMPORARILY_REJECTED)
             are expected and not reported.
(Commands that never ACKed either message type are "Unknown" — silence is not a NACK, so
they are listed separately, in neither Supported nor Unsupported.)

Usage: python scripts/generate_support_report.py OUT.txt LOG [LOG ...]
"""
import re
import sys
from pathlib import Path

_ROW = re.compile(r"^\s*(\d+)\s+(SUPPORTED|UNSUPPORTED|UNKNOWN\S*)\s+(MAV_CMD_\S+)\s+(.*)$")
_PAIR = re.compile(r"(?:COMMAND_)?(INT|LONG)=(\S+)")
_ONLY = {"COMMAND_LONG_ONLY", "COMMAND_INT_ONLY"}


def _disp(r: str) -> str:
    return f"MAV_RESULT_{r}" if r in _ONLY else r


def parse(log: Path):
    text = log.read_text(errors="replace")
    ver = re.search(r"Firmware version:\s*(.*)", text).group(1)
    vt = re.search(r"Vehicle type:\s*(.*)", text).group(1)
    ap = re.search(r"Autopilot:\s*(.*)", text).group(1)
    rows = []
    for line in text.splitlines():
        m = _ROW.match(line)
        if not m:
            continue
        cid, _, name, raw = m.groups()
        pairs = dict(_PAIR.findall(raw))
        i, l = (pairs["INT"], pairs["LONG"]) if pairs else (raw.strip(),) * 2
        rows.append((int(cid), name, i, l))
    return f"{ap} {vt}", ver, rows


def main(out: str, logs: list[str]) -> None:
    lines = ["MAV_CMD support - Supported / Unsupported lists", ""]
    for log in logs:
        plat, ver, rows = parse(Path(log))
        lines.append(f"- {plat}: {ver}")
    for log in logs:
        plat, ver, rows = parse(Path(log))
        uns = [r for r in rows if r[2] == "UNSUPPORTED" and r[3] == "UNSUPPORTED"]
        silent = [r for r in rows if r[2] == "NO_ACK" and r[3] == "NO_ACK"]
        sup = [r for r in rows if r not in uns and r not in silent]
        lines += ["", "", f"{plat} - {ver}", "=" * 72, "", f"Unsupported ({len(uns)})", "-" * 40]
        lines += [f"{c:>6}  {n}" for c, n, _, _ in uns]
        lines += ["", f"Supported ({len(sup)})", "-" * 40]
        for c, n, i, l in sup:
            s = f"{c:>6}  {n:<40}"
            if i != l and ({i, l} & (_ONLY | {"UNSUPPORTED", "NO_ACK"})):
                s += f" (INT={_disp(i)} / LONG={_disp(l)})"
                if i == "UNSUPPORTED":
                    s += " - NOTE: FAIL: Should NACK MAV_RESULT_COMMAND_LONG_ONLY"
                elif l == "UNSUPPORTED":
                    s += " - NOTE: FAIL: Should NACK MAV_RESULT_COMMAND_INT_ONLY"
            lines.append(s.rstrip())
        lines += ["", f"Unknown - no ACK to either message type ({len(silent)})", "-" * 40]
        lines += [f"{c:>6}  {n}" for c, n, _, _ in silent]
    Path(out).write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:])
