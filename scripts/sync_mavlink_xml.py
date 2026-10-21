#!/usr/bin/env python3
"""
Keep the tests in sync with the MAVLink XML.

What it does
------------
1. (optional) ``--update-submodule [REF]`` — fetch and check out the ``mavlink``
   submodule at REF (default ``origin/master``).  Leaves the new pointer
   unstaged for you to review and commit.
2. Diffs the current XML against the committed snapshot
   (``snapshots/mavlink_xml.json``) and reports what changed since the tests
   were last reviewed:
     * MAV_CMDs added / removed / renamed
     * ``mission``/``command``/``fence``/``rally`` tag changes
     * MAV_CMD param additions/label changes
     * message fields added — *extension fields* called out separately —
       and enum values added, for messages/enums the tests actually reference
3. Checks the tests against the XML (blocking):
     * every Tier 1 SPEC's params/labels/hasLocation match the XML
     * every ``tests/mission/*`` / ``tests/command/*`` SPEC targets a command
       the XML tags for that context (no exemptions)
     * ``MAV_FRAME_CATALOGUE`` (tests/param_spec.py) matches the XML's MAV_FRAME
4. Lists tagged commands that have no test yet (``--coverage`` for full list).
5. ``--accept`` rewrites the snapshot once you've dealt with the report.

Exit status: 0 = clean, 1 = drift or blocking problems.  Suitable for CI.

Usage
-----
    python scripts/sync_mavlink_xml.py                     # report
    python scripts/sync_mavlink_xml.py --update-submodule  # pull latest XML, then report
    python scripts/sync_mavlink_xml.py --accept            # record current XML as reviewed
    python scripts/sync_mavlink_xml.py --defs DIR          # use another definitions dir
"""

import argparse
import importlib
import json
import subprocess
import sys
import warnings
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from tests import mavlink_xml as mx  # noqa: E402  (pure stdlib)

SNAPSHOT = REPO / "snapshots" / "mavlink_xml.json"
# Enums whose values the tests mirror or interpret.
WATCHED_ENUMS = ("MAV_FRAME", "MAV_RESULT", "MAV_MISSION_RESULT", "MAV_MISSION_TYPE", "MAV_PROTOCOL_CAPABILITY")
# (context, test-file glob pattern) — where Tier 1 SPECs live.
SPEC_MODULES = {"command": "test_command", "mission": "test_protocol"}


# ---------------------------------------------------------------- snapshot --

def build_snapshot(defs: Path) -> dict:
    cmds = mx.load_command_defs(defs)
    messages: dict[str, list[str]] = {}
    enums: dict[str, dict[str, int]] = {}
    for _, tree in mx.parse_dialects(defs):
        for m in tree.findall(".//messages/message"):
            fields, ext = [], False
            for child in m:
                if child.tag == "extensions":
                    ext = True
                elif child.tag == "field":
                    fields.append(f"{child.get('name')}:{child.get('type')}{'*' if ext else ''}")
            messages[m.get("name")] = fields  # '*' suffix = extension field
        for e in tree.findall(".//enums/enum"):
            if e.get("name") in WATCHED_ENUMS:
                enums.setdefault(e.get("name"), {}).update(
                    {x.get("name"): int(x.get("value")) for x in e.findall("entry") if x.get("value")}
                )
    return {
        "mavlink_commit": _submodule_commit() if defs.resolve() == mx.DEFAULT_DEFINITIONS_DIR.resolve() else f"custom:{defs}",
        "commands": {
            str(c.id): {
                "name": c.name,
                "contexts": sorted(c.contexts),
                "hasLocation": c.has_location,
                "wip": c.wip,
                "deprecated": c.deprecated,
                "params": [f"{p.index}:{p.label}" for p in c.params if not p.empty],  # XML may omit Empty params
            }
            for c in sorted(cmds.values(), key=lambda c: c.id)
        },
        "messages": messages,
        "enums": enums,
    }


def _submodule_commit() -> str:
    r = subprocess.run(["git", "-C", str(REPO / "mavlink"), "rev-parse", "--short", "HEAD"],
                       capture_output=True, text=True)
    return r.stdout.strip() or "unknown"


def watched_names(names) -> set[str]:
    """Names (messages/enums) that appear literally in the test sources."""
    src = "\n".join(p.read_text(errors="ignore") for p in (REPO / "tests").rglob("*.py"))
    return {n for n in names if n in src}


def diff_snapshots(old: dict, new: dict) -> tuple[list[str], list[str]]:
    """Returns (action_items, info_lines)."""
    act, info = [], []
    oc, nc = old["commands"], new["commands"]
    for i in sorted(set(nc) - set(oc), key=int):
        c = nc[i]
        act.append(f"NEW command {c['name']} ({i}) tagged {c['contexts'] or 'none'}"
                   f"{' [wip]' if c['wip'] else ''} — consider tests")
    for i in sorted(set(oc) - set(nc), key=int):
        act.append(f"REMOVED command {oc[i]['name']} ({i}) — remove/retarget its tests")
    old_tagged = any(c["contexts"] for c in oc.values())
    if not old_tagged and any(c["contexts"] for c in nc.values()):
        act.append(f"XML now carries mission/command/fence/rally tags "
                   f"({sum(bool(c['contexts']) for c in nc.values())} commands tagged) — "
                   f"survey and test gating now active")
    for i in sorted(set(oc) & set(nc), key=int):
        o, n = oc[i], nc[i]
        for key in ("name", "contexts", "hasLocation", "wip", "deprecated", "params"):
            if key == "contexts" and not old_tagged:
                continue  # summarised above
            if o[key] != n[key]:
                act.append(f"CHANGED {n['name']} ({i}) {key}: {o[key]} -> {n[key]}")
    om, nm = old["messages"], new["messages"]
    watched = watched_names(set(nm) | set(om))
    for name in sorted(set(nm) - set(om)):
        (act if name in watched else info).append(f"NEW message {name}")
    for name in sorted(set(nm) & set(om)):
        if om[name] != nm[name]:
            added = [f for f in nm[name] if f not in om[name]]
            gone = [f for f in om[name] if f not in nm[name]]
            ext = [f.rstrip("*") for f in added if f.endswith("*")]
            line = f"message {name}: +{[f.rstrip('*') for f in added]} -{gone}"
            if ext:
                line += f"  (EXTENSION fields added: {ext})"
            (act if name in watched else info).append(line)
    for name, vals in new["enums"].items():
        ov = old["enums"].get(name, {})
        for k in sorted(set(vals) - set(ov)):
            act.append(f"NEW {name} value {k}={vals[k]}")
        for k in sorted(set(ov) - set(vals)):
            act.append(f"REMOVED {name} value {k}")
        for k in sorted(set(ov) & set(vals)):
            if ov[k] != vals[k]:
                act.append(f"CHANGED {name}.{k}: {ov[k]} -> {vals[k]}")
    return act, info


# ------------------------------------------------------- tests vs. the XML --

def load_specs() -> tuple[list[tuple[str, str, object]], list[str]]:
    """Import each Tier 1 test module; return ([(context, module, SPEC)], problems)."""
    specs, problems = [], []
    warnings.simplefilter("ignore")
    for context, fname in SPEC_MODULES.items():
        for path in sorted((REPO / "tests" / context).glob(f"*/{fname}.py")):
            mod = f"tests.{context}.{path.parent.name}.{fname}"
            try:
                m = importlib.import_module(mod)
            except mx.ContextNotTagged as e:
                problems.append(f"{mod}: {e}")
                continue
            except Exception as e:  # noqa: BLE001 — report and carry on
                problems.append(f"{mod}: could not import ({type(e).__name__}: {e})")
                continue
            if hasattr(m, "SPEC"):
                specs.append((context, mod, m.SPEC))
    return specs, problems


def check_tests(cmds: dict[int, "mx.CmdDef"], snapshot_new: dict) -> tuple[list[str], set[tuple[str, int]]]:
    problems: list[str] = []
    specs, import_problems = load_specs()
    problems += import_problems
    tested: set[tuple[str, int]] = set()
    tagged = mx.tagging_available(cmds)
    for context, mod, spec in specs:
        tested.add((context, spec.cmd_id))
        cmd = cmds.get(spec.cmd_id)
        if cmd is None:
            problems.append(f"{mod}: cmd {spec.cmd_id} ({spec.name}) not in XML")
            continue
        if tagged and context not in cmd.contexts:
            problems.append(f"{mod}: {cmd.name} not tagged {context}=\"true\" in the XML — remove this test")
        for d in mx.spec_drift(cmd, [(p.slot, p.label, p.defined) for p in spec.params],
                               getattr(spec, "has_location", None)):
            problems.append(f"{mod}: {cmd.name} {d}")
    # MAV_FRAME_CATALOGUE vs XML
    from tests.param_spec import MAV_FRAME_CATALOGUE
    xml_frames = snapshot_new["enums"].get("MAV_FRAME", {})
    if xml_frames:
        cat = {n: v for v, n in MAV_FRAME_CATALOGUE}
        for n in sorted(set(xml_frames) - set(cat)):
            problems.append(f"tests/param_spec.py MAV_FRAME_CATALOGUE missing {n}={xml_frames[n]}")
        for n in sorted(set(cat) - set(xml_frames)):
            problems.append(f"tests/param_spec.py MAV_FRAME_CATALOGUE has {n}, not in XML")
        for n in sorted(set(cat) & set(xml_frames)):
            if cat[n] != xml_frames[n]:
                problems.append(f"MAV_FRAME_CATALOGUE {n}: {cat[n]} != XML {xml_frames[n]}")
    return problems, tested


def report_coverage(cmds, tested, full: bool) -> list[str]:
    lines = []
    if not mx.tagging_available(cmds):
        return ["(coverage skipped: XML has no context tags)"]
    for ctx in mx.CONTEXTS:
        tagged = mx.commands_for_context(cmds, ctx)
        missing = sorted(c.short_name for i, c in tagged.items() if (ctx, i) not in tested)
        # Tier 1 SPEC coverage exists for mission/command only.
        if ctx in SPEC_MODULES or tagged:
            lines.append(f"{ctx}: {len(tagged) - len(missing)}/{len(tagged)} tagged commands have a "
                         f"Tier 1 test dir" + ("" if ctx in SPEC_MODULES else " (no harness for this context yet)"))
            if full and missing:
                lines.append("    untested: " + ", ".join(missing))
    return lines


# --------------------------------------------------------------------- main --

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--defs", type=Path, default=mx.DEFAULT_DEFINITIONS_DIR)
    ap.add_argument("--update-submodule", nargs="?", const="origin/master", metavar="REF")
    ap.add_argument("--accept", action="store_true", help="write the snapshot from the current XML")
    ap.add_argument("--coverage", action="store_true", help="list every tagged command without a test")
    args = ap.parse_args()

    if args.update_submodule:
        sub = ["git", "-C", str(REPO / "mavlink")]
        subprocess.run(sub + ["fetch", "origin"], check=True)
        subprocess.run(sub + ["checkout", "--quiet", args.update_submodule], check=True)
        print(f"mavlink submodule now at {_submodule_commit()} (unstaged; review and commit the pointer)")

    import os
    os.environ["MAVLINK_DEFINITIONS_DIR"] = str(args.defs.resolve())  # used by require_context
    new = build_snapshot(args.defs)
    cmds = mx.load_command_defs(args.defs)
    rc = 0

    if not SNAPSHOT.exists():
        print(f"No snapshot at {SNAPSHOT.relative_to(REPO)} — run with --accept to create it.")
        rc = 1
    else:
        old = json.loads(SNAPSHOT.read_text())
        act, info = diff_snapshots(old, new)
        print(f"== XML changes since snapshot ({old['mavlink_commit']} -> {new['mavlink_commit']}) ==")
        print("\n".join(f"  ! {a}" for a in act) or "  none")
        if info:
            print(f"  ({len(info)} other message change(s) not referenced by tests:)")
            print("\n".join(f"    - {i}" for i in info))
        rc |= bool(act)

    problems, tested = check_tests(cmds, new)
    print("\n== Tests vs. XML ==")
    print("\n".join(f"  X {p}" for p in problems) or "  ok")
    rc |= bool(problems)

    print("\n== Coverage ==")
    print("\n".join("  " + l for l in report_coverage(cmds, tested, args.coverage)))

    if args.accept:
        SNAPSHOT.parent.mkdir(exist_ok=True)
        SNAPSHOT.write_text(json.dumps(new, indent=1, sort_keys=True) + "\n")
        print(f"\nSnapshot written: {SNAPSHOT.relative_to(REPO)}")
        return int(bool(problems))
    return int(rc)


if __name__ == "__main__":
    sys.exit(main())
