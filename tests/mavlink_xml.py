"""
Shared MAVLink XML model — the one place the harness parses message definitions.

Used by:
  * ``tests/command/conftest.py`` (``_load_commands`` — survey command list)
  * ``Tier1CommandTestBase`` / ``Tier1MissionTestBase`` (``require_context`` —
    refuses a test for a MAV_CMD the XML doesn't tag for that context)
  * ``scripts/sync_mavlink_xml.py`` (snapshot diffing, spec-drift checks)

Pure stdlib on purpose: the sync script must run without mavsdk/pytest.

Context tags
------------
Upstream ``MAV_CMD`` entries carry boolean attributes saying where the command
may be used: ``mission``, ``command``, ``fence``, ``rally`` (e.g.
``<entry value="22" name="MAV_CMD_NAV_TAKEOFF" ... mission="true" command="true">``).
An absent attribute means "not usable in that context".

An XML older than that convention has *no* entry with any tag.  ``tagging_available()``
detects that so callers can degrade (warn, use every command) instead of
treating "no tags" as "nothing is allowed".
"""

from __future__ import annotations

import os
import warnings
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

CONTEXTS = ("mission", "command", "fence", "rally")

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DEFINITIONS_DIR = REPO_ROOT / "mavlink" / "message_definitions" / "v1.0"

# Dialects searched for MAV_CMD entries.  common.xml is what the survey covers;
# development.xml holds not-yet-standard commands some tests target
# (e.g. EXTERNAL_WIND_ESTIMATE).
COMMAND_DIALECTS = ("common.xml", "development.xml")


def definitions_dir() -> Path:
    """Definitions directory: $MAVLINK_DEFINITIONS_DIR, else the bundled submodule."""
    return Path(os.environ.get("MAVLINK_DEFINITIONS_DIR", DEFAULT_DEFINITIONS_DIR))


@dataclass
class ParamDef:
    index: int
    label: str  # "Empty" for an undefined slot (XML gives it no label)
    empty: bool


@dataclass
class CmdDef:
    id: int
    name: str  # full name, e.g. "MAV_CMD_NAV_TAKEOFF"
    contexts: frozenset[str]
    has_location: bool
    wip: bool
    deprecated: bool
    dialect: str
    params: list[ParamDef] = field(default_factory=list)

    @property
    def short_name(self) -> str:
        return self.name.removeprefix("MAV_CMD_")


def _parse_tree(defs: Path, filename: str, seen: set[str], out: list[ET.ElementTree]) -> None:
    """Depth-first parse of *filename* and its <include>s (includes first)."""
    if filename in seen:
        return
    seen.add(filename)
    path = defs / filename
    if not path.exists():
        return
    tree = ET.parse(path)
    for inc in tree.findall(".//include"):
        if inc.text:
            _parse_tree(defs, inc.text.strip(), seen, out)
    out.append((filename, tree))  # type: ignore[arg-type]


def parse_dialects(defs: Path, roots: tuple[str, ...] = COMMAND_DIALECTS) -> list[tuple[str, ET.ElementTree]]:
    seen: set[str] = set()
    out: list = []
    for root in roots:
        _parse_tree(defs, root, seen, out)
    return out


def _entry_to_cmd(entry: ET.Element, dialect: str) -> CmdDef:
    params = []
    for p in entry.findall("param"):
        label = p.get("label")
        text = (p.text or "").strip()
        empty = (text == "" and label is None) or text == "Empty"
        params.append(ParamDef(int(p.get("index")), label or "Empty", empty))
    return CmdDef(
        id=int(entry.get("value")),
        name=entry.get("name"),
        contexts=frozenset(c for c in CONTEXTS if entry.get(c) == "true"),
        has_location=entry.get("hasLocation") == "true",
        wip=entry.find("wip") is not None,
        deprecated=entry.find("deprecated") is not None,
        dialect=dialect,
        params=params,
    )


def load_command_defs(defs: Path | None = None, roots: tuple[str, ...] = COMMAND_DIALECTS) -> dict[int, CmdDef]:
    """Return {cmd_id: CmdDef} for every MAV_CMD reachable from *roots*."""
    defs = defs or definitions_dir()
    cmds: dict[int, CmdDef] = {}
    for filename, tree in parse_dialects(defs, roots):
        for entry in tree.findall('.//enum[@name="MAV_CMD"]/entry'):
            if entry.get("value") is None or entry.get("name") is None:
                continue
            cmd = _entry_to_cmd(entry, filename)
            cmds[cmd.id] = cmd
    return cmds


def tagging_available(cmds: dict[int, CmdDef]) -> bool:
    """True if the XML uses the mission/command/fence/rally tags at all."""
    return any(c.contexts for c in cmds.values())


def commands_for_context(cmds: dict[int, CmdDef], context: str) -> dict[int, CmdDef]:
    """Commands the XML tags for *context*.  Falls back to all commands (with a
    warning) if the XML predates the tag convention."""
    if not tagging_available(cmds):
        warnings.warn(
            f"MAVLink XML has no mission/command/fence/rally tags (submodule too old?) — "
            f"not filtering by context '{context}'. Run scripts/sync_mavlink_xml.py --update-submodule.",
            stacklevel=2,
        )
        return dict(cmds)
    return {i: c for i, c in cmds.items() if context in c.contexts}


class ContextNotTagged(Exception):
    """A test targets a MAV_CMD the XML does not tag for that context."""


def require_context(cmd_id: int, name: str, context: str) -> None:
    """
    Gate: refuse to build a *context* (mission/command/fence/rally) test for a
    MAV_CMD the XML doesn't tag for it.

    There is deliberately no bypass: if the XML doesn't say the command is
    usable in that context, the test doesn't belong here.
    """
    cmds = load_command_defs()
    if not tagging_available(cmds):
        warnings.warn(
            f"{name}: cannot verify '{context}' tag — XML has no context tags (submodule too old?).",
            stacklevel=3,
        )
        return
    cmd = cmds.get(cmd_id)
    if cmd is None:
        msg = f"{name} (cmd {cmd_id}) is not defined in {', '.join(COMMAND_DIALECTS)}."
    elif context in cmd.contexts:
        return
    else:
        tagged = ", ".join(sorted(cmd.contexts)) or "none"
        msg = (
            f"{cmd.name} (cmd {cmd_id}) is not tagged {context}=\"true\" in the MAVLink XML "
            f"(tagged for: {tagged}). Don't add a {context} test for it — fix the XML upstream "
            f"if the tag is wrong."
        )
    raise ContextNotTagged(msg)


def spec_drift(cmd: CmdDef, spec_params: list[tuple[int, str, bool]], spec_has_location: bool | None) -> list[str]:
    """
    Compare a test SPEC against the XML.  *spec_params* is
    ``[(slot, label, defined), ...]``.  Returns human-readable problems.
    """
    problems = []
    xml = {p.index: p for p in cmd.params}
    spec = {s: (label, defined) for s, label, defined in spec_params}
    for idx in sorted(set(xml) | set(spec)):
        x, s = xml.get(idx), spec.get(idx)
        if x is None:
            # XML may omit Empty params entirely (upstream now does for trailing ones).
            if s[1]:
                problems.append(f"param{idx}: SPEC defines '{s[0]}' but XML doesn't list it")
        elif s is None:
            if not x.empty:
                problems.append(f"param{idx}: in XML ('{x.label}') but missing from SPEC")
        else:
            if s[1] == x.empty:  # SPEC says defined, XML says empty (or vice versa)
                problems.append(f"param{idx}: SPEC defined={s[1]} but XML {'Empty' if x.empty else 'defines it'}")
            if not x.empty and s[0] != x.label:
                problems.append(f"param{idx}: label SPEC '{s[0]}' != XML '{x.label}'")
    if spec_has_location is not None and spec_has_location != cmd.has_location:
        problems.append(f"has_location SPEC={spec_has_location} != XML hasLocation={cmd.has_location}")
    return problems
