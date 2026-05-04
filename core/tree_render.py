"""
Render a kinematic tree as ASCII so users can see joint directions
and topology issues at a glance.

Plugs into:
  - the export log (every successful export — quick sanity check)
  - validation.md (long-form report, when debug is on)
  - the error dialog when validation fails (so the user can spot
    flipped joints, multi-parents, orphans without leaving Fusion)

The renderer flags two common design issues:

  ✱ multi-parent — a child link reachable from two distinct parents.
    URDF requires exactly one parent per link.  Symptom of a closed
    kinematic chain (e.g. a four-bar linkage); the user has to drop
    one joint or use ``<mimic>``.

  ⚠ orphan — a link no joint reaches from the root.  Already caught
    as a hard error in validation, but seeing it in the tree makes
    the missing connection obvious.

Pure helper — no Fusion API, no I/O.

Author: Adrian Valaker Eikeland
"""

from __future__ import annotations

from typing import Dict, List, Tuple


# Visual markers for joint types in the ASCII tree.
_ARROWS = {
    "revolute":   "─⟳─",
    "continuous": "─⟳─",
    "prismatic":  "─↔─",
    "fixed":      "───",
}


def render_tree(model, max_depth: int = 32) -> str:
    """Return an ASCII tree of the kinematic chain rooted at model.root_link.

    Lines look like::

        base_link (430g)
          ─⟳─ servo_joint [revolute]
            servo_mount (190g)
              ─⟳─ left_pivot2 [continuous]
                right_pivot (12g)
                  ─⟳─ left_pivot [continuous]
                    right_gripper (28g)  ✱ multi-parent: left_slider, left_pivot

    With sections at the bottom for orphan links and a topology
    summary so callers can pipe the whole thing straight into a log.
    """
    lines: List[str] = []

    # children_map[parent_urdf] = [(joint_name, child_urdf, joint_type), ...]
    children_map: Dict[str, List[Tuple[str, str, str]]] = {}
    parent_count: Dict[str, List[str]] = {}
    for j in model.joints.values():
        children_map.setdefault(j.parent_link, []).append(
            (j.name, j.child_link, j.joint_type)
        )
        parent_count.setdefault(j.child_link, []).append(j.name)

    def _link_label(name: str) -> str:
        l = model.links.get(name)
        if not l:
            return f"{name} (?)"
        mass_g = l.mass_kg * 1000.0
        merged = " [MERGED]" if getattr(l, "is_merged", False) else ""
        return f"{name} ({mass_g:.0f}g){merged}"

    visited = set()

    def _render(link: str, indent: int):
        if indent > max_depth:
            lines.append("  " * indent + "… (max depth reached)")
            return
        prefix = "  " * indent
        marks = ""
        # Multi-parent marker: this link has more than one joint
        # claiming it as child.  We only flag on the *first* visit so
        # the second-parent edge gets shown without the marker
        # repeating.
        parents = parent_count.get(link, [])
        if len(parents) > 1 and link not in visited:
            other = ", ".join(p for p in parents)
            marks = f"  ✱ multi-parent: {other}"
        if link in visited:
            # Cycle / multi-parent re-visit — show but don't recurse.
            lines.append(f"{prefix}{_link_label(link)}  (revisit)")
            return
        visited.add(link)
        lines.append(f"{prefix}{_link_label(link)}{marks}")
        for jname, child, jtype in children_map.get(link, []):
            arrow = _ARROWS.get(jtype, "───")
            lines.append(f"{prefix}  {arrow} {jname} [{jtype}]")
            _render(child, indent + 2)

    _render(model.root_link, 0)

    # Orphan section — links never visited from the root.
    orphans = set(model.links.keys()) - visited
    if orphans:
        lines.append("")
        lines.append("⚠ Orphan links (no joint reaches them from the root):")
        for o in sorted(orphans):
            lines.append(f"    {o}")

    # Topology summary — count of multi-parent + orphan + total
    multi = [k for k, v in parent_count.items() if len(v) > 1]
    if multi or orphans:
        lines.append("")
        if multi:
            lines.append(f"Closed kinematic chains detected — links with multiple parents:")
            for m in multi:
                lines.append(f"    {m} ← {', '.join(parent_count[m])}")
            lines.append("URDF requires exactly one parent per link.  Drop one joint")
            lines.append("on each multi-parent link, or express the constraint as")
            lines.append("``<mimic joint=\"...\" multiplier=\"...\"/>`` after pickng a")
            lines.append("primary kinematic path.")

    # Closed-loop joints sidecar — joints excluded from the URDF tree
    # but emitted to ``robot_data.yaml`` for downstream platforms (Isaac
    # Sim, etc.) to author as independent physics constraints.
    closing = getattr(model, "closing_joints", None) or {}
    if closing:
        lines.append("")
        lines.append("Closed-loop joints (sidecar) — emitted to robot_data.yaml,")
        lines.append("re-applied by downstream URDF→USD pipelines as USD physics joints:")
        for jname, j in closing.items():
            arrow = _ARROWS.get(j.joint_type, "───")
            src = f" [{j.closing_source}]" if getattr(j, "closing_source", "") else ""
            lines.append(
                f"    {arrow} {jname} [{j.joint_type}]{src}: "
                f"{j.parent_link} → {j.child_link}"
            )

    return "\n".join(lines)
