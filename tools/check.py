"""
Check — Validate a Fusion snapshot and report kinematic chain issues.

Builds the RobotModel from snapshot.json, runs validation, and prints
a clear summary with actionable warnings.

Usage:
    cd fusion2URDF
    python tools/check.py snapshot.json
    python tools/check.py snapshot.json -o validation_report.md

From parent directory:
    python -m fusion2URDF.tools.check path/to/snapshot.json

No external dependencies.

Author: Adrian Valaker Eikeland
"""

import json
import os
import sys
from datetime import datetime

# Handle both standalone and module invocation
_pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_parent = os.path.dirname(_pkg_root)
if _parent not in sys.path:
    sys.path.insert(0, _parent)

from fusion2URDF.core.data_types import (
    FusionSnapshot, FusionOccurrence, FusionJoint,
    InertiaTensor, Transform3D, RigidGroupInfo,
)
from fusion2URDF.core.robot_model import build_model
from fusion2URDF.utils.logger import Logger


# ──────────────────────────────────────────────
# Snapshot loader (JSON dict → dataclass)
# ──────────────────────────────────────────────

def load_snapshot(path: str) -> FusionSnapshot:
    """Load a FusionSnapshot from a JSON file."""
    with open(path) as f:
        data = json.load(f)
    return _snapshot_from_dict(data)


def _snapshot_from_dict(data: dict) -> FusionSnapshot:
    """Reconstruct FusionSnapshot dataclass tree from JSON dict."""
    snap = FusionSnapshot(
        design_name=data.get('design_name', ''),
        design_name_clean=data.get('design_name_clean', ''),
        root_component_name=data.get('root_component_name', ''),
        export_timestamp=data.get('export_timestamp', ''),
        total_occurrences=data.get('total_occurrences', 0),
        total_subassemblies=data.get('total_subassemblies', 0),
        total_leaf_components=data.get('total_leaf_components', 0),
        total_joints=data.get('total_joints', 0),
        total_regular_joints=data.get('total_regular_joints', 0),
        total_as_built_joints=data.get('total_as_built_joints', 0),
        max_nesting_depth=data.get('max_nesting_depth', 0),
    )

    for path, od in data.get('occurrences', {}).items():
        t_data = od.get('local_transform', {})
        t_trans = t_data.get('translation', [0, 0, 0])
        t_rot = t_data.get('rotation', [1, 0, 0, 0, 1, 0, 0, 0, 1])
        if t_rot and isinstance(t_rot[0], list):
            t_rot = tuple(v for row in t_rot for v in row)
        else:
            t_rot = tuple(t_rot) if t_rot else (1, 0, 0, 0, 1, 0, 0, 0, 1)

        i_com = od.get('inertia_at_com', {})
        i_orig = od.get('inertia_at_origin', {})

        occ = FusionOccurrence(
            full_path=od.get('full_path', path),
            component_name=od.get('component_name', ''),
            clean_name=od.get('clean_name', ''),
            path_segments=od.get('path_segments', []),
            depth=od.get('depth', 0),
            is_subassembly=od.get('is_subassembly', False),
            global_position=tuple(od.get('global_position', [0, 0, 0])),
            local_transform=Transform3D(translation=tuple(t_trans), rotation=t_rot),
            assembly_context_depth=od.get('assembly_context_depth', 0),
            mass_kg=od.get('mass_kg', 0),
            body_count=od.get('body_count', 0),
            com_component_local=tuple(od.get('com_component_local', [0, 0, 0])),
            com_global=tuple(od.get('com_global', [0, 0, 0])),
            inertia_at_origin=InertiaTensor(**i_orig) if i_orig else InertiaTensor(),
            inertia_at_com=InertiaTensor(**i_com) if i_com else InertiaTensor(),
            bbox_size=tuple(od.get('bbox_size', [0, 0, 0])),
            volume_m3=od.get('volume_m3', 0),
            area_m2=od.get('area_m2', 0),
            material_name=od.get('material_name', ''),
            appearance_name=od.get('appearance_name', ''),
            appearance_color_rgb=tuple(od['appearance_color_rgb']) if od.get('appearance_color_rgb') else None,
        )
        snap.occurrences[path] = occ

    for jname, jd in data.get('joints', {}).items():
        joint = FusionJoint(
            name=jd.get('name', jname),
            joint_source=jd.get('joint_source', ''),
            defining_component=jd.get('defining_component', ''),
            motion_type=jd.get('motion_type', 'rigid'),
            occurrence_one_path=jd.get('occurrence_one_path', ''),
            occurrence_one_clean=jd.get('occurrence_one_clean', ''),
            occurrence_two_path=jd.get('occurrence_two_path', ''),
            occurrence_two_clean=jd.get('occurrence_two_clean', ''),
            origin_global_m=tuple(jd.get('origin_global_m', [0, 0, 0])),
            origin_source=jd.get('origin_source', ''),
            axis_vector=tuple(jd.get('axis_vector', [0, 0, 1])),
            has_rotation_limits=jd.get('has_rotation_limits', False),
            rotation_min=jd.get('rotation_min'),
            rotation_max=jd.get('rotation_max'),
            has_slide_limits=jd.get('has_slide_limits', False),
            slide_min_m=jd.get('slide_min_m'),
            slide_max_m=jd.get('slide_max_m'),
        )
        snap.joints[jname] = joint

    for rg_data in data.get('rigid_groups', []):
        snap.rigid_groups.append(RigidGroupInfo(
            name=rg_data.get('name', ''),
            occurrence_paths=rg_data.get('occurrence_paths', []),
            member_clean_names=rg_data.get('member_clean_names', []),
            collision_member=rg_data.get('collision_member'),
            collision_path=rg_data.get('collision_path'),
        ))

    return snap


# ──────────────────────────────────────────────
# Report generation
# ──────────────────────────────────────────────

def generate_validation_report(model, snapshot, log) -> str:
    """Generate a Markdown validation report."""
    lines = []
    now = datetime.now().isoformat(timespec='seconds')

    lines.append(f"# Validation Report: {model.name}")
    lines.append(f"")
    lines.append(f"**Generated:** {now}")
    lines.append(f"")

    # Summary
    lines.append(f"## Summary")
    lines.append(f"")
    lines.append(f"| Property | Value |")
    lines.append(f"|----------|-------|")
    lines.append(f"| Robot name | `{model.name}` |")
    lines.append(f"| Root link | `{model.root_link}` |")
    lines.append(f"| Links | {len(model.links)} |")
    lines.append(f"| Joints | {len(model.joints)} |")
    lines.append(f"| Assemblies | {len(model.assemblies)} |")
    lines.append(f"| Warnings | {len(model.warnings)} |")
    lines.append(f"| Errors | {len(model.errors)} |")
    status = "PASS" if len(model.errors) == 0 else "FAIL"
    lines.append(f"| **Status** | **{status}** |")
    lines.append(f"")

    # Kinematic chain
    lines.append(f"## Kinematic Chain")
    lines.append(f"")
    lines.append(f"```")
    # Build tree from root
    children_map = {}
    for j in model.joints.values():
        children_map.setdefault(j.parent_link, []).append((j.name, j.child_link, j.joint_type))

    def print_tree(link, indent=0):
        prefix = "  " * indent
        l = model.links.get(link)
        mass_str = f"{l.mass_kg*1000:.0f}g" if l else "?"
        asm_str = f"[{l.assembly}]" if l else ""
        lines.append(f"{prefix}{link} ({mass_str}) {asm_str}")
        for jname, child, jtype in children_map.get(link, []):
            arrow = {"revolute": "─⟳─", "prismatic": "─↔─", "fixed": "───"}.get(jtype, "───")
            lines.append(f"{prefix}  {arrow} {jname} [{jtype}]")
            print_tree(child, indent + 2)

    print_tree(model.root_link)

    # Orphans
    reachable = set()
    frontier = [model.root_link]
    while frontier:
        current = frontier.pop()
        if current in reachable:
            continue
        reachable.add(current)
        for jname, child, jtype in children_map.get(current, []):
            frontier.append(child)
    orphans = set(model.links.keys()) - reachable
    if orphans:
        lines.append(f"")
        lines.append(f"  ⚠ ORPHANS (not connected to root):")
        for o in sorted(orphans):
            lines.append(f"    {o}")

    lines.append(f"```")
    lines.append(f"")

    # Links table
    lines.append(f"## Links")
    lines.append(f"")
    lines.append(f"| URDF Name | Assembly | Mass (g) | Position X,Y,Z (mm) | Material |")
    lines.append(f"|-----------|----------|----------|---------------------|----------|")
    for name in sorted(model.links.keys()):
        l = model.links[name]
        p = l.global_position
        lines.append(
            f"| `{name}` | {l.assembly} | {l.mass_kg*1000:.1f} | "
            f"({p[0]*1000:.1f}, {p[1]*1000:.1f}, {p[2]*1000:.1f}) | {l.material_name} |"
        )
    lines.append(f"")

    # Joints table
    lines.append(f"## Joints")
    lines.append(f"")
    lines.append(f"| Name | Type | Parent → Child | Origin X,Y,Z (mm) | Axis |")
    lines.append(f"|------|------|----------------|-------------------|------|")
    for name in sorted(model.joints.keys()):
        j = model.joints[name]
        o = j.origin_xyz
        a = j.axis
        lines.append(
            f"| `{name}` | {j.joint_type} | `{j.parent_link}` → `{j.child_link}` | "
            f"({o[0]*1000:.1f}, {o[1]*1000:.1f}, {o[2]*1000:.1f}) | "
            f"({a[0]:.2f}, {a[1]:.2f}, {a[2]:.2f}) |"
        )
    lines.append(f"")

    # Assemblies
    lines.append(f"## Assemblies")
    lines.append(f"")
    for aname in sorted(model.assemblies.keys()):
        a = model.assemblies[aname]
        off = a.global_offset
        lines.append(f"### {aname}")
        lines.append(f"- Offset: ({off[0]*1000:.1f}, {off[1]*1000:.1f}, {off[2]*1000:.1f}) mm")
        lines.append(f"- Links: {', '.join(f'`{l}`' for l in sorted(a.links))}")
        lines.append(f"- Joints: {', '.join(f'`{j}`' for j in sorted(a.joints))}")
        if a.parent_assembly:
            lines.append(f"- Parent assembly: `{a.parent_assembly}`")
        lines.append(f"")

    # Warnings & Errors
    if model.warnings or model.errors:
        lines.append(f"## Issues")
        lines.append(f"")
        for e in model.errors:
            lines.append(f"- ❌ **ERROR:** {e}")
        for w in model.warnings:
            lines.append(f"- ⚠️ **WARNING:** {w}")
        lines.append(f"")

    return "\n".join(lines)


# ──────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────

def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Validate a Fusion snapshot and check kinematic chain integrity."
    )
    parser.add_argument("snapshot", help="Path to snapshot.json")
    parser.add_argument("-o", "--output", help="Save validation report to file (Markdown)")
    parser.add_argument("-q", "--quiet", action="store_true", help="Only show warnings and errors")
    parser.add_argument("-v", "--verbose", action="store_true", help="Show full build log")
    args = parser.parse_args()

    if not os.path.exists(args.snapshot):
        print(f"Error: File not found: {args.snapshot}", file=sys.stderr)
        sys.exit(1)

    # Load
    snapshot = load_snapshot(args.snapshot)
    log = Logger(timestamps=False, quiet=not args.verbose)

    # Build
    model = build_model(snapshot, log)

    if args.verbose:
        print()

    # Print summary
    if not args.quiet:
        print(f"")
        print(f"  Robot:      {model.name}")
        print(f"  Root link:  {model.root_link}")
        print(f"  Links:      {len(model.links)}")
        print(f"  Joints:     {len(model.joints)}")
        print(f"  Assemblies: {len(model.assemblies)}")
        print(f"")
        print(f"  Links: {sorted(model.links.keys())}")
        print(f"")

    # Print issues
    if model.errors:
        for e in model.errors:
            print(f"  ❌ ERROR: {e}")
    if model.warnings:
        for w in model.warnings:
            print(f"  ⚠  {w}")

    if model.errors:
        print(f"\n  RESULT: FAIL ({len(model.errors)} errors, {len(model.warnings)} warnings)")
    elif model.warnings:
        print(f"\n  RESULT: PASS with {len(model.warnings)} warnings")
    else:
        print(f"\n  ✓ RESULT: PASS — no issues detected")

    # Save report
    if args.output:
        report = generate_validation_report(model, snapshot, log)
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(report)
        print(f"\n  Report saved: {args.output}")

    sys.exit(1 if model.errors else 0)


if __name__ == '__main__':
    main()
