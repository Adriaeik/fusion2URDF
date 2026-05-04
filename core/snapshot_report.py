"""
Snapshot Report — Generate readable Markdown from FusionSnapshot.

Produces a detailed report you can compare side-by-side with
Fusion 360's Properties panel to verify extraction accuracy.

Author: Adrian Valaker Eikeland
"""

from .data_types import FusionSnapshot, FusionOccurrence, FusionJoint
from ..utils import fmt, fmt_vec3


def generate_report(snapshot: FusionSnapshot) -> str:
    """Generate complete Markdown report from a FusionSnapshot."""
    
    lines = []
    w = lines.append  # shorthand
    
    w(f"# Extraction Report: {snapshot.design_name}")
    w(f"")
    w(f"**Exported:** {snapshot.export_timestamp}")
    w(f"**Exporter:** v{snapshot.exporter_version}")
    w(f"")
    w(f"## Summary")
    w(f"")
    w(f"| Metric | Value |")
    w(f"|--------|-------|")
    w(f"| Occurrences | {snapshot.total_occurrences} |")
    w(f"| Subassemblies | {snapshot.total_subassemblies} |")
    w(f"| Leaf components | {snapshot.total_leaf_components} |")
    w(f"| Joints (total) | {snapshot.total_joints} |")
    w(f"| As-built joints | {snapshot.total_as_built_joints} |")
    w(f"| Regular joints | {snapshot.total_regular_joints} |")
    w(f"| Max nesting depth | {snapshot.max_nesting_depth} |")
    w(f"")
    
    # ── Assembly hierarchy ──
    w(f"## Assembly Hierarchy")
    w(f"")
    w(f"```")
    _write_hierarchy_tree(snapshot, lines)
    w(f"```")
    w(f"")
    
    # ── Occurrences ──
    w(f"## Occurrences")
    w(f"")
    
    # Group by depth for readability
    by_depth = {}
    for occ in snapshot.occurrences.values():
        by_depth.setdefault(occ.depth, []).append(occ)
    
    for depth in sorted(by_depth.keys()):
        occs = sorted(by_depth[depth], key=lambda o: o.full_path)
        w(f"### Depth {depth}")
        w(f"")
        
        for occ in occs:
            _write_occurrence(occ, lines)
    
    # ── Joints ──
    w(f"## Joints")
    w(f"")
    
    for jname in sorted(snapshot.joints.keys()):
        joint = snapshot.joints[jname]
        _write_joint(joint, lines)
    
    # ── Comparison Table ──
    # Quick reference table for comparing with Fusion Properties panel
    w(f"## Quick Comparison Table")
    w(f"")
    w(f"Compare these values with Fusion 360 Properties panel (right-click → Properties).")
    w(f"")
    w(f"| Component | Mass (g) | World X,Y,Z (mm) | CoM X,Y,Z (mm) | Material |")
    w(f"|-----------|----------|-------------------|-----------------|----------|")
    
    for occ in sorted(snapshot.occurrences.values(), key=lambda o: o.full_path):
        if occ.is_subassembly:
            continue
        
        mass_g = occ.mass_kg * 1000.0
        # World position in mm (what Fusion shows)
        wx = occ.global_position[0] * 1000.0
        wy = occ.global_position[1] * 1000.0
        wz = occ.global_position[2] * 1000.0
        # CoM in global mm (what Fusion shows as "Center of Mass")
        cx = occ.com_global[0] * 1000.0
        cy = occ.com_global[1] * 1000.0
        cz = occ.com_global[2] * 1000.0
        
        w(f"| {occ.clean_name} | {mass_g:.3f} | "
          f"({wx:.2f}, {wy:.2f}, {wz:.2f}) | "
          f"({cx:.2f}, {cy:.2f}, {cz:.2f}) | "
          f"{occ.material_name} |")
    
    w(f"")
    
    # ── Joint origins comparison ──
    w(f"## Joint Origins Comparison")
    w(f"")
    w(f"All origins shown in multiple coordinate systems for debugging.")
    w(f"")
    w(f"| Joint | Source | Origin (cm, raw) | Origin (m, picked) | Motion | Axis |")
    w(f"|-------|--------|------------------|-------------------|--------|------|")
    
    for jname in sorted(snapshot.joints.keys()):
        j = snapshot.joints[jname]
        
        # Show raw cm value that was picked
        raw_cm = ""
        if j.geometry_origin_cm:
            raw_cm = f"geo({j.geometry_origin_cm[0]:.2f}, {j.geometry_origin_cm[1]:.2f}, {j.geometry_origin_cm[2]:.2f})"
        elif j.geometry_or_origin_one_cm:
            raw_cm = f"goo1({j.geometry_or_origin_one_cm[0]:.2f}, {j.geometry_or_origin_one_cm[1]:.2f}, {j.geometry_or_origin_one_cm[2]:.2f})"
        elif j.occ_one_global_cm:
            raw_cm = f"ctx({j.occ_one_global_cm[0]:.2f}, {j.occ_one_global_cm[1]:.2f}, {j.occ_one_global_cm[2]:.2f})"
        
        picked = f"({j.origin_global_m[0]:.4f}, {j.origin_global_m[1]:.4f}, {j.origin_global_m[2]:.4f})"
        axis = f"({j.axis_vector[0]:.1f}, {j.axis_vector[1]:.1f}, {j.axis_vector[2]:.1f})"
        
        w(f"| {j.name} | {j.origin_source} | {raw_cm} | {picked} | {j.motion_type} | {axis} |")
    
    w(f"")
    
    return "\n".join(lines)


def _write_hierarchy_tree(snapshot: FusionSnapshot, lines: list):
    """Write ASCII tree of assembly hierarchy."""
    
    # Find subassemblies and their parent paths
    subassemblies = {}
    for occ in snapshot.occurrences.values():
        if occ.is_subassembly:
            subassemblies[occ.clean_name] = occ
    
    # Find leaf components per parent
    children_of = {}  # parent_clean_name → [child occurrences]
    for occ in snapshot.occurrences.values():
        if occ.is_subassembly:
            continue
        # Find parent: last subassembly in path segments
        parent = _find_parent_assembly(occ, subassemblies)
        children_of.setdefault(parent or "ROOT", []).append(occ)
    
    # Assembly parent→child relationships
    asm_children = {}  # parent_clean_name → [child assembly names]
    for occ in subassemblies.values():
        parent = _find_parent_assembly(occ, subassemblies)
        asm_children.setdefault(parent or "ROOT", []).append(occ.clean_name)
    
    # Recursive print
    def print_tree(name, indent=""):
        asm = subassemblies.get(name)
        if asm:
            lines.append(f"{indent}[{name}]  (depth={asm.depth}, children={asm.child_count})")
        else:
            lines.append(f"{indent}[{name}]  (root)")
        
        # Print leaf components
        for child in sorted(children_of.get(name, []), key=lambda o: o.clean_name):
            mass_g = child.mass_kg * 1000.0
            lines.append(f"{indent}  ├── {child.clean_name}  ({mass_g:.1f}g, {child.material_name})")
        
        # Print child assemblies
        for child_name in sorted(asm_children.get(name, [])):
            print_tree(child_name, indent + "  ")
    
    # Start from root-level assemblies
    root_asms = asm_children.get("ROOT", [])
    root_leaves = children_of.get("ROOT", [])
    
    lines.append(f"[{snapshot.design_name_clean}]  (design root)")
    for leaf in sorted(root_leaves, key=lambda o: o.clean_name):
        mass_g = leaf.mass_kg * 1000.0
        lines.append(f"  ├── {leaf.clean_name}  ({mass_g:.1f}g)")
    for name in sorted(root_asms):
        print_tree(name, "  ")


def _find_parent_assembly(occ: FusionOccurrence, subassemblies: dict):
    """Find the deepest subassembly that contains this occurrence."""
    # Walk path segments from deepest to shallowest
    for i in range(len(occ.path_segments) - 2, -1, -1):
        seg = occ.path_segments[i]
        if seg in subassemblies and seg != occ.clean_name:
            return seg
    return None


def _write_occurrence(occ: FusionOccurrence, lines: list):
    """Write detailed occurrence info."""
    w = lines.append
    
    tag = "📦 SUBASSEMBLY" if occ.is_subassembly else "🔧 COMPONENT"
    w(f"#### {tag}: `{occ.clean_name}`")
    w(f"")
    w(f"| Property | Value |")
    w(f"|----------|-------|")
    w(f"| Full path | `{occ.full_path}` |")
    w(f"| Component name | {occ.component_name} |")
    w(f"| Depth | {occ.depth} |")
    w(f"| Path segments | {' → '.join(occ.path_segments)} |")
    
    if occ.parent_path:
        w(f"| Parent path | `{occ.parent_path}` |")
    
    if occ.is_subassembly:
        w(f"| Child occurrences | {occ.child_count} |")
    
    # Transforms
    w(f"| **Transforms** | |")
    gp = occ.global_position
    w(f"| Global position (m) | ({gp[0]:.6f}, {gp[1]:.6f}, {gp[2]:.6f}) |")
    gp_mm = (gp[0]*1000, gp[1]*1000, gp[2]*1000)
    w(f"| Global position (mm) | ({gp_mm[0]:.2f}, {gp_mm[1]:.2f}, {gp_mm[2]:.2f}) |")
    lt = occ.local_transform.translation
    w(f"| Local transform (m) | ({lt[0]:.6f}, {lt[1]:.6f}, {lt[2]:.6f}) |")
    w(f"| Assembly context depth | {occ.assembly_context_depth} |")
    
    if occ.transform2:
        t2 = occ.transform2.translation
        w(f"| transform2 (m) | ({t2[0]:.6f}, {t2[1]:.6f}, {t2[2]:.6f}) |")
    
    if not occ.is_subassembly:
        # Physical
        w(f"| **Physical** | |")
        w(f"| Mass | {occ.mass_kg:.6f} kg ({occ.mass_kg*1000:.3f} g) |")
        w(f"| Volume | {occ.volume_m3:.6e} m³ |")
        w(f"| Density | {occ.density_kg_m3:.1f} kg/m³ |")
        w(f"| Surface area | {occ.area_m2:.6e} m² |")
        w(f"| Body count | {occ.body_count} |")
        
        # CoM
        cl = occ.com_component_local
        cg = occ.com_global
        w(f"| CoM (component-local, m) | ({cl[0]:.6f}, {cl[1]:.6f}, {cl[2]:.6f}) |")
        w(f"| CoM (global, m) | ({cg[0]:.6f}, {cg[1]:.6f}, {cg[2]:.6f}) |")
        cg_mm = (cg[0]*1000, cg[1]*1000, cg[2]*1000)
        w(f"| CoM (global, mm) | ({cg_mm[0]:.2f}, {cg_mm[1]:.2f}, {cg_mm[2]:.2f}) |")
        
        # Inertia
        io = occ.inertia_at_origin
        ic = occ.inertia_at_com
        w(f"| **Inertia at origin (kg·m²)** | |")
        w(f"| Ixx, Iyy, Izz | {io.ixx:.6e}, {io.iyy:.6e}, {io.izz:.6e} |")
        w(f"| Ixy, Ixz, Iyz | {io.ixy:.6e}, {io.ixz:.6e}, {io.iyz:.6e} |")
        w(f"| **Inertia at CoM (kg·m²)** | |")
        w(f"| Ixx, Iyy, Izz | {ic.ixx:.6e}, {ic.iyy:.6e}, {ic.izz:.6e} |")
        w(f"| Ixy, Ixz, Iyz | {ic.ixy:.6e}, {ic.ixz:.6e}, {ic.iyz:.6e} |")
        
        # BBox
        bs = occ.bbox_size
        w(f"| Bounding box (m) | {bs[0]:.4f} × {bs[1]:.4f} × {bs[2]:.4f} |")
        bs_mm = (bs[0]*1000, bs[1]*1000, bs[2]*1000)
        w(f"| Bounding box (mm) | {bs_mm[0]:.2f} × {bs_mm[1]:.2f} × {bs_mm[2]:.2f} |")
        
        # Material & appearance
        w(f"| **Material & Appearance** | |")
        w(f"| Material | {occ.material_name} |")
        w(f"| Appearance | {occ.appearance_name} |")
        if occ.appearance_color_rgb:
            r, g, b = occ.appearance_color_rgb
            w(f"| Color (RGB 0-1) | ({r:.3f}, {g:.3f}, {b:.3f}) |")
            w(f"| Color (RGB 0-255) | ({int(r*255)}, {int(g*255)}, {int(b*255)}) |")
        
        # Per-body breakdown if multiple bodies
        if len(occ.bodies) > 1:
            w(f"| **Per-body breakdown** | |")
            for i, body in enumerate(occ.bodies):
                w(f"| Body {i}: {body.name} | mass={body.mass_kg*1000:.3f}g, material={body.material_name}, inertia_src={body.inertia_source} |")
    
    w(f"")


def _write_joint(joint: FusionJoint, lines: list):
    """Write detailed joint info."""
    w = lines.append
    
    tag = "🔧" if joint.joint_source == "regular" else "⚡"
    w(f"#### {tag} Joint: `{joint.name}` ({joint.joint_source})")
    w(f"")
    w(f"| Property | Value |")
    w(f"|----------|-------|")
    w(f"| Defining component | {joint.defining_component} ({joint.defining_component_raw}) |")
    w(f"| Suppressed | {joint.is_suppressed} |")
    w(f"| Motion type | {joint.motion_type} (enum={joint.motion_type_enum}) |")
    w(f"| Axis | ({joint.axis_vector[0]:.4f}, {joint.axis_vector[1]:.4f}, {joint.axis_vector[2]:.4f}) |")
    w(f"| **Connections** | |")
    w(f"| Parent (occ2) | `{joint.occurrence_two_clean}` |")
    w(f"| Parent path | `{joint.occurrence_two_path}` |")
    w(f"| Child (occ1) | `{joint.occurrence_one_clean}` |")
    w(f"| Child path | `{joint.occurrence_one_path}` |")
    
    # All geometry sources
    w(f"| **Geometry (all sources, raw cm)** | |")
    if joint.geometry_origin_cm:
        g = joint.geometry_origin_cm
        w(f"| geometry.origin | ({g[0]:.4f}, {g[1]:.4f}, {g[2]:.4f}) |")
    if joint.geometry_or_origin_one_cm:
        g = joint.geometry_or_origin_one_cm
        w(f"| geometryOrOriginOne | ({g[0]:.4f}, {g[1]:.4f}, {g[2]:.4f}) |")
    if joint.geometry_or_origin_two_cm:
        g = joint.geometry_or_origin_two_cm
        w(f"| geometryOrOriginTwo | ({g[0]:.4f}, {g[1]:.4f}, {g[2]:.4f}) |")
    if joint.occ_one_transform_cm:
        g = joint.occ_one_transform_cm
        w(f"| occ1.transform | ({g[0]:.4f}, {g[1]:.4f}, {g[2]:.4f}) ctx_depth={joint.occ_one_context_depth} |")
    if joint.occ_one_global_cm:
        g = joint.occ_one_global_cm
        w(f"| occ1.global (assembled) | ({g[0]:.4f}, {g[1]:.4f}, {g[2]:.4f}) |")
    if joint.occ_two_transform_cm:
        g = joint.occ_two_transform_cm
        w(f"| occ2.transform | ({g[0]:.4f}, {g[1]:.4f}, {g[2]:.4f}) ctx_depth={joint.occ_two_context_depth} |")
    if joint.occ_two_global_cm:
        g = joint.occ_two_global_cm
        w(f"| occ2.global (assembled) | ({g[0]:.4f}, {g[1]:.4f}, {g[2]:.4f}) |")
    
    # Picked origin
    o = joint.origin_global_m
    w(f"| **Picked origin (m)** | ({o[0]:.6f}, {o[1]:.6f}, {o[2]:.6f}) via `{joint.origin_source}` |")
    
    # Limits
    if joint.has_rotation_limits:
        w(f"| Rotation limits (rad) | [{joint.rotation_min:.4f}, {joint.rotation_max:.4f}] |")
    if joint.has_slide_limits:
        w(f"| Slide limits (m) | [{joint.slide_min_m:.4f}, {joint.slide_max_m:.4f}] |")
    
    w(f"")
