"""
Fusion Extractor — Extract everything from Fusion 360 into Python dataclasses.

This is the ONLY module that imports adsk.* — all downstream processing
works with pure Python FusionSnapshot data.

Single pass through root.allOccurrences (guaranteed global context).
Single pass through design.allComponents for joints.
Extracts ALL available properties.

Author: Adrian Valaker Eikeland
"""

import math
import struct
from datetime import datetime
from typing import Tuple

try:
    import adsk.core
    import adsk.fusion
except ImportError:
    # The CI test image does not include Fusion 360's Python API.  Keep this
    # module importable so adsk-free helpers and mock-based tests can run.
    adsk = None  # type: ignore[assignment]

from .data_types import (
    FusionSnapshot, FusionOccurrence, FusionJoint, FusionBodyInfo,
    InertiaTensor, Transform3D, Vec3, RigidGroupInfo,
    DESIGN_ROOT_OCCURRENCE_PATH,
)
from ..utils import (
    clean_name, safe_identifier, is_collision_component_name,
    is_collision_body_name, is_collision_excluded_body_name,
    is_frame_only_name, strip_frame_prefix, strip_link_metadata_prefixes,
    clean_link_name, parse_occurrence_path,
    cm_to_m, cm3_to_m3, cm2_to_m2, kg_cm2_to_kg_m2,
)
from ..utils import Logger


def _require_fusion_api():
    if adsk is None:
        raise RuntimeError(
            "Fusion 360 API package 'adsk' is not available. "
            "This operation must run inside Fusion 360."
        )


def extract(design, log: Logger) -> FusionSnapshot:
    """
    Extract complete design data from Fusion 360.
    
    Args:
        design: adsk.fusion.Design — the active Fusion design
        log: Logger instance
        
    Returns:
        FusionSnapshot with all occurrences and joints
    """
    _require_fusion_api()
    root = design.rootComponent
    
    snapshot = FusionSnapshot(
        design_name=root.name,
        design_name_clean=clean_link_name(root.name),
        root_component_name=root.name,
        export_timestamp=datetime.now().isoformat(),
    )
    
    # Capture document length unit (affects STL export scale)
    try:
        snapshot.document_length_unit = design.unitsManager.defaultLengthUnits
        log(f"  Document unit: {snapshot.document_length_unit}")
    except Exception:
        snapshot.document_length_unit = "cm"  # safe default
    
    log.section("EXTRACTION: OCCURRENCES")
    _extract_all_occurrences(root, snapshot, log)
    
    log.section("EXTRACTION: JOINTS")
    _extract_all_joints(design, snapshot, log)
    
    log.section("EXTRACTION: RIGID GROUPS")
    _extract_rigid_groups(design, snapshot, log)
    
    # Summary stats
    snapshot.total_occurrences = len(snapshot.occurrences)
    snapshot.total_subassemblies = sum(1 for o in snapshot.occurrences.values() if o.is_subassembly)
    snapshot.total_leaf_components = sum(1 for o in snapshot.occurrences.values() if not o.is_subassembly)
    snapshot.total_joints = len(snapshot.joints)
    snapshot.total_regular_joints = sum(1 for j in snapshot.joints.values() if j.joint_source == "regular")
    snapshot.total_as_built_joints = sum(1 for j in snapshot.joints.values() if j.joint_source == "as_built")
    snapshot.max_nesting_depth = max((o.depth for o in snapshot.occurrences.values()), default=0)
    
    log.section("EXTRACTION SUMMARY")
    log(f"  Occurrences: {snapshot.total_occurrences} "
        f"({snapshot.total_subassemblies} subassemblies, "
        f"{snapshot.total_leaf_components} leaf components)")
    log(f"  Joints: {snapshot.total_joints} "
        f"({snapshot.total_as_built_joints} as-built, "
        f"{snapshot.total_regular_joints} regular)")
    log(f"  Max nesting depth: {snapshot.max_nesting_depth}")
    
    return snapshot


# ──────────────────────────────────────────────
# Occurrence extraction
# ──────────────────────────────────────────────

def _extract_all_occurrences(root, snapshot: FusionSnapshot, log: Logger):
    """Walk root.allOccurrences — guaranteed full global context."""
    
    count = 0
    for occ in root.allOccurrences:
        if not occ or not occ.component:
            continue
        
        fo = _extract_one_occurrence(occ, log)
        snapshot.occurrences[fo.full_path] = fo
        count += 1
    
    log(f"  Extracted {count} occurrences")


def _extract_one_occurrence(occ, log: Logger) -> FusionOccurrence:
    """Extract all properties from a single occurrence."""
    
    comp = occ.component
    full_path = getattr(occ, 'fullPathName', '') or ''
    comp_name = comp.name if comp else ''
    is_collision_geometry = is_collision_component_name(comp_name)
    name_source, collision_override, is_frame_only = strip_link_metadata_prefixes(comp_name)
    raw_clean_name = clean_name(name_source) or clean_name(comp_name)
    cname = raw_clean_name
    segments = parse_occurrence_path(full_path)
    
    fo = FusionOccurrence(
        full_path=full_path,
        component_name=comp_name,
        clean_name=cname,
        path_segments=segments,
        depth=len(segments) - 1 if segments else 0,
        is_frame_only=is_frame_only,
        is_collision_geometry=is_collision_geometry,
        collision_override=collision_override,
        _fusion_occurrence=occ,  # Keep live reference for mesh export
    )
    
    # Parent path: everything before the last '+'
    if '+' in full_path:
        fo.parent_path = full_path.rsplit('+', 1)[0]
    
    # Subassembly detection
    fo.is_subassembly = _is_subassembly(occ)
    fo.child_count = _count_children(occ)
    
    # Transforms
    _extract_transforms(occ, fo)
    
    has_direct_bodies = _has_direct_bodies(occ)

    # Physical properties for direct bodies.  A subassembly may also own
    # bodies; child occurrences are still handled separately.  Frame-only
    # components intentionally ignore placeholder bodies and emit only a
    # named URDF frame.
    if is_frame_only and has_direct_bodies:
        log.warning(
            f"  Frame-only component '{comp_name}' has bodies; ignoring "
            f"visual/collision/inertial geometry for link '{cname}'"
        )
    elif has_direct_bodies:
        _extract_physical_properties(occ, fo, log)
    
    # Material & appearance
    _extract_appearance(occ, fo)
    
    # Log
    tag = "FRAME" if fo.is_frame_only else (
        "SUBASM+BODY" if fo.is_subassembly and fo.body_count > 0 else (
        "SUBASM" if fo.is_subassembly else "LEAF"
        )
    )
    log(f"  [{tag}] d={fo.depth} {cname}")
    log(f"    path: {full_path}")
    if fo.collision_override:
        log(f"    collision override: {fo.collision_override}")
    if fo.collision_excluded_body_names:
        log(
            f"    collision-excluded bodies: "
            f"{', '.join(fo.collision_excluded_body_names)}"
        )
    log(f"    global_pos: ({fo.global_position[0]:.6f}, {fo.global_position[1]:.6f}, {fo.global_position[2]:.6f}) m")
    
    if fo.body_count > 0:
        log(f"    mass: {fo.mass_kg:.6f} kg, bodies: {fo.body_count}")
        log(f"    com_global: ({fo.com_global[0]:.6f}, {fo.com_global[1]:.6f}, {fo.com_global[2]:.6f}) m")
        log(f"    com_component_local: ({fo.com_component_local[0]:.6f}, {fo.com_component_local[1]:.6f}, {fo.com_component_local[2]:.6f}) m")
        log(f"    inertia@origin: ixx={fo.inertia_at_origin.ixx:.6e} iyy={fo.inertia_at_origin.iyy:.6e} izz={fo.inertia_at_origin.izz:.6e} kg·m²")
        log(f"    inertia@com:    ixx={fo.inertia_at_com.ixx:.6e} iyy={fo.inertia_at_com.iyy:.6e} izz={fo.inertia_at_com.izz:.6e} kg·m²")
        log(f"    material: {fo.material_name}")
        if fo.appearance_color_rgb:
            r, g, b = fo.appearance_color_rgb
            log(f"    color: RGB({r:.2f}, {g:.2f}, {b:.2f}) [{fo.appearance_name}]")
        log(f"    bbox: ({fo.bbox_size[0]:.4f} x {fo.bbox_size[1]:.4f} x {fo.bbox_size[2]:.4f}) m")
    
    return fo


def _strip_frame_link_prefix(cleaned_name: str) -> str:
    """Remove !frame_* metadata while keeping a safe fallback name."""
    stripped = strip_frame_prefix(cleaned_name)
    return stripped or cleaned_name


def _is_subassembly(occ) -> bool:
    """Check if occurrence has child occurrences (is an assembly, not a leaf)."""
    try:
        if hasattr(occ, 'childOccurrences') and occ.childOccurrences:
            if occ.childOccurrences.count > 0:
                return True
    except Exception:
        pass
    
    try:
        comp = occ.component
        if comp and hasattr(comp, 'occurrences') and comp.occurrences:
            if comp.occurrences.count > 0:
                return True
    except Exception:
        pass
    
    return False


def _has_direct_bodies(occ) -> bool:
    """True when this occurrence's component owns bodies directly."""
    try:
        comp = occ.component
        bodies = getattr(comp, 'bRepBodies', None) if comp else None
        return bool(bodies and bodies.count > 0)
    except Exception:
        return False


def _count_children(occ) -> int:
    """Count direct child occurrences."""
    try:
        if hasattr(occ, 'childOccurrences') and occ.childOccurrences:
            return occ.childOccurrences.count
    except Exception:
        pass
    return 0


# ──────────────────────────────────────────────
# Transform extraction
# ──────────────────────────────────────────────

def _extract_transforms(occ, fo: FusionOccurrence):
    """Extract local, global, and transform2 from occurrence."""
    
    # Local transform (relative to parent assembly)
    if hasattr(occ, 'transform') and occ.transform:
        t = occ.transform.translation
        fo.local_transform = Transform3D(
            translation=(cm_to_m(t.x), cm_to_m(t.y), cm_to_m(t.z)),
            rotation=_extract_rotation(occ.transform)
        )
    
    # transform2 (Fusion's "design position")
    if hasattr(occ, 'transform2') and occ.transform2:
        t2 = occ.transform2.translation
        fo.transform2 = Transform3D(
            translation=(cm_to_m(t2.x), cm_to_m(t2.y), cm_to_m(t2.z)),
            rotation=_extract_rotation(occ.transform2)
        )
    
    # Global position from Fusion's ``transform2`` — the occurrence's
    # pose in the design's WORLD frame.  Earlier this code summed
    # ``transform.translation`` up the assemblyContext chain, which
    # silently dropped intermediate sub-asm rotations: when a gripper
    # sub-asm was mounted into Assem1 with a non-identity mount
    # rotation, every nested leaf's "global position" came out as a
    # plain vector sum instead of the rotated chain.  All bake
    # offsets and joint origins inside the gripper then computed
    # against the wrong frame, which scattered the visual meshes
    # across space.
    #
    # ``transform2`` is the right primitive: Fusion composes the chain
    # for us with proper rotation handling.  Walk-the-chain remains
    # only as a defensive fallback for occurrences that somehow lack
    # transform2 (very rare; imported or unusual designs).
    if hasattr(occ, 'transform2') and occ.transform2:
        t2 = occ.transform2.translation
        fo.global_position = (cm_to_m(t2.x), cm_to_m(t2.y), cm_to_m(t2.z))
        depth = _count_chain_depth(occ)
    else:
        total_x, total_y, total_z = 0.0, 0.0, 0.0
        current = occ
        depth = 0
        while current and depth < 20:
            if hasattr(current, 'transform') and current.transform:
                t = current.transform.translation
                total_x += t.x
                total_y += t.y
                total_z += t.z
            if hasattr(current, 'assemblyContext') and current.assemblyContext:
                current = current.assemblyContext
                depth += 1
            else:
                break
        fo.global_position = (cm_to_m(total_x), cm_to_m(total_y), cm_to_m(total_z))

    fo.assembly_context_depth = depth
    fo.global_transform = Transform3D(translation=fo.global_position)


def _count_chain_depth(occ) -> int:
    """Count assemblyContext chain depth without doing the buggy sum.
    Kept for diagnostic logging — debug pages report nesting depth."""
    current = occ
    depth = 0
    while current and depth < 20:
        if hasattr(current, 'assemblyContext') and current.assemblyContext:
            current = current.assemblyContext
            depth += 1
        else:
            break
    return depth


def _extract_rotation(transform) -> tuple:
    """Extract 3x3 rotation matrix from Fusion Matrix3D as flat 9-tuple."""
    try:
        # Fusion Matrix3D stores as 16-element array (row-major 4x4)
        cells = []
        for i in range(16):
            cells.append(transform.getCell(i // 4, i % 4))
        # Extract 3x3 rotation submatrix
        r = (cells[0], cells[1], cells[2],
             cells[4], cells[5], cells[6],
             cells[8], cells[9], cells[10])
        return r
    except Exception:
        return (1, 0, 0, 0, 1, 0, 0, 0, 1)


# ──────────────────────────────────────────────
# Physical properties extraction
# ──────────────────────────────────────────────

def _extract_physical_properties(occ, fo: FusionOccurrence, log: Logger):
    """Extract mass, CoM, inertia, volume, area from all bodies in component."""
    
    comp = occ.component
    if not comp or not hasattr(comp, 'bRepBodies') or not comp.bRepBodies:
        return
    
    total_mass = 0.0
    weighted_com = [0.0, 0.0, 0.0]
    total_volume = 0.0
    total_area = 0.0
    
    # Accumulated inertia at origin (will sum across bodies)
    ti_xx, ti_yy, ti_zz = 0.0, 0.0, 0.0
    ti_xy, ti_xz, ti_yz = 0.0, 0.0, 0.0
    
    bbox_min = [1e10, 1e10, 1e10]
    bbox_max = [-1e10, -1e10, -1e10]
    collision_bbox_min = [1e10, 1e10, 1e10]
    collision_bbox_max = [-1e10, -1e10, -1e10]
    collision_body_count = 0
    collision_volume = 0.0
    collision_excluded_body_names = []
    
    for i in range(comp.bRepBodies.count):
        body = comp.bRepBodies.item(i)
        if not body:
            continue
        
        bi = FusionBodyInfo(name=getattr(body, 'name', f'body_{i}'))
        bi.exclude_from_collision = is_collision_excluded_body_name(bi.name)
        if bi.exclude_from_collision:
            collision_excluded_body_names.append(bi.name)
        
        # Physical properties
        props = None
        if hasattr(body, 'physicalProperties'):
            props = body.physicalProperties
        
        if props:
            # Mass — Fusion returns kg
            try:
                if hasattr(props, 'mass') and props.mass > 0:
                    bi.mass_kg = props.mass
            except Exception:
                pass
            
            # Volume — Fusion returns cm³
            try:
                if hasattr(props, 'volume'):
                    bi.volume_m3 = cm3_to_m3(props.volume)
            except Exception:
                pass
            
            # Density — Fusion returns g/cm³ or kg/m³ depending on version
            try:
                if hasattr(props, 'density'):
                    bi.density_kg_m3 = props.density * 1000.0  # Assume g/cm³ → kg/m³
            except Exception:
                pass
            
            # Area — Fusion returns cm²
            try:
                if hasattr(props, 'area'):
                    bi.area_m2 = cm2_to_m2(props.area)
            except Exception:
                pass
            
            # Center of mass — Fusion returns Point3D in cm, component-local
            try:
                if hasattr(props, 'centerOfMass') and props.centerOfMass:
                    com = props.centerOfMass
                    bi.com_component_local = (cm_to_m(com.x), cm_to_m(com.y), cm_to_m(com.z))
            except Exception:
                pass
            
            # Inertia at origin — Fusion API getXYZMomentsOfInertia()
            # Returns (success, ixx, iyy, izz, ixy, iyz, ixz) in kg·cm²
            try:
                if hasattr(props, 'getXYZMomentsOfInertia'):
                    result = props.getXYZMomentsOfInertia()
                    if result and result[0]:  # success flag
                        _, ixx, iyy, izz, ixy, iyz, ixz = result
                        bi.inertia_at_origin = InertiaTensor(
                            ixx=kg_cm2_to_kg_m2(ixx),
                            ixy=kg_cm2_to_kg_m2(ixy),
                            ixz=kg_cm2_to_kg_m2(ixz),
                            iyy=kg_cm2_to_kg_m2(iyy),
                            iyz=kg_cm2_to_kg_m2(iyz),
                            izz=kg_cm2_to_kg_m2(izz)
                        )
                        bi.inertia_source = "api"
            except Exception:
                pass
            
            # Fallback: estimate from bounding box
            if not bi.inertia_at_origin and bi.mass_kg > 0:
                try:
                    bb = body.boundingBox
                    if bb:
                        dx = cm_to_m(bb.maxPoint.x - bb.minPoint.x)
                        dy = cm_to_m(bb.maxPoint.y - bb.minPoint.y)
                        dz = cm_to_m(bb.maxPoint.z - bb.minPoint.z)
                        m = bi.mass_kg
                        bi.inertia_at_origin = InertiaTensor(
                            ixx=(1/12) * m * (dy**2 + dz**2),
                            iyy=(1/12) * m * (dx**2 + dz**2),
                            izz=(1/12) * m * (dx**2 + dy**2)
                        )
                        bi.inertia_source = "bbox_estimate"
                except Exception:
                    pass
        
        # Bounding box
        try:
            bb = body.boundingBox
            if bb:
                bi.bbox_min = (cm_to_m(bb.minPoint.x), cm_to_m(bb.minPoint.y), cm_to_m(bb.minPoint.z))
                bi.bbox_max = (cm_to_m(bb.maxPoint.x), cm_to_m(bb.maxPoint.y), cm_to_m(bb.maxPoint.z))
                for d in range(3):
                    bbox_min[d] = min(bbox_min[d], bi.bbox_min[d])
                    bbox_max[d] = max(bbox_max[d], bi.bbox_max[d])
                if (not bi.exclude_from_collision
                        and not is_collision_body_name(bi.name)):
                    collision_body_count += 1
                    for d in range(3):
                        collision_bbox_min[d] = min(
                            collision_bbox_min[d], bi.bbox_min[d]
                        )
                        collision_bbox_max[d] = max(
                            collision_bbox_max[d], bi.bbox_max[d]
                        )
        except Exception:
            pass
        
        # Material name — sanitize at extraction so it never reaches
        # URDF/USD as a non-ASCII identifier.  Downloaded Fusion assets
        # frequently carry localized material names (Chinese, Cyrillic,
        # Japanese), and Isaac Sim's URDF importer crashes on those
        # ("LLVM ERROR: out of memory" after a partial rewrite).
        try:
            if hasattr(body, 'material') and body.material:
                bi.material_name = safe_identifier(
                    body.material.name, fallback="material"
                )
        except Exception:
            pass
        
        fo.bodies.append(bi)
        
        # Accumulate totals
        if bi.mass_kg > 0:
            total_mass += bi.mass_kg
            cx, cy, cz = bi.com_component_local
            weighted_com[0] += bi.mass_kg * cx
            weighted_com[1] += bi.mass_kg * cy
            weighted_com[2] += bi.mass_kg * cz
            
            if bi.inertia_at_origin:
                ti_xx += bi.inertia_at_origin.ixx
                ti_yy += bi.inertia_at_origin.iyy
                ti_zz += bi.inertia_at_origin.izz
                ti_xy += bi.inertia_at_origin.ixy
                ti_xz += bi.inertia_at_origin.ixz
                ti_yz += bi.inertia_at_origin.iyz
        
        total_volume += bi.volume_m3
        total_area += bi.area_m2
        if (not bi.exclude_from_collision
                and not is_collision_body_name(bi.name)):
            collision_volume += bi.volume_m3
        
        # Material name (take from first body that has one)
        if bi.material_name and not fo.material_name:
            fo.material_name = bi.material_name
    
    # Store totals
    fo.body_count = comp.bRepBodies.count
    fo.mass_kg = total_mass
    fo.volume_m3 = total_volume
    fo.area_m2 = total_area
    fo.collision_excluded_body_names = collision_excluded_body_names
    fo.collision_body_count = collision_body_count
    fo.collision_volume_m3 = collision_volume
    
    # Component-local CoM (weighted average)
    if total_mass > 0:
        fo.com_component_local = (
            weighted_com[0] / total_mass,
            weighted_com[1] / total_mass,
            weighted_com[2] / total_mass
        )
    
    # Global CoM = component-local CoM + occurrence global position
    # NOTE: This is a simplification — proper transform would use the full
    # rotation matrix, not just translation. For assemblies where the
    # component isn't rotated relative to global, this is exact.
    # TODO: Use full rotation transform for rotated components.
    gx, gy, gz = fo.global_position
    cx, cy, cz = fo.com_component_local
    fo.com_global = (gx + cx, gy + cy, gz + cz)
    
    # Inertia at origin (summed over all bodies)
    fo.inertia_at_origin = InertiaTensor(
        ixx=ti_xx, ixy=ti_xy, ixz=ti_xz,
        iyy=ti_yy, iyz=ti_yz, izz=ti_zz
    )
    
    # Inertia at CoM via parallel axis theorem (Steiner):
    # I_origin = I_com + m * [(d·d)E - d⊗d]
    # I_com = I_origin - m * [(d·d)E - d⊗d]
    # where d = vector from CoM to origin (in component-local coords) = -com_component_local
    if total_mass > 0:
        cx, cy, cz = fo.com_component_local
        dd = cx*cx + cy*cy + cz*cz
        fo.inertia_at_com = InertiaTensor(
            ixx=ti_xx - total_mass * (dd - cx*cx),  # -m*(cy²+cz²)
            ixy=ti_xy + total_mass * cx * cy,
            ixz=ti_xz + total_mass * cx * cz,
            iyy=ti_yy - total_mass * (dd - cy*cy),  # -m*(cx²+cz²)
            iyz=ti_yz + total_mass * cy * cz,
            izz=ti_zz - total_mass * (dd - cz*cz)   # -m*(cx²+cy²)
        )
    
    # Bounding box
    if bbox_min[0] < 1e9:
        fo.bbox_min = tuple(bbox_min)
        fo.bbox_max = tuple(bbox_max)
        fo.bbox_size = (
            bbox_max[0] - bbox_min[0],
            bbox_max[1] - bbox_min[1],
            bbox_max[2] - bbox_min[2]
        )
    if collision_bbox_min[0] < 1e9:
        fo.collision_bbox_min = tuple(collision_bbox_min)
        fo.collision_bbox_max = tuple(collision_bbox_max)
        fo.collision_bbox_size = (
            collision_bbox_max[0] - collision_bbox_min[0],
            collision_bbox_max[1] - collision_bbox_min[1],
            collision_bbox_max[2] - collision_bbox_min[2]
        )
    
    # Density (from total mass/volume or first body)
    if total_volume > 0:
        fo.density_kg_m3 = total_mass / total_volume


# ──────────────────────────────────────────────
# Appearance extraction
# ──────────────────────────────────────────────

def _extract_appearance(occ, fo: FusionOccurrence):
    """Extract material appearance and color from occurrence."""
    try:
        appearance = None
        source = ""
        
        # Priority: occurrence → component → first body
        if hasattr(occ, 'appearance') and occ.appearance:
            appearance = occ.appearance
            source = "occurrence"
        elif occ.component:
            if hasattr(occ.component, 'appearance') and occ.component.appearance:
                appearance = occ.component.appearance
                source = "component"
            elif occ.component.bRepBodies and occ.component.bRepBodies.count > 0:
                body = occ.component.bRepBodies.item(0)
                if hasattr(body, 'appearance') and body.appearance:
                    appearance = body.appearance
                    source = "body"
        
        if not appearance:
            return
        
        # Sanitize appearance name same as body material name above —
        # it ends up as ``LinkNode.material_name`` and emits to URDF
        # ``<material name="...">`` which must be a valid XML/USD ID.
        raw_app_name = getattr(appearance, 'name', '') or ''
        fo.appearance_name = safe_identifier(raw_app_name, fallback="appearance")
        fo.appearance_color_rgb = _extract_color(appearance)
        
    except Exception:
        pass


def _extract_color(appearance):
    """Extract RGB color from a Fusion appearance. Returns (r,g,b) 0-1 or None."""
    if not appearance:
        return None
    
    try:
        # Method 1: ColorProperty
        for prop in appearance.appearanceProperties:
            if prop.objectType == adsk.core.ColorProperty.classType():
                cp = adsk.core.ColorProperty.cast(prop)
                if cp and cp.value:
                    c = cp.value
                    return (c.red / 255.0, c.green / 255.0, c.blue / 255.0)
        
        # Method 2: Properties with color/albedo in name (PBR materials)
        for prop in appearance.appearanceProperties:
            pid = (prop.id or '').lower()
            pname = (prop.name or '').lower()
            
            if any(t in pid or t in pname for t in ('color', 'albedo', 'tint', 'diffuse')):
                if prop.objectType == adsk.core.ColorProperty.classType():
                    cp = adsk.core.ColorProperty.cast(prop)
                    if cp and cp.value:
                        c = cp.value
                        return (c.red / 255.0, c.green / 255.0, c.blue / 255.0)
                
                if prop.objectType == adsk.core.FloatProperty.classType():
                    fp = adsk.core.FloatProperty.cast(prop)
                    if fp and 0.0 <= fp.value <= 1.0:
                        v = fp.value
                        return (v, v, v)
        
        # Method 3: Direct color attribute
        if hasattr(appearance, 'color') and appearance.color:
            c = appearance.color
            return (c.red / 255.0, c.green / 255.0, c.blue / 255.0)
    
    except Exception:
        pass
    
    return None


# ──────────────────────────────────────────────
# Joint extraction
# ──────────────────────────────────────────────

def _extract_all_joints(design, snapshot: FusionSnapshot, log: Logger):
    """Collect all joints from all components — both regular and as-built."""
    
    seen = set()  # Dedup by (name, component, source)
    skipped = 0
    
    for comp in design.allComponents:
        comp_clean = clean_link_name(comp.name)
        
        # As-built joints
        if hasattr(comp, 'asBuiltJoints') and comp.asBuiltJoints:
            for i in range(comp.asBuiltJoints.count):
                joint = comp.asBuiltJoints.item(i)
                if not joint:
                    continue
                raw_name = _safe_fusion_attr(
                    joint, 'name', f'as_built_{i}', log,
                    f"as_built joint #{i} in component '{comp.name}'"
                )
                key = (raw_name, comp_clean, 'as_built')
                if key in seen:
                    continue
                seen.add(key)
                
                try:
                    fj = _extract_one_joint(
                        joint, "as_built", comp, log,
                        snapshot=snapshot,
                        root_component=design.rootComponent,
                    )
                except Exception as exc:
                    log.warning(
                        f"  Skipping as_built joint '{raw_name}' in "
                        f"component '{comp.name}': {exc}"
                    )
                    skipped += 1
                    continue
                if fj:
                    snapshot.joints[fj.name] = fj
                else:
                    skipped += 1
        
        # Regular joints
        if hasattr(comp, 'joints') and comp.joints:
            for i in range(comp.joints.count):
                joint = comp.joints.item(i)
                if not joint:
                    continue
                raw_name = _safe_fusion_attr(
                    joint, 'name', f'joint_{i}', log,
                    f"regular joint #{i} in component '{comp.name}'"
                )
                key = (raw_name, comp_clean, 'regular')
                if key in seen:
                    continue
                seen.add(key)
                
                try:
                    fj = _extract_one_joint(
                        joint, "regular", comp, log,
                        snapshot=snapshot,
                        root_component=design.rootComponent,
                    )
                except Exception as exc:
                    log.warning(
                        f"  Skipping regular joint '{raw_name}' in "
                        f"component '{comp.name}': {exc}"
                    )
                    skipped += 1
                    continue
                if fj:
                    snapshot.joints[fj.name] = fj
                else:
                    skipped += 1
    
    log(f"  Extracted {len(snapshot.joints)} unique joints")
    if skipped:
        log.warning(f"  Skipped {skipped} joint(s) that Fusion reported as invalid or unreadable")


def _ensure_design_root_occurrence(
    snapshot: FusionSnapshot,
    root_component,
    log: Logger,
) -> FusionOccurrence:
    """Add a synthetic occurrence representing bodies in the design root.

    Fusion has no Occurrence object for the root component.  Imported files
    often author joints between root-owned bodies and top-level occurrences;
    reading the root-side ``occurrenceTwo`` then raises
    ``InternalValidationError``.  This synthetic occurrence gives those joints
    a concrete parent link in the pure-Python model.
    """
    existing = snapshot.occurrences.get(DESIGN_ROOT_OCCURRENCE_PATH)
    if existing:
        return existing

    raw_name = getattr(root_component, "name", "") or snapshot.design_name_clean
    name_source, collision_override, is_frame_only = strip_link_metadata_prefixes(raw_name)
    clean = snapshot.design_name_clean or clean_name(name_source) or clean_name(raw_name) or "base_link"
    fo = FusionOccurrence(
        full_path=DESIGN_ROOT_OCCURRENCE_PATH,
        component_name=raw_name,
        clean_name=clean,
        path_segments=[clean],
        depth=0,
        is_subassembly=False,
        is_frame_only=is_frame_only,
        collision_override=collision_override,
        global_position=(0.0, 0.0, 0.0),
        local_transform=Transform3D(),
        transform2=Transform3D(),
        assembly_context_depth=0,
        # This is a Component, not an Occurrence.  Mesh export handles this
        # specially by hiding child occurrences and exporting only root bodies.
        _fusion_occurrence=root_component,
    )

    class _RootOccurrenceProxy:
        component = root_component

    if is_frame_only:
        log.warning(
            f"  Frame-only design root '{raw_name}' has root bodies; ignoring "
            f"visual/collision/inertial geometry for link '{clean}'"
        )
    else:
        proxy = _RootOccurrenceProxy()
        _extract_physical_properties(proxy, fo, log)
        _extract_appearance(proxy, fo)
    snapshot.occurrences[DESIGN_ROOT_OCCURRENCE_PATH] = fo
    log.warning(
        f"  Added synthetic design-root link '{clean}' for root-owned joint endpoint(s)"
    )
    return fo


def _extract_one_joint(
    joint,
    source: str,
    defining_comp,
    log: Logger,
    snapshot: FusionSnapshot = None,
    root_component=None,
):
    """Extract all properties from a single joint."""
    from ..utils import strip_joint_prefix

    defining_name = getattr(defining_comp, "name", "unknown")
    context = f"{source} joint in component '{defining_name}'"
    raw_name = _safe_fusion_attr(joint, 'name', 'unknown', log, context)
    suppressed = _safe_fusion_attr(joint, 'isSuppressed', False, log, context)

    # Recognise convention-keyword prefixes on the joint name.  A
    # joint named ``closing_left_slider`` becomes name=``left_slider``,
    # is_closing=True, is_passive=True (closing implies passive).
    # ``passive_idler`` becomes name=``idler``, is_passive=True.
    # The clean (post-strip) name is what flows into URDF and
    # robot_data.yaml; the raw prefixed name stays on ``raw_name``
    # for traceability in the snapshot.
    cleaned_name, is_passive, is_closing = strip_joint_prefix(raw_name)
    name = cleaned_name or raw_name  # fall back to raw if user named the joint just "passive" / "closing"

    # Sanitize for URDF/USD identifier rules — spaces, accents, and
    # other non-alphanumerics in Fusion joint names (often typos like
    # ``left_pivot _joint1`` with a stray space) silently leaked into
    # the URDF and crashed Isaac Sim's importer with
    # ``LLVM ERROR: out of memory`` after a partial path rewrite.
    # ``safe_identifier`` collapses bad chars to underscore, drops
    # leading digits, and falls back to ``"joint"`` if the result is
    # empty.  Component names already go through ``clean_name``;
    # joint names did not, until now.
    name = safe_identifier(name, fallback="joint")

    fj = FusionJoint(
        name=name,
        raw_name=raw_name,
        is_passive=is_passive,
        is_closing=is_closing,
        joint_source=source,
        defining_component=clean_link_name(defining_comp.name),
        defining_component_raw=defining_comp.name,
        is_suppressed=suppressed,
    )
    
    # Connected occurrences.  Downloaded/imported designs can contain joints
    # to bodies owned directly by the design root.  Fusion exposes the moving
    # occurrence but raises RuntimeError for the root-side occurrence because
    # there is no Occurrence object for the root component itself.
    occ_one = _safe_fusion_attr(joint, 'occurrenceOne', None, log, context)
    occ_two = _safe_fusion_attr(joint, 'occurrenceTwo', None, log, context)
    if not occ_one:
        log.warning(
            f"  Skipping {context} '{raw_name}': missing/unreadable endpoint "
            f"(occurrenceOne={bool(occ_one)}, occurrenceTwo={bool(occ_two)})"
        )
        return None
    
    if occ_one:
        fj.occurrence_one_path = _safe_fusion_attr(
            occ_one, 'fullPathName', '', log, context
        ) or ''
        occ_one_comp = _safe_fusion_attr(
            occ_one, 'component', None, log, context
        )
        if occ_one_comp:
            endpoint_clean = clean_link_name(
                _safe_fusion_attr(occ_one_comp, 'name', '', log, context)
            )
            fj.occurrence_one_clean = endpoint_clean
    
    if occ_two:
        fj.occurrence_two_path = _safe_fusion_attr(
            occ_two, 'fullPathName', '', log, context
        ) or ''
        occ_two_comp = _safe_fusion_attr(
            occ_two, 'component', None, log, context
        )
        if occ_two_comp:
            endpoint_clean = clean_link_name(
                _safe_fusion_attr(occ_two_comp, 'name', '', log, context)
            )
            fj.occurrence_two_clean = endpoint_clean
    else:
        if snapshot is not None and root_component is not None:
            _ensure_design_root_occurrence(snapshot, root_component, log)
            fj.occurrence_two_path = DESIGN_ROOT_OCCURRENCE_PATH
            fj.occurrence_two_clean = snapshot.design_name_clean
            log.warning(
                f"  {context} '{raw_name}': occurrenceTwo is unavailable; "
                f"using design root '{snapshot.design_name_clean}' as parent endpoint"
            )
        else:
            log.warning(
                f"  Skipping {context} '{raw_name}': occurrenceTwo is unreadable "
                f"and no design-root fallback is available"
            )
            return None
    
    # ── Geometry origins — extract ALL methods ──
    
    # geometry.origin
    try:
        if hasattr(joint, 'geometry') and joint.geometry:
            if hasattr(joint.geometry, 'origin') and joint.geometry.origin:
                o = joint.geometry.origin
                fj.geometry_origin_cm = (o.x, o.y, o.z)
    except Exception:
        pass
    
    # geometryOrOriginOne
    try:
        if hasattr(joint, 'geometryOrOriginOne') and joint.geometryOrOriginOne:
            geo = joint.geometryOrOriginOne
            if hasattr(geo, 'origin') and geo.origin:
                o = geo.origin
                fj.geometry_or_origin_one_cm = (o.x, o.y, o.z)
    except Exception:
        pass
    
    # geometryOrOriginTwo
    try:
        if hasattr(joint, 'geometryOrOriginTwo') and joint.geometryOrOriginTwo:
            geo = joint.geometryOrOriginTwo
            if hasattr(geo, 'origin') and geo.origin:
                o = geo.origin
                fj.geometry_or_origin_two_cm = (o.x, o.y, o.z)
    except Exception:
        pass
    
    # occurrenceOne transform variants
    if occ_one:
        try:
            if occ_one.transform:
                t = occ_one.transform.translation
                fj.occ_one_transform_cm = (t.x, t.y, t.z)
        except Exception:
            pass
        
        try:
            if hasattr(occ_one, 'transform2') and occ_one.transform2:
                t = occ_one.transform2.translation
                fj.occ_one_transform2_cm = (t.x, t.y, t.z)
        except Exception:
            pass
        
        # Global via assemblyContext chain
        fj.occ_one_global_cm, fj.occ_one_context_depth = _walk_assembly_context(occ_one)
    
    # occurrenceTwo transforms
    if occ_two:
        try:
            if occ_two.transform:
                t = occ_two.transform.translation
                fj.occ_two_transform_cm = (t.x, t.y, t.z)
        except Exception:
            pass
        
        fj.occ_two_global_cm, fj.occ_two_context_depth = _walk_assembly_context(occ_two)
    
    # ── Pick best origin (global, meters) ──
    fj.origin_global_m, fj.origin_source = _pick_joint_origin(fj)
    
    # ── Motion type ──
    _extract_joint_motion(joint, fj)
    
    # ── Log ──
    tag = source.upper()
    log(f"  [{tag} in {fj.defining_component}] {name}")
    log(f"    parent(occ2): {fj.occurrence_two_clean} path={fj.occurrence_two_path}")
    log(f"    child(occ1):  {fj.occurrence_one_clean} path={fj.occurrence_one_path}")
    
    if fj.geometry_origin_cm:
        log(f"    geometry.origin: ({fj.geometry_origin_cm[0]:.4f}, {fj.geometry_origin_cm[1]:.4f}, {fj.geometry_origin_cm[2]:.4f}) cm")
    if fj.geometry_or_origin_one_cm:
        log(f"    geometryOrOriginOne: ({fj.geometry_or_origin_one_cm[0]:.4f}, {fj.geometry_or_origin_one_cm[1]:.4f}, {fj.geometry_or_origin_one_cm[2]:.4f}) cm")
    if fj.geometry_or_origin_two_cm:
        log(f"    geometryOrOriginTwo: ({fj.geometry_or_origin_two_cm[0]:.4f}, {fj.geometry_or_origin_two_cm[1]:.4f}, {fj.geometry_or_origin_two_cm[2]:.4f}) cm")
    if fj.occ_one_transform_cm:
        log(f"    occ1.transform: ({fj.occ_one_transform_cm[0]:.4f}, {fj.occ_one_transform_cm[1]:.4f}, {fj.occ_one_transform_cm[2]:.4f}) cm (ctx_depth={fj.occ_one_context_depth})")
    if fj.occ_one_global_cm:
        log(f"    occ1.global:    ({fj.occ_one_global_cm[0]:.4f}, {fj.occ_one_global_cm[1]:.4f}, {fj.occ_one_global_cm[2]:.4f}) cm")
    if fj.occ_two_transform_cm:
        log(f"    occ2.transform: ({fj.occ_two_transform_cm[0]:.4f}, {fj.occ_two_transform_cm[1]:.4f}, {fj.occ_two_transform_cm[2]:.4f}) cm (ctx_depth={fj.occ_two_context_depth})")
    if fj.occ_two_global_cm:
        log(f"    occ2.global:    ({fj.occ_two_global_cm[0]:.4f}, {fj.occ_two_global_cm[1]:.4f}, {fj.occ_two_global_cm[2]:.4f}) cm")
    
    log(f"    → origin_global: ({fj.origin_global_m[0]:.6f}, {fj.origin_global_m[1]:.6f}, {fj.origin_global_m[2]:.6f}) m [via {fj.origin_source}]")
    log(f"    motion: {fj.motion_type}, axis: ({fj.axis_vector[0]:.3f}, {fj.axis_vector[1]:.3f}, {fj.axis_vector[2]:.3f})")
    
    if fj.has_rotation_limits:
        log(f"    rotation limits: [{fj.rotation_min:.4f}, {fj.rotation_max:.4f}] rad")
    if fj.has_slide_limits:
        log(f"    slide limits: [{fj.slide_min_m:.4f}, {fj.slide_max_m:.4f}] m")
    
    return fj


def _safe_fusion_attr(obj, attr: str, default=None, log: Logger = None, context: str = ""):
    """Read a Fusion API property without letting API validation errors abort."""
    try:
        return getattr(obj, attr)
    except Exception as exc:
        if log:
            where = f"{context}: " if context else ""
            log.warning(f"  {where}failed to read {attr}: {exc}")
        return default


def _walk_assembly_context(occ):
    """Walk assemblyContext chain to compute global position. Returns (xyz_cm, depth)."""
    total_x, total_y, total_z = 0.0, 0.0, 0.0
    current = occ
    depth = 0
    while current and depth < 20:
        if hasattr(current, 'transform') and current.transform:
            t = current.transform.translation
            total_x += t.x
            total_y += t.y
            total_z += t.z
        if hasattr(current, 'assemblyContext') and current.assemblyContext:
            current = current.assemblyContext
            depth += 1
        else:
            break
    return (total_x, total_y, total_z), depth


def _pick_joint_origin(fj: FusionJoint):
    """
    Pick the best available origin and convert to global meters.
    
    Priority:
      1. geometry.origin — most reliable (regular joints, some as-built)
      2. geometryOrOriginOne.origin — fallback for regular joints
      3. occ_one_global via assemblyContext chain — universal fallback
      4. occ_one_transform — last resort
    
    Note: geometry.origin and geometryOrOriginOne.origin are in the
    DEFINING COMPONENT's coordinate frame. For joints defined in the
    root component, this IS global. For joints defined inside a
    sub-assembly, the coordinates are assembly-local.
    
    We store the raw value here and let Phase 2 (robot_model) handle
    the frame conversion once the assembly hierarchy is known.
    """
    if fj.geometry_origin_cm:
        x, y, z = fj.geometry_origin_cm
        return (cm_to_m(x), cm_to_m(y), cm_to_m(z)), "geometry.origin"
    
    if fj.geometry_or_origin_one_cm:
        x, y, z = fj.geometry_or_origin_one_cm
        return (cm_to_m(x), cm_to_m(y), cm_to_m(z)), "geometryOrOriginOne"
    
    if fj.occ_one_global_cm:
        x, y, z = fj.occ_one_global_cm
        return (cm_to_m(x), cm_to_m(y), cm_to_m(z)), "occ_one_global"
    
    if fj.occ_one_transform_cm:
        x, y, z = fj.occ_one_transform_cm
        return (cm_to_m(x), cm_to_m(y), cm_to_m(z)), "occ_one_transform"
    
    return (0.0, 0.0, 0.0), "none"


# ──────────────────────────────────────────────
# Joint motion extraction
# ──────────────────────────────────────────────

def _extract_joint_motion(joint, fj: FusionJoint):
    """Extract motion type, axis, and limits."""
    
    motion = getattr(joint, 'jointMotion', None)
    if not motion:
        fj.motion_type = "rigid"
        return
    
    # Motion type
    jtype = getattr(motion, 'jointType', None)
    fj.motion_type_enum = jtype
    
    type_map = {
        0: "rigid",       # RigidJointType
        1: "revolute",    # RevoluteJointType
        2: "slider",      # SliderJointType
        3: "cylindrical", # CylindricalJointType
        4: "pin_slot",    # PinSlotJointType
        5: "planar",      # PlanarJointType
        6: "ball",        # BallJointType
    }
    fj.motion_type = type_map.get(jtype, f"unknown_{jtype}")
    
    # Axis
    try:
        if hasattr(motion, 'rotationAxisVector') and motion.rotationAxisVector:
            v = motion.rotationAxisVector
            fj.axis_vector = (v.x, v.y, v.z)
        elif hasattr(motion, 'slideDirectionVector') and motion.slideDirectionVector:
            v = motion.slideDirectionVector
            fj.axis_vector = (v.x, v.y, v.z)
    except Exception:
        pass
    
    # Rotation limits (radians)
    try:
        if hasattr(motion, 'rotationLimits') and motion.rotationLimits:
            limits = motion.rotationLimits
            if hasattr(limits, 'isMinimumValueEnabled') and limits.isMinimumValueEnabled:
                fj.has_rotation_limits = True
                fj.rotation_min = limits.minimumValue  # Already radians
            if hasattr(limits, 'isMaximumValueEnabled') and limits.isMaximumValueEnabled:
                fj.has_rotation_limits = True
                fj.rotation_max = limits.maximumValue
    except Exception:
        pass
    
    # Slide limits (cm → meters)
    try:
        if hasattr(motion, 'slideLimits') and motion.slideLimits:
            limits = motion.slideLimits
            if hasattr(limits, 'isMinimumValueEnabled') and limits.isMinimumValueEnabled:
                fj.has_slide_limits = True
                fj.slide_min_m = cm_to_m(limits.minimumValue)
            if hasattr(limits, 'isMaximumValueEnabled') and limits.isMaximumValueEnabled:
                fj.has_slide_limits = True
                fj.slide_max_m = cm_to_m(limits.maximumValue)
    except Exception:
        pass


# ══════════════════════════════════════════════
# MESH EXPORT — OBJ+MTL (visual) + STL (collision)
# ══════════════════════════════════════════════
# All Fusion API mesh operations live here alongside extraction,
# so this remains the ONLY module that imports adsk.*.

import os
import shutil
import tempfile


# ──────────────────────────────────────────────
# Rigid group extraction
# ──────────────────────────────────────────────

def _walk_leaf_occurrences(occ):
    """Yield physical rigid-group members under ``occ``.

    Fusion's ``RigidGroup.occurrences`` returns whatever the user dragged
    into the group — a sub-assembly occurrence stays as a sub-assembly
    here, it is NOT auto-flattened.  We expand it ourselves so a user who
    grouped the ``esp32`` sub-asm gets every cap/resistor inside merged
    into the same URDF link, not a single mystery placeholder.

    If the sub-assembly also owns direct bodies, yield it too.  Imported
    CAD often stores the main fuselage/body geometry on an assembly
    occurrence while panels and internals live as child occurrences.

    Uses ``childOccurrences`` (the Fusion sample pattern from
    AssemblyTraversalUsingRecursion) — that respects assembly context, so
    each yielded occurrence has the correct ``fullPathName``.
    """
    try:
        children = occ.childOccurrences
        has_children = children is not None and children.count > 0
    except Exception:
        has_children = False

    has_direct_bodies = _has_direct_bodies(occ)
    if has_direct_bodies:
        yield occ

    if not has_children:
        if not has_direct_bodies:
            yield occ
        return

    for i in range(children.count):
        child = children.item(i)
        if child is None:
            continue
        yield from _walk_leaf_occurrences(child)


def _extract_rigid_groups(design, snapshot: FusionSnapshot, log: Logger):
    """Extract rigid groups, expanding sub-assembly members.

    Each rigid group becomes ONE URDF link downstream (see
    ``robot_model._process_rigid_groups``).  Container-only assemblies
    expand to their physical leaves.  Assemblies that own direct bodies
    are recorded as members too, because their own geometry belongs in
    the merged link.
    """
    try:
        root = design.rootComponent
        all_groups = root.allRigidGroups
        if not all_groups:
            log(f"  No rigid groups found")
            return
    except Exception:
        log(f"  Rigid group API not available")
        return

    for rg in all_groups:
        try:
            suppressed = rg.isSuppressed
        except Exception:
            suppressed = False  # Fusion API bug — assume not suppressed
        if suppressed:
            continue

        # Recognise the ``acc_`` / ``acc-`` prefix that flags a rigid
        # group as needing accurate (visual-mesh) collision instead of
        # the default bounding-primitive auto-fit.  Strip the prefix
        # from the stored group name so the merged link doesn't end
        # up called ``acc_gripper_jaws`` — the prefix is metadata, not
        # part of the public name.
        from ..utils import parse_collision_override_prefix
        rg_raw_name = rg.name or ""
        collision_override, stripped_name = parse_collision_override_prefix(rg_raw_name)
        rg_clean_name = stripped_name or rg_raw_name

        group = RigidGroupInfo(
            name=rg_clean_name,
            wants_accurate_collision=(collision_override == "visual"),
            collision_override=collision_override,
        )
        direct_count = 0
        expanded_count = 0
        seen_paths = set()

        try:
            for occ in rg.occurrences:
                if occ is None:
                    continue
                direct_count += 1

                for leaf in _walk_leaf_occurrences(occ):
                    full_path = getattr(leaf, 'fullPathName', '') or ''
                    if not full_path or full_path in seen_paths:
                        continue
                    if leaf.component is None:
                        continue
                    seen_paths.add(full_path)

                    raw_member_name = leaf.component.name or ""
                    cname = clean_link_name(raw_member_name)
                    group.occurrence_paths.append(full_path)
                    group.member_clean_names.append(cname)
                    expanded_count += 1

                    if is_collision_component_name(raw_member_name):
                        group.collision_member = cname
                        group.collision_path = full_path
        except Exception as e:
            log.warning(f"  Rigid group '{rg.name}': failed to read occurrences: {e}")
            continue

        snapshot.rigid_groups.append(group)

        coll_tag = f" → collision: {group.collision_member}" if group.collision_member else ""
        expansion_tag = (
            f" (expanded {direct_count}→{expanded_count})"
            if expanded_count != direct_count else ""
        )
        log(f"  {group.name}: {group.member_clean_names}{coll_tag}{expansion_tag}")

    log(f"  Extracted {len(snapshot.rigid_groups)} rigid groups")


def capture_screenshot(
    pkg_dir: str,
    filename: str = "robot.png",
    width: int = 1920,
    height: int = 1080,
    log: Logger = None,
) -> str:
    """
    Capture Fusion 360 viewport as image for the robot README.
    
    Saves to <pkg_dir>/images/<filename>.
    Returns filepath on success, empty string on failure.
    """
    try:
        app = adsk.core.Application.get()
        if not app or not app.activeViewport:
            return ""
        images_dir = os.path.join(pkg_dir, "images")
        os.makedirs(images_dir, exist_ok=True)
        filepath = os.path.join(images_dir, filename)
        app.activeViewport.fit()
        success = app.activeViewport.saveAsImageFile(filepath, width, height)
        if success and log:
            log(f"  → images/{filename}")
        return filepath if success else ""
    except Exception as e:
        if log:
            log.warning(f"  Screenshot capture failed: {e}")
        return ""


def export_meshes(
    model,  # RobotModel (imported at call time to avoid circular)
    snapshot: FusionSnapshot,
    pkg_dir: str,
    config,  # ExportConfig
    log: Logger,
):
    """
    Export visual (OBJ+MTL) and collision (STL) meshes for all links.

    REQUIRES FUSION 360 API — call from within Fusion script.

    For each link:
      1. Find the Fusion occurrence via snapshot._fusion_occurrence
      2. Detect collision sub-components/bodies
      3. Export visual OBJ+MTL (collision geometry suppressed)
      4. Export collision STL (if explicit collision found)
    """
    _require_fusion_api()
    app = adsk.core.Application.get()
    exportMgr = app.activeProduct.exportManager

    refinement_map = {
        "low": adsk.fusion.MeshRefinementSettings.MeshRefinementLow,
        "medium": adsk.fusion.MeshRefinementSettings.MeshRefinementMedium,
        "high": adsk.fusion.MeshRefinementSettings.MeshRefinementHigh,
    }
    refinement = refinement_map.get(
        config.mesh_refinement,
        adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
    )

    log.section("MESH EXPORT")

    # Create output directories
    mesh_base = os.path.join(pkg_dir, "meshes")
    os.makedirs(mesh_base, exist_ok=True)
    for asm_name in model.assemblies:
        os.makedirs(os.path.join(mesh_base, asm_name), exist_ok=True)

    stats = {"visual": 0, "collision_explicit": 0, "collision_body_warn": 0, "skipped": 0}
    
    # Import rescaler for Fusion-exported STL normalization
    from .collision_generator import rescale_stl_to_cm
    doc_unit = snapshot.document_length_unit
    
    # Track rigid group collision files already exported (shared by multiple links)
    exported_rg_collision = set()

    for link_name, link in model.links.items():
        occ_data = snapshot.occurrences.get(link.occurrence_path)
        if not occ_data or not occ_data._fusion_occurrence:
            log.warning(f"  {link_name}: no Fusion occurrence — skipping")
            link.has_visual_mesh = False
            link.mesh_visual = ""
            stats["skipped"] += 1
            continue

        fusion_occ = occ_data._fusion_occurrence
        log(f"  {link_name}:")

        if getattr(link, "is_frame_only", False):
            log("    Frame-only link - no mesh export")
            link.has_visual_mesh = False
            link.mesh_visual = ""
            stats["skipped"] += 1
            continue

        # Skip empty links (reference frames with no bodies)
        if link.is_empty:
            log(f"    Empty link (reference frame) — no mesh export")
            continue

        # ── Visual: OBJ + MTL ──
        obj_path = os.path.join(pkg_dir, link.mesh_visual)
        mtl_path = os.path.splitext(obj_path)[0] + ".mtl"

        # A "merged" link with only ONE visual member is really just
        # a regular per-component link with a !collision_* sibling for
        # collision STL.  Falling through to the merge path would
        # invoke createOBJExportOptions on rootComponent + visibility
        # hiding, which Fusion silently ignores for top-level occurrences
        # — yielding the entire-design OBJ instead of just the member.
        # Treat single-member groups as per-component links here.
        single_member_group = (
            getattr(link, "is_merged", False)
            and len(getattr(link, "merged_member_paths", []) or []) <= 1
        )

        visual_reason = ""
        if getattr(link, "is_merged", False) and not single_member_group:
            # Merged rigid-group anchor: hand off to Fusion's native OBJ
            # API on the LCA sub-asm.  See _export_merged_visual_obj.
            visual_ok, visual_reason = _export_merged_visual_obj(
                link, snapshot, obj_path, mtl_path, exportMgr, refinement, log
            )
            if visual_ok:
                stats["visual"] += 1
            else:
                log.error(f"    Merged visual export failed for {link.urdf_name}: {visual_reason}")
                stats["skipped"] += 1
            collision_found = "merged"
        else:
            # Per-component path (also handles single-member groups so
            # the rigid-group's !collision_* sibling still wires up via
            # link.rigid_group_collision_path below).
            collision_found = _detect_collision(fusion_occ, link, log)
            visual_ok, visual_reason = _export_visual_obj(
                fusion_occ, obj_path, mtl_path, link, exportMgr, refinement, log
            )
            if visual_ok:
                stats["visual"] += 1
            else:
                stats["skipped"] += 1
            if single_member_group:
                log(f"    Single-member rigid group — exported member directly")

        # When the visual export failed, mark the link so the URDF
        # generator skips the ``<visual>`` element entirely instead of
        # emitting a ``<mesh filename>`` pointing at a file that isn't
        # on disk (which is a 404 in any URDF previewer).
        if not visual_ok:
            link.has_visual_mesh = False
            # Surface the actual failure reason in the model warning so
            # the user sees it in the export-summary dialog without
            # needing to dig through a debug log.
            why = f" — {visual_reason}" if visual_reason else ""
            model.warnings.append(
                f"Link '{link.urdf_name}': visual mesh export failed{why}"
            )
            link.mesh_visual = ""

        # ── Optional: convert OBJ+MTL to DAE ──
        # When config.visual_format == 'dae', convert the just-written
        # OBJ+MTL into a single self-contained .dae and update the
        # link's mesh_visual path.  DAE writes vertices in METERS, so
        # the URDF/xacro generator emits ``scale="1 1 1"`` for it
        # (vs ``scale="0.01"`` for OBJ which is in centimeters).
        if (visual_ok
                and getattr(config, "visual_format", "obj") == "dae"
                and os.path.isfile(obj_path)):
            from .obj_to_dae import obj_to_dae
            dae_rel = os.path.splitext(link.mesh_visual)[0] + ".dae"
            dae_path = os.path.join(pkg_dir, dae_rel)
            try:
                if obj_to_dae(obj_path, mtl_path if os.path.isfile(mtl_path) else None,
                              dae_path, name=link.urdf_name):
                    # Keep the OBJ+MTL on disk for now — collision_generator
                    # samples its vertices in fit_primitive's circularity
                    # check, and deleting before that runs makes
                    # cylinder-vs-box decisions worse.  Stash the paths so
                    # the package generator's final cleanup step can drop
                    # them once collision is fully resolved.
                    link.mesh_visual = dae_rel
                    link._dae_pending_obj_cleanup = (obj_path, mtl_path)
                    log(f"    DAE written → {dae_rel} (OBJ retained for collision fit)")
                else:
                    log.warning(f"    DAE conversion produced no geometry for {link.urdf_name}; keeping OBJ")
            except Exception as e:
                log.error(f"    DAE conversion failed for {link.urdf_name}: {e}")

        # ── Collision: STL (only if explicit) ──
        # Each helper returns True iff the STL actually landed on disk.
        # On failure we leave has_explicit_collision=False so the
        # collision_generator falls back to a primitive rather than
        # producing a URDF that references a missing file.
        if visual_ok and getattr(link, "has_collision_exclusions", False):
            input_rel = os.path.splitext(link.mesh_collision)[0] + "_input.obj"
            input_rel = input_rel.replace("\\", "/")
            input_path = os.path.join(pkg_dir, input_rel)
            input_mtl_path = os.path.splitext(input_path)[0] + ".mtl"
            input_ok, input_reason = _export_collision_input_obj(
                link, fusion_occ, snapshot, input_path, input_mtl_path,
                exportMgr, refinement, log,
            )
            if input_ok:
                link.mesh_collision_input = input_rel
                link._collision_input_cleanup = (input_path, input_mtl_path)
                log(
                    f"    Collision input OBJ excludes bodies: "
                    f"{', '.join(link.collision_excluded_body_names)}"
                )
            else:
                why = f" - {input_reason}" if input_reason else ""
                msg = (
                    f"Link '{link.urdf_name}': could not export filtered "
                    f"collision input for ! body exclusions{why}"
                )
                model.warnings.append(msg)
                log.warning(f"    {msg}")

        coll_path = os.path.join(pkg_dir, link.mesh_collision)
        if collision_found == "subcomponent":
            if _export_collision_subcomponent(fusion_occ, coll_path, exportMgr, refinement, log):
                rescale_stl_to_cm(coll_path, doc_unit)
                link.has_explicit_collision = True
                stats["collision_explicit"] += 1
        elif collision_found == "body":
            if _export_collision_bodies(fusion_occ, coll_path, link, exportMgr, refinement, log):
                rescale_stl_to_cm(coll_path, doc_unit)
                link.has_explicit_collision = True
                stats["collision_body_warn"] += 1
        elif hasattr(link, '_collision_sibling_path') and link._collision_sibling_path:
            # Flattened collision sub-assembly: sibling component provides collision.
            # has_explicit_collision was set True in _build_links; clear it on any
            # failure so collision_generator falls back to a primitive.
            sibling_data = snapshot.occurrences.get(link._collision_sibling_path)
            ok = False
            if sibling_data and sibling_data._fusion_occurrence:
                ok = _export_collision_sibling(
                    sibling_data._fusion_occurrence, coll_path,
                    exportMgr, refinement, log
                )
                if ok:
                    rescale_stl_to_cm(coll_path, doc_unit)
                    stats["collision_explicit"] += 1
                    log(f"    Collision from flattened sibling: {sibling_data.clean_name}")
            else:
                log.warning(f"    Flattened collision sibling not found: {link._collision_sibling_path}")
            link.has_explicit_collision = ok
        elif link.rigid_group_collision_path:
            # Rigid group collision: all members share ONE collision mesh.
            # has_explicit_collision was set True in _build_links; clear it on any
            # failure so collision_generator falls back to a primitive.
            if link.mesh_collision in exported_rg_collision:
                # Earlier sibling already exported this STL successfully.
                log(f"    Collision shared → {link.mesh_collision}")
                link.has_explicit_collision = True
            else:
                rg_coll_data = snapshot.occurrences.get(link.rigid_group_collision_path)
                ok = False
                if rg_coll_data and rg_coll_data._fusion_occurrence:
                    ok = _export_collision_sibling(
                        rg_coll_data._fusion_occurrence, coll_path,
                        exportMgr, refinement, log
                    )
                    if ok:
                        rescale_stl_to_cm(coll_path, doc_unit)
                        exported_rg_collision.add(link.mesh_collision)
                        stats["collision_explicit"] += 1
                        log(f"    Collision STL exported (shared): {rg_coll_data.clean_name}")
                else:
                    log.warning(f"    Rigid group collision member not found: {link.rigid_group_collision_path}")
                link.has_explicit_collision = ok

    log(f"\n  Mesh export summary:")
    log(f"    Visual (OBJ+MTL):              {stats['visual']}")
    log(f"    Collision sub-component (STL):  {stats['collision_explicit']}")
    log(f"    Collision body + warning (STL): {stats['collision_body_warn']}")
    log(f"    Skipped (no Fusion ref):        {stats['skipped']}")


# ── Collision detection ──

def _is_component_ref(fusion_ref) -> bool:
    """True when ``fusion_ref`` is a Component rather than an Occurrence."""
    return not hasattr(fusion_ref, "component")


def _fusion_ref_component(fusion_ref):
    """Return the Component represented by an Occurrence or Component ref."""
    return getattr(fusion_ref, "component", None) or fusion_ref


def _fusion_export_target(fusion_ref):
    """Object passed to Fusion's export manager for this link."""
    return fusion_ref


def _detect_collision(fusion_occ, link, log: Logger):
    """Detect collision geometry. Returns 'subcomponent', 'body', or None."""
    try:
        comp = _fusion_ref_component(fusion_occ)
    except Exception:
        return None

    # Prefer sub-component (doesn't affect dynamics)
    for child_occ in comp.occurrences:
        child_name = child_occ.component.name
        if is_collision_component_name(child_name):
            log(f"    Found collision sub-component: '{child_occ.component.name}'")
            return "subcomponent"

    # Fallback: collision bodies (contaminates mass/inertia)
    collision_bodies = []
    for body in comp.bRepBodies:
        if is_collision_body_name(body.name):
            collision_bodies.append(body.name)

    if collision_bodies:
        link.collision_body_names = collision_bodies
        log.warning(
            f"    Collision as body ({collision_bodies}) — affects mass/inertia! "
            f"Sub-component recommended. See DESIGN_RULES.md §1.4 "
            f"Reserved Prefixes and §2.4 Collision Patterns."
        )
        return "body"

    return None


# ── Visual export (OBJ + MTL) ──

def _export_visual_obj(
    fusion_occ, obj_path, mtl_path, link, exportMgr, refinement, log,
    suppress_collision_excluded=False,
):
    """Export visual mesh as OBJ+MTL, suppressing collision geometry.

    Returns ``(success, reason)`` — ``success`` is True iff a non-empty
    OBJ ended up on disk; ``reason`` is a short failure description
    (or empty string on success) that the caller surfaces in the
    export-summary warning so the user can debug without enabling
    a debug log.

    A component built entirely from ``!collision_*`` geometry (common
    for simple parts where one cylinder serves as both visual and
    collision) would otherwise lose ALL its bodies to suppression and
    export an empty OBJ.  Detect that case up front and skip the
    suppression — the visual ends up identical to the collision
    geometry, which is still correct for rendering.
    """
    suppressed_bodies = []
    suppressed_occs = []
    success = False
    reason = ""
    try:
        os.makedirs(os.path.dirname(obj_path), exist_ok=True)
        comp = _fusion_ref_component(fusion_occ)
        export_target = _fusion_export_target(fusion_occ)
        root_component_target = _is_component_ref(fusion_occ)

        # First pass: classify every visible body / sub-occurrence as
        # collision-only or non-collision.  If there's no non-collision
        # geometry, suppression would empty the component entirely —
        # use the collision geometry as the visual instead.
        visible_collision_bodies = []
        visible_visual_bodies = []
        for body in comp.bRepBodies:
            if not body.isVisible:
                continue
            if (is_collision_body_name(body.name)
                    or (suppress_collision_excluded
                        and is_collision_excluded_body_name(body.name))):
                visible_collision_bodies.append(body)
            else:
                visible_visual_bodies.append(body)

        visible_collision_occs = []
        visible_visual_occs = []
        for child_occ in comp.occurrences:
            if not child_occ.isVisible:
                continue
            cname = child_occ.component.name
            if is_collision_component_name(cname):
                visible_collision_occs.append(child_occ)
            else:
                visible_visual_occs.append(child_occ)

        has_non_collision_geometry = bool(
            visible_visual_bodies or visible_visual_occs
        )

        if root_component_target:
            # The synthetic design-root link is made from bodies owned
            # directly by the root component, not from its child
            # occurrences.  Fusion's component-level OBJ export is fuzzy
            # here: depending on the document it may ignore top-level
            # occurrence visibility or write no root-body mesh at all.
            # Exporting the root bodies individually is exact and avoids
            # duplicating the legs into base_link.dae.
            root_ok, root_reason = _export_root_bodies_visual_obj(
                comp, obj_path, mtl_path, link, exportMgr, refinement, log,
                include_collision=(
                    not bool(visible_visual_bodies)
                    and not suppress_collision_excluded
                ),
                exclude_collision_excluded=suppress_collision_excluded,
            )
            if root_ok:
                return (True, "")
            log.warning(
                f"    Root-body visual export failed for {link.urdf_name}: "
                f"{root_reason}; trying component export fallback"
            )

        if has_non_collision_geometry:
            # Normal path: suppress !collision_* so the visual OBJ
            # contains only the renderable geometry.
            for body in visible_collision_bodies:
                body.isVisible = False
                suppressed_bodies.append(body)
            for child_occ in visible_collision_occs:
                child_occ.isVisible = False
                suppressed_occs.append(child_occ)
            if root_component_target:
                # A synthetic design-root link represents only bodies owned
                # directly by the root component.  Hide all child occurrences
                # so exporting the root component does not duplicate the legs.
                for child_occ in visible_visual_occs:
                    child_occ.isVisible = False
                    suppressed_occs.append(child_occ)
        else:
            if suppress_collision_excluded:
                reason = "all visible bodies were excluded from generated collision"
                log.warning(f"    Collision input skipped for {link.urdf_name}: {reason}")
                return (False, reason)
            # Component is all-collision (e.g. a simple cylinder used
            # as both visual and collision).  Export everything; the
            # visual will be identical to the collision, which is
            # better than no visual at all.
            log(
                f"    No non-collision geometry on {link.urdf_name}; "
                f"using !collision_* bodies as visual (visual = collision)"
            )

        # Export OBJ
        obj_opts = exportMgr.createOBJExportOptions(export_target, obj_path)
        if not obj_opts:
            reason = "createOBJExportOptions returned None"
            log.error(f"    OBJ export failed for {link.urdf_name}: {reason}")
            return (False, reason)
        obj_opts.meshRefinement = refinement
        executed = exportMgr.execute(obj_opts)
        on_disk = os.path.exists(obj_path)
        file_size = os.path.getsize(obj_path) if on_disk else 0

        if not executed:
            reason = "exportMgr.execute returned False"
            log.error(f"    OBJ export failed for {link.urdf_name}: {reason}")
            return (False, reason)
        if not on_disk:
            reason = f"file not at {obj_path} after execute()"
            log.error(f"    OBJ export failed for {link.urdf_name}: {reason}")
            return (False, reason)
        if file_size == 0:
            # Fusion silently writes a 0-byte OBJ when there's nothing
            # visible to export — common when ALL of a component's
            # bodies are tagged !collision_*, or when the visual body
            # was hidden in the design and not surfaced through an
            # ancestor's visibility.  Tell the user concretely so they
            # can fix the design rather than chasing a phantom bug.
            reason = (
                "Fusion wrote a 0-byte OBJ — no exportable geometry.  "
                "Likely cause: the component's visual body is hidden in "
                "Fusion, or the component lives inside a parent "
                "occurrence that's hidden.  Toggle visibility in the "
                "Fusion browser tree, save, re-export."
            )
            log.error(f"    OBJ export wrote 0 bytes for {link.urdf_name}: {reason}")
            try:
                os.remove(obj_path)
            except OSError:
                pass
            return (False, reason)
        log(f"    OBJ exported ({file_size} bytes)")

        # Handle MTL
        link_name = os.path.splitext(os.path.basename(obj_path))[0]
        expected_mtl = f"{link_name}.mtl"
        fusion_mtl = _find_fusion_mtl(obj_path)

        if fusion_mtl:
            if fusion_mtl != mtl_path:
                shutil.move(fusion_mtl, mtl_path)
            _patch_obj_mtllib(obj_path, expected_mtl)
            log(f"    MTL preserved from Fusion (multi-material)")
        else:
            color = link.color_rgb if link.color_rgb else (0.7, 0.7, 0.7)
            mat_name = f"{link.urdf_name}_material"
            _write_mtl_file(mtl_path, mat_name, color)
            _patch_obj_mtl_reference(obj_path, expected_mtl, mat_name)
            log(f"    MTL fallback: RGB({color[0]:.2f}, {color[1]:.2f}, {color[2]:.2f})")

        success = True

    except Exception as e:
        reason = f"exception: {e}"
        log.error(f"    Visual export failed for {link.urdf_name}: {e}")
        success = False
    finally:
        # Restore visibility unconditionally — leaking a body/occ in
        # the hidden state poisons subsequent links' exports.
        for body in suppressed_bodies:
            try:
                body.isVisible = True
            except Exception:
                pass
        for occ in suppressed_occs:
            try:
                occ.isVisible = True
            except Exception:
                pass
    return (success, reason)


def _export_collision_input_obj(
    link, fusion_occ, snapshot: FusionSnapshot, obj_path, mtl_path,
    exportMgr, refinement, log,
):
    """Export an OBJ used only for generated collision fitting.

    Body-level ``!`` exclusions stay visible in the normal visual mesh, but
    they are hidden for this filtered OBJ so primitive and convex-hull
    collision ignore antennas, cables, handles, and similar visual detail.
    """
    if getattr(link, "is_merged", False) and len(
        getattr(link, "merged_member_paths", []) or []
    ) > 1:
        return _export_merged_via_per_member(
            link, snapshot, obj_path, mtl_path, exportMgr, refinement, log,
            suppress_collision_excluded=True,
        )
    return _export_visual_obj(
        fusion_occ, obj_path, mtl_path, link, exportMgr, refinement, log,
        suppress_collision_excluded=True,
    )


def _iter_fusion_collection(collection):
    """Yield items from a Fusion collection or a test double."""
    if collection is None:
        return
    try:
        count = getattr(collection, "count")
        item = getattr(collection, "item")
        for i in range(count):
            value = item(i)
            if value is not None:
                yield value
        return
    except Exception:
        pass
    try:
        for value in collection:
            if value is not None:
                yield value
    except Exception:
        return


def _export_root_bodies_visual_obj(
    root_comp, obj_path, mtl_path, link, exportMgr, refinement, log,
    include_collision=False,
    exclude_collision_excluded=False,
):
    """Export only bodies owned directly by the design root.

    Fusion has no Occurrence for the design root, and root Component OBJ
    export is not a reliable isolator when the same component also owns
    top-level child occurrences.  For imported CAD that keeps the chassis
    as loose root bodies, export each root body as its own OBJ and stitch
    the result into the link-local root frame.
    """
    bodies = []
    for body in _iter_fusion_collection(getattr(root_comp, "bRepBodies", None)):
        try:
            if not getattr(body, "isVisible", True):
                continue
        except Exception:
            pass
        bname = getattr(body, "name", "") or ""
        if not include_collision and is_collision_body_name(bname):
            continue
        if exclude_collision_excluded and is_collision_excluded_body_name(bname):
            continue
        bodies.append(body)

    if not bodies:
        return (False, "no visible root-owned bodies")

    temp_parent = os.path.dirname(obj_path) or None
    temp_paths = []
    member_data = []
    failures = []
    try:
        for i, body in enumerate(bodies):
            raw_name = getattr(body, "name", "") or f"body_{i}"
            body_name = safe_identifier(clean_name(raw_name), fallback=f"body_{i}")
            member_name = f"{i:02d}_{body_name}"
            fd, temp_obj = tempfile.mkstemp(
                prefix=f"root_body_{member_name}_",
                suffix=".obj",
                dir=temp_parent,
            )
            os.close(fd)
            temp_paths.append(temp_obj)

            try:
                opts = exportMgr.createOBJExportOptions(body, temp_obj)
            except Exception as exc:
                failures.append(f"{raw_name}: createOBJExportOptions raised {exc}")
                continue
            if not opts:
                failures.append(f"{raw_name}: createOBJExportOptions returned None")
                continue

            try:
                opts.meshRefinement = refinement
            except Exception:
                pass

            try:
                executed = exportMgr.execute(opts)
            except Exception as exc:
                failures.append(f"{raw_name}: execute raised {exc}")
                continue
            if not executed or not os.path.exists(temp_obj):
                failures.append(f"{raw_name}: OBJ export failed")
                continue
            if os.path.getsize(temp_obj) == 0:
                failures.append(f"{raw_name}: OBJ export was empty")
                continue

            obj_data = _read_obj_data(temp_obj)
            if not obj_data["vertices"] or not obj_data["faces"]:
                failures.append(f"{raw_name}: OBJ had no faces")
                continue

            mtl_path_for_body = _find_fusion_mtl(temp_obj)
            if mtl_path_for_body:
                temp_paths.append(mtl_path_for_body)

            member_data.append({
                "name": member_name,
                "index": i,
                "obj_data": obj_data,
                "mtl_path": mtl_path_for_body,
                "R": (1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0),
                "t_cm": (0.0, 0.0, 0.0),
            })

        if not member_data:
            detail = "; ".join(failures[:3]) if failures else "no body OBJ data"
            return (False, detail)

        _write_concatenated_obj_mtl(
            obj_path, mtl_path, member_data, link, log,
            source_label="Root-body visual",
        )
        if failures:
            log.warning(
                f"    Root-body visual exported with {len(failures)} skipped "
                f"body/bodies"
            )
        return (True, "")
    finally:
        for path in temp_paths:
            try:
                if os.path.exists(path):
                    os.remove(path)
            except Exception:
                pass


# ── Collision export (STL) ──

# The bbox helper and threshold live in collision_generator (which is
# adsk-free) so scripts/validate_examples.py can import them in CI
# without pulling in the Fusion API.
from .collision_generator import _stl_bbox_size_cm, _DEGENERATE_BBOX_THRESHOLD_CM


def _try_stl_export(target, output_path, exportMgr, refinement, log, label: str) -> bool:
    """Export ``target`` (Component or Occurrence) to STL, then verify the
    file actually exists AND has non-degenerate bbox.  Returns True on
    real success; False on any failure (including degenerate geometry).
    """
    stl_opts = exportMgr.createSTLExportOptions(target, output_path)
    if not stl_opts:
        return False
    stl_opts.meshRefinement = refinement
    if not exportMgr.execute(stl_opts):
        return False
    if not os.path.exists(output_path):
        log.error(f"    Collision STL write failed — file not at {output_path}")
        return False
    bbox = _stl_bbox_size_cm(output_path)
    if max(bbox) < _DEGENERATE_BBOX_THRESHOLD_CM:
        log.warning(
            f"    Collision STL from {label} is degenerate "
            f"(bbox ≈ {bbox[0]*10:.3f} × {bbox[1]*10:.3f} × {bbox[2]*10:.3f} mm)"
        )
        return False
    size_kb = os.path.getsize(output_path) / 1024
    log(
        f"    Collision STL exported ({label}, {size_kb:.1f} KB, "
        f"bbox {bbox[0]*10:.1f} × {bbox[1]*10:.1f} × {bbox[2]*10:.1f} mm)"
    )
    return True


def _export_collision_subcomponent(fusion_occ, output_path, exportMgr, refinement, log) -> bool:
    """Export collision sub-component as STL.  True iff real geometry was written.

    Two-step strategy: try the child occurrence first; if Fusion writes
    degenerate geometry, retry with the child's component reference.
    """
    output_path = os.path.normpath(output_path)
    try:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        comp = _fusion_ref_component(fusion_occ)
        for child_occ in comp.occurrences:
            cname = child_occ.component.name
            if is_collision_component_name(cname):
                log(f"    Collision source: sub-component={child_occ.component.name}")
                if _try_stl_export(child_occ, output_path, exportMgr, refinement, log, "subcomp-occ"):
                    return True
                child_comp = getattr(child_occ, 'component', None)
                if child_comp is not None:
                    log("    Retrying via component reference...")
                    if _try_stl_export(child_comp, output_path, exportMgr, refinement, log, "subcomp-comp"):
                        return True
                log.error(f"    Collision STL failed for {output_path} — falling back to primitive")
                return False
    except Exception as e:
        log.error(f"    Collision sub-component export failed: {e}")
    return False


def _export_collision_sibling(fusion_occ, output_path, exportMgr, refinement, log) -> bool:
    """Export a collision sibling component as STL.  True iff real geometry was written.

    Two-step strategy: try the occurrence first; if Fusion writes
    degenerate geometry (all vertices near origin), retry with the
    component reference directly.
    """
    output_path = os.path.normpath(output_path)
    try:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        log(f"    Collision source: occurrence={getattr(fusion_occ, 'fullPathName', '?')}")
        if _try_stl_export(fusion_occ, output_path, exportMgr, refinement, log, "sibling-occ"):
            return True
        comp = getattr(fusion_occ, 'component', None)
        if comp is not None:
            log("    Retrying via component reference...")
            if _try_stl_export(comp, output_path, exportMgr, refinement, log, "sibling-comp"):
                return True
        log.error(f"    Collision STL failed for {output_path} — falling back to primitive")
    except Exception as e:
        log.error(f"    Collision sibling export failed: {e}")
    return False


def _export_collision_bodies(fusion_occ, output_path, link, exportMgr, refinement, log) -> bool:
    """Export ``!collision_*`` bodies as STL.  True iff real geometry was written.

    Hides non-collision bodies, exports, then restores visibility
    (in ``finally`` so a raised exception doesn't leave bodies hidden
    in the user's Fusion document).
    """
    output_path = os.path.normpath(output_path)
    hidden = []
    hidden_occs = []
    try:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        comp = _fusion_ref_component(fusion_occ)

        for body in comp.bRepBodies:
            if not is_collision_body_name(body.name):
                if body.isVisible:
                    body.isVisible = False
                    hidden.append(body)

        if _is_component_ref(fusion_occ):
            for child_occ in comp.occurrences:
                if child_occ.isVisible:
                    child_occ.isVisible = False
                    hidden_occs.append(child_occ)

        log(f"    Collision source: bodies={[b.name for b in comp.bRepBodies if is_collision_body_name(b.name)]}")
        if _try_stl_export(fusion_occ, output_path, exportMgr, refinement, log, "body-occ"):
            return True
        if comp is not None:
            log("    Retrying via component reference...")
            if _try_stl_export(comp, output_path, exportMgr, refinement, log, "body-comp"):
                return True
        log.error(f"    Collision STL failed for {output_path} — falling back to primitive")
        return False
    except Exception as e:
        log.error(f"    Collision body export failed for {link.urdf_name}: {e}")
        return False
    finally:
        for body in hidden:
            try:
                body.isVisible = True
            except Exception:
                pass
        for occ in hidden_occs:
            try:
                occ.isVisible = True
            except Exception:
                pass


# ── MTL file handling ──

def _find_fusion_mtl(obj_path):
    """Find the MTL file Fusion created alongside an OBJ export."""
    if not obj_path:
        return ""
    try:
        # Strategy 1: same base name
        base = os.path.splitext(obj_path)[0]
        candidate = base + '.mtl'
        if os.path.exists(candidate):
            return candidate

        # Strategy 2: parse mtllib from OBJ
        if os.path.exists(obj_path):
            with open(obj_path, 'r', errors='replace') as f:
                for line in f:
                    if line.startswith('mtllib '):
                        mtl_name = line.strip().split(' ', 1)[1].strip()
                        mtl_candidate = os.path.join(os.path.dirname(obj_path), mtl_name)
                        if os.path.exists(mtl_candidate):
                            return mtl_candidate
                        break

        # Strategy 3: scan directory for matching .mtl
        obj_dir = os.path.dirname(obj_path)
        obj_stem = os.path.splitext(os.path.basename(obj_path))[0]
        if os.path.isdir(obj_dir):
            for fname in os.listdir(obj_dir):
                if fname.endswith('.mtl') and obj_stem in fname:
                    return os.path.join(obj_dir, fname)
    except Exception:
        pass
    return ""


def _patch_obj_mtllib(obj_path, mtl_filename):
    """Patch mtllib reference in OBJ, preserving per-face usemtl lines."""
    try:
        with open(obj_path, 'r', errors='replace') as f:
            lines = f.readlines()
        new_lines = []
        has_mtllib = False
        for line in lines:
            if line.startswith('mtllib'):
                if not has_mtllib:
                    new_lines.append(f"mtllib {mtl_filename}\n")
                    has_mtllib = True
                continue
            new_lines.append(line)
        if not has_mtllib:
            new_lines.insert(0, f"mtllib {mtl_filename}\n")
        with open(obj_path, 'w', encoding='utf-8') as f:
            f.writelines(new_lines)
    except Exception as e:
        print(f"  OBJ mtllib patch error: {e}")


def _patch_obj_mtl_reference(obj_path, mtl_filename, material_name):
    """Patch OBJ to use single material (fallback when Fusion didn't create MTL)."""
    try:
        with open(obj_path, 'r', errors='replace') as f:
            lines = f.readlines()
        new_lines = [f"mtllib {mtl_filename}\n"]
        added_usemtl = False
        for line in lines:
            if line.startswith('mtllib'):
                continue
            if line.startswith('usemtl'):
                if not added_usemtl:
                    new_lines.append(f"usemtl {material_name}\n")
                    added_usemtl = True
                continue
            new_lines.append(line)
        if not added_usemtl:
            insert_idx = next(
                (i for i, l in enumerate(new_lines) if l.startswith('f ')),
                len(new_lines)
            )
            new_lines.insert(insert_idx, f"usemtl {material_name}\n")
        with open(obj_path, 'w', encoding='utf-8') as f:
            f.writelines(new_lines)
    except Exception as e:
        print(f"  OBJ mtl reference patch error: {e}")


def _write_mtl_file(mtl_path, material_name, color):
    """Write single-material MTL file."""
    try:
        with open(mtl_path, 'w', encoding='utf-8') as f:
            f.write(f"# Generated by Fusion URDF/XACRO Exporter\n")
            f.write(f"newmtl {material_name}\n")
            f.write(f"Ka {color[0]:.4f} {color[1]:.4f} {color[2]:.4f}\n")
            f.write(f"Kd {color[0]:.4f} {color[1]:.4f} {color[2]:.4f}\n")
            f.write(f"Ks 0.2000 0.2000 0.2000\n")
            f.write(f"Ns 50.0\n")
            f.write(f"d 1.0\n")
            f.write(f"illum 2\n")
        return True
    except Exception:
        return False


# ──────────────────────────────────────────────
# Rigid-group merge: ONE OBJ+MTL via Fusion's native sub-asm export
#
# Earlier this module manually exported each rigid-group member, parsed
# their OBJs, transformed vertices through (R_anchor · R_memberᵀ) +
# offset, and concatenated.  That approach fought the Fusion API every
# step of the way and put deep-nested members in the wrong orientation.
#
# The much simpler path: pass the LOWEST COMMON ANCESTOR sub-asm of the
# rigid group's members directly to ``createOBJExportOptions``.  Fusion
# already composes its assemblyContext chain correctly when given a
# sub-asm — that's how ``File → Export → OBJ`` produces a clean assembly
# OBJ.  We just hide any leaf inside the LCA that is NOT in the rigid
# group (and any ``!collision_*`` member), invoke one OBJ export, and
# the result is ready.
#
# If the LCA is the design rootComponent, we pass that.  If members
# span multiple sub-asms with no enclosing sub-asm, we still fall back
# to rootComponent — that handles cross-assembly groups too.
# ──────────────────────────────────────────────


def _common_ancestor_path(member_paths):
    """Return the longest occurrence-path prefix shared by every member.

    Fusion occurrence paths are joined by ``+`` (e.g.
    ``pendel_with_esp v5:1+esp32 v2:1+ESP32-WROOM-32E:1``).  The LCA
    drops the deepest segment the members no longer agree on.  Returns
    ``""`` when the only common prefix is the design root (i.e. no
    shared sub-asm — caller should use ``rootComponent``).
    """
    if not member_paths:
        return ""
    split = [p.split('+') for p in member_paths]
    common = []
    for segs in zip(*split):
        first = segs[0]
        if all(s == first for s in segs):
            common.append(first)
        else:
            break
    # Drop the last segment if it's the leaf component itself (i.e. only
    # one member, where the entire path is a "common prefix" but not a
    # sub-asm).  We want the enclosing assembly, not the leaf.
    if len(common) == len(split[0]):
        common = common[:-1]
    return '+'.join(common)


def _has_shared_component_with_non_member(member_paths, snapshot) -> bool:
    """True if any rigid-group member's component is also used by an
    occurrence that's NOT in the group.

    When this returns True, the visibility-based merge approach
    (hide non-members, export the LCA) leaks the sibling occurrence's
    geometry into the merged OBJ.  Fusion's OBJ exporter renders
    every occurrence of a shared component at its own world transform
    regardless of which visibility flag we toggle on it — verified
    on the gripper-on-Assem1 design where a left_gripper merge ended
    up containing the right side's MGN carriage and screws floating
    off in space.  Caller falls back to per-member export-and-concat.
    """
    member_set = set(member_paths)
    member_components = set()
    for mp in member_paths:
        m = snapshot.occurrences.get(mp)
        if m and getattr(m, "_fusion_occurrence", None) is not None:
            try:
                comp = m._fusion_occurrence.component
                if comp is not None:
                    member_components.add(comp.entityToken)
            except Exception:
                pass

    for path, occ in snapshot.occurrences.items():
        if path in member_set or occ.is_subassembly:
            continue
        f_occ = getattr(occ, "_fusion_occurrence", None)
        if f_occ is None:
            continue
        try:
            comp = f_occ.component
            if comp is not None and comp.entityToken in member_components:
                return True
        except Exception:
            pass
    return False


def _read_obj_data(path):
    """Parse an OBJ file into vertex/normal/texcoord/face arrays.

    Faces preserve per-vertex (v, vt, vn) indices and the active
    material at the time the face was declared.  Used by the
    per-member merge below to concatenate multiple OBJs correctly.
    """
    out = {"vertices": [], "normals": [], "texcoords": [],
           "faces": [], "mtllib": ""}
    current_mat = None
    try:
        with open(path, 'r', errors='replace') as f:
            for line in f:
                tokens = line.split()
                if not tokens:
                    continue
                tag = tokens[0]
                if tag == 'v' and len(tokens) >= 4:
                    out["vertices"].append(
                        (float(tokens[1]), float(tokens[2]), float(tokens[3])))
                elif tag == 'vn' and len(tokens) >= 4:
                    out["normals"].append(
                        (float(tokens[1]), float(tokens[2]), float(tokens[3])))
                elif tag == 'vt' and len(tokens) >= 3:
                    out["texcoords"].append(
                        (float(tokens[1]), float(tokens[2])))
                elif tag == 'f':
                    v_idx, vt_idx, vn_idx = [], [], []
                    for vert in tokens[1:]:
                        parts = vert.split('/')
                        v_idx.append(int(parts[0]) if parts[0] else 0)
                        vt_idx.append(int(parts[1]) if len(parts) > 1 and parts[1] else 0)
                        vn_idx.append(int(parts[2]) if len(parts) > 2 and parts[2] else 0)
                    out["faces"].append((v_idx, vt_idx, vn_idx, current_mat))
                elif tag == 'usemtl' and len(tokens) > 1:
                    current_mat = tokens[1]
                elif tag == 'mtllib' and len(tokens) > 1:
                    out["mtllib"] = tokens[1]
    except Exception as e:
        return out
    return out


def _read_mtl_data(path):
    """Parse an MTL file into ``{material_name: [property_lines]}``."""
    out = {}
    if not path or not os.path.isfile(path):
        return out
    current = None
    try:
        with open(path, 'r', errors='replace') as f:
            for line in f:
                tokens = line.split(None, 1)
                if not tokens:
                    continue
                if tokens[0] == 'newmtl' and len(tokens) > 1:
                    current = tokens[1].strip()
                    if current:
                        out[current] = []
                elif current is not None and tokens[0] not in ('#',):
                    out[current].append(line.rstrip('\n'))
    except Exception:
        pass
    return out


def _export_merged_via_per_member(
    link, snapshot: FusionSnapshot, obj_path: str, mtl_path: str,
    exportMgr, refinement, log: Logger,
    suppress_collision_excluded=False,
):
    """Export each rigid-group member individually and concatenate
    into a single merged OBJ + MTL with vertices transformed into the
    anchor's local frame.

    Used when the rigid group contains components shared with
    non-member occurrences — the visibility-based LCA approach can't
    suppress sibling geometry in that case (Fusion's OBJ exporter
    renders every occurrence of a shared component at its own world
    transform regardless of visibility flags).  Per-member exports
    sidestep the visibility dance entirely: each call exports exactly
    one occurrence to a temp OBJ, we transform its vertices into the
    anchor's frame, and stitch them all together.

    The result is bit-identical to what the LCA approach would have
    produced if visibility had worked correctly — same vertices, same
    materials, same mesh refinement.
    """
    from .robot_model import (
        _mat3_mul, _mat3_transpose, _rotate_vec3_by_mat3,
        _vec_sub,
    )
    import tempfile

    member_paths = list(link.merged_member_paths)
    anchor_path = link.occurrence_path
    anchor_occ = snapshot.occurrences.get(anchor_path)

    if not (anchor_occ and anchor_occ.transform2 and anchor_occ.transform2.rotation):
        return (False, "anchor occurrence missing transform2 rotation")

    R_anchor = anchor_occ.transform2.rotation
    R_anchor_inv = _mat3_transpose(R_anchor)
    t_anchor = anchor_occ.transform2.translation

    temp_dir = tempfile.mkdtemp(prefix="fusion_merge_")
    member_data = []
    try:
        for i, mp in enumerate(member_paths):
            m = snapshot.occurrences.get(mp)
            if m is None or m._fusion_occurrence is None:
                continue
            if getattr(m, "is_frame_only", False):
                continue
            if getattr(m, "is_collision_geometry", False):
                continue
            if not (m.transform2 and m.transform2.rotation):
                log.warning(f"    skipping member {m.clean_name}: no transform2")
                continue

            f_occ = m._fusion_occurrence

            # Suppress !collision_* bodies + sub-occurrences inside the
            # member's own component before exporting it (same hide
            # pattern used by the per-component visual export).  When a
            # rigid-group member is a body-owning subassembly, also hide
            # its direct children for this member export; those children
            # are exported as their own members so including them here
            # would duplicate mesh geometry.
            suppressed_bodies = []
            suppressed_occs = []
            def _suppress_occ_attr(target_occ, attr):
                try:
                    prev = getattr(target_occ, attr)
                    if prev:
                        setattr(target_occ, attr, False)
                        suppressed_occs.append((target_occ, attr, prev))
                except Exception:
                    pass

            try:
                for body in f_occ.component.bRepBodies:
                    if (is_collision_body_name(body.name or "")
                            or (suppress_collision_excluded
                                and is_collision_excluded_body_name(body.name or ""))):
                        if body.isVisible:
                            body.isVisible = False
                            suppressed_bodies.append(body)
            except Exception:
                pass
            try:
                for child_occ in f_occ.component.occurrences:
                    cname = child_occ.component.name or ""
                    if is_collision_component_name(cname):
                        _suppress_occ_attr(child_occ, 'isVisible')
                        _suppress_occ_attr(child_occ, 'isLightBulbOn')
            except Exception:
                pass
            if m.is_subassembly and m.body_count > 0:
                try:
                    children = f_occ.childOccurrences
                    for ci in range(children.count):
                        child_occ = children.item(ci)
                        if child_occ is None:
                            continue
                        _suppress_occ_attr(child_occ, 'isVisible')
                        _suppress_occ_attr(child_occ, 'isLightBulbOn')
                except Exception:
                    pass
                try:
                    for child_occ in f_occ.component.occurrences:
                        if child_occ is None:
                            continue
                        _suppress_occ_attr(child_occ, 'isVisible')
                        _suppress_occ_attr(child_occ, 'isLightBulbOn')
                except Exception:
                    pass

            try:
                safe_name = "".join(
                    c if c.isalnum() or c in '_-' else '_'
                    for c in (m.clean_name or f"m{i}")
                )
                temp_obj = os.path.join(temp_dir, f"member_{i:02d}_{safe_name}.obj")
                opts = exportMgr.createOBJExportOptions(f_occ, temp_obj)
                if not opts:
                    log.warning(f"    member {m.clean_name}: createOBJExportOptions failed")
                    continue
                opts.meshRefinement = refinement
                if not exportMgr.execute(opts) or not os.path.exists(temp_obj):
                    log.warning(f"    member {m.clean_name}: OBJ export failed")
                    continue
                if os.path.getsize(temp_obj) == 0:
                    log.warning(f"    member {m.clean_name}: empty OBJ — skipping")
                    continue
            finally:
                for body in suppressed_bodies:
                    try:
                        body.isVisible = True
                    except Exception:
                        pass
                for occ, attr, prev in suppressed_occs:
                    try:
                        setattr(occ, attr, prev)
                    except Exception:
                        pass

            # Member-to-anchor transform.  ``transform2`` is local-to-world;
            # to express member-local in anchor-local coords:
            #   v_anchor_local = R_anchorᵀ · (R_member · v_member + t_member - t_anchor)
            #                  = R_anchorᵀ · R_member · v_member
            #                    + R_anchorᵀ · (t_member - t_anchor)
            R_member = m.transform2.rotation
            t_member = m.transform2.translation
            R_m_to_a = _mat3_mul(R_anchor_inv, R_member)
            t_m_to_a_m = _rotate_vec3_by_mat3(
                _vec_sub(t_member, t_anchor), R_anchor_inv,
            )
            # OBJ vertices are in centimetres; transform2 translations
            # are in metres.  Convert before applying.
            t_m_to_a_cm = (t_m_to_a_m[0] * 100.0,
                            t_m_to_a_m[1] * 100.0,
                            t_m_to_a_m[2] * 100.0)

            obj_data = _read_obj_data(temp_obj)
            mtl_file = _find_fusion_mtl(temp_obj)
            member_data.append({
                "name": safe_name,
                "index": i,
                "obj_data": obj_data,
                "mtl_path": mtl_file,
                "R": R_m_to_a,
                "t_cm": t_m_to_a_cm,
            })

        if not member_data:
            return (False, "per-member export produced no members")

        _write_concatenated_obj_mtl(obj_path, mtl_path, member_data, link, log)
        return (True, "")
    finally:
        try:
            import shutil as _shutil
            _shutil.rmtree(temp_dir, ignore_errors=True)
        except Exception:
            pass


def _write_concatenated_obj_mtl(
    obj_path, mtl_path, member_data, link, log,
    source_label="Merged rigid group",
):
    """Stitch member OBJ data + MTLs into one OBJ + one MTL.  Material
    names are namespaced ``<member>__<material>`` to avoid collisions."""
    mtl_basename = os.path.basename(mtl_path)
    merged_mtl_data = {}

    with open(obj_path, 'w', encoding='utf-8') as f:
        f.write(f"# {source_label} OBJ for '{link.urdf_name}' "
                f"({len(member_data)} member(s))\n")
        f.write(f"mtllib {mtl_basename}\n\n")

        v_offset = 0
        n_offset = 0
        t_offset = 0
        total_v = 0
        total_f = 0

        for mem in member_data:
            obj = mem["obj_data"]
            R = mem["R"]
            t = mem["t_cm"]
            ns = mem["name"]
            n_v = len(obj["vertices"])
            n_n = len(obj["normals"])
            n_t = len(obj["texcoords"])
            total_v += n_v
            total_f += len(obj["faces"])

            f.write(f"o member_{mem['index']:02d}_{ns}\n")

            # Vertices: full rigid transform
            for v in obj["vertices"]:
                vx = R[0]*v[0] + R[1]*v[1] + R[2]*v[2] + t[0]
                vy = R[3]*v[0] + R[4]*v[1] + R[5]*v[2] + t[1]
                vz = R[6]*v[0] + R[7]*v[1] + R[8]*v[2] + t[2]
                f.write(f"v {vx:.6f} {vy:.6f} {vz:.6f}\n")

            # Normals: rotation only, no translation
            for n in obj["normals"]:
                nx = R[0]*n[0] + R[1]*n[1] + R[2]*n[2]
                ny = R[3]*n[0] + R[4]*n[1] + R[5]*n[2]
                nz = R[6]*n[0] + R[7]*n[1] + R[8]*n[2]
                f.write(f"vn {nx:.6f} {ny:.6f} {nz:.6f}\n")

            # Texcoords: copy verbatim
            for tc in obj["texcoords"]:
                f.write(f"vt {tc[0]:.6f} {tc[1]:.6f}\n")

            # Faces: offset indices, namespace material
            current_mat = None
            for face in obj["faces"]:
                v_idx, vt_idx, vn_idx, mat = face
                ns_mat = f"{ns}__{mat}" if mat else f"{ns}__default"
                if ns_mat != current_mat:
                    f.write(f"usemtl {ns_mat}\n")
                    current_mat = ns_mat
                f.write("f")
                for j in range(len(v_idx)):
                    v = v_idx[j] + v_offset
                    vt = (vt_idx[j] + t_offset) if (j < len(vt_idx) and vt_idx[j]) else 0
                    n = (vn_idx[j] + n_offset) if (j < len(vn_idx) and vn_idx[j]) else 0
                    if n and vt:
                        f.write(f" {v}/{vt}/{n}")
                    elif n:
                        f.write(f" {v}//{n}")
                    elif vt:
                        f.write(f" {v}/{vt}")
                    else:
                        f.write(f" {v}")
                f.write("\n")
            f.write("\n")

            v_offset += n_v
            n_offset += n_n
            t_offset += n_t

            # Pull materials from the member's MTL for the merged MTL.
            mtl_data = _read_mtl_data(mem.get("mtl_path"))
            for mat_name, lines in mtl_data.items():
                ns_mat = f"{ns}__{mat_name}"
                merged_mtl_data[ns_mat] = lines
            # Also register the implicit ``__default`` namespace if
            # the OBJ used a usemtl without a corresponding MTL entry.
            if not mtl_data:
                merged_mtl_data.setdefault(f"{ns}__default", [])

    # Write merged MTL
    with open(mtl_path, 'w', encoding='utf-8') as f:
        f.write(f"# Merged MTL for {link.urdf_name}\n\n")
        if not merged_mtl_data:
            color = link.color_rgb if getattr(link, "color_rgb", None) else (0.7, 0.7, 0.7)
            mat_name = f"{link.urdf_name}_material"
            f.write(f"newmtl {mat_name}\n")
            f.write(f"Ka {color[0]:.4f} {color[1]:.4f} {color[2]:.4f}\n")
            f.write(f"Kd {color[0]:.4f} {color[1]:.4f} {color[2]:.4f}\n")
            f.write("Ks 0.2000 0.2000 0.2000\nNs 50.0\nd 1.0\nillum 2\n")
        else:
            for mat_name, lines in merged_mtl_data.items():
                f.write(f"newmtl {mat_name}\n")
                if lines:
                    for ln in lines:
                        f.write(f"{ln}\n")
                else:
                    # Member's OBJ had a usemtl but no matching MTL —
                    # synthesize a neutral grey so the URDF still works.
                    f.write("Ka 0.7000 0.7000 0.7000\n")
                    f.write("Kd 0.7000 0.7000 0.7000\n")
                    f.write("Ks 0.2000 0.2000 0.2000\nNs 50.0\nd 1.0\nillum 2\n")
                f.write("\n")

    log(f"    {source_label} ({len(member_data)} members, "
        f"{total_v} verts, {total_f} faces)")


def _export_merged_visual_obj(
    link, snapshot: FusionSnapshot, obj_path: str, mtl_path: str,
    exportMgr, refinement, log: Logger,
):
    """Export a merged rigid-group anchor link's OBJ+MTL.

    Two strategies, picked automatically:

      A. **LCA + visibility (default).** Compute the lowest common
         ancestor of every member, hide non-members under it, call
         ``createOBJExportOptions`` once on the LCA.  Fast, single
         export call, multi-material MTL preserved verbatim.

      B. **Per-member + concatenate (fallback).** When ANY member's
         component is also used by a non-member occurrence (shared
         library parts — fasteners, MGN rails, etc.), strategy A
         leaks the sibling occurrence's geometry into the merged OBJ
         regardless of which visibility flag we toggle.  Switch to
         exporting each member individually, transforming vertices
         into the anchor's local frame, and stitching them together.

    The detection runs once at the top — if ``True``, we delegate to
    :func:`_export_merged_via_per_member` immediately and skip the
    visibility dance entirely.
    """
    member_paths = list(link.merged_member_paths)
    if not member_paths:
        log.warning(f"    Merged link '{link.urdf_name}' has no member paths")
        return (False, "no member paths on the merged link")

    anchor_path = link.occurrence_path
    anchor_occ = snapshot.occurrences.get(anchor_path)
    if not anchor_occ or not anchor_occ._fusion_occurrence:
        log.error(f"    Anchor occurrence missing or no Fusion handle: {anchor_path}")
        return (False, f"anchor occurrence missing: {anchor_path}")

    if any(
        (snapshot.occurrences.get(mp) is not None
         and getattr(snapshot.occurrences[mp], "is_frame_only", False))
        for mp in member_paths
    ):
        log(f"    {link.urdf_name}: rigid group uses frame-only anchor/member "
            f"- using per-member export")
        return _export_merged_via_per_member(
            link, snapshot, obj_path, mtl_path, exportMgr, refinement, log,
        )

    if any(
        (snapshot.occurrences.get(mp) is not None
         and snapshot.occurrences[mp].is_subassembly
         and snapshot.occurrences[mp].body_count > 0)
        for mp in member_paths
    ):
        log(f"    {link.urdf_name}: rigid group has body-owning "
            f"subassembly member(s) - using per-member export")
        return _export_merged_via_per_member(
            link, snapshot, obj_path, mtl_path, exportMgr, refinement, log,
        )

    # Strategy selection: when ANY member's component is shared with a
    # non-member occurrence, the visibility dance can't isolate the
    # group (Fusion renders every occurrence of a shared component
    # regardless of visibility flags).  Fall through to per-member
    # immediately to avoid emitting an OBJ with leaked sibling
    # geometry.
    if _has_shared_component_with_non_member(member_paths, snapshot):
        log(f"    {link.urdf_name}: rigid group has shared-component "
            f"member(s) — using per-member export to avoid sibling-geometry "
            f"leak")
        return _export_merged_via_per_member(
            link, snapshot, obj_path, mtl_path, exportMgr, refinement, log,
        )

    _require_fusion_api()
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design:
        log.error("    No active Fusion design")
        return (False, "no active Fusion design")
    root_comp = design.rootComponent

    # ── Pick export target: LCA sub-asm if all members share one,
    # otherwise the rootComponent ──
    member_set = set(member_paths)
    lca_path = _common_ancestor_path(member_paths)
    target = None
    target_label = ""
    if lca_path:
        lca_occ_data = snapshot.occurrences.get(lca_path)
        if lca_occ_data and lca_occ_data._fusion_occurrence:
            target = lca_occ_data._fusion_occurrence
            target_label = f"sub-asm '{lca_occ_data.clean_name}'"
    if target is None:
        target = root_comp
        target_label = f"rootComponent '{root_comp.name}'"

    log(f"    Merge target: {target_label} (members={len(member_paths)})")

    # ── Identify everything to hide: any leaf occurrence under target
    # that's not in the rigid group, plus the !collision_* member ──
    #
    # Three-layer suppression covering every export-target case.
    # Different layers happen to be honoured by different Fusion
    # internal code paths, so we set all three for non-member
    # occurrences (belt-and-suspenders — whichever the current
    # target's OBJ exporter consults, we're covered):
    #
    #   1. ``Occurrence.isVisible = False``.  Per-occurrence.
    #      Empirically respected by SUB-ASM-targeted OBJ exports —
    #      these were the failing case for the gripper-mounted-on-
    #      Assem1 design where the LCA resolves to the gripper
    #      sub-asm and lightBulb alone wasn't enough to suppress
    #      sibling occurrences of shared components.  Ignored by
    #      rootComponent-targeted exports, so layer 2 covers that.
    #
    #   2. ``Occurrence.isLightBulbOn = False`` — the browser-tree
    #      light bulb the user toggles by hand.  Per-occurrence,
    #      respected by rootComponent OBJ exports (the case d2ef3bd
    #      was originally tested on with gripper-as-design-root).
    #
    #   3. ``Component.bRepBodies[i].isVisible = False`` — global to
    #      the component.  Used to suppress ``!collision_*`` bodies
    #      that share a component with a non-collision visual body
    #      (the body is the only granularity that lets us separate
    #      the two), and as final belt-and-braces.  Gated by
    #      ``member_components``: if any occurrence of this component
    #      is in the current rigid group, we skip the body-level hide
    #      so the member's geometry stays visible (otherwise we'd kill
    #      the member alongside the sibling — leva:2 hide would also
    #      hide leva:1 because they share the same component bodies).
    #
    # Per-occurrence ``Occurrence.bRepBodies[i].isVisible`` proxies
    # delegate to the component-level body — same blast radius as (3),
    # so they're redundant.  Not used.
    member_components = set()
    for mp in member_paths:
        m_data = snapshot.occurrences.get(mp)
        if m_data and m_data._fusion_occurrence is not None:
            try:
                comp = m_data._fusion_occurrence.component
                if comp is not None:
                    member_components.add(comp.entityToken)
            except Exception:
                pass

    # Track restorations as ``(item, attr, prev_value)`` so the
    # finally-block can put each touched property back to whatever
    # the design had originally.
    restorations = []
    try:
        for occ_data in snapshot.occurrences.values():
            if occ_data.is_subassembly:
                continue
            full_path = occ_data.full_path
            # Only consider occurrences inside the LCA's subtree (or
            # all leaves when target is rootComponent).
            if lca_path and not (full_path == lca_path or full_path.startswith(lca_path + '+')):
                continue
            in_group = full_path in member_set
            is_collision = getattr(occ_data, "is_collision_geometry", False)
            if in_group and not is_collision:
                continue  # keep visible
            f_occ = occ_data._fusion_occurrence
            if f_occ is None:
                continue

            # Layer 1: occurrence-level visibility.  Per-occurrence,
            # honoured by SUB-ASM OBJ exports (the gripper-on-Assem1
            # case — LCA resolves to the gripper sub-asm and lightBulb
            # alone wasn't fully suppressing sibling occurrences of
            # shared library components).  Doesn't cascade to siblings.
            try:
                prev = f_occ.isVisible
                if prev:
                    f_occ.isVisible = False
                    restorations.append((f_occ, 'isVisible', prev))
            except Exception:
                pass

            # Layer 2: per-occurrence light bulb.  Honoured by
            # rootComponent OBJ exports (the original gripper-as-
            # design-root case d2ef3bd was tested on).  Per-occurrence;
            # safe for shared components.
            try:
                prev = f_occ.isLightBulbOn
                if prev:
                    f_occ.isLightBulbOn = False
                    restorations.append((f_occ, 'isLightBulbOn', prev))
            except Exception:
                pass

            # Layer 3: component-body visibility — global, only safe
            # when the component is NOT shared with an in-group member.
            try:
                comp = f_occ.component
                comp_token = getattr(comp, 'entityToken', None) if comp else None
                if comp is not None and comp_token not in member_components:
                    for body in comp.bRepBodies:
                        try:
                            prev = body.isVisible
                            if prev:
                                body.isVisible = False
                                restorations.append((body, 'isVisible', prev))
                        except Exception:
                            pass
            except Exception:
                pass

        # Also walk the anchor's collision sub-children and hide them
        # (single-component-style explicit collision lives on the anchor).
        try:
            anchor_fusion = anchor_occ._fusion_occurrence
            for child_occ in anchor_fusion.component.occurrences:
                if is_collision_component_name(child_occ.component.name):
                    try:
                        prev_lb = child_occ.isLightBulbOn
                        if prev_lb:
                            child_occ.isLightBulbOn = False
                            restorations.append((child_occ, 'isLightBulbOn', prev_lb))
                    except Exception:
                        pass
            for body in anchor_fusion.component.bRepBodies:
                if is_collision_body_name(body.name):
                    try:
                        prev = body.isVisible
                        if prev:
                            body.isVisible = False
                            restorations.append((body, 'isVisible', prev))
                    except Exception:
                        pass
        except Exception:
            pass

        # ── Run Fusion's OBJ export ──
        os.makedirs(os.path.dirname(obj_path), exist_ok=True)
        try:
            obj_opts = exportMgr.createOBJExportOptions(target, obj_path)
        except Exception as e:
            log.error(f"    createOBJExportOptions failed: {e}")
            return (False, f"createOBJExportOptions raised: {e}")
        if not obj_opts:
            log.error("    Failed to create OBJExportOptions")
            return (False, "createOBJExportOptions returned None")
        obj_opts.meshRefinement = refinement
        if not exportMgr.execute(obj_opts):
            log.error("    Fusion OBJ export execute() returned False")
            return (False, "exportMgr.execute returned False")
        if not os.path.exists(obj_path):
            log.error(f"    OBJ not at expected path after export: {obj_path}")
            return (False, f"file not at {obj_path} after execute()")
        # 0-byte OBJ: Fusion silently writes an empty file when the
        # filtered geometry under ``target`` has nothing to render.  In
        # the merge path this happens when the rigid-group member's
        # bodies were inadvertently hidden — usually because a sibling
        # rigid group reuses the same component and our visibility pass
        # over-suppressed it.  Tell the user concretely.
        if os.path.getsize(obj_path) == 0:
            try:
                os.remove(obj_path)
            except OSError:
                pass
            reason = (
                "Fusion wrote a 0-byte OBJ from the merge target — every "
                "member body in the rigid group ended up hidden.  Most "
                "common cause: the member component is reused in another "
                "rigid group AND its visibility was suppressed at the "
                "component level.  Re-check that the member is visible "
                "in Fusion's browser tree and that no parent occurrence "
                "is hidden."
            )
            log.error(f"    {reason}")
            return (False, reason)

        # ── Wire up the MTL the same way the per-link path does ──
        mtl_basename = os.path.basename(mtl_path)
        fusion_mtl = _find_fusion_mtl(obj_path)
        if fusion_mtl:
            if fusion_mtl != mtl_path:
                shutil.move(fusion_mtl, mtl_path)
            _patch_obj_mtllib(obj_path, mtl_basename)
            log(f"    MTL preserved from Fusion (multi-material)")
        else:
            color = anchor_occ.appearance_color_rgb or (0.7, 0.7, 0.7)
            mat_name = f"{link.urdf_name}_material"
            _write_mtl_file(mtl_path, mat_name, color)
            _patch_obj_mtl_reference(obj_path, mtl_basename, mat_name)
            log(f"    MTL fallback: RGB({color[0]:.2f}, {color[1]:.2f}, {color[2]:.2f})")

        # ── Anchor frame correction ──
        # Fusion writes vertices in the LCA's frame.  The merged URDF
        # link's frame is the ANCHOR's component frame.  When the anchor
        # is placed at identity within the LCA (the typical case — pendel
        # in pendel_with_esp), no correction is needed.  Otherwise,
        # transform vertices using the anchor's local transform to
        # express them in anchor-local coordinates.
        _maybe_apply_anchor_frame_correction(
            obj_path, anchor_occ, lca_path, snapshot, log,
        )

        kb = os.path.getsize(obj_path) / 1024
        log(f"    Merged OBJ written via Fusion API ({kb:.1f} KB)")
        return (True, "")

    finally:
        # Restore every property we touched, in reverse order, to its
        # original value — putting the design back exactly as we found
        # it (light bulbs, body visibility, anchor's collision sub-occs).
        for item, attr, prev in reversed(restorations):
            try:
                setattr(item, attr, prev)
            except Exception:
                pass


def _maybe_apply_anchor_frame_correction(
    obj_path: str, anchor_occ, lca_path: str, snapshot: FusionSnapshot, log: Logger,
):
    """If the anchor is offset/rotated within the LCA, transform OBJ
    vertices into the anchor's local frame.  No-op when the anchor sits
    at identity within the LCA.

    The LCA-frame OBJ has each vertex v_LCA satisfying:
        v_LCA = R_anchor_in_LCA · v_anchor + t_anchor_in_LCA
    Solving for v_anchor:
        v_anchor = R_anchor_in_LCAᵀ · (v_LCA - t_anchor_in_LCA)
    """
    # Compute anchor's pose relative to LCA.  When LCA is rootComponent
    # (lca_path == ""), use the anchor's transform2 (which is in root
    # frame).  Otherwise, compute (LCA_transform2)⁻¹ · anchor_transform2
    # — but in practice the simpler check below covers the cases we hit.
    if anchor_occ.local_transform is None:
        return
    t_loc = anchor_occ.local_transform.translation
    r_loc = anchor_occ.local_transform.rotation

    is_identity = (
        abs(t_loc[0]) < 1e-9 and abs(t_loc[1]) < 1e-9 and abs(t_loc[2]) < 1e-9
        and all(abs(r_loc[i] - (1.0 if i in (0, 4, 8) else 0.0)) < 1e-9 for i in range(9))
    )
    if is_identity:
        return  # Common case — nothing to do

    log(f"    Applying anchor frame correction "
        f"(anchor not at identity within {lca_path or 'rootComponent'})")

    # Read OBJ, transform v / vn lines, write back.
    # Convert translation from m → cm to match OBJ vertex units.
    CM_PER_M = 100.0
    tx = CM_PER_M * t_loc[0]
    ty = CM_PER_M * t_loc[1]
    tz = CM_PER_M * t_loc[2]
    # r_loc is row-major.  We need the transpose (= inverse for
    # orthonormal rotations) to map LCA → anchor-local.
    rt = (r_loc[0], r_loc[3], r_loc[6],
          r_loc[1], r_loc[4], r_loc[7],
          r_loc[2], r_loc[5], r_loc[8])

    try:
        with open(obj_path, 'r', errors='replace') as f:
            lines = f.readlines()
    except Exception as e:
        log.error(f"    Could not read OBJ for correction: {e}")
        return

    out_lines = []
    for line in lines:
        if line.startswith('v '):
            parts = line.split()
            try:
                x, y, z = float(parts[1]) - tx, float(parts[2]) - ty, float(parts[3]) - tz
                nx = rt[0]*x + rt[1]*y + rt[2]*z
                ny = rt[3]*x + rt[4]*y + rt[5]*z
                nz = rt[6]*x + rt[7]*y + rt[8]*z
                out_lines.append(f"v {nx:.6f} {ny:.6f} {nz:.6f}\n")
                continue
            except (ValueError, IndexError):
                pass
        if line.startswith('vn '):
            parts = line.split()
            try:
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                nx = rt[0]*x + rt[1]*y + rt[2]*z
                ny = rt[3]*x + rt[4]*y + rt[5]*z
                nz = rt[6]*x + rt[7]*y + rt[8]*z
                out_lines.append(f"vn {nx:.6f} {ny:.6f} {nz:.6f}\n")
                continue
            except (ValueError, IndexError):
                pass
        out_lines.append(line)

    try:
        with open(obj_path, 'w', encoding='utf-8') as f:
            f.writelines(out_lines)
    except Exception as e:
        log.error(f"    Could not write corrected OBJ: {e}")
