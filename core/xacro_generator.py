"""
Xacro Generator — Phase 3 of the Fusion URDF Exporter.

Generates hierarchical xacro structure:
  - One xacro file per assembly (macro with prefix parameter)
  - Top-level xacro: includes, instantiates macros, defines mount joints
  - Assembly macros: internal links + internal joints only
  - Mount joints (cross-assembly) live in top-level

This enables the swap workflow:
  Fusion → xacro → swap dummy assemblies with real parts → Gazebo/RViz

Pure Python — no Fusion API imports.

Author: Adrian Valaker Eikeland
"""

import math
import os
import xml.dom.minidom
from typing import Dict, List, Optional, Set

from .data_types import (
    RobotModel, LinkNode, JointNode, JointLimits, AssemblyInfo,
    CollisionInfo, CollisionPrimitive, ExportConfig,
    InertiaTensor, Vec3, RPY,
)
from .ros2_control_generator import (
    generate_ros2_control_xml,
    ros2_control_enabled,
    ros2_control_joints,
)


# Minimum inertia/mass for Gazebo stability
MIN_INERTIA = 1e-6
MIN_MASS = 1e-3

XACRO_NS = 'http://www.ros.org/wiki/xacro'


# ──────────────────────────────────────────────
# Public API
# ──────────────────────────────────────────────

def generate_xacro_package(
    model: RobotModel,
    config: ExportConfig,
    output_dir: str,
) -> Dict[str, str]:
    """
    Generate hierarchical xacro files for the robot.
    
    Args:
        model: Robot model from Phase 2 (with collision resolved)
        config: Export configuration
        output_dir: Directory for urdf/ folder
        
    Returns:
        Dict of relative_path → content for all generated xacro files
    """
    files = {}
    
    # Build lookup: link URDF name → assembly name
    link_to_asm = {}
    for link in model.links.values():
        link_to_asm[link.urdf_name] = link.assembly
    
    # Classify joints: internal vs mount
    internal_joints = {}   # assembly_name → [JointNode]
    mount_joints = []      # list of JointNode
    
    for jname, joint in model.joints.items():
        if joint.is_mount:
            mount_joints.append(joint)
        else:
            parent_asm = link_to_asm.get(joint.parent_link, "")
            internal_joints.setdefault(parent_asm, []).append(joint)
    
    # Collect links per assembly (using URDF names)
    asm_links = {}  # assembly_name → [LinkNode]
    for link in model.links.values():
        asm_links.setdefault(link.assembly, []).append(link)
    
    # Generate only assemblies that actually own links/joints. Empty
    # Fusion wrapper assemblies are useful as browser organization, but
    # exporting them as xacro files creates noise and can collide with
    # real assemblies whose names differ only by case on Windows.
    assembly_names = _exportable_assembly_names(
        model, asm_links, internal_joints
    )
    assembly_file_stems = _assembly_file_stems(assembly_names)

    control_joints_by_asm = {}
    top_level_control_joints = []
    for joint in ros2_control_joints(model):
        parent_asm = link_to_asm.get(joint.parent_link, "")
        child_asm = link_to_asm.get(joint.child_link, "")
        if parent_asm and parent_asm == child_asm:
            control_joints_by_asm.setdefault(parent_asm, []).append(joint)
        else:
            top_level_control_joints.append(joint)

    # Generate assembly xacro files
    for asm_name in assembly_names:
        links = asm_links.get(asm_name, [])
        joints = internal_joints.get(asm_name, [])
        
        content = _generate_assembly_xacro(
            asm_name,
            links,
            joints,
            config,
            model,
            control_joints_by_asm.get(asm_name, []),
        )
        rel_path = f"urdf/assemblies/{assembly_file_stems[asm_name]}.urdf.xacro"
        files[rel_path] = content
    
    # Generate top-level xacro
    top_content = _generate_top_level_xacro(
        model,
        config,
        mount_joints,
        assembly_names,
        assembly_file_stems,
        top_level_control_joints,
    )
    files[f"urdf/{model.name}.urdf.xacro"] = top_content
    
    _remove_stale_assembly_xacros(output_dir, files)

    # Write files
    for rel_path, content in files.items():
        full_path = os.path.join(output_dir, rel_path)
        os.makedirs(os.path.dirname(full_path), exist_ok=True)
        with open(full_path, 'w', encoding='utf-8') as f:
            f.write(content)
    
    return files


def _remove_stale_assembly_xacros(output_dir: str, files: Dict[str, str]) -> None:
    """Delete assembly xacro files from older exports that are no longer emitted."""
    assembly_dir = os.path.join(output_dir, "urdf", "assemblies")
    if not os.path.isdir(assembly_dir):
        return

    expected_exact = {
        os.path.abspath(os.path.join(output_dir, rel_path))
        for rel_path in files
        if rel_path.startswith("urdf/assemblies/")
    }
    expected_norm = {os.path.normcase(path) for path in expected_exact}
    for filename in os.listdir(assembly_dir):
        if not filename.endswith(".urdf.xacro"):
            continue
        path = os.path.abspath(os.path.join(assembly_dir, filename))
        norm_path = os.path.normcase(path)
        if norm_path in expected_norm and path in expected_exact:
            continue
        try:
            os.remove(path)
        except OSError:
            pass


# ──────────────────────────────────────────────
# Top-level xacro
# ──────────────────────────────────────────────

def _generate_top_level_xacro(
    model: RobotModel,
    config: ExportConfig,
    mount_joints: List[JointNode],
    assembly_names: List[str],
    assembly_file_stems: Dict[str, str],
    top_level_control_joints: Optional[List[JointNode]] = None,
) -> str:
    """Generate the top-level xacro that includes and instantiates assemblies."""
    lines = []
    lines.append('<?xml version="1.0"?>')
    lines.append(f'<robot xmlns:xacro="{XACRO_NS}" name="{model.name}">')
    lines.append('')
    
    # Materials (defined once here, referenced in assembly macros)
    materials = _collect_materials(model)
    if materials:
        lines.append('  <!-- Materials (shared across all assemblies) -->')
        for mat_name, rgba in materials.items():
            lines.append(f'  <material name="{_xml_safe(mat_name)}">')
            lines.append(f'    <color rgba="{rgba[0]:.3f} {rgba[1]:.3f} {rgba[2]:.3f} 1.0"/>')
            lines.append(f'  </material>')
        lines.append('')
    
    # Includes
    lines.append('  <!-- Assembly includes -->')
    for asm_name in assembly_names:
        stem = assembly_file_stems.get(asm_name, asm_name)
        lines.append(f'  <xacro:include filename="$(find {config.package_name})/urdf/assemblies/{stem}.urdf.xacro"/>')
    lines.append('')
    
    # Instantiate macros
    lines.append('  <!-- Instantiate assemblies -->')
    for asm_name in assembly_names:
        lines.append(f'  <xacro:{asm_name} prefix=""/>')
    lines.append('')
    
    # Mount joints (cross-assembly connections)
    if mount_joints:
        lines.append('  <!-- Mount joints (cross-assembly connections) -->')
        lines.append('  <!-- These define how assemblies connect to each other. -->')
        lines.append('  <!-- When swapping a dummy assembly with a real part, -->')
        lines.append('  <!-- update the include + macro call above, and adjust -->')
        lines.append('  <!-- parent/child link names in the mount joint below. -->')
        lines.append('')
        
        for joint in mount_joints:
            lines.extend(_generate_joint_xml(joint, indent=2))
            lines.append('')

    top_level_control_joints = top_level_control_joints or []
    if ros2_control_enabled(config, model) and top_level_control_joints:
        lines.extend(generate_ros2_control_xml(
            model,
            config,
            indent=2,
            joints=top_level_control_joints,
            system_name=f"{model.name}_system",
        ))
        lines.append('')
    
    lines.append('</robot>')
    return '\n'.join(lines)


# ──────────────────────────────────────────────
# Assembly xacro macro
# ──────────────────────────────────────────────

def _generate_assembly_xacro(
    asm_name: str,
    links: List[LinkNode],
    joints: List[JointNode],
    config: ExportConfig,
    model: RobotModel,
    control_joints: Optional[List[JointNode]] = None,
) -> str:
    """Generate a xacro macro for one assembly."""
    lines = []
    lines.append('<?xml version="1.0"?>')
    lines.append(f'<robot xmlns:xacro="{XACRO_NS}" name="{asm_name}_macro">')
    lines.append('')
    lines.append(f'  <xacro:macro name="{asm_name}" params="prefix">')
    lines.append('')
    
    # Sort links: root-like first (by position in kinematic chain)
    link_order = _order_links(links, joints, model)
    
    # Materials are defined in the top-level xacro only (to avoid duplicates).
    # Assembly macros just reference them by name in <visual><material name="..."/>.
    
    # Links
    lines.append('    <!-- Links -->')
    for link in link_order:
        lines.extend(_generate_link_xml(link, config, indent=4, use_prefix=True))
        lines.append('')
    
    # Joints
    if joints:
        lines.append('    <!-- Joints (internal to assembly) -->')
        for joint in joints:
            lines.extend(_generate_joint_xml(joint, indent=4, use_prefix=True))
            lines.append('')

    if ros2_control_enabled(config, model) and control_joints:
        lines.extend(generate_ros2_control_xml(
            model,
            config,
            indent=4,
            joints=control_joints,
            system_name=f"{asm_name}_system",
            use_prefix=True,
        ))
        lines.append('')
    
    lines.append(f'  </xacro:macro>')
    lines.append('')
    lines.append('</robot>')
    return '\n'.join(lines)


# ──────────────────────────────────────────────
# Link XML generation
# ──────────────────────────────────────────────

def _generate_link_xml(
    link: LinkNode,
    config: ExportConfig,
    indent: int = 2,
    use_prefix: bool = False,
) -> List[str]:
    """Generate <link> element.
    
    When a link has mesh_bake_offset (joint frame ≠ component origin),
    all sub-element origins are shifted so the mesh, inertial, and
    collision stay aligned with the component geometry.
    """
    lines = []
    pad = ' ' * indent
    name = f'${{prefix}}{link.urdf_name}' if use_prefix else link.urdf_name

    # Explicit !frame_* links are pure named frames: no geometry and no
    # inertial.  This is different from accidental empty components, which
    # retain the minimal inertial below for parser compatibility.
    if getattr(link, "is_frame_only", False):
        lines.append(f'{pad}<link name="{name}"/>')
        return lines
    
    # Empty link (reference frame) — no geometry, minimal inertial
    if link.is_empty:
        lines.append(f'{pad}<link name="{name}">')
        lines.append(f'{pad}  <inertial>')
        lines.append(f'{pad}    <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append(f'{pad}    <mass value="{MIN_MASS}"/>')
        lines.append(f'{pad}    <inertia ixx="{MIN_INERTIA:.6e}" ixy="0" ixz="0"')
        lines.append(f'{pad}             iyy="{MIN_INERTIA:.6e}" iyz="0" izz="{MIN_INERTIA:.6e}"/>')
        lines.append(f'{pad}  </inertial>')
        lines.append(f'{pad}</link>')
        return lines
    
    # Bake offset: shift from joint frame to component origin
    # Applied to visual, inertial, and collision origins
    bake = link.mesh_bake_offset if link.needs_mesh_bake else (0.0, 0.0, 0.0)
    
    lines.append(f'{pad}<link name="{name}">')
    
    # Inertial — CoM is relative to component origin, shift by bake offset
    mass = max(link.mass_kg, MIN_MASS)
    com = (
        link.com_link_local[0] + bake[0],
        link.com_link_local[1] + bake[1],
        link.com_link_local[2] + bake[2],
    )
    inertia = link.inertia_at_com
    ixx = max(abs(inertia.ixx), MIN_INERTIA)
    iyy = max(abs(inertia.iyy), MIN_INERTIA)
    izz = max(abs(inertia.izz), MIN_INERTIA)
    
    lines.append(f'{pad}  <inertial>')
    lines.append(f'{pad}    <origin xyz="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}" rpy="0 0 0"/>')
    lines.append(f'{pad}    <mass value="{mass:.6f}"/>')
    lines.append(f'{pad}    <inertia ixx="{ixx:.6e}" ixy="{inertia.ixy:.6e}" ixz="{inertia.ixz:.6e}"')
    lines.append(f'{pad}             iyy="{iyy:.6e}" iyz="{inertia.iyz:.6e}" izz="{izz:.6e}"/>')
    lines.append(f'{pad}  </inertial>')
    
    # Mesh scale depends on format:
    #   .obj/.stl from Fusion → centimeters → scale="0.01" converts to URDF meters.
    #   .dae from our converter → already in meters → scale="1".
    visual_scale = _scale_for_mesh(link.mesh_visual) if link.mesh_visual else "1 1 1"

    # Visual — shifted by bake offset; scale matches the mesh file's units.
    # ``has_visual_mesh`` flips to False when the export pipeline failed
    # to write a real OBJ/DAE for this link.  In that case we omit the
    # ``<visual>`` element entirely — better a link with no visual than
    # a URDF that points at a missing file (404 in URDF previews and a
    # silent failure in many parsers).
    if link.has_visual_mesh and link.mesh_visual:
        lines.append(f'{pad}  <visual>')
        lines.append(f'{pad}    <origin xyz="{bake[0]:.6f} {bake[1]:.6f} {bake[2]:.6f}" rpy="0 0 0"/>')
        lines.append(f'{pad}    <geometry>')
        lines.append(f'{pad}      <mesh filename="package://{config.package_name}/{link.mesh_visual}" scale="{visual_scale}"/>')
        lines.append(f'{pad}    </geometry>')
        if link.material_name:
            lines.append(f'{pad}    <material name="{_xml_safe(link.material_name)}"/>')
        lines.append(f'{pad}  </visual>')

    # Collision — mesh or STL, shifted by bake offset + rigid group offset
    collision = link.collision
    # Collision origin: bake offset + rigid group offset (if from a different component)
    coll_origin = list(bake)
    if link.rigid_group_collision_offset:
        rgo = link.rigid_group_collision_offset
        coll_origin[0] += rgo[0]
        coll_origin[1] += rgo[1]
        coll_origin[2] += rgo[2]
    if collision and collision.origin_xyz:
        coll_origin = list(collision.origin_xyz)  # full override

    if collision and collision.mesh_path:
        # Collision STL (explicit from Fusion, or generated automatically).
        # STL is always in centimeters (rescaled at export); scale="0.01".
        coll_scale = _scale_for_mesh(collision.mesh_path)
        lines.append(f'{pad}  <collision>')
        lines.append(f'{pad}    <origin xyz="{coll_origin[0]:.6f} {coll_origin[1]:.6f} {coll_origin[2]:.6f}" rpy="0 0 0"/>')
        lines.append(f'{pad}    <geometry>')
        lines.append(f'{pad}      <mesh filename="package://{config.package_name}/{collision.mesh_path}" scale="{coll_scale}"/>')
        lines.append(f'{pad}    </geometry>')
        lines.append(f'{pad}  </collision>')
    elif link.has_visual_mesh and link.mesh_visual:
        # Fallback: use visual mesh as collision (only when the visual
        # actually exists — otherwise we'd reference a missing file).
        lines.append(f'{pad}  <collision>')
        lines.append(f'{pad}    <origin xyz="{bake[0]:.6f} {bake[1]:.6f} {bake[2]:.6f}" rpy="0 0 0"/>')
        lines.append(f'{pad}    <geometry>')
        lines.append(f'{pad}      <mesh filename="package://{config.package_name}/{link.mesh_visual}" scale="{visual_scale}"/>')
        lines.append(f'{pad}    </geometry>')
        lines.append(f'{pad}  </collision>')
    # Else: no collision element at all — the link still functions as a
    # kinematic frame with mass/inertia.  Better than referencing a
    # nonexistent mesh.
    
    lines.append(f'{pad}</link>')
    return lines


def _exportable_assembly_names(
    model: RobotModel,
    asm_links: Dict[str, List[LinkNode]],
    internal_joints: Dict[str, List[JointNode]],
) -> List[str]:
    """Assemblies that need a xacro macro.

    Empty Fusion wrapper assemblies should stay out of the generated
    package. They have no links and no internal joints, so including them
    only creates empty files/folders and can overwrite real assemblies on
    case-insensitive filesystems.
    """
    names = []
    for asm_name in model.assemblies:
        if asm_links.get(asm_name) or internal_joints.get(asm_name):
            names.append(asm_name)
    return names


def _assembly_file_stems(assembly_names: List[str]) -> Dict[str, str]:
    """Map assembly macro names to case-insensitive-unique filenames."""
    stems = {}
    used_lower = set()
    for asm_name in assembly_names:
        base = asm_name or "assembly"
        stem = base
        idx = 2
        while stem.lower() in used_lower:
            stem = f"{base}_{idx}"
            idx += 1
        stems[asm_name] = stem
        used_lower.add(stem.lower())
    return stems


# ──────────────────────────────────────────────
# Joint XML generation
# ──────────────────────────────────────────────

def _generate_joint_xml(
    joint: JointNode,
    indent: int = 2,
    use_prefix: bool = False,
) -> List[str]:
    """Generate <joint> element."""
    lines = []
    pad = ' ' * indent
    pfx = '${prefix}' if use_prefix else ''
    
    o = joint.origin_xyz
    r = joint.origin_rpy
    a = joint.axis
    
    lines.append(f'{pad}<joint name="{pfx}{joint.name}" type="{joint.joint_type}">')
    lines.append(f'{pad}  <parent link="{pfx}{joint.parent_link}"/>')
    lines.append(f'{pad}  <child link="{pfx}{joint.child_link}"/>')
    lines.append(f'{pad}  <origin xyz="{o[0]:.6f} {o[1]:.6f} {o[2]:.6f}" '
                f'rpy="{r[0]:.6f} {r[1]:.6f} {r[2]:.6f}"/>')
    
    if joint.joint_type in ('revolute', 'prismatic', 'continuous'):
        lines.append(f'{pad}  <axis xyz="{a[0]:.6f} {a[1]:.6f} {a[2]:.6f}"/>')
    
    if joint.limits and joint.joint_type in ('revolute', 'prismatic'):
        lim = joint.limits
        lines.append(f'{pad}  <limit lower="{lim.lower:.6f}" upper="{lim.upper:.6f}" '
                    f'effort="{lim.effort:.1f}" velocity="{lim.velocity:.2f}"/>')
    elif joint.joint_type == 'continuous':
        # URDF continuous joints have no lower/upper by definition.
        # effort/velocity still help Gazebo/physics sims size the actuator.
        lines.append(f'{pad}  <limit effort="100.0" velocity="1.00"/>')
    elif joint.joint_type == 'prismatic':
        # Unbounded prismatic has no URDF equivalent to `continuous`;
        # fall back to a large numeric range so the URDF validates.
        lines.append(f'{pad}  <limit lower="-1000.000000" upper="1000.000000" '
                    f'effort="100.0" velocity="1.00"/>')

    lines.append(f'{pad}</joint>')
    return lines


# ──────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────

def _order_links(
    links: List[LinkNode],
    joints: List[JointNode],
    model: RobotModel,
) -> List[LinkNode]:
    """Order links: root-adjacent first, then BFS through local joints."""
    link_map = {l.urdf_name: l for l in links}
    children = {}
    for j in joints:
        children.setdefault(j.parent_link, []).append(j.child_link)
    
    ordered = []
    visited = set()
    
    # Start from links that are parents but not children (local roots)
    child_set = {j.child_link for j in joints}
    local_roots = [l for l in links if l.urdf_name not in child_set]
    
    if not local_roots:
        local_roots = links[:1] if links else []
    
    queue = [l.urdf_name for l in local_roots]
    
    while queue:
        current = queue.pop(0)
        if current in visited:
            continue
        visited.add(current)
        if current in link_map:
            ordered.append(link_map[current])
        for child_name in children.get(current, []):
            if child_name not in visited:
                queue.append(child_name)
    
    # Add any remaining (orphaned within assembly)
    for link in links:
        if link.urdf_name not in visited:
            ordered.append(link)
    
    return ordered


def _xml_safe(s: str) -> str:
    """Make string safe for XML attribute."""
    return s.replace('&', '&amp;').replace('"', '&quot;').replace('<', '&lt;').replace('>', '&gt;')


def _scale_for_mesh(mesh_path: str) -> str:
    """URDF ``scale="..."`` value matching the mesh file's units.

    * ``.dae`` from our converter writes meters (per COLLADA's
      ``<unit name="meter" meter="1.0"/>``) → ``"1 1 1"``.
    * ``.obj`` and ``.stl`` from Fusion are centimeters → ``"0.01 0.01 0.01"``.

    URDF expects meshes in meters; the scale converts whatever the
    file is in into meters at load time.
    """
    if mesh_path.lower().endswith('.dae'):
        return "1 1 1"
    return "0.01 0.01 0.01"


# ══════════════════════════════════════════════
# FLAT URDF (for validation / debugging)
# ══════════════════════════════════════════════
# Kept here alongside xacro generation — same data, different format.
# Xacro is the primary output; flat URDF is for quick validation
# without requiring a xacro processor.



# ──────────────────────────────────────────────
# Public API
# ──────────────────────────────────────────────

def generate_urdf(model: RobotModel, config: ExportConfig) -> str:
    """
    Generate complete URDF XML string from a RobotModel.
    
    Args:
        model: Robot model from Phase 2
        config: Export configuration
        
    Returns:
        Pretty-printed URDF XML string
    """
    if not model.root_link or model.root_link not in model.links:
        raise ValueError(
            f"Cannot generate URDF: root link {model.root_link!r} is not a "
            f"resolvable link. Likely cause: a joint's parent or child "
            f"resolves to a subassembly instead of a leaf component. "
            f"See DESIGN_RULES.md §1.1 Link Names, §1.2 Joint Names, "
            f"and §3.1 Kinematic Tree, plus the validation errors in the "
            f"export log."
        )

    lines = []
    lines.append('<?xml version="1.0"?>')
    lines.append(f'<robot name="{model.name}">')
    lines.append('')

    # Materials (deduplicated)
    materials = _collect_materials(model)
    if materials:
        lines.append('  <!-- Materials -->')
        for mat_name, rgba in materials.items():
            lines.append(f'  <material name="{_xml_safe(mat_name)}">')
            lines.append(f'    <color rgba="{rgba[0]:.3f} {rgba[1]:.3f} {rgba[2]:.3f} 1.0"/>')
            lines.append(f'  </material>')
        lines.append('')

    # Links — ordered: root first, then by kinematic chain
    link_order = _kinematic_order(model)
    
    for link_name in link_order:
        link = model.links[link_name]
        lines.extend(_generate_link(link, config, model))
        lines.append('')
    
    # Joints — ordered to match link order
    joint_order = _joint_order(model, link_order)
    
    for joint_name in joint_order:
        joint = model.joints[joint_name]
        lines.extend(_generate_joint(joint))
        lines.append('')

    if ros2_control_enabled(config, model):
        lines.extend(generate_ros2_control_xml(model, config, indent=2))
        lines.append('')
    
    lines.append('</robot>')
    
    return '\n'.join(lines)


# ──────────────────────────────────────────────
# Link generation
# ──────────────────────────────────────────────

def _generate_link(link: LinkNode, config: ExportConfig, model: RobotModel) -> list:
    """Generate URDF <link> element (flat URDF version)."""
    # Reuse the xacro link generator without prefix
    return _generate_link_xml(link, config, indent=2, use_prefix=False)


# ──────────────────────────────────────────────
# Joint generation (flat URDF)
# ──────────────────────────────────────────────

def _generate_joint(joint: JointNode) -> list:
    """Generate URDF <joint> element."""
    lines = []
    
    o = joint.origin_xyz
    r = joint.origin_rpy
    a = joint.axis
    
    lines.append(f'  <joint name="{joint.name}" type="{joint.joint_type}">')
    lines.append(f'    <parent link="{joint.parent_link}"/>')
    lines.append(f'    <child link="{joint.child_link}"/>')
    lines.append(f'    <origin xyz="{o[0]:.6f} {o[1]:.6f} {o[2]:.6f}" '
                f'rpy="{r[0]:.6f} {r[1]:.6f} {r[2]:.6f}"/>')
    
    # Axis (only meaningful for revolute/prismatic/continuous)
    if joint.joint_type in ('revolute', 'prismatic', 'continuous'):
        lines.append(f'    <axis xyz="{a[0]:.6f} {a[1]:.6f} {a[2]:.6f}"/>')
    
    # Limits (required for revolute/prismatic; continuous carries only effort/velocity)
    if joint.limits and joint.joint_type in ('revolute', 'prismatic'):
        lim = joint.limits
        lines.append(f'    <limit lower="{lim.lower:.6f}" upper="{lim.upper:.6f}" '
                    f'effort="{lim.effort:.1f}" velocity="{lim.velocity:.2f}"/>')
    elif joint.joint_type == 'continuous':
        lines.append(f'    <limit effort="100.0" velocity="1.00"/>')
    elif joint.joint_type == 'prismatic':
        # Unbounded prismatic: URDF requires lower/upper, large range as fallback
        lines.append(f'    <limit lower="-1000.000000" upper="1000.000000" '
                    f'effort="100.0" velocity="1.00"/>')
    
    lines.append(f'  </joint>')
    return lines


# ──────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────

def _collect_materials(model: RobotModel) -> dict:
    """Collect unique material definitions."""
    materials = {}
    for link in model.links.values():
        if link.material_name and link.color_rgb:
            if link.material_name not in materials:
                materials[link.material_name] = link.color_rgb
    return materials


def _kinematic_order(model: RobotModel) -> list:
    """Order links: root first, then BFS through kinematic tree."""
    ordered = []
    visited = set()
    queue = [model.root_link]
    
    while queue:
        current = queue.pop(0)
        if current in visited:
            continue
        visited.add(current)
        ordered.append(current)
        
        # Add children in joint order
        for joint in model.joints.values():
            if joint.parent_link == current and joint.child_link not in visited:
                queue.append(joint.child_link)
    
    # Add any orphaned links at the end
    for name in model.links:
        if name not in visited:
            ordered.append(name)
    
    return ordered


def _joint_order(model: RobotModel, link_order: list) -> list:
    """Order joints to match child link order."""
    ordered = []
    seen = set()
    
    for link_name in link_order:
        for jname, joint in model.joints.items():
            if joint.child_link == link_name and jname not in seen:
                ordered.append(jname)
                seen.add(jname)
    
    # Add remaining
    for jname in model.joints:
        if jname not in seen:
            ordered.append(jname)
    
    return ordered
