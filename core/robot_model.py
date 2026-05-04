"""
Robot Model Builder — Phase 2 of the Fusion URDF Exporter.

Takes a FusionSnapshot (Phase 1 output) and builds a RobotModel:
  - Resolves joint occurrence paths (local → global)
  - Builds assembly hierarchy from occurrence paths
  - Auto-detects root link (no base_link naming requirement)
  - Resolves name collisions (prefix only when necessary)
  - Classifies joints (internal vs mount/cross-assembly)
  - Computes URDF joint origins (parent-relative)
  - Computes mesh bake offsets for revolute/prismatic children
  - Validates kinematic tree

Pure Python — no Fusion API imports. Testable with serialized snapshot.

Author: Adrian Valaker Eikeland
"""

import math
from dataclasses import dataclass, field as dc_field
from typing import Dict, List, Optional, Set, Tuple

from .data_types import (
    FusionSnapshot, FusionOccurrence, FusionJoint,
    RobotModel, LinkNode, JointNode, JointLimits, AssemblyInfo,
    InertiaTensor, Vec3, RPY, RigidGroupInfo,
    DESIGN_ROOT_OCCURRENCE_PATH,
)
from ..utils import Logger, clean_name


# ──────────────────────────────────────────────
# Internal types
# ──────────────────────────────────────────────

@dataclass
class KinematicEdge:
    """Resolved joint connecting two components (internal to model builder)."""
    joint_name: str = ""
    parent_asm: str = ""
    parent_comp: str = ""
    parent_path: str = ""        # full occurrence path (unique disambiguator)
    child_asm: str = ""
    child_comp: str = ""
    child_path: str = ""         # full occurrence path (unique disambiguator)
    is_cross_assembly: bool = False
    motion_type: str = ""
    defining_component: str = ""
    fusion_joint: object = dc_field(repr=False, default=None)
    # Closing-loop classification — see ``_classify_closing_joints``.
    # True when the joint should be excluded from the URDF tree and
    # emitted to the closing-joints sidecar.  ``closing_source``
    # records how it was identified.
    is_closing: bool = False
    closing_source: str = ""     # "user_tag" | "auto_detected" | ""


# ──────────────────────────────────────────────
# Rotation helpers
#
# DEBUG / THEORY — joint origin rotation (pendulum bug, 2026-04-13)
#
# Symptom: a pendulum authored in Fusion with a revolute joint that
# visually hangs straight down in the assembly came out of the URDF
# exporter with the rod extending along the link's local +Y axis
# instead of -Z.  The CoM landed 30° off the joint's rotation axis,
# so physics equilibrium was at joint θ ≈ -30° rather than 0, and
# consumers had to hand-edit the xacro with rpy compensators on
# visual/collision/joint-origin to make the simulation behave.
#
# Theory: Fusion represents a joint mate with two `JointOrigin`
# objects — one on each mated occurrence — each carrying BOTH a
# position AND an orientation.  When the mate is applied, the child
# occurrence's `transform2` captures the resulting design pose
# relative to its parent assembly.  The rotation part of that
# transform is the orientation the user actually assembled.
#
# Until now the exporter:
#   • captured `transform2.rotation` on each FusionOccurrence
#     (fusion_extractor._extract_transforms → _extract_rotation), BUT
#   • only accumulated translation through the assemblyContext chain
#     when computing `global_transform` (rotation stayed at identity),
#   • never read orientation from `geometryOrOriginOne/Two`, and
#   • hardcoded URDF `<joint><origin rpy="0 0 0"/>` at
#     `_build_joints` line ~808 with the comment "No rotation — link
#     frames are global-aligned".
#
# Net effect: the user's assembled orientation was silently dropped
# and every downstream consumer had to recover it from CoM geometry.
#
# Fix (this change):
#   1. Walk the same assemblyContext chain that `_extract_transforms`
#      walks for translation, but for rotation — multiply the local
#      rotations together to get a real global rotation on
#      `FusionOccurrence.global_transform.rotation`.
#   2. In `_build_joints`, compute the URDF joint origin rpy as the
#      child occurrence's global rotation expressed in the parent's
#      global frame (R_parent⁻¹ · R_child), converted to extrinsic
#      XYZ Euler angles (URDF rpy convention: R = Rz·Ry·Rx).
#   3. Log what we extracted so this is visible in the export log.
#
# Scope / limitations:
#   • Handles revolute, prismatic, and fixed joints the same way —
#     the child's pose at rest is whatever Fusion mated it to.
#   • Does NOT read `JointOrigin.geometry` orientation (second-order
#     improvement — transform2 already captures the effective pose
#     for standard mates).
#   • Nested assemblies are handled via context-chain accumulation;
#     if a non-identity transform sits between the joint's owning
#     assembly and a mated occurrence, it composes correctly.
#   • Assumes right-handed, orthonormal rotation matrices (true for
#     Fusion `Matrix3D`).  No orthonormalisation pass on output.
# ──────────────────────────────────────────────

_IDENTITY_3X3: Tuple[float, ...] = (1.0, 0.0, 0.0,
                                     0.0, 1.0, 0.0,
                                     0.0, 0.0, 1.0)


def _mat3_mul(a: Tuple[float, ...], b: Tuple[float, ...]) -> Tuple[float, ...]:
    """Row-major 3×3 multiply: returns a · b."""
    return (
        a[0]*b[0] + a[1]*b[3] + a[2]*b[6],
        a[0]*b[1] + a[1]*b[4] + a[2]*b[7],
        a[0]*b[2] + a[1]*b[5] + a[2]*b[8],
        a[3]*b[0] + a[4]*b[3] + a[5]*b[6],
        a[3]*b[1] + a[4]*b[4] + a[5]*b[7],
        a[3]*b[2] + a[4]*b[5] + a[5]*b[8],
        a[6]*b[0] + a[7]*b[3] + a[8]*b[6],
        a[6]*b[1] + a[7]*b[4] + a[8]*b[7],
        a[6]*b[2] + a[7]*b[5] + a[8]*b[8],
    )


def _mat3_transpose(m: Tuple[float, ...]) -> Tuple[float, ...]:
    """Transpose a row-major 3×3 matrix (= inverse for orthonormal rotations)."""
    return (m[0], m[3], m[6],
            m[1], m[4], m[7],
            m[2], m[5], m[8])


def _rotation_to_rpy(m: Tuple[float, ...]) -> Tuple[float, float, float]:
    """Convert a row-major 3×3 rotation matrix to URDF rpy (roll, pitch, yaw).

    URDF convention: R = Rz(yaw) · Ry(pitch) · Rx(roll) (extrinsic XYZ).
    Handles gimbal lock (pitch = ±π/2) by zeroing yaw and recovering roll.
    """
    r11, r12, r13, r21, r22, r23, r31, r32, r33 = m
    # Clamp to guard against tiny numerical drift outside [-1, 1].
    sp = -r31
    if sp > 1.0:
        sp = 1.0
    elif sp < -1.0:
        sp = -1.0
    pitch = math.asin(sp)
    # cos(pitch) ≈ 0 → gimbal lock
    if abs(math.cos(pitch)) < 1e-9:
        roll = math.atan2(-r23, r22)
        yaw = 0.0
    else:
        roll = math.atan2(r32, r33)
        yaw = math.atan2(r21, r11)
    return (roll, pitch, yaw)


def _global_rotation_for_occurrence(
    occ_path: str, snapshot: FusionSnapshot
) -> Tuple[float, ...]:
    """Return the occurrence's WORLD rotation as a row-major 3×3 matrix.

    Reads ``transform2.rotation`` directly — Fusion's ``transform2`` is
    already composed through the assemblyContext chain on Fusion's side,
    despite the API documentation saying "relative to parent component."
    Empirical evidence (``debug/fusion_transforms.json`` from the
    pendulum design): for ``pendel`` at depth 1 inside ``pendel_with_esp``,
    pendel.transform2.translation matches pwe.transform2.translation
    because pendel sits at local (0, 0, 0) within pwe — the rotation
    component similarly already includes the parent's effect.

    Walking ``parent_path`` and multiplying transform2.rotation at each
    step (the previous behavior of this function) DOUBLE-applied the
    parent transform, producing M² where the truth was M.  That's why
    URDF joint origin rpy shipped as (-π/2, -π/2, 0) instead of the
    correct (π/2, 0, π/2) for ``pendel_joint`` and broke any consumer
    that took the URDF orientation literally.

    Falls back to ``local_transform.rotation`` when ``transform2`` is
    absent (synthetic test snapshots without a Fusion API).  Identity
    when the occurrence is missing entirely.
    """
    occ = snapshot.occurrences.get(occ_path)
    if occ is None:
        return _IDENTITY_3X3
    if occ.transform2 is not None:
        return occ.transform2.rotation
    if occ.local_transform is not None:
        return occ.local_transform.rotation
    return _IDENTITY_3X3


def _joint_origin_rpy(
    child_full_path: str, parent_full_path: str, snapshot: FusionSnapshot
) -> Tuple[Tuple[float, float, float], Tuple[float, ...], Tuple[float, ...]]:
    """Compute URDF joint origin rpy from the child's mated pose.

    Takes FULL occurrence paths (keys into ``snapshot.occurrences``) — the
    FusionJoint's ``occurrence_one_path``/``occurrence_two_path`` are
    context-local relative to the joint's defining component and won't key
    into the occurrences dict directly.  Callers must resolve first (see
    ``_build_joints``).

    Returns ``(rpy, child_global_rot, parent_global_rot)`` — the rpy is what
    goes into ``<joint><origin rpy="..."/>``; the two rotation matrices are
    returned for diagnostic logging.

    The joint origin rpy expresses the child link's orientation at joint
    angle θ = 0, in the parent link's frame:
        R_joint_origin = R_parent_global⁻¹ · R_child_global
    """
    r_child = _global_rotation_for_occurrence(child_full_path, snapshot)
    r_parent = _global_rotation_for_occurrence(parent_full_path, snapshot)
    r_rel = _mat3_mul(_mat3_transpose(r_parent), r_child)
    return _rotation_to_rpy(r_rel), r_child, r_parent


def _rotate_vec3_by_mat3(v: Vec3, m: Tuple[float, ...]) -> Vec3:
    """Apply a row-major 3×3 rotation to a 3-vector: returns m · v."""
    return (
        m[0]*v[0] + m[1]*v[1] + m[2]*v[2],
        m[3]*v[0] + m[4]*v[1] + m[5]*v[2],
        m[6]*v[0] + m[7]*v[1] + m[8]*v[2],
    )


# ──────────────────────────────────────────────
# Public API
# ──────────────────────────────────────────────

def build_model(snapshot: FusionSnapshot, log: Logger) -> RobotModel:
    """
    Build a RobotModel from a FusionSnapshot.
    
    Args:
        snapshot: Complete extraction from Phase 1
        log: Logger instance
        
    Returns:
        RobotModel ready for Phase 3 generators
    """
    model = RobotModel(name=snapshot.design_name_clean)
    
    log.section("MODEL: ASSEMBLY HIERARCHY")
    assemblies, occ_to_asm = _build_assembly_hierarchy(snapshot, model, log)
    
    log.section("MODEL: RIGID GROUP MERGE")
    _process_rigid_groups(snapshot, occ_to_asm, model, log)
    
    log.section("MODEL: RESOLVE JOINT PATHS")
    edges = _resolve_joint_paths(snapshot, occ_to_asm, model, log)

    _drop_unreferenced_empty_occurrences(snapshot, occ_to_asm, edges, model, log)
    
    log.section("MODEL: DETECT ROOT")
    root_asm, root_comp = _detect_root(edges, model, log)
    
    log.section("MODEL: RESOLVE NAMES")
    name_map = _resolve_names(snapshot, occ_to_asm, root_asm, root_comp, model, log)
    
    log.section("MODEL: BUILD LINKS")
    _build_links(snapshot, occ_to_asm, name_map, model, log)
    
    log.section("MODEL: BUILD JOINTS")
    _build_joints(snapshot, edges, assemblies, name_map, occ_to_asm, model, log)
    
    log.section("MODEL: VALIDATE")
    _validate(model, log)
    
    log.section("MODEL SUMMARY")
    log(f"  Robot: {model.name}")
    log(f"  Root link: {model.root_link}")
    log(f"  Links: {len(model.links)}")
    log(f"  Joints: {len(model.joints)}")
    log(f"  Assemblies: {len(model.assemblies)}")
    log(f"  Warnings: {len(model.warnings)}")
    log(f"  Errors: {len(model.errors)}")
    
    return model


# ──────────────────────────────────────────────
# Step 1: Build assembly hierarchy
# ──────────────────────────────────────────────

def _occurrence_is_link_candidate(occ: FusionOccurrence) -> bool:
    """True when an occurrence should become or merge into a URDF link."""
    return (not occ.is_subassembly) or occ.body_count > 0


def _build_assembly_hierarchy(
    snapshot: FusionSnapshot,
    model: RobotModel,
    log: Logger,
) -> Tuple[Dict[str, AssemblyInfo], Dict[str, str]]:
    """
    Parse occurrence paths to identify assemblies and assign components.
    
    Returns:
        assemblies: name → AssemblyInfo
        occ_to_asm: occurrence full_path → assembly name
    """
    assemblies: Dict[str, AssemblyInfo] = {}
    occ_to_asm: Dict[str, str] = {}

    # Synthetic root-assembly name for designs with leaf components
    # placed directly under the design root (no Fusion sub-assembly
    # wrapping them).  Phase 2 expects every component in
    # ``lw_components`` to have a ``urdf/assemblies/<name>.urdf.xacro``
    # macro it can xacro:include + xacro:macro instantiate; without a
    # real sub-asm we'd produce an empty top-level xacro (just
    # materials, no links).  Wrap the root-level leaves into one
    # synthetic assembly named after the design so the xacro generator
    # emits a proper macro file.  ``model.name`` is the
    # ``design_name_clean`` from Fusion (already an ASCII identifier).
    synthetic_root_name = model.name or "main"

    # First pass: identify all assemblies (subassembly occurrences).
    # Capture both translation (global_offset) and rotation
    # (global_rotation) from the sub-asm's transform2 — joints defined
    # inside a sub-asm have their origins/axes in the sub-asm's local
    # frame, and we'll need both to lift them into world frame later.
    for path, occ in snapshot.occurrences.items():
        if occ.is_subassembly:
            name = occ.clean_name
            if name not in assemblies:
                global_rot = (1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0)
                if occ.transform2 is not None and occ.transform2.rotation:
                    global_rot = occ.transform2.rotation
                assemblies[name] = AssemblyInfo(
                    name=name,
                    global_offset=occ.global_position,
                    global_rotation=global_rot,
                    depth=occ.depth,
                )
            log(f"  Assembly: {name} d={occ.depth} offset=({occ.global_position[0]*1000:.1f}, {occ.global_position[1]*1000:.1f}, {occ.global_position[2]*1000:.1f}) mm")

    # Second pass: assign physical link candidates to their immediate
    # parent assembly.  Most subassemblies are containers only, but an
    # imported subassembly can own direct bodies and child occurrences;
    # those direct bodies need a URDF link or rigid-group merge member.
    # Leaves at the design root are routed to the synthetic root
    # assembly created lazily on first use.
    for path, occ in snapshot.occurrences.items():
        if not _occurrence_is_link_candidate(occ):
            continue

        # Find immediate parent assembly: second-to-last segment in path
        segs = occ.path_segments
        if len(segs) >= 2:
            parent_asm = segs[-2]
        else:
            parent_asm = synthetic_root_name
            if synthetic_root_name not in assemblies:
                assemblies[synthetic_root_name] = AssemblyInfo(
                    name=synthetic_root_name,
                    global_offset=(0.0, 0.0, 0.0),
                    depth=0,
                )
                log(
                    f"  Assembly: {synthetic_root_name} (synthetic root, "
                    f"wraps design-root leaves so phase 2 has a macro "
                    f"to xacro:include)"
                )

        occ_to_asm[path] = parent_asm

        # NOTE: ``assemblies[asm].links`` is populated in ``_build_links``
        # once the URDF names are resolved (with proper uniquification
        # for re-used components — e.g. ``leva``, ``leva_2``).  Doing it
        # here too with ``clean_name`` produced a list with both
        # clean-name duplicates AND urdf-name entries, e.g. four
        # ``AS_1420_1973_M3_x_6`` entries plus the actual unique URDF
        # names — visible in robot_data.yaml's assemblies block.

        log(f"  {occ.clean_name} → {parent_asm}")
    
    # Build parent-child relationships between assemblies
    for path, occ in snapshot.occurrences.items():
        if not occ.is_subassembly:
            continue
        
        name = occ.clean_name
        segs = occ.path_segments
        
        if len(segs) >= 2:
            parent_asm = segs[-2]
            if parent_asm in assemblies:
                asm = assemblies[name]
                asm.parent_assembly = parent_asm
                if name not in assemblies[parent_asm].children_assemblies:
                    assemblies[parent_asm].children_assemblies.append(name)
    
    # ── Collision sub-assembly flattening ──────────────
    # Pattern: a sub-assembly with exactly 2 leaf children where one is
    # tagged ``!collision_*``.  The collision child is not a URDF link;
    # it provides explicit collision mesh for the visual sibling.
    _flatten_collision_subassemblies(
        snapshot, assemblies, occ_to_asm, model, log
    )
    
    model.assemblies = assemblies
    return assemblies, occ_to_asm


def _flatten_collision_subassemblies(
    snapshot: FusionSnapshot,
    assemblies: Dict[str, AssemblyInfo],
    occ_to_asm: Dict[str, str],
    model: RobotModel,
    log: Logger,
):
    """
    Detect and flatten collision sub-assemblies.
    
    A collision sub-assembly has exactly 2 leaf children where one is
    tagged ``!collision_*``.  The visual leaf gets re-parented to the grandparent
    assembly and the collision leaf is marked for explicit collision export.
    
    Example Fusion structure:
        dummy_zed2i/                   ← parent assembly
          zed2i_camera_link            ← leaf
          zed2i_center_link/           ← collision sub-asm (FLATTENED)
            zed2i_link                 ← visual leaf → re-parented to dummy_zed2i
            !collision_zed             ← collision → provides STL for zed2i_link
    
    After flattening, joints that target zed2i_center_link (the sub-asm)
    will be redirected to zed2i_link (the visual leaf) via
    model._collision_flatten_redirects.
    """
    to_remove = []
    
    # model stores redirect info for joint resolution
    if not hasattr(model, '_collision_flatten_redirects'):
        model._collision_flatten_redirects = {}
    if not hasattr(model, '_collision_flatten_pairs'):
        model._collision_flatten_pairs = {}
    
    for asm_name, asm_info in list(assemblies.items()):
        links = asm_info.links
        if len(links) != 2:
            continue
        
        # Identify collision vs visual leaf
        collision_name = None
        visual_name = None
        for name in links:
            is_collision_leaf = any(
                assigned_asm == asm_name
                and (occ := snapshot.occurrences.get(path)) is not None
                and occ.clean_name == name
                and getattr(occ, "is_collision_geometry", False)
                for path, assigned_asm in occ_to_asm.items()
            )
            if is_collision_leaf:
                collision_name = name
            else:
                visual_name = name
        
        if not collision_name or not visual_name:
            continue
        
        parent_asm = asm_info.parent_assembly
        if not parent_asm or parent_asm not in assemblies:
            continue
        
        log(f"  FLATTEN: {asm_name} → visual={visual_name}, collision={collision_name} (→ {parent_asm})")
        
        # Find occurrence paths for both leaves
        visual_path = None
        collision_path = None
        for path, assigned_asm in list(occ_to_asm.items()):
            if assigned_asm != asm_name:
                continue
            occ = snapshot.occurrences.get(path)
            if occ and occ.clean_name == visual_name:
                visual_path = path
            elif (occ and occ.clean_name == collision_name
                    and getattr(occ, "is_collision_geometry", False)):
                collision_path = path
        
        if not visual_path or not collision_path:
            log.warning(f"  FLATTEN: could not find paths for {asm_name}")
            continue
        
        # Re-parent visual leaf to grandparent assembly
        occ_to_asm[visual_path] = parent_asm
        if visual_name not in assemblies[parent_asm].links:
            assemblies[parent_asm].links.append(visual_name)
        
        # Remove collision leaf from occ_to_asm (not a URDF link)
        del occ_to_asm[collision_path]
        
        # Store info for later stages:
        # 1. Joint resolution: sub-assembly name → visual leaf name
        model._collision_flatten_redirects[asm_name] = visual_name
        # 2. Mesh export: visual leaf → collision occurrence path
        model._collision_flatten_pairs[visual_name] = collision_path
        
        # Remove sub-assembly from parent's children list
        if asm_name in assemblies[parent_asm].children_assemblies:
            assemblies[parent_asm].children_assemblies.remove(asm_name)
        
        to_remove.append(asm_name)
    
    for name in to_remove:
        del assemblies[name]
        log(f"  FLATTEN: removed assembly '{name}'")


# ──────────────────────────────────────────────
# Step 1b: Rigid group merge — collapse N members into 1 URDF link
# ──────────────────────────────────────────────
#
# Refactored 2026-04-30 from the earlier "shared collision" semantics.  A
# rigid group in Fusion now means "treat as one body" for URDF: pick an
# anchor (explicit frame-only member first, otherwise heaviest
# non-collision member), aggregate mass / CoM / inertia via
# parallel-axis theorem in the anchor's local frame, build a merged AABB
# from physical members, and remove every non-anchor leaf from occ_to_asm
# so downstream stages see only the anchor.  Joints touching any member
# get redirected to the anchor (see _resolve_joint_paths).
# ──────────────────────────────────────────────

import re as _re

# Fusion's auto-assigned name for an unrenamed rigid group: "Rigid Group 7".
# When the user has bothered to rename it (e.g. "pendel_with_esp") we use
# that as the merged URDF link name; otherwise fall back to the anchor's
# clean_name.
_GENERIC_RIGID_GROUP_NAME = _re.compile(r"^\s*Rigid Group\s+\d+\s*$")


def _pick_rigid_group_anchor(
    rg: RigidGroupInfo,
    members: List[Tuple[str, str, FusionOccurrence]],
    log: Logger,
) -> Tuple[str, str, FusionOccurrence]:
    """Choose the occurrence that defines a merged rigid group's frame.

    An explicit ``!frame_*`` member is the user's coordinate-system
    declaration and must win over mass.  If multiple frame members exist,
    prefer the one whose stripped name matches the rigid group name;
    otherwise fall back to a stable name/path order and warn.
    """
    frame_members = [m for m in members if getattr(m[2], "is_frame_only", False)]
    if frame_members:
        group_name = clean_name(rg.name) if rg.name else ""
        matching = [m for m in frame_members if m[1] == group_name]
        candidates = matching or frame_members
        candidates.sort(key=lambda m: (m[1], m[0]))
        if len(frame_members) > 1:
            names = ", ".join(m[1] for m in frame_members)
            log.warning(
                f"  Rigid group '{rg.name}' has multiple frame-only "
                f"members ({names}); using '{candidates[0][1]}' as the "
                f"merged link frame"
            )
        return candidates[0]

    preferred_anchor = getattr(rg, "preferred_anchor_path", "")
    preferred_members = [m for m in members if m[0] == preferred_anchor]
    if preferred_members:
        return preferred_members[0]

    physical_members = [m for m in members if not getattr(m[2], "is_frame_only", False)]
    anchor_pool = physical_members or members
    # Anchor = heaviest non-collision member; tie-break alphabetically.
    anchor_pool.sort(key=lambda m: (-m[2].mass_kg, m[1], m[0]))
    return anchor_pool[0]


def _aggregate_rigid_group(
    rg: RigidGroupInfo, snapshot: FusionSnapshot, log: Logger
) -> Optional[Dict[str, object]]:
    """Pick anchor, aggregate mass/CoM/inertia/AABB for one rigid group.

    Returns ``None`` if the group has no usable visual members (e.g. only
    a ``!collision_*`` member, or every member is missing from the
    snapshot).  All aggregated quantities are expressed in the anchor's
    local frame so the merged URDF link's pose comes from the anchor's
    component origin.
    """
    members = []
    for occ_path, cname in zip(rg.occurrence_paths, rg.member_clean_names):
        if cname == rg.collision_member:
            continue
        occ = snapshot.occurrences.get(occ_path)
        if occ is None:
            log.warning(f"  Rigid group '{rg.name}': member '{cname}' not in snapshot")
            continue
        members.append((occ_path, cname, occ))

    if not members:
        log.warning(f"  Rigid group '{rg.name}': no visual members — skipping merge")
        return None

    anchor_path, anchor_clean, anchor_occ = _pick_rigid_group_anchor(rg, members, log)
    physical_members = [m for m in members if not getattr(m[2], "is_frame_only", False)]
    if not physical_members:
        log.warning(
            f"  Rigid group '{rg.name}': only frame-only members found - "
            f"leaving them as standalone frame links"
        )
        return None

    # Anchor's world rotation and translation define the merged link's
    # frame.  All inertia tensors and CoM positions get rotated/translated
    # into this frame below.  Use ``transform2.translation`` for world
    # position — Fusion composes the assemblyContext chain properly there
    # (rotations applied to nested local translations).  The project's
    # chain-walked ``global_position`` ignores parent rotations and is
    # wrong for any leaf inside a rotated sub-asm.
    r_anchor = _global_rotation_for_occurrence(anchor_path, snapshot)
    r_anchor_inv = _mat3_transpose(r_anchor)
    t_anchor = (
        anchor_occ.transform2.translation
        if anchor_occ.transform2 is not None
        else anchor_occ.global_position
    )

    total_mass = 0.0
    com_weighted = [0.0, 0.0, 0.0]
    members_in_anchor_frame: List[Tuple[float, Vec3, InertiaTensor, Vec3, Vec3]] = []
    bbox_min = [float("inf"), float("inf"), float("inf")]
    bbox_max = [-float("inf"), -float("inf"), -float("inf")]
    collision_bbox_min = [float("inf"), float("inf"), float("inf")]
    collision_bbox_max = [-float("inf"), -float("inf"), -float("inf")]
    total_volume = 0.0
    total_area = 0.0
    collision_volume = 0.0
    collision_body_count = 0
    collision_excluded_body_names: List[str] = []

    for occ_path, cname, occ in physical_members:
        m = occ.mass_kg

        # Member world rotation & translation.  ``transform2.rotation``
        # acts as local-to-world: R_member · v_local = v_world (verified
        # against fusion_transforms.json — applying the BatteryHolder's
        # transform2 directly to a known design corner produces the
        # corner's world position, with no transpose).  ``transform2``
        # values for inner occurrences are pre-composed by Fusion through
        # the assemblyContext chain, so a single read is correct.
        r_member = _global_rotation_for_occurrence(occ_path, snapshot)
        t_member_world = (
            occ.transform2.translation
            if occ.transform2 is not None
            else occ.global_position
        )

        # CoM in world: lift component-local CoM to world via R_member
        # (NOT R_memberᵀ — the transpose was a leftover from the
        # old "world→local" convention assumption that produced
        # wildly inflated AABBs and a comically large auto-fit
        # collision cylinder for the pendel rigid group).
        com_world = _vec_add(
            _rotate_vec3_by_mat3(occ.com_component_local, r_member),
            t_member_world,
        )
        com_anchor = _rotate_vec3_by_mat3(_vec_sub(com_world, t_anchor), r_anchor_inv)

        # Member's component-axes inertia, rotated into anchor axes.
        # Change-of-basis matrix from member-local to anchor-local:
        #   v_anchor = R_anchorᵀ · v_world = R_anchorᵀ · R_member · v_member
        # so M = R_anchorᵀ · R_member.  I_anchor = M · I_member · Mᵀ.
        change_of_basis = _mat3_mul(r_anchor_inv, r_member)
        inertia_anchor = _rotate_inertia_tensor(occ.inertia_at_com, change_of_basis)

        members_in_anchor_frame.append((m, com_anchor, inertia_anchor, com_world, occ.bbox_min))
        total_mass += m
        com_weighted[0] += m * com_anchor[0]
        com_weighted[1] += m * com_anchor[1]
        com_weighted[2] += m * com_anchor[2]

        # AABB: transform each of the 8 corners of the member's bbox into
        # anchor-local and accumulate min/max.  Member bbox is in the
        # MEMBER's component-local frame, lifted to world via R_member
        # (local-to-world), then pulled into anchor-local via R_anchorᵀ.
        for corner in _bbox_corners(occ.bbox_min, occ.bbox_max):
            corner_world = _vec_add(
                _rotate_vec3_by_mat3(corner, r_member),
                t_member_world,
            )
            corner_anchor = _rotate_vec3_by_mat3(
                _vec_sub(corner_world, t_anchor), r_anchor_inv
            )
            for i in range(3):
                if corner_anchor[i] < bbox_min[i]:
                    bbox_min[i] = corner_anchor[i]
                if corner_anchor[i] > bbox_max[i]:
                    bbox_max[i] = corner_anchor[i]

        if getattr(occ, "collision_body_count", 0) > 0:
            for corner in _bbox_corners(
                getattr(occ, "collision_bbox_min", occ.bbox_min),
                getattr(occ, "collision_bbox_max", occ.bbox_max),
            ):
                corner_world = _vec_add(
                    _rotate_vec3_by_mat3(corner, r_member),
                    t_member_world,
                )
                corner_anchor = _rotate_vec3_by_mat3(
                    _vec_sub(corner_world, t_anchor), r_anchor_inv
                )
                for i in range(3):
                    if corner_anchor[i] < collision_bbox_min[i]:
                        collision_bbox_min[i] = corner_anchor[i]
                    if corner_anchor[i] > collision_bbox_max[i]:
                        collision_bbox_max[i] = corner_anchor[i]
            collision_volume += getattr(occ, "collision_volume_m3", occ.volume_m3)
            collision_body_count += getattr(occ, "collision_body_count", 0)

        for body_name in getattr(occ, "collision_excluded_body_names", []) or []:
            collision_excluded_body_names.append(f"{cname}/{body_name}")

        total_volume += occ.volume_m3
        total_area += occ.area_m2

    if total_mass <= 0.0:
        # Defensive: aggregate CoM is undefined if no mass.  Place at
        # anchor origin and skip parallel-axis shifts (everything stays
        # zero, which matches an empty/reference frame link's behavior).
        com_total = (0.0, 0.0, 0.0)
    else:
        com_total = (
            com_weighted[0] / total_mass,
            com_weighted[1] / total_mass,
            com_weighted[2] / total_mass,
        )

    # Aggregate inertia at the COMBINED CoM, in anchor axes, via the
    # parallel-axis theorem on each member.
    inertia_total = InertiaTensor()
    for m, com_anchor, inertia_anchor, _, _ in members_in_anchor_frame:
        d = _vec_sub(com_anchor, com_total)
        # Member inertia translates from member CoM to combined CoM.
        # Both points are now in anchor frame, so this is a pure shift.
        shift_xx = m * (d[1] * d[1] + d[2] * d[2])
        shift_yy = m * (d[0] * d[0] + d[2] * d[2])
        shift_zz = m * (d[0] * d[0] + d[1] * d[1])
        shift_xy = -m * d[0] * d[1]
        shift_xz = -m * d[0] * d[2]
        shift_yz = -m * d[1] * d[2]

        inertia_total.ixx += inertia_anchor.ixx + shift_xx
        inertia_total.iyy += inertia_anchor.iyy + shift_yy
        inertia_total.izz += inertia_anchor.izz + shift_zz
        inertia_total.ixy += inertia_anchor.ixy + shift_xy
        inertia_total.ixz += inertia_anchor.ixz + shift_xz
        inertia_total.iyz += inertia_anchor.iyz + shift_yz

    # Merged link name: rigid group name (cleaned) when user-renamed,
    # else fall back to anchor's clean_name.
    if rg.name and not _GENERIC_RIGID_GROUP_NAME.match(rg.name):
        merged_link_name = clean_name(rg.name)
    else:
        merged_link_name = anchor_clean

    # Priority-1 collision offset: where the explicit !collision_* member
    # sits relative to the anchor frame.  Use transform2.translation for
    # the same reason as t_anchor above.
    collision_offset = None
    if rg.collision_path:
        coll_occ = snapshot.occurrences.get(rg.collision_path)
        if coll_occ is not None:
            coll_world = (
                coll_occ.transform2.translation
                if coll_occ.transform2 is not None
                else coll_occ.global_position
            )
            collision_offset = _rotate_vec3_by_mat3(
                _vec_sub(coll_world, t_anchor), r_anchor_inv
            )

    bbox_min_t = (bbox_min[0], bbox_min[1], bbox_min[2])
    bbox_max_t = (bbox_max[0], bbox_max[1], bbox_max[2])
    bbox_size = (
        bbox_max[0] - bbox_min[0],
        bbox_max[1] - bbox_min[1],
        bbox_max[2] - bbox_min[2],
    )
    if collision_bbox_min[0] < float("inf"):
        collision_bbox_min_t = (
            collision_bbox_min[0], collision_bbox_min[1], collision_bbox_min[2],
        )
        collision_bbox_max_t = (
            collision_bbox_max[0], collision_bbox_max[1], collision_bbox_max[2],
        )
        collision_bbox_size = (
            collision_bbox_max[0] - collision_bbox_min[0],
            collision_bbox_max[1] - collision_bbox_min[1],
            collision_bbox_max[2] - collision_bbox_min[2],
        )
    else:
        collision_bbox_min_t = (0.0, 0.0, 0.0)
        collision_bbox_max_t = (0.0, 0.0, 0.0)
        collision_bbox_size = (0.0, 0.0, 0.0)

    log(
        f"  {rg.name}: anchor={anchor_clean} merged_name={merged_link_name} "
        f"members={len(members)} mass={total_mass*1000:.2f} g "
        f"bbox=({bbox_size[0]*1000:.1f} × {bbox_size[1]*1000:.1f} × "
        f"{bbox_size[2]*1000:.1f}) mm"
    )

    return {
        "anchor_path": anchor_path,
        "anchor_clean_name": anchor_clean,
        "merged_link_name": merged_link_name,
        "mass": total_mass,
        "com": com_total,
        "inertia": inertia_total,
        "bbox_min": bbox_min_t,
        "bbox_max": bbox_max_t,
        "bbox_size": bbox_size,
        "collision_bbox_min": collision_bbox_min_t,
        "collision_bbox_max": collision_bbox_max_t,
        "collision_bbox_size": collision_bbox_size,
        "collision_volume_m3": collision_volume,
        "collision_body_count": collision_body_count,
        "collision_excluded_body_names": collision_excluded_body_names,
        "volume_m3": total_volume,
        "area_m2": total_area,
        "member_paths": [m[0] for m in members],
        "member_clean_names": [m[1] for m in members],
        "physical_member_paths": [m[0] for m in physical_members],
        "physical_member_clean_names": [m[1] for m in physical_members],
        "collision_path": rg.collision_path,
        "collision_offset": collision_offset,
        "collision_member": rg.collision_member,
        "rigid_group_name": rg.name,
        "wants_accurate_collision": getattr(rg, "wants_accurate_collision", False),
        "collision_override": getattr(rg, "collision_override", ""),
    }


def _process_rigid_groups(
    snapshot: FusionSnapshot,
    occ_to_asm: Dict[str, str],
    model: RobotModel,
    log: Logger,
):
    """Collapse each rigid group into a single anchor link.

    Side effects on ``model``:
      - ``_rigid_group_anchor_data``: anchor_path → aggregated dict
        (consumed by ``_build_links``).
      - ``_rigid_group_member_to_anchor``: occurrence path →
        (anchor_path, assembly, anchor_clean) (consumed by
        ``_resolve_joint_paths`` to redirect joint endpoints from
        collapsed members to the anchor).
      - ``_rigid_group_anchor_merged_name``: anchor_path → merged URDF
        link name (consumed by ``_resolve_names`` to honor the rigid
        group name in place of the anchor's clean_name).

    Side effects on ``occ_to_asm``: removes every non-anchor member and
    every ``!collision_*`` member.  After this function returns, the
    occurrence-to-assembly map only contains free link candidates and
    merged anchors.
    """
    model._rigid_group_anchor_data = {}
    model._rigid_group_member_to_anchor = {}
    model._rigid_group_anchor_merged_name = {}

    if not snapshot.rigid_groups:
        log(f"  No explicit rigid groups to merge")

    for rg in snapshot.rigid_groups:
        agg = _aggregate_rigid_group(rg, snapshot, log)
        if agg is None:
            continue
        _register_rigid_group_merge(rg, agg, occ_to_asm, model, log)

    _process_auto_rigid_islands(snapshot, occ_to_asm, model, log)

    _redirect_synthetic_root_to_base_rigid_group(
        snapshot, occ_to_asm, model, log
    )

    if not model._rigid_group_anchor_data:
        log(f"  No rigid groups or auto rigid islands produced merged links")


def _register_rigid_group_merge(
    rg: RigidGroupInfo,
    agg: Dict[str, object],
    occ_to_asm: Dict[str, str],
    model: RobotModel,
    log: Logger,
) -> bool:
    """Register an explicit or synthetic rigid-group aggregate."""
    anchor_path = agg["anchor_path"]
    anchor_clean = agg["anchor_clean_name"]
    merged_name = agg["merged_link_name"]

    # Persist on the RigidGroupInfo too so phase-1 fixtures round-trip.
    rg.anchor_path = anchor_path
    rg.anchor_clean_name = anchor_clean
    rg.merged_link_name = merged_name

    anchor_asm = occ_to_asm.get(anchor_path)
    if anchor_asm is None:
        log.warning(
            f"  Rigid group '{rg.name}': anchor '{anchor_clean}' not in "
            f"occ_to_asm - skipping merge"
        )
        return False

    # Drop every non-anchor member from occ_to_asm.  Track redirects by
    # full occurrence path, not (assembly, clean_name), so reused
    # components in separate groups stay disambiguated.
    for occ_path, cname in zip(rg.occurrence_paths, rg.member_clean_names):
        if occ_path == anchor_path:
            continue
        member_asm = occ_to_asm.pop(occ_path, None)
        if member_asm is None:
            continue
        model._rigid_group_member_to_anchor[occ_path] = (
            anchor_path,
            anchor_asm,
            anchor_clean,
        )
        log(f"  {rg.name}: dropped {member_asm}/{cname} ({occ_path}) -> "
            f"{anchor_asm}/{anchor_clean}")

    # The !collision_* member is also dropped from URDF links.  Downstream
    # uses it as an STL source, not as a separate link.
    if rg.collision_path and rg.collision_path in occ_to_asm:
        del occ_to_asm[rg.collision_path]
        log(f"  {rg.name}: removed collision member '{rg.collision_member}' from links")

    model._rigid_group_anchor_data[anchor_path] = agg
    model._rigid_group_anchor_merged_name[anchor_path] = merged_name
    return True


def _process_auto_rigid_islands(
    snapshot: FusionSnapshot,
    occ_to_asm: Dict[str, str],
    model: RobotModel,
    log: Logger,
) -> None:
    """Collapse no-internal-joint subassemblies into implicit links."""
    blocked_paths = set(getattr(model, '_rigid_group_anchor_data', {}).keys())
    blocked_paths.update(getattr(model, '_rigid_group_member_to_anchor', {}).keys())

    selected_roots: List[str] = []
    collapsed = 0

    subassemblies = sorted(
        (
            (path, occ) for path, occ in snapshot.occurrences.items()
            if occ.is_subassembly and path != DESIGN_ROOT_OCCURRENCE_PATH
        ),
        key=lambda item: (item[1].depth, item[0]),
    )

    for asm_path, asm_occ in subassemblies:
        if any(_path_is_descendant_or_self(asm_path, root) for root in selected_roots):
            continue
        if _auto_candidate_overlaps_existing_merge(asm_path, blocked_paths):
            continue
        member_paths = _auto_rigid_member_paths(asm_path, snapshot, occ_to_asm)
        if len(member_paths) <= 1:
            continue
        if _auto_candidate_has_internal_joint(asm_path, snapshot):
            continue
        # A top-level wrapper with design-root fallback joints usually
        # represents the imported robot model, not a physical link.  Skip
        # it so wheels/legs do not get swallowed into the base.
        if asm_occ.depth == 0 and _auto_candidate_has_design_root_child_joint(
            asm_path, snapshot
        ):
            continue

        rg = RigidGroupInfo(
            name=asm_occ.clean_name,
            occurrence_paths=member_paths,
            member_clean_names=[
                snapshot.occurrences[p].clean_name for p in member_paths
            ],
            preferred_anchor_path=(
                asm_path if asm_path in member_paths and asm_occ.body_count > 0 else ""
            ),
            is_auto=True,
            wants_accurate_collision=(getattr(asm_occ, "collision_override", "") == "visual"),
            collision_override=getattr(asm_occ, "collision_override", ""),
        )
        agg = _aggregate_rigid_group(rg, snapshot, log)
        if agg is None:
            continue
        if not _register_rigid_group_merge(rg, agg, occ_to_asm, model, log):
            continue

        # If the subassembly itself is a container-only occurrence, joints
        # may still target that occurrence.  Redirect those endpoints to
        # the implicit merged link anchor.
        if asm_path != rg.anchor_path:
            anchor_asm = occ_to_asm.get(rg.anchor_path)
            if anchor_asm:
                model._rigid_group_member_to_anchor[asm_path] = (
                    rg.anchor_path,
                    anchor_asm,
                    rg.anchor_clean_name,
                )

        selected_roots.append(asm_path)
        blocked_paths.add(rg.anchor_path)
        blocked_paths.update(rg.occurrence_paths)
        collapsed += 1
        log(f"  AUTO rigid island: {asm_occ.clean_name} "
            f"({len(member_paths)} members) -> {rg.merged_link_name}")

    if collapsed == 0:
        log(f"  No auto rigid islands found")


def _auto_rigid_member_paths(
    asm_path: str,
    snapshot: FusionSnapshot,
    occ_to_asm: Dict[str, str],
) -> List[str]:
    """Physical descendants still visible as link candidates."""
    out = []
    for path in sorted(occ_to_asm):
        if path == DESIGN_ROOT_OCCURRENCE_PATH:
            continue
        occ = snapshot.occurrences.get(path)
        if not occ or not _occurrence_is_link_candidate(occ):
            continue
        if getattr(occ, "is_collision_geometry", False):
            continue
        if _path_is_descendant_or_self(path, asm_path):
            out.append(path)
    return out


def _path_is_descendant_or_self(path: str, ancestor_path: str) -> bool:
    return path == ancestor_path or path.startswith(ancestor_path + '+')


def _auto_candidate_overlaps_existing_merge(
    asm_path: str, blocked_paths: Set[str],
) -> bool:
    for blocked in blocked_paths:
        if (
            _path_is_descendant_or_self(blocked, asm_path)
            or _path_is_descendant_or_self(asm_path, blocked)
        ):
            return True
    return False


def _auto_candidate_has_internal_joint(
    asm_path: str, snapshot: FusionSnapshot,
) -> bool:
    for fj in snapshot.joints.values():
        one_paths = _auto_joint_endpoint_paths(
            fj.occurrence_one_path, fj.occurrence_one_clean, snapshot
        )
        two_paths = _auto_joint_endpoint_paths(
            fj.occurrence_two_path, fj.occurrence_two_clean, snapshot
        )
        one_inside = [p for p in one_paths if _path_is_descendant_or_self(p, asm_path)]
        two_inside = [p for p in two_paths if _path_is_descendant_or_self(p, asm_path)]
        if any(a != b for a in one_inside for b in two_inside):
            return True
    return False


def _auto_candidate_has_design_root_child_joint(
    asm_path: str, snapshot: FusionSnapshot,
) -> bool:
    for fj in snapshot.joints.values():
        one_paths = _auto_joint_endpoint_paths(
            fj.occurrence_one_path, fj.occurrence_one_clean, snapshot
        )
        two_paths = _auto_joint_endpoint_paths(
            fj.occurrence_two_path, fj.occurrence_two_clean, snapshot
        )
        one_root = DESIGN_ROOT_OCCURRENCE_PATH in one_paths
        two_root = DESIGN_ROOT_OCCURRENCE_PATH in two_paths
        one_inside = any(_path_is_descendant_or_self(p, asm_path) for p in one_paths)
        two_inside = any(_path_is_descendant_or_self(p, asm_path) for p in two_paths)
        if (one_root and two_inside) or (two_root and one_inside):
            return True
    return False


def _auto_joint_endpoint_paths(
    local_path: str,
    clean: str,
    snapshot: FusionSnapshot,
) -> List[str]:
    """Best-effort endpoint path match before full joint resolution."""
    if local_path == DESIGN_ROOT_OCCURRENCE_PATH:
        return [DESIGN_ROOT_OCCURRENCE_PATH]

    matches = []
    for full_path, occ in snapshot.occurrences.items():
        if full_path == DESIGN_ROOT_OCCURRENCE_PATH:
            continue
        if clean and occ.clean_name != clean:
            continue
        if local_path and full_path.endswith(local_path):
            matches.append(full_path)

    if not matches and clean:
        matches = [
            full_path for full_path, occ in snapshot.occurrences.items()
            if full_path != DESIGN_ROOT_OCCURRENCE_PATH and occ.clean_name == clean
        ]
    return matches


def _redirect_synthetic_root_to_base_rigid_group(
    snapshot: FusionSnapshot,
    occ_to_asm: Dict[str, str],
    model: RobotModel,
    log: Logger,
) -> None:
    """Route design-root fallback joint endpoints to a base_link group.

    Some Fusion files expose wheel/leg joints whose occurrenceTwo cannot
    be read, so phase 1 records a synthetic design-root endpoint.  If the
    user also made a rigid group named base_link, that group is the real
    physical root and should receive those joints instead of producing an
    empty duplicate base_link.
    """
    root_path = DESIGN_ROOT_OCCURRENCE_PATH
    root_occ = snapshot.occurrences.get(root_path)
    if root_path not in occ_to_asm or root_occ is None:
        return
    if root_occ.body_count > 0:
        return

    candidates = []
    anchor_data = getattr(model, '_rigid_group_anchor_data', {})
    anchor_names = getattr(model, '_rigid_group_anchor_merged_name', {})
    for anchor_path, agg in anchor_data.items():
        merged_name = anchor_names.get(anchor_path, agg.get("merged_link_name", ""))
        if merged_name != "base_link":
            continue
        anchor_asm = occ_to_asm.get(anchor_path)
        if anchor_asm is None:
            continue
        candidates.append((
            anchor_path,
            anchor_asm,
            agg.get("anchor_clean_name", ""),
            agg.get("rigid_group_name", merged_name),
        ))

    if len(candidates) != 1:
        return

    anchor_path, anchor_asm, anchor_clean, group_name = candidates[0]
    occ_to_asm.pop(root_path, None)
    model._rigid_group_member_to_anchor[root_path] = (
        anchor_path,
        anchor_asm,
        anchor_clean,
    )
    log(
        f"  Synthetic design-root endpoint redirected to rigid group "
        f"'{group_name}' ({anchor_asm}/{anchor_clean})"
    )


def _bbox_corners(bbox_min: Vec3, bbox_max: Vec3) -> List[Vec3]:
    """Return the eight corner points of an axis-aligned bbox."""
    return [
        (bbox_min[0], bbox_min[1], bbox_min[2]),
        (bbox_max[0], bbox_min[1], bbox_min[2]),
        (bbox_min[0], bbox_max[1], bbox_min[2]),
        (bbox_max[0], bbox_max[1], bbox_min[2]),
        (bbox_min[0], bbox_min[1], bbox_max[2]),
        (bbox_max[0], bbox_min[1], bbox_max[2]),
        (bbox_min[0], bbox_max[1], bbox_max[2]),
        (bbox_max[0], bbox_max[1], bbox_max[2]),
    ]


def _rotate_inertia_tensor(I: InertiaTensor, R: Tuple[float, ...]) -> InertiaTensor:
    """Compute R · I · Rᵀ for a symmetric inertia tensor under a rotation R.

    R is row-major 3×3.  The result is a new ``InertiaTensor`` whose
    six components are the upper triangle of the rotated tensor.
    """
    # Build full I as flat 9-tuple (ixx ixy ixz ixy iyy iyz ixz iyz izz).
    Im = (
        I.ixx, I.ixy, I.ixz,
        I.ixy, I.iyy, I.iyz,
        I.ixz, I.iyz, I.izz,
    )
    RI = _mat3_mul(R, Im)
    RIRt = _mat3_mul(RI, _mat3_transpose(R))
    return InertiaTensor(
        ixx=RIRt[0], ixy=RIRt[1], ixz=RIRt[2],
        iyy=RIRt[4], iyz=RIRt[5], izz=RIRt[8],
    )


# ──────────────────────────────────────────────
# Step 2: Resolve joint occurrence paths
# ──────────────────────────────────────────────

def _resolve_joint_paths(
    snapshot: FusionSnapshot,
    occ_to_asm: Dict[str, str],
    model: RobotModel,
    log: Logger,
) -> List[KinematicEdge]:
    """
    Map joint occurrence paths (often local to defining component) to
    resolved (assembly, component) pairs.
    
    Fusion joints store occurrenceOne/Two paths that are LOCAL to the
    defining component. We need to resolve them to global paths.
    """
    edges = []
    redirects = getattr(model, '_collision_flatten_redirects', {})
    member_to_anchor = getattr(model, '_rigid_group_member_to_anchor', {})

    for jname, fj in snapshot.joints.items():
        defining = fj.defining_component

        # Resolve parent (occurrenceTwo)
        p_asm, p_comp, p_path = _resolve_occ_path(
            fj.occurrence_two_path, fj.occurrence_two_clean,
            defining, snapshot, occ_to_asm, redirects, member_to_anchor,
        )

        # Resolve child (occurrenceOne)
        c_asm, c_comp, c_path = _resolve_occ_path(
            fj.occurrence_one_path, fj.occurrence_one_clean,
            defining, snapshot, occ_to_asm, redirects, member_to_anchor,
        )

        # Drop joints internal to a rigid group.  Use full_path
        # comparison — (asm, comp) collides for re-used components, so
        # an "internal" check on (asm, comp) alone false-positives
        # joints between two distinct occurrences of the same component.
        same_anchor = (
            p_path and c_path and p_path == c_path
        ) or (
            not (p_path and c_path) and (p_asm, p_comp) == (c_asm, c_comp)
        )
        if same_anchor:
            if fj.motion_type == "rigid":
                log(
                    f"  {jname}: internal rigid joint inside rigid group "
                    f"{p_asm}/{p_comp}; dropped as redundant"
                )
            else:
                msg = (
                    f"Joint '{jname}' is internal to a rigid group "
                    f"(both endpoints → {p_asm}/{p_comp}).  Dropped — rigid "
                    f"groups represent one rigid body, joints between members "
                    f"are ignored.  Move this joint to assemble between rigid "
                    f"groups instead."
                )
                model.warnings.append(msg)
                log.warning(f"  {jname}: {msg}")
            continue

        is_cross = p_asm != c_asm

        edge = KinematicEdge(
            joint_name=jname,
            parent_asm=p_asm,
            parent_comp=p_comp,
            parent_path=p_path,
            child_asm=c_asm,
            child_comp=c_comp,
            child_path=c_path,
            is_cross_assembly=is_cross,
            motion_type=fj.motion_type,
            defining_component=defining,
            fusion_joint=fj,
            # User-tagged closing joint (Fusion joint name had
            # ``!closing_*`` prefix - recognised at extraction time).
            # ``_classify_closing_joints`` runs after this loop and
            # may additionally flag edges as ``auto_detected`` when
            # a multi-parent child has no explicit closing tag.
            is_closing=getattr(fj, "is_closing", False),
            closing_source="user_tag" if getattr(fj, "is_closing", False) else "",
        )
        edges.append(edge)

        tag = "MOUNT" if is_cross else "internal"
        if edge.is_closing:
            tag += " CLOSING(user_tag)"
        log(f"  {jname:<20} {p_asm}/{p_comp} → {c_asm}/{c_comp}  [{fj.motion_type}] {tag}")

    # Auto-detect closing joints for any multi-parent link that has
    # no explicit ``!closing_*`` tag.  Keeps the first edge in topo
    # order from root, routes the rest to ``is_closing=True`` with
    # ``source="auto_detected"`` and a warning suggesting the user
    # tag explicitly.
    _classify_closing_joints(edges, model, log)

    return edges


def _classify_closing_joints(
    edges: List[KinematicEdge], model: RobotModel, log: Logger,
) -> None:
    """Detect multi-parent children among the URDF-bound edges and
    classify the "extra" parents as closing.

    Algorithm:
      1. Group non-closing edges by child_path.  Children with more
         than one such parent are multi-parent.
      2. Determine a topological "primary" parent for each multi-parent
         child.  Without a clear root yet (we run before _detect_root),
         we use a stable heuristic: the edge whose joint_name comes
         first alphabetically among the candidates.  In practice the
         user usually tags exactly one of the joints with
         ``!closing_*`` and we never enter this fallback; if all are
         untagged the alphabetical pick is at least deterministic.
      3. Mark all OTHER candidates as ``is_closing=True``,
         ``closing_source="auto_detected"``, and emit a warning per
         child suggesting the user add an explicit ``!closing_*`` tag.
    """
    # Skip edges already user-tagged as closing — those are settled.
    by_child: Dict[str, List[KinematicEdge]] = {}
    for e in edges:
        if e.is_closing:
            continue
        # Use full_path when available for disambiguation; fall back
        # to (asm, comp) for older snapshot paths.
        key = e.child_path or f"{e.child_asm}/{e.child_comp}"
        by_child.setdefault(key, []).append(e)

    for key, parents in by_child.items():
        if len(parents) <= 1:
            continue
        parents_sorted = sorted(parents, key=lambda e: e.joint_name)
        keep = parents_sorted[0]
        drop = parents_sorted[1:]
        names_drop = [e.joint_name for e in drop]
        msg = (
            f"Multi-parent link detected (child={key}): keeping "
            f"'{keep.joint_name}' as the URDF tree parent; routing "
            f"{names_drop} to ``closing_joints`` (sidecar) — closed "
            f"kinematic loop.  Tag the loop-closing joint(s) with "
            f"prefix ``!closing_*`` in Fusion to make the choice "
            f"explicit and avoid this warning."
        )
        model.warnings.append(msg)
        log.warning(f"  CLOSING(auto): {msg}")
        for e in drop:
            e.is_closing = True
            e.closing_source = "auto_detected"


def _drop_unreferenced_empty_occurrences(
    snapshot: FusionSnapshot,
    occ_to_asm: Dict[str, str],
    edges: List[KinematicEdge],
    model: RobotModel,
    log: Logger,
) -> None:
    """Remove empty CAD reference occurrences that no joint uses.

    Downloaded/imported Fusion files often contain zero-body helper
    components.  If no joint references them, they are not robot links; keeping
    them creates disconnected empty URDF frames and unnecessary validation
    noise.  Connected empty links are preserved because they may be intentional
    frames.
    """
    referenced_paths = {
        p for e in edges for p in (e.parent_path, e.child_path) if p
    }
    dropped = []
    for path in list(occ_to_asm.keys()):
        if path in referenced_paths:
            continue
        occ = snapshot.occurrences.get(path)
        if not occ or occ.is_subassembly or occ.body_count != 0:
            continue
        dropped.append(occ.clean_name or path)
        del occ_to_asm[path]

    if dropped:
        members = ", ".join(sorted(dropped))
        msg = (
            f"Dropped unreferenced empty occurrence(s): {members}. "
            f"These look like imported CAD reference components, not robot links."
        )
        model.warnings.append(msg)
        log.warning(msg)


def _resolve_occ_path(
    local_path: str,
    clean: str,
    defining_component: str,
    snapshot: FusionSnapshot,
    occ_to_asm: Dict[str, str],
    redirects: Optional[Dict[str, str]] = None,
    member_to_anchor: Optional[Dict[str, Tuple[str, str, str]]] = None,
) -> Tuple[str, str, str]:
    """
    Resolve a joint's local occurrence path to (assembly, component, full_path).

    Returning ``full_path`` is the key disambiguator: when a component
    is reused (e.g. ``leva:1`` and ``leva:2``), every (asm, clean_name)
    lookup collides and one of the occurrences is silently lost.  The
    full_path is unique per occurrence; downstream lookups use it as
    the canonical key.

    Logic:
      1. Check if clean name is a flattened collision sub-assembly → redirect
      2. Find candidates in ``occ_to_asm`` matching path suffix + clean
         name; prefer match in defining assembly.
      3. If nothing found in occ_to_asm, the target may have been
         collapsed into a rigid-group anchor.  Find the leaf's
         full_path in ``snapshot.occurrences``, look up the redirect
         by full_path, and return the anchor's (asm, clean, full_path).
      4. Fallback: suffix-only match in occ_to_asm.
      5. Last resort: defining component as assembly, full_path empty.
    """
    # Check redirect: joint targets a flattened collision sub-assembly
    if redirects and clean in redirects:
        visual_name = redirects[clean]
        for full_path, asm in occ_to_asm.items():
            occ = snapshot.occurrences.get(full_path)
            if occ and occ.clean_name == visual_name:
                return asm, visual_name, full_path

    candidates = []

    for full_path, asm in occ_to_asm.items():
        occ = snapshot.occurrences.get(full_path)
        if not occ:
            continue
        if occ.clean_name == clean and full_path.endswith(local_path):
            candidates.append((asm, clean, full_path))

    if candidates:
        # Prefer match in the defining assembly
        for asm, comp, fp in candidates:
            if asm == defining_component:
                return asm, comp, fp
        return candidates[0]

    # Member may have been collapsed into a rigid-group anchor —
    # snapshot still has it, but occ_to_asm dropped it.  Look up the
    # redirect by full_path so we get the right anchor when the same
    # clean_name appears in multiple rigid groups.
    if member_to_anchor:
        for full_path, occ in snapshot.occurrences.items():
            if occ.clean_name != clean or not full_path.endswith(local_path):
                continue
            redirected = member_to_anchor.get(full_path)
            if redirected is not None:
                anchor_path, anchor_asm, anchor_clean = redirected
                return anchor_asm, anchor_clean, anchor_path

    # No match by name+suffix — try suffix only
    for full_path, asm in occ_to_asm.items():
        if full_path.endswith(local_path):
            occ = snapshot.occurrences.get(full_path)
            if occ:
                return asm, occ.clean_name, full_path

    # Last resort: defining component as assembly
    return defining_component, clean, ""


# ──────────────────────────────────────────────
# Step 3: Detect root link
# ──────────────────────────────────────────────

def _detect_root(
    edges: List[KinematicEdge],
    model: RobotModel,
    log: Logger,
) -> Tuple[str, str]:
    """
    Find root link: appears as parent but never as child, with most descendants.
    
    Returns:
        (assembly_name, component_name) of root
    """
    # Collect all parent and child identities
    all_parents: Set[Tuple[str, str]] = set()
    all_children: Set[Tuple[str, str]] = set()
    
    for e in edges:
        all_parents.add((e.parent_asm, e.parent_comp))
        all_children.add((e.child_asm, e.child_comp))
    
    # Root candidates: parent but never child
    roots = all_parents - all_children
    
    log(f"  Parent-only nodes: {len(roots)}")
    for r in roots:
        log(f"    {r[0]}/{r[1]}")
    
    if len(roots) == 0:
        model.errors.append("No root found — kinematic chain has a cycle")
        # Fallback: pick the node with most outgoing edges
        from collections import Counter
        parent_counts = Counter((e.parent_asm, e.parent_comp) for e in edges)
        root = parent_counts.most_common(1)[0][0]
        log.warning(f"Cycle detected! Using most-connected node as root: {root[0]}/{root[1]}")
        return root
    
    if len(roots) == 1:
        root = roots.pop()
        log(f"  → Root: {root[0]}/{root[1]}")
        return root
    
    # Multiple roots — pick the one with most descendants
    def count_descendants(asm, comp):
        count = 0
        frontier = [(asm, comp)]
        visited = set()
        while frontier:
            a, c = frontier.pop()
            if (a, c) in visited:
                continue
            visited.add((a, c))
            for e in edges:
                if e.parent_asm == a and e.parent_comp == c:
                    count += 1
                    frontier.append((e.child_asm, e.child_comp))
        return count
    
    root = max(roots, key=lambda r: count_descendants(r[0], r[1]))
    
    # Warn about disconnected roots
    for r in roots:
        if r != root:
            model.warnings.append(
                f"Disconnected root: {r[0]}/{r[1]} has no parent joint. "
                f"Check joint direction in Fusion — this link may need "
                f"to be a child instead of a parent."
            )
            log.warning(f"Disconnected root: {r[0]}/{r[1]} — possible flipped joint")
    
    log(f"  → Root (by descendant count): {root[0]}/{root[1]}")
    return root


# ──────────────────────────────────────────────
# Step 4: Resolve name collisions
# ──────────────────────────────────────────────

def _resolve_names(
    snapshot: FusionSnapshot,
    occ_to_asm: Dict[str, str],
    root_asm: str,
    root_comp: str,
    model: RobotModel,
    log: Logger,
) -> Dict[str, str]:
    """
    Build mapping: occurrence full_path → URDF link name.
    
    Rules:
      - clean_name is used directly if unique
      - On collision: prefix with immediate parent assembly name
      - Root link's base_link keeps "base_link" (REP 120)
    """
    # Anchor occurrences use the rigid-group's merged name, not the
    # anchor component's own clean_name.  This ensures collision
    # detection in name_groups operates on the final URDF names.
    anchor_merged = getattr(model, '_rigid_group_anchor_merged_name', {})

    def _effective_name(path: str, occ) -> str:
        return anchor_merged.get(path, occ.clean_name)

    # Collect all (assembly, effective_name) → [paths]
    name_groups: Dict[str, List[str]] = {}
    for path, occ in snapshot.occurrences.items():
        if not _occurrence_is_link_candidate(occ):
            continue
        # Skip collision leaves and rigid-group members removed earlier
        if path not in occ_to_asm:
            continue
        name_groups.setdefault(_effective_name(path, occ), []).append(path)

    # Find collisions
    colliding_names = {name for name, paths in name_groups.items() if len(paths) > 1}

    if colliding_names:
        log(f"  Name collisions: {colliding_names}")

    # Build name map
    name_map: Dict[str, str] = {}  # occurrence path → URDF name
    # Track URDF names already taken so we can disambiguate further
    # collisions deterministically.  ``leva:1`` and ``leva:2`` placed
    # in the same assembly both want ``ROOT_leva``; the second one
    # gets ``ROOT_leva_2``, the third ``ROOT_leva_3``, etc.  Path-order
    # iteration in dict + sorting paths inside a collision group makes
    # the suffix assignment stable across runs.
    used_names: Set[str] = set()

    def _uniquify(candidate: str) -> str:
        if candidate not in used_names:
            return candidate
        i = 2
        while f"{candidate}_{i}" in used_names:
            i += 1
        return f"{candidate}_{i}"

    # Sort the iteration so collision-suffixing is deterministic — a
    # plain dict ordering is insertion-order, but for snapshots round-
    # tripped through JSON / pickle the order can shift.  Sort by
    # full_path (the unique key) for stable URDF names.
    paths_sorted = sorted(
        (p for p in snapshot.occurrences if _occurrence_is_link_candidate(snapshot.occurrences[p])
            and p in occ_to_asm),
        key=lambda p: snapshot.occurrences[p].full_path or p,
    )

    for path in paths_sorted:
        occ = snapshot.occurrences[path]
        asm = occ_to_asm.get(path, "ROOT")
        eff_name = _effective_name(path, occ)

        # Root link always becomes "base_link" (REP 120)
        if asm == root_asm and occ.clean_name == root_comp:
            urdf_name = "base_link"
            if eff_name != "base_link":
                model.warnings.append(
                    f"Root link renamed: '{eff_name}' → 'base_link' (REP 120 convention). "
                    f"Consider renaming the component to 'base_link' in Fusion."
                )
                log.warning(f"Root link renamed: '{eff_name}' → 'base_link'")
        elif eff_name in colliding_names:
            urdf_name = _uniquify(f"{asm}_{eff_name}")
        else:
            urdf_name = _uniquify(eff_name)

        used_names.add(urdf_name)
        name_map[path] = urdf_name
        log(f"  {eff_name} ({asm}) → {urdf_name}")

    # Set root
    for path, urdf_name in name_map.items():
        occ = snapshot.occurrences[path]
        asm = occ_to_asm.get(path, "ROOT")
        if asm == root_asm and occ.clean_name == root_comp:
            model.root_link = urdf_name
            break

    log(f"  Root link URDF name: {model.root_link}")

    return name_map


# ──────────────────────────────────────────────
# Step 5: Build links
# ──────────────────────────────────────────────

def _build_links(
    snapshot: FusionSnapshot,
    occ_to_asm: Dict[str, str],
    name_map: Dict[str, str],
    model: RobotModel,
    log: Logger,
):
    """Create LinkNode for each physical occurrence or merged-group anchor."""

    # Collision flatten pairs: visual_clean_name → collision_occ_path
    flatten_pairs = getattr(model, '_collision_flatten_pairs', {})

    # Rigid-group anchor data: anchor_path → aggregated dict
    anchor_data = getattr(model, '_rigid_group_anchor_data', {})

    for path, occ in snapshot.occurrences.items():
        if not _occurrence_is_link_candidate(occ):
            continue

        # Skip collision leaves that were flattened or rigid-group members
        # that were collapsed into an anchor.
        if path not in occ_to_asm:
            continue

        urdf_name = name_map.get(path)
        if not urdf_name:
            continue

        asm = occ_to_asm.get(path, "ROOT")

        # Is this the anchor of a merged rigid group?  If so, override
        # mass / CoM / inertia / bbox with aggregated values; mesh paths
        # use the merged link name; the link is flagged is_merged so
        # mesh export can pick the right code path.
        merged = anchor_data.get(path)
        if merged is not None:
            inertia = merged["inertia"]
            link = LinkNode(
                urdf_name=urdf_name,
                clean_name=occ.clean_name,
                assembly=asm,
                occurrence_path=path,
                global_position=occ.global_position,
                mass_kg=merged["mass"],
                com_link_local=merged["com"],
                inertia_at_com=InertiaTensor(**inertia.as_dict()),
                material_name=occ.material_name,
                color_rgb=occ.appearance_color_rgb,
                bbox_size=merged["bbox_size"],
                bbox_min=merged["bbox_min"],
                bbox_max=merged["bbox_max"],
                volume_m3=merged["volume_m3"],
                area_m2=merged["area_m2"],
                collision_bbox_size=merged.get("collision_bbox_size", (0.0, 0.0, 0.0)),
                collision_bbox_min=merged.get("collision_bbox_min", (0.0, 0.0, 0.0)),
                collision_bbox_max=merged.get("collision_bbox_max", (0.0, 0.0, 0.0)),
                collision_volume_m3=merged.get("collision_volume_m3", 0.0),
                mesh_visual=f"meshes/{asm}/{urdf_name}.obj",
                mesh_collision=f"meshes/{asm}/{urdf_name}_collision.stl",
                body_count=occ.body_count,
                is_empty=(merged["mass"] <= 0.0),
                is_merged=True,
                merged_member_paths=list(merged["member_paths"]),
                rigid_group_name=merged["rigid_group_name"],
                collision_override=merged.get("collision_override", ""),
                has_collision_exclusions=bool(merged.get("collision_excluded_body_names")),
                collision_body_count=merged.get("collision_body_count", 0),
                collision_excluded_body_names=list(
                    merged.get("collision_excluded_body_names", [])
                ),
            )
            if merged["collision_path"]:
                link.rigid_group_collision_path = merged["collision_path"]
                link.rigid_group_collision_offset = merged["collision_offset"]
                link.has_explicit_collision = True
            # Propagate the ``!acc_*`` flag onto the LinkNode so the
            # collision generator can skip primitive auto-fit and use
            # the visual mesh as collision.  See LinkNode docstring +
            # collision_generator._resolve_link_collision.
            if merged.get("wants_accurate_collision"):
                link.wants_accurate_collision = True
            if link.collision_override == "visual":
                link.wants_accurate_collision = True
            log(
                f"  {urdf_name}: MERGED ({len(merged['member_paths'])} members) "
                f"mass={merged['mass']*1000:.2f} g"
            )
        elif getattr(occ, "is_frame_only", False):
            link = LinkNode(
                urdf_name=urdf_name,
                clean_name=occ.clean_name,
                assembly=asm,
                occurrence_path=path,
                global_position=occ.global_position,
                mass_kg=0.0,
                com_link_local=(0.0, 0.0, 0.0),
                inertia_at_com=InertiaTensor(),
                material_name="",
                color_rgb=None,
                bbox_size=(0.0, 0.0, 0.0),
                bbox_min=(0.0, 0.0, 0.0),
                bbox_max=(0.0, 0.0, 0.0),
                volume_m3=0.0,
                area_m2=0.0,
                collision_bbox_size=(0.0, 0.0, 0.0),
                collision_bbox_min=(0.0, 0.0, 0.0),
                collision_bbox_max=(0.0, 0.0, 0.0),
                collision_volume_m3=0.0,
                mesh_visual="",
                mesh_collision="",
                has_visual_mesh=False,
                body_count=0,
                is_empty=True,
                is_frame_only=True,
            )
            log(f"  {urdf_name}: frame-only link (no visual/collision/inertial)")
        else:
            link = LinkNode(
                urdf_name=urdf_name,
                clean_name=occ.clean_name,
                assembly=asm,
                occurrence_path=path,
                global_position=occ.global_position,
                mass_kg=occ.mass_kg,
                com_link_local=occ.com_component_local,
                inertia_at_com=InertiaTensor(**occ.inertia_at_com.as_dict())
                    if hasattr(occ.inertia_at_com, 'as_dict')
                    else occ.inertia_at_com,
                material_name=occ.material_name,
                color_rgb=occ.appearance_color_rgb,
                bbox_size=occ.bbox_size,
                bbox_min=occ.bbox_min,
                bbox_max=occ.bbox_max,
                volume_m3=occ.volume_m3,
                area_m2=occ.area_m2,
                collision_bbox_size=occ.collision_bbox_size,
                collision_bbox_min=occ.collision_bbox_min,
                collision_bbox_max=occ.collision_bbox_max,
                collision_volume_m3=occ.collision_volume_m3,
                mesh_visual=f"meshes/{asm}/{urdf_name}.obj",
                mesh_collision=f"meshes/{asm}/{urdf_name}_collision.stl",
                body_count=occ.body_count,
                is_empty=(occ.body_count == 0),
                collision_override=getattr(occ, "collision_override", ""),
                has_collision_exclusions=bool(
                    getattr(occ, "collision_excluded_body_names", [])
                ),
                collision_body_count=getattr(occ, "collision_body_count", 0),
                collision_excluded_body_names=list(
                    getattr(occ, "collision_excluded_body_names", [])
                ),
            )
            if link.collision_override == "visual":
                link.wants_accurate_collision = True

            if link.is_empty:
                log(f"  {urdf_name}: empty link (reference frame, 0 bodies)")

            # Mark explicit collision from flattened collision sibling
            if occ.clean_name in flatten_pairs:
                link.has_explicit_collision = True
                # Store the collision occurrence path for mesh export
                link._collision_sibling_path = flatten_pairs[occ.clean_name]
                log(f"  {urdf_name}: explicit collision from flattened sibling")

        model.links[urdf_name] = link

        # Add to assembly info
        if asm in model.assemblies:
            if urdf_name not in model.assemblies[asm].links:
                model.assemblies[asm].links.append(urdf_name)

    log(f"  Built {len(model.links)} links")


# ──────────────────────────────────────────────
# Step 6: Build joints
# ──────────────────────────────────────────────

# Map Fusion motion types to URDF joint types
MOTION_TO_URDF = {
    "rigid": "fixed",
    "revolute": "revolute",
    "slider": "prismatic",
    "cylindrical": "revolute",  # Approximate — URDF has no cylindrical
    "pin_slot": "revolute",     # Approximate
    "planar": "fixed",          # URDF has no planar
    "ball": "fixed",            # URDF has no ball
}


def _build_joints(
    snapshot: FusionSnapshot,
    edges: List[KinematicEdge],
    assemblies: Dict[str, AssemblyInfo],
    name_map: Dict[str, str],
    occ_to_asm: Dict[str, str],
    model: RobotModel,
    log: Logger,
):
    """Create JointNode for each kinematic edge."""

    # ── DEBUG: joint origin rpy theory (see comment block near the top
    # of this file for the long form).
    #
    # Previously the exporter wrote `<origin rpy="0 0 0"/>` on every
    # joint.  That silently discarded any rotation the user had applied
    # while mating occurrences in Fusion — e.g. on a simple pendulum,
    # the pendel rod ended up along the link's local +Y instead of
    # hanging straight down.
    #
    # The rotation was never missing from the snapshot: Fusion stored
    # it on `occurrence.transform2`, and our extractor captured it
    # (fusion_extractor._extract_rotation).  It just never made it to
    # the URDF.
    #
    # From now on, for every joint we compose the child occurrence's
    # transform2 rotation through the assemblyContext chain, express it
    # relative to the parent's composed rotation, and emit the result
    # as `<joint><origin rpy>` in extrinsic XYZ Euler angles.
    #
    # Verified against the pendulum snapshot: `pendel.transform2`
    # rotation decodes to rpy=(π/2, 0, π/2) — exactly the compensator
    # downstream users were hand-applying to their xacro.
    log("  NOTE: joint origin rpy derived from child occurrence's "
        "transform2 rotation (was hardcoded 0,0,0 pre-2026-04-13)")

    # Reverse lookup: full occurrence path → URDF link name.  Keying by
    # full_path (unique) instead of (asm, clean_name) (collides for
    # re-used components) is what fixed the parallel-gripper bug where
    # both pivots and both grippers shared a single URDF name.  Edges
    # carry their resolved parent_path / child_path; we look them up
    # here.  A fallback (asm, clean_name) map covers older edges or
    # edges where path resolution returned an empty string.
    path_to_urdf: Dict[str, str] = {}
    asm_comp_to_urdf: Dict[Tuple[str, str], str] = {}
    for path, urdf_name in name_map.items():
        occ = snapshot.occurrences[path]
        asm_key = occ_to_asm.get(path, "ROOT")
        path_to_urdf[path] = urdf_name
        # Fallback mapping — overwrites on collision but kept for the
        # rare edge where the resolver couldn't return a full_path.
        asm_comp_to_urdf[(asm_key, occ.clean_name)] = urdf_name

    def _edge_urdf(asm: str, comp: str, full_path: str) -> Optional[str]:
        """Resolve a KinematicEdge endpoint to its URDF link name."""
        if full_path and full_path in path_to_urdf:
            return path_to_urdf[full_path]
        return asm_comp_to_urdf.get((asm, comp))

    # Topologically sort edges so each joint is processed after its parent's
    # incoming joint.  The parent-bake-correction below reads
    # ``parent_link.needs_mesh_bake`` / ``parent_link.mesh_bake_offset`` —
    # fields that get set when the joint that CREATED that parent is built.
    # Extraction order can put a child joint (e.g. ``imu_joint`` on the
    # pendel) before its parent's joint (``pendel_joint``), especially when
    # the parent joint is MOUNT (cross-assembly) and extraction orders
    # mounts last.  Without this sort, ``imu_joint`` would be missing the
    # 120 mm bake shift from pendel and the IMU lands on the pivot instead
    # of the end of the rod.
    children_by_parent: Dict[str, List[KinematicEdge]] = {}
    for e in edges:
        p_urdf = _edge_urdf(e.parent_asm, e.parent_comp, e.parent_path)
        children_by_parent.setdefault(p_urdf, []).append(e)
    ordered_edges: List[KinematicEdge] = []
    seen_joints: Set[str] = set()
    queue: List[str] = [model.root_link]
    while queue:
        current = queue.pop(0)
        for e in children_by_parent.get(current, []):
            if e.joint_name in seen_joints:
                continue
            seen_joints.add(e.joint_name)
            ordered_edges.append(e)
            c_urdf = _edge_urdf(e.child_asm, e.child_comp, e.child_path)
            if c_urdf:
                queue.append(c_urdf)
    for e in edges:
        if e.joint_name not in seen_joints:
            ordered_edges.append(e)
    edges = ordered_edges

    for edge in edges:
        fj = edge.fusion_joint

        # Resolve URDF names — full_path keyed lookup disambiguates
        # re-used components.
        parent_urdf = _edge_urdf(edge.parent_asm, edge.parent_comp, edge.parent_path)
        child_urdf = _edge_urdf(edge.child_asm, edge.child_comp, edge.child_path)
        
        if not parent_urdf or not child_urdf:
            model.warnings.append(
                f"Joint '{edge.joint_name}': could not resolve "
                f"parent={edge.parent_asm}/{edge.parent_comp} or "
                f"child={edge.child_asm}/{edge.child_comp}"
            )
            log.warning(f"Unresolved joint: {edge.joint_name}")
            continue
        
        parent_link = model.links.get(parent_urdf)
        child_link = model.links.get(child_urdf)
        
        if not parent_link or not child_link:
            model.warnings.append(f"Joint '{edge.joint_name}': missing link data")
            continue
        
        # Compute joint origin in global coordinates
        joint_global = _compute_joint_global_origin(fj, edge, assemblies, snapshot)
        
        # URDF joint type
        urdf_type = MOTION_TO_URDF.get(fj.motion_type, "fixed")

        # Unlimited Fusion revolute → URDF "continuous".  Fusion's default
        # revolute with no rotation limits set is a free pivot (pendulum,
        # wheel, spinner).  URDF spec: `continuous` is literally "revolute
        # with no upper/lower" — that's the right type, not `revolute`
        # with fake ±π limits the way the code used to emit.
        if urdf_type == "revolute" and not fj.has_rotation_limits:
            urdf_type = "continuous"

        if getattr(child_link, "is_frame_only", False) and urdf_type != "fixed":
            msg = (
                f"Joint '{edge.joint_name}' has a frame-only child "
                f"'{child_urdf}' - forcing fixed; !frame_* links are "
                f"attachment frames, not articulated bodies."
            )
            model.warnings.append(msg)
            log.warning(msg)
            urdf_type = "fixed"

        # Resolve rotations up-front so bake offsets and origin_xyz can
        # both be expressed in the correct frames below.  Use the
        # edge's resolved full_path (set by _resolve_occ_path), which
        # uniquely identifies the right occurrence even when the
        # component is re-used.
        child_full = edge.child_path
        parent_full = edge.parent_path
        r_child_world = _global_rotation_for_occurrence(child_full, snapshot)
        r_parent_world = _global_rotation_for_occurrence(parent_full, snapshot)

        child_anchor_occ = snapshot.occurrences.get(child_link.occurrence_path)
        child_uses_explicit_frame_anchor = (
            child_link.is_merged
            and child_anchor_occ is not None
            and getattr(child_anchor_occ, "is_frame_only", False)
        )

        # For fixed joints: origin = child_global - parent_global.
        # For revolute/prismatic: origin = joint_axis_global - parent_global.
        # When a merged rigid-group child is anchored on a !frame_* marker,
        # that marker is the user's explicit link/joint frame declaration.
        # In that case let it override Fusion's geometry.origin. This keeps
        # frame-anchored wheels rotating around their own frame instead of
        # around a stale CAD/component origin.
        if urdf_type == "fixed":
            origin_xyz = _vec_sub(child_link.global_position, parent_link.global_position)
            origin_method = "child_minus_parent"
        else:
            if child_uses_explicit_frame_anchor:
                joint_global = child_link.global_position
                log(
                    f"  {edge.joint_name}: using frame-only rigid-group "
                    f"anchor '{child_urdf}' as movable joint origin"
                )
            origin_xyz = _vec_sub(joint_global, parent_link.global_position)
            origin_method = (
                "frame_anchor_minus_parent"
                if child_uses_explicit_frame_anchor else "joint_minus_parent"
            )

            # Mesh bake offset: child link origin relative to joint frame.
            # Computed in world frame then rotated into the child's local
            # frame (URDF visual/collision origin xyz must be in link frame,
            # not world frame — matters when the child link has a non-
            # identity world rotation, which it now can).
            #
            # Skip the bake update for closing edges — they're NOT the
            # tree parent of their child, and the tree parent (the
            # other edge for the same child) owns the link frame.
            # Letting a closing edge overwrite ``child_link.mesh_bake_offset``
            # would shift the visual mesh by the wrong joint's offset.
            if not edge.is_closing:
                bake_world = _vec_sub(child_link.global_position, joint_global)
                bake = _rotate_vec3_by_mat3(bake_world, _mat3_transpose(r_child_world))
                if abs(bake[0]) > 1e-6 or abs(bake[1]) > 1e-6 or abs(bake[2]) > 1e-6:
                    child_link.mesh_bake_offset = bake
                    child_link.needs_mesh_bake = True
                    log(f"  {edge.joint_name}: mesh bake offset = "
                        f"({bake[0]*1000:.2f}, {bake[1]*1000:.2f}, {bake[2]*1000:.2f}) mm "
                        f"[child-local frame]")

        # If parent link has a bake offset, its URDF frame sits at the
        # joint that created it (not at component origin).  All downstream
        # joint origins must be expressed relative to that URDF frame.
        #   parent_urdf_frame = parent_component_origin − bake_offset
        #   corrected_origin  = original_origin + bake_offset
        # The parent's bake_offset is now stored in parent-LOCAL frame, so
        # we rotate it back into world frame to stay consistent with
        # origin_xyz (also in world frame here; transformed below).
        if parent_link.needs_mesh_bake:
            pb_local = parent_link.mesh_bake_offset
            pb_world = _rotate_vec3_by_mat3(pb_local, r_parent_world)
            origin_xyz = _vec_add(origin_xyz, pb_world)
            log(f"  {edge.joint_name}: parent bake correction += "
                f"({pb_world[0]*1000:.2f}, {pb_world[1]*1000:.2f}, {pb_world[2]*1000:.2f}) mm "
                f"[world frame]")
        
        # Limits.  ``or`` would silently replace a legitimate 0.0 limit
        # with the fallback (a Fusion joint with upper=0 ended up as
        # upper=π in the URDF — well past the user's intended range,
        # which then caused the four-bar mechanism to over-extend
        # past its kinematic singularity in Isaac Sim).  Compare
        # against ``None`` explicitly so 0 stays 0.
        limits = None
        if urdf_type == "revolute" and fj.has_rotation_limits:
            lower = fj.rotation_min if fj.rotation_min is not None else -math.pi
            upper = fj.rotation_max if fj.rotation_max is not None else math.pi
            limits = JointLimits(lower=lower, upper=upper)
        elif urdf_type == "prismatic" and fj.has_slide_limits:
            lower = fj.slide_min_m if fj.slide_min_m is not None else 0.0
            upper = fj.slide_max_m if fj.slide_max_m is not None else 0.0
            limits = JointLimits(lower=lower, upper=upper)

        # Joint origin rpy — recovered from the child occurrence's mated
        # pose (transform2 rotation composed through the assemblyContext
        # chain).  See rotation-helpers header block above for the full
        # theory; tl;dr: until this fix the exporter always wrote
        # rpy="0 0 0" and silently dropped every rotation the user had
        # applied in Fusion.
        r_rel = _mat3_mul(_mat3_transpose(r_parent_world), r_child_world)
        origin_rpy = _rotation_to_rpy(r_rel)
        if any(abs(v) > 1e-6 for v in origin_rpy):
            log(f"  {edge.joint_name}: joint origin rpy = "
                f"({origin_rpy[0]:.6f}, {origin_rpy[1]:.6f}, {origin_rpy[2]:.6f}) rad "
                f"({math.degrees(origin_rpy[0]):+.2f}°, "
                f"{math.degrees(origin_rpy[1]):+.2f}°, "
                f"{math.degrees(origin_rpy[2]):+.2f}°) "
                f"[from child transform2 rotation]")

        # Joint origin xyz was computed above in world frame (child_world −
        # parent_world + any parent bake).  URDF requires it in the PARENT
        # LINK'S frame.  Pre-multiply by R_parent⁻¹ to rotate world-frame
        # offset into parent-local coordinates.  No-op when parent is at
        # identity world rotation.
        if any(abs(r_parent_world[i] - _IDENTITY_3X3[i]) > 1e-9 for i in range(9)):
            before = origin_xyz
            origin_xyz = _rotate_vec3_by_mat3(origin_xyz, _mat3_transpose(r_parent_world))
            log(f"  {edge.joint_name}: origin_xyz rotated into parent-local frame: "
                f"({before[0]:.6f}, {before[1]:.6f}, {before[2]:.6f}) → "
                f"({origin_xyz[0]:.6f}, {origin_xyz[1]:.6f}, {origin_xyz[2]:.6f}) m")

        # Joint axis — Fusion's ``rotationAxisVector`` is documented as
        # "world space" but empirically it's in the JOINT'S DEFINING
        # COMPONENT'S frame (same as the origin source — the gripper's
        # joint axes came out wrong when the gripper sub-asm was
        # mounted into Assem1 with a non-identity rotation).  Lift to
        # true world frame by rotating through the defining assembly's
        # ``global_rotation`` (no-op when the joint is defined in the
        # design root — global_rotation is identity there), THEN rotate
        # into the joint's local frame with R_child_worldᵀ.
        if urdf_type in ("revolute", "prismatic", "continuous"):
            axis_in_defining = fj.axis_vector
            defining = edge.defining_component
            if (defining and defining in assemblies
                    and defining != snapshot.design_name_clean):
                R_defining_world = assemblies[defining].global_rotation
                axis_world = _rotate_vec3_by_mat3(axis_in_defining, R_defining_world)
            else:
                axis_world = axis_in_defining
            axis_local = _rotate_vec3_by_mat3(axis_world, _mat3_transpose(r_child_world))
        else:
            axis_local = fj.axis_vector

        # Closing edges are passive by definition — set the flag so
        # downstream consumers (yaml emitter, USD authoring) can
        # apply ``drive_type: none`` / no-DriveAPI without re-deriving.
        joint_is_passive = getattr(fj, "is_passive", False) or edge.is_closing

        joint = JointNode(
            name=edge.joint_name,
            joint_type=urdf_type,
            parent_link=parent_urdf,
            child_link=child_urdf,
            origin_xyz=origin_xyz,
            origin_rpy=origin_rpy,
            axis=axis_local,
            limits=limits,
            is_mount=edge.is_cross_assembly,
            fusion_source=fj.joint_source,
            origin_method=origin_method,
            origin_global=joint_global,
            is_passive=joint_is_passive,
            is_closing=edge.is_closing,
            closing_source=edge.closing_source,
        )

        # Closing-loop joints don't go in ``model.joints`` (which is
        # the URDF tree).  They live in ``model.closing_joints`` for
        # sidecar emission and downstream USD authoring.
        if edge.is_closing:
            model.closing_joints[edge.joint_name] = joint
        else:
            model.joints[edge.joint_name] = joint

        # Add to assembly info
        defining = edge.defining_component
        if defining in model.assemblies:
            model.assemblies[defining].joints.append(edge.joint_name)

        kind_tag = f" [closing/{edge.closing_source}]" if edge.is_closing else ""
        log(f"  {edge.joint_name}: {parent_urdf} → {child_urdf} [{urdf_type}]{kind_tag}")
        log(f"    origin_xyz: ({origin_xyz[0]:.6f}, {origin_xyz[1]:.6f}, {origin_xyz[2]:.6f}) m [{origin_method}]")
        log(f"    origin_global: ({joint_global[0]:.6f}, {joint_global[1]:.6f}, {joint_global[2]:.6f}) m")
    
    log(f"  Built {len(model.joints)} joints")


def _compute_joint_global_origin(
    fj: FusionJoint,
    edge: KinematicEdge,
    assemblies: Dict[str, AssemblyInfo],
    snapshot: FusionSnapshot,
) -> Vec3:
    """
    Compute joint origin in global coordinates.
    
    geometry.origin and geometryOrOriginOne are in the DEFINING COMPONENT's
    local frame. For joints defined inside sub-assemblies, we add the
    assembly's global offset.
    
    Fallback sources (occ_one_global) are already global.
    """
    defining = edge.defining_component
    source = fj.origin_source
    raw_origin = fj.origin_global_m  # Already converted to meters in Phase 1

    # Check if origin is assembly-local (from geometry methods)
    needs_offset = (
        source in ("geometry.origin", "geometryOrOriginOne", "geometryOrOriginTwo")
        and defining in assemblies
        and defining != snapshot.design_name_clean
    )

    if needs_offset:
        # Lift the joint origin from the defining sub-asm's local frame
        # into world frame.  Earlier this was a translation-only
        # ``_vec_add(raw_origin, asm_offset)`` which silently dropped
        # the sub-asm's rotation — fine when the sub-asm sat at world
        # identity but wrong otherwise (visible on the gripper-on-arm
        # mount where the gripper sub-asm carries the mount joint's
        # rotation).  Use the full rigid transform: world = R_asm ·
        # raw + t_asm.
        R_asm = assemblies[defining].global_rotation
        t_asm = assemblies[defining].global_offset
        rotated = _rotate_vec3_by_mat3(raw_origin, R_asm)
        return _vec_add(rotated, t_asm)

    # occ_one_global / occ_one_transform are already global (via assemblyContext)
    # But occ_one_transform might be local if context_depth==0 for nested joints
    # occ_one_global should be reliable — it walks assemblyContext chain

    return raw_origin


# ──────────────────────────────────────────────
# Step 7: Validate
# ──────────────────────────────────────────────

def _validate(model: RobotModel, log: Logger):
    """Validate kinematic tree integrity."""
    
    errors = 0
    
    # 1. Root exists
    if model.root_link not in model.links:
        if not model.root_link:
            # Empty root_link means _resolve_names couldn't match the detected
            # root back to any leaf occurrence — almost always caused by a
            # joint whose parent/child ended up pointing at a subassembly
            # rather than a leaf component (e.g. a subassembly named
            # "base_link" with no leaf of the same name inside it).
            msg = (
                "Root link could not be resolved. Likely cause: a joint's "
                "parent or child resolves to a subassembly instead of a "
                "leaf component. Check that each joint in Fusion connects "
                "two leaf components, not a subassembly container. "
                "See DESIGN_RULES.md §1.1 Link Names, §1.2 Joint Names, "
                "and §3.1 Kinematic Tree."
            )
        else:
            msg = (
                f"Root link '{model.root_link}' not found in links. "
                f"This is an internal naming mismatch between root detection "
                f"and name resolution."
            )
        model.errors.append(msg)
        log.error(msg)
        errors += 1
    
    # 2. All joints reference existing links
    for jname, joint in model.joints.items():
        if joint.parent_link not in model.links:
            model.errors.append(f"Joint '{jname}' parent '{joint.parent_link}' not in links")
            log.error(f"Joint '{jname}': parent '{joint.parent_link}' not found")
            errors += 1
        if joint.child_link not in model.links:
            model.errors.append(f"Joint '{jname}' child '{joint.child_link}' not in links")
            log.error(f"Joint '{jname}': child '{joint.child_link}' not found")
            errors += 1
    
    # 3. Tree connectivity: every non-root link must have a parent joint
    #    attaching it to the kinematic tree.  Orphan links are an export
    #    blocker — promoted to a hard error 2026-04-30 after the
    #    rigid-group merge refactor.  Pre-refactor this was a warning,
    #    which let the pendulum disaster (30+ floating links) ship.
    reachable = set()
    frontier = [model.root_link]
    while frontier:
        current = frontier.pop()
        if current in reachable:
            continue
        reachable.add(current)
        for joint in model.joints.values():
            if joint.parent_link == current:
                frontier.append(joint.child_link)

    unreachable = set(model.links.keys()) - reachable
    empty_unreachable = {
        name for name in unreachable
        if getattr(model.links.get(name), "is_empty", False)
    }
    blocking_unreachable = unreachable - empty_unreachable
    if empty_unreachable:
        members = ", ".join(sorted(empty_unreachable))
        model.warnings.append(
            f"Empty orphan reference link(s) ignored: {members}"
        )
        log.warning(f"Empty orphan reference link(s) ignored: {members}")
    if blocking_unreachable:
        members = ", ".join(sorted(blocking_unreachable))
        msg = (
            f"Orphan links — no joint attaches them to the root '{model.root_link}': "
            f"{members}. Either add a Fusion joint connecting each one to the tree, "
            f"or put them in a Rigid Group with their parent so they merge into a "
            f"single URDF link. See DESIGN_RULES.md §2.3 Rigid Groups "
            f"and §3.1 Kinematic Tree."
        )
        model.errors.append(msg)
        log.error(msg)
        errors += 1
    
    # 4. No duplicate children (each link has at most one parent joint)
    child_counts: Dict[str, List[str]] = {}
    for jname, joint in model.joints.items():
        child_counts.setdefault(joint.child_link, []).append(jname)
    
    for child, joints in child_counts.items():
        if len(joints) > 1:
            model.warnings.append(
                f"Link '{child}' has multiple parent joints: {joints}. "
                f"URDF requires exactly one parent. Check joint directions in Fusion, "
                f"or tag intentional loop-closing joints with !closing_*. "
                f"See DESIGN_RULES.md §3.1.1 Closed Kinematic Chains and "
                f"§3.2 Joint Direction."
            )
            log.warning(f"Multiple parents for '{child}': {joints}")
    
    # 5. Positive mass for all links (skip empty reference frames)
    zero_mass = [
        name for name, link in model.links.items()
        if link.mass_kg <= 0 and not link.is_empty
    ]
    if zero_mass:
        model.warnings.append(f"Zero-mass links: {zero_mass}")
        log.warning(f"Zero-mass links: {zero_mass}")
    
    # 6. No self-referencing joints
    for jname, joint in model.joints.items():
        if joint.parent_link == joint.child_link:
            model.errors.append(f"Joint '{jname}' connects '{joint.parent_link}' to itself")
            log.error(f"Self-referencing joint: {jname}")
            errors += 1
    
    # 7. Per-assembly connectivity is now covered by the global orphan
    #    check above (rule 3).  After the rigid-group merge refactor a
    #    rigid group collapses to one link, so per-assembly connectivity
    #    can no longer be a meaningful diagnostic — a single-link
    #    assembly is a normal outcome.

    if errors == 0:
        log(f"  Validation passed ({len(model.warnings)} warnings)")
    else:
        log.error(f"Validation FAILED: {errors} errors, {len(model.warnings)} warnings")

    # Render the kinematic tree into the log.  Useful on every run as
    # a quick sanity check, essential when validation fails so the
    # user can see *why* (orphans, multi-parents, flipped joints) at
    # a glance.  Stash the rendered string on the model so the
    # entry script can show it in the error dialog without redoing
    # the work.
    try:
        from .tree_render import render_tree
        tree_text = render_tree(model)
        model._kinematic_tree_text = tree_text
        log("")
        log("Kinematic tree:")
        for line in tree_text.splitlines():
            log(f"  {line}")
    except Exception as e:
        log.warning(f"  Tree render skipped: {e}")


# ──────────────────────────────────────────────
# Vector helpers
# ──────────────────────────────────────────────

def _vec_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

def _vec_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
