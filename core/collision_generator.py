"""
Collision Generator — Auto-generate collision geometry for URDF.

Resolves collision for each link according to priority chain:
  1. Explicit collision sub-component/body (from Fusion design)
  2. Auto-generated primitive STL (box/cylinder/sphere from BBox), or
     generated convex-hull STL from visual OBJ vertices
  3. Visual mesh fallback (with warning)

Generates STL files for auto collision so all collision uses consistent
mesh references (no inline XML primitives).

Pure Python — no Fusion API imports.

Author: Adrian Valaker Eikeland
"""

import math
import os
import struct
from typing import Dict, List, Optional, Set, Tuple

from .data_types import (
    RobotModel, LinkNode, JointNode, CollisionInfo, CollisionPrimitive,
    ExportConfig, InertiaTensor, Vec3,
)
from ..utils import Logger


# ──────────────────────────────────────────────
# Public API
# ──────────────────────────────────────────────

def resolve_collision(model: RobotModel, config: ExportConfig, log: Logger,
                      pkg_dir: str = ""):
    """
    Resolve collision geometry for all links in the model.
    
    Modifies model.links in-place, setting link.collision for each link.
    
    Args:
        model: Robot model (modified in-place)
        config: Export configuration (collision_auto_method)
        log: Logger
        pkg_dir: Package root dir (for OBJ vertex sampling fallback)
    """
    log.section("COLLISION: RESOLVE")
    
    # Build child→parent_joint map for joint hints
    child_to_joint: Dict[str, 'JointNode'] = {}
    for jname, joint in model.joints.items():
        child_to_joint[joint.child_link] = joint
    
    stats = {
        "explicit": 0,
        "primitive": 0,
        "convex_hull": 0,
        "visual_fallback": 0,
        "visual_reuse": 0,
        "frame_only": 0,
        "empty": 0,
        "visual_only": 0,
    }
    
    for name, link in model.links.items():
        if getattr(link, "is_frame_only", False):
            link.collision = None
            stats["frame_only"] += 1
            continue
        if link.is_empty:
            stats["empty"] += 1
            continue
        collision = _resolve_link_collision(link, config, log,
                                            parent_joint=child_to_joint.get(name),
                                            pkg_dir=pkg_dir)
        link.collision = collision
        if collision is None:
            stats["visual_only"] += 1
            continue
        stats.setdefault(collision.source, 0)
        stats[collision.source] += 1
    
    log(f"\n  Collision summary:")
    log(f"    Explicit:        {stats['explicit']}")
    log(f"    Primitive (STL): {stats['primitive']}")
    log(f"    Convex hull STL: {stats['convex_hull']}")
    log(f"    Visual reuse:    {stats['visual_reuse']}")
    log(f"    Visual fallback: {stats['visual_fallback']}")
    if stats["frame_only"]:
        log(f"    Frame-only:      {stats['frame_only']}")
    if stats["empty"]:
        log(f"    Empty (no geom): {stats['empty']}")
    if stats["visual_only"]:
        log(f"    Visual-only:     {stats['visual_only']}")


def generate_collision_meshes(model: RobotModel, pkg_dir: str, log: Logger):
    """
    Generate collision STL files for links using auto-generated collision.

    For primitive collision, generate a simple STL mesh
    (box/cylinder/sphere). For convex-hull collision, generate an STL from
    the link's exported visual OBJ vertices.
    STL coordinates are in centimeters (matching Fusion OBJ convention,
    scaled to meters via scale="0.01" in xacro).
    """
    log.section("COLLISION: GENERATE STL")
    count = 0

    for name, link in model.links.items():
        collision = link.collision
        if not collision:
            continue

        if collision.source == "primitive" and collision.primitive:
            label = collision.primitive.shape
            triangles = _primitive_to_triangles(collision.primitive)
        elif collision.source == "convex_hull":
            label = "convex hull"
            obj_path = _visual_obj_path(link, pkg_dir)
            triangles = _convex_hull_triangles_from_obj(obj_path)
            if not triangles and collision.primitive:
                log.warning(
                    f"  {name}: convex hull failed; falling back to "
                    f"{collision.primitive.shape} primitive"
                )
                collision.source = "primitive"
                label = collision.primitive.shape
                triangles = _primitive_to_triangles(collision.primitive)
        else:
            continue

        if triangles:
            stl_path = os.path.join(pkg_dir, link.mesh_collision)
            os.makedirs(os.path.dirname(stl_path), exist_ok=True)
            _write_binary_stl(stl_path, triangles)
            collision.mesh_path = link.mesh_collision
            size_kb = os.path.getsize(stl_path) / 1024
            log(f"  {name}: {label} -> {len(triangles)} tris ({size_kb:.1f} KB)")
            count += 1
        else:
            log.warning(f"  {name}: failed to generate {label} STL")

    log(f"\n  Generated {count} collision STL files")


def _visual_obj_path(link: LinkNode, pkg_dir: str) -> str:
    """Return the exported OBJ path used for collision analysis."""
    if not pkg_dir:
        return ""
    mesh_visual = getattr(link, "mesh_collision_input", "") or link.mesh_visual
    if not mesh_visual:
        return ""
    if mesh_visual.lower().endswith(".dae"):
        mesh_visual = os.path.splitext(mesh_visual)[0] + ".obj"
    return os.path.join(pkg_dir, mesh_visual)


def _fit_link_primitive(
    link: LinkNode,
    parent_joint: JointNode = None,
    obj_path: str = "",
) -> Optional[CollisionPrimitive]:
    bbox_size = link.bbox_size
    bbox_min = link.bbox_min
    bbox_max = link.bbox_max
    volume = link.volume_m3
    if (getattr(link, "has_collision_exclusions", False)
            and getattr(link, "collision_body_count", 0) > 0
            and max(getattr(link, "collision_bbox_size", (0.0, 0.0, 0.0))) > 1e-9):
        bbox_size = link.collision_bbox_size
        bbox_min = link.collision_bbox_min
        bbox_max = link.collision_bbox_max
        volume = getattr(link, "collision_volume_m3", 0.0) or 0.0
    return fit_primitive(
        bbox_size=bbox_size,
        bbox_min=bbox_min,
        bbox_max=bbox_max,
        volume=volume,
        parent_joint=parent_joint,
        obj_path=obj_path,
    )


def _primitive_shape_detail(prim: CollisionPrimitive) -> str:
    shape_detail = prim.shape
    if prim.shape == "box":
        shape_detail += (f" [{prim.size[0]*1000:.1f} x "
                         f"{prim.size[1]*1000:.1f} x "
                         f"{prim.size[2]*1000:.1f}] mm")
    elif prim.shape == "cylinder":
        shape_detail += (f" r={prim.size[0]*1000:.1f} "
                         f"h={prim.size[2]*1000:.1f} mm")
    elif prim.shape == "sphere":
        shape_detail += f" r={prim.size[0]*1000:.1f} mm"
    return shape_detail


def _resolve_link_collision(
    link: LinkNode, config: ExportConfig, log: Logger,
    parent_joint: JointNode = None,
    pkg_dir: str = "",
) -> Optional[CollisionInfo]:
    """Resolve collision for one link via priority chain."""

    # Priority 1: Explicit collision from Fusion design
    if link.has_explicit_collision and not config.override_explicit_collision:
        log(f"  {link.urdf_name}: explicit collision")
        return CollisionInfo(
            source="explicit",
            mesh_path=link.mesh_collision,
        )

    if link.has_explicit_collision and config.override_explicit_collision:
        log(f"  {link.urdf_name}: explicit collision overridden → {config.collision_auto_method}")

    if (getattr(link, "has_collision_exclusions", False)
            and getattr(link, "collision_body_count", 0) <= 0):
        log(
            f"  {link.urdf_name}: no generated collision "
            f"(! body exclusions removed all collision input)"
        )
        return None

    # ``!acc_*`` rigid-group prefix -> skip the bounding-primitive
    # auto-fit and use the visual mesh as the collision shape.  Lets
    # users opt specific links into accurate collision (gripper jaws
    # with grooves, tool tips, mating surfaces) while the rest of the
    # robot stays cheap with primitive collision.
    visual_collision_mesh = (
        getattr(link, "mesh_collision_input", "")
        if getattr(link, "has_collision_exclusions", False)
        and getattr(link, "mesh_collision_input", "")
        else link.mesh_visual
    )

    if (not getattr(link, "collision_override", "")
            and getattr(link, "wants_accurate_collision", False)
            and visual_collision_mesh):
        log(f"  {link.urdf_name}: accurate collision from visual mesh "
            f"(!acc_* tag - visual_fallback)")
        return CollisionInfo(
            source="visual_fallback",
            mesh_path=visual_collision_mesh,
        )

    obj_path = _visual_obj_path(link, pkg_dir)
    override = getattr(link, "collision_override", "") or ""

    # Per-link name prefixes override the TOML default:
    #   !acc_* -> visual mesh as collision
    #   !cxh_* -> convex hull
    #   !pri_* -> primitive
    if override == "visual" and visual_collision_mesh:
        log(f"  {link.urdf_name}: accurate collision from visual mesh "
            f"(!acc tag - visual_fallback)")
        return CollisionInfo(
            source="visual_fallback",
            mesh_path=visual_collision_mesh,
        )

    if override == "primitive":
        prim = _fit_link_primitive(link, parent_joint, obj_path)
        if prim:
            log(f"  {link.urdf_name}: primitive "
                f"(!pri tag - {_primitive_shape_detail(prim)})")
            return CollisionInfo(source="primitive", primitive=prim)
        log.warning(
            f"  {link.urdf_name}: !pri collision override could not fit a "
            f"primitive; falling back to visual mesh"
        )

    if override == "convex_hull":
        fallback_prim = _fit_link_primitive(link, parent_joint, obj_path)
        if obj_path and os.path.isfile(obj_path):
            log(f"  {link.urdf_name}: convex hull (!cxh tag)")
            return CollisionInfo(source="convex_hull", primitive=fallback_prim)
        if fallback_prim:
            log.warning(
                f"  {link.urdf_name}: !cxh collision override has no OBJ; "
                f"falling back to primitive ({_primitive_shape_detail(fallback_prim)})"
            )
            return CollisionInfo(source="primitive", primitive=fallback_prim)

    # Priority 2a: Auto-generate primitive
    if config.collision_auto_method == "primitive":
        prim = _fit_link_primitive(link, parent_joint, obj_path)
        if prim:
            log(f"  {link.urdf_name}: primitive ({_primitive_shape_detail(prim)})")
            return CollisionInfo(source="primitive", primitive=prim)

    # Priority 2b: Auto-generate convex hull from visual OBJ vertices.
    if config.collision_auto_method == "convex_hull":
        fallback_prim = _fit_link_primitive(link, parent_joint, obj_path)
        if obj_path and os.path.isfile(obj_path):
            log(f"  {link.urdf_name}: convex hull")
            return CollisionInfo(source="convex_hull", primitive=fallback_prim)
        if fallback_prim:
            log.warning(
                f"  {link.urdf_name}: no OBJ available for convex hull; "
                f"falling back to primitive ({_primitive_shape_detail(fallback_prim)})"
            )
            return CollisionInfo(source="primitive", primitive=fallback_prim)
    
    # Priority 3a: Visual reuse (same OBJ file, no separate collision mesh)
    if config.collision_auto_method == "visual_reuse":
        if visual_collision_mesh != link.mesh_visual:
            log(f"  {link.urdf_name}: visual collision with ! body exclusions")
        else:
            log(f"  {link.urdf_name}: visual reuse (collision = visual mesh)")
        return CollisionInfo(
            source="visual_reuse",
            mesh_path=visual_collision_mesh,
        )
    
    # Priority 3b: Visual fallback (separate STL copy of visual)
    log(f"  {link.urdf_name}: visual fallback (separate collision STL)")
    return CollisionInfo(
        source="visual_fallback",
        mesh_path=visual_collision_mesh,
    )


# ──────────────────────────────────────────────
# Primitive fitting
# ──────────────────────────────────────────────

def fit_primitive(
    bbox_size: Vec3,
    bbox_min: Vec3 = (0, 0, 0),
    bbox_max: Vec3 = (0, 0, 0),
    volume: float = 0.0,
    parent_joint: JointNode = None,
    obj_path: str = "",
    inertia_at_com: Optional[InertiaTensor] = None,
) -> CollisionPrimitive:
    """
    Fit a URDF collision primitive using multi-level classification.
    
    Level 1: Dimension pattern — plate (box), rod (cylinder candidate), cube
    Level 2: Volume fill ratio — boxy (f→1) vs cylindrical (f≈π/4) vs spherical (f≈π/6)
    Level 3: Joint hint — revolute child → cylinder along joint axis
    Fallback: OBJ vertex sampling for ambiguous cases
    
    Origin is bbox center (not CoM).
    
    Args:
        bbox_size: (x, y, z) bounding box in meters
        bbox_min:  bbox minimum corner (link-local)
        bbox_max:  bbox maximum corner (link-local)
        volume:    actual volume in m³ (from Fusion)
        parent_joint: Joint where this link is child (for axis hint)
        obj_path:  absolute path to OBJ file (for vertex sampling)
        inertia_at_com: deprecated; collision orientation is fitted from mesh
            geometry, not mass properties
    """
    oriented_bbox = None
    if obj_path and os.path.isfile(obj_path):
        oriented_bbox = _fit_oriented_bbox_from_obj(obj_path, bbox_size)

    orientation_matrix = None
    if oriented_bbox is not None:
        bbox_size = oriented_bbox["size"]
        bbox_min = oriented_bbox["min"]
        bbox_max = oriented_bbox["max"]
        orientation_matrix = oriented_bbox["orientation"]

    dx, dy, dz = bbox_size
    
    # Skip zero-size
    if max(dx, dy, dz) < 1e-6:
        return None
    
    # ── Origin: bbox center ──
    origin = (
        (bbox_min[0] + bbox_max[0]) / 2.0,
        (bbox_min[1] + bbox_max[1]) / 2.0,
        (bbox_min[2] + bbox_max[2]) / 2.0,
    )
    # If bbox_min/max are both zero (not populated), fall back to (0,0,0)
    if abs(origin[0]) < 1e-10 and abs(origin[1]) < 1e-10 and abs(origin[2]) < 1e-10:
        if max(abs(bbox_min[0]), abs(bbox_min[1]), abs(bbox_min[2])) < 1e-10:
            origin = (0.0, 0.0, 0.0)
    if oriented_bbox is not None:
        origin = oriented_bbox["origin"]
    
    # ── Sort dimensions, keep track of original axis ──
    # dims[i] = (size, original_axis_index)  where 0=X, 1=Y, 2=Z
    dims = sorted([(dx, 0), (dy, 1), (dz, 2)])
    d0, ax0 = dims[0]  # smallest
    d1, ax1 = dims[1]  # middle
    d2, ax2 = dims[2]  # largest
    
    # ── Ratios ──
    r01 = d0 / d1 if d1 > 1e-10 else 0.0  # small / mid
    r12 = d1 / d2 if d2 > 1e-10 else 1.0  # mid / large
    r02 = d0 / d2 if d2 > 1e-10 else 1.0  # small / large
    
    # ── Volume fill ratio ──
    bbox_vol = dx * dy * dz
    fill = volume / bbox_vol if bbox_vol > 1e-15 and volume > 0 else 0.0
    
    # ── Level 3 (early): Joint hint ──
    # If revolute/continuous, strongly prefer cylinder along joint axis
    joint_cylinder_axis = None  # original axis index (0/1/2) or None
    if parent_joint and parent_joint.joint_type in ("revolute", "continuous"):
        jax = parent_joint.axis
        if orientation_matrix:
            jax = _mat3_transpose_vec(orientation_matrix, jax)
        # Find which bbox axis is most aligned with joint axis
        dots = [abs(jax[0]), abs(jax[1]), abs(jax[2])]
        joint_cylinder_axis = dots.index(max(dots))
    
    # ── Level 1: Dimension pattern ──
    
    # Near-cube: all within ~20%
    if r02 > 0.80:
        if fill > 0 and 0.40 <= fill < 0.65:
            # Fill in actual sphere range (π/6 ≈ 0.524) → sphere
            radius = (dx + dy + dz) / 6.0
            return CollisionPrimitive(
                shape="sphere", size=(radius, 0.0, 0.0),
                origin_xyz=origin,
            )
        if fill > 0 and fill < 0.40:
            # Very low fill in near-cube → hollow cylindrical shape (ring, flange)
            # Cylinder axis = thinnest dimension (outlier)
            cyl_axis, cyl_dim = _find_outlier_axis(dims)
            return _build_cylinder_prim(
                cyl_dim, cyl_axis, dims, origin, joint_cylinder_axis,
                orientation_matrix,
            )
        # High fill (≥ 0.65) or unknown volume → box
        return _build_box_prim(bbox_size, origin, orientation_matrix)
    
    # Plate/slab: one thin, two large  (d0 << d1, d1 ≈ d2)
    is_plate = r01 < 0.15 and r12 > 0.6
    if is_plate:
        # Plate is always box (even if volume fill is low, a plate is a plate)
        return _build_box_prim(bbox_size, origin, orientation_matrix)
    
    # Disc: one thin, two NEARLY EQUAL large (d0 << d1 ≈ d2)
    # e.g. wheel [0.37, 0.151, 0.37] → r01=0.41, r12=1.0
    # The two large dims must be similar (they're the diameter directions)
    is_disc = r01 < 0.5 and r12 > 0.85
    if is_disc:
        if fill > 0 and fill < 0.88:
            # Cylindrical disc — axis along the thin dimension
            return _build_cylinder_prim(
                d0, ax0, dims, origin, joint_cylinder_axis,
                orientation_matrix,
            )
        # High fill → it's a boxy plate
        return _build_box_prim(bbox_size, origin, orientation_matrix)
    
    # Rod/stang: two small, one large  (d0 ≈ d1, d1 << d2)
    is_rod = r12 < 0.35 and r01 > 0.6
    if is_rod:
        if fill > 0 and fill >= 0.88:
            # High fill rod → rectangular beam → box
            return _build_box_prim(bbox_size, origin, orientation_matrix)
        # Cylinder only if cross-section is clearly circular (two very similar dims)
        if r01 > 0.80:
            if obj_path and os.path.isfile(obj_path):
                circularity = _measure_circularity(
                    obj_path, ax2, orientation_matrix=orientation_matrix
                )
                if circularity is None or circularity > 0.12:
                    return _build_box_prim(bbox_size, origin, orientation_matrix)
            return _build_cylinder_prim(
                d2, ax2, dims, origin, joint_cylinder_axis,
                orientation_matrix,
            )
        # Cross-section not circular enough → box
        return _build_box_prim(bbox_size, origin, orientation_matrix)
    
    # ── Level 2: Ambiguous shape — use volume fill + joint hint ──
    
    # Joint hint: revolute → cylinder
    if joint_cylinder_axis is not None:
        # Use joint axis as cylinder axis regardless of dimension pattern
        cyl_axis = joint_cylinder_axis
        cyl_dim = bbox_size[cyl_axis]
        
        # Validate with vertex sampling if available
        if obj_path and os.path.isfile(obj_path):
            circularity = _measure_circularity(
                obj_path, cyl_axis, orientation_matrix=orientation_matrix
            )
            if circularity is not None and circularity > 0.15:
                # Not circular — use box instead
                return _build_box_prim(bbox_size, origin, orientation_matrix)
        
        return _build_cylinder_prim(
            cyl_dim, cyl_axis, dims, origin, cyl_axis,
            orientation_matrix,
        )
    
    # No joint hint — use volume fill
    if fill > 0:
        if fill >= 0.88:
            return _build_box_prim(bbox_size, origin, orientation_matrix)
        if 0.40 <= fill < 0.65 and r02 > 0.75:
            # Fill in actual sphere range, near-equal dims → sphere
            radius = (dx + dy + dz) / 6.0
            return CollisionPrimitive(
                shape="sphere", size=(radius, 0.0, 0.0),
                origin_xyz=origin,
            )
        if fill < 0.40 and r02 > 0.65:
            # Very low fill, roughly equal dims → hollow cylinder
            cyl_axis, cyl_dim = _find_outlier_axis(dims)
            return _build_cylinder_prim(
                cyl_dim, cyl_axis, dims, origin, None,
                orientation_matrix,
            )
        # Moderate fill (0.65–0.88) → could be cylinder or box
        # Only choose cylinder if vertex sampling confirms circularity
        cyl_axis, cyl_dim = _find_outlier_axis(dims)
        
        if obj_path and os.path.isfile(obj_path):
            circularity = _measure_circularity(
                obj_path, cyl_axis, orientation_matrix=orientation_matrix
            )
            if circularity is not None and circularity < 0.12:
                # Confirmed circular cross-section → cylinder
                return _build_cylinder_prim(
                    cyl_dim, cyl_axis, dims, origin, None,
                    orientation_matrix,
                )
        
        # No vertex confirmation → box is the safe default
        return _build_box_prim(bbox_size, origin, orientation_matrix)
    
    # ── No volume data: fall back to dimension pattern only ──
    # Without volume we can't distinguish cylinder from box reliably.
    # Only make cylinder if dimensions strongly suggest it (one clear outlier
    # with two similar perpendicular dims).
    cyl_axis, cyl_dim = _find_outlier_axis(dims)
    
    # Check if the two non-outlier dims are similar (suggests cylinder)
    perp = [d for d, ax in dims if ax != cyl_axis]
    if len(perp) == 2:
        perp_ratio = min(perp) / max(perp) if max(perp) > 1e-10 else 1.0
        outlier_ratio = min(cyl_dim, max(perp)) / max(cyl_dim, max(perp)) if max(cyl_dim, max(perp)) > 1e-10 else 1.0
        # Two perp dims similar AND outlier is clearly different
        if perp_ratio > 0.8 and outlier_ratio < 0.4:
            if obj_path and os.path.isfile(obj_path):
                circularity = _measure_circularity(
                    obj_path, cyl_axis, orientation_matrix=orientation_matrix
                )
                if circularity is None or circularity > 0.12:
                    return _build_box_prim(bbox_size, origin, orientation_matrix)
            return _build_cylinder_prim(
                cyl_dim, cyl_axis, dims, origin, joint_cylinder_axis,
                orientation_matrix,
            )
    
    # Default: box
    return _build_box_prim(bbox_size, origin, orientation_matrix)


def _build_box_prim(
    size: Vec3,
    origin: Vec3,
    orientation_matrix: Optional[Tuple[float, ...]] = None,
) -> CollisionPrimitive:
    """Construct a box primitive, optionally oriented in link-local space."""
    return CollisionPrimitive(
        shape="box",
        size=size,
        origin_xyz=origin,
        origin_rpy=_matrix_to_rpy(orientation_matrix) if orientation_matrix else (0.0, 0.0, 0.0),
        orientation_matrix=orientation_matrix,
    )


def _build_cylinder_prim(
    cyl_length: float,
    cyl_axis: int,
    sorted_dims: list,
    origin: Vec3,
    joint_axis_override: int = None,
    orientation_matrix: Optional[Tuple[float, ...]] = None,
) -> CollisionPrimitive:
    """
    Construct a cylinder primitive.
    
    Args:
        cyl_length: length along cylinder axis (meters)
        cyl_axis: original axis index (0=X, 1=Y, 2=Z)
        sorted_dims: [(size, axis_idx), ...] sorted ascending
        origin: collision origin (bbox center)
        joint_axis_override: if set, use this axis instead
    """
    actual_axis = joint_axis_override if joint_axis_override is not None else cyl_axis
    
    # Perpendicular dimensions → radius = average diameter / 2
    all_dims = {d[1]: d[0] for d in sorted_dims}
    perp = [all_dims[i] for i in range(3) if i != actual_axis]
    radius = sum(perp) / 4.0  # (d_perp1 + d_perp2) / 2 / 2
    length = all_dims[actual_axis]
    
    if orientation_matrix:
        axis_matrix = _axis_alignment_matrix(actual_axis)
        full_matrix = _mat3_mul(orientation_matrix, axis_matrix)
        rpy = _matrix_to_rpy(full_matrix)
    else:
        full_matrix = None
        rpy = _cylinder_rpy(actual_axis)
    return CollisionPrimitive(
        shape="cylinder",
        size=(radius, 0.0, length),
        origin_xyz=origin,
        origin_rpy=rpy,
        orientation_matrix=full_matrix,
    )


def _fit_oriented_bbox_from_obj(
    obj_path: str,
    fallback_bbox_size: Vec3,
    min_volume_gain: float = 0.08,
):
    """Fit a stable oriented bounding box from OBJ vertices.

    Returns None when the oriented box is not meaningfully tighter than the
    existing axis-aligned bbox.  That keeps ordinary axis-aligned parts stable
    and only rotates collision for links that clearly benefit from it.

    The orientation is based on vertex geometry only.  Inertia can be offset
    by uneven mass distribution and is not a reliable collision-fit frame.
    """
    try:
        vertices_cm = _read_obj_vertices(obj_path, max_count=5000)
    except Exception:
        return None

    if len(vertices_cm) < 8:
        return None

    vertices = [(x * 0.01, y * 0.01, z * 0.01) for x, y, z in vertices_cm]
    aabb_vol = fallback_bbox_size[0] * fallback_bbox_size[1] * fallback_bbox_size[2]
    if aabb_vol <= 1e-15:
        return None

    candidates = []

    pca_axes = _principal_axes(vertices)
    if pca_axes is not None:
        pca_box = _oriented_bbox_candidate(vertices, _axes_to_matrix(pca_axes))
        if pca_box is not None:
            pca_box["source"] = "pca"
            candidates.append(pca_box)

    if not candidates:
        return None

    def improves(candidate):
        return candidate["volume"] < aabb_vol * (1.0 - min_volume_gain)

    improving = [c for c in candidates if improves(c)]
    if not improving:
        return None

    best = min(improving, key=lambda c: c["volume"])

    # Prefer the unrotated link frame when PCA only buys a small reduction.
    # This avoids visually noisy 45-degree boxes on broad bodies while still
    # allowing obvious diagonal beams to rotate.
    if aabb_vol <= best["volume"] * 1.12:
        return None

    if _is_axis_aligned_matrix(best["orientation"]):
        return None

    return {
        "size": best["size"],
        "min": best["min"],
        "max": best["max"],
        "origin": best["origin"],
        "orientation": best["orientation"],
    }


def _oriented_bbox_candidate(
    vertices: List[Vec3],
    orientation: Tuple[float, ...],
) -> Optional[Dict[str, object]]:
    mins = [float("inf"), float("inf"), float("inf")]
    maxs = [-float("inf"), -float("inf"), -float("inf")]
    for point in vertices:
        local = _mat3_transpose_vec(orientation, point)
        for i in range(3):
            if local[i] < mins[i]:
                mins[i] = local[i]
            if local[i] > maxs[i]:
                maxs[i] = local[i]

    size = tuple(maxs[i] - mins[i] for i in range(3))
    if max(size) < 1e-6:
        return None

    center_local = tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))
    center = _mat3_vec(orientation, center_local)
    return {
        "size": size,
        "min": tuple(mins),
        "max": tuple(maxs),
        "origin": center,
        "orientation": orientation,
        "volume": size[0] * size[1] * size[2],
    }


def _principal_axes(points: List[Vec3]) -> Optional[List[Vec3]]:
    n = len(points)
    if n < 3:
        return None

    centroid = (
        sum(p[0] for p in points) / n,
        sum(p[1] for p in points) / n,
        sum(p[2] for p in points) / n,
    )
    cov = [[0.0, 0.0, 0.0] for _ in range(3)]
    for p in points:
        d = (p[0] - centroid[0], p[1] - centroid[1], p[2] - centroid[2])
        for i in range(3):
            for j in range(i, 3):
                cov[i][j] += d[i] * d[j]
    for i in range(3):
        for j in range(i, 3):
            cov[i][j] /= n
            cov[j][i] = cov[i][j]

    eig = _jacobi_eigen_3x3(cov)
    if eig is None:
        return None
    values, vectors = eig
    order = sorted(range(3), key=lambda i: values[i], reverse=True)
    ordered_values = [values[i] for i in order]

    # If the two dominant moments are almost equal, PCA has no stable
    # direction in that plane.  That is the classic "square-ish body turns
    # 45 degrees" case: mathematically valid, visually annoying, and often
    # a worse collision proxy.  Keep those parts axis-aligned.
    largest = max(abs(ordered_values[0]), 1e-12)
    primary_gap = (ordered_values[0] - ordered_values[1]) / largest
    if primary_gap < 0.12:
        return None

    axes = [_vec_normalize(vectors[i]) for i in order]
    if any(_vec_len(a) < 1e-8 for a in axes):
        return None

    # Keep a right-handed frame; the sign of PCA vectors is arbitrary.
    cross = _vec_cross(axes[0], axes[1])
    if _vec_dot(cross, axes[2]) < 0.0:
        axes[2] = (-axes[2][0], -axes[2][1], -axes[2][2])
    return axes


def _inertia_principal_axes(
    inertia: Optional[InertiaTensor],
) -> Optional[List[Vec3]]:
    """Return principal axes from a symmetric inertia tensor.

    The smallest moment axis is listed first because it normally follows the
    longest physical dimension of the link.  This ordering is only used as a
    stable frame; primitive classification still sorts dimensions later.
    """
    if inertia is None:
        return None

    matrix = [
        [inertia.ixx, inertia.ixy, inertia.ixz],
        [inertia.ixy, inertia.iyy, inertia.iyz],
        [inertia.ixz, inertia.iyz, inertia.izz],
    ]
    scale = max(abs(v) for row in matrix for v in row)
    if scale < 1e-18:
        return None

    eig = _jacobi_eigen_3x3(matrix)
    if eig is None:
        return None
    values, vectors = eig
    span = (max(values) - min(values)) / max(max(abs(v) for v in values), 1e-18)
    if span < 0.05:
        return None

    order = sorted(range(3), key=lambda i: values[i])
    axes = [_vec_normalize(vectors[i]) for i in order]
    if any(_vec_len(a) < 1e-8 for a in axes):
        return None

    cross = _vec_cross(axes[0], axes[1])
    if _vec_dot(cross, axes[2]) < 0.0:
        axes[2] = (-axes[2][0], -axes[2][1], -axes[2][2])
    return axes


def _is_axis_aligned_matrix(m: Tuple[float, ...], tol: float = 0.02) -> bool:
    """True for identity/axis permutation frames with possible sign flips."""
    for row in range(3):
        vals = [abs(m[row * 3 + col]) for col in range(3)]
        if max(vals) < 1.0 - tol:
            return False
        if sum(1 for v in vals if v > tol) != 1:
            return False
    for col in range(3):
        vals = [abs(m[row * 3 + col]) for row in range(3)]
        if max(vals) < 1.0 - tol:
            return False
        if sum(1 for v in vals if v > tol) != 1:
            return False
    return True


def _jacobi_eigen_3x3(a):
    a = [[float(a[i][j]) for j in range(3)] for i in range(3)]
    v = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]

    for _ in range(50):
        p, q = 0, 1
        max_off = abs(a[p][q])
        for i, j in ((0, 2), (1, 2)):
            if abs(a[i][j]) > max_off:
                p, q = i, j
                max_off = abs(a[i][j])
        if max_off < 1e-12:
            break

        theta = 0.5 * math.atan2(2.0 * a[p][q], a[q][q] - a[p][p])
        c = math.cos(theta)
        s = math.sin(theta)

        app = c*c*a[p][p] - 2.0*s*c*a[p][q] + s*s*a[q][q]
        aqq = s*s*a[p][p] + 2.0*s*c*a[p][q] + c*c*a[q][q]
        a[p][p] = app
        a[q][q] = aqq
        a[p][q] = 0.0
        a[q][p] = 0.0

        for r in range(3):
            if r == p or r == q:
                continue
            arp = c*a[r][p] - s*a[r][q]
            arq = s*a[r][p] + c*a[r][q]
            a[r][p] = a[p][r] = arp
            a[r][q] = a[q][r] = arq

        for r in range(3):
            vrp = c*v[r][p] - s*v[r][q]
            vrq = s*v[r][p] + c*v[r][q]
            v[r][p] = vrp
            v[r][q] = vrq

    values = [a[i][i] for i in range(3)]
    vectors = [(v[0][i], v[1][i], v[2][i]) for i in range(3)]
    return values, vectors


def _axes_to_matrix(axes: List[Vec3]) -> Tuple[float, ...]:
    """Return row-major matrix whose columns are the supplied basis axes."""
    a0, a1, a2 = axes
    return (
        a0[0], a1[0], a2[0],
        a0[1], a1[1], a2[1],
        a0[2], a1[2], a2[2],
    )


def _axis_alignment_matrix(axis: int) -> Tuple[float, ...]:
    """Map a default Z-axis cylinder into the requested local axis."""
    if axis == 0:
        return (0.0, 0.0, 1.0,
                0.0, 1.0, 0.0,
                -1.0, 0.0, 0.0)
    if axis == 1:
        return (1.0, 0.0, 0.0,
                0.0, 0.0, 1.0,
                0.0, -1.0, 0.0)
    return (1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0)


def _mat3_mul(a: Tuple[float, ...], b: Tuple[float, ...]) -> Tuple[float, ...]:
    return tuple(
        sum(a[row * 3 + k] * b[k * 3 + col] for k in range(3))
        for row in range(3)
        for col in range(3)
    )


def _mat3_vec(m: Tuple[float, ...], v: Vec3) -> Vec3:
    return (
        m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
        m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
        m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
    )


def _mat3_transpose_vec(m: Tuple[float, ...], v: Vec3) -> Vec3:
    return (
        m[0] * v[0] + m[3] * v[1] + m[6] * v[2],
        m[1] * v[0] + m[4] * v[1] + m[7] * v[2],
        m[2] * v[0] + m[5] * v[1] + m[8] * v[2],
    )


def _matrix_to_rpy(m: Optional[Tuple[float, ...]]) -> Vec3:
    if not m:
        return (0.0, 0.0, 0.0)
    # Informational/debug RPY for the common URDF convention.  Generated STL
    # uses orientation_matrix directly, so this does not need to be the source
    # of truth for collision mesh rotation.
    pitch = math.atan2(-m[6], math.sqrt(m[0] * m[0] + m[3] * m[3]))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(m[7], m[8])
        yaw = math.atan2(m[3], m[0])
    else:
        roll = 0.0
        yaw = math.atan2(-m[1], m[4])
    return (roll, pitch, yaw)


def _vec_len(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _vec_normalize(v: Vec3) -> Vec3:
    length = _vec_len(v)
    if length < 1e-12:
        return (0.0, 0.0, 0.0)
    return (v[0] / length, v[1] / length, v[2] / length)


def _vec_dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _vec_cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _find_outlier_axis(sorted_dims: list) -> Tuple[int, float]:
    """
    Find which axis is the "odd one out" — most different from the other two.
    
    Returns (original_axis_index, dimension_along_that_axis).
    """
    d0, ax0 = sorted_dims[0]
    d1, ax1 = sorted_dims[1]
    d2, ax2 = sorted_dims[2]
    
    # Gap between d0-d1 vs d1-d2
    gap_low = (d1 - d0) / d2 if d2 > 1e-10 else 0  # how far d0 is from d1
    gap_high = (d2 - d1) / d2 if d2 > 1e-10 else 0  # how far d2 is from d1
    
    if gap_low > gap_high:
        # d0 is the outlier (disc/wheel pattern: thin axis is cylinder axis)
        return ax0, d0
    else:
        # d2 is the outlier (rod pattern: long axis is cylinder axis)
        return ax2, d2


def _cylinder_rpy(longest_axis: int):
    """
    Compute RPY to align URDF cylinder (Z-axis default) with the given axis.
    
    Args:
        longest_axis: 0=X, 1=Y, 2=Z
    """
    if longest_axis == 2:
        return (0.0, 0.0, 0.0)
    elif longest_axis == 0:
        return (0.0, math.pi / 2, 0.0)
    elif longest_axis == 1:
        return (math.pi / 2, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


# ──────────────────────────────────────────────
# OBJ vertex sampling — circularity check
# ──────────────────────────────────────────────

def _measure_circularity(
    obj_path: str,
    axis: int,
    max_vertices: int = 2000,
    orientation_matrix: Optional[Tuple[float, ...]] = None,
) -> Optional[float]:
    """
    Measure how circular a mesh cross-section is perpendicular to the given axis.
    
    Projects vertices onto the plane normal to `axis`, measures radial distances
    from centroid, and returns coefficient of variation (σ/μ).
    
    Low value (< 0.12) → circular cross-section → cylinder
    High value (> 0.15) → non-circular → box
    
    Returns None if OBJ can't be read or has too few vertices.
    """
    try:
        vertices = _read_obj_vertices(obj_path, max_vertices)
    except Exception:
        return None
    
    if len(vertices) < 10:
        return None

    if orientation_matrix:
        vertices = [
            _mat3_transpose_vec(orientation_matrix, (x, y, z))
            for x, y, z in vertices
        ]
    
    # Project to 2D plane perpendicular to axis
    # axis=0 → project to YZ, axis=1 → XZ, axis=2 → XY
    proj_axes = [(1, 2), (0, 2), (0, 1)][axis]
    u = [v[proj_axes[0]] for v in vertices]
    v = [v[proj_axes[1]] for v in vertices]
    
    # Centroid
    cu = sum(u) / len(u)
    cv = sum(v) / len(v)
    
    # Radial distances from centroid
    radii = [math.sqrt((ui - cu)**2 + (vi - cv)**2) for ui, vi in zip(u, v)]
    
    if not radii:
        return None
    
    mean_r = sum(radii) / len(radii)
    if mean_r < 1e-10:
        return None
    
    variance = sum((r - mean_r)**2 for r in radii) / len(radii)
    std_r = math.sqrt(variance)

    # A rectangular prism with only corner vertices can have a deceptively
    # low radial variance.  Require vertices to occupy many angle bins before
    # accepting "round enough"; cylinders exported by Fusion normally have
    # many segments, while boxes/beams cluster around a few corners.
    bins = set()
    for ui, vi in zip(u, v):
        angle = math.atan2(vi - cv, ui - cu)
        bins.add(int(((angle + math.pi) / (2.0 * math.pi)) * 16.0) % 16)
    if len(bins) < 8:
        return None

    return std_r / mean_r  # coefficient of variation


def _read_obj_vertices(obj_path: str, max_count: int = 2000) -> List[Tuple[float, float, float]]:
    """
    Read vertex positions from an OBJ file.
    
    Reads up to max_count vertices. OBJ coordinates are in centimeters
    (Fusion convention), returned as-is (units don't affect circularity ratio).
    """
    vertices = []
    
    with open(obj_path, 'r', encoding='utf-8', errors='replace') as f:
        for line in f:
            point = _parse_obj_vertex(line)
            if point is None:
                continue
            vertices.append(point)
            if max_count and max_count > 0 and len(vertices) >= max_count:
                break

    return vertices


def _parse_obj_vertex(line: str) -> Optional[Vec3]:
    if not line.startswith('v '):
        return None
    parts = line.split()
    if len(parts) < 4:
        return None
    try:
        return (float(parts[1]), float(parts[2]), float(parts[3]))
    except ValueError:
        return None


def _read_obj_vertices_for_hull(
    obj_path: str,
    max_count: Optional[int] = None,
) -> List[Vec3]:
    """Read hull candidates from the full OBJ without depending on vertex order.

    Fusion can write mirrored copies of the same part with different OBJ vertex
    ordering.  Reading only the first N vertices can therefore miss the tire
    outside on one side while catching it on the other.  For convex hulls we
    scan every vertex for support points, then add an even spread of sample
    vertices so detailed shapes still get a reasonable approximation.
    """
    if max_count is None:
        max_count = CONVEX_HULL_MAX_POINTS
    if not obj_path or not os.path.isfile(obj_path) or max_count <= 0:
        return []

    directions = _support_directions()
    support_points: List[Optional[Vec3]] = [None] * len(directions)
    support_dots = [float("-inf")] * len(directions)
    total_vertices = 0

    with open(obj_path, 'r', encoding='utf-8', errors='replace') as f:
        for line in f:
            point = _parse_obj_vertex(line)
            if point is None:
                continue
            total_vertices += 1
            for i, direction in enumerate(directions):
                dot = _vec_dot(point, direction)
                if dot > support_dots[i]:
                    support_dots[i] = dot
                    support_points[i] = point

    if total_vertices == 0:
        return []

    selected = _dedupe_points([p for p in support_points if p is not None])
    remaining = max_count - len(selected)
    if remaining <= 0:
        return selected[:max_count]

    samples: List[Vec3] = []
    if total_vertices <= remaining:
        step = 1.0
    else:
        step = total_vertices / float(remaining)
    next_pick = step / 2.0
    vertex_index = 0

    with open(obj_path, 'r', encoding='utf-8', errors='replace') as f:
        for line in f:
            point = _parse_obj_vertex(line)
            if point is None:
                continue
            if vertex_index + 0.5 >= next_pick:
                samples.append(point)
                next_pick += step
                if len(samples) >= remaining:
                    break
            vertex_index += 1

    return _dedupe_points(selected + samples)[:max_count]


# ──────────────────────────────────────────────
# STL mesh generation
# ──────────────────────────────────────────────
# Generates simple triangulated meshes for box, cylinder, sphere.
# Coordinates are in CENTIMETERS (matching Fusion OBJ convention).
# The xacro generator applies scale="0.01" to convert to meters.

Triangle = Tuple[Vec3, Vec3, Vec3]  # Three vertices

CONVEX_HULL_MAX_POINTS = 1200


def _convex_hull_triangles_from_obj(
    obj_path: str,
    max_vertices: int = CONVEX_HULL_MAX_POINTS,
) -> List[Triangle]:
    """Read OBJ vertices and return a generated convex hull triangle mesh."""
    if not obj_path or not os.path.isfile(obj_path):
        return []
    try:
        vertices = _read_obj_vertices_for_hull(obj_path, max_count=max_vertices)
    except Exception:
        return []
    return _convex_hull_triangles(vertices, max_vertices=max_vertices)


def _convex_hull_triangles(
    points: List[Vec3],
    max_vertices: int = CONVEX_HULL_MAX_POINTS,
) -> List[Triangle]:
    """Build a deterministic 3D convex hull from points in centimeters.

    This is a small incremental hull implementation so the Fusion add-in does
    not need scipy/trimesh inside Fusion's bundled Python environment.
    """
    points = _dedupe_points(points)
    if len(points) < 4:
        return []

    points = _select_hull_candidate_points(points, max_vertices)
    eps = _hull_epsilon(points)
    tetra = _initial_tetrahedron(points, eps)
    if tetra is None:
        return []

    i0, i1, i2, i3 = tetra
    inside = (
        (points[i0][0] + points[i1][0] + points[i2][0] + points[i3][0]) / 4.0,
        (points[i0][1] + points[i1][1] + points[i2][1] + points[i3][1]) / 4.0,
        (points[i0][2] + points[i1][2] + points[i2][2] + points[i3][2]) / 4.0,
    )

    faces = [
        _orient_face(i0, i1, i2, points, inside),
        _orient_face(i0, i3, i1, points, inside),
        _orient_face(i0, i2, i3, points, inside),
        _orient_face(i1, i3, i2, points, inside),
    ]
    tetra_set = {i0, i1, i2, i3}

    for idx, point in enumerate(points):
        if idx in tetra_set:
            continue

        visible = [
            face_index
            for face_index, face in enumerate(faces)
            if _signed_face_distance(face, point, points) > eps
        ]
        if not visible:
            continue

        visible_set = set(visible)
        edge_counts: Dict[Tuple[int, int], int] = {}
        for face_index in visible:
            a, b, c = faces[face_index]
            for edge in ((a, b), (b, c), (c, a)):
                key = tuple(sorted(edge))
                edge_counts[key] = edge_counts.get(key, 0) + 1

        horizon_edges = [edge for edge, count in edge_counts.items() if count == 1]
        faces = [face for face_index, face in enumerate(faces)
                 if face_index not in visible_set]

        for a, b in horizon_edges:
            if a == idx or b == idx:
                continue
            face = _orient_face(a, b, idx, points, inside)
            if _face_area2(face, points) > eps * eps:
                faces.append(face)

    triangles: List[Triangle] = []
    seen: Set[Tuple[int, int, int]] = set()
    for face in faces:
        key = tuple(sorted(face))
        if key in seen or _face_area2(face, points) <= eps * eps:
            continue
        seen.add(key)
        a, b, c = face
        triangles.append((points[a], points[b], points[c]))
    return triangles


def _dedupe_points(points: List[Vec3]) -> List[Vec3]:
    out: List[Vec3] = []
    seen: Set[Tuple[float, float, float]] = set()
    for point in points:
        key = (round(point[0], 6), round(point[1], 6), round(point[2], 6))
        if key in seen:
            continue
        seen.add(key)
        out.append(point)
    return out


def _select_hull_candidate_points(points: List[Vec3], max_vertices: int) -> List[Vec3]:
    if max_vertices <= 0 or len(points) <= max_vertices:
        return points

    selected: Set[int] = set()
    directions = _support_directions()
    for direction in directions:
        best_index = max(
            range(len(points)),
            key=lambda i: _vec_dot(points[i], direction),
        )
        selected.add(best_index)
        if len(selected) >= max_vertices:
            break

    if len(selected) < max_vertices:
        step = max(1, len(points) // (max_vertices - len(selected)))
        for i in range(0, len(points), step):
            selected.add(i)
            if len(selected) >= max_vertices:
                break

    return [points[i] for i in sorted(selected)]


def _support_directions() -> List[Vec3]:
    directions: List[Vec3] = []
    for x in (-1.0, 0.0, 1.0):
        for y in (-1.0, 0.0, 1.0):
            for z in (-1.0, 0.0, 1.0):
                if x == 0.0 and y == 0.0 and z == 0.0:
                    continue
                directions.append(_vec_normalize((x, y, z)))

    # A small Fibonacci sphere fills the gaps between cardinal/diagonal
    # directions and keeps the reduced hull deterministic.
    samples = 96
    golden = math.pi * (3.0 - math.sqrt(5.0))
    for i in range(samples):
        z = 1.0 - (2.0 * i + 1.0) / samples
        radius = math.sqrt(max(0.0, 1.0 - z * z))
        theta = golden * i
        directions.append((
            math.cos(theta) * radius,
            math.sin(theta) * radius,
            z,
        ))
    return directions


def _hull_epsilon(points: List[Vec3]) -> float:
    mins = [min(p[i] for p in points) for i in range(3)]
    maxs = [max(p[i] for p in points) for i in range(3)]
    diag = math.sqrt(sum((maxs[i] - mins[i]) ** 2 for i in range(3)))
    return max(diag, 1.0) * 1e-7


def _initial_tetrahedron(points: List[Vec3], eps: float):
    extremes = set()
    for axis in range(3):
        extremes.add(min(range(len(points)), key=lambda i: points[i][axis]))
        extremes.add(max(range(len(points)), key=lambda i: points[i][axis]))

    pair = None
    pair_dist = -1.0
    extremes_list = list(extremes)
    for i, a in enumerate(extremes_list):
        for b in extremes_list[i + 1:]:
            dist = _dist2(points[a], points[b])
            if dist > pair_dist:
                pair_dist = dist
                pair = (a, b)
    if pair is None or pair_dist <= eps * eps:
        return None

    i0, i1 = pair
    line = _vec_sub(points[i1], points[i0])
    line_len = _vec_len(line)
    i2 = max(
        range(len(points)),
        key=lambda i: _vec_len(_vec_cross(_vec_sub(points[i], points[i0]), line)),
    )
    line_dist = _vec_len(_vec_cross(_vec_sub(points[i2], points[i0]), line)) / line_len
    if line_dist <= eps:
        return None

    normal = _vec_cross(_vec_sub(points[i1], points[i0]),
                        _vec_sub(points[i2], points[i0]))
    normal = _vec_normalize(normal)
    i3 = max(
        range(len(points)),
        key=lambda i: abs(_vec_dot(normal, _vec_sub(points[i], points[i0]))),
    )
    plane_dist = abs(_vec_dot(normal, _vec_sub(points[i3], points[i0])))
    if plane_dist <= eps:
        return None
    return i0, i1, i2, i3


def _orient_face(
    a: int,
    b: int,
    c: int,
    points: List[Vec3],
    inside: Vec3,
) -> Tuple[int, int, int]:
    normal = _face_normal((a, b, c), points)
    if _vec_dot(normal, _vec_sub(inside, points[a])) > 0.0:
        return a, c, b
    return a, b, c


def _signed_face_distance(face: Tuple[int, int, int], point: Vec3, points: List[Vec3]) -> float:
    normal = _face_normal(face, points)
    length = _vec_len(normal)
    if length < 1e-12:
        return 0.0
    a = points[face[0]]
    return _vec_dot(normal, _vec_sub(point, a)) / length


def _face_normal(face: Tuple[int, int, int], points: List[Vec3]) -> Vec3:
    a, b, c = face
    return _vec_cross(
        _vec_sub(points[b], points[a]),
        _vec_sub(points[c], points[a]),
    )


def _face_area2(face: Tuple[int, int, int], points: List[Vec3]) -> float:
    return _vec_len(_face_normal(face, points))


def _vec_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dist2(a: Vec3, b: Vec3) -> float:
    d = _vec_sub(a, b)
    return _vec_dot(d, d)


def _primitive_to_triangles(prim: CollisionPrimitive) -> List[Triangle]:
    """Convert a collision primitive to triangles centered at its origin (in cm)."""
    # Convert origin and size from meters to centimeters
    ox, oy, oz = prim.origin_xyz[0] * 100, prim.origin_xyz[1] * 100, prim.origin_xyz[2] * 100
    rr, rp, ry_ = prim.origin_rpy
    
    if prim.shape == "box":
        sx = prim.size[0] * 100  # meters → cm
        sy = prim.size[1] * 100
        sz = prim.size[2] * 100
        tris = _make_box(sx, sy, sz)
    elif prim.shape == "cylinder":
        radius = prim.size[0] * 100
        length = prim.size[2] * 100
        tris = _make_cylinder(radius, length, segments=24)
    elif prim.shape == "sphere":
        radius = prim.size[0] * 100
        tris = _make_sphere(radius, rings=12, segments=24)
    else:
        return []
    
    # Apply orientation.  PCA-fitted primitives carry a matrix so generated
    # STL can use the exact oriented frame instead of going through Euler
    # angles; older axis-aligned primitives keep the RPY path.
    orientation_matrix = getattr(prim, "orientation_matrix", None)
    if orientation_matrix:
        tris = _rotate_triangles_matrix(tris, orientation_matrix)
    elif abs(rr) > 1e-6 or abs(rp) > 1e-6 or abs(ry_) > 1e-6:
        tris = _rotate_triangles(tris, rr, rp, ry_)
    
    # Translate to origin
    if abs(ox) > 1e-6 or abs(oy) > 1e-6 or abs(oz) > 1e-6:
        tris = [((a[0]+ox, a[1]+oy, a[2]+oz),
                 (b[0]+ox, b[1]+oy, b[2]+oz),
                 (c[0]+ox, c[1]+oy, c[2]+oz)) for a, b, c in tris]
    
    return tris


def _make_box(sx: float, sy: float, sz: float) -> List[Triangle]:
    """Generate box triangles centered at origin. Dimensions in cm."""
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    # 8 corners
    v = [
        (-hx, -hy, -hz), ( hx, -hy, -hz), ( hx,  hy, -hz), (-hx,  hy, -hz),
        (-hx, -hy,  hz), ( hx, -hy,  hz), ( hx,  hy,  hz), (-hx,  hy,  hz),
    ]
    # 6 faces, 2 triangles each (outward normals)
    faces = [
        (0,3,2,1), (4,5,6,7),  # bottom, top
        (0,1,5,4), (2,3,7,6),  # front, back
        (1,2,6,5), (0,4,7,3),  # right, left
    ]
    tris = []
    for f in faces:
        tris.append((v[f[0]], v[f[1]], v[f[2]]))
        tris.append((v[f[0]], v[f[2]], v[f[3]]))
    return tris


def _make_cylinder(radius: float, length: float, segments: int = 24) -> List[Triangle]:
    """Generate cylinder along Z axis, centered at origin. Dimensions in cm."""
    hz = length / 2
    tris = []
    
    for i in range(segments):
        a0 = 2 * math.pi * i / segments
        a1 = 2 * math.pi * (i + 1) / segments
        x0, y0 = radius * math.cos(a0), radius * math.sin(a0)
        x1, y1 = radius * math.cos(a1), radius * math.sin(a1)
        
        # Side quad (2 tris)
        tris.append(((x0, y0, -hz), (x1, y1, -hz), (x1, y1, hz)))
        tris.append(((x0, y0, -hz), (x1, y1, hz), (x0, y0, hz)))
        
        # Top cap
        tris.append(((0, 0, hz), (x0, y0, hz), (x1, y1, hz)))
        # Bottom cap
        tris.append(((0, 0, -hz), (x1, y1, -hz), (x0, y0, -hz)))
    
    return tris


def _make_sphere(radius: float, rings: int = 12, segments: int = 24) -> List[Triangle]:
    """Generate UV sphere centered at origin. Dimensions in cm."""
    tris = []
    
    for i in range(rings):
        theta0 = math.pi * i / rings
        theta1 = math.pi * (i + 1) / rings
        st0, ct0 = math.sin(theta0), math.cos(theta0)
        st1, ct1 = math.sin(theta1), math.cos(theta1)
        
        for j in range(segments):
            phi0 = 2 * math.pi * j / segments
            phi1 = 2 * math.pi * (j + 1) / segments
            sp0, cp0 = math.sin(phi0), math.cos(phi0)
            sp1, cp1 = math.sin(phi1), math.cos(phi1)
            
            v00 = (radius * st0 * cp0, radius * st0 * sp0, radius * ct0)
            v10 = (radius * st1 * cp0, radius * st1 * sp0, radius * ct1)
            v01 = (radius * st0 * cp1, radius * st0 * sp1, radius * ct0)
            v11 = (radius * st1 * cp1, radius * st1 * sp1, radius * ct1)
            
            if i == 0:
                tris.append((v00, v11, v01))
            elif i == rings - 1:
                tris.append((v00, v10, v11))
            else:
                tris.append((v00, v10, v11))
                tris.append((v00, v11, v01))
    
    return tris


def _rotate_triangles(tris: List[Triangle], roll: float, pitch: float, yaw: float) -> List[Triangle]:
    """Apply RPY rotation to all triangle vertices."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    
    # ZYX rotation matrix
    def rot(v):
        x, y, z = v
        # Yaw (Z)
        x1 = cy * x - sy * y
        y1 = sy * x + cy * y
        z1 = z
        # Pitch (Y)
        x2 = cp * x1 + sp * z1
        y2 = y1
        z2 = -sp * x1 + cp * z1
        # Roll (X)
        x3 = x2
        y3 = cr * y2 - sr * z2
        z3 = sr * y2 + cr * z2
        return (x3, y3, z3)
    
    return [(rot(a), rot(b), rot(c)) for a, b, c in tris]


def _rotate_triangles_matrix(
    tris: List[Triangle],
    matrix: Tuple[float, ...],
) -> List[Triangle]:
    """Apply a row-major 3x3 rotation matrix to all triangle vertices."""
    def rot(v):
        return _mat3_vec(matrix, v)

    return [(rot(a), rot(b), rot(c)) for a, b, c in tris]


def _compute_normal(a: Vec3, b: Vec3, c: Vec3) -> Vec3:
    """Compute face normal from three vertices."""
    ux, uy, uz = b[0]-a[0], b[1]-a[1], b[2]-a[2]
    vx, vy, vz = c[0]-a[0], c[1]-a[1], c[2]-a[2]
    nx = uy*vz - uz*vy
    ny = uz*vx - ux*vz
    nz = ux*vy - uy*vx
    length = math.sqrt(nx*nx + ny*ny + nz*nz)
    if length > 1e-12:
        return (nx/length, ny/length, nz/length)
    return (0.0, 0.0, 1.0)


def _write_binary_stl(path: str, triangles: List[Triangle]):
    """Write binary STL file."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    
    with open(path, 'wb') as f:
        # Header (80 bytes)
        header = b'Generated by Fusion URDF/XACRO Export' + b'\0' * 43
        f.write(header[:80])
        
        # Triangle count
        f.write(struct.pack('<I', len(triangles)))
        
        # Triangles
        for a, b, c in triangles:
            n = _compute_normal(a, b, c)
            f.write(struct.pack('<fff', n[0], n[1], n[2]))
            f.write(struct.pack('<fff', a[0], a[1], a[2]))
            f.write(struct.pack('<fff', b[0], b[1], b[2]))
            f.write(struct.pack('<fff', c[0], c[1], c[2]))
            f.write(struct.pack('<H', 0))  # attribute byte count


# ──────────────────────────────────────────────
# STL re-scaling (normalize Fusion exports to cm)
# ──────────────────────────────────────────────

# Fusion exports STL in the document's display unit.
# OBJ is always in cm. To keep a uniform scale="0.01" in xacro,
# we re-scale Fusion-exported STL to centimeters after export.

UNIT_TO_CM = {
    "mm": 0.1,
    "cm": 1.0,
    "m": 100.0,
    "in": 2.54,
    "ft": 30.48,
}


# Sanity threshold for post-export bbox checks.  Fusion has been observed
# to return True from exportMgr.execute() while writing a binary STL whose
# vertices are all within ~14 nm of the origin (degenerate geometry).  Any
# real collision shape is safely above 1 mm; anything smaller is garbage.
# Kept in this module (not fusion_extractor) so scripts/validate_examples.py
# can import it in CI without also pulling in adsk.
_DEGENERATE_BBOX_THRESHOLD_CM = 0.1  # 1 mm expressed in Fusion-internal cm


def _stl_bbox_size_cm(stl_path: str) -> Tuple[float, float, float]:
    """Axis-aligned bbox size of a binary STL, in the file's own units.

    Returns (0, 0, 0) on any read failure.  Used right after Fusion STL
    export to detect the degenerate "all vertices at origin" failure
    mode that the execute()-returned-True + file-exists checks can't
    catch.
    """
    try:
        with open(stl_path, 'rb') as f:
            f.read(80)
            tri_count = struct.unpack('<I', f.read(4))[0]
            mins = [float('inf')] * 3
            maxs = [float('-inf')] * 3
            for _ in range(tri_count):
                data = f.read(50)
                if len(data) < 50:
                    break
                vals = struct.unpack('<12fH', data)
                for v_start in (3, 6, 9):
                    for axis in range(3):
                        v = vals[v_start + axis]
                        if v < mins[axis]:
                            mins[axis] = v
                        if v > maxs[axis]:
                            maxs[axis] = v
            if any(m == float('inf') for m in mins):
                return (0.0, 0.0, 0.0)
            return (maxs[0] - mins[0], maxs[1] - mins[1], maxs[2] - mins[2])
    except Exception:
        return (0.0, 0.0, 0.0)


def rescale_stl_to_cm(stl_path: str, document_unit: str) -> bool:
    """
    Re-scale a Fusion-exported binary STL from document_unit to centimeters.
    
    Reads the file, multiplies all vertex coordinates by the conversion
    factor, and writes back in-place. Returns True on success.
    
    No-op if document_unit is already 'cm'.
    """
    factor = UNIT_TO_CM.get(document_unit, 1.0)
    if abs(factor - 1.0) < 1e-9:
        return True  # Already in cm
    
    try:
        with open(stl_path, 'rb') as f:
            header = f.read(80)
            count_bytes = f.read(4)
            if len(count_bytes) < 4:
                return False
            tri_count = struct.unpack('<I', count_bytes)[0]
            
            triangles = []
            for _ in range(tri_count):
                data = f.read(50)  # 12 floats + 1 uint16
                if len(data) < 50:
                    return False
                vals = struct.unpack('<12fH', data)
                # vals: nx,ny,nz, ax,ay,az, bx,by,bz, cx,cy,cz, attr
                # Scale vertices (not normals — recompute after)
                a = (vals[3] * factor, vals[4] * factor, vals[5] * factor)
                b = (vals[6] * factor, vals[7] * factor, vals[8] * factor)
                c = (vals[9] * factor, vals[10] * factor, vals[11] * factor)
                triangles.append((a, b, c))
        
        # Write back
        _write_binary_stl(stl_path, triangles)
        return True
    except Exception:
        return False
