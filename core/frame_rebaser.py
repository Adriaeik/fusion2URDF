"""Post-export link-frame rebasing without mesh modification.

The Fusion extractor produces a physically correct canonical ``RobotModel``
whose mesh files live in the original component/link coordinate systems.  This
module changes only the coordinate frames used by URDF/Xacro:

* the root frame can be aligned with Fusion design world (X forward, Z up),
* revolute/continuous child frames can use local +Z as their joint axis, and
* a per-package CSV can request an absolute world RPY for any link.

For every link, ``C`` is the orientation of the post frame expressed in the
original link frame.  Geometry is compensated with ``C^-1`` and joint zero
poses are changed with ``C_parent^-1 * T_parent_child * C_child``.  Therefore
the visible/collision geometry and kinematics are unchanged for all joint
angles; only the names/axes of the coordinate frames change.

Translations are intentionally not part of the v1 override format.  A
translation perpendicular to a revolute axis does not commute with rotation
and would require an additional virtual joint frame to preserve motion.
"""

from __future__ import annotations

import csv
import json
import math
import os
from dataclasses import asdict, fields
from typing import Dict, Iterable, Optional, Tuple

from .data_types import (
    AssemblyInfo,
    CollisionInfo,
    CollisionPrimitive,
    ExportConfig,
    InertiaTensor,
    JointLimits,
    JointNode,
    LinkNode,
    RobotModel,
    RPY,
    Vec3,
)


Matrix3 = Tuple[float, ...]

IDENTITY_3: Matrix3 = (
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
)

FRAME_CACHE_VERSION = 1
FRAME_CACHE_FILENAME = "frame_model.json"
FRAME_GUIDE_FILENAME = "FRAME_OVERRIDES.md"

CSV_COLUMNS = (
    "link",
    "parent_joint",
    "joint_type",
    "rule",
    "original_roll_deg",
    "original_pitch_deg",
    "original_yaw_deg",
    "post_roll_deg",
    "post_pitch_deg",
    "post_yaw_deg",
    "role",
)


def configure_frames(
    model: RobotModel,
    config: ExportConfig,
    package_dir: str,
    log=None,
    emit_artifacts: bool = True,
) -> Dict[str, dict]:
    """Load/update the package CSV and apply its frame plan in-place.

    The caller must pass the canonical collision-resolved model.  In the main
    export pipeline this runs after collision STL generation and immediately
    before URDF/Xacro generation.
    """
    config_dir = os.path.join(package_dir, "config")
    filename = getattr(config, "frame_overrides_filename", "") or "frame_overrides.csv"
    csv_path = os.path.join(config_dir, os.path.basename(filename))
    existing = load_frame_overrides(csv_path, log=log)
    convention = (getattr(config, "frame_convention", "ros") or "ros").strip().lower()

    plan = plan_frame_rebases(
        model,
        overrides=existing,
        convention=convention,
        log=log,
    )
    if emit_artifacts:
        os.makedirs(config_dir, exist_ok=True)
        write_frame_overrides(csv_path, plan)
        write_frame_guide(os.path.join(config_dir, FRAME_GUIDE_FILENAME))
    apply_frame_rebases(model, plan, log=log)

    changed = sum(1 for item in plan.values() if not _matrix_close(item["rebase"], IDENTITY_3))
    _info(log, f"  Frame convention: {convention}")
    if emit_artifacts:
        _info(log, f"  Frame overrides:  config/{os.path.basename(csv_path)}")
    else:
        _info(log, "  Frame overrides:  not emitted (minimal package)")
    _info(log, f"  Rebased links:    {changed}/{len(plan)}")
    return plan


def plan_frame_rebases(
    model: RobotModel,
    overrides: Optional[Dict[str, dict]] = None,
    convention: str = "ros",
    log=None,
) -> Dict[str, dict]:
    """Return a simultaneous orientation-only rebase plan for every link."""
    overrides = overrides or {}
    convention = (convention or "ros").strip().lower()
    if convention not in ("ros", "fusion"):
        _warn(log, f"Unknown frame convention '{convention}'; using 'ros'")
        convention = "ros"

    original_world = _original_world_rotations(model)
    parent_joint = {joint.child_link: joint for joint in model.joints.values()}
    plan: Dict[str, dict] = {}

    for link_name, link in model.links.items():
        old_world = original_world.get(link_name, IDENTITY_3)
        joint = parent_joint.get(link_name)
        role = _automatic_role(model, link_name, joint, convention)
        default_rule = "auto" if role != "unchanged" else "keep"
        source_row = overrides.get(link_name, {})
        rule = str(source_row.get("rule", default_rule) or default_rule).strip().lower()
        if rule not in ("auto", "keep", "world_rpy"):
            _warn(log, f"Frame '{link_name}': unknown rule '{rule}', using '{default_rule}'")
            rule = default_rule

        if rule == "keep":
            desired_world = old_world
        elif rule == "world_rpy":
            requested = _post_rpy_degrees(source_row, link_name, log)
            if requested is None:
                rule = default_rule
                desired_world = _automatic_world_rotation(
                    model, link_name, joint, role, old_world, log
                )
            elif link_name == model.root_link:
                # A URDF root has no parent joint: its frame *is* the URDF
                # world and therefore cannot carry an arbitrary world RPY
                # without adding a wrapper link.  Keep the invariant promised
                # by this layer (geometry unchanged) and use the convention's
                # automatic root instead.
                _warn(
                    log,
                    f"Frame '{link_name}': world_rpy is not valid for a URDF "
                    f"root; using '{default_rule}'",
                )
                rule = default_rule
                desired_world = _automatic_world_rotation(
                    model, link_name, joint, role, old_world, log
                )
            else:
                desired_world = rpy_to_matrix(tuple(math.radians(v) for v in requested))
        else:
            desired_world = _automatic_world_rotation(
                model, link_name, joint, role, old_world, log
            )

        # C = R_world_old^T * R_world_new: new basis expressed in old axes.
        rebase = mat3_mul(mat3_transpose(old_world), desired_world)
        plan[link_name] = {
            "link": link_name,
            "parent_joint": joint.name if joint else "",
            "joint_type": joint.joint_type if joint else "root",
            "rule": rule,
            "role": role,
            "original_world": old_world,
            "desired_world": desired_world,
            "rebase": rebase,
        }

    return plan


def apply_frame_rebases(model: RobotModel, plan: Dict[str, dict], log=None) -> None:
    """Apply a previously computed frame plan to ``model`` in-place."""
    if getattr(model, "_frames_applied", False):
        raise ValueError("Frame rebases have already been applied to this RobotModel")

    rebases: Dict[str, Matrix3] = {
        name: tuple(item.get("rebase", IDENTITY_3))
        for name, item in plan.items()
    }

    # Link payloads are transformed first.  All reads below are from the
    # canonical model and each link is independent, so ordering is irrelevant.
    for link_name, link in model.links.items():
        c = rebases.get(link_name, IDENTITY_3)
        old_to_new = mat3_transpose(c)  # C^-1 for an orthonormal rotation
        old_bake = (
            tuple(link.mesh_bake_offset)
            if getattr(link, "needs_mesh_bake", False)
            else (0.0, 0.0, 0.0)
        )

        old_mesh_rotation = rpy_to_matrix(
            tuple(getattr(link, "mesh_origin_rpy", (0.0, 0.0, 0.0)))
        )
        new_mesh_rotation = mat3_mul(old_to_new, old_mesh_rotation)
        new_bake = mat3_vec(old_to_new, old_bake)

        # Resolve CoM in the old link frame before rebasing.  Canonical models
        # store component-local CoM plus the movable-joint bake offset.
        if getattr(link, "inertial_origin_xyz", None) is not None:
            old_com = tuple(link.inertial_origin_xyz)
        else:
            old_com = vec_add(tuple(link.com_link_local), old_bake)
        new_com = mat3_vec(old_to_new, old_com)

        link.mesh_bake_offset = _clean_vec(new_bake)
        link.mesh_origin_rpy = _clean_vec(matrix_to_rpy(new_mesh_rotation))
        link.inertial_origin_xyz = _clean_vec(new_com)
        # robot_data.yaml historically exposes com_link_local.  Once a post
        # frame exists, report the actual exported-link CoM there as well.
        link.com_link_local = _clean_vec(new_com)
        link.inertia_at_com = rotate_inertia(link.inertia_at_com, old_to_new)
        link.needs_mesh_bake = (
            getattr(link, "needs_mesh_bake", False)
            or not _matrix_close(c, IDENTITY_3)
            or not _vec_close(new_bake, (0.0, 0.0, 0.0))
        )
        link.frame_rebase_rpy = _clean_vec(matrix_to_rpy(c))
        link.frame_rule = plan.get(link_name, {}).get("rule", "keep")

        collision = getattr(link, "collision", None)
        if collision is not None:
            if collision.origin_xyz is not None:
                old_collision_origin = tuple(collision.origin_xyz)
            else:
                old_collision_origin = old_bake
                if link.rigid_group_collision_offset:
                    old_collision_origin = vec_add(
                        old_collision_origin,
                        tuple(link.rigid_group_collision_offset),
                    )
            old_collision_rotation = rpy_to_matrix(
                tuple(getattr(collision, "origin_rpy", (0.0, 0.0, 0.0)))
            )
            collision.origin_xyz = _clean_vec(
                mat3_vec(old_to_new, old_collision_origin)
            )
            collision.origin_rpy = _clean_vec(
                matrix_to_rpy(mat3_mul(old_to_new, old_collision_rotation))
            )

    # Joint zero-pose transform: T' = C_parent^-1 * T * C_child.
    for joint in _all_joints(model):
        cp = rebases.get(joint.parent_link, IDENTITY_3)
        cc = rebases.get(joint.child_link, IDENTITY_3)
        cp_inv = mat3_transpose(cp)
        old_rotation = rpy_to_matrix(tuple(joint.origin_rpy))
        new_rotation = mat3_mul(mat3_mul(cp_inv, old_rotation), cc)
        joint.origin_xyz = _clean_vec(mat3_vec(cp_inv, tuple(joint.origin_xyz)))
        joint.origin_rpy = _clean_vec(matrix_to_rpy(new_rotation))
        if joint.joint_type in ("revolute", "continuous", "prismatic"):
            axis = normalize(mat3_vec(mat3_transpose(cc), tuple(joint.axis)))
            joint.axis = _clean_vec(axis)

    model._frames_applied = True

    for joint in model.joints.values():
        if joint.joint_type in ("revolute", "continuous"):
            if not _vec_close(joint.axis, (0.0, 0.0, 1.0), tol=1e-7):
                _warn(
                    log,
                    f"Joint '{joint.name}' axis remains {joint.axis}; "
                    f"set child '{joint.child_link}' to rule=auto to use local +Z",
                )


def load_frame_overrides(path: str, log=None) -> Dict[str, dict]:
    """Read an existing override CSV keyed by link name."""
    if not os.path.isfile(path):
        return {}
    rows: Dict[str, dict] = {}
    try:
        with open(path, "r", encoding="utf-8-sig", newline="") as handle:
            for row in csv.DictReader(handle):
                name = str(row.get("link", "") or "").strip()
                if not name or name.startswith("#"):
                    continue
                rows[name] = dict(row)
    except Exception as exc:
        _warn(log, f"Could not read frame overrides '{path}': {exc}")
        return {}
    return rows


def write_frame_overrides(path: str, plan: Dict[str, dict]) -> str:
    """Write a deterministic reference/edit CSV from a frame plan."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS, lineterminator="\n")
        writer.writeheader()
        for name in sorted(plan):
            item = plan[name]
            original = tuple(math.degrees(v) for v in matrix_to_rpy(item["original_world"]))
            post = tuple(math.degrees(v) for v in matrix_to_rpy(item["desired_world"]))
            writer.writerow({
                "link": name,
                "parent_joint": item.get("parent_joint", ""),
                "joint_type": item.get("joint_type", ""),
                "rule": item.get("rule", "keep"),
                "original_roll_deg": _fmt_deg(original[0]),
                "original_pitch_deg": _fmt_deg(original[1]),
                "original_yaw_deg": _fmt_deg(original[2]),
                "post_roll_deg": _fmt_deg(post[0]),
                "post_pitch_deg": _fmt_deg(post[1]),
                "post_yaw_deg": _fmt_deg(post[2]),
                "role": item.get("role", "unchanged"),
            })
    return path


def write_frame_guide(path: str) -> str:
    """Write the short user guide beside ``frame_overrides.csv``."""
    text = """# Frame overrides

The exporter changes coordinate frames without changing mesh vertices.

- `rule=auto`: root uses Fusion design world (X forward, Z up); a
  revolute/continuous child uses local +Z as its rotation axis.
- `rule=keep`: keep the original extracted link orientation.
- `rule=world_rpy`: for a non-root link, use `post_roll_deg`, `post_pitch_deg`, and
  `post_yaw_deg` as the link frame's absolute zero-pose orientation in the
  Fusion design-world frame.

The URDF root frame is the URDF world and is therefore always handled by
`auto`/`keep`; an arbitrary root RPY would need an extra wrapper link.

The `original_*` columns are regenerated reference values. Edit only `rule`
and the `post_*` columns. Translations are intentionally unsupported because
an arbitrary translated revolute frame would change the physical rotation.

After editing, regenerate only URDF/Xacro/YAML/docs (no Fusion mesh export):

```powershell
# From the fusion2URDF repository root:
python tools/reframe.py <path-to-description-package>
```

From the directory containing the checkout, the equivalent module command is
`python -m fusion2URDF.tools.reframe <path-to-description-package>`.
"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(text)
    return path


def save_frame_cache(
    model: RobotModel,
    config: ExportConfig,
    path: str,
) -> str:
    """Serialize a canonical, collision-resolved model for offline reframing."""
    payload = {
        "cache_version": FRAME_CACHE_VERSION,
        "model": asdict(model),
        "config": asdict(config),
    }
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return path


def load_frame_cache(path: str) -> Tuple[RobotModel, ExportConfig]:
    """Load a model/config pair written by :func:`save_frame_cache`."""
    with open(path, "r", encoding="utf-8") as handle:
        payload = json.load(handle)
    version = payload.get("cache_version")
    if version != FRAME_CACHE_VERSION:
        raise ValueError(
            f"Unsupported frame cache version {version!r}; expected {FRAME_CACHE_VERSION}"
        )
    return _model_from_dict(payload.get("model", {})), _config_from_dict(
        payload.get("config", {})
    )


def rpy_to_matrix(rpy: RPY) -> Matrix3:
    """URDF RPY (Rz(yaw) * Ry(pitch) * Rx(roll)) to row-major matrix."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        cy * cp,
        cy * sp * sr - sy * cr,
        cy * sp * cr + sy * sr,
        sy * cp,
        sy * sp * sr + cy * cr,
        sy * sp * cr - cy * sr,
        -sp,
        cp * sr,
        cp * cr,
    )


def matrix_to_rpy(matrix: Matrix3) -> RPY:
    """Row-major rotation matrix to URDF extrinsic-XYZ RPY."""
    m = tuple(matrix)
    pitch = math.atan2(-m[6], math.sqrt(m[0] * m[0] + m[3] * m[3]))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(m[7], m[8])
        yaw = math.atan2(m[3], m[0])
    else:
        # At gimbal lock choose yaw=0 and recover an equivalent roll.
        roll = math.atan2(-m[5], m[4])
        yaw = 0.0
    return (roll, pitch, yaw)


def mat3_mul(a: Matrix3, b: Matrix3) -> Matrix3:
    return tuple(
        sum(a[row * 3 + k] * b[k * 3 + col] for k in range(3))
        for row in range(3)
        for col in range(3)
    )


def mat3_transpose(matrix: Matrix3) -> Matrix3:
    m = tuple(matrix)
    return (m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8])


def mat3_vec(matrix: Matrix3, vector: Vec3) -> Vec3:
    m, v = tuple(matrix), tuple(vector)
    return (
        m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
        m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
        m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
    )


def rotate_inertia(inertia: InertiaTensor, rotation: Matrix3) -> InertiaTensor:
    """Return ``rotation * I * rotation^T`` for a symmetric tensor."""
    source = (
        inertia.ixx, inertia.ixy, inertia.ixz,
        inertia.ixy, inertia.iyy, inertia.iyz,
        inertia.ixz, inertia.iyz, inertia.izz,
    )
    result = mat3_mul(mat3_mul(rotation, source), mat3_transpose(rotation))
    return InertiaTensor(
        ixx=_clean(result[0]),
        ixy=_clean(result[1]),
        ixz=_clean(result[2]),
        iyy=_clean(result[4]),
        iyz=_clean(result[5]),
        izz=_clean(result[8]),
    )


def _automatic_role(
    model: RobotModel,
    link_name: str,
    joint: Optional[JointNode],
    convention: str,
) -> str:
    if convention != "ros":
        return "unchanged"
    if link_name == model.root_link:
        return "root_x_forward_z_up"
    if joint and joint.joint_type in ("revolute", "continuous"):
        return "revolute_axis_z"
    return "unchanged"


def _automatic_world_rotation(
    model: RobotModel,
    link_name: str,
    joint: Optional[JointNode],
    role: str,
    old_world: Matrix3,
    log=None,
) -> Matrix3:
    if role == "root_x_forward_z_up":
        return IDENTITY_3
    if role != "revolute_axis_z" or joint is None:
        return old_world

    axis_world = normalize(mat3_vec(old_world, tuple(joint.axis)))
    if length(axis_world) < 1e-10:
        _warn(log, f"Joint '{joint.name}' has a zero axis; keeping child frame")
        return old_world

    # Make X as close as possible to design-world forward while remaining
    # perpendicular to the physical rotation axis.  If forward is parallel to
    # the axis, use world Y (then world Z) as a deterministic fallback.
    x_axis = _project_onto_plane((1.0, 0.0, 0.0), axis_world)
    if length(x_axis) < 1e-8:
        x_axis = _project_onto_plane((0.0, 1.0, 0.0), axis_world)
    if length(x_axis) < 1e-8:
        x_axis = _project_onto_plane((0.0, 0.0, 1.0), axis_world)
    x_axis = normalize(x_axis)
    y_axis = normalize(cross(axis_world, x_axis))
    # Recompute X to remove accumulated projection error and guarantee a
    # right-handed orthonormal basis: X x Y = Z.
    x_axis = normalize(cross(y_axis, axis_world))
    return _matrix_from_columns(x_axis, y_axis, axis_world)


def _original_world_rotations(model: RobotModel) -> Dict[str, Matrix3]:
    """Use Fusion source rotations, with zero-pose FK as a cache fallback."""
    rotations: Dict[str, Matrix3] = {}
    for name, link in model.links.items():
        value = getattr(link, "source_world_rotation", None)
        if value is not None and len(value) == 9:
            rotations[name] = tuple(float(v) for v in value)

    # Old caches/synthetic tests may not carry source rotations.  Derive their
    # URDF zero-pose rotations, rooted at identity.
    rotations.setdefault(model.root_link, IDENTITY_3)
    remaining = list(model.joints.values())
    progress = True
    while remaining and progress:
        progress = False
        next_remaining = []
        for joint in remaining:
            if joint.parent_link not in rotations:
                next_remaining.append(joint)
                continue
            rotations.setdefault(
                joint.child_link,
                mat3_mul(rotations[joint.parent_link], rpy_to_matrix(tuple(joint.origin_rpy))),
            )
            progress = True
        remaining = next_remaining
    for name in model.links:
        rotations.setdefault(name, IDENTITY_3)
    return rotations


def _post_rpy_degrees(row: dict, link_name: str, log=None) -> Optional[Vec3]:
    values = []
    for key in ("post_roll_deg", "post_pitch_deg", "post_yaw_deg"):
        raw = row.get(key, "")
        try:
            value = float(raw)
        except (TypeError, ValueError):
            _warn(log, f"Frame '{link_name}': {key}={raw!r} is not a number")
            return None
        if not math.isfinite(value):
            _warn(log, f"Frame '{link_name}': {key} must be finite")
            return None
        values.append(value)
    return (values[0], values[1], values[2])


def _all_joints(model: RobotModel) -> Iterable[JointNode]:
    yield from model.joints.values()
    yield from getattr(model, "closing_joints", {}).values()


def _model_from_dict(data: dict) -> RobotModel:
    model = RobotModel(
        name=data.get("name", ""),
        root_link=data.get("root_link", ""),
        warnings=list(data.get("warnings", [])),
        errors=list(data.get("errors", [])),
    )
    model.links = {
        name: _link_from_dict(item)
        for name, item in data.get("links", {}).items()
    }
    model.joints = {
        name: _joint_from_dict(item)
        for name, item in data.get("joints", {}).items()
    }
    model.closing_joints = {
        name: _joint_from_dict(item)
        for name, item in data.get("closing_joints", {}).items()
    }
    model.assemblies = {
        name: AssemblyInfo(**_filtered_kwargs(AssemblyInfo, item))
        for name, item in data.get("assemblies", {}).items()
    }
    return model


def _link_from_dict(data: dict) -> LinkNode:
    item = dict(data)
    inertia = item.pop("inertia_at_com", {}) or {}
    collision = item.pop("collision", None)
    link = LinkNode(**_filtered_kwargs(LinkNode, item))
    link.inertia_at_com = InertiaTensor(**_filtered_kwargs(InertiaTensor, inertia))
    if collision:
        link.collision = _collision_from_dict(collision)
    return link


def _collision_from_dict(data: dict) -> CollisionInfo:
    item = dict(data)
    primitive = item.pop("primitive", None)
    collision = CollisionInfo(**_filtered_kwargs(CollisionInfo, item))
    if primitive:
        collision.primitive = CollisionPrimitive(
            **_filtered_kwargs(CollisionPrimitive, primitive)
        )
    return collision


def _joint_from_dict(data: dict) -> JointNode:
    item = dict(data)
    limits = item.pop("limits", None)
    joint = JointNode(**_filtered_kwargs(JointNode, item))
    if limits:
        joint.limits = JointLimits(**_filtered_kwargs(JointLimits, limits))
    return joint


def _config_from_dict(data: dict) -> ExportConfig:
    config = ExportConfig(**_filtered_kwargs(ExportConfig, data))
    config.ros2_control_command_interfaces = tuple(
        config.ros2_control_command_interfaces
    )
    return config


def _filtered_kwargs(cls, data: dict) -> dict:
    allowed = {field.name for field in fields(cls)}
    return {key: value for key, value in data.items() if key in allowed}


def _matrix_from_columns(x_axis: Vec3, y_axis: Vec3, z_axis: Vec3) -> Matrix3:
    return (
        x_axis[0], y_axis[0], z_axis[0],
        x_axis[1], y_axis[1], z_axis[1],
        x_axis[2], y_axis[2], z_axis[2],
    )


def _project_onto_plane(vector: Vec3, normal: Vec3) -> Vec3:
    scale = dot(vector, normal)
    return (
        vector[0] - scale * normal[0],
        vector[1] - scale * normal[1],
        vector[2] - scale * normal[2],
    )


def vec_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def length(vector: Vec3) -> float:
    return math.sqrt(dot(vector, vector))


def normalize(vector: Vec3) -> Vec3:
    magnitude = length(vector)
    if magnitude < 1e-12:
        return (0.0, 0.0, 0.0)
    return (
        vector[0] / magnitude,
        vector[1] / magnitude,
        vector[2] / magnitude,
    )


def _matrix_close(a: Matrix3, b: Matrix3, tol: float = 1e-9) -> bool:
    return all(abs(float(x) - float(y)) <= tol for x, y in zip(a, b))


def _vec_close(a: Vec3, b: Vec3, tol: float = 1e-9) -> bool:
    return all(abs(float(x) - float(y)) <= tol for x, y in zip(a, b))


def _clean(value: float, tol: float = 1e-12) -> float:
    if abs(value) < tol:
        return 0.0
    if abs(value - 1.0) < tol:
        return 1.0
    if abs(value + 1.0) < tol:
        return -1.0
    return float(value)


def _clean_vec(vector: Vec3) -> Vec3:
    return (_clean(vector[0]), _clean(vector[1]), _clean(vector[2]))


def _fmt_deg(value: float) -> str:
    cleaned = _clean(value, tol=5e-10)
    return f"{cleaned:.6f}"


def _info(log, message: str) -> None:
    if log is not None:
        log(message)


def _warn(log, message: str) -> None:
    if log is None:
        return
    warning = getattr(log, "warning", None)
    if callable(warning):
        warning(message)
    else:
        log(f"WARNING: {message}")
