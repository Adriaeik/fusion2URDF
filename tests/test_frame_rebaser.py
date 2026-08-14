"""Regression tests for the offline frame-rebasing layer."""

from __future__ import annotations

import copy
import csv
import math
import os
import tempfile

from ..core.data_types import (
    AssemblyInfo,
    CollisionInfo,
    ExportConfig,
    InertiaTensor,
    JointNode,
    LinkNode,
    RobotModel,
)
from ..core.frame_rebaser import (
    IDENTITY_3,
    apply_frame_rebases,
    configure_frames,
    load_frame_cache,
    mat3_mul,
    mat3_transpose,
    mat3_vec,
    plan_frame_rebases,
    rpy_to_matrix,
    save_frame_cache,
)
from ..core.robot_model import build_model
from ..core.snapshot_io import load_snapshot
from ..core.xacro_generator import generate_urdf
from ..utils.logger import Logger


TOL = 2e-8


def _logger():
    return Logger(timestamps=False, quiet=True)


def _assert_vec_close(actual, expected, tol=TOL):
    assert len(actual) == len(expected)
    for a, b in zip(actual, expected):
        assert abs(a - b) <= tol, (actual, expected)


def _assert_matrix_close(actual, expected, tol=TOL):
    _assert_vec_close(tuple(actual), tuple(expected), tol=tol)


def _axis_rotation(axis, angle):
    x, y, z = axis
    length = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / length, y / length, z / length
    c, s, one_c = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return (
        c + x * x * one_c,
        x * y * one_c - z * s,
        x * z * one_c + y * s,
        y * x * one_c + z * s,
        c + y * y * one_c,
        y * z * one_c - x * s,
        z * x * one_c - y * s,
        z * y * one_c + x * s,
        c + z * z * one_c,
    )


def _compose(a, b):
    """Compose two (rotation, translation) rigid transforms."""
    ra, ta = a
    rb, tb = b
    rotated_tb = mat3_vec(ra, tb)
    return (
        mat3_mul(ra, rb),
        (ta[0] + rotated_tb[0], ta[1] + rotated_tb[1], ta[2] + rotated_tb[2]),
    )


def _zero_or_configured_fk(model, root_rotation, joint_positions):
    poses = {model.root_link: (root_rotation, (0.0, 0.0, 0.0))}
    remaining = list(model.joints.values())
    while remaining:
        progressed = False
        next_remaining = []
        for joint in remaining:
            parent = poses.get(joint.parent_link)
            if parent is None:
                next_remaining.append(joint)
                continue
            origin = (rpy_to_matrix(tuple(joint.origin_rpy)), tuple(joint.origin_xyz))
            q = joint_positions.get(joint.name, 0.0)
            if joint.joint_type in ("revolute", "continuous"):
                motion = (_axis_rotation(tuple(joint.axis), q), (0.0, 0.0, 0.0))
            elif joint.joint_type == "prismatic":
                motion = (
                    IDENTITY_3,
                    (joint.axis[0] * q, joint.axis[1] * q, joint.axis[2] * q),
                )
            else:
                motion = (IDENTITY_3, (0.0, 0.0, 0.0))
            poses[joint.child_link] = _compose(_compose(parent, origin), motion)
            progressed = True
        if not progressed:
            raise AssertionError("Test model is not a connected tree")
        remaining = next_remaining
    return poses


def _mesh_world_poses(model, root_rotation, joint_positions):
    link_poses = _zero_or_configured_fk(model, root_rotation, joint_positions)
    result = {}
    for name, link in model.links.items():
        bake = link.mesh_bake_offset if link.needs_mesh_bake else (0.0, 0.0, 0.0)
        mesh = (rpy_to_matrix(tuple(link.mesh_origin_rpy)), tuple(bake))
        result[name] = _compose(link_poses[name], mesh)
    return result


def _make_articulated_model():
    root_world = rpy_to_matrix((math.radians(-90.0), 0.0, math.radians(20.0)))
    joint_rpy = (math.radians(15.0), math.radians(-25.0), math.radians(35.0))
    child_world = mat3_mul(root_world, rpy_to_matrix(joint_rpy))
    sensor_joint_rpy = (0.0, math.radians(12.0), math.radians(-18.0))
    sensor_world = mat3_mul(child_world, rpy_to_matrix(sensor_joint_rpy))
    model = RobotModel(name="frame_test", root_link="base_link")
    model.links = {
        "base_link": LinkNode(
            urdf_name="base_link",
            assembly="frame_assembly",
            source_world_rotation=root_world,
            mass_kg=2.0,
            com_link_local=(0.02, -0.01, 0.04),
            inertia_at_com=InertiaTensor(
                ixx=0.11, ixy=0.012, ixz=-0.007,
                iyy=0.22, iyz=0.009, izz=0.31,
            ),
            mesh_visual="meshes/base.dae",
            mesh_collision="meshes/base_collision.stl",
            collision=CollisionInfo(
                source="explicit",
                mesh_path="meshes/base_collision.stl",
            ),
        ),
        "tilt_link": LinkNode(
            urdf_name="tilt_link",
            assembly="frame_assembly",
            source_world_rotation=child_world,
            mass_kg=1.0,
            com_link_local=(0.08, 0.01, -0.03),
            inertia_at_com=InertiaTensor(
                ixx=0.04, ixy=-0.003, ixz=0.006,
                iyy=0.08, iyz=0.002, izz=0.09,
            ),
            mesh_visual="meshes/tilt.dae",
            mesh_collision="meshes/tilt_collision.stl",
            mesh_bake_offset=(0.03, -0.02, 0.01),
            needs_mesh_bake=True,
            collision=CollisionInfo(
                source="explicit",
                mesh_path="meshes/tilt_collision.stl",
            ),
        ),
        "sensor_link": LinkNode(
            urdf_name="sensor_link",
            assembly="frame_assembly",
            source_world_rotation=sensor_world,
            mass_kg=0.1,
            com_link_local=(0.01, 0.0, 0.0),
            inertia_at_com=InertiaTensor(ixx=0.001, iyy=0.002, izz=0.003),
            mesh_visual="meshes/sensor.dae",
            mesh_collision="meshes/sensor_collision.stl",
            collision=CollisionInfo(
                source="explicit",
                mesh_path="meshes/sensor_collision.stl",
            ),
        ),
    }
    model.joints = {
        "tilt_joint": JointNode(
            name="tilt_joint",
            joint_type="revolute",
            parent_link="base_link",
            child_link="tilt_link",
            origin_xyz=(0.15, -0.04, 0.09),
            origin_rpy=joint_rpy,
            axis=(1.0, 0.0, 0.0),
        ),
        "sensor_joint": JointNode(
            name="sensor_joint",
            joint_type="fixed",
            parent_link="tilt_link",
            child_link="sensor_link",
            origin_xyz=(0.04, 0.02, -0.01),
            origin_rpy=sensor_joint_rpy,
        ),
    }
    model.assemblies = {
        "frame_assembly": AssemblyInfo(
            name="frame_assembly",
            links=["base_link", "tilt_link", "sensor_link"],
            joints=["tilt_joint", "sensor_joint"],
        )
    }
    return model, root_world


def test_ros_rebase_preserves_mesh_fk_and_inertia():
    canonical, old_root_world = _make_articulated_model()
    rebased = copy.deepcopy(canonical)
    plan = plan_frame_rebases(rebased, convention="ros", log=_logger())

    _assert_matrix_close(plan["base_link"]["desired_world"], IDENTITY_3)
    apply_frame_rebases(rebased, plan, log=_logger())
    _assert_vec_close(rebased.joints["tilt_joint"].axis, (0.0, 0.0, 1.0))

    for q in (0.0, 0.37, -1.1):
        positions = {"tilt_joint": q}
        before = _mesh_world_poses(canonical, old_root_world, positions)
        after = _mesh_world_poses(rebased, IDENTITY_3, positions)
        for name in canonical.links:
            _assert_matrix_close(after[name][0], before[name][0])
            _assert_vec_close(after[name][1], before[name][1])

        # World inertia at the moving link CoM must also be invariant.
        before_link_r = _zero_or_configured_fk(
            canonical, old_root_world, positions
        )["tilt_link"][0]
        after_link_r = _zero_or_configured_fk(
            rebased, IDENTITY_3, positions
        )["tilt_link"][0]
        before_i = _inertia_matrix(canonical.links["tilt_link"].inertia_at_com)
        after_i = _inertia_matrix(rebased.links["tilt_link"].inertia_at_com)
        before_world_i = mat3_mul(
            mat3_mul(before_link_r, before_i), mat3_transpose(before_link_r)
        )
        after_world_i = mat3_mul(
            mat3_mul(after_link_r, after_i), mat3_transpose(after_link_r)
        )
        _assert_matrix_close(after_world_i, before_world_i)


def test_csv_override_and_cache_round_trip():
    model, _ = _make_articulated_model()
    cfg = ExportConfig(
        package_name="frame_test_description",
        frame_convention="ros",
        include_ros2_control=False,
        include_docs=False,
        include_robot_data_yaml=False,
        include_readme=False,
        include_rviz=False,
    )
    with tempfile.TemporaryDirectory(dir=os.getcwd()) as temp_dir:
        package_dir = os.path.join(temp_dir, cfg.package_name)
        cache_path = os.path.join(temp_dir, "debug", "frame_model.json")
        save_frame_cache(model, cfg, cache_path)

        loaded, loaded_cfg = load_frame_cache(cache_path)
        plan = configure_frames(loaded, loaded_cfg, package_dir, _logger())
        csv_path = os.path.join(package_dir, "config", "frame_overrides.csv")
        assert os.path.isfile(csv_path)
        assert plan["base_link"]["role"] == "root_x_forward_z_up"
        assert plan["tilt_link"]["role"] == "revolute_axis_z"

        with open(csv_path, "r", encoding="utf-8", newline="") as handle:
            rows = list(csv.DictReader(handle))
        for row in rows:
            if row["link"] == "sensor_link":
                row["rule"] = "world_rpy"
                row["post_roll_deg"] = "10"
                row["post_pitch_deg"] = "20"
                row["post_yaw_deg"] = "30"
        with open(csv_path, "w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=rows[0].keys(), lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)

        loaded_again, loaded_cfg_again = load_frame_cache(cache_path)
        custom_plan = configure_frames(
            loaded_again, loaded_cfg_again, package_dir, _logger()
        )
        expected = rpy_to_matrix(tuple(math.radians(v) for v in (10, 20, 30)))
        _assert_matrix_close(custom_plan["sensor_link"]["desired_world"], expected)
        # The revolute row remained auto and therefore still becomes +Z.
        _assert_vec_close(
            loaded_again.joints["tilt_joint"].axis,
            (0.0, 0.0, 1.0),
        )

        xml = generate_urdf(loaded_again, loaded_cfg_again)
        assert 'mesh filename="package://frame_test_description/meshes/base.dae"' in xml
        assert '<axis xyz="0.000000 0.000000 1.000000"/>' in xml
        assert 'rpy="0 0 0"' not in _visual_origin_line(xml, "base_link")

        # Exercise the actual user-facing offline command path from the same
        # canonical cache. It must rewrite descriptions and leave meshes alone.
        from ..tools.reframe import reframe_package
        reframe_package(package_dir, cache_path, log=_logger())
        generated_urdf = os.path.join(package_dir, "urdf", "frame_test.urdf")
        assert os.path.isfile(generated_urdf)
        with open(generated_urdf, "r", encoding="utf-8") as handle:
            regenerated_xml = handle.read()
        assert '<axis xyz="0.000000 0.000000 1.000000"/>' in regenerated_xml


def test_real_snapshot_axes_and_geometry_if_available():
    package_root = os.path.dirname(os.path.dirname(__file__))
    snapshot_path = os.path.join(package_root, "debug", "snapshot.json")
    if not os.path.isfile(snapshot_path):
        print("  real_snapshot_frame_rebase: SKIPPED (local snapshot not found)")
        return

    snapshot = load_snapshot(snapshot_path)

    canonical = build_model(snapshot, _logger())
    rebased = copy.deepcopy(canonical)
    plan = plan_frame_rebases(rebased, convention="ros", log=_logger())
    old_root_world = tuple(
        canonical.links[canonical.root_link].source_world_rotation or IDENTITY_3
    )
    apply_frame_rebases(rebased, plan, log=_logger())

    movable = [
        joint for joint in rebased.joints.values()
        if joint.joint_type in ("revolute", "continuous")
    ]
    for joint in movable:
        _assert_vec_close(joint.axis, (0.0, 0.0, 1.0), tol=1e-7)

    samples = ({}, {
        joint.name: (0.37 if index % 2 == 0 else -0.28)
        for index, joint in enumerate(movable)
    })
    for positions in samples:
        before = _mesh_world_poses(canonical, old_root_world, positions)
        after = _mesh_world_poses(rebased, IDENTITY_3, positions)
        for name in canonical.links:
            _assert_matrix_close(after[name][0], before[name][0], tol=5e-8)
            _assert_vec_close(after[name][1], before[name][1], tol=5e-8)

    # Legacy-export bootstrap: snapshot + existing mesh files are sufficient
    # to create frame_model.json and regenerate descriptions without Fusion.
    from ..tools.reframe import reframe_package
    with tempfile.TemporaryDirectory(dir=os.getcwd()) as temp_dir:
        package_dir = os.path.join(temp_dir, f"{canonical.name}_description")
        for link in canonical.links.values():
            cached_paths = []
            if link.mesh_visual:
                cached_paths.append(os.path.splitext(link.mesh_visual)[0] + ".dae")
            if link.mesh_collision:
                cached_paths.append(link.mesh_collision)
            for relative in cached_paths:
                if not relative:
                    continue
                absolute = os.path.join(package_dir, relative)
                os.makedirs(os.path.dirname(absolute), exist_ok=True)
                with open(absolute, "wb") as handle:
                    handle.write(b"cached-mesh-placeholder")
        reframe_package(
            package_dir,
            log=_logger(),
            snapshot_path=snapshot_path,
        )
        cache = os.path.join(package_dir, "debug", "frame_model.json")
        urdf = os.path.join(package_dir, "urdf", f"{canonical.name}.urdf")
        assert os.path.isfile(cache)
        with open(urdf, "r", encoding="utf-8") as handle:
            regenerated = handle.read()
        assert regenerated.count(
            '<axis xyz="0.000000 0.000000 1.000000"/>'
        ) >= len(movable)
    print("  real_snapshot_frame_rebase: PASS")


def _inertia_matrix(inertia):
    return (
        inertia.ixx, inertia.ixy, inertia.ixz,
        inertia.ixy, inertia.iyy, inertia.iyz,
        inertia.ixz, inertia.iyz, inertia.izz,
    )


def _visual_origin_line(xml, link_name):
    marker = f'<link name="{link_name}">'
    section = xml.split(marker, 1)[1].split("</link>", 1)[0]
    visual = section.split("<visual>", 1)[1].split("</visual>", 1)[0]
    return next(line for line in visual.splitlines() if "<origin " in line)


def main():
    print("Running frame rebaser tests...")
    test_ros_rebase_preserves_mesh_fk_and_inertia()
    print("  ros_rebase_preserves_mesh_fk_and_inertia: PASS")
    test_csv_override_and_cache_round_trip()
    print("  csv_override_and_cache_round_trip: PASS")
    test_real_snapshot_axes_and_geometry_if_available()
    print("All frame rebaser tests passed.")


if __name__ == "__main__":
    main()
