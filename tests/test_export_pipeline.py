"""
Tests for Export Pipeline: URDF/Xacro generation, collision resolution,
STL processing, package structure, and validation reports.

Run from the PARENT directory:
    python -m fusion2URDF.tests.test_export_pipeline
"""

import json
import os
import sys
import tempfile
import xml.etree.ElementTree as ET

# Reuse snapshot helper from test_robot_model
from .test_robot_model import _make_snapshot, _make_logger, MINI_SNAPSHOT


# ──────────────────────────────────────────────
# URDF Generator tests
# ──────────────────────────────────────────────

def test_urdf_valid_xml():
    """Generated URDF must be valid XML."""
    from ..core.robot_model import build_model
    from ..core.xacro_generator import generate_urdf
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    config = ExportConfig(package_name="test_description")
    urdf = generate_urdf(model, config)
    
    # Must parse as XML
    root = ET.fromstring(urdf)
    assert root.tag == 'robot', f"Root tag should be 'robot', got '{root.tag}'"
    assert root.attrib['name'] == 'test_robot'
    
    print("  urdf_valid_xml: PASS")


def test_urdf_link_count():
    """URDF should have one <link> per model link."""
    from ..core.robot_model import build_model
    from ..core.xacro_generator import generate_urdf
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    config = ExportConfig(package_name="test_description")
    urdf = generate_urdf(model, config)
    root = ET.fromstring(urdf)
    
    links = root.findall('link')
    assert len(links) == 4, f"Expected 4 links, got {len(links)}"
    
    link_names = {l.attrib['name'] for l in links}
    assert link_names == {'base_link', 'link1', 'link2', 'tool'}
    
    print("  urdf_link_count: PASS")


def test_urdf_joint_count():
    """URDF should have one <joint> per model joint."""
    from ..core.robot_model import build_model
    from ..core.xacro_generator import generate_urdf
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    config = ExportConfig(package_name="test_description")
    urdf = generate_urdf(model, config)
    root = ET.fromstring(urdf)
    
    joints = root.findall('joint')
    assert len(joints) == 3, f"Expected 3 joints, got {len(joints)}"
    
    # Check types
    joint_types = {j.attrib['name']: j.attrib['type'] for j in joints}
    assert joint_types['joint1'] == 'revolute'
    assert joint_types['joint2'] == 'revolute'
    assert joint_types['joint3'] == 'fixed'
    
    print("  urdf_joint_count: PASS")


def test_urdf_inertial():
    """Each link should have valid <inertial> data."""
    from ..core.robot_model import build_model
    from ..core.xacro_generator import generate_urdf
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    config = ExportConfig(package_name="test_description")
    urdf = generate_urdf(model, config)
    root = ET.fromstring(urdf)
    
    for link_elem in root.findall('link'):
        name = link_elem.attrib['name']
        inertial = link_elem.find('inertial')
        assert inertial is not None, f"Link '{name}' missing <inertial>"
        
        mass = inertial.find('mass')
        assert mass is not None, f"Link '{name}' missing <mass>"
        mass_val = float(mass.attrib['value'])
        assert mass_val > 0, f"Link '{name}' has zero/negative mass"
        
        inertia = inertial.find('inertia')
        assert inertia is not None, f"Link '{name}' missing <inertia>"
        ixx = float(inertia.attrib['ixx'])
        assert ixx > 0, f"Link '{name}' has zero ixx"
    
    print("  urdf_inertial: PASS")


def test_urdf_joint_parent_child():
    """Joint parent/child should reference existing links."""
    from ..core.robot_model import build_model
    from ..core.xacro_generator import generate_urdf
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    config = ExportConfig(package_name="test_description")
    urdf = generate_urdf(model, config)
    root = ET.fromstring(urdf)
    
    link_names = {l.attrib['name'] for l in root.findall('link')}
    
    for joint_elem in root.findall('joint'):
        jname = joint_elem.attrib['name']
        parent = joint_elem.find('parent').attrib['link']
        child = joint_elem.find('child').attrib['link']
        
        assert parent in link_names, f"Joint '{jname}' parent '{parent}' not in links"
        assert child in link_names, f"Joint '{jname}' child '{child}' not in links"
    
    print("  urdf_joint_parent_child: PASS")


def test_urdf_revolute_has_limits():
    """Revolute joints must have <limit> for Gazebo compatibility."""
    from ..core.robot_model import build_model
    from ..core.xacro_generator import generate_urdf
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    config = ExportConfig(package_name="test_description")
    urdf = generate_urdf(model, config)
    root = ET.fromstring(urdf)
    
    for joint_elem in root.findall('joint'):
        jtype = joint_elem.attrib['type']
        jname = joint_elem.attrib['name']
        if jtype == 'revolute':
            limit = joint_elem.find('limit')
            assert limit is not None, f"Revolute joint '{jname}' missing <limit>"
            assert 'lower' in limit.attrib, f"Joint '{jname}' limit missing 'lower'"
            assert 'upper' in limit.attrib, f"Joint '{jname}' limit missing 'upper'"
    
    print("  urdf_revolute_has_limits: PASS")


def test_urdf_root_first():
    """Root link should be the first <link> in URDF."""
    from ..core.robot_model import build_model
    from ..core.xacro_generator import generate_urdf
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    config = ExportConfig(package_name="test_description")
    urdf = generate_urdf(model, config)
    root = ET.fromstring(urdf)
    
    first_link = root.findall('link')[0]
    assert first_link.attrib['name'] == 'base_link', \
        f"First link should be root 'base_link', got '{first_link.attrib['name']}'"
    
    print("  urdf_root_first: PASS")


# ──────────────────────────────────────────────
# Collision generator tests
# ──────────────────────────────────────────────

def test_collision_primitive_box():
    """Roughly cubic bbox without volume data should produce a box."""
    from ..core.collision_generator import fit_primitive
    
    prim = fit_primitive(bbox_size=(0.1, 0.12, 0.11))
    assert prim is not None
    assert prim.shape == "box", f"Expected box for near-cubic bbox, got {prim.shape}"
    
    print("  collision_primitive_box: PASS")


def test_collision_primitive_cylinder():
    """Elongated bbox (rod pattern) should produce a cylinder."""
    from ..core.collision_generator import fit_primitive
    
    # 15:1 aspect ratio — clearly elongated rod
    prim = fit_primitive(bbox_size=(0.02, 0.02, 0.30))
    assert prim is not None
    assert prim.shape == "cylinder", f"Expected cylinder for elongated bbox, got {prim.shape}"
    # Cylinder should be along Z (longest axis)
    assert abs(prim.origin_rpy[0]) < 0.01 and abs(prim.origin_rpy[1]) < 0.01, \
        "Cylinder should be along Z axis (no rotation needed)"
    
    print("  collision_primitive_cylinder: PASS")


def test_collision_primitive_cylinder_rotated():
    """Elongated along X should produce rotated cylinder."""
    from ..core.collision_generator import fit_primitive
    import math
    
    prim = fit_primitive(bbox_size=(0.30, 0.02, 0.02))
    assert prim is not None
    assert prim.shape == "cylinder"
    # Should be rotated to align with X
    assert abs(prim.origin_rpy[1] - math.pi / 2) < 0.01, \
        f"Cylinder along X should have pitch=π/2, got {prim.origin_rpy}"
    
    print("  collision_primitive_cylinder_rotated: PASS")


def test_collision_oriented_box_from_obj_vertices():
    """A diagonal rectangular beam should fit an oriented box, not a world
    axis-aligned box."""
    from ..core.collision_generator import fit_primitive, _primitive_to_triangles
    import math

    obj_path = os.path.join(os.getcwd(), "_oriented_box_fit.obj")
    length, width, height = 0.40, 0.04, 0.02
    yaw = math.radians(35.0)
    cy, sy = math.cos(yaw), math.sin(yaw)

    local = [
        (sx * length / 2.0, sy_ * width / 2.0, sz * height / 2.0)
        for sx in (-1, 1)
        for sy_ in (-1, 1)
        for sz in (-1, 1)
    ]
    world = [
        (cy * x - sy * y, sy * x + cy * y, z)
        for x, y, z in local
    ]
    bbox_min = tuple(min(p[i] for p in world) for i in range(3))
    bbox_max = tuple(max(p[i] for p in world) for i in range(3))
    bbox_size = tuple(bbox_max[i] - bbox_min[i] for i in range(3))

    try:
        with open(obj_path, "w", encoding="utf-8") as f:
            for x, y, z in world:
                f.write(f"v {x * 100.0:.8f} {y * 100.0:.8f} {z * 100.0:.8f}\n")

        prim = fit_primitive(
            bbox_size=bbox_size,
            bbox_min=bbox_min,
            bbox_max=bbox_max,
            obj_path=obj_path,
        )
    finally:
        if os.path.exists(obj_path):
            os.remove(obj_path)

    assert prim is not None
    assert prim.shape == "box", f"Expected oriented box, got {prim.shape}"
    assert prim.orientation_matrix is not None, "Expected PCA orientation matrix"
    prim_vol = prim.size[0] * prim.size[1] * prim.size[2]
    bbox_vol = bbox_size[0] * bbox_size[1] * bbox_size[2]
    assert prim_vol < bbox_vol * 0.30, (
        f"Oriented fit should be much tighter than world AABB: "
        f"size={prim.size}, bbox={bbox_size}"
    )
    tris = _primitive_to_triangles(prim)
    tri_points = [p for tri in tris for p in tri]
    tri_bbox = tuple(
        (max(p[i] for p in tri_points) - min(p[i] for p in tri_points)) / 100.0
        for i in range(3)
    )
    assert tri_bbox[0] > prim.size[1] * 3.0 and tri_bbox[1] > prim.size[1] * 3.0, (
        f"Generated STL should be rotated in link frame; got bbox {tri_bbox}"
    )

    print("  collision_oriented_box_from_obj_vertices: PASS")


def test_collision_oriented_box_rejects_ambiguous_square_footprint():
    """Square-ish meshes should not get arbitrary PCA yaw."""
    from ..core.collision_generator import fit_primitive
    import math

    length, width, height = 0.20, 0.20, 0.04
    yaw = math.radians(45.0)
    cy, sy = math.cos(yaw), math.sin(yaw)
    local = [
        (sx * length / 2.0, sy_ * width / 2.0, sz * height / 2.0)
        for sx in (-1, 1)
        for sy_ in (-1, 1)
        for sz in (-1, 1)
    ]
    world = [
        (cy * x - sy * y, sy * x + cy * y, z)
        for x, y, z in local
    ]
    bbox_min = tuple(min(p[i] for p in world) for i in range(3))
    bbox_max = tuple(max(p[i] for p in world) for i in range(3))
    bbox_size = tuple(bbox_max[i] - bbox_min[i] for i in range(3))

    with tempfile.TemporaryDirectory() as tmp:
        obj_path = os.path.join(tmp, "square_body.obj")
        with open(obj_path, "w", encoding="utf-8") as f:
            for x, y, z in world:
                f.write(f"v {x * 100.0:.8f} {y * 100.0:.8f} {z * 100.0:.8f}\n")

        prim = fit_primitive(
            bbox_size=bbox_size,
            bbox_min=bbox_min,
            bbox_max=bbox_max,
            volume=length * width * height,
            obj_path=obj_path,
        )

    assert prim is not None
    assert prim.shape == "box"
    assert prim.orientation_matrix is None, (
        "PCA should stay axis-aligned when the dominant footprint axes are ambiguous"
    )

    print("  collision_oriented_box_rejects_ambiguous_square_footprint: PASS")


def test_collision_oriented_box_prefers_mesh_over_inertia_frame():
    """Collision orientation follows mesh geometry, not mass properties."""
    from ..core.collision_generator import fit_primitive
    from ..core.data_types import InertiaTensor
    import math

    length, width, height = 0.22, 0.18, 0.04
    yaw = math.radians(45.0)
    cy, sy = math.cos(yaw), math.sin(yaw)
    local = [
        (sx * length / 2.0, sy_ * width / 2.0, sz * height / 2.0)
        for sx in (-1, 1)
        for sy_ in (-1, 1)
        for sz in (-1, 1)
    ]
    world = [
        (cy * x - sy * y, sy * x + cy * y, z)
        for x, y, z in local
    ]
    bbox_min = tuple(min(p[i] for p in world) for i in range(3))
    bbox_max = tuple(max(p[i] for p in world) for i in range(3))
    bbox_size = tuple(bbox_max[i] - bbox_min[i] for i in range(3))

    inertia_axis_aligned = InertiaTensor(
        ixx=(width * width + height * height) / 12.0,
        iyy=(length * length + height * height) / 12.0,
        izz=(length * length + width * width) / 12.0,
    )

    with tempfile.TemporaryDirectory() as tmp:
        obj_path = os.path.join(tmp, "broad_body.obj")
        with open(obj_path, "w", encoding="utf-8") as f:
            for x, y, z in world:
                f.write(f"v {x * 100.0:.8f} {y * 100.0:.8f} {z * 100.0:.8f}\n")

        pca_prim = fit_primitive(
            bbox_size=bbox_size,
            bbox_min=bbox_min,
            bbox_max=bbox_max,
            volume=length * width * height,
            obj_path=obj_path,
        )
        mesh_prim = fit_primitive(
            bbox_size=bbox_size,
            bbox_min=bbox_min,
            bbox_max=bbox_max,
            volume=length * width * height,
            obj_path=obj_path,
            inertia_at_com=inertia_axis_aligned,
        )

    assert pca_prim is not None
    assert pca_prim.orientation_matrix is not None, (
        "Without inertia, this deliberately skewed body should use PCA"
    )
    assert mesh_prim is not None
    assert mesh_prim.shape == "box"
    assert mesh_prim.orientation_matrix is not None, (
        "Collision OBB should follow the visual mesh even when inertia "
        "suggests an axis-aligned frame"
    )
    mesh_vol = mesh_prim.size[0] * mesh_prim.size[1] * mesh_prim.size[2]
    aabb_vol = bbox_size[0] * bbox_size[1] * bbox_size[2]
    assert mesh_vol < aabb_vol * 0.60, (
        f"Mesh-aligned OBB should be tighter than the axis-aligned bbox: "
        f"mesh={mesh_prim.size}, bbox={bbox_size}"
    )

    print("  collision_oriented_box_prefers_mesh_over_inertia_frame: PASS")


def test_convex_hull_tetrahedron_triangles():
    """The dependency-free hull builder handles a minimal 3D hull."""
    from ..core.collision_generator import _convex_hull_triangles

    points = [
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (0.0, 10.0, 0.0),
        (0.0, 0.0, 10.0),
    ]
    tris = _convex_hull_triangles(points)
    assert len(tris) == 4, f"tetrahedron hull should have 4 faces, got {len(tris)}"

    print("  convex_hull_tetrahedron_triangles: PASS")


def test_convex_hull_obj_reads_late_extreme_vertices():
    """Convex hull OBJ sampling must not depend on vertex order.

    Fusion can write mirrored copies with different vertex ordering.  The
    old reader stopped after the first 20k vertices, so late outer tire
    vertices could be missed even though the visual mesh was correct.
    """
    from ..core.collision_generator import _convex_hull_triangles_from_obj

    def cube_points(radius):
        return [
            (sx * radius, sy * radius, sz * radius)
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ]

    with tempfile.TemporaryDirectory() as tmp:
        obj_path = os.path.join(tmp, "late_extremes.obj")
        inner = cube_points(1.0)
        outer = cube_points(10.0)
        with open(obj_path, "w", encoding="utf-8") as f:
            for _ in range(2601):  # 20,808 inner vertices; old limit was 20,000
                for x, y, z in inner:
                    f.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
            for x, y, z in outer:
                f.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")

        tris = _convex_hull_triangles_from_obj(obj_path)

    points = [p for tri in tris for p in tri]
    assert points, "Expected convex hull triangles"
    mins = tuple(min(p[i] for p in points) for i in range(3))
    maxs = tuple(max(p[i] for p in points) for i in range(3))
    sizes = tuple(maxs[i] - mins[i] for i in range(3))
    assert all(size >= 19.9 for size in sizes), (
        f"Hull should include late outer vertices; got bbox sizes {sizes}"
    )

    print("  convex_hull_obj_reads_late_extreme_vertices: PASS")


def test_collision_excluded_body_uses_filtered_collision_input():
    """Body-level ! visual details should not inflate generated collision."""
    from ..core.collision_generator import (
        resolve_collision, generate_collision_meshes, _stl_bbox_size_cm,
    )
    from ..core.data_types import ExportConfig, LinkNode, RobotModel

    model = RobotModel(name="detailbot", root_link="base_link")
    link = LinkNode(
        urdf_name="base_link",
        clean_name="base_link",
        body_count=2,
        mesh_visual="meshes/Model/base_link.obj",
        mesh_collision="meshes/Model/base_link_collision.stl",
        mesh_collision_input="meshes/Model/base_link_collision_input.obj",
        bbox_size=(1.0, 0.10, 0.10),
        bbox_min=(-0.50, -0.05, -0.05),
        bbox_max=(0.50, 0.05, 0.05),
        volume_m3=0.010,
        has_collision_exclusions=True,
        collision_body_count=1,
        collision_excluded_body_names=["!antenna"],
        collision_bbox_size=(0.10, 0.10, 0.10),
        collision_bbox_min=(-0.05, -0.05, -0.05),
        collision_bbox_max=(0.05, 0.05, 0.05),
        collision_volume_m3=0.001,
    )
    model.links["base_link"] = link

    with tempfile.TemporaryDirectory() as tmp:
        pkg_dir = os.path.join(tmp, "detailbot_description")
        obj_dir = os.path.join(pkg_dir, "meshes", "Model")
        os.makedirs(obj_dir)
        filtered_obj = os.path.join(obj_dir, "base_link_collision_input.obj")
        with open(filtered_obj, "w", encoding="utf-8") as f:
            for sx in (-5.0, 5.0):
                for sy in (-5.0, 5.0):
                    for sz in (-5.0, 5.0):
                        f.write(f"v {sx:.6f} {sy:.6f} {sz:.6f}\n")

        config = ExportConfig(
            package_name="detailbot_description",
            collision_auto_method="primitive",
        )
        resolve_collision(model, config, _make_logger(), pkg_dir=pkg_dir)
        assert link.collision is not None
        assert link.collision.source == "primitive"
        assert max(link.collision.primitive.size) <= 0.101, (
            f"Primitive should use filtered bbox, got {link.collision.primitive.size}"
        )

        config.collision_auto_method = "convex_hull"
        resolve_collision(model, config, _make_logger(), pkg_dir=pkg_dir)
        assert link.collision.source == "convex_hull"
        generate_collision_meshes(model, pkg_dir, _make_logger())
        stl_bbox = _stl_bbox_size_cm(os.path.join(pkg_dir, link.mesh_collision))
        assert max(stl_bbox) <= 10.1, (
            f"Convex hull should use filtered OBJ, got bbox cm {stl_bbox}"
        )

    print("  collision_excluded_body_uses_filtered_collision_input: PASS")


def test_collision_excluded_only_link_has_no_collision():
    """A link made only from ! visual detail should stay visual-only."""
    from ..core.collision_generator import resolve_collision
    from ..core.data_types import ExportConfig, LinkNode, RobotModel

    model = RobotModel(name="detailbot", root_link="antenna_link")
    link = LinkNode(
        urdf_name="antenna_link",
        clean_name="antenna_link",
        body_count=1,
        mesh_visual="meshes/Model/antenna_link.obj",
        mesh_collision="meshes/Model/antenna_link_collision.stl",
        bbox_size=(1.0, 0.01, 0.01),
        bbox_min=(0.0, -0.005, -0.005),
        bbox_max=(1.0, 0.005, 0.005),
        volume_m3=0.0001,
        has_collision_exclusions=True,
        collision_body_count=0,
        collision_excluded_body_names=["!antenna"],
    )
    model.links["antenna_link"] = link

    config = ExportConfig(
        package_name="detailbot_description",
        collision_auto_method="primitive",
    )
    resolve_collision(model, config, _make_logger())
    assert link.collision is None

    print("  collision_excluded_only_link_has_no_collision: PASS")


def test_square_beam_obj_is_box_not_cylinder():
    """A square-section beam with only corner-like OBJ vertices is boxy, even
    though its two short dimensions are equal."""
    from ..core.collision_generator import fit_primitive

    obj_path = os.path.join(os.getcwd(), "_square_beam_fit.obj")
    length, width, height = 0.40, 0.04, 0.04
    vertices = [
        (sx * length / 2.0, sy * width / 2.0, sz * height / 2.0)
        for sx in (-1, 1)
        for sy in (-1, 1)
        for sz in (-1, 1)
    ]

    try:
        with open(obj_path, "w", encoding="utf-8") as f:
            for x, y, z in vertices:
                f.write(f"v {x * 100.0:.8f} {y * 100.0:.8f} {z * 100.0:.8f}\n")

        prim = fit_primitive(
            bbox_size=(length, width, height),
            bbox_min=(-length / 2.0, -width / 2.0, -height / 2.0),
            bbox_max=(length / 2.0, width / 2.0, height / 2.0),
            obj_path=obj_path,
        )
    finally:
        if os.path.exists(obj_path):
            os.remove(obj_path)

    assert prim is not None
    assert prim.shape == "box", f"Expected square beam to stay box, got {prim.shape}"

    print("  square_beam_obj_is_box_not_cylinder: PASS")


def test_collision_primitive_sphere():
    """Equal dimensions + low volume fill → sphere."""
    from ..core.collision_generator import fit_primitive
    import math
    
    # Perfect sphere: volume = (4/3)π r³ ≈ 0.000524 for r=0.05 (bbox 0.1³)
    # Fill ratio ≈ π/6 ≈ 0.524
    sphere_vol = (4.0 / 3.0) * math.pi * 0.05**3
    prim = fit_primitive(
        bbox_size=(0.10, 0.10, 0.10),
        volume=sphere_vol,
    )
    assert prim is not None
    assert prim.shape == "sphere", f"Expected sphere, got {prim.shape}"
    
    print("  collision_primitive_sphere: PASS")


def test_collision_primitive_zero_bbox():
    """Zero bbox should return None."""
    from ..core.collision_generator import fit_primitive
    
    prim = fit_primitive(bbox_size=(0.0, 0.0, 0.0))
    assert prim is None
    
    print("  collision_primitive_zero_bbox: PASS")


def test_collision_plate_is_box():
    """Flat plate (two large, one thin) should be box, not cylinder."""
    from ..core.collision_generator import fit_primitive
    
    # Plate: 0.584 x 0.488 x 0.005 — real data from rail_mount_plate
    prim = fit_primitive(
        bbox_size=(0.584, 0.488, 0.005),
        volume=0.001399,  # fill ≈ 0.98
    )
    assert prim is not None
    assert prim.shape == "box", f"Expected box for plate, got {prim.shape}"
    
    print("  collision_plate_is_box: PASS")


def test_collision_wheel_is_cylinder():
    """Wheel (disc shape, low fill) should be cylinder with correct axis."""
    from ..core.collision_generator import fit_primitive
    import math
    
    # Wheel: 0.37 x 0.151 x 0.37 — real data from rr_wheel_link
    prim = fit_primitive(
        bbox_size=(0.37, 0.151424, 0.37),
        volume=0.0151906,  # fill ≈ 0.73
    )
    assert prim is not None
    assert prim.shape == "cylinder", f"Expected cylinder for wheel, got {prim.shape}"
    # Axis should be Y (the thin dimension) → RPY = (π/2, 0, 0)
    assert abs(prim.origin_rpy[0] - math.pi / 2) < 0.01, \
        f"Wheel cylinder should have roll=π/2 (axis Y), got {prim.origin_rpy}"
    # Radius should be ~ 0.37/2 = 0.185
    assert abs(prim.size[0] - 0.185) < 0.01, \
        f"Wheel radius should be ~0.185, got {prim.size[0]}"
    
    print("  collision_wheel_is_cylinder: PASS")


def test_collision_revolute_joint_hint():
    """Revolute joint should force cylinder along joint axis."""
    from ..core.collision_generator import fit_primitive
    from ..core.data_types import JointNode
    import math
    
    # Ambiguous bbox but revolute joint along Y
    joint = JointNode(
        name="wheel_joint", joint_type="revolute",
        parent_link="base", child_link="wheel",
        origin_xyz=(0, 0, 0), origin_rpy=(0, 0, 0),
        axis=(0, 1, 0),
    )
    prim = fit_primitive(
        bbox_size=(0.2, 0.15, 0.2),
        volume=0.003,  # moderate fill
        parent_joint=joint,
    )
    assert prim is not None
    assert prim.shape == "cylinder", f"Revolute child should be cylinder, got {prim.shape}"
    # Should be along Y (joint axis)
    assert abs(prim.origin_rpy[0] - math.pi / 2) < 0.01, \
        f"Cylinder should align with Y axis, got {prim.origin_rpy}"
    
    print("  collision_revolute_joint_hint: PASS")


def test_collision_bbox_center_origin():
    """Collision origin should be bbox center, not (0,0,0)."""
    from ..core.collision_generator import fit_primitive
    
    prim = fit_primitive(
        bbox_size=(0.1, 0.1, 0.1),
        bbox_min=(-0.05, 0.0, -0.05),
        bbox_max=(0.05, 0.1, 0.05),
    )
    assert prim is not None
    # Origin should be (0.0, 0.05, 0.0) — center of bbox
    assert abs(prim.origin_xyz[1] - 0.05) < 0.001, \
        f"Origin Y should be bbox center 0.05, got {prim.origin_xyz[1]}"
    
    print("  collision_bbox_center_origin: PASS")


def test_collision_resolve_model():
    """Resolve collision for all links in a model."""
    from ..core.robot_model import build_model
    from ..core.collision_generator import resolve_collision
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    config = ExportConfig(collision_auto_method="primitive")
    resolve_collision(model, config, log)
    
    # All links should have collision resolved
    for name, link in model.links.items():
        assert link.collision is not None, f"Link '{name}' has no collision"
        assert link.collision.source in ("explicit", "primitive", "convex_hull", "visual_reuse", "visual_fallback"), \
            f"Link '{name}' unexpected collision source: {link.collision.source}"
    
    print("  collision_resolve_model: PASS")


def test_collision_primitive_in_urdf():
    """Primitive collision should generate collision STL mesh reference in URDF."""
    from ..core.robot_model import build_model
    from ..core.collision_generator import resolve_collision
    from ..core.xacro_generator import generate_urdf
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    config = ExportConfig(package_name="test_description", collision_auto_method="primitive")
    resolve_collision(model, config, log)
    
    # After resolve, primitive links should have mesh_path set on collision
    has_collision_mesh = False
    for name, link in model.links.items():
        if link.collision and link.collision.source == "primitive":
            # mesh_path gets set after generate_collision_meshes, but primitive should exist
            assert link.collision.primitive is not None, f"{name} should have primitive"
            has_collision_mesh = True
    
    assert has_collision_mesh, "Expected at least one link with primitive collision"
    
    # Generate URDF — collision should reference mesh files (after STL gen sets mesh_path)
    # Simulate what generate_collision_meshes does: set mesh_path
    for name, link in model.links.items():
        if link.collision and link.collision.source == "primitive":
            link.collision.mesh_path = link.mesh_collision
    
    urdf = generate_urdf(model, config)
    root = ET.fromstring(urdf)
    
    # All collision elements should use mesh references
    for link_elem in root.findall('link'):
        collision = link_elem.find('collision')
        if collision is not None:
            geom = collision.find('geometry')
            if geom is not None:
                mesh = geom.find('mesh')
                assert mesh is not None, f"Link '{link_elem.get('name')}' collision should use mesh, not inline primitive"
    
    print("  collision_primitive_in_urdf: PASS")


def test_collision_stl_files_generated():
    """Collision STL files should be generated for primitive collision links."""
    import tempfile
    from ..core.robot_model import build_model
    from ..core.collision_generator import resolve_collision, generate_collision_meshes
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    config = ExportConfig(package_name="test_description", collision_auto_method="primitive")
    resolve_collision(model, config, log)
    
    with tempfile.TemporaryDirectory() as tmpdir:
        pkg_dir = os.path.join(tmpdir, "test_description")
        os.makedirs(pkg_dir)
        generate_collision_meshes(model, pkg_dir, log)
        
        # Check STL files exist
        for name, link in model.links.items():
            if link.collision and link.collision.source == "primitive":
                stl_path = os.path.join(pkg_dir, link.mesh_collision)
                assert os.path.exists(stl_path), f"Missing collision STL: {stl_path}"
                # Verify binary STL: 80-byte header + 4-byte count + N*50-byte triangles
                size = os.path.getsize(stl_path)
                assert size > 84, f"STL too small ({size} bytes): {stl_path}"
                assert (size - 84) % 50 == 0, f"Invalid STL size ({size} bytes): {stl_path}"
                # mesh_path should be set
                assert link.collision.mesh_path == link.mesh_collision
    
    print("  collision_stl_files_generated: PASS")


def test_convex_hull_collision_stl_files_generated():
    """convex_hull collision mode should emit a generated STL mesh."""
    from ..core.collision_generator import resolve_collision, generate_collision_meshes
    from ..core.data_types import ExportConfig, LinkNode, RobotModel

    model = RobotModel(name="hullbot", root_link="body")
    link = LinkNode(
        urdf_name="body",
        clean_name="body",
        body_count=1,
        mesh_visual="meshes/body/body.obj",
        mesh_collision="meshes/body/body_collision.stl",
        bbox_size=(0.10, 0.08, 0.06),
        bbox_min=(-0.05, -0.04, -0.03),
        bbox_max=(0.05, 0.04, 0.03),
        volume_m3=0.00048,
    )
    model.links["body"] = link

    with tempfile.TemporaryDirectory() as tmp:
        pkg_dir = os.path.join(tmp, "hullbot_description")
        obj_dir = os.path.join(pkg_dir, "meshes", "body")
        os.makedirs(obj_dir)
        obj_path = os.path.join(obj_dir, "body.obj")
        vertices = [
            (-5.0, -4.0, -3.0), (5.0, -4.0, -3.0),
            (5.0, 4.0, -3.0), (-5.0, 4.0, -3.0),
            (-5.0, -4.0, 3.0), (5.0, -4.0, 3.0),
            (5.0, 4.0, 3.0), (-5.0, 4.0, 3.0),
        ]
        with open(obj_path, "w", encoding="utf-8") as f:
            for x, y, z in vertices:
                f.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")

        log = _make_logger()
        config = ExportConfig(
            package_name="hullbot_description",
            collision_auto_method="convex_hull",
        )
        resolve_collision(model, config, log, pkg_dir=pkg_dir)
        assert link.collision is not None
        assert link.collision.source == "convex_hull"

        generate_collision_meshes(model, pkg_dir, log)

        stl_path = os.path.join(pkg_dir, link.mesh_collision)
        assert os.path.exists(stl_path), f"Missing convex hull STL: {stl_path}"
        size = os.path.getsize(stl_path)
        assert size > 84 and (size - 84) % 50 == 0, f"Invalid STL size: {size}"
        assert link.collision.mesh_path == link.mesh_collision
        assert link.collision.source == "convex_hull"

    print("  convex_hull_collision_stl_files_generated: PASS")


def test_bake_offset_in_xacro():
    """Links with mesh_bake_offset should have shifted visual/inertial origins."""
    from ..core.robot_model import build_model
    from ..core.collision_generator import resolve_collision
    from ..core.xacro_generator import generate_urdf
    from ..core.data_types import ExportConfig, LinkNode
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    # Manually set a bake offset on link1 to simulate Elevation-like scenario
    link1 = model.links.get("link1")
    assert link1, "Expected link1 in model"
    link1.needs_mesh_bake = True
    link1.mesh_bake_offset = (0.0, -0.1, 0.0)  # 100mm Y offset
    
    config = ExportConfig(package_name="test_description", collision_auto_method="primitive")
    resolve_collision(model, config, log)
    # Set mesh_path for collision (simulate generate_collision_meshes)
    for name, link in model.links.items():
        if link.collision and link.collision.source == "primitive":
            link.collision.mesh_path = link.mesh_collision
    
    urdf = generate_urdf(model, config)
    root = ET.fromstring(urdf)
    
    # Find link1 and check origins
    link1_elem = None
    for le in root.findall('link'):
        if le.get('name') == 'link1':
            link1_elem = le
            break
    assert link1_elem is not None, "link1 not found in URDF"
    
    # Visual origin should have the bake offset
    visual = link1_elem.find('visual')
    vis_origin = visual.find('origin')
    vis_xyz = vis_origin.get('xyz').split()
    assert abs(float(vis_xyz[1]) - (-0.1)) < 1e-4, \
        f"Visual Y origin should be -0.1, got {vis_xyz[1]}"
    
    # Inertial origin should be shifted too
    inertial = link1_elem.find('inertial')
    inert_origin = inertial.find('origin')
    inert_xyz = inert_origin.get('xyz').split()
    # Original com_link_local Y + bake Y offset
    expected_y = link1.com_link_local[1] + (-0.1)
    assert abs(float(inert_xyz[1]) - expected_y) < 1e-4, \
        f"Inertial Y origin should be {expected_y}, got {inert_xyz[1]}"
    
    print("  bake_offset_in_xacro: PASS")


# ──────────────────────────────────────────────
# Package generator tests
# ──────────────────────────────────────────────

def test_package_structure():
    """Generate package and verify directory structure."""
    from ..core.robot_model import build_model
    from ..core.package_generator import generate_package
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    with tempfile.TemporaryDirectory() as tmpdir:
        config = ExportConfig(
            package_name="test_robot_description",
            output_dir=tmpdir,
            collision_auto_method="primitive",
        )
        
        pkg_dir = generate_package(model, config, log)
        
        # Verify structure
        assert os.path.isdir(pkg_dir)
        assert os.path.isfile(os.path.join(pkg_dir, "package.xml"))
        assert os.path.isfile(os.path.join(pkg_dir, "CMakeLists.txt"))
        assert os.path.isfile(os.path.join(pkg_dir, "urdf", "test_robot.urdf"))
        assert os.path.isfile(os.path.join(pkg_dir, "urdf", "test_robot.urdf.xacro"))
        assert os.path.isdir(os.path.join(pkg_dir, "urdf", "assemblies"))
        assert os.path.isfile(os.path.join(pkg_dir, "launch", "display.launch.py"))
        assert os.path.isfile(os.path.join(pkg_dir, "rviz", "display.rviz"))
        assert os.path.isfile(os.path.join(pkg_dir, "config", "joint_state.yaml"))
        assert os.path.isfile(os.path.join(pkg_dir, "config", "frame_overrides.csv"))
        assert os.path.isfile(os.path.join(pkg_dir, "config", "FRAME_OVERRIDES.md"))
        assert os.path.isfile(os.path.join(tmpdir, "debug", "frame_model.json"))
        assert os.path.isdir(os.path.join(pkg_dir, "meshes"))
    
    print("  package_structure: PASS")


def test_package_urdf_parseable():
    """Generated URDF file should be parseable XML."""
    from ..core.robot_model import build_model
    from ..core.package_generator import generate_package
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    with tempfile.TemporaryDirectory() as tmpdir:
        config = ExportConfig(
            package_name="test_robot_description",
            output_dir=tmpdir,
            collision_auto_method="primitive",
        )
        
        pkg_dir = generate_package(model, config, log)
        urdf_path = os.path.join(pkg_dir, "urdf", "test_robot.urdf")
        
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        assert root.tag == 'robot'
        assert root.attrib['name'] == 'test_robot'
    
    print("  package_urdf_parseable: PASS")


def test_package_xml_valid():
    """Generated package.xml should be valid XML with required fields."""
    from ..core.robot_model import build_model
    from ..core.package_generator import generate_package
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    with tempfile.TemporaryDirectory() as tmpdir:
        config = ExportConfig(
            package_name="test_robot_description",
            output_dir=tmpdir,
            collision_auto_method="primitive",
        )
        
        pkg_dir = generate_package(model, config, log)
        pkg_xml_path = os.path.join(pkg_dir, "package.xml")
        
        tree = ET.parse(pkg_xml_path)
        root = tree.getroot()
        assert root.tag == 'package'
        
        # Must have required ROS2 package.xml fields
        name_elem = root.find('name')
        assert name_elem is not None, "Missing <name> in package.xml"
        assert name_elem.text == "test_robot_description"
        
        version = root.find('version')
        assert version is not None, "Missing <version> in package.xml"
    
    print("  package_xml_valid: PASS")


def test_ros2_control_generated_package():
    """Default package export should include generic ros2_control scaffolding."""
    from ..core.robot_model import build_model
    from ..core.package_generator import generate_package
    from ..core.data_types import ExportConfig

    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)

    with tempfile.TemporaryDirectory() as tmpdir:
        config = ExportConfig(
            package_name="test_robot_description",
            output_dir=tmpdir,
            collision_auto_method="primitive",
        )
        pkg_dir = generate_package(model, config, log)

        top_xacro = os.path.join(pkg_dir, "urdf", "test_robot.urdf.xacro")
        top_root = ET.parse(top_xacro).getroot()
        assert top_root.find("ros2_control") is None, (
            "assembly-owned movable joints should put ros2_control in "
            "their assembly macro, not the top-level xacro"
        )

        asm_xacro = os.path.join(pkg_dir, "urdf", "assemblies", "arm.urdf.xacro")
        root = ET.parse(asm_xacro).getroot()
        control = root.find(".//ros2_control")
        assert control is not None, "assembly xacro should include ros2_control"
        assert control.attrib["name"] == "${prefix}arm_system"

        plugin = control.find("hardware/plugin")
        assert plugin is not None
        assert plugin.text == "mock_components/GenericSystem"
        assert control.find("hardware/param[@name='cmd_topic']").text == "sim/arm_system/cmd"
        assert control.find("hardware/param[@name='state_topic']").text == "sim/arm_system/state"

        control_joints = {
            elem.attrib["name"]: elem for elem in control.findall("joint")
        }
        assert set(control_joints) == {"${prefix}joint1", "${prefix}joint2"}
        assert "${prefix}joint3" not in control_joints
        for elem in control_joints.values():
            commands = [c.attrib["name"] for c in elem.findall("command_interface")]
            states = [s.attrib["name"] for s in elem.findall("state_interface")]
            assert commands == ["position", "velocity"]
            assert states == ["position", "velocity", "effort"]

        controller_yaml = os.path.join(pkg_dir, "config", "ros2_controllers.yaml")
        assert os.path.isfile(controller_yaml)
        with open(controller_yaml, encoding="utf-8") as f:
            yaml_text = f.read()
        assert "joint_state_broadcaster/JointStateBroadcaster" in yaml_text
        assert "forward_command_controller/ForwardCommandController" in yaml_text
        assert "position_controller:" in yaml_text
        assert "velocity_controller:" in yaml_text
        assert "# Command topic: /position_controller/commands" in yaml_text
        assert "joints: [joint1, joint2]" in yaml_text

        with open(os.path.join(pkg_dir, "launch", "display.launch.py"), encoding="utf-8") as f:
            launch_text = f.read()
        assert "ros2_control_node" in launch_text
        assert "spawn_joint_state_broadcaster" in launch_text
        assert "spawn_position_controller" in launch_text
        assert "joint_state_publisher_gui" not in launch_text

    print("  ros2_control_generated_package: PASS")


def test_ros2_control_can_be_disabled():
    """TOML/config toggle should remove ros2_control output."""
    from ..core.robot_model import build_model
    from ..core.package_generator import generate_package
    from ..core.data_types import ExportConfig

    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)

    with tempfile.TemporaryDirectory() as tmpdir:
        config = ExportConfig(
            package_name="test_robot_description",
            output_dir=tmpdir,
            collision_auto_method="primitive",
            include_ros2_control=False,
        )
        pkg_dir = generate_package(model, config, log)

        top_xacro = os.path.join(pkg_dir, "urdf", "test_robot.urdf.xacro")
        assert "ros2_control" not in open(top_xacro, encoding="utf-8").read()
        assert not os.path.exists(
            os.path.join(pkg_dir, "config", "ros2_controllers.yaml")
        )
        with open(os.path.join(pkg_dir, "launch", "display.launch.py"), encoding="utf-8") as f:
            launch_text = f.read()
        assert "joint_state_publisher_gui" in launch_text
        assert "ros2_control_node" not in launch_text

    print("  ros2_control_can_be_disabled: PASS")


def test_ros2_control_passive_joint_is_state_only():
    """Passive movable joints publish state but receive no commands."""
    from ..core.robot_model import build_model
    from ..core.xacro_generator import generate_urdf
    from ..core.data_types import ExportConfig

    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    model.joints["joint1"].is_passive = True

    urdf = generate_urdf(model, ExportConfig(package_name="test_description"))
    root = ET.fromstring(urdf)
    control = root.find("ros2_control")
    assert control is not None
    joint1 = next(j for j in control.findall("joint") if j.attrib["name"] == "joint1")
    joint2 = next(j for j in control.findall("joint") if j.attrib["name"] == "joint2")
    assert not joint1.findall("command_interface")
    assert joint1.findall("state_interface")
    assert joint2.findall("command_interface")

    print("  ros2_control_passive_joint_is_state_only: PASS")


def test_cmake_has_project():
    """CMakeLists.txt should have project() with correct name."""
    from ..core.robot_model import build_model
    from ..core.package_generator import generate_package
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    with tempfile.TemporaryDirectory() as tmpdir:
        config = ExportConfig(
            package_name="test_robot_description",
            output_dir=tmpdir,
            collision_auto_method="primitive",
        )
        
        pkg_dir = generate_package(model, config, log)
        
        with open(os.path.join(pkg_dir, "CMakeLists.txt")) as f:
            cmake = f.read()
        
        assert "project(test_robot_description)" in cmake
        assert "ament_cmake" in cmake
        assert "install(DIRECTORY" in cmake
    
    print("  cmake_has_project: PASS")


# ──────────────────────────────────────────────
# Xacro generator tests
# ──────────────────────────────────────────────

def test_xacro_top_level_exists():
    """Package should contain top-level xacro file."""
    from ..core.robot_model import build_model
    from ..core.package_generator import generate_package
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    with tempfile.TemporaryDirectory() as tmpdir:
        config = ExportConfig(
            package_name="test_robot_description",
            output_dir=tmpdir,
            collision_auto_method="primitive",
        )
        
        pkg_dir = generate_package(model, config, log)
        
        xacro_path = os.path.join(pkg_dir, "urdf", "test_robot.urdf.xacro")
        assert os.path.isfile(xacro_path), f"Top-level xacro not found: {xacro_path}"
        
        # Must be valid XML
        tree = ET.parse(xacro_path)
        root = tree.getroot()
        assert root.tag == 'robot'
    
    print("  xacro_top_level_exists: PASS")


def test_xacro_assembly_files():
    """Each assembly should have its own xacro file."""
    from ..core.robot_model import build_model
    from ..core.package_generator import generate_package
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    with tempfile.TemporaryDirectory() as tmpdir:
        config = ExportConfig(
            package_name="test_robot_description",
            output_dir=tmpdir,
            collision_auto_method="primitive",
        )
        
        pkg_dir = generate_package(model, config, log)
        
        for asm_name in model.assemblies:
            asm_path = os.path.join(pkg_dir, "urdf", "assemblies", f"{asm_name}.urdf.xacro")
            assert os.path.isfile(asm_path), f"Assembly xacro not found: {asm_path}"
            
            # Must be valid XML with macro definition
            tree = ET.parse(asm_path)
            root = tree.getroot()
            assert root.tag == 'robot'
    
    print("  xacro_assembly_files: PASS")


def test_xacro_skips_empty_case_colliding_assemblies():
    """Empty wrapper assemblies must not overwrite real case-different macros."""
    from ..core.xacro_generator import generate_xacro_package
    from ..core.data_types import (
        AssemblyInfo,
        ExportConfig,
        LinkNode,
        RobotModel,
    )

    model = RobotModel(name="casebot", root_link="base_link")
    model.assemblies["Panther"] = AssemblyInfo(name="Panther")
    model.assemblies["panther"] = AssemblyInfo(
        name="panther",
        links=["base_link"],
    )
    model.links["base_link"] = LinkNode(
        urdf_name="base_link",
        clean_name="base_link",
        assembly="panther",
        is_empty=True,
        has_visual_mesh=False,
    )

    with tempfile.TemporaryDirectory() as tmpdir:
        files = generate_xacro_package(
            model,
            ExportConfig(package_name="casebot_description"),
            tmpdir,
        )
        assert "urdf/assemblies/panther.urdf.xacro" in files
        assert "urdf/assemblies/Panther.urdf.xacro" not in files

        top = open(
            os.path.join(tmpdir, "urdf", "casebot.urdf.xacro"),
            encoding="utf-8",
        ).read()
        assert "assemblies/panther.urdf.xacro" in top
        assert "assemblies/Panther.urdf.xacro" not in top
        assert "<xacro:panther " in top
        assert "<xacro:Panther " not in top

        asm = open(
            os.path.join(tmpdir, "urdf", "assemblies", "panther.urdf.xacro"),
            encoding="utf-8",
        ).read()
        assert '<xacro:macro name="panther"' in asm
        assert 'link name="${prefix}base_link"' in asm

    print("  xacro_skips_empty_case_colliding_assemblies: PASS")


def test_xacro_reexport_normalizes_stale_case_collision_file():
    """Re-export should remove an old case-colliding wrapper filename."""
    from ..core.xacro_generator import generate_xacro_package
    from ..core.data_types import (
        AssemblyInfo,
        ExportConfig,
        LinkNode,
        RobotModel,
    )

    model = RobotModel(name="casebot", root_link="base_link")
    model.assemblies["Panther"] = AssemblyInfo(name="Panther")
    model.assemblies["panther"] = AssemblyInfo(
        name="panther",
        links=["base_link"],
    )
    model.links["base_link"] = LinkNode(
        urdf_name="base_link",
        clean_name="base_link",
        assembly="panther",
        is_empty=True,
        has_visual_mesh=False,
    )

    with tempfile.TemporaryDirectory() as tmpdir:
        asm_dir = os.path.join(tmpdir, "urdf", "assemblies")
        os.makedirs(asm_dir, exist_ok=True)
        with open(
            os.path.join(asm_dir, "Panther.urdf.xacro"),
            "w",
            encoding="utf-8",
        ) as f:
            f.write("stale wrapper")

        generate_xacro_package(
            model,
            ExportConfig(package_name="casebot_description"),
            tmpdir,
        )

        names = os.listdir(asm_dir)
        assert "panther.urdf.xacro" in names
        assert "Panther.urdf.xacro" not in names
        asm = open(
            os.path.join(asm_dir, "panther.urdf.xacro"),
            encoding="utf-8",
        ).read()
        assert '<xacro:macro name="panther"' in asm
        assert "stale wrapper" not in asm

    print("  xacro_reexport_normalizes_stale_case_collision_file: PASS")


def test_xacro_case_colliding_nonempty_assemblies_get_unique_files():
    """Case-only assembly names get unique filenames on Windows."""
    from ..core.xacro_generator import generate_xacro_package
    from ..core.data_types import (
        AssemblyInfo,
        ExportConfig,
        LinkNode,
        RobotModel,
    )

    model = RobotModel(name="casebot", root_link="upper_link")
    model.assemblies["Panther"] = AssemblyInfo(
        name="Panther",
        links=["upper_link"],
    )
    model.assemblies["panther"] = AssemblyInfo(
        name="panther",
        links=["lower_link"],
    )
    model.links["upper_link"] = LinkNode(
        urdf_name="upper_link",
        clean_name="upper_link",
        assembly="Panther",
        is_empty=True,
        has_visual_mesh=False,
    )
    model.links["lower_link"] = LinkNode(
        urdf_name="lower_link",
        clean_name="lower_link",
        assembly="panther",
        is_empty=True,
        has_visual_mesh=False,
    )

    with tempfile.TemporaryDirectory() as tmpdir:
        files = generate_xacro_package(
            model,
            ExportConfig(package_name="casebot_description"),
            tmpdir,
        )
        assert "urdf/assemblies/Panther.urdf.xacro" in files
        assert "urdf/assemblies/panther_2.urdf.xacro" in files

        top = open(
            os.path.join(tmpdir, "urdf", "casebot.urdf.xacro"),
            encoding="utf-8",
        ).read()
        assert "assemblies/Panther.urdf.xacro" in top
        assert "assemblies/panther_2.urdf.xacro" in top
        assert "<xacro:Panther " in top
        assert "<xacro:panther " in top

    print("  xacro_case_colliding_nonempty_assemblies_get_unique_files: PASS")


def test_xacro_mount_joints_in_top_level():
    """Mount joints (cross-assembly) should be in top-level, not assembly files."""
    from ..core.robot_model import build_model
    from ..core.package_generator import generate_package
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    # The mini snapshot has only 1 assembly, so no mount joints.
    # Check that top-level has no joints (all internal to assembly)
    with tempfile.TemporaryDirectory() as tmpdir:
        config = ExportConfig(
            package_name="test_robot_description",
            output_dir=tmpdir,
            collision_auto_method="primitive",
        )
        
        pkg_dir = generate_package(model, config, log)
        
        # Read top-level xacro
        xacro_path = os.path.join(pkg_dir, "urdf", "test_robot.urdf.xacro")
        with open(xacro_path) as f:
            content = f.read()
        
        # Single assembly = no mount joints, all joints in assembly macro
        mount_count = sum(1 for j in model.joints.values() if j.is_mount)
        
        # Count <joint> elements in top-level (not inside include/macro)
        tree = ET.parse(xacro_path)
        root = tree.getroot()
        top_joints = root.findall('joint')
        
        assert len(top_joints) == mount_count, \
            f"Expected {mount_count} mount joints in top-level, got {len(top_joints)}"
    
    print("  xacro_mount_joints_in_top_level: PASS")


def test_xacro_prefix_parameter():
    """Assembly macros should use ${prefix} on all link/joint names."""
    from ..core.robot_model import build_model
    from ..core.package_generator import generate_package
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    with tempfile.TemporaryDirectory() as tmpdir:
        config = ExportConfig(
            package_name="test_robot_description",
            output_dir=tmpdir,
            collision_auto_method="primitive",
        )
        
        pkg_dir = generate_package(model, config, log)
        
        for asm_name in model.assemblies:
            asm_path = os.path.join(pkg_dir, "urdf", "assemblies", f"{asm_name}.urdf.xacro")
            with open(asm_path) as f:
                content = f.read()
            
            # Should contain ${prefix} in link and joint names
            assert '${prefix}' in content, \
                f"Assembly {asm_name} xacro missing ${{prefix}} usage"
            
            # Should have macro definition with params="prefix"
            assert f'params="prefix"' in content, \
                f"Assembly {asm_name} missing params='prefix'"
    
    print("  xacro_prefix_parameter: PASS")


# ──────────────────────────────────────────────
# Integration test with real snapshot
# ──────────────────────────────────────────────

def test_stl_rescale_mm_to_cm():
    """STL rescaling should convert mm vertices to cm."""
    import struct
    from ..core.collision_generator import rescale_stl_to_cm
    
    with tempfile.TemporaryDirectory() as tmpdir:
        stl_path = os.path.join(tmpdir, "test.stl")
        
        # Write a simple STL with one triangle in mm (180mm = 18cm)
        with open(stl_path, 'wb') as f:
            f.write(b'\0' * 80)  # header
            f.write(struct.pack('<I', 1))  # 1 triangle
            # normal
            f.write(struct.pack('<fff', 0, 0, 1))
            # vertices in mm
            f.write(struct.pack('<fff', 0, 0, 0))
            f.write(struct.pack('<fff', 180, 0, 0))
            f.write(struct.pack('<fff', 0, 180, 0))
            f.write(struct.pack('<H', 0))
        
        # Rescale from mm to cm
        result = rescale_stl_to_cm(stl_path, "mm")
        assert result, "rescale should succeed"
        
        # Read back and verify
        with open(stl_path, 'rb') as f:
            f.read(80)  # header
            count = struct.unpack('<I', f.read(4))[0]
            assert count == 1
            data = f.read(50)
            vals = struct.unpack('<12fH', data)
            # Vertex B should be 18.0 cm (180mm * 0.1)
            assert abs(vals[3] - 0.0) < 1e-4, f"v1.x should be 0, got {vals[3]}"
            assert abs(vals[6] - 18.0) < 1e-4, f"v2.x should be 18.0, got {vals[6]}"
            assert abs(vals[9] - 0.0) < 1e-4, f"v3.x should be 0, got {vals[9]}"
            assert abs(vals[10] - 18.0) < 1e-4, f"v3.y should be 18.0, got {vals[10]}"
    
    print("  stl_rescale_mm_to_cm: PASS")


def test_stl_rescale_cm_noop():
    """Rescaling from cm to cm should be a no-op."""
    from ..core.collision_generator import rescale_stl_to_cm
    
    with tempfile.TemporaryDirectory() as tmpdir:
        stl_path = os.path.join(tmpdir, "test.stl")
        
        # Write a simple STL
        import struct
        with open(stl_path, 'wb') as f:
            f.write(b'\0' * 80)
            f.write(struct.pack('<I', 1))
            f.write(struct.pack('<fff', 0, 0, 1))
            f.write(struct.pack('<fff', 5.0, 0, 0))
            f.write(struct.pack('<fff', 0, 5.0, 0))
            f.write(struct.pack('<fff', 0, 0, 5.0))
            f.write(struct.pack('<H', 0))
        
        original_size = os.path.getsize(stl_path)
        result = rescale_stl_to_cm(stl_path, "cm")
        assert result, "noop rescale should succeed"
        # File should not be rewritten
        assert os.path.getsize(stl_path) == original_size
    
    print("  stl_rescale_cm_noop: PASS")


def test_empty_link_minimal_urdf():
    """Empty links (0 bodies) should generate minimal URDF with no visual/collision."""
    from ..core.robot_model import build_model
    from ..core.collision_generator import resolve_collision
    from ..core.xacro_generator import generate_urdf
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    # Make link2 empty
    link2 = model.links.get("link2")
    assert link2, "Expected link2 in model"
    link2.is_empty = True
    link2.body_count = 0
    
    config = ExportConfig(package_name="test_description", collision_auto_method="primitive")
    resolve_collision(model, config, log)
    for name, link in model.links.items():
        if link.collision and link.collision.source == "primitive":
            link.collision.mesh_path = link.mesh_collision
    
    urdf = generate_urdf(model, config)
    root = ET.fromstring(urdf)
    
    link2_elem = None
    for le in root.findall('link'):
        if le.get('name') == 'link2':
            link2_elem = le
            break
    assert link2_elem is not None, "link2 not found"
    
    # Should have inertial but no visual or collision
    assert link2_elem.find('inertial') is not None, "Empty link should have minimal inertial"
    assert link2_elem.find('visual') is None, "Empty link should have no visual"
    assert link2_elem.find('collision') is None, "Empty link should have no collision"
    
    print("  empty_link_minimal_urdf: PASS")


def test_frame_only_link_omits_geometry_and_inertial():
    """Explicit frame_* links emit only the <link> tag."""
    from ..core.collision_generator import resolve_collision
    from ..core.xacro_generator import generate_urdf
    from ..core.data_types import ExportConfig, RobotModel, LinkNode, JointNode

    model = RobotModel(name="framebot", root_link="base_link")
    model.links["base_link"] = LinkNode(
        urdf_name="base_link",
        clean_name="base_link",
        mass_kg=1.0,
        body_count=1,
        has_visual_mesh=False,
    )
    model.links["imu"] = LinkNode(
        urdf_name="imu",
        clean_name="imu",
        is_empty=True,
        is_frame_only=True,
        has_visual_mesh=False,
        body_count=0,
    )
    model.joints["imu_mount"] = JointNode(
        name="imu_mount",
        joint_type="fixed",
        parent_link="base_link",
        child_link="imu",
    )

    config = ExportConfig(package_name="test_description", collision_auto_method="primitive")
    resolve_collision(model, config, _make_logger())
    assert model.links["imu"].collision is None

    urdf = generate_urdf(model, config)
    root = ET.fromstring(urdf)
    imu = next(le for le in root.findall("link") if le.get("name") == "imu")
    assert imu.find("inertial") is None, "frame-only link should have no inertial"
    assert imu.find("visual") is None, "frame-only link should have no visual"
    assert imu.find("collision") is None, "frame-only link should have no collision"

    print("  frame_only_link_omits_geometry_and_inertial: PASS")


def test_bake_offset_propagates_to_child_joints():
    """Downstream joints from a bake-offset link should include bake correction."""
    from ..core.robot_model import build_model
    from ..core.data_types import ExportConfig
    
    # Look only inside this repo's debug/ folder (legacy paths walked one
    # directory too far and grabbed unrelated snapshots from the user's
    # home directory). Skip if the snapshot for the specific design that
    # exercised this bug isn't present locally.
    snap_path = os.path.join(os.path.dirname(__file__), '..', 'debug', 'snapshot.json')
    if not os.path.exists(snap_path):
        print("  bake_offset_propagation: SKIPPED (no local snapshot)")
        return

    with open(snap_path) as f:
        data = json.load(f)

    from .test_robot_model import _make_snapshot as ms
    snap = ms(data)
    log = _make_logger()
    model = build_model(snap, log)

    # "Rigid 13" connects head_link → zed2i_camera_link in the original
    # bug-reproducer design. If the local snapshot is from a different
    # design, skip rather than fail.
    rigid13 = model.joints.get("Rigid 13")
    if not rigid13:
        print("  bake_offset_propagation: SKIPPED (snapshot lacks 'Rigid 13')")
        return
    
    # head_link has bake offset (0, -0.248, 0)
    head = model.links.get("head_link")
    assert head and head.needs_mesh_bake, "head_link should have bake offset"
    
    # Rigid 13 origin Y should include the -0.248 bake correction
    assert abs(rigid13.origin_xyz[1] - (-0.248)) < 0.001, \
        f"Rigid 13 Y should be ~-0.248, got {rigid13.origin_xyz[1]}"
    
    print("  bake_offset_propagation: PASS")


def test_collision_sub_assembly_flattening():
    """Collision sub-assembly (2 children, one collision_*) should be flattened."""
    from ..core.robot_model import build_model
    
    # Look only inside this repo's debug/ folder; skip if the snapshot
    # for the original zed2i-center-link bug isn't present locally.
    snap_path = os.path.join(os.path.dirname(__file__), '..', 'debug', 'snapshot.json')
    if not os.path.exists(snap_path):
        print("  collision_sub_assembly_flattening: SKIPPED (no local snapshot)")
        return

    with open(snap_path) as f:
        data = json.load(f)

    from .test_robot_model import _make_snapshot as ms
    snap = ms(data)
    log = _make_logger()
    model = build_model(snap, log)

    # The bug-reproducer design had a zed2i_center_link sub-assembly.
    # Skip if the local snapshot is from a different design.
    if "zed2i_center_link" not in str(data):
        print("  collision_sub_assembly_flattening: SKIPPED (snapshot lacks zed2i_center_link)")
        return

    # zed2i_center_link should NOT be an assembly
    assert "zed2i_center_link" not in model.assemblies, \
        "zed2i_center_link should be flattened out"
    
    # zed2i_link should be a link (not collision_zed)
    assert "zed2i_link" in model.links, "zed2i_link should be a URDF link"
    assert "collision_zed" not in model.links, "collision_zed should NOT be a URDF link"
    
    # zed2i_link should have explicit collision
    zed_link = model.links["zed2i_link"]
    assert zed_link.has_explicit_collision, "zed2i_link should have explicit collision"
    
    # Rigid 2 should connect zed2i_camera_link → zed2i_link
    rigid2 = model.joints.get("Rigid 2")
    assert rigid2, "Expected 'Rigid 2' joint"
    assert rigid2.child_link == "zed2i_link", \
        f"Rigid 2 child should be zed2i_link, got {rigid2.child_link}"
    
    # No warnings
    assert len(model.warnings) == 0, f"Expected 0 warnings, got: {model.warnings}"
    
    print("  collision_sub_assembly_flattening: PASS")


def test_validation_report():
    """Validation report should be generated with correct content."""
    from ..core.robot_model import build_model
    from ..core.collision_generator import resolve_collision
    from ..core.package_generator import generate_validation_report
    from ..core.data_types import ExportConfig
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    config = ExportConfig(package_name="test_description", collision_auto_method="primitive")
    resolve_collision(model, config, log)
    
    with tempfile.TemporaryDirectory() as tmpdir:
        path = generate_validation_report(model, config, tmpdir)
        assert os.path.exists(path), "validation.md should exist"
        
        content = open(path).read()
        assert "# Validation Report" in content
        assert "PASS" in content
        assert "Kinematic Tree" in content
        assert "Collision Geometry" in content
        assert "base_link" in content
    
    print("  validation_report: PASS")


def test_real_snapshot_package():
    """Build full package from real snapshot.json."""
    from ..core.robot_model import build_model
    from ..core.package_generator import generate_package
    from ..core.data_types import ExportConfig
    
    snap_path = None
    for p in [
        os.path.join(os.path.dirname(__file__), '..', '..', 'debug', 'snapshot.json'),
        os.path.join(os.path.dirname(__file__), '..', 'snapshot.json'),
    ]:
        if os.path.exists(p):
            snap_path = p
            break
    
    if not snap_path:
        print("  real_snapshot_package: SKIPPED (snapshot.json not found)")
        return
    
    with open(snap_path) as f:
        data = json.load(f)
    
    snap = _make_snapshot(data)
    log = _make_logger()
    model = build_model(snap, log)
    
    with tempfile.TemporaryDirectory() as tmpdir:
        pkg_name = f"{model.name}_description"
        config = ExportConfig(
            package_name=pkg_name,
            output_dir=tmpdir,
            collision_auto_method="primitive",
        )
        
        pkg_dir = generate_package(model, config, log)
        
        # Parse URDF (filename = model.name)
        urdf_path = os.path.join(pkg_dir, "urdf", f"{model.name}.urdf")
        assert os.path.isfile(urdf_path), f"URDF not found: {urdf_path}"
        
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        
        links = root.findall('link')
        joints = root.findall('joint')
        
        # Structural checks (snapshot-agnostic)
        assert len(links) >= 2, f"Expected at least 2 links, got {len(links)}"
        assert len(joints) >= 1, f"Expected at least 1 joint, got {len(joints)}"
        
        # Root link first
        assert links[0].attrib['name'] == 'base_link'
        
        # Link element contents depend on the source role:
        # physical links have inertial/visual/collision, accidental empty
        # links keep minimal inertial only, and explicit !frame_* links are
        # pure frames with no geometry or inertial.
        for link_elem in links:
            name = link_elem.attrib['name']
            model_link = model.links.get(name)
            assert model_link is not None, f"URDF link not found in model: {name}"

            inertial = link_elem.find('inertial')
            visual = link_elem.find('visual')
            collision = link_elem.find('collision')

            if getattr(model_link, "is_frame_only", False):
                assert inertial is None, f"Frame-only link should omit inertial: {name}"
                assert visual is None, f"Frame-only link should omit visual: {name}"
                assert collision is None, f"Frame-only link should omit collision: {name}"
                continue

            assert inertial is not None, f"Missing inertial: {name}"
            if getattr(model_link, "is_empty", False):
                assert visual is None, f"Empty link should omit visual: {name}"
                assert collision is None, f"Empty link should omit collision: {name}"
                continue

            assert visual is not None, f"Missing visual: {name}"
            assert collision is not None, f"Missing collision: {name}"
        
        # All joints reference existing links
        link_names = {l.attrib['name'] for l in links}
        for joint_elem in root.findall('joint'):
            jname = joint_elem.attrib['name']
            parent = joint_elem.find('parent').attrib['link']
            child = joint_elem.find('child').attrib['link']
            assert parent in link_names, f"Joint '{jname}' parent '{parent}' not in links"
            assert child in link_names, f"Joint '{jname}' child '{child}' not in links"
        
        # Mesh directories exist only for assemblies that own exported links
        export_assemblies = sorted({link.assembly for link in model.links.values()})
        for asm_name in export_assemblies:
            asm_dir = os.path.join(pkg_dir, "meshes", asm_name)
            assert os.path.isdir(asm_dir), f"Missing mesh dir: {asm_name}"
        
        # Xacro structure
        xacro_path = os.path.join(pkg_dir, "urdf", f"{model.name}.urdf.xacro")
        assert os.path.isfile(xacro_path), f"Top-level xacro not found"
        
        # Each assembly has a xacro file
        mount_joint_count = sum(1 for j in model.joints.values() if j.is_mount)
        asm_xacro_count = 0
        for asm_name in export_assemblies:
            asm_xacro = os.path.join(pkg_dir, "urdf", "assemblies", f"{asm_name}.urdf.xacro")
            assert os.path.isfile(asm_xacro), f"Missing assembly xacro: {asm_name}"
            asm_xacro_count += 1
        
        # Top-level should have mount joints
        top_tree = ET.parse(xacro_path)
        top_root = top_tree.getroot()
        top_joints = top_root.findall('joint')
        assert len(top_joints) == mount_joint_count, \
            f"Expected {mount_joint_count} mount joints in top-level, got {len(top_joints)}"
        
        print(f"  real_snapshot_package: PASS")
        print(f"    Package: {pkg_dir}")
        print(f"    URDF: {len(links)} links, {len(joints)} joints")
        print(f"    Xacro: {asm_xacro_count} assembly macros, {mount_joint_count} mount joints")


def test_collision_ground_truth():
    """Validate primitive fitting against designer-verified ground truth."""
    import json
    import math
    from ..core.collision_generator import fit_primitive
    
    fixtures = os.path.join(os.path.dirname(__file__), "fixtures")
    snap_path = os.path.join(fixtures, "snapshot.json")
    gt_path = os.path.join(fixtures, "ground_truth.json")
    
    if not os.path.isfile(snap_path) or not os.path.isfile(gt_path):
        print("  collision_ground_truth: SKIPPED (fixtures not found)")
        return
    
    with open(snap_path) as f:
        snap = json.load(f)
    with open(gt_path) as f:
        gt = json.load(f)
    
    expected = gt["expected_shapes"]
    
    # Map occurrence clean_name → occurrence data (use first non-subassembly)
    occ_by_name = {}
    for path, occ in snap["occurrences"].items():
        if occ.get("is_subassembly", False):
            continue
        name = occ["clean_name"]
        if name not in occ_by_name:
            occ_by_name[name] = occ
    
    failures = []
    tested = 0
    
    for name, truth in expected.items():
        # Ground truth uses URDF names (base_link for root), but snapshot uses clean names
        lookup = name if name in occ_by_name else name
        if name == "base_link":
            lookup = "body_link"
        
        if lookup not in occ_by_name:
            failures.append(f"  {name}: NOT FOUND in snapshot")
            continue
        
        occ = occ_by_name[lookup]
        bs = occ.get("bbox_size", [0, 0, 0])
        vol = occ.get("volume_m3", 0)
        bmin = occ.get("bbox_min", [0, 0, 0])
        bmax = occ.get("bbox_max", [0, 0, 0])
        
        prim = fit_primitive(
            bbox_size=tuple(bs),
            bbox_min=tuple(bmin),
            bbox_max=tuple(bmax),
            volume=vol,
        )
        
        tested += 1
        got_shape = prim.shape if prim else "none"
        want_shape = truth["shape"]
        
        if got_shape != want_shape:
            fill = vol / (bs[0]*bs[1]*bs[2]) if bs[0]*bs[1]*bs[2] > 1e-15 else 0
            failures.append(
                f"  {name}: expected {want_shape}, got {got_shape} "
                f"(bbox=[{bs[0]*1000:.0f}x{bs[1]*1000:.0f}x{bs[2]*1000:.0f}]mm "
                f"fill={fill:.3f})"
            )
            continue
        
        # For cylinders, also check axis if specified
        if want_shape == "cylinder" and "axis" in truth and prim:
            want_axis = truth["axis"]
            got_axis = 2
            if abs(prim.origin_rpy[1] - math.pi / 2) < 0.1:
                got_axis = 0
            elif abs(prim.origin_rpy[0] - math.pi / 2) < 0.1:
                got_axis = 1
            
            if got_axis != want_axis:
                axis_names = {0: "X", 1: "Y", 2: "Z"}
                failures.append(
                    f"  {name}: correct shape (cylinder) but wrong axis: "
                    f"expected {axis_names[want_axis]}, got {axis_names[got_axis]}"
                )
    
    if failures:
        print(f"  collision_ground_truth: FAIL ({len(failures)}/{tested} wrong)")
        for f in failures:
            print(f)
        assert False, f"Ground truth validation failed: {len(failures)} errors"
    
    print(f"  collision_ground_truth: PASS ({tested}/{tested} correct)")


def test_fixture_model_structure():
    """Build model from fixture snapshot and validate against ground truth."""
    import json
    from ..core.robot_model import build_model
    
    fixtures = os.path.join(os.path.dirname(__file__), "fixtures")
    snap_path = os.path.join(fixtures, "snapshot.json")
    gt_path = os.path.join(fixtures, "ground_truth.json")
    
    if not os.path.isfile(snap_path) or not os.path.isfile(gt_path):
        print("  fixture_model_structure: SKIPPED (fixtures not found)")
        return
    
    with open(snap_path) as f:
        data = json.load(f)
    with open(gt_path) as f:
        gt = json.load(f)
    
    snap = _make_snapshot(data)
    log = _make_logger()
    model = build_model(snap, log)
    
    gt_model = gt["model"]
    failures = []
    
    # Check basic counts
    if model.name != gt_model["name"]:
        failures.append(f"  name: expected '{gt_model['name']}', got '{model.name}'")
    if model.root_link != gt_model["root_link"]:
        failures.append(f"  root: expected '{gt_model['root_link']}', got '{model.root_link}'")
    if len(model.links) != gt_model["link_count"]:
        failures.append(f"  links: expected {gt_model['link_count']}, got {len(model.links)}")
    if len(model.joints) != gt_model["joint_count"]:
        failures.append(f"  joints: expected {gt_model['joint_count']}, got {len(model.joints)}")
    if len(model.assemblies) != gt_model["assembly_count"]:
        failures.append(f"  assemblies: expected {gt_model['assembly_count']}, got {len(model.assemblies)}")
    
    # Check assembly names
    got_asms = sorted(model.assemblies.keys())
    want_asms = sorted(gt_model["assemblies"])
    if got_asms != want_asms:
        failures.append(f"  assembly names: expected {want_asms}, got {got_asms}")
    
    # Check model has no errors
    if model.errors:
        failures.append(f"  model errors: {model.errors}")
    
    if failures:
        print(f"  fixture_model_structure: FAIL")
        for f in failures:
            print(f)
        assert False, f"Model structure validation failed"
    
    print(f"  fixture_model_structure: PASS "
          f"({len(model.links)} links, {len(model.joints)} joints, "
          f"{len(model.assemblies)} assemblies)")


def test_fixture_joint_mapping():
    """Verify every joint has correct type, parent, and child."""
    import json
    from ..core.robot_model import build_model
    
    fixtures = os.path.join(os.path.dirname(__file__), "fixtures")
    snap_path = os.path.join(fixtures, "snapshot.json")
    gt_path = os.path.join(fixtures, "ground_truth.json")
    
    if not os.path.isfile(snap_path) or not os.path.isfile(gt_path):
        print("  fixture_joint_mapping: SKIPPED (fixtures not found)")
        return
    
    with open(snap_path) as f:
        data = json.load(f)
    with open(gt_path) as f:
        gt = json.load(f)
    
    snap = _make_snapshot(data)
    log = _make_logger()
    model = build_model(snap, log)
    
    failures = []
    tested = 0
    
    for jname, truth in gt["expected_joints"].items():
        tested += 1
        if jname not in model.joints:
            failures.append(f"  {jname}: NOT FOUND in model")
            continue
        
        joint = model.joints[jname]
        
        if joint.joint_type != truth["type"]:
            failures.append(f"  {jname}: type expected '{truth['type']}', got '{joint.joint_type}'")
        if joint.parent_link != truth["parent"]:
            failures.append(f"  {jname}: parent expected '{truth['parent']}', got '{joint.parent_link}'")
        if joint.child_link != truth["child"]:
            failures.append(f"  {jname}: child expected '{truth['child']}', got '{joint.child_link}'")
    
    if failures:
        print(f"  fixture_joint_mapping: FAIL ({len(failures)} errors)")
        for f in failures:
            print(f)
        assert False, f"Joint mapping validation failed"
    
    print(f"  fixture_joint_mapping: PASS ({tested}/{tested} joints correct)")


def test_fixture_package_generation():
    """Generate full package from fixture snapshot, verify output structure."""
    import json
    from ..core.robot_model import build_model
    from ..core.package_generator import generate_package
    from ..core.data_types import ExportConfig
    
    fixtures = os.path.join(os.path.dirname(__file__), "fixtures")
    snap_path = os.path.join(fixtures, "snapshot.json")
    
    if not os.path.isfile(snap_path):
        print("  fixture_package_generation: SKIPPED (fixtures not found)")
        return
    
    with open(snap_path) as f:
        data = json.load(f)
    
    snap = _make_snapshot(data)
    log = _make_logger()
    model = build_model(snap, log)
    
    with tempfile.TemporaryDirectory() as tmpdir:
        config = ExportConfig(
            package_name="basic_platform_description",
            output_dir=tmpdir,
            collision_auto_method="primitive",
        )
        generate_package(model, config, log)
        
        pkg = os.path.join(tmpdir, "basic_platform_description")
        failures = []
        
        # Check required files exist
        required = [
            "urdf/basic_platform.urdf.xacro",
            "urdf/basic_platform.urdf",
            "launch/display.launch.py",
            "package.xml",
            "CMakeLists.txt",
        ]
        for f in required:
            if not os.path.isfile(os.path.join(pkg, f)):
                failures.append(f"  missing: {f}")
        
        # Check assembly xacro files exist
        for asm in ["dummy_panther", "dummy_zed2i", "rail_mount_plate", "turret"]:
            xacro = os.path.join(pkg, "urdf", "assemblies", f"{asm}.urdf.xacro")
            if not os.path.isfile(xacro):
                failures.append(f"  missing assembly xacro: {asm}")
        
        # Check URDF is valid XML with correct counts
        urdf_path = os.path.join(pkg, "urdf", "basic_platform.urdf")
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        links = root.findall("link")
        joints = root.findall("joint")
        
        if len(links) != 11:
            failures.append(f"  URDF links: expected 11, got {len(links)}")
        if len(joints) != 10:
            failures.append(f"  URDF joints: expected 10, got {len(joints)}")
        
        # Root link should be first
        if links[0].get("name") != "base_link":
            failures.append(f"  URDF root not first: got '{links[0].get('name')}'")
        
        if failures:
            print(f"  fixture_package_generation: FAIL")
            for f in failures:
                print(f)
            assert False, f"Package generation failed"
        
        print(f"  fixture_package_generation: PASS (package OK, URDF valid)")


# ──────────────────────────────────────────────
# Runner
# ──────────────────────────────────────────────

def run_all():
    print("Running export pipeline tests...\n")
    
    # URDF generator
    test_urdf_valid_xml()
    test_urdf_link_count()
    test_urdf_joint_count()
    test_urdf_inertial()
    test_urdf_joint_parent_child()
    test_urdf_revolute_has_limits()
    test_urdf_root_first()
    
    # Collision
    test_collision_primitive_box()
    test_collision_primitive_cylinder()
    test_collision_primitive_cylinder_rotated()
    test_collision_oriented_box_from_obj_vertices()
    test_collision_oriented_box_rejects_ambiguous_square_footprint()
    test_collision_oriented_box_prefers_mesh_over_inertia_frame()
    test_convex_hull_tetrahedron_triangles()
    test_convex_hull_obj_reads_late_extreme_vertices()
    test_collision_excluded_body_uses_filtered_collision_input()
    test_collision_excluded_only_link_has_no_collision()
    test_square_beam_obj_is_box_not_cylinder()
    test_collision_primitive_sphere()
    test_collision_primitive_zero_bbox()
    test_collision_plate_is_box()
    test_collision_wheel_is_cylinder()
    test_collision_revolute_joint_hint()
    test_collision_bbox_center_origin()
    test_collision_resolve_model()
    test_collision_primitive_in_urdf()
    test_collision_stl_files_generated()
    test_convex_hull_collision_stl_files_generated()
    test_bake_offset_in_xacro()
    test_stl_rescale_mm_to_cm()
    test_stl_rescale_cm_noop()
    test_empty_link_minimal_urdf()
    test_frame_only_link_omits_geometry_and_inertial()
    test_validation_report()
    
    # Package
    test_package_structure()
    test_package_urdf_parseable()
    test_package_xml_valid()
    test_ros2_control_generated_package()
    test_ros2_control_can_be_disabled()
    test_ros2_control_passive_joint_is_state_only()
    test_cmake_has_project()
    
    # Xacro
    test_xacro_top_level_exists()
    test_xacro_assembly_files()
    test_xacro_skips_empty_case_colliding_assemblies()
    test_xacro_reexport_normalizes_stale_case_collision_file()
    test_xacro_case_colliding_nonempty_assemblies_get_unique_files()
    test_xacro_mount_joints_in_top_level()
    test_xacro_prefix_parameter()
    
    # Integration (real snapshot)
    test_bake_offset_propagates_to_child_joints()
    test_collision_sub_assembly_flattening()
    
    # Fixture-based validation (snapshot + ground truth)
    test_collision_ground_truth()
    test_fixture_model_structure()
    test_fixture_joint_mapping()
    test_fixture_package_generation()
    test_real_snapshot_package()

    # OBJ→DAE converter
    test_dae_basic_conversion()
    test_dae_multi_material()
    test_dae_meters_scale()
    test_dae_xacro_emits_scale_one()

    # User config (TOML)
    test_user_config_minimal_parser()
    test_user_config_apply_minimal_verbosity()
    test_user_config_per_key_override_wins()
    test_user_config_accepts_convex_hull_collision_method()
    test_user_config_accepts_frame_options()
    test_user_config_accepts_ros2_control_options()
    test_user_config_prefers_plugin_local_config_path()
    test_verbose_package_emits_frame_config_without_rviz_or_control()
    test_minimal_package_skips_optional_dirs()

    # Tree viz
    test_tree_render_simple_chain()
    test_tree_render_marks_multi_parent()

    # Closing-joint sidecar in robot_data.yaml
    test_robot_data_yaml_passive_field_per_joint()
    test_robot_data_yaml_parses_when_merged_member_has_bang_prefix()
    test_robot_data_yaml_closing_joints_empty_section()
    test_robot_data_yaml_closing_joints_section_populated()
    test_urdf_omits_visual_when_export_failed()
    test_urdf_uses_collision_stl_when_visual_missing()
    test_flat_design_emits_assembly_xacro()
    test_safe_identifier_strips_non_ascii_for_urdf()
    test_numeric_assembly_name_is_xacro_safe()
    test_urdf_material_names_are_ascii_safe()
    test_urdf_joint_names_are_ascii_safe()
    test_acc_prefix_recognised_and_stripped()
    test_acc_rigid_group_uses_visual_mesh_as_collision()
    test_collision_override_prefixes_force_method()
    test_per_member_obj_concat_transforms_and_namespaces()
    test_merged_obj_anchor_correction_uses_lca_relative_transform()
    test_invalid_fusion_joint_endpoint_is_skipped()
    test_root_side_invalid_joint_endpoint_uses_design_root()
    test_root_component_visual_export_uses_root_bodies_directly()
    test_root_component_visual_export_hides_child_occurrences()
    test_minimal_mode_still_emits_yaml_when_closing_joints_present()
    test_closing_joints_fixture_well_formed()

    print("\n✓ All export pipeline tests passed!")


# ──────────────────────────────────────────────
# OBJ → DAE converter tests
# ──────────────────────────────────────────────

_MINIMAL_OBJ = """\
# Minimal OBJ: a single triangle in cm
mtllib test.mtl
o triangle
v 0.0 0.0 0.0
v 100.0 0.0 0.0
v 0.0 100.0 0.0
vn 0.0 0.0 1.0
usemtl Steel
f 1//1 2//1 3//1
"""

_MINIMAL_MTL = """\
newmtl Steel
Kd 0.188 0.231 0.588
Ka 0.05 0.06 0.10
Ks 0.20 0.20 0.20
"""


def _write_minimal_obj_pair(tmpdir):
    obj_path = os.path.join(tmpdir, "shape.obj")
    mtl_path = os.path.join(tmpdir, "test.mtl")
    with open(obj_path, "w", encoding="utf-8") as f:
        f.write(_MINIMAL_OBJ)
    with open(mtl_path, "w", encoding="utf-8") as f:
        f.write(_MINIMAL_MTL)
    return obj_path, mtl_path


def test_dae_basic_conversion():
    """OBJ + MTL → DAE produces a valid XML file with positions and one material."""
    from ..core.obj_to_dae import obj_to_dae
    with tempfile.TemporaryDirectory() as tmp:
        obj, mtl = _write_minimal_obj_pair(tmp)
        dae = os.path.join(tmp, "shape.dae")
        ok = obj_to_dae(obj, mtl, dae, name="shape")
        assert ok, "Conversion returned False"
        assert os.path.isfile(dae), "DAE file not created"

        tree = ET.parse(dae)
        root = tree.getroot()
        # Strip the COLLADA namespace for sane lookups.
        ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
        assert root.tag.endswith("COLLADA")
        # 1 geometry, 1 material, 1 effect, 1 visual scene
        assert len(root.findall(".//c:geometry", ns)) == 1
        assert len(root.findall(".//c:material", ns)) == 1
        assert len(root.findall(".//c:effect", ns)) == 1
        # Triangle count
        tris = root.findall(".//c:triangles", ns)
        assert len(tris) == 1
        assert tris[0].get("count") == "1"
        # Material assignment
        assert tris[0].get("material").startswith("Steel") or tris[0].get("material") == "Steel"

    print("  dae_basic_conversion: PASS")


def test_dae_multi_material():
    """OBJ with multiple usemtl groups produces separate <triangles> blocks."""
    from ..core.obj_to_dae import obj_to_dae
    multi_obj = """\
mtllib mm.mtl
v 0 0 0
v 1 0 0
v 0 1 0
v 0 0 1
vn 0 0 1
usemtl red
f 1//1 2//1 3//1
usemtl blue
f 1//1 3//1 4//1
"""
    multi_mtl = """\
newmtl red
Kd 1 0 0
newmtl blue
Kd 0 0 1
"""
    with tempfile.TemporaryDirectory() as tmp:
        op = os.path.join(tmp, "x.obj")
        mp = os.path.join(tmp, "mm.mtl")
        with open(op, "w") as f: f.write(multi_obj)
        with open(mp, "w") as f: f.write(multi_mtl)
        dp = os.path.join(tmp, "x.dae")
        assert obj_to_dae(op, mp, dp, name="x")
        tree = ET.parse(dp)
        ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
        tris = tree.getroot().findall(".//c:triangles", ns)
        assert len(tris) == 2, f"expected 2 triangles blocks, got {len(tris)}"
        materials = sorted(t.get("material") for t in tris)
        assert materials == ["blue", "red"], f"got {materials}"

    print("  dae_multi_material: PASS")


def test_dae_meters_scale():
    """Vertices in DAE are 1/100 of OBJ values when scale_to_meters=True."""
    from ..core.obj_to_dae import obj_to_dae
    with tempfile.TemporaryDirectory() as tmp:
        obj, mtl = _write_minimal_obj_pair(tmp)
        dae = os.path.join(tmp, "shape.dae")
        assert obj_to_dae(obj, mtl, dae, name="shape", scale_to_meters=True)
        with open(dae, "r") as f:
            content = f.read()
        # The OBJ has v=100.0 cm; DAE should emit 1.0 (m).
        assert "1.000000 0.000000 0.000000" in content, \
            "OBJ vertex 100,0,0 cm should be 1,0,0 m in DAE"
        # And the OBJ's 0,100,0 cm vertex.
        assert "0.000000 1.000000 0.000000" in content
    print("  dae_meters_scale: PASS")


def test_dae_xacro_emits_scale_one():
    """When link.mesh_visual ends in .dae, xacro emits scale='1 1 1'."""
    from ..core.xacro_generator import _scale_for_mesh
    assert _scale_for_mesh("meshes/foo/bar.dae") == "1 1 1"
    assert _scale_for_mesh("meshes/foo/bar.obj") == "0.01 0.01 0.01"
    assert _scale_for_mesh("meshes/foo/bar.stl") == "0.01 0.01 0.01"
    print("  dae_xacro_emits_scale_one: PASS")


# ──────────────────────────────────────────────
# User config (xacro_export.toml) tests
# ──────────────────────────────────────────────

def test_user_config_minimal_parser():
    """The fallback TOML parser handles strings, booleans, ints, comments."""
    from ..core.user_config import _parse_toml_minimal
    text = '''
# top-level comment
[output]
verbosity = "minimal"  # trailing comment
zip = false
zip_name = ""

[features]
include_debug = true
include_rviz = false
'''
    parsed = _parse_toml_minimal(text)
    assert parsed["output"]["verbosity"] == "minimal"
    assert parsed["output"]["zip"] is False
    assert parsed["output"]["zip_name"] == ""
    assert parsed["features"]["include_debug"] is True
    assert parsed["features"]["include_rviz"] is False
    print("  user_config_minimal_parser: PASS")


def test_user_config_apply_minimal_verbosity():
    """verbosity='minimal' flips most include_* flags off."""
    from ..core.user_config import apply_to_config
    from ..core.data_types import ExportConfig
    cfg = ExportConfig()
    # Pre-condition: defaults are 'verbose'-ish (everything on)
    assert cfg.include_debug is True
    apply_to_config(cfg, {"output": {"verbosity": "minimal"}})
    assert cfg.include_debug is False
    assert cfg.include_docs is False
    assert cfg.include_robot_data_yaml is False
    assert cfg.include_rviz is False
    assert cfg.include_ros2_control is False
    # launch stays on even in minimal — small file, useful default
    assert cfg.include_launch is True
    print("  user_config_apply_minimal_verbosity: PASS")


def test_user_config_per_key_override_wins():
    """Per-key flag in [features] overrides the verbosity preset."""
    from ..core.user_config import apply_to_config
    from ..core.data_types import ExportConfig
    cfg = ExportConfig()
    apply_to_config(cfg, {
        "output": {"verbosity": "minimal"},
        "features": {"include_docs": True},  # override the minimal default
    })
    assert cfg.include_docs is True       # override applied
    assert cfg.include_robot_data_yaml is False  # untouched
    print("  user_config_per_key_override_wins: PASS")


def test_user_config_accepts_convex_hull_collision_method():
    """TOML collision_method supports the generated convex hull mode."""
    from ..core.user_config import apply_to_config
    from ..core.data_types import ExportConfig
    cfg = ExportConfig()
    apply_to_config(cfg, {"mesh": {"collision_method": "convex-hull"}})
    assert cfg.collision_auto_method == "convex_hull"
    print("  user_config_accepts_convex_hull_collision_method: PASS")


def test_user_config_accepts_frame_options():
    """TOML selects the frame convention and keeps its CSV inside config/."""
    from ..core.user_config import apply_to_config
    from ..core.data_types import ExportConfig

    cfg = ExportConfig()
    changes = apply_to_config(cfg, {
        "frames": {
            "convention": "FUSION",
            "overrides_file": "nested/custom_frames.csv",
        },
    })
    assert cfg.frame_convention == "fusion"
    assert cfg.frame_overrides_filename == "custom_frames.csv"
    assert "frames.convention = fusion" in changes
    assert "frames.overrides_file = custom_frames.csv" in changes

    print("  user_config_accepts_frame_options: PASS")


def test_user_config_accepts_ros2_control_options():
    """TOML can disable ros2_control and tune the generated mock system."""
    from ..core.user_config import apply_to_config
    from ..core.data_types import ExportConfig
    cfg = ExportConfig()
    apply_to_config(cfg, {
        "features": {"include_ros2_control": False},
        "ros2_control": {
            "hardware_plugin": "my_robot/MySystem",
            "update_rate": 250,
            "command_interfaces": "velocity,effort",
        },
    })
    assert cfg.include_ros2_control is False
    assert cfg.ros2_control_hardware_plugin == "my_robot/MySystem"
    assert cfg.ros2_control_update_rate == 250
    assert cfg.ros2_control_command_interfaces == ("velocity", "effort")
    print("  user_config_accepts_ros2_control_options: PASS")


def test_user_config_prefers_plugin_local_config_path():
    """Only the plugin-local config path is supported."""
    from ..core.user_config import _config_candidates, CONFIG_FILENAME
    plugin_root = os.path.join("repo", "fusion2URDF")
    candidates = _config_candidates(plugin_root)
    assert candidates == (os.path.join(plugin_root, CONFIG_FILENAME),)
    print("  user_config_prefers_plugin_local_config_path: PASS")


def test_tree_render_simple_chain():
    """Render a 3-link chain.  Output contains every link, every joint,
    and no multi-parent / orphan markers."""
    from ..core.robot_model import build_model
    from ..core.tree_render import render_tree
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    tree = render_tree(model)
    for link in model.links:
        assert link in tree, f"link {link} missing from tree:\n{tree}"
    for joint in model.joints:
        assert joint in tree, f"joint {joint} missing from tree:\n{tree}"
    assert "multi-parent" not in tree
    assert "Orphan" not in tree
    print("  tree_render_simple_chain: PASS")


def test_tree_render_marks_multi_parent():
    """A link with two parent joints is auto-resolved into a closing
    joint sidecar.  The URDF tree itself is left clean (one parent per
    link); the tree-renderer surfaces the routed-out joint in a
    "Closed-loop joints (sidecar)" section so the user sees what
    happened."""
    from ..core.data_types import (
        FusionSnapshot, FusionOccurrence, FusionJoint, Transform3D,
    )
    from ..core.robot_model import build_model
    from ..core.tree_render import render_tree

    # base → child via two paths: direct (j1) and via mid (j2 + j3)
    snap = FusionSnapshot(design_name="loop", design_name_clean="loop")
    snap.occurrences = {
        "asm v1:1": FusionOccurrence(full_path="asm v1:1", clean_name="asm",
                                       path_segments=["asm"], depth=0,
                                       is_subassembly=True,
                                       local_transform=Transform3D()),
        "asm v1:1+base v1:1": FusionOccurrence(full_path="asm v1:1+base v1:1",
                                                 clean_name="base",
                                                 path_segments=["asm", "base"],
                                                 depth=1, mass_kg=1.0, body_count=1,
                                                 local_transform=Transform3D()),
        "asm v1:1+mid v1:1": FusionOccurrence(full_path="asm v1:1+mid v1:1",
                                                clean_name="mid",
                                                path_segments=["asm", "mid"],
                                                depth=1, mass_kg=1.0, body_count=1,
                                                local_transform=Transform3D()),
        "asm v1:1+child v1:1": FusionOccurrence(full_path="asm v1:1+child v1:1",
                                                  clean_name="child",
                                                  path_segments=["asm", "child"],
                                                  depth=1, mass_kg=1.0, body_count=1,
                                                  local_transform=Transform3D()),
    }
    snap.joints = {
        "j1": FusionJoint(name="j1", defining_component="asm", motion_type="rigid",
                          occurrence_one_path="child v1:1", occurrence_one_clean="child",
                          occurrence_two_path="base v1:1", occurrence_two_clean="base",
                          axis_vector=(0, 0, 1)),
        "j2": FusionJoint(name="j2", defining_component="asm", motion_type="rigid",
                          occurrence_one_path="mid v1:1", occurrence_one_clean="mid",
                          occurrence_two_path="base v1:1", occurrence_two_clean="base",
                          axis_vector=(0, 0, 1)),
        "j3": FusionJoint(name="j3", defining_component="asm", motion_type="rigid",
                          occurrence_one_path="child v1:1", occurrence_one_clean="child",
                          occurrence_two_path="mid v1:1", occurrence_two_clean="mid",
                          axis_vector=(0, 0, 1)),
    }
    model = build_model(snap, _make_logger())
    tree = render_tree(model)

    # Auto-classification: the alphabetically-first joint among the
    # multi-parent candidates stays in the tree, the rest are routed
    # to ``model.closing_joints``.
    parents_of_child = [j for j in model.joints.values() if j.child_link == "child"]
    assert len(parents_of_child) == 1, (
        f"expected exactly one tree parent for 'child' after auto-classification, "
        f"got {len(parents_of_child)}: {[j.name for j in parents_of_child]}\n{tree}"
    )
    assert len(model.closing_joints) == 1, (
        f"expected exactly one closing joint, got {len(model.closing_joints)}: "
        f"{list(model.closing_joints.keys())}"
    )
    closing = next(iter(model.closing_joints.values()))
    assert closing.is_closing
    assert closing.is_passive, "closing joints are implicitly passive"
    assert closing.closing_source == "auto_detected"

    # A multi-parent warning was emitted suggesting an explicit tag.
    multi_warn = [w for w in model.warnings if "multi-parent" in w.lower()]
    assert multi_warn, f"expected multi-parent warning, got: {model.warnings}"

    # The tree renderer shows the routed-out joint in the sidecar
    # section so the user can see the auto-resolution.
    assert "Closed-loop joints (sidecar)" in tree, (
        f"expected sidecar section in tree:\n{tree}"
    )
    assert closing.name in tree, (
        f"expected closing joint '{closing.name}' in tree sidecar:\n{tree}"
    )
    # And the URDF tree itself should NOT show a multi-parent marker
    # (the topology is now clean by construction).
    assert "multi-parent" not in tree, (
        f"tree should be clean of multi-parent markers after auto-classification:\n{tree}"
    )
    print("  tree_render_marks_multi_parent: PASS")


def test_verbose_package_emits_frame_config_without_rviz_or_control():
    """Verbose frame controls do not depend on unrelated config features."""
    from ..core.robot_model import build_model
    from ..core.package_generator import generate_package
    from ..core.data_types import ExportConfig

    model = build_model(_make_snapshot(), _make_logger())
    with tempfile.TemporaryDirectory() as tmp:
        cfg = ExportConfig(
            package_name="test_robot_description",
            output_dir=tmp,
            verbosity="verbose",
            include_rviz=False,
            include_ros2_control=False,
            include_docs=False,
            include_robot_data_yaml=False,
            include_readme=False,
        )
        pkg = generate_package(model, cfg, _make_logger())
        assert os.path.isfile(
            os.path.join(pkg, "config", "frame_overrides.csv")
        )
        assert os.path.isfile(
            os.path.join(pkg, "config", "FRAME_OVERRIDES.md")
        )
        with open(os.path.join(pkg, "CMakeLists.txt"), encoding="utf-8") as f:
            cmake = f.read()
        assert "config" in cmake.split("install(DIRECTORY", 1)[1].split(")", 1)[0]

    print("  verbose_package_emits_frame_config_without_rviz_or_control: PASS")


def test_minimal_package_skips_optional_dirs():
    """A minimal-mode package generation skips rviz/, config/, debug/, README, etc."""
    import json
    from ..core.robot_model import build_model
    from ..core.package_generator import generate_package
    from ..core.data_types import ExportConfig

    fixtures = os.path.join(os.path.dirname(__file__), "fixtures")
    snap_path = os.path.join(fixtures, "snapshot.json")
    if not os.path.isfile(snap_path):
        print("  minimal_package_skips_optional_dirs: SKIPPED (no snapshot fixture)")
        return

    with open(snap_path) as f:
        data = json.load(f)
    snap = _make_snapshot(data)
    log = _make_logger()
    model = build_model(snap, log)

    with tempfile.TemporaryDirectory() as tmp:
        cfg = ExportConfig(
            package_name="basic_platform_description",
            output_dir=tmp,
            collision_auto_method="primitive",
            verbosity="minimal",
            include_debug=False,
            include_docs=False,
            include_robot_data_yaml=False,
            include_screenshot=False,
            include_launch=False,
            include_rviz=False,
            include_readme=False,
            include_ros2_control=False,
        )
        generate_package(model, cfg, log)
        pkg = os.path.join(tmp, "basic_platform_description")

        # Required things still present
        for required in ("urdf/basic_platform.urdf", "package.xml", "CMakeLists.txt"):
            assert os.path.isfile(os.path.join(pkg, required)), f"missing required: {required}"

        # Optional things gone
        for absent in ("launch", "rviz", "config", "robot_data.yaml",
                        "docs/transforms.md", "README.md"):
            full = os.path.join(pkg, absent)
            assert not os.path.exists(full), f"minimal mode should not produce: {absent}"

        # CMakeLists shouldn't reference launch/rviz/config either —
        # otherwise colcon build fails on missing dirs.
        with open(os.path.join(pkg, "CMakeLists.txt")) as f:
            cmake = f.read()
        for unwanted in ("launch ", "rviz ", "config "):
            assert unwanted not in cmake.split("install(DIRECTORY")[1].split(")")[0], \
                f"CMakeLists install rule still mentions '{unwanted.strip()}'"

    print("  minimal_package_skips_optional_dirs: PASS")


# ──────────────────────────────────────────────
# robot_data.yaml — closing joints + passive sidecar fields
# ──────────────────────────────────────────────

def _yaml_section_lines(text: str, header: str):
    """Extract the contiguous indented block under a top-level
    ``header:`` line.  Tiny helper so these tests don't pull a YAML
    parser dependency just to assert structure.  Comment lines are
    dropped so callers can ``count('passive:')`` etc. without false
    positives from prose."""
    lines = text.splitlines()
    out = []
    in_block = False
    for line in lines:
        if line.startswith(header + ":"):
            in_block = True
            continue
        if in_block:
            if line and not line.startswith(" ") and not line.startswith("\t"):
                break
            stripped = line.lstrip()
            if stripped.startswith("#"):
                continue
            out.append(line)
    return out


def test_robot_data_yaml_passive_field_per_joint():
    """Every joint in robot_data.yaml's ``joints:`` block carries an
    explicit ``passive: <bool>`` line — downstream URDF→USD pipelines
    rely on this being present, not optional."""
    from ..core.robot_model import build_model
    from ..core.supplementary_export import generate_supplementary_yaml
    from ..core.data_types import ExportConfig

    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)

    with tempfile.TemporaryDirectory() as tmp:
        cfg = ExportConfig(package_name="test_description", output_dir=tmp)
        path = generate_supplementary_yaml(model, cfg, tmp, log)
        with open(path, encoding="utf-8") as f:
            text = f.read()

    joints_block = "\n".join(_yaml_section_lines(text, "joints"))
    assert "passive:" in joints_block, \
        f"expected 'passive:' on every joint in joints: block:\n{joints_block}"
    # One ``passive:`` line per emitted joint.
    n_joints = sum(
        1 for line in joints_block.splitlines()
        if line.startswith("  ") and not line.startswith("    ") and line.rstrip().endswith(":")
    )
    n_passive = joints_block.count("passive:")
    assert n_joints == n_passive, \
        f"joint/passive mismatch: {n_joints} joints, {n_passive} passive lines"

    print("  robot_data_yaml_passive_field_per_joint: PASS")


def test_robot_data_yaml_parses_when_merged_member_has_bang_prefix():
    """``!`` is a YAML tag marker.  Raw Fusion source names that keep
    their reserved prefix (e.g. ``!frame_base_link``) used to leak
    into the ``merged_from:`` traceability list and made the entire
    document unparseable by ``yaml.safe_load``.  Regression guard:
    emit a model where a merged link's member paths include a
    ``!frame_*`` source, then prove the result parses cleanly *and*
    the leading ``!`` is stripped from the recorded member name."""
    from ..core.robot_model import build_model
    from ..core.supplementary_export import generate_supplementary_yaml
    from ..core.data_types import ExportConfig

    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)

    # Force one of the model's links into the "merged with a !frame
    # member" shape that triggered the bug.  Any link will do — we
    # only need ``is_merged`` + a member path that starts with ``!``.
    target_name = next(iter(model.links.keys()))
    target = model.links[target_name]
    target.is_merged = True
    target.merged_member_paths = [
        "!frame_base_link:1",
        "Cover v3:1",
    ]

    with tempfile.TemporaryDirectory() as tmp:
        cfg = ExportConfig(package_name="test_description", output_dir=tmp)
        path = generate_supplementary_yaml(model, cfg, tmp, log)
        with open(path, encoding="utf-8") as f:
            text = f.read()

    try:
        import yaml  # type: ignore
    except ImportError:
        # PyYAML is the canonical guard; fall back to a structural
        # check when it isn't installed in the test environment.
        assert "!frame_base_link" not in text, \
            f"raw '!frame_*' leaked into YAML:\n{text}"
        print("  robot_data_yaml_parses_when_merged_member_has_bang_prefix: PASS (no yaml)")
        return

    data = yaml.safe_load(text)
    members = data["links"][target_name]["merged_from"]
    assert "!frame_base_link:1" not in members, \
        f"unstripped '!' leaked into merged_from: {members}"
    assert "frame_base_link:1" in members, \
        f"expected stripped name in merged_from: {members}"

    print("  robot_data_yaml_parses_when_merged_member_has_bang_prefix: PASS")


def test_robot_data_yaml_closing_joints_empty_section():
    """Designs with no closing joints still emit ``closing_joints: []``
    so downstream consumers can rely on the key existing."""
    from ..core.robot_model import build_model
    from ..core.supplementary_export import generate_supplementary_yaml
    from ..core.data_types import ExportConfig

    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    assert not model.closing_joints, "fixture should have no closing joints"

    with tempfile.TemporaryDirectory() as tmp:
        cfg = ExportConfig(package_name="test_description", output_dir=tmp)
        path = generate_supplementary_yaml(model, cfg, tmp, log)
        with open(path, encoding="utf-8") as f:
            text = f.read()

    assert "closing_joints: []" in text, \
        f"expected 'closing_joints: []' for design with no closing joints:\n{text}"

    print("  robot_data_yaml_closing_joints_empty_section: PASS")


def test_robot_data_yaml_closing_joints_section_populated():
    """A design with a closing joint emits a populated
    ``closing_joints:`` section with name/source/type/parent/child/
    origin/axis fields per entry."""
    from ..core.data_types import (
        FusionSnapshot, FusionOccurrence, FusionJoint, Transform3D,
        InertiaTensor, ExportConfig,
    )
    from ..core.robot_model import build_model
    from ..core.supplementary_export import generate_supplementary_yaml

    # Same four-bar fixture as the model test, with one user-tagged
    # closing edge so the source field reads 'user_tag' (not the
    # auto-detected fallback).
    snap = FusionSnapshot(design_name="loop", design_name_clean="loop")
    snap.occurrences = {
        "asm v1:1": FusionOccurrence(
            full_path="asm v1:1", clean_name="asm",
            path_segments=["asm"], depth=0, is_subassembly=True,
            local_transform=Transform3D(),
        ),
        "asm v1:1+base v1:1": FusionOccurrence(
            full_path="asm v1:1+base v1:1", clean_name="base",
            path_segments=["asm", "base"], depth=1, mass_kg=1.0,
            body_count=1, local_transform=Transform3D(),
        ),
        "asm v1:1+mid v1:1": FusionOccurrence(
            full_path="asm v1:1+mid v1:1", clean_name="mid",
            path_segments=["asm", "mid"], depth=1, mass_kg=1.0,
            body_count=1, local_transform=Transform3D(),
        ),
        "asm v1:1+child v1:1": FusionOccurrence(
            full_path="asm v1:1+child v1:1", clean_name="child",
            path_segments=["asm", "child"], depth=1, mass_kg=1.0,
            body_count=1, local_transform=Transform3D(),
        ),
    }
    snap.joints = {
        "alpha": FusionJoint(
            name="alpha", defining_component="asm", motion_type="rigid",
            occurrence_one_path="mid v1:1", occurrence_one_clean="mid",
            occurrence_two_path="base v1:1", occurrence_two_clean="base",
            axis_vector=(0, 0, 1),
        ),
        "beta": FusionJoint(
            name="beta", defining_component="asm", motion_type="rigid",
            occurrence_one_path="child v1:1", occurrence_one_clean="child",
            occurrence_two_path="mid v1:1", occurrence_two_clean="mid",
            axis_vector=(0, 0, 1),
        ),
        "gamma": FusionJoint(
            name="gamma", raw_name="closing_gamma",
            defining_component="asm", motion_type="rigid",
            occurrence_one_path="child v1:1", occurrence_one_clean="child",
            occurrence_two_path="base v1:1", occurrence_two_clean="base",
            axis_vector=(0, 0, 1),
            is_closing=True, is_passive=True,
        ),
    }

    model = build_model(snap, _make_logger())
    assert "gamma" in model.closing_joints, "test setup: gamma must be closing"

    with tempfile.TemporaryDirectory() as tmp:
        cfg = ExportConfig(package_name="loop_description", output_dir=tmp)
        path = generate_supplementary_yaml(model, cfg, tmp, _make_logger())
        with open(path, encoding="utf-8") as f:
            text = f.read()

    closing_block = "\n".join(_yaml_section_lines(text, "closing_joints"))
    assert closing_block.strip(), f"closing_joints section empty:\n{text}"
    assert "- name: gamma" in closing_block, \
        f"expected entry for gamma:\n{closing_block}"
    assert "source: user_tag" in closing_block, \
        f"expected 'source: user_tag':\n{closing_block}"
    for required in ("type:", "parent:", "child:", "origin_xyz_m:",
                       "origin_rpy_rad:", "axis:"):
        assert required in closing_block, \
            f"missing {required} in closing_joints entry:\n{closing_block}"

    # ``gamma`` must NOT also appear under ``joints:`` (invariant: a
    # joint is in either tree or sidecar, never both).
    joints_block = "\n".join(_yaml_section_lines(text, "joints"))
    assert "gamma:" not in joints_block, \
        f"closing joint must not appear in URDF joints: block:\n{joints_block}"

    print("  robot_data_yaml_closing_joints_section_populated: PASS")


def test_urdf_omits_visual_when_export_failed():
    """When the visual mesh export fails (link.has_visual_mesh=False),
    the URDF must omit the ``<visual>`` element entirely rather than
    reference a missing OBJ/DAE file.  Likewise, no implicit
    visual-as-collision fallback when there's no real visual on disk."""
    from ..core.robot_model import build_model
    from ..core.xacro_generator import generate_urdf
    from ..core.data_types import ExportConfig

    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)

    # Pick a link and mark its visual export as failed (the same flag
    # the post-export pipeline sets in fusion_extractor when the OBJ
    # is missing or zero bytes).
    target_link = next(iter(n for n in model.links if n != "base_link"))
    link = model.links[target_link]
    link.has_visual_mesh = False
    link.mesh_visual = ""
    link.collision = None  # no explicit collision either

    config = ExportConfig(package_name="test_description")
    urdf = generate_urdf(model, config)
    root = ET.fromstring(urdf)

    # Locate our link
    target = None
    for l in root.findall("link"):
        if l.attrib.get("name") == target_link:
            target = l
            break
    assert target is not None, f"link {target_link} missing from URDF"

    # No <visual>, no <collision> — the link still exists with its
    # inertials, just no geometry.
    visuals = target.findall("visual")
    assert len(visuals) == 0, (
        f"expected no <visual> on link with failed mesh export; "
        f"got {len(visuals)}\nURDF:\n{urdf}"
    )
    collisions = target.findall("collision")
    assert len(collisions) == 0, (
        f"expected no <collision> when neither explicit STL nor visual "
        f"is available; got {len(collisions)}\nURDF:\n{urdf}"
    )
    inertials = target.findall("inertial")
    assert len(inertials) == 1, (
        f"link must keep its inertial element; got {len(inertials)}"
    )

    print("  urdf_omits_visual_when_export_failed: PASS")


def test_urdf_uses_collision_stl_when_visual_missing():
    """If the visual mesh failed but an explicit collision STL is
    present, the URDF emits ``<collision>`` (referencing the real STL)
    and skips ``<visual>``.  Validates that the visual-as-collision
    fallback isn't triggered when the visual file isn't on disk."""
    from ..core.robot_model import build_model
    from ..core.xacro_generator import generate_urdf
    from ..core.data_types import ExportConfig, CollisionInfo

    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)

    target_link = next(iter(n for n in model.links if n != "base_link"))
    link = model.links[target_link]
    link.has_visual_mesh = False
    link.mesh_visual = ""
    link.collision = CollisionInfo(
        mesh_path=f"meshes/ROOT/{target_link}_collision.stl",
        origin_xyz=(0.0, 0.0, 0.0),
    )

    config = ExportConfig(package_name="test_description")
    urdf = generate_urdf(model, config)
    root = ET.fromstring(urdf)

    target = next(l for l in root.findall("link")
                   if l.attrib.get("name") == target_link)
    assert len(target.findall("visual")) == 0
    collisions = target.findall("collision")
    assert len(collisions) == 1, "explicit collision STL must produce one <collision>"
    mesh = collisions[0].find("geometry/mesh")
    assert mesh is not None
    assert mesh.attrib["filename"].endswith("_collision.stl"), \
        f"expected collision STL ref; got {mesh.attrib['filename']}"

    print("  urdf_uses_collision_stl_when_visual_missing: PASS")


def test_per_member_obj_concat_transforms_and_namespaces():
    """``_write_concatenated_obj_mtl`` glues per-member OBJ data into
    one merged OBJ with vertices transformed into the anchor frame
    and material names namespaced per member.  Locks in the math +
    output structure that the per-member rigid-group merge relies on
    (the Fusion API path is shared with single-link export, tested
    elsewhere)."""
    import math
    from ..core.fusion_extractor import (
        _read_obj_data, _write_concatenated_obj_mtl,
    )
    from ..core.data_types import LinkNode

    with tempfile.TemporaryDirectory() as tmp:
        # Member A's OBJ — a unit triangle at origin (cm).
        a_obj = os.path.join(tmp, "a.obj")
        with open(a_obj, "w", encoding="utf-8") as f:
            f.write("mtllib a.mtl\n")
            f.write("v 0.0 0.0 0.0\nv 1.0 0.0 0.0\nv 0.0 1.0 0.0\n")
            f.write("usemtl steel\nf 1 2 3\n")
        with open(os.path.join(tmp, "a.mtl"), "w", encoding="utf-8") as f:
            f.write("newmtl steel\nKd 0.5 0.5 0.5\n")
        a_data = _read_obj_data(a_obj)
        a_data_mtl = os.path.join(tmp, "a.mtl")
        # Member B's OBJ — same vertex names + same material name (would
        # collide without namespacing).
        b_obj = os.path.join(tmp, "b.obj")
        with open(b_obj, "w", encoding="utf-8") as f:
            f.write("mtllib b.mtl\n")
            f.write("v 0.0 0.0 0.0\nv 1.0 0.0 0.0\nv 0.0 1.0 0.0\n")
            f.write("usemtl steel\nf 1 2 3\n")
        with open(os.path.join(tmp, "b.mtl"), "w", encoding="utf-8") as f:
            f.write("newmtl steel\nKd 0.1 0.2 0.3\n")
        b_data = _read_obj_data(b_obj)
        b_data_mtl = os.path.join(tmp, "b.mtl")

        # Member A: identity transform; Member B: translate +5cm in X.
        identity = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
        member_data = [
            {"name": "a", "index": 0, "obj_data": a_data,
             "mtl_path": a_data_mtl, "R": identity, "t_cm": (0.0, 0.0, 0.0)},
            {"name": "b", "index": 1, "obj_data": b_data,
             "mtl_path": b_data_mtl, "R": identity, "t_cm": (5.0, 0.0, 0.0)},
        ]
        link = LinkNode(urdf_name="merged", clean_name="merged")
        merged_obj = os.path.join(tmp, "merged.obj")
        merged_mtl = os.path.join(tmp, "merged.mtl")

        _write_concatenated_obj_mtl(
            merged_obj, merged_mtl, member_data, link, _make_logger(),
        )

        # Read back and check.  Member B's vertices got translated by +5cm.
        out = _read_obj_data(merged_obj)
        assert len(out["vertices"]) == 6, (
            f"expected 6 verts (3 per member); got {len(out['vertices'])}"
        )
        # First three are member A at origin.
        assert out["vertices"][0] == (0.0, 0.0, 0.0)
        # Last three are member B translated by +5cm.
        assert out["vertices"][3] == (5.0, 0.0, 0.0)
        assert out["vertices"][4] == (6.0, 0.0, 0.0)
        # Face indices in merged OBJ refer to the merged vertex array.
        assert len(out["faces"]) == 2
        assert out["faces"][0][0] == [1, 2, 3]
        assert out["faces"][1][0] == [4, 5, 6], (
            f"member B's face indices should be offset by 3; got "
            f"{out['faces'][1][0]}"
        )

        # Materials are namespaced per member.
        with open(merged_mtl, encoding="utf-8") as f:
            mtl_text = f.read()
        assert "a__steel" in mtl_text
        assert "b__steel" in mtl_text
        # Both kept their original Kd values — no collision.
        assert "0.5 0.5 0.5" in mtl_text
        assert "0.1 0.2 0.3" in mtl_text

    print("  per_member_obj_concat_transforms_and_namespaces: PASS")


def test_merged_obj_anchor_correction_uses_lca_relative_transform():
    """Merged OBJ vertices use the anchor pose relative to the export LCA.

    Using an anchor transform relative to an intermediate parent assembly
    displaces root-spanning rigid groups when the nested component has an
    unusual parent-local origin.
    """
    from ..core.data_types import FusionOccurrence, FusionSnapshot, Transform3D
    from ..core.fusion_extractor import (
        _maybe_apply_anchor_frame_correction, _read_obj_data,
    )
    from ..core.robot_model import _mat3_mul, _rotate_vec3_by_mat3

    rz_90 = (0.0, -1.0, 0.0,
             1.0,  0.0, 0.0,
             0.0,  0.0, 1.0)
    rx_90 = (1.0, 0.0,  0.0,
             0.0, 0.0, -1.0,
             0.0, 1.0,  0.0)
    identity = (1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0)

    def write_obj(path, vertex, normal):
        with open(path, "w", encoding="utf-8") as f:
            f.write(f"v {vertex[0]} {vertex[1]} {vertex[2]}\n")
            f.write(f"vn {normal[0]} {normal[1]} {normal[2]}\n")

    def assert_vec_close(actual, expected):
        assert all(abs(a - e) < 1e-6 for a, e in zip(actual, expected)), (
            f"expected {expected}, got {actual}"
        )

    with tempfile.TemporaryDirectory() as tmp:
        # Root LCA: local_transform is deliberately unrelated. Fusion's
        # root-targeted OBJ must instead be inverted with transform2.
        root_obj = os.path.join(tmp, "root_lca.obj")
        anchor_local_vertex = (2.0, 3.0, 4.0)  # OBJ units: cm
        anchor_local_normal = (1.0, 0.0, 0.0)
        anchor_world_translation = (0.12, -0.34, 0.56)  # metres
        rotated_vertex = _rotate_vec3_by_mat3(anchor_local_vertex, rz_90)
        root_vertex = tuple(
            rotated_vertex[i] + anchor_world_translation[i] * 100.0
            for i in range(3)
        )
        root_normal = _rotate_vec3_by_mat3(anchor_local_normal, rz_90)
        write_obj(root_obj, root_vertex, root_normal)

        root_anchor = FusionOccurrence(
            local_transform=Transform3D(
                translation=(1.94, -0.06, 0.04), rotation=identity,
            ),
            transform2=Transform3D(
                translation=anchor_world_translation, rotation=rz_90,
            ),
        )
        _maybe_apply_anchor_frame_correction(
            root_obj, root_anchor, "", FusionSnapshot(), _make_logger(),
        )
        corrected = _read_obj_data(root_obj)
        assert_vec_close(corrected["vertices"][0], anchor_local_vertex)
        assert_vec_close(corrected["normals"][0], anchor_local_normal)

        # Nested LCA: derive LCA inverse * anchor from the two world poses.
        nested_obj = os.path.join(tmp, "nested_lca.obj")
        lca_translation = (1.0, 2.0, 3.0)
        anchor_in_lca_translation = (0.2, 0.3, 0.4)
        anchor_world_rotation = _mat3_mul(rz_90, rx_90)
        anchor_world_offset = _rotate_vec3_by_mat3(
            anchor_in_lca_translation, rz_90,
        )
        nested_anchor_world_translation = tuple(
            lca_translation[i] + anchor_world_offset[i] for i in range(3)
        )
        lca_vertex_rotated = _rotate_vec3_by_mat3(
            anchor_local_vertex, rx_90,
        )
        lca_vertex = tuple(
            lca_vertex_rotated[i] + anchor_in_lca_translation[i] * 100.0
            for i in range(3)
        )
        lca_normal = _rotate_vec3_by_mat3(anchor_local_normal, rx_90)
        write_obj(nested_obj, lca_vertex, lca_normal)

        lca_path = "outer:1"
        snap = FusionSnapshot(occurrences={
            lca_path: FusionOccurrence(
                full_path=lca_path,
                transform2=Transform3D(
                    translation=lca_translation, rotation=rz_90,
                ),
            ),
        })
        nested_anchor = FusionOccurrence(
            parent_path="outer:1+middle:1",
            local_transform=Transform3D(
                translation=(9.0, 8.0, 7.0), rotation=identity,
            ),
            transform2=Transform3D(
                translation=nested_anchor_world_translation,
                rotation=anchor_world_rotation,
            ),
        )
        _maybe_apply_anchor_frame_correction(
            nested_obj, nested_anchor, lca_path, snap, _make_logger(),
        )
        corrected = _read_obj_data(nested_obj)
        assert_vec_close(corrected["vertices"][0], anchor_local_vertex)
        assert_vec_close(corrected["normals"][0], anchor_local_normal)

    print("  merged_obj_anchor_correction_uses_lca_relative_transform: PASS")


def test_invalid_fusion_joint_endpoint_is_skipped():
    """Fusion can raise RuntimeError while reading occurrenceOne/Two for
    stale imported joints.  Extraction should skip that joint instead of
    aborting the whole export."""
    from ..core.fusion_extractor import _extract_one_joint

    class FakeComponent:
        name = "imported_base v1"

    class BadJoint:
        name = "Rigid 42"
        isSuppressed = False

        @property
        def occurrenceOne(self):
            return object()

        @property
        def occurrenceTwo(self):
            raise RuntimeError("2 : InternalValidationError : res")

    log = _make_logger()
    assert _extract_one_joint(BadJoint(), "as_built", FakeComponent(), log) is None
    assert any("Rigid 42" in line and "Skipping" in line for line in log.lines)

    print("  invalid_fusion_joint_endpoint_is_skipped: PASS")


def test_root_side_invalid_joint_endpoint_uses_design_root():
    """A joint whose root-side endpoint is not an Occurrence should be kept
    as a design-root -> child edge, matching imported flat assemblies."""
    from ..core.data_types import (
        DESIGN_ROOT_OCCURRENCE_PATH, FusionOccurrence, FusionSnapshot,
        Transform3D,
    )
    from ..core.fusion_extractor import _extract_one_joint
    from ..core.robot_model import build_model

    class EmptyBodies:
        count = 0

    class RootComponent:
        name = "SPOT_MINI_v7 v1"
        bRepBodies = EmptyBodies()

    class DefiningComponent(RootComponent):
        pass

    class ChildComponent:
        name = "Component1 v1"

    class ChildOccurrence:
        fullPathName = "Component1:1"
        component = ChildComponent()
        transform = None
        transform2 = None
        assemblyContext = None

    class RootSideJoint:
        name = "Revolute3"
        isSuppressed = False
        occurrenceOne = ChildOccurrence()
        jointMotion = None

        @property
        def occurrenceTwo(self):
            raise RuntimeError("2 : InternalValidationError : res")

    snap = FusionSnapshot(design_name="SPOT_MINI_v7 v1", design_name_clean="SPOT_MINI_v7")
    snap.occurrences["Component1:1"] = FusionOccurrence(
        full_path="Component1:1",
        component_name="Component1 v1",
        clean_name="Component1",
        path_segments=["Component1"],
        depth=0,
        is_subassembly=False,
        global_position=(1.0, 0.0, 0.0),
        local_transform=Transform3D(translation=(1.0, 0.0, 0.0)),
        transform2=Transform3D(translation=(1.0, 0.0, 0.0)),
        mass_kg=1.0,
        body_count=1,
    )

    log = _make_logger()
    fj = _extract_one_joint(
        RootSideJoint(), "regular", DefiningComponent(), log,
        snapshot=snap, root_component=RootComponent(),
    )
    assert fj is not None
    assert fj.occurrence_two_path == DESIGN_ROOT_OCCURRENCE_PATH
    assert DESIGN_ROOT_OCCURRENCE_PATH in snap.occurrences

    snap.joints[fj.name] = fj
    model = build_model(snap, log)
    assert model.root_link == "base_link"
    assert model.joints["Revolute3"].parent_link == "base_link"
    assert model.joints["Revolute3"].child_link == "Component1"
    assert not model.errors, f"unexpected model errors: {model.errors}"

    print("  root_side_invalid_joint_endpoint_uses_design_root: PASS")


def test_root_component_visual_export_uses_root_bodies_directly():
    """Synthetic design-root links export root-owned bodies directly.

    The root component can also contain child occurrences (the legs in the
    Spot import).  The base visual must be built from bRepBodies owned by
    the root itself, without asking Fusion to export the whole component.
    """
    from ..core.fusion_extractor import _export_visual_obj

    class VisibleBody:
        def __init__(self, name):
            self.name = name
            self.isVisible = True

    class Bodies:
        def __init__(self, bodies):
            self._bodies = bodies

        def __iter__(self):
            return iter(self._bodies)

    class ChildComponent:
        def __init__(self, name):
            self.name = name

    class ChildOccurrence:
        def __init__(self, name):
            self.component = ChildComponent(name)
            self.isVisible = True

    class Occurrences:
        def __init__(self, occs):
            self._occs = occs

        def __iter__(self):
            return iter(self._occs)

    class RootComponent:
        def __init__(self):
            self.bRepBodies = Bodies([
                VisibleBody("Body1"),
                VisibleBody("!collision_proxy"),
            ])
            self.occurrences = Occurrences([
                ChildOccurrence("leg_upper"),
                ChildOccurrence("!collision_base"),
            ])

    class Link:
        urdf_name = "base_link"
        color_rgb = (0.7, 0.7, 0.7)

    class ObjOptions:
        def __init__(self, target, path):
            self.target = target
            self.path = path
            self.meshRefinement = None

    class ExportManager:
        def __init__(self):
            self.targets = []

        def createOBJExportOptions(self, target, obj_path):
            self.targets.append(target)
            assert isinstance(target, VisibleBody), (
                "root visual should export individual root bodies"
            )
            return ObjOptions(target, obj_path)

        def execute(self, opts):
            with open(opts.path, "w", encoding="utf-8") as f:
                f.write("v 0 0 0\nv 1 0 0\nv 0 1 0\n")
                f.write("usemtl mat\nf 1 2 3\n")
            return True

    root = RootComponent()
    export_mgr = ExportManager()

    obj_path = os.path.join(os.getcwd(), "_root_visual_export_test.obj")
    mtl_path = os.path.join(os.getcwd(), "_root_visual_export_test.mtl")
    for path in (obj_path, mtl_path):
        if os.path.exists(path):
            os.remove(path)

    try:
        ok, reason = _export_visual_obj(
            root, obj_path, mtl_path, Link(), export_mgr, refinement=0,
            log=_make_logger(),
        )
    finally:
        for path in (obj_path, mtl_path):
            if os.path.exists(path):
                os.remove(path)

    assert ok, reason
    assert [target.name for target in export_mgr.targets] == ["Body1"]
    assert [child.isVisible for child in root.occurrences] == [True, True]

    print("  root_component_visual_export_uses_root_bodies_directly: PASS")


def test_root_component_visual_export_hides_child_occurrences():
    """If body-level OBJ export is unavailable, the component fallback still
    hides child occurrences so the root visual does not duplicate legs."""
    from ..core.fusion_extractor import _export_visual_obj

    class VisibleBody:
        def __init__(self, name):
            self.name = name
            self.isVisible = True

    class Bodies:
        def __init__(self, bodies):
            self._bodies = bodies

        def __iter__(self):
            return iter(self._bodies)

    class ChildComponent:
        def __init__(self, name):
            self.name = name

    class ChildOccurrence:
        def __init__(self, name):
            self.component = ChildComponent(name)
            self.isVisible = True

    class Occurrences:
        def __init__(self, occs):
            self._occs = occs

        def __iter__(self):
            return iter(self._occs)

    class RootComponent:
        def __init__(self):
            self.bRepBodies = Bodies([VisibleBody("Body1")])
            self.occurrences = Occurrences([
                ChildOccurrence("leg_upper"),
                ChildOccurrence("collision_base"),
            ])

    class Link:
        urdf_name = "base_link"
        color_rgb = (0.7, 0.7, 0.7)

    class ObjOptions:
        def __init__(self, target, path):
            self.target = target
            self.path = path
            self.meshRefinement = None

    class ExportManager:
        def __init__(self):
            self.targets = []
            self.child_visibility_during_export = None

        def createOBJExportOptions(self, target, obj_path):
            self.targets.append(target)
            if isinstance(target, VisibleBody):
                return None
            return ObjOptions(target, obj_path)

        def execute(self, opts):
            root = opts.target
            self.child_visibility_during_export = [
                child.isVisible for child in root.occurrences
            ]
            with open(opts.path, "w", encoding="utf-8") as f:
                f.write("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
            return True

    root = RootComponent()
    export_mgr = ExportManager()

    obj_path = os.path.join(os.getcwd(), "_root_visual_export_test.obj")
    mtl_path = os.path.join(os.getcwd(), "_root_visual_export_test.mtl")
    for path in (obj_path, mtl_path):
        if os.path.exists(path):
            os.remove(path)

    try:
        ok, reason = _export_visual_obj(
            root, obj_path, mtl_path, Link(), export_mgr, refinement=0,
            log=_make_logger(),
        )
    finally:
        for path in (obj_path, mtl_path):
            if os.path.exists(path):
                os.remove(path)

    assert ok, reason
    assert export_mgr.targets[0].name == "Body1"
    assert export_mgr.targets[-1] == root
    assert export_mgr.child_visibility_during_export == [False, False]
    assert [child.isVisible for child in root.occurrences] == [True, True]

    print("  root_component_visual_export_hides_child_occurrences: PASS")


def test_acc_prefix_recognised_and_stripped():
    """Helper-level: only the bang-prefixed ``!acc_*`` metadata form is
    recognised and stripped."""
    from ..utils import (
        is_accurate_collision_group_name,
        strip_accurate_collision_prefix,
    )
    assert not is_accurate_collision_group_name("acc_gripper_jaws")
    assert not is_accurate_collision_group_name("acc-gripper")
    assert not is_accurate_collision_group_name("acc")
    assert is_accurate_collision_group_name("!acc_gripper_jaws")
    # Unrelated names pass through.
    assert not is_accurate_collision_group_name("accelerometer")  # 'acc' inside but not prefix
    assert not is_accurate_collision_group_name("gripper_jaws")
    assert not is_accurate_collision_group_name("")

    assert strip_accurate_collision_prefix("acc_gripper") == "acc_gripper"
    assert strip_accurate_collision_prefix("acc-gripper") == "acc-gripper"
    assert strip_accurate_collision_prefix("!acc_gripper") == "gripper"
    assert strip_accurate_collision_prefix("!acc") == ""
    assert strip_accurate_collision_prefix("acc") == "acc"
    assert strip_accurate_collision_prefix("plain_name") == "plain_name"

    print("  acc_prefix_recognised_and_stripped: PASS")


def test_acc_rigid_group_uses_visual_mesh_as_collision():
    """A LinkNode flagged ``wants_accurate_collision`` skips the
    bounding-primitive auto-fit and is resolved as ``visual_fallback``
    — the visual mesh becomes the collision shape.  Untagged links
    still get the primitive (cheap collision)."""
    from ..core.data_types import LinkNode, ExportConfig
    from ..core.collision_generator import _resolve_link_collision

    # Tagged link: bbox + visual mesh both set.
    tagged = LinkNode(
        urdf_name="gripper_jaws",
        clean_name="gripper_jaws",
        assembly="ROOT",
        mass_kg=0.05,
        bbox_size=(0.02, 0.05, 0.01),
        bbox_min=(-0.01, -0.025, -0.005),
        bbox_max=(0.01, 0.025, 0.005),
        volume_m3=1e-5,
        mesh_visual="meshes/ROOT/gripper_jaws.dae",
        wants_accurate_collision=True,
    )
    cfg = ExportConfig(package_name="t_description",
                        collision_auto_method="primitive")
    info = _resolve_link_collision(tagged, cfg, _make_logger(), pkg_dir="/tmp")
    assert info.source == "visual_fallback", (
        f"acc-tagged link must use visual mesh as collision; got source={info.source}"
    )
    assert info.mesh_path == tagged.mesh_visual, (
        f"acc-tagged collision should reference the visual mesh; "
        f"got {info.mesh_path}"
    )

    # Untagged link: same config → primitive auto-fit (existing behaviour).
    untagged = LinkNode(
        urdf_name="servo_mount",
        clean_name="servo_mount",
        assembly="ROOT",
        mass_kg=0.2,
        bbox_size=(0.04, 0.03, 0.02),
        bbox_min=(-0.02, -0.015, -0.01),
        bbox_max=(0.02, 0.015, 0.01),
        volume_m3=2e-5,
        mesh_visual="meshes/ROOT/servo_mount.dae",
        wants_accurate_collision=False,
    )
    info2 = _resolve_link_collision(untagged, cfg, _make_logger(), pkg_dir="/tmp")
    assert info2.source == "primitive", (
        f"untagged link should still get a primitive; got {info2.source}"
    )

    print("  acc_rigid_group_uses_visual_mesh_as_collision: PASS")


def test_collision_override_prefixes_force_method():
    """Per-link collision override prefixes beat the global TOML method."""
    from ..core.data_types import LinkNode, ExportConfig
    from ..core.collision_generator import _resolve_link_collision

    primitive_link = LinkNode(
        urdf_name="wheel",
        clean_name="wheel",
        assembly="ROOT",
        mass_kg=0.2,
        bbox_size=(0.04, 0.03, 0.02),
        bbox_min=(-0.02, -0.015, -0.01),
        bbox_max=(0.02, 0.015, 0.01),
        volume_m3=2e-5,
        mesh_visual="meshes/ROOT/wheel.obj",
        collision_override="primitive",
    )
    visual_cfg = ExportConfig(package_name="t_description",
                              collision_auto_method="visual_reuse")
    prim_info = _resolve_link_collision(
        primitive_link, visual_cfg, _make_logger(), pkg_dir="/tmp"
    )
    assert prim_info.source == "primitive"

    with tempfile.TemporaryDirectory() as tmpdir:
        obj_dir = os.path.join(tmpdir, "meshes", "ROOT")
        os.makedirs(obj_dir)
        obj_path = os.path.join(obj_dir, "body.obj")
        with open(obj_path, "w", encoding="utf-8") as f:
            f.write(
                "v 0 0 0\n"
                "v 1 0 0\n"
                "v 0 1 0\n"
                "v 0 0 1\n"
            )

        hull_link = LinkNode(
            urdf_name="body",
            clean_name="body",
            assembly="ROOT",
            mass_kg=1.0,
            bbox_size=(1.0, 1.0, 1.0),
            bbox_min=(0.0, 0.0, 0.0),
            bbox_max=(1.0, 1.0, 1.0),
            volume_m3=1.0,
            mesh_visual="meshes/ROOT/body.obj",
            collision_override="convex_hull",
        )
        primitive_cfg = ExportConfig(package_name="t_description",
                                     collision_auto_method="primitive")
        hull_info = _resolve_link_collision(
            hull_link, primitive_cfg, _make_logger(), pkg_dir=tmpdir
        )
        assert hull_info.source == "convex_hull"

    print("  collision_override_prefixes_force_method: PASS")


def test_urdf_joint_names_are_ascii_safe():
    """A Fusion joint name with a stray space (e.g. user typo
    ``"left_pivot _joint1"``) used to flow straight into the URDF
    ``<joint name="...">`` attribute.  Spaces in URDF identifiers
    crash Isaac Sim's importer because the corresponding USD prim
    path is invalid.  Joint names must be sanitized the same way
    component / material names are."""
    from ..core.data_types import (
        FusionSnapshot, FusionOccurrence, FusionJoint, Transform3D, ExportConfig,
    )
    from ..core.robot_model import build_model
    from ..core.xacro_generator import generate_urdf

    snap = FusionSnapshot(design_name="t v1", design_name_clean="t")
    snap.occurrences = {
        "t v1:1+a v1:1": FusionOccurrence(
            full_path="t v1:1+a v1:1", clean_name="a",
            path_segments=["a"], depth=0, is_subassembly=False,
            mass_kg=1.0, body_count=1, local_transform=Transform3D(),
        ),
        "t v1:1+b v1:1": FusionOccurrence(
            full_path="t v1:1+b v1:1", clean_name="b",
            path_segments=["b"], depth=0, is_subassembly=False,
            mass_kg=1.0, body_count=1, local_transform=Transform3D(),
        ),
    }
    # Joint name with space + accented char — the kind of thing Fusion
    # accepts but URDF/USD doesn't.  Plugin caches the joint under
    # the SANITIZED key, so we look up by that.
    snap.joints = {
        "left_pivot_joint1": FusionJoint(
            name="left_pivot_joint1",
            raw_name="left_pivot _jøint1",  # note space + ø
            defining_component="t",
            motion_type="continuous",
            occurrence_one_path="b v1:1", occurrence_one_clean="b",
            occurrence_two_path="a v1:1", occurrence_two_clean="a",
            origin_global_m=(0, 0, 0), origin_source="geometry.origin",
            axis_vector=(0, 0, 1),
        ),
    }
    model = build_model(snap, _make_logger())

    config = ExportConfig(package_name="t_description")
    urdf = generate_urdf(model, config)

    # No spaces or non-ASCII in any joint or link name attribute
    assert all(ord(c) < 128 for c in urdf), \
        f"URDF must be pure ASCII"
    assert 'name="left_pivot _' not in urdf, (
        f"sanitization didn't strip the space:\n{urdf}"
    )
    assert 'name="left_pivot_joint1"' in urdf

    print("  urdf_joint_names_are_ascii_safe: PASS")


def test_flat_design_emits_assembly_xacro():
    """A flat Fusion design (every component at the design root, no
    sub-assemblies) must still emit ``urdf/assemblies/<name>.urdf.xacro``
    so downstream tooling has a macro to ``xacro:include`` +
    ``xacro:macro`` instantiate.  Before this fix, flat designs got
    an empty top-level xacro (just materials, no link/joint elements
    anywhere) and downstream swap/attach machinery had nothing to wire."""
    from ..core.data_types import (
        FusionSnapshot, FusionOccurrence, FusionJoint, Transform3D, ExportConfig,
    )
    from ..core.robot_model import build_model
    from ..core.xacro_generator import generate_xacro_package

    # Three leaves directly under the design root, two joints.  Zero
    # sub-asms — exactly the gripper's topology.
    snap = FusionSnapshot(design_name="flatbot v1", design_name_clean="flatbot")
    snap.occurrences = {
        "flatbot v1:1+base v1:1": FusionOccurrence(
            full_path="flatbot v1:1+base v1:1", clean_name="base",
            path_segments=["base"], depth=0, is_subassembly=False,
            mass_kg=1.0, body_count=1, local_transform=Transform3D(),
        ),
        "flatbot v1:1+arm v1:1": FusionOccurrence(
            full_path="flatbot v1:1+arm v1:1", clean_name="arm",
            path_segments=["arm"], depth=0, is_subassembly=False,
            mass_kg=0.5, body_count=1, local_transform=Transform3D(),
        ),
        "flatbot v1:1+tool v1:1": FusionOccurrence(
            full_path="flatbot v1:1+tool v1:1", clean_name="tool",
            path_segments=["tool"], depth=0, is_subassembly=False,
            mass_kg=0.1, body_count=1, local_transform=Transform3D(),
        ),
    }
    snap.joints = {
        "j_arm": FusionJoint(
            name="j_arm", defining_component="flatbot", motion_type="revolute",
            occurrence_one_path="arm v1:1", occurrence_one_clean="arm",
            occurrence_two_path="base v1:1", occurrence_two_clean="base",
            origin_global_m=(0, 0, 0.05), origin_source="geometry.origin",
            axis_vector=(0, 0, 1),
        ),
        "j_tool": FusionJoint(
            name="j_tool", defining_component="flatbot", motion_type="rigid",
            occurrence_one_path="tool v1:1", occurrence_one_clean="tool",
            occurrence_two_path="arm v1:1", occurrence_two_clean="arm",
            origin_global_m=(0, 0, 0.20), origin_source="geometry.origin",
            axis_vector=(0, 0, 1),
        ),
    }
    model = build_model(snap, _make_logger())

    # Synthetic root assembly named after the design must exist.
    assert "flatbot" in model.assemblies, (
        f"flat design must produce a synthetic 'flatbot' assembly; "
        f"got {list(model.assemblies.keys())}"
    )
    asm = model.assemblies["flatbot"]
    # All three leaves should be in this assembly's link list, by their
    # URDF names — ``base`` was renamed to ``base_link`` (REP 120
    # convention; root link of the design).
    assert {"base_link", "arm", "tool"}.issubset(set(asm.links)), (
        f"synthetic root assembly missing leaves; got {asm.links}"
    )

    config = ExportConfig(package_name="flatbot_description")
    with tempfile.TemporaryDirectory() as tmp:
        files = generate_xacro_package(model, config, tmp)

        # Top-level xacro xacro:includes the synthetic assembly, not nothing.
        top_path = os.path.join(tmp, "urdf", "flatbot.urdf.xacro")
        assert os.path.isfile(top_path), "top-level xacro missing"
        top = open(top_path, encoding="utf-8").read()
        assert "xacro:include" in top, (
            f"top-level xacro must include the synthetic assembly:\n{top}"
        )
        assert "assemblies/flatbot.urdf.xacro" in top
        assert "<xacro:flatbot prefix=" in top

        # The assembly xacro itself exists, defines the macro, and
        # carries every link + joint.
        asm_path = os.path.join(tmp, "urdf", "assemblies", "flatbot.urdf.xacro")
        assert os.path.isfile(asm_path), (
            "synthetic assembly xacro not generated; phase 2 swap/attach "
            "would have no macro to bind"
        )
        asm_xacro = open(asm_path, encoding="utf-8").read()
        assert '<xacro:macro name="flatbot"' in asm_xacro
        for link_name in ("base_link", "arm", "tool"):
            assert f'name="${{prefix}}{link_name}"' in asm_xacro, (
                f"link '{link_name}' missing from synthetic assembly:\n{asm_xacro}"
            )
        for joint_name in ("j_arm", "j_tool"):
            assert f'name="${{prefix}}{joint_name}"' in asm_xacro, (
                f"joint '{joint_name}' missing from synthetic assembly:\n{asm_xacro}"
            )

    print("  flat_design_emits_assembly_xacro: PASS")


def test_safe_identifier_strips_non_ascii_for_urdf():
    """Non-ASCII material names (downloaded models with Chinese,
    Cyrillic, accented Latin) crash Isaac Sim's URDF importer with
    ``LLVM ERROR: out of memory`` after a partial path-rewrite.  The
    plugin's ``safe_identifier`` must strip those at extraction time
    so the URDF only ever carries ASCII-safe ``name="..."`` attributes."""
    from ..utils import safe_identifier

    # Direct unit checks
    assert safe_identifier("钢") == "unnamed", \
        "all-non-ASCII collapses to fallback"
    assert safe_identifier("Steel 钢") == "Steel", \
        "trailing non-ASCII stripped, kept ASCII chars"
    assert safe_identifier("Tøråk") == "T_r_k"
    assert safe_identifier("") == "unnamed"
    assert safe_identifier(None) == "unnamed"  # type: ignore[arg-type]
    # Leading-digit guard (USD/URDF rule)
    assert safe_identifier("3M_tape") == "_3M_tape"
    # Custom fallback
    assert safe_identifier("钢", fallback="material") == "material"
    # ASCII-clean names pass through unchanged
    assert safe_identifier("aluminum_6061") == "aluminum_6061"

    print("  safe_identifier_strips_non_ascii_for_urdf: PASS")


def test_numeric_assembly_name_is_xacro_safe():
    """Part-number assembly names must remain valid XML/xacro tag names."""
    from ..core.xacro_generator import XACRO_NS, _xacro_macro_name

    assert _xacro_macro_name("turret") == "turret"
    assert _xacro_macro_name("10034_Servo_Pack") == "assembly_10034_Servo_Pack"
    assert _xacro_macro_name("10034 Servo Pack") == "assembly_10034_Servo_Pack"

    macro_name = _xacro_macro_name("10034_Servo_Pack")
    ET.fromstring(
        f'<robot xmlns:xacro="{XACRO_NS}">'
        f'<xacro:{macro_name} prefix=""/>'
        f'</robot>'
    )

    print("  numeric_assembly_name_is_xacro_safe: PASS")


def test_urdf_material_names_are_ascii_safe():
    """End-to-end: a link with a non-ASCII material name produces a
    URDF whose ``<material name="...">`` element is ASCII-only.  This
    is the regression we need to ship — non-sanitized names crash
    Isaac Sim's URDF importer."""
    from ..core.robot_model import build_model
    from ..core.xacro_generator import generate_urdf
    from ..core.data_types import ExportConfig

    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)

    # Simulate what the Fusion extractor would produce with a
    # localized material name (post-sanitization the value should
    # already be ASCII; we set raw to mimic a hypothetical bypass).
    target = next(iter(n for n in model.links if n != "base_link"))
    model.links[target].material_name = "钢"  # raw, unsanitized
    # Apply the same sanitizer the extractor would have applied
    from ..utils import safe_identifier
    model.links[target].material_name = safe_identifier(
        model.links[target].material_name, fallback="material"
    )

    config = ExportConfig(package_name="test_description")
    urdf = generate_urdf(model, config)

    # No non-ASCII in any attribute value
    assert all(ord(c) < 128 for c in urdf), (
        f"URDF must be pure ASCII; found non-ASCII chars: "
        f"{[c for c in urdf if ord(c) >= 128]}"
    )
    # Specifically, no Chinese ``钢``
    assert "钢" not in urdf

    print("  urdf_material_names_are_ascii_safe: PASS")


def test_minimal_mode_still_emits_yaml_when_closing_joints_present():
    """Minimal mode skips ``robot_data.yaml`` for normal designs, but
    a design with closing joints must emit it anyway — the
    ``closing_joints:`` sidecar lives there and is REQUIRED for
    downstream URDF→USD pipelines to author the corresponding USD
    physics joints. Without the override, a minimal export of a
    closed-kinematic-chain design ships a package that's silently
    broken downstream."""
    import json
    from ..core.data_types import (
        FusionSnapshot, FusionOccurrence, FusionJoint, Transform3D, ExportConfig,
    )
    from ..core.robot_model import build_model
    from ..core.package_generator import generate_package

    # Build a tiny snapshot with one user-tagged closing joint.
    snap = FusionSnapshot(design_name="loop", design_name_clean="loop")
    snap.occurrences = {
        "asm v1:1": FusionOccurrence(
            full_path="asm v1:1", clean_name="asm",
            path_segments=["asm"], depth=0, is_subassembly=True,
            local_transform=Transform3D(),
        ),
        "asm v1:1+base v1:1": FusionOccurrence(
            full_path="asm v1:1+base v1:1", clean_name="base",
            path_segments=["asm", "base"], depth=1, mass_kg=1.0,
            body_count=1, local_transform=Transform3D(),
        ),
        "asm v1:1+mid v1:1": FusionOccurrence(
            full_path="asm v1:1+mid v1:1", clean_name="mid",
            path_segments=["asm", "mid"], depth=1, mass_kg=1.0,
            body_count=1, local_transform=Transform3D(),
        ),
        "asm v1:1+child v1:1": FusionOccurrence(
            full_path="asm v1:1+child v1:1", clean_name="child",
            path_segments=["asm", "child"], depth=1, mass_kg=1.0,
            body_count=1, local_transform=Transform3D(),
        ),
    }
    snap.joints = {
        "alpha": FusionJoint(
            name="alpha", defining_component="asm", motion_type="rigid",
            occurrence_one_path="mid v1:1", occurrence_one_clean="mid",
            occurrence_two_path="base v1:1", occurrence_two_clean="base",
            axis_vector=(0, 0, 1),
        ),
        "beta": FusionJoint(
            name="beta", defining_component="asm", motion_type="rigid",
            occurrence_one_path="child v1:1", occurrence_one_clean="child",
            occurrence_two_path="mid v1:1", occurrence_two_clean="mid",
            axis_vector=(0, 0, 1),
        ),
        "gamma": FusionJoint(
            name="gamma", raw_name="closing_gamma",
            defining_component="asm", motion_type="rigid",
            occurrence_one_path="child v1:1", occurrence_one_clean="child",
            occurrence_two_path="base v1:1", occurrence_two_clean="base",
            axis_vector=(0, 0, 1),
            is_closing=True, is_passive=True,
        ),
    }
    model = build_model(snap, _make_logger())
    assert "gamma" in model.closing_joints

    with tempfile.TemporaryDirectory() as tmp:
        cfg = ExportConfig(
            package_name="loop_description",
            output_dir=tmp,
            verbosity="minimal",
            include_docs=False,
            include_robot_data_yaml=False,  # user opts out…
            include_screenshot=False,
            include_launch=False,
            include_rviz=False,
            include_readme=False,
            include_debug=False,
        )
        generate_package(model, cfg, _make_logger())
        pkg = os.path.join(tmp, "loop_description")

        rd_path = os.path.join(pkg, "robot_data.yaml")
        assert os.path.isfile(rd_path), (
            "robot_data.yaml must be emitted even in minimal mode when "
            "closing_joints exist (sidecar is required for platform "
            "pipeline to reconstruct the loop)"
        )
        with open(rd_path, encoding="utf-8") as f:
            text = f.read()
        assert "closing_joints:" in text
        assert "- name: gamma" in text

        # Warning surfaced so the user knows minimal-mode was overridden.
        forced_warnings = [
            w for w in model.warnings
            if "closing joint" in w.lower() and "minimal" in w.lower()
        ]
        assert forced_warnings, (
            f"expected a warning explaining the override; got: {model.warnings}"
        )

    print("  minimal_mode_still_emits_yaml_when_closing_joints_present: PASS")


def test_closing_joints_fixture_well_formed():
    """The shared fixture is structurally sane — round-trip via the
    same Python YAML loader downstream consumers use ensures the
    format stays consumable from both sides."""
    fixtures_dir = os.path.join(os.path.dirname(__file__), "fixtures")
    fixture = os.path.join(fixtures_dir, "closing_joints_sample.yaml")
    assert os.path.isfile(fixture), \
        f"missing shared fixture: {fixture}"

    with open(fixture, encoding="utf-8") as f:
        text = f.read()

    # Top-level keys.
    assert "robot_name:" in text
    assert "joints:" in text
    assert "closing_joints:" in text

    # Every joint has a passive flag.
    joints_block = "\n".join(_yaml_section_lines(text, "joints"))
    n_joint_headers = sum(
        1 for line in joints_block.splitlines()
        if line.startswith("  ") and not line.startswith("    ") and line.rstrip().endswith(":")
    )
    n_passive = joints_block.count("passive:")
    assert n_joint_headers == n_passive, \
        f"fixture: {n_joint_headers} joints but {n_passive} passive: lines"

    closing_block = "\n".join(_yaml_section_lines(text, "closing_joints"))
    assert "- name: left_slider" in closing_block
    assert "- name: right_slider" in closing_block
    assert "source:" in closing_block

    # Verify with PyYAML if it's installed (preferred) — falls back to
    # the structural-only check above otherwise.  Pipeline side will
    # always have PyYAML, so any subtle indent or quoting issues we
    # catch on either repo's CI.
    try:
        import yaml  # type: ignore
    except ImportError:
        print("  closing_joints_fixture_well_formed: PASS (no pyyaml)")
        return

    parsed = yaml.safe_load(text)
    assert isinstance(parsed, dict), "fixture must parse as a YAML mapping"
    assert "joints" in parsed and "closing_joints" in parsed
    assert isinstance(parsed["closing_joints"], list)
    assert {e["name"] for e in parsed["closing_joints"]} == {
        "left_slider", "right_slider"
    }
    for entry in parsed["closing_joints"]:
        assert entry["source"] in ("user_tag", "auto_detected")
        assert entry["type"] in ("revolute", "continuous", "prismatic", "fixed")
        assert "parent" in entry and "child" in entry

    print("  closing_joints_fixture_well_formed: PASS")


if __name__ == '__main__':
    run_all()
