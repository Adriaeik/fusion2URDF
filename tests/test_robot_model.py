"""
Tests for Phase 2: Robot Model Builder.

Run from the PARENT directory of fusion2URDF/:
    python -m fusion2URDF.tests.test_robot_model

Tests the full model build pipeline against real snapshot.json data
and validates kinematic chain properties.
"""

import json
import os
import sys

# Test data: inline minimal snapshot for unit tests (no file dependency)
MINI_SNAPSHOT = {
    "design_name": "test_robot v1",
    "design_name_clean": "test_robot",
    "root_component_name": "test_robot",
    "export_timestamp": "2026-01-01T00:00:00",
    "total_occurrences": 6,
    "total_subassemblies": 1,
    "total_leaf_components": 5,
    "total_joints": 4,
    "total_regular_joints": 0,
    "total_as_built_joints": 4,
    "max_nesting_depth": 1,
    "occurrences": {
        "arm v1:1": {
            "full_path": "arm v1:1",
            "component_name": "arm v1",
            "clean_name": "arm",
            "path_segments": ["arm"],
            "depth": 0,
            "is_subassembly": True,
            "global_position": [0.0, 0.0, 0.0],
            "local_transform": {"translation": [0.0, 0.0, 0.0], "rotation": [[1,0,0],[0,1,0],[0,0,1]], "is_identity": True},
            "assembly_context_depth": 0,
            "mass_kg": 0.0, "body_count": 0, "bodies": [],
            "com_component_local": [0,0,0], "com_global": [0,0,0],
            "inertia_at_origin": {"ixx":0,"iyy":0,"izz":0,"ixy":0,"ixz":0,"iyz":0},
            "inertia_at_com": {"ixx":0,"iyy":0,"izz":0,"ixy":0,"ixz":0,"iyz":0},
            "bbox_size": [0,0,0], "volume_m3": 0, "area_m2": 0,
            "material_name": "", "appearance_name": "", "appearance_color_rgb": None,
        },
        "arm v1:1+base_link v1:1": {
            "full_path": "arm v1:1+base_link v1:1",
            "component_name": "base_link v1",
            "clean_name": "base_link",
            "path_segments": ["arm", "base_link"],
            "depth": 1,
            "is_subassembly": False,
            "global_position": [0.0, 0.0, 0.0],
            "local_transform": {"translation": [0.0, 0.0, 0.0], "rotation": [[1,0,0],[0,1,0],[0,0,1]], "is_identity": True},
            "assembly_context_depth": 1,
            "mass_kg": 5.0, "body_count": 1, "bodies": [],
            "com_component_local": [0.0, 0.0, 0.05], "com_global": [0.0, 0.0, 0.05],
            "inertia_at_origin": {"ixx":0.01,"iyy":0.01,"izz":0.005,"ixy":0,"ixz":0,"iyz":0},
            "inertia_at_com": {"ixx":0.008,"iyy":0.008,"izz":0.005,"ixy":0,"ixz":0,"iyz":0},
            "bbox_size": [0.2, 0.2, 0.1], "volume_m3": 0.001, "area_m2": 0.05,
            "material_name": "Steel", "appearance_name": "Steel - Satin", "appearance_color_rgb": [0.6, 0.6, 0.6],
        },
        "arm v1:1+link1 v1:1": {
            "full_path": "arm v1:1+link1 v1:1",
            "component_name": "link1 v1",
            "clean_name": "link1",
            "path_segments": ["arm", "link1"],
            "depth": 1,
            "is_subassembly": False,
            "global_position": [0.0, 0.0, 0.1],
            "local_transform": {"translation": [0.0, 0.0, 0.1], "rotation": [[1,0,0],[0,1,0],[0,0,1]], "is_identity": False},
            "assembly_context_depth": 1,
            "mass_kg": 2.0, "body_count": 1, "bodies": [],
            "com_component_local": [0.0, 0.0, 0.15], "com_global": [0.0, 0.0, 0.25],
            "inertia_at_origin": {"ixx":0.005,"iyy":0.005,"izz":0.001,"ixy":0,"ixz":0,"iyz":0},
            "inertia_at_com": {"ixx":0.003,"iyy":0.003,"izz":0.001,"ixy":0,"ixz":0,"iyz":0},
            "bbox_size": [0.05, 0.05, 0.3], "volume_m3": 0.0005, "area_m2": 0.03,
            "material_name": "Aluminum", "appearance_name": "Aluminum", "appearance_color_rgb": [0.8, 0.8, 0.8],
        },
        "arm v1:1+link2 v1:1": {
            "full_path": "arm v1:1+link2 v1:1",
            "component_name": "link2 v1",
            "clean_name": "link2",
            "path_segments": ["arm", "link2"],
            "depth": 1,
            "is_subassembly": False,
            "global_position": [0.0, 0.0, 0.4],
            "local_transform": {"translation": [0.0, 0.0, 0.3], "rotation": [[1,0,0],[0,1,0],[0,0,1]], "is_identity": False},
            "assembly_context_depth": 1,
            "mass_kg": 1.5, "body_count": 1, "bodies": [],
            "com_component_local": [0.0, 0.0, 0.1], "com_global": [0.0, 0.0, 0.5],
            "inertia_at_origin": {"ixx":0.003,"iyy":0.003,"izz":0.0005,"ixy":0,"ixz":0,"iyz":0},
            "inertia_at_com": {"ixx":0.002,"iyy":0.002,"izz":0.0005,"ixy":0,"ixz":0,"iyz":0},
            "bbox_size": [0.04, 0.04, 0.2], "volume_m3": 0.0003, "area_m2": 0.02,
            "material_name": "Aluminum", "appearance_name": "Aluminum", "appearance_color_rgb": [0.8, 0.8, 0.8],
        },
        "arm v1:1+tool v1:1": {
            "full_path": "arm v1:1+tool v1:1",
            "component_name": "tool v1",
            "clean_name": "tool",
            "path_segments": ["arm", "tool"],
            "depth": 1,
            "is_subassembly": False,
            "global_position": [0.0, 0.0, 0.6],
            "local_transform": {"translation": [0.0, 0.0, 0.2], "rotation": [[1,0,0],[0,1,0],[0,0,1]], "is_identity": False},
            "assembly_context_depth": 1,
            "mass_kg": 0.5, "body_count": 1, "bodies": [],
            "com_component_local": [0.0, 0.0, 0.025], "com_global": [0.0, 0.0, 0.625],
            "inertia_at_origin": {"ixx":0.001,"iyy":0.001,"izz":0.0002,"ixy":0,"ixz":0,"iyz":0},
            "inertia_at_com": {"ixx":0.0008,"iyy":0.0008,"izz":0.0002,"ixy":0,"ixz":0,"iyz":0},
            "bbox_size": [0.03, 0.03, 0.05], "volume_m3": 0.0001, "area_m2": 0.01,
            "material_name": "Steel", "appearance_name": "Steel", "appearance_color_rgb": [0.5, 0.5, 0.5],
        },
    },
    "joints": {
        "joint1": {
            "name": "joint1", "joint_source": "as_built", "defining_component": "arm",
            "motion_type": "revolute",
            "occurrence_one_path": "link1 v1:1", "occurrence_one_clean": "link1",
            "occurrence_two_path": "base_link v1:1", "occurrence_two_clean": "base_link",
            "origin_global_m": [0.0, 0.0, 0.1],
            "origin_source": "geometry.origin",
            "axis_vector": [0.0, 0.0, 1.0],
            "has_rotation_limits": True, "rotation_min": -3.14, "rotation_max": 3.14,
            "has_slide_limits": False, "slide_min_m": None, "slide_max_m": None,
        },
        "joint2": {
            "name": "joint2", "joint_source": "as_built", "defining_component": "arm",
            "motion_type": "revolute",
            "occurrence_one_path": "link2 v1:1", "occurrence_one_clean": "link2",
            "occurrence_two_path": "link1 v1:1", "occurrence_two_clean": "link1",
            "origin_global_m": [0.0, 0.0, 0.4],
            "origin_source": "geometry.origin",
            "axis_vector": [0.0, 1.0, 0.0],
            "has_rotation_limits": True, "rotation_min": -1.57, "rotation_max": 1.57,
            "has_slide_limits": False, "slide_min_m": None, "slide_max_m": None,
        },
        "joint3": {
            "name": "joint3", "joint_source": "as_built", "defining_component": "arm",
            "motion_type": "rigid",
            "occurrence_one_path": "tool v1:1", "occurrence_one_clean": "tool",
            "occurrence_two_path": "link2 v1:1", "occurrence_two_clean": "link2",
            "origin_global_m": [0.0, 0.0, 0.6],
            "origin_source": "geometry.origin",
            "axis_vector": [0.0, 0.0, 1.0],
            "has_rotation_limits": False, "rotation_min": None, "rotation_max": None,
            "has_slide_limits": False, "slide_min_m": None, "slide_max_m": None,
        },
    },
}


def _make_snapshot(data=None):
    """Create a FusionSnapshot from dict data."""
    from ..core.data_types import (
        FusionSnapshot, FusionOccurrence, FusionJoint,
        InertiaTensor, Transform3D, RigidGroupInfo,
    )
    
    if data is None:
        data = MINI_SNAPSHOT
    
    snap = FusionSnapshot(
        design_name=data['design_name'],
        design_name_clean=data['design_name_clean'],
        root_component_name=data.get('root_component_name', ''),
        export_timestamp=data.get('export_timestamp', ''),
        document_length_unit=data.get('document_length_unit', 'cm'),
        total_occurrences=data.get('total_occurrences', 0),
        total_subassemblies=data.get('total_subassemblies', 0),
        total_leaf_components=data.get('total_leaf_components', 0),
        total_joints=data.get('total_joints', 0),
        total_regular_joints=data.get('total_regular_joints', 0),
        total_as_built_joints=data.get('total_as_built_joints', 0),
        max_nesting_depth=data.get('max_nesting_depth', 0),
    )
    
    for path, occ_data in data['occurrences'].items():
        inertia_com = occ_data.get('inertia_at_com', {})
        inertia_orig = occ_data.get('inertia_at_origin', {})
        
        def _to_transform(td):
            if not td:
                return None
            tt = td.get('translation', [0, 0, 0])
            tr = td.get('rotation', [[1,0,0],[0,1,0],[0,0,1]])
            if tr and isinstance(tr[0], list):
                tr = tuple(v for row in tr for v in row)
            else:
                tr = tuple(tr) if tr else (1,0,0, 0,1,0, 0,0,1)
            return Transform3D(translation=tuple(tt), rotation=tr)

        trans = _to_transform(occ_data.get('local_transform', {})) or Transform3D()
        # transform2 is the bug-fixed local-to-world per Fusion's API.
        # Without it the test path falls back to local_transform, which
        # for nested occurrences differs significantly and silently
        # masks any merge/inertia bug that depends on the right rotation.
        trans2 = _to_transform(occ_data.get('transform2'))

        occ = FusionOccurrence(
            full_path=occ_data['full_path'],
            component_name=occ_data.get('component_name', ''),
            clean_name=occ_data['clean_name'],
            path_segments=occ_data['path_segments'],
            depth=occ_data['depth'],
            is_subassembly=occ_data['is_subassembly'],
            is_frame_only=occ_data.get('is_frame_only', False),
            global_position=tuple(occ_data['global_position']),
            local_transform=trans,
            transform2=trans2,
            assembly_context_depth=occ_data.get('assembly_context_depth', 0),
            mass_kg=occ_data.get('mass_kg', 0),
            body_count=occ_data.get('body_count', 0),
            com_component_local=tuple(occ_data.get('com_component_local', [0, 0, 0])),
            com_global=tuple(occ_data.get('com_global', [0, 0, 0])),
            inertia_at_origin=InertiaTensor(**inertia_orig) if inertia_orig else InertiaTensor(),
            inertia_at_com=InertiaTensor(**inertia_com) if inertia_com else InertiaTensor(),
            bbox_size=tuple(occ_data.get('bbox_size', [0, 0, 0])),
            bbox_min=tuple(occ_data.get('bbox_min', [0, 0, 0])),
            bbox_max=tuple(occ_data.get('bbox_max', [0, 0, 0])),
            volume_m3=occ_data.get('volume_m3', 0),
            area_m2=occ_data.get('area_m2', 0),
            material_name=occ_data.get('material_name', ''),
            appearance_name=occ_data.get('appearance_name', ''),
            appearance_color_rgb=tuple(occ_data['appearance_color_rgb']) if occ_data.get('appearance_color_rgb') else None,
        )
        snap.occurrences[path] = occ
    
    for jname, j_data in data.get('joints', {}).items():
        joint = FusionJoint(
            name=j_data['name'],
            joint_source=j_data.get('joint_source', 'as_built'),
            defining_component=j_data.get('defining_component', ''),
            motion_type=j_data.get('motion_type', 'rigid'),
            occurrence_one_path=j_data.get('occurrence_one_path', ''),
            occurrence_one_clean=j_data.get('occurrence_one_clean', ''),
            occurrence_two_path=j_data.get('occurrence_two_path', ''),
            occurrence_two_clean=j_data.get('occurrence_two_clean', ''),
            origin_global_m=tuple(j_data.get('origin_global_m', [0, 0, 0])),
            origin_source=j_data.get('origin_source', ''),
            axis_vector=tuple(j_data.get('axis_vector', [0, 0, 1])),
            has_rotation_limits=j_data.get('has_rotation_limits', False),
            rotation_min=j_data.get('rotation_min'),
            rotation_max=j_data.get('rotation_max'),
            has_slide_limits=j_data.get('has_slide_limits', False),
            slide_min_m=j_data.get('slide_min_m'),
            slide_max_m=j_data.get('slide_max_m'),
        )
        snap.joints[jname] = joint
    
    # Rigid groups
    for rg_data in data.get('rigid_groups', []):
        rg = RigidGroupInfo(
            name=rg_data.get('name', ''),
            occurrence_paths=rg_data.get('occurrence_paths', []),
            member_clean_names=rg_data.get('member_clean_names', []),
            collision_member=rg_data.get('collision_member'),
            collision_path=rg_data.get('collision_path'),
        )
        snap.rigid_groups.append(rg)
    
    return snap


def _make_logger():
    """Create a silent logger for testing."""
    from ..utils.logger import Logger
    return Logger(timestamps=False)


# ──────────────────────────────────────────────
# Unit Tests
# ──────────────────────────────────────────────

def test_build_mini_model():
    """Build model from minimal test data and check structure."""
    from ..core.robot_model import build_model
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    # Basic counts
    assert len(model.links) == 4, f"Expected 4 links, got {len(model.links)}"
    assert len(model.joints) == 3, f"Expected 3 joints, got {len(model.joints)}"
    
    # Root detected correctly (base_link is only parent-never-child)
    assert model.root_link == "base_link", f"Expected root='base_link', got '{model.root_link}'"
    
    # All expected links present
    for name in ['base_link', 'link1', 'link2', 'tool']:
        assert name in model.links, f"Missing link: {name}"
    
    # Joint types correct
    assert model.joints['joint1'].joint_type == 'revolute'
    assert model.joints['joint2'].joint_type == 'revolute'
    assert model.joints['joint3'].joint_type == 'fixed'
    
    # No errors
    assert len(model.errors) == 0, f"Unexpected errors: {model.errors}"
    
    print("  build_mini_model: PASS")


def test_root_detection():
    """Root = parent-only node with most descendants."""
    from ..core.robot_model import build_model
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    assert model.root_link == "base_link"
    
    # base_link should have 3 descendants (link1 → link2 → tool)
    reachable = set()
    frontier = [model.root_link]
    while frontier:
        current = frontier.pop()
        if current in reachable:
            continue
        reachable.add(current)
        for j in model.joints.values():
            if j.parent_link == current:
                frontier.append(j.child_link)
    
    assert reachable == {'base_link', 'link1', 'link2', 'tool'}
    
    print("  root_detection: PASS")


def test_kinematic_chain_order():
    """Verify parent → child relationships match expected chain."""
    from ..core.robot_model import build_model
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    # Expected chain: base_link → link1 → link2 → tool
    chain = {
        'joint1': ('base_link', 'link1'),
        'joint2': ('link1', 'link2'),
        'joint3': ('link2', 'tool'),
    }
    
    for jname, (expected_parent, expected_child) in chain.items():
        j = model.joints[jname]
        assert j.parent_link == expected_parent, \
            f"{jname}: expected parent={expected_parent}, got {j.parent_link}"
        assert j.child_link == expected_child, \
            f"{jname}: expected child={expected_child}, got {j.child_link}"
    
    print("  kinematic_chain_order: PASS")


def test_joint_origins_parent_relative():
    """Joint origins should be expressed relative to parent link."""
    from ..core.robot_model import build_model
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    # joint1: base_link(0,0,0) → link1. Joint at (0,0,0.1) global.
    # Parent-relative = (0,0,0.1) - (0,0,0) = (0,0,0.1)
    j1 = model.joints['joint1']
    assert abs(j1.origin_xyz[2] - 0.1) < 1e-6, f"joint1 z: expected 0.1, got {j1.origin_xyz[2]}"
    
    # joint2: link1(0,0,0.1) → link2. Joint at (0,0,0.4) global.
    # Parent-relative = (0,0,0.4) - (0,0,0.1) = (0,0,0.3)
    j2 = model.joints['joint2']
    assert abs(j2.origin_xyz[2] - 0.3) < 1e-6, f"joint2 z: expected 0.3, got {j2.origin_xyz[2]}"
    
    # joint3 (fixed): link2(0,0,0.4) → tool(0,0,0.6)
    # Origin = child - parent = (0,0,0.2)
    j3 = model.joints['joint3']
    assert abs(j3.origin_xyz[2] - 0.2) < 1e-6, f"joint3 z: expected 0.2, got {j3.origin_xyz[2]}"
    
    print("  joint_origins_parent_relative: PASS")


def test_joint_limits():
    """Revolute joints should have limits, fixed should not."""
    from ..core.robot_model import build_model
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    j1 = model.joints['joint1']
    assert j1.limits is not None, "joint1 should have limits"
    assert abs(j1.limits.lower - (-3.14)) < 0.01
    assert abs(j1.limits.upper - 3.14) < 0.01
    
    j3 = model.joints['joint3']
    assert j3.limits is None, "joint3 (fixed) should have no limits"
    
    print("  joint_limits: PASS")


def test_name_collision_resolution():
    """Test that duplicate names get assembly prefix."""
    from ..core.robot_model import build_model
    from ..core.data_types import FusionSnapshot, FusionOccurrence, FusionJoint, Transform3D, InertiaTensor
    
    log = _make_logger()
    
    # Build a snapshot with name collision: two assemblies each with "base_link"
    snap = FusionSnapshot(design_name="test v1", design_name_clean="test")
    
    def make_occ(path, name, segs, is_sub=False, pos=(0,0,0), mass=1.0):
        return FusionOccurrence(
            full_path=path, clean_name=name, path_segments=segs,
            depth=len(segs)-1, is_subassembly=is_sub,
            global_position=pos, mass_kg=mass,
            com_component_local=(0,0,0), com_global=pos,
            inertia_at_com=InertiaTensor(), inertia_at_origin=InertiaTensor(),
            local_transform=Transform3D(),
        )
    
    snap.occurrences = {
        "asm_a v1:1": make_occ("asm_a v1:1", "asm_a", ["asm_a"], is_sub=True),
        "asm_b v1:1": make_occ("asm_b v1:1", "asm_b", ["asm_b"], is_sub=True),
        "asm_a v1:1+base_link v1:1": make_occ(
            "asm_a v1:1+base_link v1:1", "base_link", ["asm_a", "base_link"],
            pos=(0, 0, 0), mass=10.0),
        "asm_b v1:1+base_link v1:1": make_occ(
            "asm_b v1:1+base_link v1:1", "base_link", ["asm_b", "base_link"],
            pos=(0, 0, 0.5), mass=5.0),
        "asm_a v1:1+arm v1:1": make_occ(
            "asm_a v1:1+arm v1:1", "arm", ["asm_a", "arm"],
            pos=(0, 0, 0.1), mass=2.0),
    }
    
    snap.joints = {
        "j1": FusionJoint(
            name="j1", defining_component="asm_a", motion_type="revolute",
            occurrence_one_path="arm v1:1", occurrence_one_clean="arm",
            occurrence_two_path="base_link v1:1", occurrence_two_clean="base_link",
            origin_global_m=(0, 0, 0.1), origin_source="geometry.origin",
            axis_vector=(0, 0, 1),
        ),
        "mount": FusionJoint(
            name="mount", defining_component="test", motion_type="rigid",
            occurrence_one_path="asm_b v1:1+base_link v1:1", occurrence_one_clean="base_link",
            occurrence_two_path="asm_a v1:1+base_link v1:1", occurrence_two_clean="base_link",
            origin_global_m=(0, 0, 0.5), origin_source="occ_one_global",
            axis_vector=(0, 0, 1),
        ),
    }
    
    model = build_model(snap, log)
    
    # Root should be asm_a/base_link (most descendants)
    assert model.root_link == "base_link", f"Root: {model.root_link}"
    
    # The other base_link should be prefixed
    assert "asm_b_base_link" in model.links, \
        f"Expected 'asm_b_base_link' in links, got: {list(model.links.keys())}"
    
    # All names unique
    names = list(model.links.keys())
    assert len(names) == len(set(names)), f"Duplicate names: {names}"
    
    print("  name_collision_resolution: PASS")


def test_assembly_hierarchy():
    """Test assembly detection and link assignment."""
    from ..core.robot_model import build_model
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    # Should detect 'arm' as an assembly
    assert 'arm' in model.assemblies, f"Missing 'arm' assembly, got: {list(model.assemblies.keys())}"
    
    asm = model.assemblies['arm']
    # All 4 links should be in 'arm'
    assert len(asm.links) == 4, f"Expected 4 links in arm, got {len(asm.links)}: {asm.links}"
    
    print("  assembly_hierarchy: PASS")


def test_tree_connectivity():
    """Ensure all links reachable from root."""
    from ..core.robot_model import build_model
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    # Walk from root
    reachable = set()
    frontier = [model.root_link]
    while frontier:
        current = frontier.pop()
        if current in reachable:
            continue
        reachable.add(current)
        for j in model.joints.values():
            if j.parent_link == current:
                frontier.append(j.child_link)
    
    all_links = set(model.links.keys())
    unreachable = all_links - reachable
    assert len(unreachable) == 0, f"Unreachable links: {unreachable}"
    
    print("  tree_connectivity: PASS")


def test_multi_parent_detection():
    """Detect when a link has multiple parent joints (modeling error)."""
    from ..core.robot_model import build_model
    from ..core.data_types import FusionSnapshot, FusionOccurrence, FusionJoint, Transform3D, InertiaTensor
    
    log = _make_logger()
    
    # Create scenario: link_c has two parents (link_a and link_b both connect to it)
    snap = FusionSnapshot(design_name="bad v1", design_name_clean="bad")
    
    def make_occ(path, name, segs, pos=(0,0,0)):
        return FusionOccurrence(
            full_path=path, clean_name=name, path_segments=segs,
            depth=len(segs)-1, is_subassembly=False,
            global_position=pos, mass_kg=1.0,
            com_component_local=(0,0,0), com_global=pos,
            inertia_at_com=InertiaTensor(), inertia_at_origin=InertiaTensor(),
            local_transform=Transform3D(),
        )
    
    snap.occurrences = {
        "asm v1:1": FusionOccurrence(
            full_path="asm v1:1", clean_name="asm", path_segments=["asm"],
            depth=0, is_subassembly=True, global_position=(0,0,0),
            local_transform=Transform3D(),
            inertia_at_com=InertiaTensor(), inertia_at_origin=InertiaTensor(),
        ),
        "asm v1:1+link_a v1:1": make_occ("asm v1:1+link_a v1:1", "link_a", ["asm", "link_a"]),
        "asm v1:1+link_b v1:1": make_occ("asm v1:1+link_b v1:1", "link_b", ["asm", "link_b"], pos=(0.1, 0, 0)),
        "asm v1:1+link_c v1:1": make_occ("asm v1:1+link_c v1:1", "link_c", ["asm", "link_c"], pos=(0, 0, 0.2)),
    }
    
    snap.joints = {
        "j1": FusionJoint(
            name="j1", defining_component="asm", motion_type="rigid",
            occurrence_one_path="link_b v1:1", occurrence_one_clean="link_b",
            occurrence_two_path="link_a v1:1", occurrence_two_clean="link_a",
            origin_global_m=(0.1, 0, 0), origin_source="geometry.origin",
            axis_vector=(0, 0, 1),
        ),
        "j2": FusionJoint(
            name="j2", defining_component="asm", motion_type="rigid",
            occurrence_one_path="link_c v1:1", occurrence_one_clean="link_c",
            occurrence_two_path="link_a v1:1", occurrence_two_clean="link_a",
            origin_global_m=(0, 0, 0.2), origin_source="geometry.origin",
            axis_vector=(0, 0, 1),
        ),
        "j3": FusionJoint(
            name="j3", defining_component="asm", motion_type="rigid",
            occurrence_one_path="link_c v1:1", occurrence_one_clean="link_c",
            occurrence_two_path="link_b v1:1", occurrence_two_clean="link_b",
            origin_global_m=(0, 0, 0.2), origin_source="geometry.origin",
            axis_vector=(0, 0, 1),
        ),
    }
    
    model = build_model(snap, log)

    # New behavior: multi-parent links are auto-classified — the
    # alphabetically-first joint stays as the URDF tree parent, the
    # rest get routed to ``model.closing_joints`` (the sidecar).  A
    # warning fires per multi-parent child suggesting the user tag
    # the closing joint explicitly.
    multi_parent_warnings = [
        w for w in model.warnings
        if 'multi-parent' in w.lower() or 'multiple parent' in w.lower()
    ]
    assert len(multi_parent_warnings) > 0, \
        f"Expected multi-parent warning, got warnings: {model.warnings}"

    # link_c had two parents (j2 and j3).  After auto-classification,
    # exactly ONE remains in model.joints; the other is in closing_joints.
    parents_of_c = [j for j in model.joints.values() if j.child_link == "link_c"]
    assert len(parents_of_c) == 1, \
        f"Expected exactly one URDF parent of link_c, got {len(parents_of_c)}"
    assert len(model.closing_joints) == 1, \
        f"Expected one closing joint, got {len(model.closing_joints)}"
    closing = next(iter(model.closing_joints.values()))
    assert closing.is_closing
    assert closing.closing_source == "auto_detected"
    assert closing.is_passive  # closing implies passive

    print("  multi_parent_detection: PASS")


def test_link_properties_preserved():
    """Verify physical properties transfer correctly from snapshot to model."""
    from ..core.robot_model import build_model
    
    snap = _make_snapshot()
    log = _make_logger()
    model = build_model(snap, log)
    
    link1 = model.links['link1']
    assert abs(link1.mass_kg - 2.0) < 1e-6
    assert link1.material_name == "Aluminum"
    assert abs(link1.global_position[2] - 0.1) < 1e-6
    assert abs(link1.com_link_local[2] - 0.15) < 1e-6
    assert link1.mesh_visual == "meshes/arm/link1.obj"
    assert link1.assembly == "arm"
    
    print("  link_properties_preserved: PASS")


# ──────────────────────────────────────────────
# Integration test with real snapshot.json
# ──────────────────────────────────────────────

def test_real_snapshot():
    """Build model from the actual basic_platform snapshot.json if available."""
    from ..core.robot_model import build_model
    
    # Try to find snapshot.json
    search_paths = [
        os.path.join(os.path.dirname(__file__), '..', '..', 'debug', 'snapshot.json'),
        os.path.join(os.path.dirname(__file__), '..', 'snapshot.json'),
    ]
    
    snap_path = None
    for p in search_paths:
        if os.path.exists(p):
            snap_path = p
            break
    
    if snap_path is None:
        print("  real_snapshot: SKIPPED (snapshot.json not found)")
        return
    
    with open(snap_path) as f:
        data = json.load(f)
    
    snap = _make_snapshot(data)
    log = _make_logger()
    model = build_model(snap, log)
    
    # ── Structural assertions (snapshot-agnostic) ──
    
    # Root is always renamed to base_link (REP 120)
    assert model.root_link == "base_link", f"Root should be 'base_link', got: {model.root_link}"
    assert "base_link" in model.links, "Root 'base_link' missing from links"
    
    # Must have links and joints
    assert len(model.links) >= 2, f"Expected at least 2 links, got {len(model.links)}"
    assert len(model.joints) >= 1, f"Expected at least 1 joint, got {len(model.joints)}"
    
    # All link names must be unique
    names = list(model.links.keys())
    assert len(names) == len(set(names)), f"Duplicate link names: {names}"
    
    # All joints reference existing links
    for jname, j in model.joints.items():
        assert j.parent_link in model.links, \
            f"Joint '{jname}' parent '{j.parent_link}' not in links"
        assert j.child_link in model.links, \
            f"Joint '{jname}' child '{j.child_link}' not in links"
    
    # Joint types are valid URDF types
    valid_types = {'fixed', 'revolute', 'prismatic', 'continuous'}
    for jname, j in model.joints.items():
        assert j.joint_type in valid_types, \
            f"Joint '{jname}' has invalid type: {j.joint_type}"
    
    # No errors (warnings are expected from intentional modeling issues)
    assert len(model.errors) == 0, f"Unexpected errors: {model.errors}"
    
    # Print summary
    print(f"  real_snapshot: PASS")
    print(f"    Root: {model.root_link}")
    print(f"    Links: {sorted(model.links.keys())}")
    print(f"    Warnings ({len(model.warnings)}):")
    for w in model.warnings:
        print(f"      ⚠ {w}")


# ──────────────────────────────────────────────
# Rigid-group merge tests
#
# These exercise the 2026-04-30 refactor where a Fusion Rigid Group
# becomes ONE merged URDF link instead of N orphan links sharing a
# collision.  Each test builds a small snapshot in-memory, attaches a
# RigidGroupInfo, builds the model, and asserts the merge invariants.
# ──────────────────────────────────────────────

def _merge_snapshot_two_links(rg_name="my_group", collision_member=None):
    """Snapshot: base_link + (heavy + light) joined by a fixed joint,
    with both heavy and light in one rigid group.  Used to verify that
    the merge collapses heavy+light into one anchor link and that the
    in-group fixed joint is dropped.
    """
    snap_dict = json.loads(json.dumps(MINI_SNAPSHOT))  # deep copy

    snap_dict["occurrences"] = {
        "asm v1:1": {
            **MINI_SNAPSHOT["occurrences"]["arm v1:1"],
            "full_path": "asm v1:1",
            "component_name": "asm v1",
            "clean_name": "asm",
            "path_segments": ["asm"],
        },
        "asm v1:1+base_link v1:1": {
            **MINI_SNAPSHOT["occurrences"]["arm v1:1+base_link v1:1"],
            "full_path": "asm v1:1+base_link v1:1",
            "path_segments": ["asm", "base_link"],
        },
        "asm v1:1+heavy v1:1": {
            **MINI_SNAPSHOT["occurrences"]["arm v1:1+link1 v1:1"],
            "full_path": "asm v1:1+heavy v1:1",
            "component_name": "heavy v1",
            "clean_name": "heavy",
            "path_segments": ["asm", "heavy"],
            "mass_kg": 3.0,
            "com_component_local": [0.0, 0.0, 0.0],
            "com_global": [0.0, 0.0, 0.1],
            "global_position": [0.0, 0.0, 0.1],
            "inertia_at_com": {"ixx": 0.01, "iyy": 0.01, "izz": 0.005,
                                "ixy": 0, "ixz": 0, "iyz": 0},
            "bbox_size": [0.1, 0.1, 0.1],
            "bbox_min": [-0.05, -0.05, -0.05],
            "bbox_max": [0.05, 0.05, 0.05],
        },
        "asm v1:1+light v1:1": {
            **MINI_SNAPSHOT["occurrences"]["arm v1:1+link2 v1:1"],
            "full_path": "asm v1:1+light v1:1",
            "component_name": "light v1",
            "clean_name": "light",
            "path_segments": ["asm", "light"],
            "mass_kg": 1.0,
            "com_component_local": [0.0, 0.0, 0.0],
            "com_global": [0.2, 0.0, 0.1],   # 0.2 m offset from heavy
            "global_position": [0.2, 0.0, 0.1],
            "inertia_at_com": {"ixx": 0.001, "iyy": 0.001, "izz": 0.0005,
                                "ixy": 0, "ixz": 0, "iyz": 0},
            "bbox_size": [0.05, 0.05, 0.05],
            "bbox_min": [-0.025, -0.025, -0.025],
            "bbox_max": [0.025, 0.025, 0.025],
        },
    }

    snap_dict["joints"] = {
        "mount": {
            "name": "mount", "joint_source": "as_built",
            "defining_component": "asm", "motion_type": "rigid",
            "occurrence_one_path": "heavy v1:1", "occurrence_one_clean": "heavy",
            "occurrence_two_path": "base_link v1:1", "occurrence_two_clean": "base_link",
            "origin_global_m": [0.0, 0.0, 0.0],
            "origin_source": "geometry.origin",
            "axis_vector": [0.0, 0.0, 1.0],
            "has_rotation_limits": False,
            "has_slide_limits": False,
        },
        "internal": {
            # heavy ↔ light, both in the rigid group → must be dropped.
            "name": "internal", "joint_source": "as_built",
            "defining_component": "asm", "motion_type": "rigid",
            "occurrence_one_path": "light v1:1", "occurrence_one_clean": "light",
            "occurrence_two_path": "heavy v1:1", "occurrence_two_clean": "heavy",
            "origin_global_m": [0.1, 0.0, 0.1],
            "origin_source": "geometry.origin",
            "axis_vector": [0.0, 0.0, 1.0],
            "has_rotation_limits": False,
            "has_slide_limits": False,
        },
    }

    snap = _make_snapshot(snap_dict)
    snap.rigid_groups.append(RigidGroupInfo(
        name=rg_name,
        occurrence_paths=["asm v1:1+heavy v1:1", "asm v1:1+light v1:1"],
        member_clean_names=["heavy", "light"],
        collision_member=collision_member,
    ))
    return snap


def test_rigid_group_merge_two_members():
    """Two-member rigid group: heavier member becomes the anchor; mass
    is summed; combined CoM lies at the mass-weighted midpoint of the
    two members; the internal joint between them is dropped."""
    from ..core.data_types import RigidGroupInfo
    globals().setdefault("RigidGroupInfo", RigidGroupInfo)
    from ..core.robot_model import build_model

    snap = _merge_snapshot_two_links(rg_name="cluster")
    log = _make_logger()
    model = build_model(snap, log)

    assert "cluster" in model.links, \
        f"Expected merged link 'cluster' (rigid-group name), got: {sorted(model.links.keys())}"
    cluster = model.links["cluster"]
    assert cluster.is_merged, "Anchor link should be is_merged=True"
    assert cluster.clean_name == "heavy", \
        f"Anchor occurrence should be 'heavy', got: {cluster.clean_name}"

    # Mass = 3 + 1 = 4 kg.
    assert abs(cluster.mass_kg - 4.0) < 1e-9, \
        f"Aggregated mass: expected 4.0, got {cluster.mass_kg}"

    # Combined CoM = (3·[0,0,0] + 1·[0.2,0,0]) / 4 = [0.05, 0, 0]
    # in the anchor's local frame (anchor is at world origin in this snapshot
    # because identity transforms make global_position equal local_position).
    cx, cy, cz = cluster.com_link_local
    assert abs(cx - 0.05) < 1e-6, f"CoM_x: expected 0.05, got {cx}"
    assert abs(cy) < 1e-6, f"CoM_y: expected 0, got {cy}"
    assert abs(cz) < 1e-6, f"CoM_z: expected 0, got {cz}"

    # Only ONE joint should remain — the in-group joint is dropped.
    assert "internal" not in model.joints, \
        "Internal joint between rigid-group members must be dropped"
    assert "mount" in model.joints
    assert model.joints["mount"].child_link == "cluster", \
        f"Mount joint's child should redirect to 'cluster', got: {model.joints['mount'].child_link}"

    print("  rigid_group_merge_two_members: PASS")


def test_rigid_group_merge_anchor_is_heaviest():
    """When members differ in mass, the heaviest is chosen as anchor."""
    from ..core.data_types import RigidGroupInfo
    globals().setdefault("RigidGroupInfo", RigidGroupInfo)
    from ..core.robot_model import build_model

    snap = _merge_snapshot_two_links(rg_name="Rigid Group 7")  # generic name
    log = _make_logger()
    model = build_model(snap, log)

    # Generic Fusion name → fall back to anchor's clean_name.
    assert "heavy" in model.links, \
        f"Generic group name should fall back to anchor clean_name; got: {sorted(model.links.keys())}"
    assert model.links["heavy"].is_merged

    print("  rigid_group_merge_anchor_is_heaviest: PASS")


def test_rigid_group_frame_member_becomes_anchor():
    """A ``!frame_*`` member declares the merged link frame.

    The frame member should win anchor selection even with zero mass, but
    it must not contribute geometry/mass to the merged link.
    """
    from ..core.data_types import RigidGroupInfo
    globals().setdefault("RigidGroupInfo", RigidGroupInfo)
    from ..core.robot_model import build_model

    snap = _merge_snapshot_two_links(rg_name="cluster")
    frame_path = "asm v1:1+!frame_cluster v1:1"
    snap.occurrences[frame_path] = type(snap.occurrences["asm v1:1+heavy v1:1"])(
        full_path=frame_path,
        component_name="!frame_cluster v1",
        clean_name="cluster",
        path_segments=["asm", "cluster"],
        depth=1,
        is_subassembly=False,
        is_frame_only=True,
        global_position=(0.0, 0.0, 0.1),
        com_global=(0.0, 0.0, 0.1),
        mass_kg=0.0,
        body_count=1,
        bbox_min=(-10.0, -10.0, -10.0),
        bbox_max=(10.0, 10.0, 10.0),
        bbox_size=(20.0, 20.0, 20.0),
    )
    snap.rigid_groups[0].occurrence_paths.append(frame_path)
    snap.rigid_groups[0].member_clean_names.append("cluster")

    log = _make_logger()
    model = build_model(snap, log)

    cluster = model.links.get("cluster")
    assert cluster is not None
    assert cluster.is_merged
    assert not cluster.is_frame_only
    assert cluster.clean_name == "cluster", \
        f"Frame member should be anchor, got: {cluster.clean_name}"
    assert cluster.occurrence_path == frame_path, \
        f"Merged link should use frame occurrence path, got: {cluster.occurrence_path}"
    assert abs(cluster.mass_kg - 4.0) < 1e-9, \
        f"Frame-only member must not add mass, got {cluster.mass_kg}"
    assert cluster.bbox_size[0] < 0.4, \
        f"Frame-only marker body must not expand bbox, got {cluster.bbox_size}"
    assert model.joints["mount"].child_link == "cluster"

    print("  rigid_group_frame_member_becomes_anchor: PASS")


def test_rigid_group_frame_anchor_overrides_movable_joint_origin():
    """For a movable rigid-group child, a ``!frame_*`` anchor is the joint frame.

    This covers the Panther wheel pattern: a physical wheel component and a
    small ``!frame_fl_wheel_link`` marker are merged into one wheel link.  If
    Fusion reports a stale/mirrored ``geometry.origin`` for the wheel joint,
    the explicit frame anchor must win so the wheel rotates around itself
    instead of orbiting another corner of the chassis.
    """
    from ..core.data_types import (
        FusionSnapshot,
        FusionOccurrence,
        FusionJoint,
        InertiaTensor,
        RigidGroupInfo,
        Transform3D,
    )
    from ..core.robot_model import build_model

    snap = FusionSnapshot(design_name="frame_wheel", design_name_clean="frame_wheel")

    def occ(path, clean, segs, *, pos=(0.0, 0.0, 0.0), mass=0.0, bodies=0,
            is_sub=False, is_frame=False, bbox=0.1):
        half = bbox / 2.0
        return FusionOccurrence(
            full_path=path,
            component_name=clean,
            clean_name=clean,
            path_segments=list(segs),
            depth=len(segs) - 1,
            is_subassembly=is_sub,
            child_count=1 if is_sub else 0,
            is_frame_only=is_frame,
            global_position=pos,
            local_transform=Transform3D(translation=pos),
            transform2=Transform3D(translation=pos),
            mass_kg=mass,
            body_count=bodies,
            com_component_local=(0.0, 0.0, 0.0),
            com_global=pos,
            inertia_at_origin=InertiaTensor(ixx=0.01, iyy=0.01, izz=0.01),
            inertia_at_com=InertiaTensor(ixx=0.01, iyy=0.01, izz=0.01),
            bbox_min=(-half, -half, -half),
            bbox_max=(half, half, half),
            bbox_size=(bbox, bbox, bbox),
            volume_m3=bbox ** 3,
            area_m2=6 * bbox * bbox,
        )

    asm = "robot v1:1"
    base = f"{asm}+base_link v1:1"
    mount = f"{asm}+fl_wheel_base_link v1:1"
    wheel_body = f"{asm}+WH01_fl v1:1"
    wheel_frame = f"{asm}+!frame_fl_wheel_link v1:1"

    snap.occurrences = {
        asm: occ(asm, "robot", ["robot"], is_sub=True),
        base: occ(base, "base_link", ["robot", "base_link"], mass=5.0, bodies=1),
        mount: occ(
            mount, "fl_wheel_base_link", ["robot", "fl_wheel_base_link"],
            pos=(0.220, 0.258, 0.0), is_frame=True,
        ),
        wheel_body: occ(
            wheel_body, "WH01_fl", ["robot", "WH01_fl"],
            pos=(0.220, 0.349, 0.0), mass=1.0, bodies=1,
        ),
        wheel_frame: occ(
            wheel_frame, "fl_wheel_link", ["robot", "fl_wheel_link"],
            pos=(0.220, 0.349, 0.0), is_frame=True,
        ),
    }
    snap.rigid_groups.append(RigidGroupInfo(
        name="fl_wheel_link",
        occurrence_paths=[wheel_body, wheel_frame],
        member_clean_names=["WH01_fl", "fl_wheel_link"],
    ))
    snap.joints = {
        "fl_wheel_base_joint": FusionJoint(
            name="fl_wheel_base_joint",
            defining_component="robot",
            motion_type="rigid",
            occurrence_one_path="fl_wheel_base_link v1:1",
            occurrence_one_clean="fl_wheel_base_link",
            occurrence_two_path="base_link v1:1",
            occurrence_two_clean="base_link",
            origin_global_m=(0.220, 0.258, 0.0),
            origin_source="geometry.origin",
            axis_vector=(0.0, 0.0, 1.0),
        ),
        "fl_wheel_joint": FusionJoint(
            name="fl_wheel_joint",
            defining_component="robot",
            motion_type="revolute",
            occurrence_one_path="WH01_fl v1:1",
            occurrence_one_clean="WH01_fl",
            occurrence_two_path="fl_wheel_base_link v1:1",
            occurrence_two_clean="fl_wheel_base_link",
            # Stale/mirrored Fusion origin that would put the pivot at the
            # opposite chassis corner if the frame anchor did not override it.
            origin_global_m=(-0.220, -0.349, 0.0),
            origin_source="geometry.origin",
            axis_vector=(0.0, -1.0, 0.0),
            has_rotation_limits=False,
        ),
    }

    model = build_model(snap, _make_logger())
    joint = model.joints["fl_wheel_joint"]

    assert joint.origin_method == "frame_anchor_minus_parent"
    assert abs(joint.origin_xyz[0] - 0.0) < 1e-9
    assert abs(joint.origin_xyz[1] - 0.091) < 1e-9
    assert abs(joint.origin_xyz[2] - 0.0) < 1e-9
    assert joint.origin_global == (0.220, 0.349, 0.0)
    wheel_link = model.links["fl_wheel_link"]
    assert wheel_link.occurrence_path == wheel_frame
    assert not wheel_link.needs_mesh_bake, (
        f"frame-anchored wheel mesh should already be in link frame, got "
        f"bake={wheel_link.mesh_bake_offset}"
    )

    print("  rigid_group_frame_anchor_overrides_movable_joint_origin: PASS")


def test_base_frame_anchor_rotation_defines_root_link_frame():
    """A ``!frame_base_link`` member should define base_link orientation."""
    from ..core.data_types import (
        FusionSnapshot,
        FusionOccurrence,
        FusionJoint,
        InertiaTensor,
        RigidGroupInfo,
        Transform3D,
    )
    from ..core.robot_model import build_model

    rot_z180 = (-1.0, 0.0, 0.0,
                 0.0, -1.0, 0.0,
                 0.0, 0.0, 1.0)
    snap = FusionSnapshot(design_name="base_frame", design_name_clean="base_frame")

    def occ(path, clean, segs, *, pos=(0.0, 0.0, 0.0), rotation=None,
            mass=0.0, bodies=0, is_sub=False, is_frame=False):
        rotation = rotation or (1.0, 0.0, 0.0,
                                0.0, 1.0, 0.0,
                                0.0, 0.0, 1.0)
        return FusionOccurrence(
            full_path=path,
            component_name=clean,
            clean_name=clean,
            path_segments=list(segs),
            depth=len(segs) - 1,
            is_subassembly=is_sub,
            child_count=1 if is_sub else 0,
            is_frame_only=is_frame,
            global_position=pos,
            local_transform=Transform3D(translation=pos, rotation=rotation),
            transform2=Transform3D(translation=pos, rotation=rotation),
            mass_kg=mass,
            body_count=bodies,
            com_component_local=(0.0, 0.0, 0.0),
            com_global=pos,
            inertia_at_origin=InertiaTensor(ixx=0.01, iyy=0.01, izz=0.01),
            inertia_at_com=InertiaTensor(ixx=0.01, iyy=0.01, izz=0.01),
            bbox_min=(-0.05, -0.05, -0.05),
            bbox_max=(0.05, 0.05, 0.05),
            bbox_size=(0.1, 0.1, 0.1),
            volume_m3=0.001,
            area_m2=0.06,
        )

    asm = "robot v1:1"
    base_body = f"{asm}+chassis v1:1"
    base_frame = f"{asm}+!frame_base_link v1:1"
    mount = f"{asm}+fl_wheel_base_link v1:1"

    snap.occurrences = {
        asm: occ(asm, "robot", ["robot"], is_sub=True),
        base_body: occ(base_body, "chassis", ["robot", "chassis"], mass=5.0, bodies=1),
        base_frame: occ(
            base_frame, "base_link", ["robot", "base_link"],
            rotation=rot_z180, is_frame=True,
        ),
        mount: occ(
            mount, "fl_wheel_base_link", ["robot", "fl_wheel_base_link"],
            pos=(-0.220, -0.258, 0.0), is_frame=True,
        ),
    }
    snap.rigid_groups.append(RigidGroupInfo(
        name="base_link",
        occurrence_paths=[base_body, base_frame],
        member_clean_names=["chassis", "base_link"],
    ))
    snap.joints["fl_wheel_base_joint"] = FusionJoint(
        name="fl_wheel_base_joint",
        defining_component="robot",
        motion_type="rigid",
        occurrence_one_path="fl_wheel_base_link v1:1",
        occurrence_one_clean="fl_wheel_base_link",
        occurrence_two_path="chassis v1:1",
        occurrence_two_clean="chassis",
        origin_global_m=(-0.220, -0.258, 0.0),
        origin_source="geometry.origin",
        axis_vector=(0.0, 0.0, 1.0),
    )

    model = build_model(snap, _make_logger())
    base = model.links["base_link"]
    joint = model.joints["fl_wheel_base_joint"]

    assert base.occurrence_path == base_frame
    assert abs(joint.origin_xyz[0] - 0.220) < 1e-9
    assert abs(joint.origin_xyz[1] - 0.258) < 1e-9
    assert abs(joint.origin_xyz[2] - 0.0) < 1e-9

    print("  base_frame_anchor_rotation_defines_root_link_frame: PASS")


def test_rigid_group_merge_collision_member_excluded_from_anchor_choice():
    """A ``collision_*`` member must never be picked as the anchor."""
    from ..core.data_types import RigidGroupInfo
    globals().setdefault("RigidGroupInfo", RigidGroupInfo)
    from ..core.robot_model import build_model

    snap = _merge_snapshot_two_links(rg_name="cluster")
    # Add a heavy collision_* member that would beat 'heavy' on mass.
    coll_path = "asm v1:1+collision_zed v1:1"
    snap.occurrences[coll_path] = type(snap.occurrences["asm v1:1+heavy v1:1"])(
        full_path=coll_path,
        component_name="collision_zed v1",
        clean_name="collision_zed",
        path_segments=["asm", "collision_zed"],
        depth=1,
        is_subassembly=False,
        global_position=(0.05, 0.0, 0.1),
        com_global=(0.05, 0.0, 0.1),
        mass_kg=99.0,  # absurdly heavy — tries to win anchor selection
    )
    snap.rigid_groups[0].occurrence_paths.append(coll_path)
    snap.rigid_groups[0].member_clean_names.append("collision_zed")
    snap.rigid_groups[0].collision_member = "collision_zed"
    snap.rigid_groups[0].collision_path = coll_path

    log = _make_logger()
    model = build_model(snap, log)

    cluster = model.links.get("cluster")
    assert cluster is not None
    # Anchor must still be 'heavy' — 'collision_zed' is never an anchor.
    assert cluster.clean_name == "heavy", \
        f"Anchor must skip collision members; got: {cluster.clean_name}"
    # Mass excludes the collision member.
    assert abs(cluster.mass_kg - 4.0) < 1e-9, \
        f"Aggregated mass should exclude collision_*; got {cluster.mass_kg}"
    # Collision metadata should be wired up.
    assert cluster.has_explicit_collision
    assert cluster.rigid_group_collision_path == coll_path

    print("  rigid_group_merge_collision_member_excluded_from_anchor_choice: PASS")


def test_rigid_group_body_owning_subassembly_with_design_root_joints():
    """Panther-style topology: an empty design-root fallback endpoint
    should resolve to a rigid group named base_link, and a subassembly
    with its own direct bodies must contribute to that merged link."""
    from ..core.data_types import (
        DESIGN_ROOT_OCCURRENCE_PATH,
        FusionSnapshot,
        FusionOccurrence,
        FusionJoint,
        InertiaTensor,
        RigidGroupInfo,
        Transform3D,
    )
    from ..core.robot_model import build_model

    snap = FusionSnapshot(design_name="Panther v7", design_name_clean="Panther")

    def make_occ(
        path, clean, segs, *, is_sub=False, mass=0.0, bodies=0,
        pos=(0.0, 0.0, 0.0), bbox=0.1,
    ):
        half = bbox / 2.0
        return FusionOccurrence(
            full_path=path,
            component_name=clean,
            clean_name=clean,
            path_segments=list(segs),
            depth=len(segs) - 1,
            is_subassembly=is_sub,
            child_count=1 if is_sub else 0,
            global_position=pos,
            local_transform=Transform3D(translation=pos),
            transform2=Transform3D(translation=pos),
            mass_kg=mass,
            body_count=bodies,
            com_component_local=(0.0, 0.0, 0.0),
            com_global=pos,
            inertia_at_origin=InertiaTensor(ixx=0.01, iyy=0.01, izz=0.01),
            inertia_at_com=InertiaTensor(ixx=0.01, iyy=0.01, izz=0.01),
            bbox_min=(-half, -half, -half),
            bbox_max=(half, half, half),
            bbox_size=(bbox, bbox, bbox),
            volume_m3=bbox ** 3,
            area_m2=6 * bbox * bbox,
        )

    fuselage = "Model:1+Fusalage:1"
    cover = "Model:1+Fusalage:1+Cover:1"
    rails = "Model:1+Mountings Rails:1"
    battery = "Model:1+Internal components:1+BAT02:1"
    wheel = "Model:1+WH01_lf:1"

    snap.occurrences = {
        "Model:1": make_occ("Model:1", "Model", ["Model"], is_sub=True),
        fuselage: make_occ(
            fuselage, "Fusalage", ["Model", "Fusalage"],
            is_sub=True, mass=7.0, bodies=3, bbox=0.6,
        ),
        cover: make_occ(
            cover, "Cover", ["Model", "Fusalage", "Cover"],
            mass=2.0, bodies=1, pos=(0.0, 0.0, 0.05),
        ),
        rails: make_occ(
            rails, "Mountings_Rails", ["Model", "Mountings_Rails"],
            mass=3.0, bodies=1, pos=(0.0, 0.25, 0.0),
        ),
        "Model:1+Internal components:1": make_occ(
            "Model:1+Internal components:1", "Internal_components",
            ["Model", "Internal_components"], is_sub=True,
        ),
        battery: make_occ(
            battery, "BAT02", ["Model", "Internal_components", "BAT02"],
            mass=12.0, bodies=1, pos=(0.1, 0.0, 0.0),
        ),
        wheel: make_occ(
            wheel, "WH01_lf", ["Model", "WH01_lf"],
            mass=1.0, bodies=1, pos=(0.5, 0.3, 0.0),
        ),
        DESIGN_ROOT_OCCURRENCE_PATH: make_occ(
            DESIGN_ROOT_OCCURRENCE_PATH, "Panther", ["Panther"],
        ),
    }
    snap.rigid_groups.append(RigidGroupInfo(
        name="base_link",
        occurrence_paths=[fuselage, cover, rails, battery],
        member_clean_names=["Fusalage", "Cover", "Mountings_Rails", "BAT02"],
    ))
    snap.joints["Revolute_2"] = FusionJoint(
        name="Revolute_2",
        defining_component="Panther",
        motion_type="revolute",
        occurrence_one_path="WH01_lf:1",
        occurrence_one_clean="WH01_lf",
        occurrence_two_path=DESIGN_ROOT_OCCURRENCE_PATH,
        occurrence_two_clean="Panther",
        origin_global_m=(0.5, 0.3, 0.0),
        origin_source="geometry.origin",
        axis_vector=(0.0, 1.0, 0.0),
    )

    model = build_model(snap, _make_logger())

    assert not model.errors, f"unexpected errors: {model.errors}"
    assert model.root_link == "base_link"
    assert "Panther" not in model.links, "empty design-root link must be redirected"
    base = model.links["base_link"]
    assert base.is_merged
    assert not base.is_empty
    assert fuselage in base.merged_member_paths
    assert abs(base.mass_kg - 24.0) < 1e-9
    assert model.joints["Revolute_2"].parent_link == "base_link"
    assert model.joints["Revolute_2"].child_link == "WH01_lf"

    print("  rigid_group_body_owning_subassembly_with_design_root_joints: PASS")


def test_auto_rigid_island_collapses_nested_subassemblies_without_joints():
    """A no-joint subassembly should become one implicit merged link,
    including physical descendants at arbitrary nesting depth."""
    from ..core.data_types import (
        FusionSnapshot,
        FusionOccurrence,
        FusionJoint,
        InertiaTensor,
        Transform3D,
    )
    from ..core.robot_model import build_model

    snap = FusionSnapshot(design_name="auto_bot", design_name_clean="auto_bot")

    def make_occ(
        path, clean, segs, *, is_sub=False, mass=0.0, bodies=0,
        pos=(0.0, 0.0, 0.0), bbox=0.1,
    ):
        half = bbox / 2.0
        return FusionOccurrence(
            full_path=path,
            component_name=clean,
            clean_name=clean,
            path_segments=list(segs),
            depth=len(segs) - 1,
            is_subassembly=is_sub,
            child_count=1 if is_sub else 0,
            global_position=pos,
            local_transform=Transform3D(translation=pos),
            transform2=Transform3D(translation=pos),
            mass_kg=mass,
            body_count=bodies,
            com_component_local=(0.0, 0.0, 0.0),
            com_global=pos,
            inertia_at_origin=InertiaTensor(ixx=0.01, iyy=0.01, izz=0.01),
            inertia_at_com=InertiaTensor(ixx=0.01, iyy=0.01, izz=0.01),
            bbox_min=(-half, -half, -half),
            bbox_max=(half, half, half),
            bbox_size=(bbox, bbox, bbox),
            volume_m3=bbox ** 3,
            area_m2=6 * bbox * bbox,
        )

    base = "robot:1+base_pack:1"
    shell = "robot:1+base_pack:1+shell:1"
    electronics = "robot:1+base_pack:1+electronics:1"
    battery = "robot:1+base_pack:1+electronics:1+battery:1"
    board = "robot:1+base_pack:1+electronics:1+board:1"
    wheel = "robot:1+wheel:1"

    snap.occurrences = {
        "robot:1": make_occ("robot:1", "robot", ["robot"], is_sub=True),
        base: make_occ(
            base, "base_pack", ["robot", "base_pack"],
            is_sub=True, mass=1.0, bodies=1, bbox=0.4,
        ),
        shell: make_occ(
            shell, "shell", ["robot", "base_pack", "shell"],
            mass=2.0, bodies=1, pos=(0.0, 0.0, 0.02),
        ),
        electronics: make_occ(
            electronics, "electronics", ["robot", "base_pack", "electronics"],
            is_sub=True,
        ),
        battery: make_occ(
            battery, "battery", ["robot", "base_pack", "electronics", "battery"],
            mass=8.0, bodies=1, pos=(0.05, 0.0, 0.0),
        ),
        board: make_occ(
            board, "board", ["robot", "base_pack", "electronics", "board"],
            mass=0.5, bodies=1, pos=(-0.05, 0.0, 0.0),
        ),
        wheel: make_occ(
            wheel, "wheel", ["robot", "wheel"],
            mass=1.0, bodies=1, pos=(0.4, 0.0, 0.0),
        ),
    }
    snap.joints["wheel_joint"] = FusionJoint(
        name="wheel_joint",
        defining_component="robot",
        motion_type="revolute",
        occurrence_one_path="wheel:1",
        occurrence_one_clean="wheel",
        occurrence_two_path="base_pack:1",
        occurrence_two_clean="base_pack",
        origin_global_m=(0.4, 0.0, 0.0),
        origin_source="geometry.origin",
        axis_vector=(0.0, 1.0, 0.0),
    )

    model = build_model(snap, _make_logger())

    assert not model.errors, f"unexpected errors: {model.errors}"
    assert model.root_link == "base_link"
    assert set(model.links) == {"base_link", "wheel"}
    merged = model.links["base_link"]
    assert merged.is_merged
    assert merged.clean_name == "base_pack", "body-owning subassembly should be anchor"
    assert merged.occurrence_path == base
    assert set(merged.merged_member_paths) == {base, shell, battery, board}
    assert abs(merged.mass_kg - 11.5) < 1e-9
    assert model.joints["wheel_joint"].parent_link == "base_link"
    assert model.joints["wheel_joint"].child_link == "wheel"

    print("  auto_rigid_island_collapses_nested_subassemblies_without_joints: PASS")


def test_auto_rigid_island_preserves_articulated_subassembly():
    """A subassembly with an internal joint must remain articulated."""
    from ..core.data_types import (
        FusionSnapshot,
        FusionOccurrence,
        FusionJoint,
        InertiaTensor,
        Transform3D,
    )
    from ..core.robot_model import build_model

    snap = FusionSnapshot(design_name="arm_bot", design_name_clean="arm_bot")

    def make_occ(path, clean, segs, *, is_sub=False, pos=(0.0, 0.0, 0.0)):
        return FusionOccurrence(
            full_path=path,
            component_name=clean,
            clean_name=clean,
            path_segments=list(segs),
            depth=len(segs) - 1,
            is_subassembly=is_sub,
            child_count=1 if is_sub else 0,
            global_position=pos,
            local_transform=Transform3D(translation=pos),
            transform2=Transform3D(translation=pos),
            mass_kg=1.0 if not is_sub else 0.0,
            body_count=1 if not is_sub else 0,
            com_component_local=(0.0, 0.0, 0.0),
            com_global=pos,
            inertia_at_origin=InertiaTensor(ixx=0.01, iyy=0.01, izz=0.01),
            inertia_at_com=InertiaTensor(ixx=0.01, iyy=0.01, izz=0.01),
            bbox_min=(-0.05, -0.05, -0.05),
            bbox_max=(0.05, 0.05, 0.05),
            bbox_size=(0.1, 0.1, 0.1),
        )

    link_a = "robot:1+arm:1+link_a:1"
    link_b = "robot:1+arm:1+link_b:1"
    snap.occurrences = {
        "robot:1": make_occ("robot:1", "robot", ["robot"], is_sub=True),
        "robot:1+arm:1": make_occ(
            "robot:1+arm:1", "arm", ["robot", "arm"], is_sub=True,
        ),
        link_a: make_occ(link_a, "link_a", ["robot", "arm", "link_a"]),
        link_b: make_occ(
            link_b, "link_b", ["robot", "arm", "link_b"],
            pos=(0.2, 0.0, 0.0),
        ),
    }
    snap.joints["elbow"] = FusionJoint(
        name="elbow",
        defining_component="arm",
        motion_type="revolute",
        occurrence_one_path="link_b:1",
        occurrence_one_clean="link_b",
        occurrence_two_path="link_a:1",
        occurrence_two_clean="link_a",
        origin_global_m=(0.2, 0.0, 0.0),
        origin_source="geometry.origin",
        axis_vector=(0.0, 0.0, 1.0),
    )

    model = build_model(snap, _make_logger())

    assert "arm" not in model.links
    assert "link_b" in model.links
    assert not any(link.is_merged for link in model.links.values())
    assert model.joints["elbow"].child_link == "link_b"

    print("  auto_rigid_island_preserves_articulated_subassembly: PASS")


def test_orphan_link_is_error():
    """A non-root link with no parent joint and not in a rigid group
    must produce a hard error (was a warning pre-2026-04-30)."""
    from ..core.data_types import (
        FusionSnapshot, FusionOccurrence, FusionJoint, InertiaTensor, Transform3D,
    )
    from ..core.robot_model import build_model

    snap = FusionSnapshot(design_name="orphan_test", design_name_clean="orphan_test")
    snap.occurrences = {
        "asm v1:1": FusionOccurrence(
            full_path="asm v1:1", clean_name="asm",
            path_segments=["asm"], depth=0, is_subassembly=True,
            local_transform=Transform3D(),
        ),
        "asm v1:1+root_link v1:1": FusionOccurrence(
            full_path="asm v1:1+root_link v1:1", clean_name="root_link",
            path_segments=["asm", "root_link"], depth=1, is_subassembly=False,
            mass_kg=1.0, body_count=1,
            local_transform=Transform3D(),
        ),
        "asm v1:1+child v1:1": FusionOccurrence(
            full_path="asm v1:1+child v1:1", clean_name="child",
            path_segments=["asm", "child"], depth=1, is_subassembly=False,
            mass_kg=1.0, body_count=1,
            local_transform=Transform3D(),
        ),
        "asm v1:1+orphan v1:1": FusionOccurrence(
            full_path="asm v1:1+orphan v1:1", clean_name="orphan",
            path_segments=["asm", "orphan"], depth=1, is_subassembly=False,
            mass_kg=1.0, body_count=1,
            local_transform=Transform3D(),
        ),
    }
    # One joint root → child.  'orphan' has nothing.
    snap.joints["j1"] = FusionJoint(
        name="j1", defining_component="asm", motion_type="rigid",
        occurrence_one_path="child v1:1", occurrence_one_clean="child",
        occurrence_two_path="root_link v1:1", occurrence_two_clean="root_link",
        origin_global_m=(0, 0, 0), origin_source="geometry.origin",
        axis_vector=(0, 0, 1),
    )

    model = build_model(snap, _make_logger())

    orphan_errors = [e for e in model.errors if "orphan" in e.lower()]
    assert orphan_errors, \
        f"Expected an orphan-link error in model.errors; got: {model.errors}"
    assert "orphan" in orphan_errors[0].lower()

    print("  orphan_link_is_error: PASS")


def test_unreferenced_empty_occurrence_is_dropped():
    """Imported zero-body helper components that no joint uses should not
    become disconnected URDF links."""
    from ..core.data_types import FusionSnapshot, FusionOccurrence, FusionJoint, Transform3D
    from ..core.robot_model import build_model

    snap = FusionSnapshot(design_name="imported", design_name_clean="imported")
    snap.occurrences = {
        "root:1": FusionOccurrence(
            full_path="root:1", clean_name="root",
            path_segments=["root"], depth=0, is_subassembly=False,
            mass_kg=1.0, body_count=1, local_transform=Transform3D(),
        ),
        "child:1": FusionOccurrence(
            full_path="child:1", clean_name="child",
            path_segments=["child"], depth=0, is_subassembly=False,
            mass_kg=1.0, body_count=1, local_transform=Transform3D(),
        ),
        "empty_ref:1": FusionOccurrence(
            full_path="empty_ref:1", clean_name="empty_ref",
            path_segments=["empty_ref"], depth=0, is_subassembly=False,
            mass_kg=0.0, body_count=0, local_transform=Transform3D(),
        ),
    }
    snap.joints["j"] = FusionJoint(
        name="j", defining_component="imported", motion_type="rigid",
        occurrence_one_path="child:1", occurrence_one_clean="child",
        occurrence_two_path="root:1", occurrence_two_clean="root",
    )

    model = build_model(snap, _make_logger())
    assert "empty_ref" not in model.links
    assert not model.errors, f"unexpected errors: {model.errors}"

    print("  unreferenced_empty_occurrence_is_dropped: PASS")


def test_frame_only_child_joint_forced_fixed():
    """A frame_* child stays in the tree but cannot be an articulated body."""
    from ..core.data_types import FusionSnapshot, FusionOccurrence, FusionJoint
    from ..core.robot_model import build_model

    snap = FusionSnapshot(design_name="framebot", design_name_clean="framebot")
    snap.occurrences = {
        "base_link v1:1": FusionOccurrence(
            full_path="base_link v1:1",
            component_name="base_link v1",
            clean_name="base_link",
            path_segments=["base_link"],
            depth=0,
            global_position=(0.0, 0.0, 0.0),
            mass_kg=1.0,
            body_count=1,
        ),
        "frame_imu v1:1": FusionOccurrence(
            full_path="frame_imu v1:1",
            component_name="frame_imu v1",
            clean_name="imu",
            path_segments=["frame_imu"],
            depth=0,
            global_position=(0.1, 0.0, 0.2),
            mass_kg=99.0,
            body_count=4,
            is_frame_only=True,
        ),
    }
    snap.joints["imu_pivot"] = FusionJoint(
        name="imu_pivot",
        joint_source="as_built",
        defining_component="framebot",
        motion_type="revolute",
        occurrence_one_path="frame_imu v1:1",
        occurrence_one_clean="imu",
        occurrence_two_path="base_link v1:1",
        occurrence_two_clean="base_link",
        origin_global_m=(0.1, 0.0, 0.2),
        axis_vector=(0.0, 0.0, 1.0),
        has_rotation_limits=False,
    )

    model = build_model(snap, _make_logger())

    assert "imu" in model.links
    frame = model.links["imu"]
    assert frame.is_frame_only
    assert frame.is_empty
    assert frame.mass_kg == 0.0
    assert not frame.has_visual_mesh
    assert frame.mesh_visual == ""

    joint = model.joints["imu_pivot"]
    assert joint.joint_type == "fixed"
    assert any("frame-only child" in w for w in model.warnings)

    print("  frame_only_child_joint_forced_fixed: PASS")


def test_user_tagged_closing_joint():
    """A joint flagged ``is_closing=True`` (from a Fusion ``closing_*``
    name) is routed to the closing-joints sidecar, never the URDF
    tree.  The resulting JointNode carries
    ``closing_source='user_tag'`` and is implicitly passive."""
    from ..core.data_types import (
        FusionSnapshot, FusionOccurrence, FusionJoint, InertiaTensor, Transform3D,
    )
    from ..core.robot_model import build_model

    snap = FusionSnapshot(design_name="loop", design_name_clean="loop")
    snap.occurrences = {
        "asm v1:1": FusionOccurrence(
            full_path="asm v1:1", clean_name="asm",
            path_segments=["asm"], depth=0, is_subassembly=True,
            local_transform=Transform3D(),
        ),
        "asm v1:1+base v1:1": FusionOccurrence(
            full_path="asm v1:1+base v1:1", clean_name="base",
            path_segments=["asm", "base"], depth=1, is_subassembly=False,
            mass_kg=1.0, body_count=1, local_transform=Transform3D(),
        ),
        "asm v1:1+mid v1:1": FusionOccurrence(
            full_path="asm v1:1+mid v1:1", clean_name="mid",
            path_segments=["asm", "mid"], depth=1, is_subassembly=False,
            mass_kg=1.0, body_count=1, local_transform=Transform3D(),
        ),
        "asm v1:1+child v1:1": FusionOccurrence(
            full_path="asm v1:1+child v1:1", clean_name="child",
            path_segments=["asm", "child"], depth=1, is_subassembly=False,
            mass_kg=1.0, body_count=1, local_transform=Transform3D(),
        ),
    }
    # base → mid → child (open chain) plus a user-tagged closing edge
    # base → child.  Without the explicit tag, the alphabetical-pick
    # would route the WRONG joint: this test guarantees the user's
    # choice wins.
    snap.joints = {
        "alpha_joint": FusionJoint(
            name="alpha_joint", defining_component="asm", motion_type="rigid",
            occurrence_one_path="mid v1:1", occurrence_one_clean="mid",
            occurrence_two_path="base v1:1", occurrence_two_clean="base",
            origin_global_m=(0, 0, 0), origin_source="geometry.origin",
            axis_vector=(0, 0, 1),
        ),
        "beta_joint": FusionJoint(
            name="beta_joint", defining_component="asm", motion_type="rigid",
            occurrence_one_path="child v1:1", occurrence_one_clean="child",
            occurrence_two_path="mid v1:1", occurrence_two_clean="mid",
            origin_global_m=(0, 0, 0), origin_source="geometry.origin",
            axis_vector=(0, 0, 1),
        ),
        # User-tagged closing — note the extractor strips 'closing_'
        # from the name, so this joint arrives at the model layer as
        # name='gamma_joint' with is_closing=True.  Synthetic test
        # mimics that post-extractor state directly.
        "gamma_joint": FusionJoint(
            name="gamma_joint", raw_name="closing_gamma_joint",
            defining_component="asm", motion_type="rigid",
            occurrence_one_path="child v1:1", occurrence_one_clean="child",
            occurrence_two_path="base v1:1", occurrence_two_clean="base",
            origin_global_m=(0, 0, 0), origin_source="geometry.origin",
            axis_vector=(0, 0, 1),
            is_closing=True, is_passive=True,
        ),
    }

    model = build_model(snap, _make_logger())

    # The user-tagged joint is in closing_joints, NOT in the URDF tree.
    assert "gamma_joint" in model.closing_joints, \
        f"user-tagged closing joint missing from sidecar; got {list(model.closing_joints.keys())}"
    assert "gamma_joint" not in model.joints, \
        f"closing joint must not appear in URDF tree; got {list(model.joints.keys())}"
    closing = model.closing_joints["gamma_joint"]
    assert closing.is_closing
    assert closing.closing_source == "user_tag", \
        f"expected source='user_tag', got '{closing.closing_source}'"
    assert closing.is_passive, "closing joints are implicitly passive"

    # No auto-detect warning should fire — the user chose explicitly.
    auto_warnings = [
        w for w in model.warnings
        if "auto" in w.lower() and "closing" in w.lower()
    ]
    assert not auto_warnings, \
        f"explicit user_tag should not trigger auto-detect warnings; got: {auto_warnings}"

    # The remaining URDF tree is clean: base → mid → child.
    parents_of_child = [j for j in model.joints.values() if j.child_link == "child"]
    assert len(parents_of_child) == 1, \
        f"expected exactly one tree parent for 'child', got {len(parents_of_child)}"
    assert parents_of_child[0].name == "beta_joint"

    print("  user_tagged_closing_joint: PASS")


def test_reused_component_gets_unique_urdf_names():
    """Two occurrences of the same Fusion component (``leva:1``,
    ``leva:2``) in the same assembly used to collide on the
    ``f"{asm}_{name}"`` fallback — both got ``ROOT_leva``, the second
    overwrote the first in ``model.links``, and one occurrence
    silently disappeared from the URDF.  Now the renamer appends
    ``_2``, ``_3``, … to keep every occurrence as a distinct link."""
    from ..core.data_types import (
        FusionSnapshot, FusionOccurrence, FusionJoint, InertiaTensor, Transform3D,
    )
    from ..core.robot_model import build_model

    snap = FusionSnapshot(design_name="reuse_test", design_name_clean="reuse_test")
    snap.occurrences = {
        "asm v1:1": FusionOccurrence(
            full_path="asm v1:1", clean_name="asm",
            path_segments=["asm"], depth=0, is_subassembly=True,
            local_transform=Transform3D(),
        ),
        "asm v1:1+base v1:1": FusionOccurrence(
            full_path="asm v1:1+base v1:1", clean_name="base",
            path_segments=["asm", "base"], depth=1, is_subassembly=False,
            mass_kg=1.0, body_count=1, local_transform=Transform3D(),
        ),
        "asm v1:1+leva v1:1": FusionOccurrence(
            full_path="asm v1:1+leva v1:1", clean_name="leva",
            path_segments=["asm", "leva"], depth=1, is_subassembly=False,
            mass_kg=0.05, body_count=1,
            global_position=(0.05, 0.0, 0.0),
            local_transform=Transform3D(),
        ),
        "asm v1:1+leva v1:2": FusionOccurrence(
            full_path="asm v1:1+leva v1:2", clean_name="leva",
            path_segments=["asm", "leva"], depth=1, is_subassembly=False,
            mass_kg=0.05, body_count=1,
            global_position=(-0.05, 0.0, 0.0),
            local_transform=Transform3D(),
        ),
    }
    snap.joints = {
        "j_left": FusionJoint(
            name="j_left", defining_component="asm", motion_type="rigid",
            occurrence_one_path="leva v1:1", occurrence_one_clean="leva",
            occurrence_two_path="base v1:1", occurrence_two_clean="base",
            origin_global_m=(0, 0, 0), origin_source="geometry.origin",
            axis_vector=(0, 0, 1),
        ),
        "j_right": FusionJoint(
            name="j_right", defining_component="asm", motion_type="rigid",
            occurrence_one_path="leva v1:2", occurrence_one_clean="leva",
            occurrence_two_path="base v1:1", occurrence_two_clean="base",
            origin_global_m=(0, 0, 0), origin_source="geometry.origin",
            axis_vector=(0, 0, 1),
        ),
    }

    model = build_model(snap, _make_logger())

    # Both leva occurrences must end up as distinct URDF links — that's
    # the regression we're guarding against.  The exact suffixed names
    # are an implementation detail; what matters is uniqueness +
    # presence + distinct mesh paths.
    leva_links = [
        ln for ln in model.links.values() if ln.clean_name == "leva"
    ]
    assert len(leva_links) == 2, (
        f"expected 2 distinct leva links (one per occurrence); got "
        f"{len(leva_links)}: {[l.urdf_name for l in leva_links]}"
    )
    urdf_names = {l.urdf_name for l in leva_links}
    assert len(urdf_names) == 2, (
        f"reused-component links must have UNIQUE urdf_names; "
        f"got {urdf_names}"
    )
    mesh_paths = {l.mesh_visual for l in leva_links}
    assert len(mesh_paths) == 2, (
        f"reused-component links must point at DIFFERENT mesh files; "
        f"got {mesh_paths}"
    )
    # Both joints survive and reference distinct child links.
    assert len(model.joints) == 2
    j_children = {j.child_link for j in model.joints.values()}
    assert j_children == urdf_names, (
        f"each joint should reference one of the unique leva URDF names; "
        f"joint children={j_children}, leva urdf names={urdf_names}"
    )

    print("  reused_component_gets_unique_urdf_names: PASS")


def test_nested_subasm_link_origin_invariant_under_parent_rotation():
    """A leaf component nested inside a sub-asm that itself sits with a
    non-identity world rotation must produce the SAME relative
    geometry (joint origins, visual bake offsets) as when the same
    sub-asm is exported standalone — these are link-local quantities
    and shouldn't depend on where the assembly sits in the world.

    Earlier the global-position calculation summed
    ``occurrence.transform.translation`` up the assemblyContext chain,
    silently dropping intermediate sub-asm rotations.  The result:
    when a gripper sub-asm was mounted under Assem1 with a non-identity
    mount rotation, every nested leaf's "global position" came out as
    a vector sum instead of a properly-rotated chain.  All bake
    offsets and joint origins inside the gripper then computed against
    the wrong frame, scattering the visual meshes across space.

    This test fakes the snapshot-level data the way Fusion's
    ``transform2.translation`` would expose it (already composed
    through the chain), and asserts the resulting joint origins are
    identical to a non-rotated parent setup.  Locks in that the
    plugin reads ``transform2`` for global positions, not the buggy
    chain sum.
    """
    from ..core.data_types import (
        FusionSnapshot, FusionOccurrence, FusionJoint, Transform3D,
    )
    from ..core.robot_model import build_model

    # ── Reference (parent at identity rotation) ──
    snap_id = FusionSnapshot(design_name="ref", design_name_clean="ref")
    snap_id.occurrences = {
        "asm v1:1": FusionOccurrence(
            full_path="asm v1:1", clean_name="asm",
            path_segments=["asm"], depth=0, is_subassembly=True,
            local_transform=Transform3D(),
        ),
        "asm v1:1+a v1:1": FusionOccurrence(
            full_path="asm v1:1+a v1:1", clean_name="a",
            path_segments=["asm", "a"], depth=1, is_subassembly=False,
            mass_kg=1.0, body_count=1,
            global_position=(0.0, 0.0, 0.0),
            local_transform=Transform3D(),
        ),
        "asm v1:1+b v1:1": FusionOccurrence(
            full_path="asm v1:1+b v1:1", clean_name="b",
            path_segments=["asm", "b"], depth=1, is_subassembly=False,
            mass_kg=1.0, body_count=1,
            # b sits at +X relative to a in the design.
            global_position=(0.10, 0.0, 0.0),
            local_transform=Transform3D(),
        ),
    }
    snap_id.joints = {
        "j": FusionJoint(
            name="j", defining_component="asm", motion_type="rigid",
            occurrence_one_path="b v1:1", occurrence_one_clean="b",
            occurrence_two_path="a v1:1", occurrence_two_clean="a",
            origin_global_m=(0.10, 0.0, 0.0), origin_source="geometry.origin",
            axis_vector=(0, 0, 1),
        ),
    }
    model_id = build_model(snap_id, _make_logger())
    j_id = model_id.joints["j"]

    # ── Same design, but parent assembly has a non-identity world
    # rotation (Z 90°).  In a real Fusion design, transform2 already
    # composes the chain, so b's global_position is the rotated
    # (0.10, 0, 0) → (0, 0.10, 0).  The point of this test: that
    # rotated global is what should produce the SAME joint-origin
    # in a-local frame, NOT a different one.
    import math
    cz, sz = math.cos(math.pi / 2), math.sin(math.pi / 2)
    rot_z90 = (cz, -sz, 0.0,
                sz,  cz, 0.0,
                0.0, 0.0, 1.0)

    snap_rot = FusionSnapshot(design_name="ref", design_name_clean="ref")
    snap_rot.occurrences = {
        "asm v1:1": FusionOccurrence(
            full_path="asm v1:1", clean_name="asm",
            path_segments=["asm"], depth=0, is_subassembly=True,
            local_transform=Transform3D(),
        ),
        "asm v1:1+a v1:1": FusionOccurrence(
            full_path="asm v1:1+a v1:1", clean_name="a",
            path_segments=["asm", "a"], depth=1, is_subassembly=False,
            mass_kg=1.0, body_count=1,
            global_position=(0.0, 0.0, 0.0),
            transform2=Transform3D(rotation=rot_z90),
            local_transform=Transform3D(),
        ),
        "asm v1:1+b v1:1": FusionOccurrence(
            full_path="asm v1:1+b v1:1", clean_name="b",
            path_segments=["asm", "b"], depth=1, is_subassembly=False,
            mass_kg=1.0, body_count=1,
            # rotated equivalent of (0.10, 0, 0) — what transform2 gives.
            global_position=(0.0, 0.10, 0.0),
            transform2=Transform3D(rotation=rot_z90),
            local_transform=Transform3D(),
        ),
    }
    snap_rot.joints = {
        "j": FusionJoint(
            name="j", defining_component="asm", motion_type="rigid",
            occurrence_one_path="b v1:1", occurrence_one_clean="b",
            occurrence_two_path="a v1:1", occurrence_two_clean="a",
            origin_global_m=(0.0, 0.10, 0.0), origin_source="geometry.origin",
            axis_vector=(0, 0, 1),
        ),
    }
    model_rot = build_model(snap_rot, _make_logger())
    j_rot = model_rot.joints["j"]

    # The joint origin in a's LOCAL frame must be (0.10, 0, 0) in BOTH
    # cases — the assembly's world rotation doesn't change link-local
    # geometry.  Tolerance for rotation float arithmetic.
    for axis_idx, expected in enumerate((0.10, 0.0, 0.0)):
        assert abs(j_id.origin_xyz[axis_idx] - expected) < 1e-6, (
            f"reference (no rotation): joint origin axis {axis_idx} "
            f"got {j_id.origin_xyz[axis_idx]}, expected {expected}"
        )
        assert abs(j_rot.origin_xyz[axis_idx] - expected) < 1e-6, (
            f"rotated assembly: joint origin axis {axis_idx} got "
            f"{j_rot.origin_xyz[axis_idx]}, expected {expected} — "
            f"the rotated parent should NOT change link-local geometry"
        )

    print("  nested_subasm_link_origin_invariant_under_parent_rotation: PASS")


def test_nested_subasm_joint_axis_invariant_under_parent_rotation():
    """A revolute joint defined inside a sub-asm that's mounted with a
    non-identity world rotation must produce the SAME URDF
    ``<axis xyz>`` as when the same sub-asm is exported standalone.
    The axis lives in the joint's local frame — assembly placement
    can't change it.

    Earlier the plugin treated ``fj.axis_vector`` as already in world
    frame.  Empirically it's in the joint's DEFINING-COMPONENT frame
    (Fusion's API doc lies — axis came out wrong for nested joints),
    so we now lift it through the defining assembly's
    ``global_rotation`` before transforming into the joint's local
    frame.  This test fakes the same sub-asm at identity vs. rotated
    and asserts the resulting URDF axes match.
    """
    import math
    from ..core.data_types import (
        FusionSnapshot, FusionOccurrence, FusionJoint, Transform3D,
    )
    from ..core.robot_model import build_model

    cz, sz = math.cos(math.pi / 2), math.sin(math.pi / 2)
    rot_z90 = (cz, -sz, 0.0,
                sz,  cz, 0.0,
                0.0, 0.0, 1.0)

    def _build(asm_rotation, design_clean_name="ref"):
        snap = FusionSnapshot(
            design_name=design_clean_name, design_name_clean=design_clean_name)
        # The joint sits inside ``inner`` sub-asm — defining_component
        # is ``inner``.  Two leaf children (a, b).
        snap.occurrences = {
            "inner v1:1": FusionOccurrence(
                full_path="inner v1:1", clean_name="inner",
                path_segments=["inner"], depth=0, is_subassembly=True,
                local_transform=Transform3D(),
                transform2=Transform3D(rotation=asm_rotation),
            ),
            "inner v1:1+a v1:1": FusionOccurrence(
                full_path="inner v1:1+a v1:1", clean_name="a",
                path_segments=["inner", "a"], depth=1, is_subassembly=False,
                mass_kg=1.0, body_count=1,
                global_position=(0.0, 0.0, 0.0),
                transform2=Transform3D(rotation=asm_rotation),
                local_transform=Transform3D(),
            ),
            "inner v1:1+b v1:1": FusionOccurrence(
                full_path="inner v1:1+b v1:1", clean_name="b",
                path_segments=["inner", "b"], depth=1, is_subassembly=False,
                mass_kg=1.0, body_count=1,
                # Position in world is rotated when assembly is rotated;
                # b sits at +X relative to a in the design frame.
                global_position=(
                    asm_rotation[0] * 0.10,
                    asm_rotation[3] * 0.10,
                    asm_rotation[6] * 0.10,
                ),
                transform2=Transform3D(rotation=asm_rotation),
                local_transform=Transform3D(),
            ),
        }
        # Joint defined inside ``inner``.  Axis vector in inner's
        # local frame: (0, 0, 1).  Origin in inner's local frame:
        # (0.10, 0, 0).
        snap.joints = {
            "j": FusionJoint(
                name="j", defining_component="inner",
                motion_type="revolute",
                occurrence_one_path="b v1:1", occurrence_one_clean="b",
                occurrence_two_path="a v1:1", occurrence_two_clean="a",
                origin_global_m=(0.10, 0.0, 0.0),
                origin_source="geometry.origin",
                axis_vector=(0.0, 0.0, 1.0),  # in inner's local frame
                has_rotation_limits=True,
                rotation_min=-1.0, rotation_max=1.0,
            ),
        }
        return build_model(snap, _make_logger())

    # Identity reference + rotated by Z90 produce the same URDF axis.
    identity = (1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0)
    j_id = _build(identity).joints["j"]
    j_rot = _build(rot_z90).joints["j"]

    for axis_idx, expected in enumerate((0.0, 0.0, 1.0)):
        assert abs(j_id.axis[axis_idx] - expected) < 1e-6, (
            f"identity reference: axis component {axis_idx} got "
            f"{j_id.axis[axis_idx]}, expected {expected}"
        )
        assert abs(j_rot.axis[axis_idx] - expected) < 1e-6, (
            f"rotated assembly: axis component {axis_idx} got "
            f"{j_rot.axis[axis_idx]}, expected {expected} — the "
            f"defining sub-asm's rotation should NOT change the "
            f"link-local axis"
        )

    print("  nested_subasm_joint_axis_invariant_under_parent_rotation: PASS")


def test_assembly_links_list_no_clean_name_duplicates():
    """``AssemblyInfo.links`` must contain URDF names (unique) only —
    not clean_names (which can have duplicates for re-used components).

    Earlier ``_build_assemblies`` appended ``clean_name`` and then
    ``_build_links`` appended ``urdf_name`` to the same list, so a
    fastener used 4 times in the same assembly produced 5 entries
    (4× clean_name + 4× urdf_name dedup-suffixed).  Visible in
    robot_data.yaml as a long list with the same screw clean-name
    repeated alongside the suffixed unique versions."""
    from ..core.data_types import (
        FusionSnapshot, FusionOccurrence, FusionJoint, InertiaTensor, Transform3D,
    )
    from ..core.robot_model import build_model

    snap = FusionSnapshot(design_name="reuse_check", design_name_clean="reuse_check")
    snap.occurrences = {
        "asm v1:1": FusionOccurrence(
            full_path="asm v1:1", clean_name="asm",
            path_segments=["asm"], depth=0, is_subassembly=True,
            local_transform=Transform3D(),
        ),
        # Re-used component (M3 fastener) used four times in the same asm.
        "asm v1:1+M3 v1:1": FusionOccurrence(
            full_path="asm v1:1+M3 v1:1", clean_name="M3",
            path_segments=["asm", "M3"], depth=1, is_subassembly=False,
            mass_kg=0.001, body_count=1,
            global_position=(0.01, 0, 0),
            local_transform=Transform3D(),
        ),
        "asm v1:1+M3 v1:2": FusionOccurrence(
            full_path="asm v1:1+M3 v1:2", clean_name="M3",
            path_segments=["asm", "M3"], depth=1, is_subassembly=False,
            mass_kg=0.001, body_count=1,
            global_position=(0.02, 0, 0),
            local_transform=Transform3D(),
        ),
        "asm v1:1+M3 v1:3": FusionOccurrence(
            full_path="asm v1:1+M3 v1:3", clean_name="M3",
            path_segments=["asm", "M3"], depth=1, is_subassembly=False,
            mass_kg=0.001, body_count=1,
            global_position=(0.03, 0, 0),
            local_transform=Transform3D(),
        ),
        "asm v1:1+M3 v1:4": FusionOccurrence(
            full_path="asm v1:1+M3 v1:4", clean_name="M3",
            path_segments=["asm", "M3"], depth=1, is_subassembly=False,
            mass_kg=0.001, body_count=1,
            global_position=(0.04, 0, 0),
            local_transform=Transform3D(),
        ),
        # Anchor for the kinematic tree.
        "asm v1:1+base v1:1": FusionOccurrence(
            full_path="asm v1:1+base v1:1", clean_name="base",
            path_segments=["asm", "base"], depth=1, is_subassembly=False,
            mass_kg=1.0, body_count=1,
            local_transform=Transform3D(),
        ),
    }
    snap.joints = {
        "j_1": FusionJoint(name="j_1", defining_component="asm",
            motion_type="rigid",
            occurrence_one_path="M3 v1:1", occurrence_one_clean="M3",
            occurrence_two_path="base v1:1", occurrence_two_clean="base",
            axis_vector=(0, 0, 1)),
        "j_2": FusionJoint(name="j_2", defining_component="asm",
            motion_type="rigid",
            occurrence_one_path="M3 v1:2", occurrence_one_clean="M3",
            occurrence_two_path="base v1:1", occurrence_two_clean="base",
            axis_vector=(0, 0, 1)),
        "j_3": FusionJoint(name="j_3", defining_component="asm",
            motion_type="rigid",
            occurrence_one_path="M3 v1:3", occurrence_one_clean="M3",
            occurrence_two_path="base v1:1", occurrence_two_clean="base",
            axis_vector=(0, 0, 1)),
        "j_4": FusionJoint(name="j_4", defining_component="asm",
            motion_type="rigid",
            occurrence_one_path="M3 v1:4", occurrence_one_clean="M3",
            occurrence_two_path="base v1:1", occurrence_two_clean="base",
            axis_vector=(0, 0, 1)),
    }

    model = build_model(snap, _make_logger())
    asm = model.assemblies.get("asm")
    assert asm is not None

    # Every entry in the list must correspond to a real model link
    # (urdf_name, not clean_name) AND every entry is unique.
    assert len(asm.links) == len(set(asm.links)), (
        f"assembly.links has duplicates: {asm.links}"
    )
    for n in asm.links:
        assert n in model.links, (
            f"assembly.links contains '{n}' which is not a URDF link "
            f"(probably a clean_name leaking through)"
        )
    # All four M3 occurrences must show up under their unique URDF names.
    m3_links = [n for n in asm.links if model.links[n].clean_name == "M3"]
    assert len(m3_links) == 4, (
        f"all four M3 occurrences should appear with unique urdf_names; "
        f"got {m3_links}"
    )

    print("  assembly_links_list_no_clean_name_duplicates: PASS")


def test_internal_rigid_group_rigid_joints_are_quietly_dropped():
    """Fusion fasteners often add many rigid joints between hardware and the
    part they are mounted to.  When the hardware is intentionally inside the
    same rigid group, those joints are redundant and should not flood the
    export warnings.  Non-rigid internal joints still warn because they would
    lose intended motion."""
    from ..core.data_types import (
        FusionSnapshot, FusionOccurrence, FusionJoint, RigidGroupInfo,
        Transform3D,
    )
    from ..core.robot_model import build_model

    snap = FusionSnapshot(design_name="fastener_rg", design_name_clean="fastener_rg")
    snap.occurrences = {
        "asm v1:1": FusionOccurrence(
            full_path="asm v1:1", clean_name="asm",
            path_segments=["asm"], depth=0, is_subassembly=True,
            local_transform=Transform3D(),
        ),
        "asm v1:1+plate v1:1": FusionOccurrence(
            full_path="asm v1:1+plate v1:1", clean_name="plate",
            path_segments=["asm", "plate"], depth=1, is_subassembly=False,
            mass_kg=1.0, body_count=1, local_transform=Transform3D(),
        ),
        "asm v1:1+base v1:1": FusionOccurrence(
            full_path="asm v1:1+base v1:1", clean_name="base",
            path_segments=["asm", "base"], depth=1, is_subassembly=False,
            mass_kg=2.0, body_count=1, local_transform=Transform3D(),
        ),
        "asm v1:1+M5 v1:1": FusionOccurrence(
            full_path="asm v1:1+M5 v1:1", clean_name="M5",
            path_segments=["asm", "M5"], depth=1, is_subassembly=False,
            mass_kg=0.01, body_count=1, local_transform=Transform3D(),
        ),
        "asm v1:1+hinge v1:1": FusionOccurrence(
            full_path="asm v1:1+hinge v1:1", clean_name="hinge",
            path_segments=["asm", "hinge"], depth=1, is_subassembly=False,
            mass_kg=0.1, body_count=1, local_transform=Transform3D(),
        ),
    }
    snap.rigid_groups.append(RigidGroupInfo(
        name="plate_link",
        occurrence_paths=[
            "asm v1:1+plate v1:1",
            "asm v1:1+M5 v1:1",
            "asm v1:1+hinge v1:1",
        ],
        member_clean_names=["plate", "M5", "hinge"],
    ))
    snap.joints = {
        "mount": FusionJoint(
            name="mount", defining_component="asm", motion_type="rigid",
            occurrence_one_path="plate v1:1", occurrence_one_clean="plate",
            occurrence_two_path="base v1:1", occurrence_two_clean="base",
            axis_vector=(0, 0, 1),
        ),
        "Rigid_1": FusionJoint(
            name="Rigid_1", defining_component="asm", motion_type="rigid",
            occurrence_one_path="M5 v1:1", occurrence_one_clean="M5",
            occurrence_two_path="plate v1:1", occurrence_two_clean="plate",
            axis_vector=(0, 0, 1),
        ),
        "hinge_axis": FusionJoint(
            name="hinge_axis", defining_component="asm", motion_type="revolute",
            occurrence_one_path="hinge v1:1", occurrence_one_clean="hinge",
            occurrence_two_path="plate v1:1", occurrence_two_clean="plate",
            axis_vector=(0, 0, 1),
        ),
    }

    model = build_model(snap, _make_logger())
    internal_warnings = [
        w for w in model.warnings if "internal to a rigid group" in w
    ]
    assert any("hinge_axis" in w for w in internal_warnings), (
        f"non-rigid internal joint should still warn; got {model.warnings}"
    )
    assert not any("Rigid_1" in w for w in internal_warnings), (
        f"redundant rigid fastener joint should not warn; got {model.warnings}"
    )

    print("  internal_rigid_group_rigid_joints_are_quietly_dropped: PASS")


def test_zero_joint_limit_preserved_not_replaced_by_default():
    """A Fusion joint with upper=0 (or lower=0) used to come out of the
    plugin as upper=π / lower=-π because the limit-merge code did
    ``fj.rotation_max or math.pi`` — Python's ``or`` treats 0.0 as
    falsy and silently substitutes the default.  Result: a servo with
    a 0-radian upper limit ended up with a 180° upper limit in the
    URDF, the user could drive past Fusion's design intent, and the
    four-bar mechanism over-extended past its kinematic singularity
    in Isaac Sim.  Compare against None explicitly so 0 stays 0."""
    from ..core.data_types import (
        FusionSnapshot, FusionOccurrence, FusionJoint, InertiaTensor, Transform3D,
    )
    from ..core.robot_model import build_model

    snap = FusionSnapshot(design_name="zero_lim", design_name_clean="zero_lim")
    snap.occurrences = {
        "asm v1:1": FusionOccurrence(
            full_path="asm v1:1", clean_name="asm",
            path_segments=["asm"], depth=0, is_subassembly=True,
            local_transform=Transform3D(),
        ),
        "asm v1:1+base v1:1": FusionOccurrence(
            full_path="asm v1:1+base v1:1", clean_name="base",
            path_segments=["asm", "base"], depth=1, is_subassembly=False,
            mass_kg=1.0, body_count=1, local_transform=Transform3D(),
        ),
        "asm v1:1+arm v1:1": FusionOccurrence(
            full_path="asm v1:1+arm v1:1", clean_name="arm",
            path_segments=["asm", "arm"], depth=1, is_subassembly=False,
            mass_kg=0.5, body_count=1, local_transform=Transform3D(),
        ),
    }
    # Revolute joint with upper=0 explicitly enabled.  Lower set to a
    # non-zero negative so we can isolate the upper=0 collapse.
    snap.joints = {
        "j_servo": FusionJoint(
            name="j_servo", defining_component="asm", motion_type="revolute",
            occurrence_one_path="arm v1:1", occurrence_one_clean="arm",
            occurrence_two_path="base v1:1", occurrence_two_clean="base",
            origin_global_m=(0, 0, 0), origin_source="geometry.origin",
            axis_vector=(0, 0, 1),
            has_rotation_limits=True,
            rotation_min=-1.0,
            rotation_max=0.0,   # the trap value — 0.0 is falsy
        ),
    }
    model = build_model(snap, _make_logger())
    j = model.joints["j_servo"]
    assert j.limits is not None
    assert j.limits.upper == 0.0, (
        f"upper=0 must survive; got {j.limits.upper} (likely π if the "
        f"``or`` fallback bit again)"
    )
    assert j.limits.lower == -1.0

    print("  zero_joint_limit_preserved_not_replaced_by_default: PASS")


def test_passive_joint_propagates_flag():
    """A FusionJoint with ``is_passive=True`` (and no closing flag)
    stays in the URDF tree but its JointNode carries
    ``is_passive=True`` so the YAML emitter writes ``passive: true``
    and downstream consumers can suppress the drive."""
    from ..core.data_types import (
        FusionSnapshot, FusionOccurrence, FusionJoint, InertiaTensor, Transform3D,
    )
    from ..core.robot_model import build_model

    snap = FusionSnapshot(design_name="idler", design_name_clean="idler")
    snap.occurrences = {
        "asm v1:1": FusionOccurrence(
            full_path="asm v1:1", clean_name="asm",
            path_segments=["asm"], depth=0, is_subassembly=True,
            local_transform=Transform3D(),
        ),
        "asm v1:1+chassis v1:1": FusionOccurrence(
            full_path="asm v1:1+chassis v1:1", clean_name="chassis",
            path_segments=["asm", "chassis"], depth=1, is_subassembly=False,
            mass_kg=2.0, body_count=1, local_transform=Transform3D(),
        ),
        "asm v1:1+wheel v1:1": FusionOccurrence(
            full_path="asm v1:1+wheel v1:1", clean_name="wheel",
            path_segments=["asm", "wheel"], depth=1, is_subassembly=False,
            mass_kg=0.2, body_count=1, local_transform=Transform3D(),
        ),
    }
    snap.joints = {
        "wheel_spin": FusionJoint(
            name="wheel_spin", raw_name="passive_wheel_spin",
            defining_component="asm", motion_type="revolute",
            occurrence_one_path="wheel v1:1", occurrence_one_clean="wheel",
            occurrence_two_path="chassis v1:1", occurrence_two_clean="chassis",
            origin_global_m=(0, 0, 0), origin_source="geometry.origin",
            axis_vector=(0, 1, 0),
            is_passive=True,
        ),
    }

    model = build_model(snap, _make_logger())

    assert "wheel_spin" in model.joints, \
        f"passive joint must stay in URDF tree; got {list(model.joints.keys())}"
    assert "wheel_spin" not in model.closing_joints, \
        "passive without closing tag must NOT be routed to sidecar"
    j = model.joints["wheel_spin"]
    assert j.is_passive, "passive flag must propagate to JointNode"
    assert not j.is_closing, "passive alone does not imply closing"

    print("  passive_joint_propagates_flag: PASS")


# ──────────────────────────────────────────────
# Runner
# ──────────────────────────────────────────────

def run_all():
    print("Running robot_model tests...\n")

    test_build_mini_model()
    test_root_detection()
    test_kinematic_chain_order()
    test_joint_origins_parent_relative()
    test_joint_limits()
    test_name_collision_resolution()
    test_assembly_hierarchy()
    test_tree_connectivity()
    test_multi_parent_detection()
    test_user_tagged_closing_joint()
    test_reused_component_gets_unique_urdf_names()
    test_zero_joint_limit_preserved_not_replaced_by_default()
    test_assembly_links_list_no_clean_name_duplicates()
    test_internal_rigid_group_rigid_joints_are_quietly_dropped()
    test_nested_subasm_link_origin_invariant_under_parent_rotation()
    test_nested_subasm_joint_axis_invariant_under_parent_rotation()
    test_passive_joint_propagates_flag()
    test_link_properties_preserved()
    test_rigid_group_merge_two_members()
    test_rigid_group_merge_anchor_is_heaviest()
    test_rigid_group_frame_member_becomes_anchor()
    test_rigid_group_frame_anchor_overrides_movable_joint_origin()
    test_base_frame_anchor_rotation_defines_root_link_frame()
    test_rigid_group_merge_collision_member_excluded_from_anchor_choice()
    test_rigid_group_body_owning_subassembly_with_design_root_joints()
    test_auto_rigid_island_collapses_nested_subassemblies_without_joints()
    test_auto_rigid_island_preserves_articulated_subassembly()
    test_orphan_link_is_error()
    test_unreferenced_empty_occurrence_is_dropped()
    test_frame_only_child_joint_forced_fixed()
    test_real_snapshot()

    print("\n✓ All robot_model tests passed!")


if __name__ == '__main__':
    run_all()
