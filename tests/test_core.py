"""
Tests for fusion2URDF core helpers.

Run from the PARENT directory of fusion2URDF/:
    python -m fusion2URDF.tests.test_core

Or use the helper script:
    cd fusion2URDF
    python run_tests.py

Tests everything that doesn't need the Fusion API.
"""


def test_clean_name():
    from ..utils.helpers import clean_name
    
    assert clean_name('head_link v8') == 'head_link'
    assert clean_name('turret v25:1') == 'turret'
    assert clean_name('dummy_zed2i v4') == 'dummy_zed2i'
    assert clean_name('neck_link v6') == 'neck_link'
    assert clean_name('basic_platform v18') == 'basic_platform'
    assert clean_name('My Part (v2):1') == 'My_Part_v2'
    assert clean_name('') == ''
    print("  clean_name: PASS")


def test_parse_occurrence_path():
    from ..utils.helpers import parse_occurrence_path
    
    assert parse_occurrence_path('turret v25:1+head_link v8:1') == ['turret', 'head_link']
    assert parse_occurrence_path('turret v25:1+dummy_zed2i v4:1+zed2i_link v5:1') == ['turret', 'dummy_zed2i', 'zed2i_link']
    assert parse_occurrence_path('base_link v5:1') == ['base_link']
    assert parse_occurrence_path('asm v1:1+!cxh_wheel v2:1') == ['asm', 'wheel']
    assert parse_occurrence_path('') == []
    assert parse_occurrence_path('a v1:1+b v2:1+c v3:1+d v4:1') == ['a', 'b', 'c', 'd']
    print("  parse_occurrence_path: PASS")


def test_frame_prefix_helpers():
    from ..utils.helpers import is_frame_only_name, strip_frame_prefix

    assert not is_frame_only_name("frame")
    assert not is_frame_only_name("frame_imu")
    assert not is_frame_only_name("frame-camera_optical")
    assert not is_frame_only_name("Frame_IMU")
    assert is_frame_only_name("!frame_imu")
    assert is_frame_only_name("!Frame-camera_optical")
    assert not is_frame_only_name("reference_frame")
    assert not is_frame_only_name("")

    assert strip_frame_prefix("frame_imu") == "frame_imu"
    assert strip_frame_prefix("frame-camera") == "frame-camera"
    assert strip_frame_prefix("Frame_IMU") == "Frame_IMU"
    assert strip_frame_prefix("!frame_imu") == "imu"
    assert strip_frame_prefix("!frame") == ""
    assert strip_frame_prefix("frame") == "frame"
    assert strip_frame_prefix("plain_name") == "plain_name"

    print("  frame_prefix_helpers: PASS")


def test_yaml_safe_name():
    """``!`` is a YAML tag marker; raw Fusion source names that keep
    their reserved prefix (e.g. in ``merged_from`` traceability lists)
    must have it stripped before they hit YAML, otherwise the document
    is unparseable by ``yaml.safe_load``.
    """
    from ..utils.helpers import yaml_safe_name

    assert yaml_safe_name("!frame_base_link:1") == "frame_base_link:1"
    assert yaml_safe_name("!collision_proxy") == "collision_proxy"
    assert yaml_safe_name("!dummy_camera") == "dummy_camera"
    # Plain names pass through unchanged.
    assert yaml_safe_name("base_link") == "base_link"
    assert yaml_safe_name("Cover:1") == "Cover:1"
    # Edge cases.
    assert yaml_safe_name("") == ""
    assert yaml_safe_name(None) is None
    # Only the leading marker is removed; later ``!`` characters stay.
    assert yaml_safe_name("a!b") == "a!b"
    assert yaml_safe_name("!!double") == "!double"

    print("  yaml_safe_name: PASS")


def test_collision_override_prefix_helpers():
    from ..utils.helpers import (
        is_collision_component_name,
        is_collision_body_name,
        is_collision_excluded_body_name,
        is_dummy_assembly_name,
        clean_link_name,
        dispatch_metadata_prefix,
        is_accurate_collision_group_name,
        strip_joint_prefix,
        parse_collision_override_prefix,
        strip_collision_override_prefix,
        strip_link_metadata_prefixes,
    )

    assert parse_collision_override_prefix("!acc_gripper") == ("visual", "gripper")
    assert parse_collision_override_prefix("!cxh-body") == ("convex_hull", "body")
    assert parse_collision_override_prefix("!pri wheel") == ("primitive", "wheel")
    assert parse_collision_override_prefix("cxh_wheel") == ("", "cxh_wheel")
    assert parse_collision_override_prefix("accelerometer") == ("", "accelerometer")
    cxh = dispatch_metadata_prefix("!cxh_body")
    assert cxh["keyword"] == "cxh"
    assert cxh["kind"] == "collision_override"
    assert cxh["method"] == "convex_hull"
    assert cxh["remainder"] == "body"
    assert cxh["tagged"]
    assert dispatch_metadata_prefix("accelerometer")["kind"] == ""

    assert is_accurate_collision_group_name("!acc_gripper")
    assert strip_collision_override_prefix("!pri_wheel") == "wheel"
    assert clean_link_name("!cxh_body v3:1") == "body"
    assert strip_link_metadata_prefixes("!cxh_!frame_mount") == (
        "mount", "convex_hull", True
    )
    assert is_collision_component_name("!collision_proxy")
    assert not is_collision_component_name("collision_proxy")
    assert not is_collision_component_name("collisionless")
    assert is_collision_body_name("!collision_proxy")
    assert is_collision_body_name("!collision (1)")
    assert not is_collision_body_name("collision (1)")
    assert is_collision_excluded_body_name("!antenna")
    assert is_collision_excluded_body_name("!prop_guard_tip")
    assert not is_collision_excluded_body_name("antenna")
    assert not is_collision_excluded_body_name("!collision_proxy")
    assert is_dummy_assembly_name("!dummy_camera")
    assert not is_dummy_assembly_name("dummy_camera")
    assert strip_joint_prefix("!passive_idler") == ("idler", True, False)
    assert strip_joint_prefix("!closing_loop") == ("loop", True, True)
    assert strip_joint_prefix("!passive_!closing_loop") == ("loop", True, True)
    assert strip_joint_prefix("passive_idler") == ("passive_idler", False, False)

    print("  collision_override_prefix_helpers: PASS")


def test_unit_conversions():
    from ..utils.helpers import cm_to_m, mm_to_m, g_to_kg, g_mm2_to_kg_m2, kg_cm2_to_kg_m2
    
    assert abs(cm_to_m(100.0) - 1.0) < 1e-10
    assert abs(mm_to_m(1000.0) - 1.0) < 1e-10
    assert abs(g_to_kg(1000.0) - 1.0) < 1e-10
    assert abs(g_mm2_to_kg_m2(1e9) - 1.0) < 1e-10
    assert abs(kg_cm2_to_kg_m2(1e4) - 1.0) < 1e-10
    print("  unit_conversions: PASS")


def test_formatting():
    from ..utils.helpers import fmt, fmt_vec3, epsilon_clean
    
    assert epsilon_clean(1e-15) == 0.0
    assert epsilon_clean(0.5) == 0.5
    assert fmt(0.0000000001) == '0.0'
    assert fmt(1.23456789, 4) == '1.2346'
    assert fmt_vec3((0.17, 0.0, 0.327)) == '0.17 0.0 0.327'
    print("  formatting: PASS")


def test_data_types():
    from ..core.data_types import (
        FusionSnapshot, FusionOccurrence, FusionJoint,
        FusionBodyInfo, InertiaTensor, Transform3D
    )
    
    it = InertiaTensor(ixx=1.0, iyy=2.0, izz=3.0)
    assert it.as_tuple() == (1.0, 0.0, 0.0, 2.0, 0.0, 3.0)
    d = it.as_dict()
    assert d['ixx'] == 1.0 and d['iyy'] == 2.0
    
    t = Transform3D()
    assert t.is_identity
    t2 = Transform3D(translation=(1.0, 0.0, 0.0))
    assert not t2.is_identity
    
    occ = FusionOccurrence()
    assert occ.mass_kg == 0.0
    assert occ.is_subassembly == False
    assert occ.is_frame_only == False
    assert occ.bodies == []
    
    snap = FusionSnapshot(design_name='test')
    assert snap.total_joints == 0
    assert len(snap.occurrences) == 0
    
    print("  data_types: PASS")


def test_body_owning_subassembly_extraction_helpers():
    """Rigid groups must keep subassemblies that own direct bodies."""
    from ..core.fusion_extractor import (
        _extract_one_occurrence,
        _walk_leaf_occurrences,
    )
    from ..utils.logger import Logger

    class FakeBodies:
        def __init__(self, names):
            self._bodies = [type("Body", (), {"name": n})() for n in names]
            self.count = len(self._bodies)

        def item(self, index):
            return self._bodies[index]

    class FakeChildren:
        def __init__(self, children):
            self._children = list(children)
            self.count = len(self._children)

        def item(self, index):
            return self._children[index]

    class FakeComponent:
        def __init__(self, name, body_names, children=None):
            self.name = name
            self.bRepBodies = FakeBodies(body_names)
            self.occurrences = FakeChildren(children or [])

    class FakeOccurrence:
        def __init__(self, path, component, children=None):
            self.fullPathName = path
            self.component = component
            self.childOccurrences = FakeChildren(children or [])

    leaf_comp = FakeComponent("Cover:1", ["cover_body"])
    leaf = FakeOccurrence("Model:1+Fusalage:1+Cover:1", leaf_comp)
    sub_comp = FakeComponent("Fusalage:1", ["shell", "floor"], [leaf])
    sub = FakeOccurrence("Model:1+Fusalage:1", sub_comp, [leaf])

    fo = _extract_one_occurrence(sub, Logger(timestamps=False, quiet=True))
    assert fo.is_subassembly
    assert fo.body_count == 2, "direct bodies on subassemblies must be extracted"

    walked = [o.fullPathName for o in _walk_leaf_occurrences(sub)]
    assert walked == [
        "Model:1+Fusalage:1",
        "Model:1+Fusalage:1+Cover:1",
    ], walked

    print("  body_owning_subassembly_extraction_helpers: PASS")


def test_logger():
    from ..utils.logger import Logger
    
    log = Logger(timestamps=False)
    log('line1')
    log.section('SEC')
    log.indent('ind', 2)
    log.warning('warn')
    log.error('err')
    log.blank()
    
    assert len(log.lines) == 6
    assert log.lines[0] == 'line1'
    assert '=== SEC ===' in log.lines[1]
    assert '    ind' in log.lines[2]
    assert 'WARNING' in log.lines[3]
    assert 'ERROR' in log.lines[4]
    assert log.lines[5] == ''
    
    log.clear()
    assert len(log.lines) == 0
    
    print("  logger: PASS")


def test_json_serialization():
    from ..core.data_types import FusionSnapshot, FusionOccurrence, InertiaTensor
    from ..utils.helpers import snapshot_to_json
    import json
    
    snap = FusionSnapshot(design_name='test_design')
    occ = FusionOccurrence(
        full_path='turret v25:1+head_link v8:1',
        clean_name='head_link',
        mass_kg=3.454,
        global_position=(0.17, 0.0, 0.459),
        inertia_at_com=InertiaTensor(ixx=0.1276, iyy=0.01647, izz=0.1155),
    )
    snap.occurrences[occ.full_path] = occ
    
    json_str = snapshot_to_json(snap)
    parsed = json.loads(json_str)
    
    assert parsed['design_name'] == 'test_design'
    assert 'turret v25:1+head_link v8:1' in parsed['occurrences']
    assert parsed['occurrences']['turret v25:1+head_link v8:1']['mass_kg'] == 3.454
    
    print("  json_serialization: PASS")


def test_snapshot_report():
    from ..core.data_types import FusionSnapshot, FusionOccurrence, InertiaTensor, Transform3D
    from ..core.snapshot_report import generate_report
    
    snap = FusionSnapshot(
        design_name='basic_platform v18',
        design_name_clean='basic_platform',
        export_timestamp='2026-02-12T16:00:00',
        total_occurrences=1,
        total_leaf_components=1,
    )
    
    neck = FusionOccurrence(
        full_path='turret v25:1+neck_link v6:1',
        component_name='neck_link v6',
        clean_name='neck_link',
        path_segments=['turret', 'neck_link'],
        depth=1,
        is_subassembly=False,
        global_position=(0.170, 0.0, 0.4588),
        local_transform=Transform3D(translation=(0.0, 0.0, 0.132)),
        assembly_context_depth=1,
        mass_kg=3.454655,
        body_count=1,
        com_component_local=(0.000834, 0.088374, 0.131125),
        com_global=(0.170834, 0.088374, 0.589925),
        inertia_at_origin=InertiaTensor(ixx=1.357e-3, iyy=1.320e-3, izz=2.433e-4),
        inertia_at_com=InertiaTensor(ixx=1.276e-4, iyy=1.647e-5, izz=1.155e-4),
        bbox_size=(0.120016, 0.55850, 0.22850),
        material_name='Nylon 101',
    )
    snap.occurrences[neck.full_path] = neck
    
    report = generate_report(snap)
    
    assert '# Extraction Report' in report
    assert 'Quick Comparison Table' in report
    assert 'neck_link' in report
    assert 'Nylon 101' in report
    assert '3454.655' in report
    
    print("  snapshot_report: PASS")


def test_steiner_theorem():
    """Verify parallel axis theorem: point mass at (1,0,0)."""
    from ..core.data_types import InertiaTensor
    
    mass = 1.0
    cx, cy, cz = 1.0, 0.0, 0.0
    dd = cx*cx + cy*cy + cz*cz
    
    i_orig = InertiaTensor(ixx=0.0, iyy=1.0, izz=1.0)
    
    com_ixx = i_orig.ixx - mass * (dd - cx*cx)
    com_iyy = i_orig.iyy - mass * (dd - cy*cy)
    com_izz = i_orig.izz - mass * (dd - cz*cz)
    
    assert abs(com_ixx) < 1e-10
    assert abs(com_iyy) < 1e-10
    assert abs(com_izz) < 1e-10
    
    print("  steiner_theorem: PASS")


def test_read_joint_geo_origin_cm():
    """JointGeometry / JointOrigin origins must be read through several
    Fusion shapes - nested Harper joints previously returned null because
    only bare ``.origin`` was tried inside a silent ``except``."""
    from ..core.fusion_extractor import _read_joint_geo_origin_cm

    class _P:
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z

    class _GeoOrigin:
        def __init__(self):
            self.origin = _P(1.0, 2.0, 3.0)

    class _JointOriginViaGeometry:
        @property
        def origin(self):
            raise RuntimeError("Fusion InternalValidationError")

        def __init__(self):
            self.geometry = _GeoOrigin()

    class _ViaTransform:
        class _T:
            translation = _P(4.0, 5.0, 6.0)

        @property
        def origin(self):
            raise RuntimeError("no origin")

        @property
        def geometry(self):
            raise RuntimeError("no geometry")

        transform = _T()

    assert _read_joint_geo_origin_cm(_GeoOrigin()) == (1.0, 2.0, 3.0)
    assert _read_joint_geo_origin_cm(_JointOriginViaGeometry()) == (1.0, 2.0, 3.0)
    assert _read_joint_geo_origin_cm(_ViaTransform()) == (4.0, 5.0, 6.0)
    assert _read_joint_geo_origin_cm(None) is None
    print(" read_joint_geo_origin_cm: PASS")


def test_proxy_joint_skips_root_component():
    from ..core.fusion_extractor import _proxy_joint_for_assembly_context

    class _Log:
        def __call__(self, *a, **k):
            pass

        def warning(self, *a, **k):
            pass

    root = object()
    joint = object()
    out, used = _proxy_joint_for_assembly_context(
        joint, root, root, _Log(), "test"
    )
    assert out is joint and used is False
    print(" proxy_joint_skips_root_component: PASS")


def run_all():
    print("Running tests...\n")
    
    test_clean_name()
    test_parse_occurrence_path()
    test_frame_prefix_helpers()
    test_yaml_safe_name()
    test_collision_override_prefix_helpers()
    test_unit_conversions()
    test_formatting()
    test_data_types()
    test_body_owning_subassembly_extraction_helpers()
    test_logger()
    test_json_serialization()
    test_snapshot_report()
    test_steiner_theorem()
    test_read_joint_geo_origin_cm()
    test_proxy_joint_skips_root_component()
    
    print("\n✓ All tests passed!")


if __name__ == '__main__':
    run_all()
