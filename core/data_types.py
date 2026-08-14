"""
Data Types — Shared dataclasses for the Fusion URDF Exporter.

These types are used across all phases:
  Phase 1 (Extraction): FusionSnapshot, FusionOccurrence, FusionJoint
  Phase 2 (Model):      RobotModel, Assembly, LinkData, JointData  (future)
  Phase 3 (Generation): Read from RobotModel                       (future)

No Fusion API imports here — these are pure Python dataclasses.

Author: Adrian Valaker Eikeland
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any


# ──────────────────────────────────────────────
# Version
# ──────────────────────────────────────────────

EXPORTER_VERSION = "3.1.0"
DESIGN_ROOT_OCCURRENCE_PATH = "__design_root__"


# ──────────────────────────────────────────────
# Primitive types
# ──────────────────────────────────────────────

# 3D vector (x, y, z) in meters unless noted
Vec3 = Tuple[float, float, float]

# Rotation (roll, pitch, yaw) in radians
RPY = Tuple[float, float, float]


@dataclass
class InertiaTensor:
    """6-component symmetric inertia tensor (kg·m²)."""
    ixx: float = 0.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyy: float = 0.0
    iyz: float = 0.0
    izz: float = 0.0
    
    def as_tuple(self) -> Tuple[float, float, float, float, float, float]:
        return (self.ixx, self.ixy, self.ixz, self.iyy, self.iyz, self.izz)
    
    def as_dict(self) -> dict:
        return {
            'ixx': self.ixx, 'ixy': self.ixy, 'ixz': self.ixz,
            'iyy': self.iyy, 'iyz': self.iyz, 'izz': self.izz
        }


@dataclass
class Transform3D:
    """4x4 homogeneous transform, stored as translation + 3x3 rotation."""
    translation: Vec3 = (0.0, 0.0, 0.0)
    # Row-major 3x3 rotation matrix, flattened
    rotation: Tuple[float, ...] = (1,0,0, 0,1,0, 0,0,1)
    
    @property
    def is_identity(self) -> bool:
        tx, ty, tz = self.translation
        return (abs(tx) < 1e-10 and abs(ty) < 1e-10 and abs(tz) < 1e-10 and
                self.rotation == (1,0,0, 0,1,0, 0,0,1))


# ──────────────────────────────────────────────
# Phase 1: Extraction dataclasses
# ──────────────────────────────────────────────

@dataclass
class FusionBodyInfo:
    """Physical properties of a single bRepBody."""
    name: str = ""
    exclude_from_collision: bool = False
    
    mass_kg: float = 0.0
    volume_m3: float = 0.0
    density_kg_m3: float = 0.0
    area_m2: float = 0.0
    
    # Center of mass in COMPONENT-LOCAL coordinates (meters)
    com_component_local: Vec3 = (0.0, 0.0, 0.0)
    
    # Inertia at component origin (kg·m²)
    # This is what getXYZMomentsOfInertia() returns
    inertia_at_origin: Optional[InertiaTensor] = None
    
    # Bounding box in component-local coordinates (meters)
    bbox_min: Vec3 = (0.0, 0.0, 0.0)
    bbox_max: Vec3 = (0.0, 0.0, 0.0)
    
    # Material
    material_name: str = ""
    
    # Was inertia obtained from API or estimated from bbox?
    inertia_source: str = ""  # "api" or "bbox_estimate"


@dataclass
class FusionOccurrence:
    """Everything extracted from one Fusion occurrence."""
    
    # ── Identity ──
    full_path: str = ""             # "turret v25:1+head_link v8:1" — unique ID
    component_name: str = ""        # "head_link v8"
    clean_name: str = ""            # "head_link"
    
    # ── Hierarchy (parsed from full_path) ──
    path_segments: List[str] = field(default_factory=list)  # ["turret", "head_link"]
    parent_path: Optional[str] = None   # "turret v25:1" or None if root-level
    depth: int = 0                      # 0=root-level, 1=in subassembly, 2=nested deeper
    is_subassembly: bool = False        # Has child occurrences; may still own direct bodies
    child_count: int = 0                # Number of direct children
    is_frame_only: bool = False         # Explicit !frame_* attachment link; ignore bodies/geometry/inertia
    is_collision_geometry: bool = False # Explicit !collision_* component; not a URDF link
    collision_override: str = ""        # "visual", "convex_hull", "primitive", or "" from !acc/!cxh/!pri
    
    # ── Transforms ──
    # Local transform: occurrence.transform (relative to parent assembly)
    local_transform: Transform3D = field(default_factory=Transform3D)
    # transform2: Fusion's "design position" transform
    transform2: Optional[Transform3D] = None
    # Global position: accumulated through assemblyContext chain (meters)
    global_position: Vec3 = (0.0, 0.0, 0.0)
    # Full global transform (4x4, includes rotation)
    global_transform: Transform3D = field(default_factory=Transform3D)
    # How many assemblyContext levels were traversed
    assembly_context_depth: int = 0
    
    # ── Physical properties (summed over all bodies) ──
    mass_kg: float = 0.0
    volume_m3: float = 0.0
    density_kg_m3: float = 0.0
    area_m2: float = 0.0
    body_count: int = 0
    
    # Per-body breakdown (for debugging / validation)
    bodies: List[FusionBodyInfo] = field(default_factory=list)
    collision_excluded_body_names: List[str] = field(default_factory=list)
    collision_body_count: int = 0
    
    # Center of mass — COMPONENT-LOCAL coordinates (meters)
    # This is physicalProperties.centerOfMass, which is in the
    # component's own coordinate system.
    com_component_local: Vec3 = (0.0, 0.0, 0.0)
    
    # Center of mass — GLOBAL coordinates (meters)
    # Computed: component_local_com transformed to global via occurrence transform
    com_global: Vec3 = (0.0, 0.0, 0.0)
    
    # Inertia at COMPONENT ORIGIN (kg·m²) — from getXYZMomentsOfInertia()
    inertia_at_origin: InertiaTensor = field(default_factory=InertiaTensor)
    
    # Inertia at CoM (kg·m²) — computed via parallel axis theorem
    inertia_at_com: InertiaTensor = field(default_factory=InertiaTensor)
    
    # Bounding box (meters, component-local frame)
    bbox_size: Vec3 = (0.0, 0.0, 0.0)
    bbox_min: Vec3 = (0.0, 0.0, 0.0)
    bbox_max: Vec3 = (0.0, 0.0, 0.0)
    collision_bbox_size: Vec3 = (0.0, 0.0, 0.0)
    collision_bbox_min: Vec3 = (0.0, 0.0, 0.0)
    collision_bbox_max: Vec3 = (0.0, 0.0, 0.0)
    collision_volume_m3: float = 0.0
    
    # ── Material & Appearance ──
    material_name: str = ""             # "Nylon 101", "Steel", etc.
    appearance_name: str = ""           # Fusion appearance name
    appearance_color_rgb: Optional[Vec3] = None  # (r, g, b) 0.0–1.0
    
    # ── Raw reference for mesh export (not serialized) ──
    _fusion_occurrence: Any = field(repr=False, default=None)


@dataclass
class FusionJoint:
    """Everything extracted from one Fusion joint."""
    
    # ── Identity ──
    name: str = ""                  # "Azimuth", "zed2i_mount"
    joint_source: str = ""          # "regular" or "as_built"
    defining_component: str = ""    # Clean name of component that owns this joint
    defining_component_raw: str = ""  # Raw component name with version
    is_suppressed: bool = False
    
    # ── Connected occurrences ──
    # occurrenceOne = typically the child (moving part)
    # occurrenceTwo = typically the parent (grounded part)
    occurrence_one_path: str = ""   # fullPathName of occurrenceOne
    occurrence_two_path: str = ""   # fullPathName of occurrenceTwo
    occurrence_one_clean: str = ""  # clean component name
    occurrence_two_clean: str = ""  # clean component name
    
    # ── Geometry — all methods, raw from Fusion (cm) ──
    # We store everything and let Phase 2 decide what to use
    
    # geometry.origin (most reliable for regular joints)
    geometry_origin_cm: Optional[Vec3] = None
    
    # geometryOrOriginOne.origin
    geometry_or_origin_one_cm: Optional[Vec3] = None
    
    # geometryOrOriginTwo.origin
    geometry_or_origin_two_cm: Optional[Vec3] = None
    
    # occurrenceOne.transform (direct)
    occ_one_transform_cm: Optional[Vec3] = None
    
    # occurrenceOne.transform2
    occ_one_transform2_cm: Optional[Vec3] = None
    
    # Accumulated global via assemblyContext chain on occurrenceOne
    occ_one_global_cm: Optional[Vec3] = None
    occ_one_context_depth: int = 0
    
    # Same for occurrenceTwo
    occ_two_transform_cm: Optional[Vec3] = None
    occ_two_global_cm: Optional[Vec3] = None
    occ_two_context_depth: int = 0
    
    # ── Computed origin in GLOBAL coordinates (meters) ──
    # Phase 1 picks the best available method and converts to meters
    origin_global_m: Vec3 = (0.0, 0.0, 0.0)
    origin_source: str = ""         # Which method gave us the origin
    
    # ── Motion ──
    motion_type: str = ""           # "rigid", "revolute", "slider", "cylindrical", "pin_slot", "planar", "ball"
    motion_type_enum: Optional[int] = None  # Fusion JointTypes enum value
    
    # Axis (unit vector) — for revolute/prismatic
    axis_vector: Vec3 = (0.0, 0.0, 1.0)
    
    # ── Limits ──
    has_rotation_limits: bool = False
    rotation_min: Optional[float] = None    # radians
    rotation_max: Optional[float] = None
    
    has_slide_limits: bool = False
    slide_min_m: Optional[float] = None     # meters
    slide_max_m: Optional[float] = None

    # Convention-keyword flags derived from the Fusion joint name —
    # see ``utils.helpers.strip_joint_prefix``.  ``raw_name`` keeps
    # the original prefixed name for traceability; ``name`` holds the
    # cleaned form used downstream.
    raw_name: str = ""
    is_passive: bool = False
    is_closing: bool = False


@dataclass
class FusionSnapshot:
    """Complete extraction of a Fusion design. Serializable to JSON."""
    
    # Design info
    design_name: str = ""
    design_name_clean: str = ""
    root_component_name: str = ""
    export_timestamp: str = ""
    exporter_version: str = EXPORTER_VERSION
    
    # Document settings
    document_length_unit: str = "cm"  # "mm", "cm", "m", "in", "ft"
    
    # All occurrences (keyed by full_path)
    occurrences: Dict[str, FusionOccurrence] = field(default_factory=dict)
    
    # All joints (keyed by name)
    joints: Dict[str, FusionJoint] = field(default_factory=dict)
    
    # Rigid groups
    rigid_groups: List['RigidGroupInfo'] = field(default_factory=list)
    
    # Summary stats
    total_occurrences: int = 0
    total_subassemblies: int = 0
    total_leaf_components: int = 0
    total_joints: int = 0
    total_regular_joints: int = 0
    total_as_built_joints: int = 0
    max_nesting_depth: int = 0


# ──────────────────────────────────────────────
# Phase 2: Robot Model dataclasses
# ──────────────────────────────────────────────

@dataclass
class JointLimits:
    """Joint motion limits for URDF."""
    lower: float = 0.0          # rad or m
    upper: float = 0.0
    effort: float = 100.0       # N·m or N (default placeholder)
    velocity: float = 1.0       # rad/s or m/s (default placeholder)


@dataclass
class LinkNode:
    """A link in the robot model, ready for URDF generation."""
    
    # Naming
    urdf_name: str = ""             # Final name in URDF (collision-resolved)
    clean_name: str = ""            # Original clean_name from Fusion
    assembly: str = ""              # Which assembly this belongs to
    occurrence_path: str = ""       # Full Fusion occurrence path (for traceability)
    
    # Position
    global_position: Vec3 = (0.0, 0.0, 0.0)    # meters, world frame
    # Original link-local -> Fusion design-world rotation.  Kept on the
    # Phase-2 model so the pure-Python frame postprocessor can recover the
    # author's X-forward/Z-up design frame without calling the Fusion API.
    source_world_rotation: Optional[Tuple[float, ...]] = None
    
    # Physical properties
    mass_kg: float = 0.0
    com_link_local: Vec3 = (0.0, 0.0, 0.0)     # CoM relative to link origin
    inertia_at_com: InertiaTensor = field(default_factory=InertiaTensor)
    # Optional fully resolved CoM origin in the exported link frame.  The
    # frame postprocessor sets this after composing component CoM + joint
    # bake offset; None retains the legacy composition in the generator.
    inertial_origin_xyz: Optional[Vec3] = None
    
    # Material & appearance
    material_name: str = ""
    color_rgb: Optional[Vec3] = None
    
    # Geometry
    bbox_size: Vec3 = (0.0, 0.0, 0.0)
    bbox_min: Vec3 = (0.0, 0.0, 0.0)
    bbox_max: Vec3 = (0.0, 0.0, 0.0)
    volume_m3: float = 0.0
    area_m2: float = 0.0
    collision_bbox_size: Vec3 = (0.0, 0.0, 0.0)
    collision_bbox_min: Vec3 = (0.0, 0.0, 0.0)
    collision_bbox_max: Vec3 = (0.0, 0.0, 0.0)
    collision_volume_m3: float = 0.0
    
    # Mesh paths (relative, for URDF)
    mesh_visual: str = ""           # e.g. "meshes/turret/neck_link.obj"
    mesh_collision: str = ""        # e.g. "meshes/turret/neck_link.stl"
    # ``has_visual_mesh`` flips to False when the visual OBJ/DAE export
    # fails or yields zero bytes.  Drives whether ``xacro_generator``
    # emits a ``<visual>`` block: a link without one is still valid URDF
    # (kinematic role + inertials + collision still work).  Better than
    # the previous behaviour of silently emitting ``<mesh filename>``
    # pointing at a file that doesn't exist on disk.
    has_visual_mesh: bool = True

    # When True, ``collision_generator`` skips the bounding-primitive
    # auto-fit and uses the visual mesh as the collision shape instead.
    # Set by an ``!acc_*`` prefix on the link's source rigid-group name
    # (gripper fingers / tool tips / mating surfaces — anywhere the
    # primitive's accuracy isn't enough).  See DESIGN_RULES.md §1.4.
    wants_accurate_collision: bool = False
    collision_override: str = ""        # "visual", "convex_hull", "primitive", or "" from !acc/!cxh/!pri
    has_collision_exclusions: bool = False
    collision_body_count: int = 0
    
    # Collision info (resolved by Phase 3)
    has_explicit_collision: bool = False     # Fusion design has collision sub-component/body
    collision_body_names: List[str] = field(default_factory=list)  # Bodies prefixed "collision"
    collision_excluded_body_names: List[str] = field(default_factory=list)  # Visual bodies prefixed "!"
    collision: Optional['CollisionInfo'] = None  # Resolved collision geometry
    
    # For revolute/prismatic joint children: offset from joint frame to mesh origin
    # Mesh vertices need baking by this offset to avoid wobble during rotation
    mesh_bake_offset: Vec3 = (0.0, 0.0, 0.0)
    needs_mesh_bake: bool = False
    # Rotation from the exported link frame to the unchanged mesh frame.
    # Normally identity.  Frame rebasing changes this origin instead of
    # rewriting mesh vertices, which keeps DAE/OBJ/STL files reusable.
    mesh_origin_rpy: RPY = (0.0, 0.0, 0.0)

    # Traceability for the post-export frame layer.  ``frame_rebase_rpy`` is
    # the pose of the post frame in the original link frame (orientation
    # only in v1); ``frame_rule`` records auto/keep/world_rpy.
    frame_rebase_rpy: RPY = (0.0, 0.0, 0.0)
    frame_rule: str = "keep"
    
    # Empty link (reference frame — no bodies, no visual/collision)
    body_count: int = 0
    is_empty: bool = False
    # Explicit !frame_* link: a named TF/attachment frame only.  Unlike an
    # accidental empty component, it emits no visual, collision, or inertial.
    is_frame_only: bool = False

    # Rigid group merge: when this link is the anchor of a merged rigid group,
    # is_merged=True and merged_member_paths lists every leaf occurrence whose
    # mass/visual/collision was folded into this link.  Non-anchor members of
    # a rigid group are NOT present in RobotModel.links — they were collapsed
    # away in _process_rigid_groups.
    is_merged: bool = False
    merged_member_paths: List[str] = field(default_factory=list)
    rigid_group_name: Optional[str] = None
    # Priority-1 collision for the merged group: occurrence path of the
    # explicit !collision_* member (if any) and its offset in anchor-local
    # coordinates.  Falls through to priority 2 (auto primitive) / 3
    # (visual reuse) when None.
    rigid_group_collision_path: Optional[str] = None
    rigid_group_collision_offset: Optional[Vec3] = None
    mesh_collision_input: str = ""      # Filtered OBJ used only for generated collision


@dataclass
class JointNode:
    """A joint in the robot model, ready for URDF generation."""
    
    # Naming
    name: str = ""
    
    # URDF joint type: "fixed", "revolute", "prismatic", "continuous"
    joint_type: str = "fixed"
    
    # Connected links (URDF names)
    parent_link: str = ""
    child_link: str = ""
    
    # Origin: transform from parent link frame to joint/child frame
    origin_xyz: Vec3 = (0.0, 0.0, 0.0)     # meters
    origin_rpy: RPY = (0.0, 0.0, 0.0)      # radians
    
    # Axis of rotation/translation (unit vector in child frame)
    axis: Vec3 = (0.0, 0.0, 1.0)
    
    # Limits (for revolute/prismatic)
    limits: Optional[JointLimits] = None
    
    # Metadata
    is_mount: bool = False          # Crosses assembly boundary
    fusion_source: str = ""         # "as_built" or "regular"
    origin_method: str = ""         # How origin was determined

    # Global position of joint axis (for visualization/debugging)
    origin_global: Vec3 = (0.0, 0.0, 0.0)

    # Convention-keyword flags (from Fusion joint name prefixes —
    # see ``utils.helpers.strip_joint_prefix``).  ``is_passive``
    # tells downstream pipelines to apply no controller / no
    # ``UsdPhysics.DriveAPI``; ``is_closing`` means the joint was
    # excluded from the URDF tree and emitted to the
    # ``closing_joints:`` sidecar instead.  ``closing_source``
    # records how it was identified: ``"user_tag"`` if the user
    # named the joint ``!closing_*``, ``"auto_detected"`` if the
    # plugin classified it as the loop closer for a multi-parent
    # link without an explicit tag.
    is_passive: bool = False
    is_closing: bool = False
    closing_source: str = ""        # "user_tag" | "auto_detected" | ""


@dataclass
class AssemblyInfo:
    """Metadata about a sub-assembly (for mesh organization and future xacro splitting)."""
    name: str = ""
    parent_assembly: Optional[str] = None
    children_assemblies: List[str] = field(default_factory=list)
    global_offset: Vec3 = (0.0, 0.0, 0.0)
    # World-frame rotation of this sub-assembly (row-major 3×3, identity
    # default).  Used by ``_compute_joint_global_origin`` and the
    # axis-vector resolution to lift joint origins / axes from the
    # defining-component's frame into world frame — Fusion stores both
    # in the defining component's local frame, not in world, despite
    # the API docstring claiming otherwise.  Without rotating, joints
    # inside a rotated sub-asm end up with wrong origins and axes
    # (visible as gripper-on-arm articulating around the wrong axis).
    global_rotation: Tuple[float, ...] = (1.0, 0.0, 0.0,
                                            0.0, 1.0, 0.0,
                                            0.0, 0.0, 1.0)
    links: List[str] = field(default_factory=list)      # URDF names
    joints: List[str] = field(default_factory=list)      # Joint names
    depth: int = 0


@dataclass
class RobotModel:
    """Complete robot model built from FusionSnapshot. Input to Phase 3 generators."""
    
    # Design info
    name: str = ""
    
    # The kinematic tree
    links: Dict[str, LinkNode] = field(default_factory=dict)        # urdf_name → LinkNode
    joints: Dict[str, JointNode] = field(default_factory=dict)      # name → JointNode

    # Closing-loop joints — those that would form a cycle in the
    # URDF tree.  Excluded from ``joints`` (so URDF stays a tree),
    # emitted to the ``closing_joints:`` section of robot_data.yaml
    # so a downstream USD authoring stage can re-create the loop
    # via independent ``UsdPhysicsJoint`` prims.  Identified by user
    # tag (joint name prefix ``!closing_``) or by auto-detection when
    # a multi-parent link has no explicit tag.
    closing_joints: Dict[str, JointNode] = field(default_factory=dict)

    # Root
    root_link: str = ""             # URDF name of root link

    # Assembly metadata
    assemblies: Dict[str, AssemblyInfo] = field(default_factory=dict)

    # Build log
    warnings: List[str] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)


# ──────────────────────────────────────────────
# Phase 3: Generation dataclasses
# ──────────────────────────────────────────────

@dataclass
class CollisionPrimitive:
    """Auto-generated collision geometry from bounding box analysis."""
    shape: str = "box"              # "box", "cylinder", "sphere"
    size: Vec3 = (0.0, 0.0, 0.0)   # box: (x,y,z), cylinder: (radius,0,length), sphere: (radius,0,0)
    origin_xyz: Vec3 = (0.0, 0.0, 0.0)   # offset from link origin
    origin_rpy: RPY = (0.0, 0.0, 0.0)    # rotation (e.g. cylinder along non-Z axis)
    orientation_matrix: Optional[Tuple[float, ...]] = None  # row-major 3x3, used for generated STL
    
    def urdf_geometry(self) -> str:
        """Return URDF <geometry> inner XML."""
        if self.shape == "box":
            return f'<box size="{self.size[0]:.6f} {self.size[1]:.6f} {self.size[2]:.6f}"/>'
        elif self.shape == "cylinder":
            return f'<cylinder radius="{self.size[0]:.6f}" length="{self.size[2]:.6f}"/>'
        elif self.shape == "sphere":
            return f'<sphere radius="{self.size[0]:.6f}"/>'
        return f'<box size="{self.size[0]:.6f} {self.size[1]:.6f} {self.size[2]:.6f}"/>'


@dataclass
class CollisionInfo:
    """Collision geometry resolved for one link."""
    source: str = ""                # "explicit", "primitive", "convex_hull", "visual_reuse", "visual_fallback"
    mesh_path: str = ""             # STL/mesh file once resolved or generated
    primitive: Optional[CollisionPrimitive] = None  # For auto-generated primitives
    origin_xyz: Optional[Vec3] = None  # Override origin (for rigid group collision offset)
    origin_rpy: RPY = (0.0, 0.0, 0.0)  # Mesh pose after optional frame rebasing
    
    @property
    def uses_primitive(self) -> bool:
        return self.source == "primitive" and self.primitive is not None


@dataclass
class RigidGroupInfo:
    """A rigid group extracted from Fusion 360.

    A rigid group means: every member moves as one body.  In URDF terms it
    becomes ONE link.  Phase 2 picks an anchor (explicit ``!frame_*``
    member first, otherwise heaviest non-collision member unless an
    implicit merge specifies its natural assembly anchor), aggregates
    mass/inertia from physical members, and collapses every non-anchor
    member out of the link set.  Container-only subassemblies expand to
    physical descendants in Phase 1; subassemblies with direct bodies can
    be members themselves.
    """
    name: str = ""

    # Physical members after Phase 1 sub-assembly expansion.  Container
    # subassemblies expand to descendants; body-owning subassemblies can
    # appear here alongside leaf occurrences.  The lists are parallel.
    occurrence_paths: List[str] = field(default_factory=list)
    member_clean_names: List[str] = field(default_factory=list)

    # Optional anchor override used by auto rigid-island collapse.
    preferred_anchor_path: str = ""
    is_auto: bool = False

    # Priority-1 collision: the single ``!collision_*`` member, if the user
    # placed one inside the group.
    collision_member: Optional[str] = None
    collision_path: Optional[str] = None

    # Filled by Phase 2 (_process_rigid_groups).  ``anchor_path`` is the
    # leaf chosen as the merged link's frame; ``merged_link_name`` is what
    # the URDF link will be called (rigid-group name when user-renamed,
    # else anchor's clean_name).
    anchor_path: str = ""
    anchor_clean_name: str = ""
    merged_link_name: str = ""

    # ``!acc_*`` prefix on the Fusion rigid group's name flips this flag.
    # When True, the merged link's collision geometry is the visual
    # mesh itself rather than the auto-fitted bounding primitive — for
    # parts that need accurate contact (gripper jaws with grooves,
    # tool tips, mating surfaces) while the rest of the robot stays
    # cheap with primitive collision.  Phase 2 propagates this onto
    # ``LinkNode.wants_accurate_collision``.
    wants_accurate_collision: bool = False
    collision_override: str = ""        # Per-link collision override propagated to the merged LinkNode


@dataclass
class ExportConfig:
    """Configuration for Phase 3 export."""
    
    # Package info
    package_name: str = ""          # ROS2 package name (default: <robot>_description)
    
    # Collision strategy
    # "primitive", "convex_hull", "visual_reuse", or "visual_fallback"
    collision_auto_method: str = "primitive"
    override_explicit_collision: bool = False  # If True, use visual mesh even for links with explicit collision
    
    # Mesh export
    export_visual_mesh: bool = True
    export_collision_mesh: bool = True
    # "obj" — Fusion native OBJ+MTL pair, centimeters, URDF uses scale="0.01"
    # "dae" — convert to a single self-contained COLLADA file, meters, URDF uses scale="1"
    visual_format: str = "obj"
    collision_format: str = "stl"
    mesh_refinement: str = "medium" # Fusion mesh quality: "low", "medium", "high"
    
    # Output paths
    output_dir: str = ""            # Where to write the ROS2 package

    # URDF options
    use_xacro: bool = False         # Phase 3.1: plain URDF. Future: xacro
    robot_prefix: str = ""          # For multi-robot (empty = no prefix)

    # Post-export frame convention.  ``ros`` aligns the root with Fusion's
    # design world (X forward, Z up) and makes every revolute/continuous
    # joint axis local +Z without changing mesh vertices.  ``fusion`` keeps
    # extracted link frames unless a CSV row explicitly overrides one.
    frame_convention: str = "ros"
    frame_overrides_filename: str = "frame_overrides.csv"

    # Optional output toggles — controllable via xacro_export.toml.
    # ``verbosity`` is a preset that sets the include_* fields below,
    # but individual fields override the preset.
    verbosity: str = "verbose"           # "verbose" or "minimal"
    include_debug: bool = True           # debug/ folder (snapshot.json, transforms, log)
    include_docs: bool = True            # docs/transforms.md
    include_robot_data_yaml: bool = True # robot_data.yaml
    include_screenshot: bool = True      # images/robot.png
    include_launch: bool = True          # launch/display.launch.py
    include_rviz: bool = True            # rviz/display.rviz + config/joint_state.yaml
    include_readme: bool = True          # auto-generated README.md
    include_ros2_control: bool = True    # ros2_control XML + controller YAML

    # ROS 2 control defaults.  The generic mock hardware gives exported
    # packages an immediately runnable controller_manager; real robots can
    # replace this plugin and keep the generated joint/interface scaffold.
    ros2_control_hardware_plugin: str = "mock_components/GenericSystem"
    ros2_control_update_rate: int = 100
    ros2_control_command_interfaces: Tuple[str, ...] = ("position", "velocity")

    # ZIP packaging — when zip_output is True, the export wraps the
    # generated package directory in a .zip and removes the directory.
    zip_output: bool = False
    zip_name: str = ""               # default: <package_name>.zip
