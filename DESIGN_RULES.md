# Design Rules for Fusion -> URDF/Xacro Export

> Living reference for Fusion designs exported through this plugin.
> Most items are recommendations. Items marked **must** are hard
> requirements because violating them can produce a broken or unloadable
> URDF.

For modelling advice and preferred workflows, see
[DESIGN_GUIDE.md](DESIGN_GUIDE.md). This file is the compact rule
reference: what the exporter expects, what names mean, and what edge
cases are supported.

## TL;DR

1. **Use ROS frame orientation from the start:** `Z` up, `X` forward,
   `Y` left. Fusion's view cube may not match this convention.
2. **Keep exportable geometry visible.** Hidden Fusion bodies and
   occurrences are not exported as visual mesh. Hiding `!frame_*` helper
   bodies is okay; those bodies are removed from exported visual,
   collision, mass, and inertia data.
3. **Use a Fusion `Rigid Group` for every set of components that should
   become one URDF link.**
4. **A subassembly file does not automatically mean one URDF link.**
   Use `Rigid Group` for multi-component links. Subassembly-as-link via
   as-built joint is supported, but not the preferred pattern.
5. **Do not put joints inside a `Rigid Group`.** A rigid group is one
   rigid body.
6. **URDF output must be a tree.** Every real non-root link needs one
   parent path to the root. Closed loops are supported by keeping one
   valid URDF tree and writing loop-closing joints to `robot_data.yaml`;
   tag intended loop joints with `!closing_*`.
7. **Fusion joint direction matters.** `occurrenceOne` is treated as the
   child or moving side; `occurrenceTwo` is treated as the parent or
   static side. Flipped joints can invert the tree or orphan links.
8. **Loose root bodies are supported.** CAD with chassis bodies
   directly under the design root can export as synthetic `base_link`.
9. **Connect every real non-root link to the kinematic tree.** Or put it
   in a connected `Rigid Group`.
10. **Use reserved prefixes intentionally.** Prefer the explicit `!`
   marker for exporter metadata: `!collision_*`, `!acc_*`, `!cxh_*`,
   `!pri_*`, `!frame_*`, `!dummy_*`, `!passive_*`, and `!closing_*`.
   Bare names without `!` are ordinary names. See
   [Reserved Prefixes](#14-reserved-prefixes).
11. **Use `!frame_*` helpers when the CAD origin is not the URDF frame
   you want.** Standalone helpers export as frame-only links; helpers
   inside rigid groups become the merged link frame. They are optional
   when the component origin and joint axis are already correct.
12. **Generated ros2_control is optional but enabled by default.**
   Movable non-passive joints get command interfaces; movable passive
   joints get state interfaces only.
13. **Name deliberately.** Fusion names flow into URDF links and joints,
   xacro macros, mesh folders, and `robot_data.yaml`. Keep assembly
   names case-unique.

## Contents

- [1. Naming](#1-naming)
- [2. Collision Geometry](#2-collision-geometry)
- [3. Assembly and Kinematics](#3-assembly-and-kinematics)
- [4. Physical Properties and Frames](#4-physical-properties-and-frames)
- [5. Special Link Types](#5-special-link-types)
- [6. Exported Files](#6-exported-files)
- [7. Checklist](#7-checklist)
- [8. Gotchas](#8-gotchas)
- [9. Compact Modelling Patterns](#9-compact-modelling-patterns)

## 1. Naming

The exporter cleans names by stripping Fusion version suffixes such as
`v3:1`, replacing unsafe characters, and deduplicating where needed. The
base names you choose still matter.

Avoid spaces, unnecessary special characters, generic Fusion names, and
temporary names like `Component5`, `Rigid Group 7`, `Rev 3`, `final`,
`test`, or `main`.

### 1.1 Link Names

| Fusion structure | Exported link name comes from | Notes |
| --- | --- | --- |
| Single-component link | Component name | Normal simple-link case |
| Multi-component `Rigid Group` | Rigid group name | Preferred multi-component link pattern |
| Subassembly connected as one link via as-built joint | Subassembly name | Supported, but mainly for compatibility |
| Design-root bodies | `base_link` | Synthetic root link for CAD |

Rules:

- Use descriptive names such as `neck_link`, `fl_wheel_link`, or
  `sensor_housing`.
- Rename rigid groups; otherwise the exporter may fall back to an anchor
  component name.
- Prefer unique link names across the design. Duplicate names are
  auto-deduplicated, but the result may not be what you intended.
- Use `base_link` for the root chassis when modelling cleanly from
  scratch.

#### 1.1.1 Duplicate Link Names

If multiple links resolve to the same URDF name, the exporter prefixes or
deduplicates them to keep the URDF valid. The true root keeps
`base_link` when applicable.

Deduplication is a safety net, not a naming strategy.

#### 1.1.2 Imported Root-Body Designs

Downloaded CAD often keeps the chassis as loose bodies directly under
the Fusion design root, with legs or tools jointed to that root. This is
supported.

When a Fusion joint references the design root, the exporter creates a
synthetic root link:

- URDF name: `base_link`
- Mass, center of mass, inertia, material, color, and bounding box come
  from root-owned bodies.
- Visual mesh is exported from root-owned bodies only, so child
  occurrences are not duplicated into the base mesh.
- Collision follows the normal priority chain.

For new clean designs, a real `base_link` component is still easier to
inspect, group, and maintain.

### 1.2 Joint Names

Joint names are exported directly after cleanup. Use names like
`azimuth_joint`, `elevation_joint`, or `fl_wheel_joint`.

Joint direction matters:

- `occurrenceOne` is treated as the child or moving side.
- `occurrenceTwo` is treated as the parent or static side.

Flipped joints can produce unreachable links, an unexpected root, or an
inverted TF tree. A fast `verbosity = "minimal"` export surfaces these
issues in the warning dialog before you spend mesh-export time.

### 1.3 Assembly Names

Assembly names are used for mesh folders, xacro macro names, metadata in
`robot_data.yaml`, and some name-resolution paths. Keep them clean and
descriptive, for example `turret`, `left_leg`, or `camera_mount`.

Assemblies that own no exported links or joints are treated as structure
only and do not get xacro macros or mesh folders. Avoid case-only pairs
such as `Panther` and `panther`; when both are real assemblies, the
exporter gives their files unique stems, but case-unique names are easier
to debug and safer across filesystems.

### 1.4 Reserved Prefixes

Reserved keywords describe exporter intent, not physical part names.
For new CAD, prefer the tagged form `!keyword_name`; Fusion allows `!`,
and the marker makes it clear that the name contains exporter metadata.
All reserved keywords are parsed by the same metadata dispatcher, so the
rule is consistent: `!<keyword>_<name>` marks exporter behavior and the
keyword is stripped from the exported URDF name where applicable.
Do not use tagged keywords for ordinary visual geometry or ordinary
joints.

| Prefix | Where | Meaning | Export behavior |
| --- | --- | --- | --- |
| `!collision_*` | Component or body | Explicit collision geometry | Excluded from visuals and exported as collision STL |
| `!acc_*` | Link component or rigid group | Accurate collision override | Uses the visual mesh as collision for that link; keyword is stripped |
| `!cxh_*` | Link component or rigid group | Convex-hull collision override | Uses generated convex hull for this link even if the global method differs |
| `!pri_*` | Link component or rigid group | Primitive collision override | Uses generated primitive collision for this link even if the global method differs |
| `!frame_*` | Component | Frame-only attachment link or rigid-group frame anchor | Standalone: emits a URDF link with no visual, collision, mass, or inertia. In a rigid group: defines the merged link frame and is ignored for geometry and physical properties. Keyword is stripped |
| `!dummy_*` | Assembly | Placeholder subsystem | Included in URDF, excluded from `robot_data.yaml` |
| `!passive_*` | Joint | Non-driven joint | Kept in URDF, keyword stripped, `passive: true` in `robot_data.yaml` |
| `!closing_*` | Joint | Closed-loop joint | Excluded from URDF tree, emitted to `closing_joints:`; implies passive |
| `!<body_name>` | Body only | Visual-only collision exclusion | Keeps the body in visual mesh, but removes it from generated collision input. Reserved body tags such as `!collision_*` keep their normal meaning |

Bare forms without `!` are not metadata. For example, `collision_proxy`
is a normal component name; use `!collision_proxy` when you mean
explicit collision geometry.

If a closed loop is not explicitly tagged, the exporter auto-detects
extra parent joints and moves them to `robot_data.yaml` with
`source: auto_detected`. Prefer explicit `!closing_*` tags when you know
which joint should close the loop.

### 1.5 Visibility

Fusion visibility controls mesh export. Hidden bodies and hidden
occurrences are omitted from visual meshes, and collision generated from
visual geometry will omit them too.

Use hidden geometry for construction bodies or design alternatives that
should not appear in the robot description. If the body still exists in
Fusion, verify mass and inertia separately; visibility is not a complete
physical-property exclusion rule.

## 2. Collision Geometry

Visual meshes are often too detailed for physics. Collision geometry
should approximate the physical contact envelope while staying simple,
convex where possible, and cheap to simulate.

### 2.1 Purpose

Use collision geometry to describe how the robot should contact the
world, not how it should look. Avoid small cosmetic details, holes, and
overly dense exact meshes unless you really need accurate contact.

### 2.2 Collision Priority Chain

The exporter resolves collision in this order:

| Priority | Source | Behavior |
| --- | --- | --- |
| 1a | `!collision_*` component or body inside a link component | Exported as `<link>_collision.stl` |
| 1b | `!collision_*` member inside a `Rigid Group` | Used as merged-link collision with transform offset preserved |
| 1c | Flattened `!collision_*` sibling | Supported for compact two-child collision subassemblies |
| 2 | Per-link override | `!pri_*`, `!cxh_*`, or `!acc_*` chooses primitive, convex hull, or exact visual collision for one link |
| 3 | Auto-generated primitive STL | Box, oriented box, cylinder, or sphere fitted from visual geometry |
| 3 alt | Generated convex hull STL | Config-selected convex mesh from visual OBJ vertices |
| 4 | Visual mesh fallback | Exact but heavier |

The export dialog lets you choose primitive collision or visual reuse,
and optionally keep or override explicit collision geometry. Set
`[mesh].collision_method = "convex_hull"` in `xacro_export.toml` when a
generated convex mesh is a better default for imported or angled CAD.

Body-level `!` names remove visual detail from generated collision
without hiding it from the visual mesh. For example, a body named
`!antenna` remains visible but is ignored when fitting primitives,
building convex hulls, or creating filtered exact collision input. If
all bodies on a link are excluded this way, the link gets no collision
element.

### 2.3 Rigid Groups -> One Merged URDF Link

A Fusion `Rigid Group` means "these components are one rigid body." The
exporter collapses the group into one URDF link with:

- one link name
- one merged visual mesh
- one collision source
- summed mass
- mass-weighted center of mass
- parallel-axis-aggregated inertia
- redirected external joints
- group membership in `robot_data.yaml`

There is no opt-out. If components should move relative to each other,
they must not be in the same rigid group.

**Must:** no joints inside a `Rigid Group`.

Internal rigid-group joints are dropped. Redundant `rigid` joints are
quietly ignored; this is expected for Fusion fasteners placed inside the
same rigid group as their mounted part. Non-rigid internal joints still
warn because their intended motion cannot exist inside one URDF link.
Move those joints between links instead.

Merge behavior in brief:

- A `!frame_*` member becomes the anchor when present. If several frame
  members exist, the exporter prefers one whose stripped name matches
  the rigid group or exported link name.
- Without a `!frame_*` member, the heaviest non-collision member becomes
  the anchor.
- The rigid group name becomes the link name when meaningful.
- Subassembly members are recursively expanded.
- Fusion's OBJ export path is used for merged visual geometry.
- Non-members, collision members, and frame helper bodies are hidden
  during visual export.
- Frame helper bodies do not contribute collision, mass, inertia, color,
  material, volume, or bounding-box data.
- Explicit collision members provide collision; otherwise the global
  collision chain applies.
- A single-member group behaves like a normal leaf link, but can still
  carry explicit collision.

### 2.4 Collision Patterns

| Pattern | Supported | Recommendation |
| --- | --- | --- |
| Single component with `!collision_*` body | Yes | Works, but body may affect Fusion mass/inertia |
| Single component with `!collision_*` subcomponent | Yes | Prefer this over collision body when possible |
| Rigid group with visual members and `!collision_*` member | Yes | Recommended multi-component collision pattern |
| Flattened visual/`!collision_*` siblings | Yes | Supported; prefer rigid group for new multi-component links |
| Body named `!antenna` or similar | Yes | Keeps visual detail while excluding it from generated collision |
| `!acc_*` link for exact collision | Yes | Use sparingly for grippers, tool tips, mating surfaces |
| `!cxh_*` link for convex hull | Yes | Useful for one imported/angled link without changing the global setting |
| `!pri_*` link for primitive collision | Yes | Useful when global convex/visual collision is too heavy for one link |
| Visual geometry named `collision*` by accident | Yes | Treated as ordinary visual geometry because it lacks `!` |

## 3. Assembly and Kinematics

### 3.1 Kinematic Tree

The URDF output must be a tree:

- exactly one root
- no cycles in the URDF tree
- no multi-parent links in the URDF tree
- every non-root link has a parent joint

Every non-root, non-empty component must either be connected by a Fusion
joint or be a member of a connected `Rigid Group`.

Exceptions:

- Root-owned bodies can be folded into the synthetic `base_link`.
- Zero-body helper components from CAD may be dropped or
  reported as non-fatal warnings when unreferenced.

Real orphan links abort the export because a URDF link without a parent
joint has no defined pose.

### 3.1.1 Closed Kinematic Chains

URDF cannot represent closed loops directly. The exporter splits them:

1. A valid URDF tree is written to `<robot>.urdf` and xacro.
2. Closing joints are written to `closing_joints:` in `robot_data.yaml`.

When a child link has multiple parent joints:

- one joint stays in the URDF tree
- `!closing_*` joints go to the sidecar
- untagged extra parent joints are auto-detected and moved to the
  sidecar
- user-tagged joints get `source: user_tag`
- auto-detected joints get `source: auto_detected`

Closing joints are implicitly passive. Downstream tools can restore them
as physics constraints, for example in Isaac Sim or a custom pipeline.

Splitting non-leaf loops is still a riskier path; inspect the generated
tree and `robot_data.yaml`.

### 3.2 Joint Direction

Symptoms of flipped or incorrect joints:

- disconnected root warnings
- unreachable links
- unexpected root link
- child fixed to the wrong parent
- reversed or odd TF tree

Inspect a previous export's snapshot:

```bash
python -m fusion2URDF.tools.check debug/snapshot.json
```

### 3.3 Cross-Assembly Joints

Joints connecting components in different assemblies are supported. They
become mount joints in the top-level xacro and define connection points
between reusable assemblies.

### 3.4 As-Built vs Regular Joints

Use regular Fusion joints when the joint origin, axis, or position
matters:

- revolute mechanisms
- prismatic mechanisms
- off-center axes
- deliberate sensor or tool frames

Use as-built joints only for simple fixed connections, cases where exact
origin does not matter, or supported subassembly-as-link cases.

When in doubt: regular joint for mechanisms, `Rigid Group` for parts
that should become one rigid body.

## 4. Physical Properties and Frames

### 4.1 Materials and Mass

Assign materials to physical leaf components. The exporter reads mass,
center of mass, inertia tensor, material name, appearance color, bounding
box, and volume where available.

Missing or incorrect physical properties can produce zero mass, invalid
inertia, or unstable simulation behavior.

### 4.2 Component Origins

For single-component links, the component origin defines the URDF link
frame. For rigid-group links, a `!frame_*` member defines the merged
link frame when present; otherwise the chosen anchor component defines
it.

Arbitrary origins are supported. The exporter applies bake offsets so
visual and collision geometry still land in the correct pose.

Good origins are still useful for debugging and downstream tools. Prefer
joint axes, mounting faces, centers of rotation, sensor frames, tool
center points, or geometric centers when practical.

For movable child links, the selected link frame is also the joint frame
used by the generated URDF. This is why wheel or hinge links should use a
frame helper at the real rotation center when the mesh origin is not
already there.

Frame helpers are not a requirement for clean CAD. If the component
origin, joint origin, and axis orientation are already correct, a normal
Fusion joint is preferable. Use `!frame_*` when Fusion makes the desired
coordinate system difficult to author directly, or when the robot needs
real reference links such as IMU frames, optical frames, tool points, or
DH-style frames.

Bake offsets appear in:

- `<visual><origin>`
- `<collision><origin>`
- `bake_offset_m` in `robot_data.yaml`

### 4.3 Post-Export Orientation Frames

The default `[frames].convention = "ros"` applies an orientation-only frame
layer after the canonical Fusion model has been built:

- the URDF root follows the Fusion design world, with `X` forward and `Z` up
- every revolute or continuous joint rotates about its child link's local `+Z`
- visual meshes, collision geometry, inertial data, and physical joint motion
  keep the same world poses

Verbose exports write `config/frame_overrides.csv`. The `original_*` columns
describe the extracted frames. Set a non-root link to `rule=world_rpy`, edit
its `post_*_deg` columns, and reapply the frame layer without Fusion or mesh
export:

```powershell
python tools/reframe.py <robot>_description
```

The offline command reads the canonical `debug/frame_model.json` cache. It
rewrites only frame-dependent URDF, xacro, YAML, config, and documentation
files. For a package exported before this cache existed, bootstrap it once
from a matching snapshot:

```powershell
python tools/reframe.py <robot>_description --snapshot debug/snapshot.json
```

CSV overrides intentionally change orientation only. Moving a revolute frame
away from its physical axis would change the motion unless an extra virtual
frame were introduced. Use a Fusion `!frame_*` helper and export again when
the frame position or joint origin must change.

## 5. Special Link Types

### 5.1 Empty Links and Reference Frames

Zero-body components, and components tagged `!frame_*`, can become
minimal URDF links without visual or collision geometry. Use them for
mounting frames, sensor origins, camera optical frames, tool frames, and
kinematic reference points.

A `!frame_*` component inside a `Rigid Group` does not create a separate
URDF link. It defines the merged link frame for that group instead.

Unreferenced zero-body helper components from CAD may be ignored
or reported as non-fatal warnings.

### 5.2 Dummy Assemblies

Assemblies tagged with `!dummy_` are placeholders intended to be swapped
later.

Behavior:

- included in URDF and visual output
- excluded from `robot_data.yaml`
- documented as swappable in generated robot docs

Use for placeholder sensors, cameras, optional tools, or example modules.

## 6. Exported Files

Meshes are written under `meshes/<assembly>/`.

The assembly folder is usually:

- the anchor's parent assembly for merged rigid-group links
- the component's parent assembly for single-component links
- the root assembly folder for imported root-owned bodies

Visual mesh notes:

- DAE is the default and is written in meters.
- OBJ+MTL is available by config and uses Fusion's centimeter units with
  URDF scale correction.

Collision mesh notes:

- STL comes from explicit collision, generated primitives, or visual
  fallback.
- Fusion may export STL in document display units.
- The exporter normalizes STL scale and applies xacro scale where needed.

Main outputs:

| File | Purpose |
| --- | --- |
| `<robot>.urdf` | Preprocessed flat URDF |
| `<robot>.urdf.xacro` | Main xacro entry point |
| `urdf/assemblies/*.urdf.xacro` | Per-assembly macros |
| `meshes/<assembly>/` | Visual and collision meshes |
| `config/ros2_controllers.yaml` | Generated ros2_control controller configuration when enabled |
| `config/frame_overrides.csv` | Editable post-export link orientation rules in verbose exports |
| `config/FRAME_OVERRIDES.md` | Short guide for the frame override CSV |
| `robot_data.yaml` | Data URDF cannot represent |
| `docs/transforms.md` | Homogeneous transform chains |
| `images/robot.png` | Fusion viewport screenshot |
| `debug/snapshot.json` | Raw extraction snapshot |
| `debug/frame_model.json` | Canonical model cache for offline frame regeneration |
| `debug/export_log.md` | Export log |
| `debug/validation.md` | Validation summary |

`robot_data.yaml` includes assembly hierarchy, rigid-group membership,
merged-from lists, materials, colors, bounding boxes, bake offsets,
passive joints, closing joints, and downstream metadata.

### 6.1 Generated ros2_control

When `[features].include_ros2_control = true`, the exporter emits a
generic ros2_control system if the model contains movable URDF joints.
`revolute`, `continuous`, and `prismatic` joints are included; fixed and
closing joints are not.

Rules:

- Every included movable joint gets `position`, `velocity`, and `effort`
  state interfaces.
- Non-passive movable joints get the configured command interfaces and
  generated forward command controllers.
- `!passive_*` joints are state-only.
- Hierarchical xacro writes control blocks beside the joints they own:
  inside an assembly macro for internal joints, or in the top-level xacro
  for cross-assembly movable joints.
- The default hardware plugin is `mock_components/GenericSystem`; real
  robots should replace it in `xacro_export.toml` or disable the feature
  and provide their own control description.
- State is published on `/joint_states`; command arrays are received on
  `/<controller_name>/commands`, for example
  `/position_controller/commands`.

## 7. Checklist

### Must

- [ ] Every non-root, non-empty component is connected by a Fusion joint
  or is in a connected `Rigid Group`.
- [ ] No joints exist inside a `Rigid Group`.
- [ ] No visual body or component is tagged `!collision_*` unless it is
  intentional collision geometry.
- [ ] The model has one valid root.
- [ ] Closed-loop joints are tagged with `!closing_*`, or you
  accept exporter auto-detection.

### Should

- [ ] Components, rigid groups, joints, and assemblies have descriptive
  names.
- [ ] Joint direction is correct.
- [ ] Materials are assigned to physical components.
- [ ] Multi-component links are defined with `Rigid Group`.
- [ ] Rigid groups that need an explicit link or joint frame contain one
  intentional `!frame_*` member.
- [ ] Collision geometry is explicit or suitable for auto-generation.
- [ ] Imported root-owned bodies are inspected after export.
- [ ] Reserved metadata tags such as `!dummy_*`, `!passive_*`,
  `!closing_*`, `!frame_*`, `!acc_*`, `!cxh_*`, and `!pri_*` are used
  only intentionally.
- [ ] Unpowered movable joints are tagged `!passive_*` before enabling
  generated ros2_control.

Validate a previous export's snapshot with:

```bash
python -m fusion2URDF.tools.check debug/snapshot.json
```

After export, inspect `debug/export_log.md` and `debug/validation.md`
when a rigid-group merge, collision mesh, or kinematic tree looks wrong.

## 8. Gotchas

### Things That Still Bite

| Issue | Meaning | Fix |
| --- | --- | --- |
| Non-rigid joint inside `Rigid Group` | One rigid body cannot contain internal URDF motion | Move the joint between links |
| Orphan non-empty component | Link has no defined parent in URDF | Add a joint or put it in a connected `Rigid Group` |
| Visual geometry tagged `!collision_*` | It becomes collision and disappears from visuals | Rename the visual geometry |
| Collision body inside regular component | May affect Fusion mass and inertia | Prefer a collision subcomponent |
| As-built joint with off-center axis | Origin may not match intended mechanism frame | Use a regular joint |
| Rigid-group link frame is wrong | Anchor origin is not the intended URDF frame | Add one matching `!frame_*` member to the group |
| Generated controller appears for an unpowered joint | Joint was not tagged passive | Rename the Fusion joint to `!passive_<name>` |
| Generic rigid-group name | Link name may be generic or fallback | Rename the rigid group |
| Duplicate link names | Exporter deduplicates names | Name links uniquely |
| Untagged closed loop | Exporter chooses which joint leaves the tree | Tag intended closing joint with `!closing_*` |

## 9. Compact Modelling Patterns

| Goal | Fusion pattern | Export result |
| --- | --- | --- |
| One simple link | One component with bodies | One URDF link |
| One link with custom collision | Component plus `!collision_*` subcomponent/body | Visual mesh plus collision STL |
| Multi-component link | `Rigid Group "sensor_link"` with visual members | One merged URDF link |
| Multi-component link with custom collision | Same group plus `!collision_sensor` member | Merged visual, explicit collision STL |
| Multi-component link with explicit frame | Same group plus one `!frame_sensor_link` member | Frame helper defines link and joint frame; physical members define mesh and mass |
| Placeholder subsystem | `!dummy_camera` assembly | URDF-visible, excluded from `robot_data.yaml` |
| Closed loop | Tag loop-closing joint `!closing_*` | Valid URDF tree plus sidecar joint |
| Passive mechanism | Tag joint `!passive_*` | Joint stays in URDF with `passive: true` metadata |
| Out-of-box ROS 2 control | Leave `[features].include_ros2_control = true` | ros2_control XML, controller YAML, joint states, and command topics |
| Accurate contact on one link | Tag link or group `!acc_*` | Visual mesh reused as collision for that link |
| Convex collision on one link | Tag link or group `!cxh_*` | Convex hull collision for that link |
| Primitive collision on one link | Tag link or group `!pri_*` | Primitive collision for that link |

Clean Fusion structure still gives cleaner URDF, cleaner xacro, better
mesh organization, and fewer surprises downstream. The exporter handles
messy imported CAD where it can, but it cannot infer design intent as
well as a deliberate model can.

