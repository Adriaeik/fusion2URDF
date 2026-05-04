# Customizing the Robot Description

## Swapping Assemblies (`!dummy_` convention)

Assemblies tagged with `!dummy_` are placeholders designed to be
replaced. This is the primary extensibility mechanism.

### Example: Replacing a Sensor Module

A `!dummy_` assembly (for example `!dummy_camera`, exported as the
`dummy_camera` macro) serves as a placeholder. To replace it:

1. Create a new xacro file `urdf/assemblies/my_camera.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_camera_macro">
  <xacro:macro name="my_camera" params="prefix">
    <link name="${prefix}camera_link">
      <!-- Your camera link definition -->
    </link>
  </xacro:macro>
</robot>
```

2. Update the include and macro call in `urdf/<robot>.urdf.xacro`:

```xml
<!-- Replace this: -->
<xacro:include filename="$(find pkg)/urdf/assemblies/dummy_camera.urdf.xacro"/>
<xacro:dummy_camera prefix=""/>

<!-- With this: -->
<xacro:include filename="$(find pkg)/urdf/assemblies/my_camera.urdf.xacro"/>
<xacro:my_camera prefix=""/>
```

3. Update the mount joint to connect to your new link name.

## Adding New Joints

New joints are added in Fusion 360 by creating joints between components. The exporter automatically detects joint type (revolute, prismatic, fixed) from the Fusion joint's motion type.

## Collision Strategy

During export you usually choose the collision method through a two-step
dialog. If `[mesh].collision_method` is set in `xacro_export.toml`, the
dialog is skipped; use that path for advanced modes such as
`convex_hull`. The answer is applied per-link according to the priority
chain in [DESIGN_RULES.md §2.2](../DESIGN_RULES.md):

**Step 1 — primary method** (one prompt, always):

- **Primitive (recommended)**: Auto-fits box/cylinder/sphere per link
  based on bounding box analysis. Lightweight, good for physics
  simulation.
- **Convex hull**: Config-only method that generates a convex STL from
  the exported visual OBJ vertices. Useful for imported or angled CAD
  where a box/cylinder is too rough.
- **Visual fallback**: Uses the full visual mesh as collision geometry.
  Exact shape but heavier for physics engines.

**Step 2 — explicit-collision reconciliation** (only appears if the
design contains `!collision_*` components or bodies):

- After primary=**primitive** → *"Use precise STL where you made it?"*
  Yes = mixed mode (explicit STL for links you designed collision on,
  primitives elsewhere). No = primitives everywhere, your explicit
  geometry is ignored.
- After primary=**visual** → *"Override explicit collision with visual?"*
  Yes = visual for all. No = keep explicit where designed, visual
  for the rest.

### Explicit Collision via Rigid Groups

1. In Fusion, create a simplified component or body named
   `!collision_<suffix>`. Pick a suffix when one assembly contains more
   than one collision so the clean names stay unique.
2. Add it to the same assembly as the visual components.
3. Create a Rigid Group containing the visual components and the
   collision component.
4. The exporter detects the collision-prefix member, exports its
   geometry once, and all other members of the rigid group reference
   the shared STL with the correct spatial offsets.

See [DESIGN_RULES.md](../DESIGN_RULES.md) for the full convention
(sub-components, body naming, and the collision priority chain).

### Per-Link Collision Overrides

Use the global `[mesh].collision_method` for the default behavior, then
tag exceptions directly in Fusion:

| Tag | Effect |
| --- | --- |
| `!pri_<link>` | Force primitive collision for this link |
| `!cxh_<link>` | Force generated convex hull collision for this link |
| `!acc_<link>` | Force exact visual mesh collision for this link |

The keyword is stripped from the exported link name. For example,
`!cxh_body` exports as link `body`.

### If explicit collision won't export

Fusion's STL exporter occasionally returns success while writing
degenerate (near-zero) geometry for nested sub-assembly occurrences.
The pipeline detects this post-export by checking the STL bounding
box and retries with the component reference directly. If both
attempts fail, the log prints

```
Collision STL failed for … — falling back to primitive
```

and the link gets a primitive box from its bounding box instead of
a broken STL reference in the URDF. Inspect `debug/export_log.md`
for the exact bbox reported on each export — this makes silent
corruption impossible and tells you at a glance whether your
explicit collision actually landed.

## Mesh Quality

The export uses Fusion's `"medium"` mesh refinement by default. Change
it in `fusion2URDF/xacro_export.toml`:

```toml
[mesh]
mesh_refinement = "high"
```

- `"low"` — fewer triangles, faster simulation
- `"medium"` — balanced (default)
- `"high"` — detailed, suitable for visualization

The same file also controls the visual mesh format and default
collision strategy:

```toml
[mesh]
visual_format = "dae"          # or "obj"
collision_method = "primitive" # or "convex_hull" / "visual_reuse"
```

See [Getting Started](GETTING_STARTED.md#optional-persistent-config)
for all supported TOML settings.

## Prefix System (Multi-Robot)

Every assembly macro accepts a `prefix` parameter that prepends to all link and joint names in the URDF. This is needed because URDF link names must be globally unique (TF frames).

```xml
<xacro:arm prefix="left_"/>
<!-- Produces: left_shoulder_link, left_elbow_link, left_wrist_link -->
```

**Prefix vs namespace**: In ROS 2 multi-robot setups you typically need both. The xacro `prefix` disambiguates URDF link/joint names (and therefore TF frame names). The ROS 2 `namespace` (set in the launch file) disambiguates topics and nodes. Our exporter handles the xacro prefix; namespace is configured at launch time.
