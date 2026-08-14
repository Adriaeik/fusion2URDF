# Limitations and Design Tradeoffs

The exporter now handles many awkward Fusion models, including design-root
bodies, rigid-group frame helpers, per-link collision overrides, and Fusion
fasteners inside rigid groups. It still cannot infer every bit of design
intent from CAD alone.

## Frame Control

Fusion makes coordinate-system cleanup harder after joints already exist.
For new designs, choose the ROS convention early: `Z` up, `X` forward,
`Y` left. Create joints after the link frames and rotation axes are correct.

If a component origin is wrong but the geometry is otherwise good, add a
`!frame_*` helper component. Inside a rigid group, that helper overrides the
merged link frame and does not export as geometry. This is the preferred fix
for base frames, wheel centers, sensor frames, mount frames, and tool frames.

Nested regular joints (joints defined inside a subassembly such as an arm)
often return null `geometry` / `geometryOrOriginOne` until the exporter
proxies them with `createForAssemblyContext`. Exporter v3.0.1+ does this
automatically so mesh bake can place the URDF link frame on the real hinge.
If a movable joint still logs a fallback to `occ_one_transform2`, add a
`!frame_*` at that hinge or recreate the joint so Fusion exposes a joint
origin.

Orientation cleanup no longer requires rebuilding the assembly. The default
post-export ROS convention keeps the Fusion design-world root `X` forward and
`Z` up and rebases revolute/continuous child frames to local `+Z`. A verbose
export writes `config/frame_overrides.csv`; `auto`, `keep`, and non-root
`world_rpy` rules can be reapplied from `debug/frame_model.json` without
reopening Fusion or regenerating meshes.

Frame overrides are orientation-only. They cannot move a link origin or a
joint's physical axis position while preserving revolute motion. For those
changes, place a `!frame_*` helper at the intended position and export again.
An arbitrary root `world_rpy` also requires a wrapper link, so the root uses
the selected automatic convention instead.

## Rigid Groups

A rigid group exports as one URDF link. Internal rigid joints are redundant and
are dropped quietly, which is expected for screws, nuts, washers, and Fusion's
fastener hardware. Internal non-rigid joints still warn because their motion
would be lost inside one link.

Put fasteners in the rigid group for the part they are attached to. Do not make
them separate links unless the fastener itself needs to move.

## Collision Geometry

Generated primitive and convex-hull collisions are approximations. They are
good defaults for simulation speed, but they are not substitutes for deliberate
contact geometry when fit matters.

Use `!collision_*` for hand-modelled simplified collision, `!cxh_*` for a
per-link convex hull, `!pri_*` for a per-link primitive, and `!acc_*` only when
exact visual-mesh collision is really needed.

## Export Time

Large assemblies can take time to export, especially when many visual meshes
must be merged or collision meshes must be generated. Convex hull and exact
mesh collision are usually slower than primitive collision. Prefer simple
collision where possible, and use per-link overrides for the few parts that
need something more accurate.

## URDF Tree Shape

URDF is a tree. Closed loops are exported as a valid tree plus sidecar metadata
in `robot_data.yaml`. Tag intentional loop-closing joints with `!closing_*` so
the exporter does not need to guess which joint should leave the URDF tree.
