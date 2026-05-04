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

If many joints were created around the wrong world orientation, rebuilding the
small assembly can be faster and safer than trying to patch every joint.

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
