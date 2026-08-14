# Changelog

All notable changes to **fusion2URDF** are documented here. The format
follows [Keep a Changelog](https://keepachangelog.com/), and the
project adheres to [Semantic Versioning](https://semver.org/).

## [3.1.0] - 2026-08-14

### Added

- Post-export, orientation-only frame rebasing. The default ROS convention
  keeps the Fusion design-world root `X` forward and `Z` up, and expresses
  revolute and continuous joint axes as child-local `+Z`.
- Editable `config/frame_overrides.csv` rules (`auto`, `keep`, and
  `world_rpy`) plus an offline `python -m fusion2URDF.tools.reframe` command.
- A canonical `debug/frame_model.json` cache, allowing frame-dependent URDF,
  xacro, YAML, and documentation files to be regenerated without reopening
  Fusion or exporting meshes again.
- Regression coverage for frame invariance, cache round-trips, nested rigid
  group anchors, and subassembly-container joint endpoints.

### Fixed

- Merged rigid-group OBJ meshes now use the anchor pose relative to the actual
  export lowest common ancestor. Nested anchors no longer displace a link's
  mesh far away from its physical placement.
- Joint endpoints that target a subassembly container now resolve to that
  subassembly's deterministic internal root link instead of producing an
  invalid or misplaced connection.
- Frame-only links remain valid links without fabricated visual, collision,
  mass, or inertia blocks.
- Jointless single-body designs now export a deterministic root link, and
  part-number assembly names produce valid XML/xacro macro names.

### Changed

- Mesh, collision, center-of-mass, inertia, joint-origin, and joint-axis data
  are compensated together when a frame is rebased, preserving the physical
  mechanism and visible geometry.
- Snapshot JSON loading is shared by the validation and reframe tools.
- Generated exports, debug data, Python caches, and local test scratch remain
  ignored; the curated source designs and examples stay tracked.

## [3.0.1] - 2026-07-22

- Proxy nested joints with `createForAssemblyContext` so geometry origins resolve.
- Lift occurrence-local joint origins through the child/parent world pose; do not double-lift world-proxied origins.
- Remap revolute/prismatic axes as `R_child^T * axis_world` (no defining-assembly pre-multiply).
- Prefer occurrence `transform2` over the assemblyContext walk for origin fallbacks.

## [3.0.0] — 2026-05-04

Initial public release.

### Highlights

- Hierarchical xacro from nested Fusion 360 assemblies, with each
  assembly emitted as a reusable macro with a `prefix` parameter.
- Generated `ros2_control` blocks (mock hardware by default; drop-in
  for real drivers) plus `config/ros2_controllers.yaml`.
- Six-strategy collision pipeline: explicit `!collision_*`, per-link
  `!acc_*` / `!cxh_*` / `!pri_*` overrides, auto-fitted primitives,
  generated convex hulls, or visual-mesh fallback.
- Closed-loop joint sidecar (`!closing_*` written to `robot_data.yaml`).
- Frame helpers (`!frame_*`) for axis/origin control without
  remodelling the geometry.
- Placeholder modules (`!dummy_*`) for swappable sensors and tools.
- Symbolic transforms in KaTeX, JSON snapshot for offline tooling,
  optional Fusion viewport screenshot in the generated package.
- Two ready-to-launch example exports under `examples/`: Husarion
  Panther (mobile robot, frame helpers, hierarchical xacro) and
  `Assem1` (6-DoF arm with parallel gripper, closed-loop kinematics
  and per-link collision overrides). Source Fusion designs included.
- A trivia window during long exports, sourced from OpenTDB, because
  watching a progress bar for two minutes is its own form of suffering.
