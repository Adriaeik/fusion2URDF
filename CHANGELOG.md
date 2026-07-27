# Changelog

All notable changes to **fusion2URDF** are documented here. The format
follows [Keep a Changelog](https://keepachangelog.com/), and the
project adheres to [Semantic Versioning](https://semver.org/).

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
