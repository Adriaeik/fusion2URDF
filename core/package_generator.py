"""
Package Generator — Generate a complete ROS2 robot_description package.

Creates the standard ROS2 package structure:
  <robot>_description/
    ├── CMakeLists.txt
    ├── package.xml
    ├── urdf/
    │   └── <robot>.urdf
    ├── meshes/
    │   ├── <assembly>/
    │   │   ├── <link>.obj + .mtl
    │   │   └── <link>.stl
    │   └── ...
    ├── launch/
    │   └── display.launch.py
    ├── rviz/
    │   └── display.rviz
    └── config/
        └── joint_state.yaml

Pure Python — no Fusion API imports.

Author: Adrian Valaker Eikeland
"""

import os
from datetime import datetime

from .data_types import RobotModel, ExportConfig
from .xacro_generator import generate_urdf
from .collision_generator import resolve_collision, generate_collision_meshes
from .frame_rebaser import (
    FRAME_CACHE_FILENAME,
    configure_frames,
    save_frame_cache,
)
from .xacro_generator import generate_xacro_package
from .ros2_control_generator import (
    command_interfaces,
    controller_name,
    generate_ros2_controllers_yaml,
    ros2_control_command_joints,
    ros2_control_enabled,
)
from ..utils import Logger


# ──────────────────────────────────────────────
# Public API
# ──────────────────────────────────────────────

def generate_package(
    model: RobotModel,
    config: ExportConfig,
    log: Logger,
) -> str:
    """
    Generate a complete ROS2 description package.
    
    Args:
        model: Robot model from Phase 2
        config: Export configuration
        log: Logger
        
    Returns:
        Path to generated package directory
    """
    # Defaults
    if not config.package_name:
        config.package_name = f"{model.name}_description"
    
    pkg_dir = os.path.join(config.output_dir, config.package_name)
    
    log.section("PACKAGE: GENERATE")
    log(f"  Package: {config.package_name}")
    log(f"  Output:  {pkg_dir}")
    
    # Create directory structure.  Optional dirs are created only if
    # the corresponding include_* config flag is on, so a "minimal"
    # export doesn't ship empty launch/ rviz/ config/ directories.
    dirs = [
        pkg_dir,
        os.path.join(pkg_dir, "urdf"),
        os.path.join(pkg_dir, "urdf", "assemblies"),
        os.path.join(pkg_dir, "meshes"),
    ]
    if getattr(config, "include_launch", True):
        dirs.append(os.path.join(pkg_dir, "launch"))
    emit_frame_artifacts = (
        str(getattr(config, "verbosity", "verbose")).strip().lower()
        != "minimal"
    )
    if getattr(config, "include_rviz", True):
        dirs.append(os.path.join(pkg_dir, "rviz"))
    if (getattr(config, "include_rviz", True)
            or ros2_control_enabled(config, model)
            or emit_frame_artifacts):
        dirs.append(os.path.join(pkg_dir, "config"))

    for d in dirs:
        os.makedirs(d, exist_ok=True)
    
    # 1. Resolve collision geometry
    resolve_collision(model, config, log, pkg_dir=pkg_dir)
    
    # 2. Generate collision STL files from automatic collision methods
    generate_collision_meshes(model, pkg_dir, log)

    # 3. Cache the canonical collision-resolved model, then apply the
    # orientation-only frame layer.  The cache makes CSV edits replayable
    # without another slow Fusion extraction or mesh export.
    log.section("PACKAGE: FRAMES")
    frame_cache_path = os.path.join(
        config.output_dir, "debug", FRAME_CACHE_FILENAME
    )
    save_frame_cache(model, config, frame_cache_path)
    log(f"  -> debug/{FRAME_CACHE_FILENAME} (pre-frame cache)")
    configure_frames(
        model,
        config,
        pkg_dir,
        log,
        emit_artifacts=emit_frame_artifacts,
    )
    
    # 4. Generate xacro files (hierarchical)
    log.section("PACKAGE: XACRO")
    xacro_files = generate_xacro_package(model, config, pkg_dir)
    for rel_path in sorted(xacro_files.keys()):
        log(f"  → {rel_path}")
    
    # 5. Also generate flat URDF for validation/debugging
    log.section("PACKAGE: URDF (flat, for validation)")
    urdf_xml = generate_urdf(model, config)
    urdf_path = os.path.join(pkg_dir, "urdf", f"{model.name}.urdf")
    _write_file(urdf_path, urdf_xml)
    log(f"  → urdf/{model.name}.urdf")
    
    # 5. Generate package.xml
    log.section("PACKAGE: ROS2 FILES")
    pkg_xml = _generate_package_xml(model, config)
    _write_file(os.path.join(pkg_dir, "package.xml"), pkg_xml)
    log(f"  → package.xml")
    
    # 4. Generate CMakeLists.txt
    cmake = _generate_cmake(model, config)
    _write_file(os.path.join(pkg_dir, "CMakeLists.txt"), cmake)
    log(f"  → CMakeLists.txt")
    
    # 5. Generate launch file (optional)
    if getattr(config, "include_launch", True):
        launch = _generate_launch(model, config)
        _write_file(os.path.join(pkg_dir, "launch", "display.launch.py"), launch)
        log(f"  → launch/display.launch.py")

    # 6. Generate RViz config (optional)
    if getattr(config, "include_rviz", True):
        rviz = _generate_rviz_config(model, config)
        _write_file(os.path.join(pkg_dir, "rviz", "display.rviz"), rviz)
        log(f"  → rviz/display.rviz")

        # 7. Joint state config (for joint_state_publisher_gui)
        joint_cfg = _generate_joint_config(model, config)
        _write_file(os.path.join(pkg_dir, "config", "joint_state.yaml"), joint_cfg)
        log(f"  → config/joint_state.yaml")

    if ros2_control_enabled(config, model):
        control_cfg = generate_ros2_controllers_yaml(model, config)
        _write_file(
            os.path.join(pkg_dir, "config", "ros2_controllers.yaml"),
            control_cfg,
        )
        log(f"  → config/ros2_controllers.yaml")

    # 8. Supplementary data (YAML + KaTeX transforms) — both optional,
    # but ``robot_data.yaml`` is FORCED when the model has closing
    # joints.  The closing-joint sidecar lives inside that file and is
    # required for downstream URDF→USD pipelines to author the
    # corresponding USD physics joints — without it, a minimal-mode
    # export of a closed-kinematic-chain design ships a
    # broken-by-omission package.
    yaml_user_opted_in = getattr(config, "include_robot_data_yaml", True)
    has_closing = bool(getattr(model, "closing_joints", None))
    yaml_required = yaml_user_opted_in or has_closing
    docs_required = getattr(config, "include_docs", True)
    if yaml_required or docs_required:
        log.section("PACKAGE: SUPPLEMENTARY DATA")
        from .supplementary_export import (
            generate_supplementary_yaml, generate_transforms_doc,
        )
        if yaml_required:
            if has_closing and not yaml_user_opted_in:
                msg = (
                    f"robot_data.yaml emitted despite minimal mode: "
                    f"{len(model.closing_joints)} closing joint(s) require "
                    f"the sidecar so downstream URDF→USD pipelines can "
                    f"reconstruct the closed kinematic loop in USD."
                )
                model.warnings.append(msg)
                log.warning(msg)
            generate_supplementary_yaml(model, config, pkg_dir, log)
        if docs_required:
            generate_transforms_doc(model, pkg_dir, log)

    # 9. Robot README (optional)
    if getattr(config, "include_readme", True):
        log.section("PACKAGE: README")
        _generate_robot_readme(model, config, pkg_dir, log)

    # Final cleanup: when DAE export is on, the per-link OBJ+MTL was
    # retained through collision resolution (so fit_primitive's
    # circularity sampler had vertices to read).  Now that the
    # collision STLs are written and the URDF references the .dae,
    # drop the leftover OBJ pair so the package contains only the
    # mesh format the URDF actually points at.
    cleaned = 0
    for link in model.links.values():
        stash = getattr(link, "_dae_pending_obj_cleanup", None)
        if stash:
            for stale in stash:
                try:
                    if stale and os.path.isfile(stale):
                        os.remove(stale)
                        cleaned += 1
                except OSError:
                    pass
            link._dae_pending_obj_cleanup = None
        input_stash = getattr(link, "_collision_input_cleanup", None)
        if input_stash:
            collision_mesh = getattr(getattr(link, "collision", None), "mesh_path", "")
            keep_input = (
                collision_mesh
                and collision_mesh == getattr(link, "mesh_collision_input", "")
            )
            if not keep_input:
                for stale in input_stash:
                    try:
                        if stale and os.path.isfile(stale):
                            os.remove(stale)
                            cleaned += 1
                    except OSError:
                        pass
                link.mesh_collision_input = ""
            link._collision_input_cleanup = None
    if cleaned:
        log(f"  Cleaned up {cleaned} retained OBJ/MTL files")

    empty_dirs = _remove_empty_dirs(os.path.join(pkg_dir, "meshes"))
    empty_dirs += _remove_empty_dirs(os.path.join(pkg_dir, "urdf", "assemblies"))
    if empty_dirs:
        log(f"  Removed {empty_dirs} empty generated directories")

    log.section("PACKAGE: COMPLETE")
    log(f"  Package generated: {pkg_dir}")
    assembly_macro_count = sum(
        1 for rel_path in xacro_files
        if rel_path.startswith("urdf/assemblies/")
    )
    log(f"  Xacro: urdf/{model.name}.urdf.xacro (+ {assembly_macro_count} assembly macros)")
    log(f"  URDF:  urdf/{model.name}.urdf (flat, for validation)")
    if getattr(config, "include_launch", True):
        log(f"  Launch: ros2 launch {config.package_name} display.launch.py")

    return pkg_dir


def regenerate_frame_outputs(
    model: RobotModel,
    config: ExportConfig,
    pkg_dir: str,
    log: Logger,
) -> None:
    """Rewrite frame-dependent outputs from a cached canonical model.

    Mesh and collision generation are deliberately skipped.  ``model`` must
    already have collision information and frame rebases applied (normally by
    ``tools/reframe.py`` after loading ``debug/frame_model.json``).
    """
    os.makedirs(os.path.join(pkg_dir, "urdf", "assemblies"), exist_ok=True)
    os.makedirs(os.path.join(pkg_dir, "config"), exist_ok=True)

    log.section("OFFLINE FRAMES: XACRO + URDF")
    xacro_files = generate_xacro_package(model, config, pkg_dir)
    for rel_path in sorted(xacro_files):
        log(f"  -> {rel_path}")

    urdf_xml = generate_urdf(model, config)
    urdf_path = os.path.join(pkg_dir, "urdf", f"{model.name}.urdf")
    _write_file(urdf_path, urdf_xml)
    log(f"  -> urdf/{model.name}.urdf")

    if getattr(config, "include_rviz", True):
        _write_file(
            os.path.join(pkg_dir, "config", "joint_state.yaml"),
            _generate_joint_config(model, config),
        )
        log("  -> config/joint_state.yaml")

    if ros2_control_enabled(config, model):
        _write_file(
            os.path.join(pkg_dir, "config", "ros2_controllers.yaml"),
            generate_ros2_controllers_yaml(model, config),
        )
        log("  -> config/ros2_controllers.yaml")

    yaml_required = (
        getattr(config, "include_robot_data_yaml", True)
        or bool(getattr(model, "closing_joints", None))
    )
    docs_required = getattr(config, "include_docs", True)
    if yaml_required or docs_required:
        from .supplementary_export import (
            generate_supplementary_yaml,
            generate_transforms_doc,
        )
        if yaml_required:
            generate_supplementary_yaml(model, config, pkg_dir, log)
        if docs_required:
            generate_transforms_doc(model, pkg_dir, log)

    if getattr(config, "include_readme", True):
        _generate_robot_readme(model, config, pkg_dir, log)


def generate_validation_report(model: RobotModel, config: ExportConfig, debug_dir: str):
    """
    Generate validation.md summarizing build quality.
    
    Written to debug/ alongside snapshot.json and export_log.md.
    """
    lines = []
    lines.append(f"# Validation Report: {model.name}")
    lines.append("")
    
    # Overall status
    has_errors = len(model.errors) > 0
    has_warnings = len(model.warnings) > 0
    if has_errors:
        lines.append("## Status: FAIL")
    elif has_warnings:
        lines.append("## Status: PASS (with warnings)")
    else:
        lines.append("## Status: PASS")
    lines.append("")
    
    # Summary table
    lines.append("## Summary")
    lines.append("")
    lines.append(f"| Metric | Value |")
    lines.append(f"|--------|-------|")
    lines.append(f"| Links | {len(model.links)} |")
    lines.append(f"| Joints | {len(model.joints)} |")
    lines.append(f"| Assemblies | {len(model.assemblies)} |")
    lines.append(f"| Root | `{model.root_link}` |")
    lines.append(f"| Errors | {len(model.errors)} |")
    lines.append(f"| Warnings | {len(model.warnings)} |")
    lines.append("")
    
    # Errors
    if model.errors:
        lines.append("## Errors")
        lines.append("")
        for e in model.errors:
            lines.append(f"- {e}")
        lines.append("")
    
    # Warnings
    if model.warnings:
        lines.append("## Warnings")
        lines.append("")
        for w in model.warnings:
            lines.append(f"- {w}")
        lines.append("")
    
    # Kinematic tree
    lines.append("## Kinematic Tree")
    lines.append("")
    lines.append("```")
    
    reachable = _reachable_links(model)
    
    def _tree(link_name, depth=0):
        link = model.links.get(link_name)
        if not link:
            return
        bake_tag = " [BAKE]" if link.needs_mesh_bake else ""
        empty_tag = " [EMPTY]" if link.is_empty else ""
        coll_tag = ""
        if link.collision:
            coll_tag = f" [{link.collision.source}]"
        reach_tag = "" if link_name in reachable else " ⚠ UNREACHABLE"
        lines.append(f"{'  ' * depth}{link_name}{empty_tag}{bake_tag}{coll_tag}{reach_tag}")
        # Find children
        for jname, joint in model.joints.items():
            if joint.parent_link == link_name:
                jtype = joint.joint_type
                lines.append(f"{'  ' * depth}  └─ {jname} [{jtype}]")
                _tree(joint.child_link, depth + 2)
    
    _tree(model.root_link)
    
    # Show unreachable links
    unreachable = set(model.links.keys()) - reachable
    if unreachable:
        lines.append("")
        lines.append("UNREACHABLE:")
        for u in sorted(unreachable):
            lines.append(f"  {u}")
    
    lines.append("```")
    lines.append("")
    
    # Collision table
    lines.append("## Collision Geometry")
    lines.append("")
    lines.append("| Link | Source | Shape/File |")
    lines.append("|------|--------|------------|")
    for link_name in sorted(model.links.keys()):
        link = model.links[link_name]
        if link.is_empty:
            lines.append(f"| `{link_name}` | empty (ref frame) | — |")
            continue
        coll = link.collision
        if coll:
            if coll.source == "primitive" and coll.primitive:
                shape = f"{coll.primitive.shape}"
                lines.append(f"| `{link_name}` | primitive STL | {shape} |")
            elif coll.source == "convex_hull":
                lines.append(f"| `{link_name}` | convex hull STL | `{coll.mesh_path}` |")
            elif coll.source == "explicit":
                lines.append(f"| `{link_name}` | explicit | `{coll.mesh_path}` |")
            elif coll.source == "visual_fallback":
                lines.append(f"| `{link_name}` | visual fallback | `{coll.mesh_path}` |")
            else:
                lines.append(f"| `{link_name}` | {coll.source} | — |")
        else:
            lines.append(f"| `{link_name}` | none | — |")
    lines.append("")
    
    # Bake offset table
    bake_links = [(n, l) for n, l in model.links.items() if l.needs_mesh_bake]
    if bake_links:
        lines.append("## Mesh Bake Offsets")
        lines.append("")
        lines.append("Links where joint frame ≠ component origin. Visual/inertial/collision origins shifted.")
        lines.append("")
        lines.append("| Link | Offset (mm) |")
        lines.append("|------|-------------|")
        for name, link in bake_links:
            b = link.mesh_bake_offset
            lines.append(f"| `{name}` | ({b[0]*1000:.1f}, {b[1]*1000:.1f}, {b[2]*1000:.1f}) |")
        lines.append("")
    
    # Write
    os.makedirs(debug_dir, exist_ok=True)
    path = os.path.join(debug_dir, "validation.md")
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))
    
    return path


def _reachable_links(model: RobotModel) -> set:
    """BFS from root to find all reachable links."""
    visited = set()
    frontier = [model.root_link]
    while frontier:
        current = frontier.pop(0)
        if current in visited:
            continue
        visited.add(current)
        for jname, joint in model.joints.items():
            if joint.parent_link == current:
                frontier.append(joint.child_link)
    return visited


# ──────────────────────────────────────────────
# File generators
# ──────────────────────────────────────────────

def _generate_package_xml(model: RobotModel, config: ExportConfig) -> str:
    """Generate ROS2 package.xml."""
    # Build XML name tags explicitly to avoid escaping issues
    name_open = "<" + "name" + ">"
    name_close = "</" + "name" + ">"
    ros2_control_deps = ""
    if ros2_control_enabled(config, model):
        ros2_control_deps = """  <exec_depend>controller_manager</exec_depend>
  <exec_depend>ros2_control</exec_depend>
  <exec_depend>ros2_controllers</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>forward_command_controller</exec_depend>
  <exec_depend>mock_components</exec_depend>
"""
    return f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  {name_open}{config.package_name}{name_close}
  <version>0.1.0</version>
  <description>Robot description for {model.name}, generated from Fusion 360</description>

  <maintainer email="todo@todo.com">TODO</maintainer>
  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>xacro</exec_depend>
{ros2_control_deps}

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""


def _generate_cmake(model: RobotModel, config: ExportConfig) -> str:
    """Generate CMakeLists.txt for a ROS2 description package.

    Lists only the directories that the export actually wrote — a
    "minimal" config drops launch/rviz/config and CMakeLists drops
    them from the install rule too, so ``colcon build`` doesn't fail
    on missing directories.
    """
    install_dirs = ["urdf", "meshes"]
    if getattr(config, "include_launch", True):
        install_dirs.append("launch")
    if getattr(config, "include_rviz", True):
        install_dirs.append("rviz")
    emit_frame_artifacts = (
        str(getattr(config, "verbosity", "verbose")).strip().lower()
        != "minimal"
    )
    if (getattr(config, "include_rviz", True)
            or ros2_control_enabled(config, model)
            or emit_frame_artifacts):
        install_dirs.append("config")
    return f"""cmake_minimum_required(VERSION 3.8)
project({config.package_name})

find_package(ament_cmake REQUIRED)

# Install package data
install(DIRECTORY {' '.join(install_dirs)}
  DESTINATION share/${{PROJECT_NAME}}
)

ament_package()
"""


def _generate_launch(model: RobotModel, config: ExportConfig) -> str:
    """Generate ROS2 launch file for visualization."""
    has_ros2_control = ros2_control_enabled(config, model)
    has_rviz = getattr(config, "include_rviz", True)

    setup_lines = [
        f"    xacro_file = os.path.join(pkg_dir, 'urdf', '{model.name}.urdf.xacro')",
    ]
    if has_rviz:
        setup_lines.append("    rviz_file = os.path.join(pkg_dir, 'rviz', 'display.rviz')")
    if has_ros2_control:
        setup_lines.append("    controllers_file = os.path.join(pkg_dir, 'config', 'ros2_controllers.yaml')")

    node_blocks = [
        """            # Robot state publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{'robot_description': robot_description}],
                output='screen',
            ),"""
    ]

    if has_ros2_control:
        node_blocks.append("""            # ros2_control controller manager using generated mock hardware
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                name='controller_manager',
                parameters=[{'robot_description': robot_description}, controllers_file],
                output='screen',
            ),""")
        spawners = ["joint_state_broadcaster"]
        if ros2_control_command_joints(model):
            spawners.extend(
                controller_name(iface) for iface in command_interfaces(config)
            )
        for spawner in spawners:
            node_blocks.append(f"""            Node(
                package='controller_manager',
                executable='spawner',
                name='spawn_{spawner}',
                arguments=['{spawner}', '--controller-manager', 'controller_manager'],
                output='screen',
            ),""")
    else:
        node_blocks.append("""            # Joint state publisher GUI (for manual joint control)
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen',
            ),""")

    if has_rviz:
        node_blocks.append("""            # RViz2
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_file],
                output='screen',
            ),""")

    setup = "\n".join(setup_lines)
    nodes = "\n\n".join(node_blocks)

    return f"""\"\"\"
Launch file for {model.name}.

Usage:
    ros2 launch {config.package_name} display.launch.py
    ros2 launch {config.package_name} display.launch.py namespace:=robot1
\"\"\"

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_dir = get_package_share_directory('{config.package_name}')

{setup}

    ns = LaunchConfiguration('namespace')
    prefix = LaunchConfiguration('prefix')

    robot_description = Command([
        'xacro ', xacro_file, ' prefix:=', prefix,
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='ROS 2 namespace for topics and nodes',
        ),
        DeclareLaunchArgument(
            'prefix', default_value='',
            description='URDF link/joint name prefix (passed to xacro)',
        ),

        GroupAction([
            PushRosNamespace(ns),

{nodes}
        ]),
    ])
"""


def _generate_rviz_config(model: RobotModel, config: ExportConfig) -> str:
    """Generate RViz2 config file."""
    return f"""Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Value: true
      Cell Size: 0.5
      Color: 160; 160; 164
      Line Style:
        Line Width: 0.03
      Plane: XY
      Plane Cell Count: 20
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Value: true
      Description Source: Topic
      Description Topic:
        Value: /robot_description
      Visual Enabled: true
      Collision Enabled: false
      Alpha: 1.0
    - Class: rviz_default_plugins/TF
      Name: TF
      Value: true
      Show Arrows: true
      Show Axes: true
      Show Names: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: {model.root_link}
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/FocusCamera
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 2.0
      Focal Point:
        X: 0
        Y: 0
        Z: 0.3
      Name: Current View
"""


def _generate_joint_config(model: RobotModel, config: ExportConfig) -> str:
    """Generate joint state publisher config."""
    lines = ["# Joint configuration for joint_state_publisher_gui", ""]
    lines.append("joint_state_publisher:")
    lines.append("  ros__parameters:")
    lines.append("    # Joint names and default positions")
    lines.append("    zeros:")
    
    for jname, joint in sorted(model.joints.items()):
        if joint.joint_type in ('revolute', 'prismatic', 'continuous'):
            lines.append(f"      {jname}: 0.0")
    
    lines.append("")
    lines.append("    # Joints to publish")
    lines.append("    source_list:")
    
    for jname, joint in sorted(model.joints.items()):
        if joint.joint_type in ('revolute', 'prismatic', 'continuous'):
            lines.append(f"      - {jname}")
    
    return "\n".join(lines)


# ──────────────────────────────────────────────
# Robot README generation
# ──────────────────────────────────────────────

def _generate_robot_readme(
    model: RobotModel,
    config: ExportConfig,
    pkg_dir: str,
    log: Logger,
):
    """
    Generate a comprehensive README.md for the robot description package.
    
    Pulls section templates from templates/sections/ and fills in
    dynamic robot data (mass, links, joints, transforms).
    """
    lines = []
    
    # Try to find the robot image
    img_path = os.path.join(pkg_dir, "images", "robot.png")
    has_image = os.path.exists(img_path)
    
    # ── Header ──
    lines.append(f"# {model.name} — Robot Description")
    lines.append("")
    
    if has_image:
        lines.append(f"![{model.name}](images/robot.png)")
        lines.append("")
    
    # ── Overview table ──
    total_mass = sum(l.mass_kg for l in model.links.values())
    movable = sum(
        1 for j in model.joints.values()
        if j.joint_type in ('revolute', 'prismatic', 'continuous')
    )
    
    lines.append("## Overview")
    lines.append("")
    lines.append("| Property | Value |")
    lines.append("|----------|-------|")
    lines.append(f"| Total mass | {total_mass:.3f} kg |")
    lines.append(f"| Links | {len(model.links)} |")
    lines.append(f"| Joints | {len(model.joints)} ({movable} movable) |")
    lines.append(f"| Assemblies | {len(model.assemblies)} |")
    lines.append(f"| Root link | `{model.root_link}` |")
    lines.append("")
    
    # ── Table of Contents ──
    lines.append("## Table of Contents")
    lines.append("")
    lines.append("- [Kinematic Tree](#kinematic-tree)")
    lines.append("- [Link Properties](#link-properties)")
    lines.append("- [Joint Properties](#joint-properties)")
    lines.append("- [Assembly Breakdown](#assembly-breakdown)")
    lines.append("- [Quick Start (ROS 2)](#quick-start-ros-2)")
    lines.append("- [Files](#files)")
    lines.append("")
    
    # ── Kinematic tree (reuse validation tree builder) ──
    lines.append("## Kinematic Tree")
    lines.append("")
    lines.append("```")
    _build_tree_lines(model, lines)
    lines.append("```")
    lines.append("")
    
    # ── Link properties ──
    lines.append("## Link Properties")
    lines.append("")
    lines.append("| Link | Mass (kg) | Material | Collision | Bodies |")
    lines.append("|------|-----------|----------|-----------|--------|")
    for name in sorted(model.links.keys()):
        link = model.links[name]
        mat = link.material_name or "—"
        if link.is_empty:
            coll = "empty"
        elif link.collision:
            src = link.collision.source
            if link.collision.uses_primitive and link.collision.primitive:
                coll = link.collision.primitive.shape
            else:
                coll = src
        else:
            coll = "—"
        lines.append(f"| `{name}` | {link.mass_kg:.4f} | {mat} | {coll} | {link.body_count} |")
    lines.append("")
    
    # ── Joint properties ──
    lines.append("## Joint Properties")
    lines.append("")
    lines.append("| Joint | Type | Parent → Child | Axis | Limits |")
    lines.append("|-------|------|---------------|------|--------|")
    for name in sorted(model.joints.keys()):
        j = model.joints[name]
        ax_str = f"({j.axis[0]:.0f},{j.axis[1]:.0f},{j.axis[2]:.0f})"
        if j.limits and j.joint_type in ('revolute', 'prismatic', 'continuous'):
            import math
            if j.joint_type == 'prismatic':
                lim = f"[{j.limits.lower*1000:.1f}, {j.limits.upper*1000:.1f}] mm"
            else:
                lim = (f"[{math.degrees(j.limits.lower):.1f}°, "
                       f"{math.degrees(j.limits.upper):.1f}°]")
        else:
            lim = "—"
        lines.append(f"| `{name}` | {j.joint_type} | "
                     f"`{j.parent_link}` → `{j.child_link}` | {ax_str} | {lim} |")
    lines.append("")
    
    # ── Assembly breakdown ──
    lines.append("## Assembly Breakdown")
    lines.append("")
    for asm_name, asm_info in sorted(model.assemblies.items()):
        asm_mass = sum(
            model.links[ln].mass_kg
            for ln in asm_info.links if ln in model.links
        )
        lines.append(f"### {asm_name}")
        lines.append(f"")
        lines.append(f"- **Links**: {', '.join(asm_info.links)}")
        lines.append(f"- **Total mass**: {asm_mass:.3f} kg")
        lines.append("")
    
    # ── Quick Start (from template if available, else inline) ──
    lines.append("## Quick Start (ROS 2)")
    lines.append("")
    ros2_template = _load_template("ros2_quickstart.md")
    if ros2_template:
        lines.append(ros2_template.replace("{{package_name}}", config.package_name)
                                  .replace("{{robot_name}}", model.name))
    else:
        lines.append(f"```bash")
        lines.append(f"# Build")
        lines.append(f"colcon build --packages-select {config.package_name}")
        lines.append(f"source install/setup.bash")
        lines.append(f"")
        lines.append(f"# Visualize in RViz2")
        lines.append(f"ros2 launch {config.package_name} display.launch.py")
        if ros2_control_enabled(config, model):
            lines.append(f"")
            lines.append(f"# Command generated controllers")
            command_joint_count = len(ros2_control_command_joints(model))
            if command_joint_count:
                zeros = ", ".join("0.0" for _ in range(command_joint_count))
                for iface in command_interfaces(config):
                    ctrl = controller_name(iface)
                    lines.append(
                        f"ros2 topic pub /{ctrl}/commands "
                        f"std_msgs/msg/Float64MultiArray \"data: [{zeros}]\""
                    )
        lines.append(f"")
        lines.append(f"# Validate URDF")
        lines.append(f"check_urdf install/{config.package_name}/share/{config.package_name}/urdf/{model.name}.urdf")
        lines.append(f"```")
    lines.append("")
    
    # ── Files ──
    lines.append("## Files")
    lines.append("")
    lines.append("| Path | Description |")
    lines.append("|------|-------------|")
    lines.append(f"| `urdf/{model.name}.urdf.xacro` | Top-level xacro (entry point) |")
    lines.append(f"| `urdf/{model.name}.urdf` | Flat URDF (for validation) |")
    lines.append(f"| `urdf/assemblies/` | Per-assembly xacro macros |")
    lines.append(f"| `meshes/` | Visual (OBJ) and collision (STL) meshes |")
    if getattr(config, "include_launch", True):
        lines.append(f"| `launch/display.launch.py` | Launch robot_state_publisher, RViz, and generated controllers |")
    if getattr(config, "include_rviz", True):
        lines.append(f"| `config/joint_state.yaml` | Joint state publisher config |")
    if ros2_control_enabled(config, model):
        lines.append(f"| `config/ros2_controllers.yaml` | Generated ros2_control controller manager config |")
    lines.append(f"| `robot_data.yaml` | Supplementary data (beyond URDF) |")
    lines.append(f"| `docs/transforms.md` | Transformation matrices (KaTeX) |")
    lines.append("")
    
    # ── Additional sections from templates ──
    custom_template = _load_template("customizing.md")
    if custom_template:
        lines.append("## Customizing")
        lines.append("")
        lines.append(custom_template.replace("{{package_name}}", config.package_name)
                                    .replace("{{robot_name}}", model.name))
        lines.append("")
    
    # ── Footer ──
    from .data_types import EXPORTER_VERSION
    lines.append("---")
    lines.append(f"*Generated by Fusion URDF/XACRO Exporter v{EXPORTER_VERSION}*")
    
    readme_path = os.path.join(pkg_dir, "README.md")
    _write_file(readme_path, "\n".join(lines))
    log(f"  → README.md")


def _build_tree_lines(model: RobotModel, lines: list, link: str = None, depth: int = 0):
    """Build kinematic tree text lines recursively."""
    if link is None:
        link = model.root_link
    
    ldata = model.links.get(link)
    tags = ""
    if ldata:
        if ldata.is_empty:
            tags += " [EMPTY]"
        if ldata.needs_mesh_bake:
            tags += " [BAKE]"
    
    lines.append(f"{'  ' * depth}{link}{tags}")
    
    for jname, joint in model.joints.items():
        if joint.parent_link == link:
            lines.append(f"{'  ' * depth}  └─ {jname} [{joint.joint_type}]")
            _build_tree_lines(model, lines, joint.child_link, depth + 2)


def _load_template(name: str) -> str:
    """Load a template section from templates/sections/."""
    # Try relative to package root
    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(pkg_root, "templates", "sections", name)
    if os.path.exists(path):
        with open(path, 'r') as f:
            return f.read().strip()
    return ""


# ──────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────

def _write_file(path: str, content: str):
    """Write content to file, creating directories as needed."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(content)


def _remove_empty_dirs(root_dir: str) -> int:
    """Remove empty generated directories below *root_dir*, deepest first."""
    if not os.path.isdir(root_dir):
        return 0
    removed = 0
    for current, _dirs, _files in os.walk(root_dir, topdown=False):
        if current == root_dir:
            continue
        try:
            os.rmdir(current)
            removed += 1
        except OSError:
            pass
    return removed
