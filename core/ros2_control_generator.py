"""ROS 2 control generation helpers.

The exporter cannot know the user's real motor drivers, so the generated
description uses a standard mock ros2_control system by default.  That
gives every movable non-passive joint command interfaces and every movable
joint state interfaces, plus controller YAML that can be launched straight
away.  Users can replace the hardware plugin in ``xacro_export.toml`` or
edit the generated xacro for real hardware.
"""

from typing import Iterable, List, Optional

from .data_types import ExportConfig, JointNode, RobotModel


MOVABLE_JOINT_TYPES = ("revolute", "continuous", "prismatic")
STATE_INTERFACES = ("position", "velocity", "effort")
DEFAULT_COMMAND_INTERFACES = ("position", "velocity")


def ros2_control_joints(model: RobotModel) -> List[JointNode]:
    """Return movable joints represented in the ros2_control block."""
    return [
        joint
        for _name, joint in sorted(model.joints.items())
        if joint.joint_type in MOVABLE_JOINT_TYPES
        and not getattr(joint, "is_closing", False)
    ]


def ros2_control_command_joints(model: RobotModel) -> List[JointNode]:
    """Return movable joints that should accept controller commands."""
    return [
        joint
        for joint in ros2_control_joints(model)
        if not getattr(joint, "is_passive", False)
    ]


def ros2_control_enabled(config: ExportConfig, model: RobotModel) -> bool:
    """True when the package should emit ros2_control files/XML."""
    return bool(
        getattr(config, "include_ros2_control", False)
        and ros2_control_joints(model)
    )


def command_interfaces(config: ExportConfig) -> List[str]:
    """Return sanitized command interfaces requested by config."""
    raw = getattr(config, "ros2_control_command_interfaces", None)
    if not raw:
        raw = DEFAULT_COMMAND_INTERFACES
    if isinstance(raw, str):
        values: Iterable[str] = raw.split(",")
    else:
        values = raw

    out = []
    for value in values:
        name = str(value).strip().lower()
        if name in STATE_INTERFACES and name not in out:
            out.append(name)
    return out


def controller_name(interface_name: str) -> str:
    """Controller name for one command interface."""
    return f"{interface_name}_controller"


def generate_ros2_control_xml(
    model: RobotModel,
    config: ExportConfig,
    indent: int = 2,
    joints: Optional[Iterable[JointNode]] = None,
    system_name: Optional[str] = None,
    use_prefix: bool = False,
) -> List[str]:
    """Generate a ros2_control XML block.

    Xacro package output calls this per assembly so downstream tools can
    see ``panther_system`` and ``turret_system`` separately.  Flat URDF
    output still calls it once for the whole robot.
    """
    joints = list(joints) if joints is not None else ros2_control_joints(model)
    command_joints = {
        joint.name
        for joint in joints
        if not getattr(joint, "is_passive", False)
    }
    cmd_ifaces = command_interfaces(config)
    pad = " " * indent
    plugin = _xml_safe(
        getattr(config, "ros2_control_hardware_plugin", "")
        or "mock_components/GenericSystem"
    )
    system_base = system_name or f"{model.name}_system"
    system_attr = _xml_safe(
        f"${{prefix}}{system_base}" if use_prefix else system_base
    )

    lines = []
    lines.append(f"{pad}<!-- ROS 2 control: generated generic system -->")
    lines.append(f'{pad}<ros2_control name="{system_attr}" type="system">')
    lines.append(f"{pad}  <hardware>")
    lines.append(f"{pad}    <plugin>{plugin}</plugin>")
    lines.append(f"{pad}    <param name=\"cmd_topic\">sim/{_xml_safe(system_base)}/cmd</param>")
    lines.append(f"{pad}    <param name=\"state_topic\">sim/{_xml_safe(system_base)}/state</param>")
    lines.append(f"{pad}  </hardware>")
    lines.append("")

    for joint in joints:
        joint_name = _xml_safe(
            f"${{prefix}}{joint.name}" if use_prefix else joint.name
        )
        lines.append(f'{pad}  <joint name="{joint_name}">')
        if joint.name in command_joints:
            for iface in cmd_ifaces:
                lines.append(f'{pad}    <command_interface name="{iface}"/>')
        for iface in STATE_INTERFACES:
            if iface == "position":
                lines.append(f'{pad}    <state_interface name="{iface}">')
                lines.append(f'{pad}      <param name="initial_value">0.0</param>')
                lines.append(f'{pad}    </state_interface>')
            else:
                lines.append(f'{pad}    <state_interface name="{iface}"/>')
        lines.append(f"{pad}  </joint>")

    lines.append(f"{pad}</ros2_control>")
    return lines


def generate_ros2_controllers_yaml(
    model: RobotModel,
    config: ExportConfig,
) -> str:
    """Generate controller_manager YAML for the generated ros2_control block."""
    command_joints = ros2_control_command_joints(model)
    cmd_ifaces = command_interfaces(config)
    update_rate = int(getattr(config, "ros2_control_update_rate", 100) or 100)

    lines = [
        "# Generated ros2_control controllers",
        "# Publishes joint states on: /joint_states",
        "# Command topics use: /<controller_name>/commands",
        "/**:",
        "  controller_manager:",
        "    ros__parameters:",
        f"      update_rate: {update_rate}",
        "",
        "      joint_state_broadcaster:",
        "        type: joint_state_broadcaster/JointStateBroadcaster",
    ]

    if command_joints and cmd_ifaces:
        for iface in cmd_ifaces:
            lines.append(f"      {controller_name(iface)}:")
            lines.append("        type: forward_command_controller/ForwardCommandController")

    lines.append("")
    lines.append("  joint_state_broadcaster:")
    lines.append("    ros__parameters: {}")

    if command_joints and cmd_ifaces:
        joint_names = [joint.name for joint in command_joints]
        formatted_joints = ", ".join(joint_names)
        for iface in cmd_ifaces:
            lines.append("")
            lines.append(f"  # Command topic: /{controller_name(iface)}/commands")
            lines.append(f"  {controller_name(iface)}:")
            lines.append("    ros__parameters:")
            lines.append(f"      joints: [{formatted_joints}]")
            lines.append(f"      interface_name: {iface}")

    return "\n".join(lines) + "\n"


def _xml_safe(text: str) -> str:
    return (
        str(text)
        .replace("&", "&amp;")
        .replace('"', "&quot;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
    )
