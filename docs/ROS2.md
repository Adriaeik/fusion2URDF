# ROS 2 Usage

## Prerequisites

```bash
sudo apt install ros-humble-joint-state-publisher-gui \
                 ros-humble-robot-state-publisher \
                 ros-humble-rviz2 \
                 ros-humble-xacro \
                 ros-humble-controller-manager \
                 ros-humble-ros2-control \
                 ros-humble-ros2-controllers
```

## Build And Launch

```bash
# Copy the generated package to your workspace
cp -r <robot>_description ~/ros2_ws/src/

cd ~/ros2_ws
colcon build --packages-select <robot>_description
source install/setup.bash

# Launch RViz2 and generated controllers
ros2 launch <robot>_description display.launch.py
```

## Launch Contents

The generated launch file starts:

- `robot_state_publisher`: publishes the URDF and TF transforms
- `controller_manager`: when generated ros2_control is enabled
- `joint_state_broadcaster`: publishes `/joint_states`
- forward command controllers such as `position_controller`
- `joint_state_publisher_gui`: only when ros2_control is disabled
- `rviz2`: with pre-configured display settings

## RViz2

For manual RViz2 setup:

1. Add **RobotModel**.
2. Set **Description Topic** to `/robot_description`.
3. Set **Fixed Frame** to the root link name.
4. Add **TF** to inspect coordinate frames.

## URDF Validation

```bash
# Check URDF syntax
check_urdf <robot>.urdf

# Visualize kinematic tree as PDF
urdf_to_graphviz <robot>.urdf
evince <robot>.pdf

# Print TF tree
ros2 run tf2_tools view_frames
```

## Xacro Processing

The export generates hierarchical xacro files. To process them manually:

```bash
xacro urdf/<robot>.urdf.xacro > /tmp/test.urdf
check_urdf /tmp/test.urdf
```

A preprocessed flat URDF is already included at `urdf/<robot>.urdf`.

## Topic Reference

| Topic | Type | Description |
| --- | --- | --- |
| `/joint_states` | `sensor_msgs/JointState` | Current joint positions |
| `/position_controller/commands` | `std_msgs/Float64MultiArray` | Position commands when enabled |
| `/velocity_controller/commands` | `std_msgs/Float64MultiArray` | Velocity commands when enabled |
| `/robot_description` | `std_msgs/String` | URDF XML parameter |
| `/tf` | `tf2_msgs/TFMessage` | Link transforms |
| `/tf_static` | `tf2_msgs/TFMessage` | Fixed link transforms |

Command arrays follow the joint order in `config/ros2_controllers.yaml`.
Generated hardware parameters also expose assembly-oriented bridge names
such as `sim/panther_system/cmd` and `sim/turret_system/state` for
downstream tools.

## Gazebo And Isaac Sim

The generated URDF is compatible with Gazebo and Isaac Sim. For Isaac
Sim, see [ISAAC_SIM.md](ISAAC_SIM.md) for the URDF-to-USD workflow and
how to use `robot_data.yaml`.

## Multi-Robot

The launch file supports both namespace and prefix arguments:

```bash
# Single robot
ros2 launch <robot>_description display.launch.py

# Namespaced topics and nodes
ros2 launch <robot>_description display.launch.py namespace:=robot1

# Prefixed URDF link/joint names and TF frames
ros2 launch <robot>_description display.launch.py prefix:=robot1_

# Both
ros2 launch <robot>_description display.launch.py namespace:=robot1 prefix:=robot1_
```

Use namespace for ROS graph isolation and prefix for URDF/TF name
isolation. For multiple robot instances, use both.
