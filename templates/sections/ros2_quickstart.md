```bash
# 1. Copy package to your ROS 2 workspace
cp -r {{package_name}} ~/ros2_ws/src/

# 2. Build
cd ~/ros2_ws
colcon build --packages-select {{package_name}}
source install/setup.bash

# 3. Visualize in RViz2
ros2 launch {{package_name}} display.launch.py

# 4. Validate URDF structure
check_urdf install/{{package_name}}/share/{{package_name}}/urdf/{{robot_name}}.urdf

# 5. Print kinematic tree
urdf_to_graphviz install/{{package_name}}/share/{{package_name}}/urdf/{{robot_name}}.urdf
```

**Joint control**: The launch file includes `joint_state_publisher_gui` —
use the sliders to move revolute/prismatic joints in RViz2.

**Topic inspection**:
```bash
# See published joint states
ros2 topic echo /joint_states

# See robot description parameter
ros2 param get /robot_state_publisher robot_description
```
