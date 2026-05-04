# Getting Started

Everything you need to go from "I cloned this" to "I have a robot bouncing around in RViz".

## Requirements

- **Fusion 360** (any license, Windows or macOS)
- **Python 3.9+** (Fusion ships its own interpreter, you don't have to install one)
- **ROS 2 Jazzy** for launching the generated package (not needed for export). Tested against Jazzy; earlier distros likely work but are unverified.

## Installation

Three install paths. Pick one. They all end with the same thing: Fusion sees a `fusion2URDF` script in **My Scripts**.

### Option A: Add the script through the Fusion GUI

The lowest-effort path, recommended if you just want to use the exporter.

1. Clone or download this repository somewhere on disk.
2. In Fusion 360: **Utilities -> Scripts and Add-Ins** (or `Shift+S`).
3. Open the **Scripts** tab. Click the green **+** next to **My Scripts**.
4. Browse to the cloned `fusion2URDF/` folder and pick it.
5. Select `fusion2URDF` in the list and click **Run**.

Autodesk's official walkthrough is at [Manage scripts and add-ins][fusion-scripts] if you want screenshots.

### Option B: Symlink the repo into Fusion's scripts directory

Recommended for development. Edits in your git checkout are picked up by Fusion immediately, no copy step.

**Windows (PowerShell)** — run from inside the cloned `fusion2URDF/` folder:

```powershell
$dest = "$env:APPDATA\Autodesk\Autodesk Fusion 360\API\Scripts\fusion2URDF"
$src  = (Get-Location).Path

# Remove an existing copy or link, if any.
if (Test-Path $dest) { Remove-Item $dest -Recurse -Force }

New-Item -ItemType SymbolicLink -Path $dest -Target $src
```

`SymbolicLink` needs either an elevated PowerShell **or** Windows Developer Mode enabled (Settings -> Privacy & Security -> For developers -> Developer Mode). If neither is available, swap `SymbolicLink` for `Junction`, which works without elevation:

```powershell
New-Item -ItemType Junction -Path $dest -Target $src
```

A junction behaves the same way for Fusion's purposes.

**macOS (terminal)** — run from inside the cloned `fusion2URDF/` folder:

```bash
DEST="$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts/fusion2URDF"
mkdir -p "$(dirname "$DEST")"
[ -e "$DEST" ] && rm -rf "$DEST"
ln -s "$(pwd)" "$DEST"
```

After symlinking, restart Fusion or refresh the Scripts dialog. `fusion2URDF` should now appear under **My Scripts**.

### Option C: Copy the repo into Fusion's scripts directory

Use this if you want a static, self-contained copy that won't change when you `git pull`.

**Windows (PowerShell)** — run from inside the cloned `fusion2URDF/` folder:

```powershell
$dest = "$env:APPDATA\Autodesk\Autodesk Fusion 360\API\Scripts\fusion2URDF"
if (Test-Path $dest) { Remove-Item $dest -Recurse -Force }
Copy-Item -Path . -Destination $dest -Recurse
```

**macOS (terminal):**

```bash
DEST="$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts/fusion2URDF"
mkdir -p "$(dirname "$DEST")"
rm -rf "$DEST"
cp -R . "$DEST"
```

## Optional Persistent Config

The exporter runs without a config file. Defaults are sensible. If you find yourself clicking through the same dialog choices every export, copy the template:

```bash
cp xacro_export.template.toml xacro_export.toml
```

```powershell
Copy-Item xacro_export.template.toml xacro_export.toml
```

`xacro_export.toml` is gitignored. The plugin reads it on every run and uses it to set dialog defaults or skip dialogs entirely for config-only options like `convex_hull` collision.

### Settings reference

```toml
[output]
# "verbose" ships everything (debug folder, transforms.md, README, RViz, screenshot).
# "minimal" drops debug/, docs/, robot_data.yaml, images/, rviz/, README.md
# and keeps just URDF + meshes + package.xml + CMakeLists + one launch file.
verbosity = "verbose"

# Wrap the generated package directory in a .zip and remove the unzipped
# directory. Convenient when shipping a description over a network or to
# a tool that wants one file instead of a tree.
zip = false
zip_name = ""        # empty = "<package_name>.zip"

[features]
# Per-output toggles, layered on top of the verbosity preset. Comment a
# line to fall back to the preset.
include_debug = true            # debug/ folder (snapshot, transforms, log)
include_docs = true             # docs/transforms.md
include_robot_data_yaml = true  # robot_data.yaml supplementary file
include_screenshot = true       # images/robot.png from the Fusion viewport
include_launch = true           # launch/display.launch.py
include_rviz = true             # rviz/display.rviz + config/joint_state.yaml
include_readme = true           # auto-generated package README
include_ros2_control = true     # ros2_control XML + config/ros2_controllers.yaml

[mesh]
# "dae" is a single-file COLLADA in meters, scale="1" in URDF. Friendliest
# for Gazebo / RViz / Isaac Sim.
# "obj" is Fusion-native OBJ + MTL in centimeters, scale="0.01" in URDF.
visual_format = "dae"

# Default collision when a link has no explicit !collision_* member.
# "primitive"    fits a box, cylinder, or sphere from the bounding box.
# "convex_hull"  generates a convex STL from the visual mesh vertices.
# "visual_reuse" uses the visual mesh as collision (heavy but exact).
collision_method = "primitive"

# Mesh resolution from Fusion: "low", "medium", "high".
mesh_refinement = "medium"

[ros2_control]
# Generic mock hardware lets controller_manager start without custom drivers.
# Replace with your real plugin when wiring real hardware.
hardware_plugin = "mock_components/GenericSystem"
update_rate = 100
# Comma-separated command interfaces generated for each movable, non-passive
# joint. State interfaces (position/velocity/effort) are always emitted for
# joint_state_broadcaster.
command_interfaces = "position,velocity"
```

## First Export

1. Open a Fusion 360 design with joints defined.
2. Run the script from **Utilities -> Scripts and Add-Ins -> My Scripts**.
3. The exporter shows an extraction summary: occurrences, joints, depth. Click **Yes** to continue, **No** to abort and fix something in Fusion first.
4. Pick an output folder.
5. **Primary collision dialog** (skipped if `[mesh].collision_method` is set in your TOML):
   - **Yes**: auto-fit primitive box/cylinder/sphere collision for every link.
   - **No**: reuse the visual mesh as collision.
6. **Secondary collision dialog** appears only when the design contains explicit `!collision_*` geometry:
   - After **Yes** (primitive): **Yes** keeps your explicit collision where it exists and uses primitives elsewhere; **No** overrides explicit geometry with primitives everywhere.
   - After **No** (visual): **Yes** uses visual mesh for every link; **No** keeps explicit STL where you designed it and visual mesh for the rest.

The script generates a complete ROS 2 description package, ready for `colcon build`. Fusion's progress is non-blocking: a small trivia window pops up so you have something to do while heavy designs export.

For the special-prefix conventions (`!frame_*`, `!collision_*`, `!dummy_*`, `!passive_*`, `!closing_*`, `!acc_*`, `!cxh_*`, `!pri_*`), see [DESIGN_RULES.md](../DESIGN_RULES.md). Bare names without `!` are ordinary names.

> Hidden bodies and occurrences in Fusion are omitted from the exported visual mesh. Keep your robot geometry visible unless you intentionally want it left out.

## Output Structure

```text
<output_dir>/
  <robot>_description/
    urdf/
      <robot>.urdf.xacro              # entry point
      <robot>.urdf                    # flat URDF for validation
      assemblies/<asm>.urdf.xacro     # per-assembly macros
    meshes/
      <assembly>/<link>.dae           # visual (DAE or OBJ+MTL)
      <assembly>/<link>_collision.stl # collision (where generated)
    launch/display.launch.py          # RViz + ros2_control bringup
    config/joint_state.yaml
    config/ros2_controllers.yaml
    rviz/display.rviz
    robot_data.yaml                   # supplementary data beyond URDF
    docs/transforms.md                # KaTeX joint transforms
    images/robot.png                  # Fusion viewport screenshot
    README.md                         # auto-generated package README
    package.xml
    CMakeLists.txt
    debug/                            # only when include_debug = true
      snapshot.json
      extraction_report.md
      validation.md
      export_log.md
```

Optional files depend on `xacro_export.toml` feature toggles and `[output].verbosity`.

## Build and launch

From any colcon workspace that contains `<robot>_description/`:

```bash
colcon build --packages-select <robot>_description
source install/setup.bash
ros2 launch <robot>_description display.launch.py
```

RViz opens with the robot, joint state publisher, and `ros2_control` already wired up. `/joint_states` publishes immediately. Movable, non-passive joints accept commands on `/<controller_name>/commands` as `std_msgs/msg/Float64MultiArray`.

## Editor Integration (VS Code)

The repo ships a pre-wired `.vscode/settings.json` for the [URDF Visualizer extension][urdf-viz]. Install the extension, reload VS Code, and the `URDF Visualizer` panel picks up the bundled exports under `examples/` automatically. This is a fast way to spot-check joint axes, origin RPY, mesh references, and collision placement without a full ROS 2 build.

To add your own exports, extend the `urdf-visualizer.packages` map:

```jsonc
{
  "urdf-visualizer.packages": {
    "my_robot_description": "${workspaceFolder}/path/to/my_robot_description"
  }
}
```

The same URDFs can be validated from the terminal:

```bash
python scripts/validate_examples.py examples/
```

This is the same check CI runs: XML well-formedness, mesh reference resolution, and collision STL bounding-box sanity.

[fusion-scripts]: https://help.autodesk.com/cloudhelp/ENU/Fusion-Model/files/SLD-MANAGE-SCRIPTS-ADD-INS.htm
[urdf-viz]: https://marketplace.visualstudio.com/items?itemName=morningfrog.urdf-visualizer