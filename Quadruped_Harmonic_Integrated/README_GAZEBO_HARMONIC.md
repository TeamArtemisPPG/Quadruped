# Quadruped — Gazebo Harmonic integration

This keeps the original `quad` ROS 2 package and adds the Gazebo Harmonic setup from
`harmonic_quadruped_tester`.

## Target environment
- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic

## Install dependencies

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-controller-manager
```

## Build

From this extracted workspace root (the folder containing `src/`):

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Launch in Gazebo Harmonic

```bash
ros2 launch quad gz_harmonic.launch.py
```

Wait for the robot to spawn and for `joint_state_broadcaster` and
`joint_group_controller` to become active. The launch file sends an initial
stance command once the controller spawner exits.

Check:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

## Make it walk

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source <workspace-root>/install/setup.bash
ros2 run quad walk_g
```

Use the terminal running `walk_g`:
- `f` — forward
- `b` — backward
- `l` — left
- `r` — right
- `s` — stop/stance

The gait code publishes 12 joint positions to
`/joint_group_controller/commands`. `walk_g` defaults to simulation mode.
The old RViz launch is still available:

```bash
ros2 launch quad display.launch.py
```

## Troubleshooting

- If the robot does not spawn, inspect the `gz_harmonic.launch.py` terminal for
  URDF/plugin errors.
- If `/controller_manager` services are absent, verify that
  `gz_ros2_control` is installed and that the plugin loaded.
- If controllers are inactive, check `ros2 control list_controllers`.
- If the robot spawns but tips/falls, this is a physics/gait tuning issue rather
  than an RViz display issue. Start with a small step and inspect joint limits,
  collision geometry, inertias, and foot contact.

The integrated files have been merged, but should be tested on the target Ubuntu
installation because this environment cannot run ROS 2 / Gazebo Harmonic.
