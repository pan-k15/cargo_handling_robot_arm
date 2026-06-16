# arm6dof — 6-DOF Robot Arm Simulation

A 6-DOF robot arm with a 2-finger parallel gripper, built in ROS 2 Jazzy + Gazebo Harmonic.  
All geometry uses primitives (cylinders and boxes) — no mesh files required.

## Packages

| Package | Purpose |
|---|---|
| `robot_description` | URDF/xacro models, RViz config, standalone visualisation launch |
| `robot_simulation` | Gazebo worlds, ros2_control config, simulation launch files |
| `robot_interfaces` | Custom service definitions (`Pick.srv`, `Place.srv`) |
| `robot_services` | Pick and place service servers (C++) |
| `moveit_config` | MoveIt 2 configuration (SRDF, kinematics, controllers) |
| `robot_vision` | Object detection and pose estimation *(planned)* |

## Robot models

### `robot.urdf.xacro` — base model
6-DOF arm + parallel gripper. Used by `gazebo.launch.py`.

### `robot_2.urdf.xacro` — arm + depth camera
Extends the base model with an eye-in-hand RGBD camera mounted on `gripper_base`.
Used by `gazebo2.launch.py` and `cargo_sim.launch.py`.

```
world (fixed)
└── base_link          disc     r=0.075 m
    └── J1  revolute Z  ±180°   shoulder pan
        └── link1      cylinder r=0.05 h=0.10
            └── J2  revolute Y  ±90°    shoulder lift
                └── link2      box  0.05×0.05×0.30
                    └── J3  revolute Y  ±135°   elbow
                        └── link3      box  0.04×0.04×0.25
                            └── J4  revolute X  ±180°   wrist roll
                                └── link4      cylinder r=0.03 h=0.08
                                    └── J5  revolute Y  ±90°    wrist pitch
                                        └── link5      cylinder r=0.03 h=0.08
                                            └── J6  revolute Z  ±180°   wrist yaw
                                                └── link6      cylinder r=0.03 h=0.06
                                                    └── gripper_base (fixed)
                                                    │   ├── finger_left   prismatic +Y  0–30 mm
                                                    │   └── finger_right  prismatic −Y  0–30 mm (mimic)
                                                    └── camera_link (fixed, robot_2 only)
                                                        └── camera_optical_frame
```

## Controllers

| Controller | Type | Interface |
|---|---|---|
| `joint_state_broadcaster` | JointStateBroadcaster | all joints |
| `arm_controller` | JointTrajectoryController | joint1–6, position |
| `gripper_controller` | GripperActionController | finger_left_joint, position |

The gripper exposes a `control_msgs/action/GripperCommand` action at  
`/gripper_controller/gripper_cmd`.

## Services

| Service | Type | Description |
|---|---|---|
| `/pick` | `robot_interfaces/srv/Pick` | Open gripper → pre-grasp → grasp → close → lift |
| `/place` | `robot_interfaces/srv/Place` | Pre-place → descend → open gripper → retract |

Both services use a geometric 2R IK (elbow-down, joint4–6 = 0).  
TCP reach: **0 – 0.83 m** from joint2 (z = 0.15 m above ground).

## Dependencies

- ROS 2 Jazzy
- Gazebo Sim 8 (Harmonic)
- `gz_ros2_control`, `ros_gz_sim`, `ros_gz_bridge`
- `robot_state_publisher`, `joint_state_publisher_gui`
- `joint_trajectory_controller`, `gripper_controllers`
- `moveit_ros_move_group`, `moveit_simple_controller_manager`
- `rclcpp_action`, `control_msgs`

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select \
  robot_description robot_simulation \
  robot_interfaces robot_services \
  moveit_config
source install/setup.bash
```

> Build all packages at once with `colcon build` (omit `--packages-select`).

## Launch

### RViz only (joint slider GUI)

```bash
ros2 launch robot_description display.launch.py
```

### Gazebo — base model (no camera)

```bash
ros2 launch robot_simulation gazebo.launch.py
```

### Gazebo — robot_2 with depth camera

```bash
ros2 launch robot_simulation gazebo2.launch.py
```

Bridges `/camera/color/image_raw`, `/camera/depth/image_raw`,
`/camera/depth/points`, `/camera/color/camera_info`.

### Cargo handling simulation (recommended)

```bash
ros2 launch robot_simulation cargo_sim.launch.py
```

Launches Gazebo with `cargo_world.sdf` (pick table + 3 cargo boxes + place table),
`robot_2` model, camera bridges, all controllers, and pick/place service servers.
Everything starts automatically in the correct order.

### MoveIt move_group (requires simulation running)

```bash
ros2 launch moveit_config move_group.launch.py
```

> **Tip:** if stale `robot_state_publisher` or `move_group` processes from other
> projects interfere, kill them before launching:
> ```bash
> pkill -9 -f robot_state_publisher; pkill -9 -f move_group
> ```

## Commanding the arm

Send a joint trajectory (positions in radians):

```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [joint1,joint2,joint3,joint4,joint5,joint6],
    points: [{positions: [0.5,-0.5,1.0,0.0,0.5,0.0], time_from_start: {sec: 3}}]}'
```

Return to home:

```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [joint1,joint2,joint3,joint4,joint5,joint6],
    points: [{positions: [0.0,0.0,0.0,0.0,0.0,0.0], time_from_start: {sec: 3}}]}'
```

## Commanding the gripper

The gripper uses a `GripperCommand` action server:

```bash
# Open (30 mm)
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  '{command: {position: 0.03, max_effort: 50.0}}'

# Close (0 mm)
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  '{command: {position: 0.0, max_effort: 50.0}}'
```

## Pick and place services

```bash
# Pick an object at (0.5, 0.0, 0.3), approaching 10 cm from above
ros2 service call /pick robot_interfaces/srv/Pick \
  '{target_pose: {position: {x: 0.5, y: 0.0, z: 0.3}}, approach_height: 0.1}'

# Place it at (0.3, 0.4, 0.3)
ros2 service call /place robot_interfaces/srv/Place \
  '{target_pose: {position: {x: 0.3, y: 0.4, z: 0.3}}, approach_height: 0.1}'
```

**Pick sequence:** open gripper → pre-grasp → descend → close gripper → lift  
**Place sequence:** pre-place → descend → open gripper → retract

## Cargo world layout

`cargo_world.sdf` is designed so pick/place service defaults work out of the box.

| Object | Position (x, y, z) | Notes |
|---|---|---|
| Pick table top | (0.50, 0.00, 0.30) | Yellow zone marker |
| cargo_box_1 (red) | (0.45, −0.10, 0.33) | |
| cargo_box_2 (green) | (0.50, 0.00, 0.33) | Default `/pick` target |
| cargo_box_3 (blue) | (0.45, 0.10, 0.33) | |
| Place table top | (0.30, 0.45, 0.30) | White cross marker |

## Depth camera topics (robot_2)

| ROS topic | Type | Content |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB 640×480 @ 30 Hz |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Depth float32 (metres) |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | Organised point cloud |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics, frame id |

Camera frame: `camera_optical_frame` (Z forward, X right, Y down).  
FOV: 60°. Range: 0.05 – 5.0 m. Noise: Gaussian σ = 0.005 m.

## Joint limits

| Joint | Axis | Range | Max velocity |
|---|---|---|---|
| joint1 | Z | ±180° | π rad/s |
| joint2 | Y | ±90° | π rad/s |
| joint3 | Y | ±135° | π rad/s |
| joint4 | X | ±180° | π rad/s |
| joint5 | Y | ±90° | π rad/s |
| joint6 | Z | ±180° | π rad/s |
| finger_left_joint | Y | 0–30 mm | 0.1 m/s |
