# arm6dof — 6-DOF Robot Arm Simulation

A 6-DOF robot arm with a 2-finger parallel gripper, built in ROS 2 Jazzy + Gazebo Harmonic.  
All geometry uses primitives (cylinders and boxes) — no mesh files required.

## Packages

| Package | Purpose |
|---|---|
| `robot_description` | URDF/xacro model, RViz config, standalone visualisation launch |
| `robot_simulation` | Gazebo world, ros2_control config, full simulation launch |
| `robot_interfaces` | Custom service definitions (`Pick.srv`, `Place.srv`) |
| `robot_services` | Pick and place service servers (C++) |

## Robot structure

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
                                                        ├── finger_left   prismatic +Y  0–30 mm
                                                        └── finger_right  prismatic −Y  0–30 mm (mimic)
```

## Controllers

| Controller | Type | Interface |
|---|---|---|
| `joint_state_broadcaster` | JointStateBroadcaster | all joints |
| `arm_controller` | JointTrajectoryController | joint1–6, position |
| `gripper_controller` | ForwardCommandController | finger_left_joint, position |

## Services

| Service | Type | Description |
|---|---|---|
| `/pick` | `robot_interfaces/srv/Pick` | Move to pre-grasp → grasp → lift |
| `/place` | `robot_interfaces/srv/Place` | Move to pre-place → release → retract |

Both services use a geometric 2R IK (elbow-down, joint4–6 = 0).  
TCP reach: **0 – 0.83 m** from joint2 (z = 0.15 m above ground).

## Dependencies

- ROS 2 Jazzy
- Gazebo Sim 8 (Harmonic)
- `gz_ros2_control`, `ros_gz_sim`, `ros_gz_bridge`
- `robot_state_publisher`, `joint_state_publisher_gui`
- `joint_trajectory_controller`, `forward_command_controller`
- `rclcpp_action`, `control_msgs`

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select \
  robot_description robot_simulation robot_interfaces robot_services
source install/setup.bash
```

## Launch

### RViz only (with joint slider GUI)

```bash
ros2 launch robot_description display.launch.py
```

### Gazebo + RViz + controllers

```bash
ros2 launch robot_simulation gazebo.launch.py
```

Startup sequence: Gazebo → robot spawns → `joint_state_broadcaster` → `arm_controller` + `gripper_controller`.

> **Tip:** if stale `robot_state_publisher` processes from other projects interfere with `/robot_description`, run `pkill -9 -f robot_state_publisher` before launching.

### Pick and place servers (requires simulation running)

```bash
ros2 run robot_services pick_service_server &
ros2 run robot_services place_service_server &
```

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

The `ForwardCommandController` uses `BEST_EFFORT` QoS — publish with `--times` to ensure delivery:

```bash
# Open (30 mm)
ros2 topic pub -r 10 --times 20 /gripper_controller/commands \
  std_msgs/msg/Float64MultiArray '{data: [0.03]}'

# Close (0 mm)
ros2 topic pub -r 10 --times 20 /gripper_controller/commands \
  std_msgs/msg/Float64MultiArray '{data: [0.0]}'
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

## Joint limits

| Joint | Axis | Range |
|---|---|---|
| joint1 | Z | ±180° |
| joint2 | Y | ±90° |
| joint3 | Y | ±135° |
| joint4 | X | ±180° |
| joint5 | Y | ±90° |
| joint6 | Z | ±180° |
| finger_left_joint | Y | 0–30 mm |
