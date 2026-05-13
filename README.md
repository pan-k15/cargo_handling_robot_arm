# arm6dof — 6-DOF Robot Arm Simulation

A 6-DOF robot arm with a 2-finger parallel gripper, built in ROS 2 Jazzy + Gazebo Harmonic.  
All geometry uses primitives (cylinders and boxes) — no mesh files required.

## Packages

| Package | Purpose |
|---|---|
| `robot_description` | URDF/xacro model, RViz config, standalone visualisation launch |
| `robot_simulation` | Gazebo world, ros2_control config, full simulation launch |

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

## Dependencies

- ROS 2 Jazzy
- Gazebo Sim 8 (Harmonic)
- `gz_ros2_control`, `ros_gz_sim`, `ros_gz_bridge`
- `robot_state_publisher`, `joint_state_publisher_gui`
- `joint_trajectory_controller`, `forward_command_controller`

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select robot_description robot_simulation
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

Startup sequence: Gazebo loads → robot spawns → `joint_state_broadcaster` activates → `arm_controller` + `gripper_controller` activate.

> **Tip:** if you have stale `robot_state_publisher` processes from other projects interfering with `/robot_description`, run `pkill -9 -f robot_state_publisher` before launching.

## Commanding the arm

Send a joint trajectory (positions in radians, 3-second duration):

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
