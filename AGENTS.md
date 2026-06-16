# AGENT.md — arm6dof Robot Interface Reference

This file describes every ROS 2 interface an autonomous agent can use to perceive
and control the arm6dof robot. Start the full simulation before issuing any commands:

```bash
ros2 launch robot_simulation cargo_sim.launch.py
```

---

## Coordinate system

- World origin is at the centre of the robot base (ground level).
- X points forward (away from the robot's "front"), Y points left, Z points up.
- All positions are in **metres**, all angles in **radians**.

### Reachable workspace

| Parameter | Value |
|---|---|
| Shoulder height (joint2) | z = 0.15 m |
| Upper arm (L1) | 0.30 m |
| Forearm + wrist + TCP (L2) | 0.53 m |
| Max horizontal + vertical reach | 0.83 m from joint2 |
| Practical pick/place height | z = 0.25 – 0.45 m |
| Practical horizontal radius | 0.25 – 0.75 m |

The IK solver (geometric 2R, elbow-down) sets joint4–6 = 0.  
Targets outside this range return `success: false` from the pick/place services.

---

## Services

### `/pick` — `robot_interfaces/srv/Pick`

Grasps an object at the given Cartesian position.

**Request fields**

| Field | Type | Description |
|---|---|---|
| `target_pose.position.x` | float64 | Target X (metres) |
| `target_pose.position.y` | float64 | Target Y (metres) |
| `target_pose.position.z` | float64 | Target Z (metres) |
| `approach_height` | float64 | Pre-grasp offset above target (metres, typically 0.08–0.15) |

**Response fields**

| Field | Type | Description |
|---|---|---|
| `success` | bool | True if all motion steps completed |
| `message` | string | Human-readable status or error description |

**Execution sequence**
1. Open gripper (30 mm)
2. Move arm to pre-grasp: `(x, y, z + approach_height)` in 3 s
3. Descend to target: `(x, y, z)` in 2 s
4. Close gripper (0 mm)
5. Lift to pre-grasp height in 2 s

**Example**
```bash
ros2 service call /pick robot_interfaces/srv/Pick \
  '{target_pose: {position: {x: 0.5, y: 0.0, z: 0.3}}, approach_height: 0.1}'
```

---

### `/place` — `robot_interfaces/srv/Place`

Releases a held object at the given Cartesian position.

**Request / response fields**: same structure as `/pick`.

**Execution sequence**
1. Move arm to pre-place: `(x, y, z + approach_height)` in 3 s
2. Descend to target: `(x, y, z)` in 2 s
3. Open gripper (30 mm)
4. Retract to pre-place height in 2 s

**Example**
```bash
ros2 service call /place robot_interfaces/srv/Place \
  '{target_pose: {position: {x: 0.3, y: 0.4, z: 0.3}}, approach_height: 0.1}'
```

---

## Actions

### `/arm_controller/follow_joint_trajectory`
Type: `control_msgs/action/FollowJointTrajectory`

Send multi-point joint-space trajectories. Joints must be named exactly:
`[joint1, joint2, joint3, joint4, joint5, joint6]`

Joint limits: ±π for joint1/4/6, ±π/2 for joint2/5, ±3π/4 for joint3.

**Home position** (all zeros):
```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [joint1,joint2,joint3,joint4,joint5,joint6],
    points: [{positions: [0,0,0,0,0,0], time_from_start: {sec: 3}}]}'
```

### `/gripper_controller/gripper_cmd`
Type: `control_msgs/action/GripperCommand`

| Field | Range | Meaning |
|---|---|---|
| `command.position` | 0.0 – 0.03 m | Finger opening (0 = closed, 0.03 = fully open) |
| `command.max_effort` | > 0 N | Maximum gripping force |

```bash
# Open
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand '{command: {position: 0.03, max_effort: 50.0}}'

# Close
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand '{command: {position: 0.0, max_effort: 50.0}}'
```

---

## Topics — subscribed (inputs to robot)

| Topic | Type | Description |
|---|---|---|
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Direct joint trajectory (bypasses action feedback) |

---

## Topics — published (robot state)

| Topic | Type | Rate | Description |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 100 Hz | Position, velocity, effort for all joints |
| `/robot_description` | `std_msgs/String` | latched | URDF string |
| `/tf` | `tf2_msgs/TFMessage` | 100 Hz | All link transforms |
| `/tf_static` | `tf2_msgs/TFMessage` | latched | Fixed transforms (world→base, camera frames) |

---

## Topics — depth camera (robot_2 / cargo_sim only)

The eye-in-hand camera is mounted on `gripper_base`, tilted 45° downward toward the
workspace. Its data is available only when using `robot_2.urdf.xacro`.

| Topic | Type | Rate | Description |
|---|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 30 Hz | RGB image, 640×480, encoding `rgb8` |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 30 Hz | Depth image, 640×480, encoding `32FC1` (metres) |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | 30 Hz | Organised XYZRGB point cloud |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | Intrinsics; frame `camera_optical_frame` |

Camera spec: 60° horizontal FOV, 0.05 – 5.0 m range, Gaussian depth noise σ = 0.005 m.

---

## Cargo world — named pick targets

These positions are pre-validated against the IK solver and placed on the cargo
table in `cargo_world.sdf`. All boxes are 60×60×60 mm cubes.

| Name | x | y | z | Colour |
|---|---|---|---|---|
| `cargo_box_1` | 0.45 | −0.10 | 0.33 | Red |
| `cargo_box_2` | 0.50 | 0.00 | 0.33 | Green |
| `cargo_box_3` | 0.45 | 0.10 | 0.33 | Blue |
| Place zone | 0.30 | 0.45 | 0.30 | — |

Recommended `approach_height`: **0.10 m** for all targets.

---

## Joint state reading

```bash
ros2 topic echo /joint_states --once
```

Field order in `name[]` / `position[]` matches declaration order:
`[joint1, joint2, joint3, joint4, joint5, joint6, finger_left_joint, finger_right_joint]`

---

## MoveIt 2 interface

When `move_group.launch.py` is running alongside the simulation:

| Interface | Details |
|---|---|
| Move group name (arm) | `arm` |
| Move group name (gripper) | `gripper` |
| Planner | KDL (default), Pilz (LIN/CIRC) |
| Arm home state | `home` (all joints = 0) |
| Gripper states | `open` (0.03 m), `close` (0.0 m) |
| Action server | `/move_action` |

Acceleration limits are defined in `moveit_config/config/joint_limits.yaml`
(arm: 5 rad/s², gripper: 0.5 rad/s²).

---

## Error handling

| Error | Likely cause | Fix |
|---|---|---|
| `IK failed: * pose unreachable` | Target outside 0.83 m reach | Move target closer to robot, reduce height extremes |
| `Arm action server not available` | `arm_controller` not active | Wait for controller spawn to complete or relaunch |
| `Gripper action server not available` | `gripper_controller` not active | Same as above |
| `Arm trajectory timed out` | Motion took > 30 s | Check for joint limit violations or collisions |
| `Gripper command timed out` | Gripper stalled or controller crashed | Check `/joint_states` for finger position |
| `j2 out of range` | Target too high or too low | Adjust z so h = z − 0.15 stays within ±0.30 m |

---

## Quick-start: pick and place a box

```bash
# 1. Launch
ros2 launch robot_simulation cargo_sim.launch.py

# 2. Wait ~15 s for controllers to activate, then:

# 3. Pick the green box
ros2 service call /pick robot_interfaces/srv/Pick \
  '{target_pose: {position: {x: 0.5, y: 0.0, z: 0.3}}, approach_height: 0.1}'

# 4. Place it on the blue table
ros2 service call /place robot_interfaces/srv/Place \
  '{target_pose: {position: {x: 0.3, y: 0.4, z: 0.3}}, approach_height: 0.1}'
```

Both calls block until the motion sequence completes (≈ 10 s each).
