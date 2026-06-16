#!/usr/bin/env python3
"""
sort_orchestrator_v2 — pick-and-place node for two blocks on the table.

Flow
----
For each block in order (left → right):
  1. (optional, use_camera=true) Move arm to scan pose and wait for
     /object_class confirmation.
  2. Call /pick  to grasp the block.
  3. Call /place to drop the block into the bin.

World layout (robot arm at world origin):
  Block A : (0.50, -0.10, 0.15)  — left block
  Block B : (0.50, +0.10, 0.15)  — right block
  Drop bin: (0.35,  0.50, 0.07)
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

from robot_interfaces.srv import Pick, Place


# ── IK constants (must match pick_service_server.cpp) ────────────────
_JOINT2_Z   = 0.15
_L1         = 0.30
_L2         = 0.53
_ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

_SCAN_HEIGHT    = 0.10   # m above object for camera scan pose
_PICK_APPROACH  = 0.10   # m passed to /pick  service
_PLACE_APPROACH = 0.10   # m passed to /place service

# Two blocks on the pick table, one shared drop bin
_OBJECTS = [
    (0.50, -0.10, 0.15),   # Block A — left
    (0.50,  0.10, 0.15),   # Block B — right
]
_DROP = (0.35, 0.50, 0.07)


def _compute_ik(tx: float, ty: float, tz: float) -> list[float] | None:
    """Analytic 2-R elbow-down IK; returns None if unreachable."""
    j1 = math.atan2(ty, tx)
    r  = math.hypot(tx, ty)
    h  = tz - _JOINT2_Z
    d  = math.hypot(r, h)

    if d > _L1 + _L2 or d < abs(_L1 - _L2):
        return None

    c3 = (d * d - _L1 * _L1 - _L2 * _L2) / (2.0 * _L1 * _L2)
    s3 = -math.sqrt(max(0.0, 1.0 - c3 * c3))
    j3 = math.atan2(s3, c3)
    j2 = math.atan2(h, r) - math.atan2(_L2 * s3, _L1 + _L2 * c3)

    # URDF joint2 from vertical; standard 2R IK from horizontal.
    j2_urdf = math.pi / 2.0 - j2
    j3_urdf = -j3

    if not (0.0 <= j2_urdf <= math.pi / 2):
        return None
    if not (0.0 <= j3_urdf <= 3 * math.pi / 4):
        return None

    return [j1, j2_urdf, j3_urdf, 0.0, 0.0, 0.0]


class PickOrchestratorV2(Node):

    def __init__(self):
        super().__init__('pick_orchestrator_v2')

        self.declare_parameter('use_camera', False)

        cbg = ReentrantCallbackGroup()

        self._latest_class: str | None = None
        self._class_lock = threading.Lock()

        self._class_sub = self.create_subscription(
            String, '/object_class', self._class_cb, 10)

        self._arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=cbg)

        self._pick_client  = self.create_client(Pick,  '/pick',  callback_group=cbg)
        self._place_client = self.create_client(Place, '/place', callback_group=cbg)

        t = threading.Thread(target=self._delayed_start, daemon=True)
        t.start()

    # ── ROS callbacks ─────────────────────────────────────────────────

    def _class_cb(self, msg: String):
        with self._class_lock:
            self._latest_class = msg.data

    # ── Internal helpers ──────────────────────────────────────────────

    def _delayed_start(self):
        time.sleep(6.0)
        self.get_logger().info('Pick orchestrator v2: starting …')
        self._run_sequence()

    def _wait_for_class(self, timeout: float = 5.0) -> str | None:
        with self._class_lock:
            self._latest_class = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            time.sleep(0.05)
            with self._class_lock:
                if self._latest_class is not None:
                    return self._latest_class
        return None

    # ── Arm control ───────────────────────────────────────────────────

    def _send_arm(self, joints: list[float], duration_s: float = 3.5) -> bool:
        if not self._arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Arm action server unavailable')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = _ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = joints
        sec  = int(duration_s)
        nsec = int((duration_s - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        goal.trajectory.points.append(pt)

        future = self._arm_client.send_goal_async(goal)
        deadline = time.monotonic() + 15.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().error('Arm goal send timed out')
            return False

        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('Arm goal rejected')
            return False

        result_future = gh.get_result_async()
        deadline = time.monotonic() + duration_s + 15.0
        while not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not result_future.done():
            self.get_logger().error('Arm trajectory execution timed out')
            return False

        return result_future.result().result.error_code == 0

    def _move_to_scan(self, x: float, y: float, z: float) -> bool:
        joints = _compute_ik(x, y, z + _SCAN_HEIGHT)
        if joints is None:
            self.get_logger().warn(f'IK failed for scan above ({x:.2f},{y:.2f},{z:.2f})')
            return False
        return self._send_arm(joints, duration_s=3.5)

    # ── Service calls ─────────────────────────────────────────────────

    def _call_service_sync(self, client, request, timeout: float = 60.0):
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'Service {client.srv_name} not available')
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.result() if future.done() else None

    def _do_pick(self, x: float, y: float, z: float) -> bool:
        req = Pick.Request()
        req.target_pose = Pose()
        req.target_pose.position.x = float(x)
        req.target_pose.position.y = float(y)
        req.target_pose.position.z = float(z)
        req.target_pose.orientation.w = 1.0
        req.approach_height = float(_PICK_APPROACH)
        resp = self._call_service_sync(self._pick_client, req)
        if resp and resp.success:
            return True
        self.get_logger().warn(f'/pick failed: {resp.message if resp else "no response"}')
        return False

    def _do_place(self, x: float, y: float, z: float) -> bool:
        req = Place.Request()
        req.target_pose = Pose()
        req.target_pose.position.x = float(x)
        req.target_pose.position.y = float(y)
        req.target_pose.position.z = float(z)
        req.target_pose.orientation.w = 1.0
        req.approach_height = float(_PLACE_APPROACH)
        resp = self._call_service_sync(self._place_client, req)
        if resp and resp.success:
            return True
        self.get_logger().warn(f'/place failed: {resp.message if resp else "no response"}')
        return False

    # ── Main sequence ─────────────────────────────────────────────────

    def _run_sequence(self):
        use_camera = self.get_parameter('use_camera').get_parameter_value().bool_value
        dx, dy, dz = _DROP

        for i, (ox, oy, oz) in enumerate(_OBJECTS):
            self.get_logger().info(
                f'--- Block {i + 1}/{len(_OBJECTS)} at ({ox}, {oy}, {oz}) ---')

            if use_camera:
                self.get_logger().info(f'Scanning above ({ox:.2f}, {oy:.2f}, {oz:.2f}) …')
                if self._move_to_scan(ox, oy, oz):
                    label = self._wait_for_class(timeout=5.0)
                    if label:
                        self.get_logger().info(f'Camera sees: {label}')
                    else:
                        self.get_logger().warn('Camera timeout — proceeding with pick')
                else:
                    self.get_logger().warn('Could not reach scan pose — proceeding with pick')

            self.get_logger().info(f'Picking block {i + 1} …')
            if not self._do_pick(ox, oy, oz):
                self.get_logger().error(f'Pick failed for block {i + 1} — skipping')
                continue

            self.get_logger().info(f'Placing block {i + 1} at ({dx}, {dy}, {dz}) …')
            if not self._do_place(dx, dy, dz):
                self.get_logger().error(f'Place failed for block {i + 1}')
                continue

            self.get_logger().info(f'Block {i + 1} done.')

        self.get_logger().info('All blocks processed.')


def main(args=None):
    rclpy.init(args=args)
    node = PickOrchestratorV2()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
