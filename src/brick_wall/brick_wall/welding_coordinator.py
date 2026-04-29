#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# --- CLASS 2: THE MISSION BRAIN ---
class WeldingCoordinator(Node):
    def __init__(self):
        super().__init__("welding_coordinator")

        # Action client for MoveGroup
        self.move_group = ActionClient(self, MoveGroup, "move_group")
        self.arm_traj_pub = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10,
        )

        # Values from your screenshot
        self.poses = {
            "pose1": [0.0, -1.0, -1.15, 0.0, -1.0, 0.0],
            "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
        self.joints = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.handled_triggers = set()
        self.server_ready = False
        self.warned_movegroup_fallback = False
        self.pending_direct_pose = None

        trigger_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(
            String,
            "/navigator/position_trigger",
            self._on_position_trigger,
            trigger_qos,
        )

        self.get_logger().info("Waiting for MoveGroup server...")
        self.create_timer(0.5, self._check_move_group_ready)
        self.create_timer(0.2, self._flush_pending_direct_pose)
        self.get_logger().info(
            "Welding coordinator ready. Waiting for position triggers."
        )

    def _check_move_group_ready(self):
        if self.server_ready:
            return
        if self.move_group.wait_for_server(timeout_sec=0.0):
            self.server_ready = True
            self.get_logger().info("MoveGroup server connected.")

    def send_arm_pose(self, pose_name):
        if not self.server_ready:
            if not self.warned_movegroup_fallback:
                self.get_logger().warn(
                    "MoveGroup not ready. Using direct /arm_controller/joint_trajectory fallback."
                )
                self.warned_movegroup_fallback = True
            self.pending_direct_pose = pose_name
            self._flush_pending_direct_pose()
            return

        self.get_logger().info(f"Arm Target: {pose_name}")
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        constraints = Constraints()
        for name, val in zip(self.joints, self.poses[pose_name]):
            jc = JointConstraint(joint_name=name, position=val, weight=1.0)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)
        self.move_group.send_goal_async(goal)

    def _flush_pending_direct_pose(self):
        if self.pending_direct_pose is None:
            return

        sub_count = self.count_subscribers("/arm_controller/joint_trajectory")
        if sub_count == 0:
            return

        pose_name = self.pending_direct_pose
        self.pending_direct_pose = None
        self._send_direct_joint_trajectory(pose_name)

    def _send_direct_joint_trajectory(self, pose_name):
        sub_count = self.count_subscribers("/arm_controller/joint_trajectory")
        if sub_count == 0:
            self.get_logger().warn(
                "No subscribers on /arm_controller/joint_trajectory. arm_controller may be inactive."
            )
            self.pending_direct_pose = pose_name
            return

        self.get_logger().info(f"Arm Target (direct): {pose_name}")
        traj = JointTrajectory()
        traj.joint_names = self.joints

        point = JointTrajectoryPoint()
        point.positions = self.poses[pose_name]
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0

        traj.points = [point]
        # Publish a few times to survive startup races and transient overrides.
        for _ in range(3):
            self.arm_traj_pub.publish(traj)

    def _parse_trigger(self, payload):
        parts = {}
        for chunk in payload.split(";"):
            if "=" not in chunk:
                continue
            key, value = chunk.split("=", 1)
            parts[key.strip()] = value.strip()
        return parts

    def _on_position_trigger(self, msg):
        data = self._parse_trigger(msg.data)
        label = data.get("label")
        direction = data.get("dir", "unknown").lower()

        if not label:
            self.get_logger().warn(f"Ignored malformed trigger: {msg.data}")
            return

        if label in self.handled_triggers:
            return

        if label == "plus_30_3":
            # At (30, 3), always move manipulator to home.
            self.send_arm_pose("home")
            self.handled_triggers.add(label)
            return

        if label == "minus_30_3":
            # At (-30, 3): east/west -> pose1, north/south -> home.
            if direction in ("east", "west"):
                self.send_arm_pose("pose1")
            else:
                self.send_arm_pose("home")
            self.handled_triggers.add(label)
            return

        if label in ("plus_30_m2_8", "minus_30_m2_8"):
            # Bottom row trigger: east/west -> pose1, north/south -> home.
            if direction in ("east", "west"):
                self.send_arm_pose("pose1")
            else:
                self.send_arm_pose("home")
            self.handled_triggers.add(label)
            return

        self.get_logger().info(f"Unhandled trigger label: {label}")


def main(args=None):
    rclpy.init(args=args)

    coordinator = WeldingCoordinator()

    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
