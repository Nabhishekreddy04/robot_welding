from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointStateGuiBridge(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_gui_bridge")

        self.arm_joints: List[str] = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]

        self.last_positions: Dict[str, float] = {name: 0.0 for name in self.arm_joints}
        self.publish_epsilon = 1e-4

        self.sub = self.create_subscription(
            JointState,
            "/joint_states_gui",
            self.joint_state_cb,
            10,
        )
        self.pub = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10,
        )

        self.get_logger().info(
            "Bridge active: /joint_states_gui -> /arm_controller/joint_trajectory"
        )

    def joint_state_cb(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return

        changed = False
        for idx, name in enumerate(msg.name):
            if name in self.last_positions and idx < len(msg.position):
                new_pos = float(msg.position[idx])
                if abs(new_pos - self.last_positions[name]) > self.publish_epsilon:
                    self.last_positions[name] = new_pos
                    changed = True

        # Avoid spamming the controller with identical GUI values,
        # which can override autonomous arm commands.
        if not changed:
            return

        traj = JointTrajectory()
        traj.joint_names = self.arm_joints

        point = JointTrajectoryPoint()
        point.positions = [self.last_positions[name] for name in self.arm_joints]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 250000000  # 0.25s

        traj.points = [point]
        self.pub.publish(traj)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateGuiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
