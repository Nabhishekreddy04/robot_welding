import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return yaw from quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class TurnByAngleNode(Node):
    def __init__(self) -> None:
        super().__init__("turn_by_angle")

        self.declare_parameter("angle_deg", 90.0)
        self.declare_parameter("angular_speed", 0.6)
        self.declare_parameter("tolerance_deg", 1.0)

        self.target_angle = math.radians(float(self.get_parameter("angle_deg").value))
        self.angular_speed = abs(float(self.get_parameter("angular_speed").value))
        self.tolerance = math.radians(float(self.get_parameter("tolerance_deg").value))

        self.start_yaw = None
        self.current_yaw = None
        self.done = False

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.timer = self.create_timer(0.02, self.control_cb)

        self.get_logger().info(
            f"Turning by {math.degrees(self.target_angle):.1f} deg at {self.angular_speed:.2f} rad/s"
        )

    def odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.current_yaw = yaw
        if self.start_yaw is None:
            self.start_yaw = yaw

    def stop_robot(self) -> None:
        self.cmd_pub.publish(Twist())

    def control_cb(self) -> None:
        if self.done:
            return

        if self.start_yaw is None or self.current_yaw is None:
            return

        turned = normalize_angle(self.current_yaw - self.start_yaw)
        remaining = normalize_angle(self.target_angle - turned)

        if abs(remaining) <= self.tolerance:
            self.stop_robot()
            self.done = True
            self.get_logger().info(
                f"Turn complete. Actual angle: {math.degrees(turned):.2f} deg"
            )
            self.destroy_node()
            rclpy.shutdown()
            return

        cmd = Twist()
        cmd.angular.z = self.angular_speed if remaining > 0.0 else -self.angular_speed

        # Slow down near the target to reduce overshoot.
        if abs(remaining) < math.radians(15.0):
            cmd.angular.z *= 0.35

        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TurnByAngleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
