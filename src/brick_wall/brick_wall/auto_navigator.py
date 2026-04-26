#!/usr/bin/env python3

import math
import time
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class AutoNavigator(Node):

    def __init__(self):
        super().__init__("auto_navigator")

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_ready = False

        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(0.5, self._check_ready)

        self.get_logger().info("Waiting for odometry...")

    # ───────────── ODOM ─────────────
    def _on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

        self.odom_ready = True

    def _check_ready(self):
        if self.odom_ready:
            self.timer.cancel()
            self.get_logger().info(
                f"Start at ({self.x:.2f}, {self.y:.2f})"
            )
            threading.Thread(target=self._run, daemon=True).start()

    # ───────────── MISSION ─────────────
    def _run(self):
        time.sleep(2.0)

        self.get_logger().info("=== MISSION START ===")

        # 1️⃣ MOVE EAST (Y = 3 fixed)
        self._drive_straight_x(target_x=35.0, target_y=3.0)

        # 2️⃣ TURN SOUTH
        self._turn_to(-math.pi / 2)

        # 3️⃣ MOVE SOUTH (X = 35 fixed)
        self._drive_straight_y(target_x=35.0, target_y=-3.0)

        # 4️⃣ TURN WEST
        self._turn_to(math.pi)
        time.sleep(1.0)
        self._align_y(-3.0)

        # 5️⃣ MOVE WEST (Y = -3 fixed)
        self._drive_straight_x(target_x=-35.0, target_y=-3.0)

        self.get_logger().info("=== DONE ===")
        self._stop()

    def _align_y(self, target_y):
        self.get_logger().info(f"Aligning to Y={target_y}")

        while rclpy.ok():
            error = target_y - self.y

            if abs(error) < 0.05:
                break

            t = Twist()

            # move slowly forward while correcting
            t.linear.x = 0.1
            t.angular.z = -3.0 * error

            self.vel_pub.publish(t)
            time.sleep(0.05)

        self._stop()
        time.sleep(0.5)

    # ───────────── STRAIGHT MOTION ─────────────

    def _drive_straight_x(self, target_x, target_y):
        self.get_logger().info(f"Driving to X={target_x}")

        while rclpy.ok():
            dx = target_x - self.x
            dy = target_y - self.y

            if abs(dx) < 0.3:
                break

            t = Twist()

            # ✅ ALWAYS MOVE FORWARD
            t.linear.x = 0.5

            # 🔥 determine direction (EAST or WEST)
            desired_yaw = 0.0 if dx > 0 else math.pi

            # heading correction
            yaw_error = self._angle_diff(desired_yaw, self.yaw)

            # lateral correction (stay on line)
            error_y = dy

            t.angular.z = (2.0 * yaw_error) - (2.5 * error_y)

            t.angular.z = max(-1.0, min(1.0, t.angular.z))

            self.vel_pub.publish(t)
            time.sleep(0.05)

        self._stop()
        time.sleep(1.0)

    def _drive_straight_y(self, target_x, target_y):
        self.get_logger().info(f"Driving NORTH/SOUTH to {target_y}")

        while rclpy.ok():
            dx = target_x - self.x
            dy = target_y - self.y

            if abs(dy) < 0.3:
                break

            t = Twist()

            # forward
            t.linear.x = 0.5

            # 🔥 KEY: keep X locked
            error_x = dx
            t.angular.z = 2.5 * error_x

            t.angular.z = max(-1.0, min(1.0, t.angular.z))

            self.vel_pub.publish(t)
            time.sleep(0.05)

        self._stop()
        time.sleep(1.0)

    # ───────────── TURN ─────────────

    def _turn_to(self, target_yaw):
        self.get_logger().info(f"Turning to {math.degrees(target_yaw):.1f}°")

        while rclpy.ok():
            err = self._angle_diff(target_yaw, self.yaw)

            if abs(err) < 0.05:
                break

            t = Twist()
            t.angular.z = 1.5 if err > 0 else -1.5

            self.vel_pub.publish(t)
            time.sleep(0.02)

        self._stop()
        time.sleep(1.0)

    # ───────────── HELPERS ─────────────

    def _angle_diff(self, a, b):
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    def _stop(self):
        self.vel_pub.publish(Twist())
        time.sleep(0.1)


def main():
    rclpy.init()
    node = AutoNavigator()
    rclpy.spin(node)


if __name__ == "__main__":
    main()