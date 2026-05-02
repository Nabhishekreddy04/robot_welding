#!/usr/bin/env python3

import math
import time
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


class AutoNavigator(Node):

    def __init__(self):
        super().__init__("auto_navigator")

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_ready = False

        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        trigger_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.trigger_pub = self.create_publisher(
            String, "/navigator/position_trigger", trigger_qos
        )

        self._triggered_labels = set()
        self._trigger_tolerance = 0.7

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
        self._publish_position_triggers()

    def _publish_position_triggers(self):
        # Publish each trigger point once per run to avoid retrigger spam.
        self._maybe_publish_trigger("minus_30_3", -30.0, 3.0)
        self._maybe_publish_trigger("plus_30_3", 35.0, 3.0)
        self._maybe_publish_trigger("plus_30_m2_8", 35.0, -3.0)
        self._maybe_publish_trigger("minus_30_m2_8", -30.0, -3.0)

    def _maybe_publish_trigger(self, label, tx, ty):
        if label in self._triggered_labels:
            return

        if abs(self.x - tx) > self._trigger_tolerance:
            return
        if abs(self.y - ty) > self._trigger_tolerance:
            return

        # Decide manipulator action based on current heading.
        # If the robot is turning to NORTH/SOUTH, only publish the "home"
        # manipulator command after the heading has actually reached north/south
        # (prevents manipulator moving too early). For EAST/WEST publish
        # immediately with pose1.
        direction = self._current_direction()

        # For north/south require that yaw has reached near the cardinal angle
        if direction in ("north", "south"):
            target_yaw = math.pi / 2 if direction == "north" else -math.pi / 2
            yaw_err = abs(self._angle_diff(target_yaw, self.yaw))
            # 20 degrees tolerance
            if yaw_err > math.radians(20.0):
                return

        manip_action = "home" if direction in ("north", "south") else "pose1"

        msg = String()
        msg.data = f"label={label};x={self.x:.2f};y={self.y:.2f};dir={direction};manip={manip_action}"
        self.trigger_pub.publish(msg)
        self._triggered_labels.add(label)
        self.get_logger().info(f"Published trigger: {msg.data}")

        # Special-case: when arriving at the far-east-then-south corner
        # (`plus_30_m2_8`), if we published `home` (because heading was
        # north/south at arrival) schedule a follow-up: when the robot
        # later turns to `west`, wait 2s and publish `pose1` so the
        # manipulator returns from `home` to `pose1`.
        if label == "plus_30_m2_8" and manip_action == "home":
            threading.Thread(
                target=self._wait_for_west_then_publish_pose1,
                args=(label,),
                daemon=True,
            ).start()

    def _current_direction(self):
        yaw_deg = math.degrees(self.yaw)
        # Normalize to [-180, 180]
        while yaw_deg > 180.0:
            yaw_deg -= 360.0
        while yaw_deg < -180.0:
            yaw_deg += 360.0

        if -45.0 <= yaw_deg <= 45.0:
            return "east"
        if yaw_deg >= 135.0 or yaw_deg <= -135.0:
            return "west"
        if 45.0 < yaw_deg < 135.0:
            return "north"
        return "south"

    def _check_ready(self):
        if self.odom_ready:
            self.timer.cancel()
            self.get_logger().info(f"Start at ({self.x:.2f}, {self.y:.2f})")
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
        # Ensure manipulator moves to pose1 for the upcoming WESTward drive.
        # Publish a recognized trigger (reuse `plus_30_m2_8`) with manip=pose1
        # after a short delay so the arm has time to move before driving.
        try:
            time.sleep(2.0)
            msg = String()
            msg.data = (
                f"label=plus_30_m2_8;x={self.x:.2f};y={self.y:.2f};dir=west;manip=pose1"
            )
            self.trigger_pub.publish(msg)
            self.get_logger().info(f"Published pose1 trigger for west leg: {msg.data}")
        except Exception:
            pass
        # self._align_y(-3.0
        # 5️⃣ MOVE WEST (Y = -3 fixed)
        self._drive_straight_x(target_x=-35.0, target_y=-3.0)

        self.get_logger().info("=== DONE ===")
        self._stop()

    # ───────────── STRAIGHT MOTION ─────────────

    def _drive_straight_x(self, target_x, target_y):
        self.get_logger().info(f"Driving to X={target_x}")

        log_interval = 10  # Log every 10 iterations (0.5 seconds)
        iteration = 0

        while rclpy.ok():
            dx = target_x - self.x
            dy = target_y - self.y

            if abs(dx) < 0.3:
                break

            t = Twist()

            # ✅ ALWAYS MOVE FORWARD, but slower so the controller can hold
            # the row tightly for gap detection / welding accuracy.
            t.linear.x = 0.25 if abs(dy) > 0.05 else 0.15

            # 🔥 determine direction (EAST or WEST)
            desired_yaw = 0.0 if dx > 0 else math.pi

            # heading correction
            yaw_error = self._angle_diff(desired_yaw, self.yaw)

            # lateral correction (stay on line)
            error_y = dy

            # The row position is critical for welding, so y-error gets a much
            # stronger correction than heading error.
            t.angular.z = (3.0 * yaw_error) - (8.0 * error_y)

            t.angular.z = max(-1.0, min(1.0, t.angular.z))

            self.vel_pub.publish(t)

            # Log position every log_interval iterations
            iteration += 1
            if iteration % log_interval == 0:
                self.get_logger().info(
                    f"  Position: X={self.x:.2f}, Y={self.y:.2f}, Yaw={math.degrees(self.yaw):.1f}°, Remaining: ΔX={dx:.2f}"
                )
            time.sleep(0.05)

        self._stop()
        time.sleep(1.0)

    def _drive_straight_y(self, target_x, target_y):
        self.get_logger().info(f"Driving NORTH/SOUTH to {target_y}")

        log_interval = 10  # Log every 10 iterations (0.5 seconds)
        iteration = 0
        stop_tolerance = 0.05

        while rclpy.ok():
            dx = target_x - self.x
            dy = target_y - self.y

            if abs(dy) < stop_tolerance:
                break

            t = Twist()

            # Slow down as we approach the row center to avoid overshoot,
            # which otherwise leaves the west/east leg starting off-row.
            t.linear.x = 0.5 if abs(dy) > 0.5 else 0.2

            # 🔥 KEY: keep X locked
            error_x = dx
            t.angular.z = 2.5 * error_x

            # Log position every log_interval iterations
            iteration += 1
            if iteration % log_interval == 0:
                self.get_logger().info(
                    f"  Position: X={self.x:.2f}, Y={self.y:.2f}, Yaw={math.degrees(self.yaw):.1f}°, Remaining: ΔY={dy:.2f}"
                )

            t.angular.z = max(-1.0, min(1.0, t.angular.z))

            self.vel_pub.publish(t)
            time.sleep(0.05)

        self._stop()
        time.sleep(1.0)

    # ───────────── TURN ─────────────

    def _turn_to(self, target_yaw):
        self.get_logger().info(f"Turning to {math.degrees(target_yaw):.1f}°")

        log_interval = 5  # Log every 5 iterations (0.1 seconds)
        iteration = 0

        while rclpy.ok():
            err = self._angle_diff(target_yaw, self.yaw)

            if abs(err) < 0.05:
                break

            t = Twist()
            t.angular.z = 1.5 if err > 0 else -1.5

            # Log position every log_interval iterations
            iteration += 1
            if iteration % log_interval == 0:
                self.get_logger().info(
                    f"  Position: X={self.x:.2f}, Y={self.y:.2f}, Yaw={math.degrees(self.yaw):.1f}°, Error={math.degrees(err):.1f}°"
                )

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

    def _wait_for_west_then_publish_pose1(self, label: str):
        """Background waiter: when heading becomes `west`, wait 2s then
        publish a `pose1` manipulator trigger for the given label.
        """
        self.get_logger().info(
            f"Scheduling follow-up pose1 for {label} when heading west"
        )
        # Poll until the robot is heading west (or rclpy shuts down)
        while rclpy.ok():
            try:
                direction = self._current_direction()
            except Exception:
                direction = None
            if direction == "west":
                break
            time.sleep(0.1)

        # Wait the requested 2 seconds after turn completes
        time.sleep(2.0)

        msg = String()
        msg.data = f"label={label};x={self.x:.2f};y={self.y:.2f};dir=west;manip=pose1"
        self.trigger_pub.publish(msg)
        self.get_logger().info(f"Published follow-up trigger: {msg.data}")


def main():
    rclpy.init()
    node = AutoNavigator()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
