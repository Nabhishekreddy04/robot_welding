#!/usr/bin/env python3
"""auto_navigator.py

Line-segment follower for welding path navigation.
The controller tracks each segment between consecutive waypoints
using heading error + cross-track error (line follower behavior).
"""

import math
import time
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class AutoNavigator(Node):

    def __init__(self):
        super().__init__("auto_navigator")

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_ready = False
        self.gap_type = "none"

        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(String, "/gap_type", self._on_gap, 10)

        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ── WAYPOINTS ──────────────────────────────────────────────────────────
        # Each entry: (target_x, target_y)
        # The robot drives straight to each point.
        # No separate yaw target — heading is computed from current→target.
        #
        # Spawn: x=-40, y=3 facing east.
        # H-gap 1 is at y=3.0 (robot is already on it at spawn).
        # H-gap 2 is at y=6.003
        # H-gap 3 is at y=9.006
        # H-gap 4 is at y=12.009
        #
        # Each horizontal pass goes the full wall length:
        # wall left edge ≈ x=-30, right edge ≈ x=+30
        # We use x=-31 and x=+31 (1m past edges) so robot is clear

        self.waypoints = [
            (-30.00, 3.00),
            (-28.00, 3.00),
            (-26.00, 3.00),
            (-24.00, 3.00),
            (-22.00, 3.00),
            (-20.00, 3.00),
            (-18.00, 3.00),
            (-15.00, 3.00),
            (-12.00, 3.00),
            (-9.00, 3.00),
            (-6.00, 3.00),
            (-3.00, 3.00),
            (0.00, 3.00),
            (2.00, 3.00),
            (3.00, 3.00),
            (4.00, 3.00),
            (5.00, 3.00),
            (6.00, 3.00),
            (7.00, 3.00),
            (8.00, 3.00),
            (9.00, 3.00),
            (10.00, 3.00),
            (11.00, 3.00),
            (12.00, 3.00),
            (13.00, 3.00),
            (14.00, 3.00),
            (15.00, 3.00),
            (16.00, 3.00),
            (17.00, 3.00),
            (18.00, 3.00),
            (19.00, 3.00),
            (20.00, 3.00),
            (21.00, 3.00),
            (22.00, 3.00),
            (23.00, 3.00),
            (24.00, 3.00),
            (25.00, 3.00),
            (26.00, 3.00),
            (27.00, 3.00),
            (28.00, 3.00),
            (29.00, 3.00),
            (30.00, 3.00),
            (32.00, 3.00),
            (35.00, 3.00),
            (35.00, -3.00),
            (32.00, -3.00),
            (30.00, -3.00),
            (28.00, -3.00),
            (26.00, -3.00),
            (24.00, -3.00),
            (22.00, -3.00),
            (20.00, -3.00),
            (18.00, -3.00),
            (15.00, -3.00),
            (12.00, -3.00),
            (9.00, -3.00),
            (6.00, -3.00),
            (3.00, -3.00),
            (0.00, -3.00),
            (-2.00, -3.00),
            (-3.00, -3.00),
            (-4.00, -3.00),
            (-5.00, -3.00),
            (-6.00, -3.00),
            (-7.00, -3.00),
            (-8.00, -3.00),
            (-9.00, -3.00),
            (-10.00, -3.00),
            (-11.00, -3.00),
            (-12.00, -3.00),
            (-13.00, -3.00),
            (-14.00, -3.00),
            (-15.00, -3.00),
            (-16.00, -3.00),
            (-17.00, -3.00),
            (-18.00, -3.00),
            (-19.00, -3.00),
            (-20.00, -3.00),
            (-21.00, -3.00),
            (-22.00, -3.00),
            (-23.00, -3.00),
            (-24.00, -3.00),
            (-25.00, -3.00),
            (-26.00, -3.00),
            (-28.00, -3.00),
            (-30.00, -3.00),
            (-32.00, -3.00),
            (-35.00, -3.00),
        ]

        # Control gains and limits
        self.DRIVE_SPEED = 0.24
        self.REPOS_SPEED = 0.24
        self.MIN_SPEED = 0.10
        self.MAX_TURN = 1.0
        self.TURN_KP = 2.2
        self.LINE_KP = 1.8
        self.GOAL_TOL = 0.12
        self.ANGLE_TOL = 0.04
        self.STUCK_PROGRESS_EPS = 0.03
        self.STUCK_TIMEOUT = 2.5
        self.SEGMENT_TIMEOUT = 45.0

        # First horizontal pass should stay straight with camera centered on gap.
        self.FIRST_ROW_Y = 3.0
        self.FIRST_ROW_END_X = -0.02
        self.FIRST_ROW_HEADING = 0.0
        self.FIRST_ROW_HEADING_KP = 1.4
        self.FIRST_ROW_CENTER_KP = 4.4
        self.FIRST_ROW_MAX_TURN = 0.18
        self.FIRST_ROW_SPEED_CAP = 0.14

        self.HORIZONTAL_AXIS_TOL = 0.12
        self.VERTICAL_AXIS_TOL = 0.12
        self.HORIZONTAL_HEADING_KP = 1.7
        self.HORIZONTAL_CENTER_KP = 4.0
        self.HORIZONTAL_MAX_TURN = 0.32
        self.HORIZONTAL_SPEED_CAP = 0.16
        self.GAP_SLOWDOWN = 0.80

        self.ready_timer = self.create_timer(0.5, self._check_ready)
        self.get_logger().info("Auto navigator started. Waiting for odometry...")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.odom_ready = True

    def _on_gap(self, msg: String):
        self.gap_type = msg.data

    def _check_ready(self):
        if self.odom_ready:
            self.ready_timer.cancel()
            self.get_logger().info(
                f"Odometry ready at ({self.x:.2f}, {self.y:.2f}). " f"Starting in 2s..."
            )
            threading.Thread(target=self._run_mission, daemon=True).start()

    # ── Mission ───────────────────────────────────────────────────────────────

    def _run_mission(self):
        time.sleep(2.0)
        self.get_logger().info("=== MISSION START ===")

        for i, (tx, ty) in enumerate(self.waypoints):
            self.get_logger().info(
                f"Segment {i+1}/{len(self.waypoints)} -> ({tx:.2f}, {ty:.2f})"
            )
            self._follow_segment(tx, ty)
            self.get_logger().info(f"Waypoint {i+1} reached.")

        self.get_logger().info("=== MISSION COMPLETE ===")
        self._stop()

    # ── Core line follower ───────────────────────────────────────────────────

    def _follow_segment(self, tx: float, ty: float):
        """Follow a segment from current pose to (tx, ty)."""
        sx, sy = self.x, self.y
        seg_dx = tx - sx
        seg_dy = ty - sy
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len < 1e-6:
            return

        path_heading = math.atan2(seg_dy, seg_dx)
        on_first_row = (
            tx <= self.FIRST_ROW_END_X + 1e-6 and abs(ty - self.FIRST_ROW_Y) < 0.08
        )
        is_horizontal = abs(seg_dy) <= abs(seg_dx)
        is_steep_segment = abs(math.degrees(path_heading)) > 35.0

        # Do not perform turn-in-place on shallow moves; only rotate for steep transitions.
        if not on_first_row and is_steep_segment:
            self._turn_to(path_heading)

        is_repos = abs(ty - sy) > 1.0 and abs(tx - sx) < 5.0
        speed_ref = self.REPOS_SPEED if is_repos else self.DRIVE_SPEED
        if self.gap_type != "none":
            speed_ref *= self.GAP_SLOWDOWN

        last_best_progress = -1.0
        last_progress_time = time.time()
        segment_start_time = time.time()

        while rclpy.ok():
            if (time.time() - segment_start_time) > self.SEGMENT_TIMEOUT:
                self.get_logger().warn(
                    "Segment timeout exceeded. Skipping to next waypoint."
                )
                break

            rx = self.x - sx
            ry = self.y - sy

            # Signed distance from robot to infinite path line.
            cross_track = (-math.sin(path_heading) * rx) + (math.cos(path_heading) * ry)

            # Forward progress projected on path direction.
            progress = (rx * seg_dx + ry * seg_dy) / seg_len

            # Stop only when the robot is actually centered on the active path.
            dist_to_goal = math.hypot(tx - self.x, ty - self.y)
            if on_first_row:
                if (
                    abs(self.x - tx) < self.HORIZONTAL_AXIS_TOL
                    and abs(self.y - ty) < self.HORIZONTAL_AXIS_TOL
                ):
                    break
            elif is_horizontal:
                if (
                    abs(self.x - tx) < self.HORIZONTAL_AXIS_TOL
                    and abs(self.y - ty) < self.HORIZONTAL_AXIS_TOL
                ):
                    break
            elif is_repos:
                if (
                    abs(self.x - tx) < self.VERTICAL_AXIS_TOL
                    and abs(self.y - ty) < self.VERTICAL_AXIS_TOL
                ):
                    break
            elif progress >= (seg_len - self.GOAL_TOL) or dist_to_goal < self.GOAL_TOL:
                break

            if on_first_row:
                # Keep the gap centered on robot camera axis during first pass.
                heading_error = self._wrap(self.FIRST_ROW_HEADING - self.yaw)
                center_error = self.y - self.FIRST_ROW_Y
                steer = (
                    self.FIRST_ROW_HEADING_KP * heading_error
                    - self.FIRST_ROW_CENTER_KP * center_error
                )
                steer = max(
                    -self.FIRST_ROW_MAX_TURN, min(self.FIRST_ROW_MAX_TURN, steer)
                )

                linear_speed = min(self.DRIVE_SPEED, self.FIRST_ROW_SPEED_CAP)
                if abs(center_error) > 0.20:
                    linear_speed = max(self.MIN_SPEED, self.FIRST_ROW_SPEED_CAP * 0.75)

                # Segment completion on first row must stay centered on the gap.
                if (
                    abs(self.x - tx) < self.HORIZONTAL_AXIS_TOL
                    and abs(center_error) < self.HORIZONTAL_AXIS_TOL
                ):
                    break

                twist = Twist()
                twist.linear.x = linear_speed
                twist.angular.z = steer
                self.vel_pub.publish(twist)
                time.sleep(0.05)
                continue

            if is_horizontal:
                desired_heading = 0.0 if seg_dx >= 0.0 else math.pi
                heading_error = self._wrap(desired_heading - self.yaw)
                center_error = self.y - ty
                steer = (
                    self.HORIZONTAL_HEADING_KP * heading_error
                    - self.HORIZONTAL_CENTER_KP * center_error
                )
                steer = max(
                    -self.HORIZONTAL_MAX_TURN,
                    min(self.HORIZONTAL_MAX_TURN, steer),
                )

                linear_speed = min(speed_ref, self.HORIZONTAL_SPEED_CAP)
                if abs(center_error) > 0.18:
                    linear_speed = max(self.MIN_SPEED, linear_speed * 0.7)

                if (
                    abs(self.x - tx) < self.HORIZONTAL_AXIS_TOL
                    and abs(center_error) < self.HORIZONTAL_AXIS_TOL
                ):
                    break

                twist = Twist()
                twist.linear.x = linear_speed
                twist.angular.z = steer
                self.vel_pub.publish(twist)
                time.sleep(0.05)
                continue

            # Reposition segments are steep and short; direct target pursuit
            # is typically more stable than strict line tracking.
            if is_repos:
                angle_to_goal = math.atan2(ty - self.y, tx - self.x)
                heading_error = self._wrap(angle_to_goal - self.yaw)
                steer = max(
                    -self.MAX_TURN, min(self.MAX_TURN, self.TURN_KP * heading_error)
                )
                speed_scale = max(0.35, 1.0 - 0.7 * abs(heading_error))
                linear_speed = max(self.MIN_SPEED, speed_ref * speed_scale)

                if (
                    abs(self.y - ty) < self.VERTICAL_AXIS_TOL
                    and abs(self.x - tx) < self.VERTICAL_AXIS_TOL
                ):
                    break

                twist = Twist()
                twist.linear.x = linear_speed
                twist.angular.z = steer
                self.vel_pub.publish(twist)
                time.sleep(0.05)
                continue

            if (
                abs(self.x - tx) < self.HORIZONTAL_AXIS_TOL
                and abs(self.y - ty) < self.HORIZONTAL_AXIS_TOL
            ):
                break

            if progress > (last_best_progress + self.STUCK_PROGRESS_EPS):
                last_best_progress = progress
                last_progress_time = time.time()

            if (time.time() - last_progress_time) > self.STUCK_TIMEOUT:
                self.get_logger().warn(
                    "Low progress detected. Running recovery motion."
                )
                self._recover_to_line(path_heading, cross_track)
                last_progress_time = time.time()

            heading_error = self._wrap(path_heading - self.yaw)
            steer = (self.TURN_KP * heading_error) - (self.LINE_KP * cross_track)
            steer = max(-self.MAX_TURN, min(self.MAX_TURN, steer))

            speed_scale = max(0.25, 1.0 - 0.7 * abs(heading_error))
            linear_speed = max(self.MIN_SPEED, speed_ref * speed_scale)

            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = steer
            self.vel_pub.publish(twist)
            time.sleep(0.05)

        self._stop()
        time.sleep(0.2)

    def _recover_to_line(self, path_heading: float, cross_track: float):
        """Short recovery when robot is not making forward progress."""
        correction = -0.5 if cross_track > 0.0 else 0.5
        target = self._wrap(path_heading + correction)
        self._turn_to(target)

        twist = Twist()
        twist.linear.x = self.MIN_SPEED
        for _ in range(8):
            self.vel_pub.publish(twist)
            time.sleep(0.05)
        self._stop()

    # ── Turn to heading ───────────────────────────────────────────────────────

    def _turn_to(self, target_yaw: float):
        """Rotate in place until facing target_yaw."""
        self.get_logger().info(f"  Turning to {math.degrees(target_yaw):.1f}°")

        while rclpy.ok():
            err = self._wrap(target_yaw - self.yaw)
            if abs(err) < self.ANGLE_TOL:
                break
            twist = Twist()
            twist.angular.z = max(
                -self.MAX_TURN, min(self.MAX_TURN, self.TURN_KP * err)
            )
            self.vel_pub.publish(twist)
            time.sleep(0.02)

        self._stop()
        time.sleep(0.2)

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _wrap(a: float) -> float:
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def _stop(self):
        self.vel_pub.publish(Twist())
        time.sleep(0.1)


def main():
    rclpy.init()
    node = AutoNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
