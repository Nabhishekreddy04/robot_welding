#!/usr/bin/env python3
"""
Live odometry plotter for ROS 2.
Subscribes to /odom (nav_msgs/Odometry) and plots:
 - x and y position vs time
 - linear speed vs time

Run (after sourcing ROS 2):
  python3 src/brick_wall/brick_wall/odom_plotter.py

Or add an entrypoint to run with `ros2 run` if desired.
"""

import threading
import time
import math

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
except Exception as e:
    print("Error importing rclpy or ROS 2 messages:", e)
    print(
        "Make sure ROS 2 is sourced (e.g. 'source /opt/ros/<distro>/setup.bash') and rclpy is available."
    )
    raise

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
except Exception as e:
    print("Error importing matplotlib:", e)
    print(
        "Install matplotlib (pip install matplotlib) in the Python environment used by ROS 2."
    )
    raise


class OdomPlotter(Node):
    def __init__(self):
        super().__init__("odom_plotter")
        self.sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )
        self.lock = threading.Lock()
        self.times = []
        self.xs = []
        self.ys = []
        self.speeds = []
        self.start_time = None
        self.max_points = 2000

    def odom_callback(self, msg: Odometry):
        with self.lock:
            t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            if self.start_time is None:
                self.start_time = t
            rel = t - self.start_time
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            speed = math.hypot(vx, vy)

            self.times.append(rel)
            self.xs.append(x)
            self.ys.append(y)
            self.speeds.append(speed)

            if len(self.times) > self.max_points:
                # trim oldest
                self.times = self.times[-self.max_points :]
                self.xs = self.xs[-self.max_points :]
                self.ys = self.ys[-self.max_points :]
                self.speeds = self.speeds[-self.max_points :]

    def get_data_snapshot(self):
        with self.lock:
            return (
                list(self.times),
                list(self.xs),
                list(self.ys),
                list(self.speeds),
            )


def run_plotter():
    rclpy.init()
    node = OdomPlotter()

    # Spin ROS in a background thread so callbacks keep populating data
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    # Setup matplotlib figure
    fig, (ax_pos, ax_speed) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    (line_x,) = ax_pos.plot([], [], label="x")
    (line_y,) = ax_pos.plot([], [], label="y")
    ax_pos.set_ylabel("Position (m)")
    ax_pos.legend(loc="upper left")
    ax_pos.grid(True)

    (line_speed,) = ax_speed.plot([], [], label="speed")
    ax_speed.set_xlabel("Time (s)")
    ax_speed.set_ylabel("Linear speed (m/s)")
    ax_speed.grid(True)

    window_seconds = 30.0

    def init():
        line_x.set_data([], [])
        line_y.set_data([], [])
        line_speed.set_data([], [])
        ax_pos.relim()
        ax_pos.autoscale_view()
        ax_speed.relim()
        ax_speed.autoscale_view()
        return line_x, line_y, line_speed

    def update(frame):
        times, xs, ys, speeds = node.get_data_snapshot()
        if not times:
            return line_x, line_y, line_speed

        # show only last window_seconds seconds
        t0 = max(0.0, times[-1] - window_seconds)
        # find index to start
        start_idx = 0
        for i, t in enumerate(times):
            if t >= t0:
                start_idx = i
                break
        t_plot = times[start_idx:]
        xs_plot = xs[start_idx:]
        ys_plot = ys[start_idx:]
        speeds_plot = speeds[start_idx:]

        line_x.set_data(t_plot, xs_plot)
        line_y.set_data(t_plot, ys_plot)
        line_speed.set_data(t_plot, speeds_plot)

        ax_pos.set_xlim(max(0.0, t_plot[0]), t_plot[-1])

        # autoscale y-limits with some margin
        for ax, data in ((ax_pos, xs_plot + ys_plot), (ax_speed, speeds_plot)):
            if len(data) > 0:
                mn = min(data)
                mx = max(data)
                if mx - mn < 1e-3:
                    mn -= 0.5
                    mx += 0.5
                ax.set_ylim(
                    mn - 0.1 * abs(mx - mn + 1e-6), mx + 0.1 * abs(mx - mn + 1e-6)
                )

        return line_x, line_y, line_speed

    ani = animation.FuncAnimation(fig, update, init_func=init, interval=200, blit=False)

    plt.tight_layout()
    try:
        plt.show()
    except Exception:
        pass

    # When the window closes, shutdown ROS
    try:
        node.destroy_node()
    except Exception:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    run_plotter()
