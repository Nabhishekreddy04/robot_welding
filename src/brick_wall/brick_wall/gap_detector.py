from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, String


class GapDetector(Node):
    """Detect floor gaps from a depth image and optionally stop the robot."""

    def __init__(self) -> None:
        super().__init__("gap_detector")

        self.declare_parameter("depth_topic", "/depth_camera/depth/image_raw")
        self.declare_parameter("image_topic", "/depth_camera/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_safety")
        self.declare_parameter("detection_topic", "/gap_detector/gap_detected")
        self.declare_parameter("gap_type_topic", "/gap_type")
        self.declare_parameter("gap_center_topic", "/gap_center_error")
        self.declare_parameter("stop_on_gap", False)
        self.declare_parameter("camera_info_topic", "/depth_camera/depth/camera_info")
        self.declare_parameter("block_size_m", 0.025)
        self.declare_parameter("gap_size_m", 0.003)
        self.declare_parameter("depth_jump_m", 0.004)
        self.declare_parameter("min_col_gap_ratio", 0.10)
        self.declare_parameter("min_run_ratio", 0.18)
        self.declare_parameter("min_valid_ratio", 0.15)
        self.declare_parameter("max_gap_depth_m", 0.80)
        self.declare_parameter("dark_pixel_threshold", 110)
        self.declare_parameter("dark_min_ratio", 0.0008)
        self.declare_parameter("red_dominance_threshold", 25)
        self.declare_parameter("edge_threshold", 10.0)
        self.declare_parameter("use_rgb_primary", True)
        self.declare_parameter("gap_confirm_frames", 3)
        self.declare_parameter("clear_confirm_frames", 2)
        self.declare_parameter("status_print_period_sec", 2.0)
        self.declare_parameter("roi_row_start", 0.55)
        self.declare_parameter("roi_row_end", 0.90)
        self.declare_parameter("roi_col_start", 0.35)
        self.declare_parameter("roi_col_end", 0.65)
        self.declare_parameter("center_deadband_px", 4.0)
        self.declare_parameter("center_alpha", 0.35)

        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.detection_topic = str(self.get_parameter("detection_topic").value)
        self.gap_type_topic = str(self.get_parameter("gap_type_topic").value)
        self.gap_center_topic = str(self.get_parameter("gap_center_topic").value)
        self.stop_on_gap = bool(self.get_parameter("stop_on_gap").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.block_size_m = float(self.get_parameter("block_size_m").value)
        self.gap_size_m = float(self.get_parameter("gap_size_m").value)
        self.depth_jump_m = float(self.get_parameter("depth_jump_m").value)
        self.min_col_gap_ratio = float(self.get_parameter("min_col_gap_ratio").value)
        self.min_run_ratio = float(self.get_parameter("min_run_ratio").value)
        self.min_valid_ratio = float(self.get_parameter("min_valid_ratio").value)
        self.max_gap_depth_m = float(self.get_parameter("max_gap_depth_m").value)
        self.dark_pixel_threshold = int(
            self.get_parameter("dark_pixel_threshold").value
        )
        self.dark_min_ratio = float(self.get_parameter("dark_min_ratio").value)
        self.red_dominance_threshold = float(
            self.get_parameter("red_dominance_threshold").value
        )
        self.edge_threshold = float(self.get_parameter("edge_threshold").value)
        self.use_rgb_primary = bool(self.get_parameter("use_rgb_primary").value)
        self.gap_confirm_frames = int(self.get_parameter("gap_confirm_frames").value)
        self.clear_confirm_frames = int(
            self.get_parameter("clear_confirm_frames").value
        )
        self.status_print_period_sec = float(
            self.get_parameter("status_print_period_sec").value
        )
        self.center_deadband_px = float(self.get_parameter("center_deadband_px").value)
        self.center_alpha = float(self.get_parameter("center_alpha").value)

        self.roi = (
            float(self.get_parameter("roi_row_start").value),
            float(self.get_parameter("roi_row_end").value),
            float(self.get_parameter("roi_col_start").value),
            float(self.get_parameter("roi_col_end").value),
        )

        self.gap_pub = self.create_publisher(Bool, self.detection_topic, 10)
        self.gap_type_pub = self.create_publisher(String, self.gap_type_topic, 10)
        self.gap_center_pub = self.create_publisher(Float32, self.gap_center_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.sub = self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.rgb_sub = self.create_subscription(
            Image, self.image_topic, self.rgb_cb, 10
        )
        self.cam_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_cb, 10
        )

        self.last_gap_state: Optional[bool] = None
        self.gap_frame_count = 0
        self.clear_frame_count = 0
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.latest_rgb: Optional[np.ndarray] = None
        self.last_center_error_norm = 0.0

        self.get_logger().info(
            "Gap detector started. "
            f"depth_topic={self.depth_topic}, block={self.block_size_m*1000:.1f}mm, "
            f"gap={self.gap_size_m*1000:.1f}mm"
        )

    @staticmethod
    def center_error_px(mask_2d: np.ndarray) -> Optional[float]:
        """Return horizontal center error in pixels (+right, -left)."""
        if mask_2d.size == 0:
            return None
        col_energy = np.sum(mask_2d.astype(np.float32), axis=0)
        total = float(np.sum(col_energy))
        if total < 1.0:
            return None
        cols = np.arange(col_energy.size, dtype=np.float32)
        center = float(np.sum(cols * col_energy) / total)
        mid = 0.5 * float(col_energy.size - 1)
        return center - mid

    def camera_info_cb(self, msg: CameraInfo) -> None:
        if len(msg.k) >= 5:
            if msg.k[0] > 0.0 and self.fx is None:
                self.fx = float(msg.k[0])
            if msg.k[4] > 0.0 and self.fy is None:
                self.fy = float(msg.k[4])

    def rgb_cb(self, msg: Image) -> None:
        image = self.decode_rgb_image(msg)
        if image is not None:
            self.latest_rgb = image

    def decode_depth_image(self, msg: Image) -> Optional[np.ndarray]:
        """Convert sensor_msgs/Image depth message to meters as float32."""
        if msg.height == 0 or msg.width == 0:
            return None

        if msg.encoding == "32FC1":
            data = np.frombuffer(msg.data, dtype=np.float32)
            if data.size < msg.height * msg.width:
                return None
            return data[: msg.height * msg.width].reshape((msg.height, msg.width))

        if msg.encoding == "16UC1":
            data = np.frombuffer(msg.data, dtype=np.uint16)
            if data.size < msg.height * msg.width:
                return None
            depth_mm = data[: msg.height * msg.width].reshape((msg.height, msg.width))
            return depth_mm.astype(np.float32) / 1000.0

        self.get_logger().warn(
            f"Unsupported depth encoding '{msg.encoding}'. Expected 32FC1 or 16UC1.",
            throttle_duration_sec=5.0,
        )
        return None

    def decode_rgb_image(self, msg: Image) -> Optional[np.ndarray]:
        if msg.height == 0 or msg.width == 0:
            return None

        enc = msg.encoding.lower()
        if enc in ("rgb8", "bgr8"):
            data = np.frombuffer(msg.data, dtype=np.uint8)
            expected = msg.height * msg.width * 3
            if data.size < expected:
                return None
            img = data[:expected].reshape((msg.height, msg.width, 3))
            if enc == "bgr8":
                img = img[:, :, ::-1]
            return img

        if enc in ("rgba8", "bgra8"):
            data = np.frombuffer(msg.data, dtype=np.uint8)
            expected = msg.height * msg.width * 4
            if data.size < expected:
                return None
            img = data[:expected].reshape((msg.height, msg.width, 4))[:, :, :3]
            if enc == "bgra8":
                img = img[:, :, ::-1]
            return img

        return None

    def roi_slice(self, image: np.ndarray) -> Tuple[slice, slice]:
        """Return center-forward ROI where floor/gap is expected."""
        rows, cols = image.shape
        r0 = int(rows * max(0.0, min(1.0, self.roi[0])))
        r1 = int(rows * max(0.0, min(1.0, self.roi[1])))
        c0 = int(cols * max(0.0, min(1.0, self.roi[2])))
        c1 = int(cols * max(0.0, min(1.0, self.roi[3])))

        if r1 <= r0:
            r0, r1 = int(rows * 0.55), int(rows * 0.90)
        if c1 <= c0:
            c0, c1 = int(cols * 0.35), int(cols * 0.65)

        return slice(r0, r1), slice(c0, c1)

    def depth_cb(self, msg: Image) -> None:
        depth = self.decode_depth_image(msg)
        if depth is None:
            return

        rs, cs = self.roi_slice(depth)
        roi = depth[rs, cs]
        if roi.size == 0:
            return

        valid = np.isfinite(roi) & (roi > 0.05)
        valid_ratio = float(np.count_nonzero(valid)) / float(roi.size)
        depth_center_error_px: Optional[float] = None

        if valid_ratio < self.min_valid_ratio:
            # Too little information: avoid false positives on noisy/occluded frames.
            depth_gap_detected = False
        else:
            valid_depth = roi[valid]
            baseline_depth = float(np.percentile(valid_depth, 35.0))
            if baseline_depth > self.max_gap_depth_m:
                # Looking too far ahead tends to create persistent false seams.
                depth_gap_detected = False
            else:
                deep_mask = valid & (roi > (baseline_depth + self.depth_jump_m))
                depth_center_error_px = self.center_error_px(deep_mask)
                col_gap_ratio = np.mean(deep_mask, axis=0)
                row_gap_ratio = np.mean(deep_mask, axis=1)
                col_is_gap = col_gap_ratio >= self.min_col_gap_ratio
                row_is_gap = row_gap_ratio >= self.min_col_gap_ratio

                # Add run-length evidence so thin seams with perspective skew are still detected.
                min_col_run_px = max(
                    1, int(round(deep_mask.shape[0] * self.min_run_ratio))
                )
                min_row_run_px = max(
                    1, int(round(deep_mask.shape[1] * self.min_run_ratio))
                )
                col_run_mask = np.array(
                    [
                        self.longest_true_run(deep_mask[:, c]) >= min_col_run_px
                        for c in range(deep_mask.shape[1])
                    ]
                )
                row_run_mask = np.array(
                    [
                        self.longest_true_run(deep_mask[r, :]) >= min_row_run_px
                        for r in range(deep_mask.shape[0])
                    ]
                )
                col_is_gap = col_is_gap | col_run_mask
                row_is_gap = row_is_gap | row_run_mask

                if self.fx is not None and baseline_depth > 0.05:
                    expected_gap_px_x = max(
                        1,
                        int(round((self.fx * self.gap_size_m) / baseline_depth)),
                    )
                    expected_block_px_x = max(
                        1,
                        int(round((self.fx * self.block_size_m) / baseline_depth)),
                    )
                else:
                    # Safe fallback until camera intrinsics are received.
                    expected_gap_px_x = 2
                    expected_block_px_x = 10

                if self.fy is not None and baseline_depth > 0.05:
                    expected_gap_px_y = max(
                        1,
                        int(round((self.fy * self.gap_size_m) / baseline_depth)),
                    )
                    expected_block_px_y = max(
                        1,
                        int(round((self.fy * self.block_size_m) / baseline_depth)),
                    )
                else:
                    expected_gap_px_y = expected_gap_px_x
                    expected_block_px_y = expected_block_px_x

                depth_gap_detected = self.has_gap_run(
                    col_is_gap,
                    expected_gap_px_x,
                    expected_block_px_x,
                ) or self.has_gap_run(
                    row_is_gap,
                    expected_gap_px_y,
                    expected_block_px_y,
                )

        frame_gap_detected = depth_gap_detected
        frame_center_error_px = depth_center_error_px

        # Fuse with visual seam detection from RGB image.
        if self.latest_rgb is not None:
            rgb_gap, rgb_center_error_px = self.detect_dark_seam_gap(
                self.latest_rgb, rs, cs
            )
            if self.use_rgb_primary:
                frame_gap_detected = rgb_gap
                frame_center_error_px = rgb_center_error_px
            else:
                frame_gap_detected = frame_gap_detected or rgb_gap
                if rgb_center_error_px is not None:
                    frame_center_error_px = rgb_center_error_px

        # Require a few consecutive frames before state changes to avoid noise.
        if frame_gap_detected:
            self.gap_frame_count += 1
            self.clear_frame_count = 0
        else:
            self.clear_frame_count += 1
            self.gap_frame_count = 0

        if self.last_gap_state in (None, False):
            gap_detected = self.gap_frame_count >= self.gap_confirm_frames
        else:
            gap_detected = not (self.clear_frame_count >= self.clear_confirm_frames)

        msg_out = Bool()
        msg_out.data = gap_detected
        self.gap_pub.publish(msg_out)

        center_msg = Float32()
        type_msg = String()

        if gap_detected and frame_center_error_px is not None:
            if abs(frame_center_error_px) <= self.center_deadband_px:
                center_norm = 0.0
            else:
                half_w = max(1.0, 0.5 * float(roi.shape[1] - 1))
                center_norm = max(-1.0, min(1.0, frame_center_error_px / half_w))
            center_norm = (
                self.center_alpha * center_norm
                + (1.0 - self.center_alpha) * self.last_center_error_norm
            )
            self.last_center_error_norm = center_norm
            center_msg.data = float(center_norm)
            type_msg.data = "centered" if abs(center_norm) < 0.12 else "off_center"
        elif gap_detected:
            # Gap seen but center estimate unavailable in this frame.
            center_msg.data = float(self.last_center_error_norm)
            type_msg.data = "off_center"
        else:
            self.last_center_error_norm = 0.0
            center_msg.data = 0.0
            type_msg.data = "none"

        self.gap_center_pub.publish(center_msg)
        self.gap_type_pub.publish(type_msg)

        if gap_detected and self.stop_on_gap:
            self.cmd_pub.publish(Twist())

        if self.last_gap_state != gap_detected:
            self.last_gap_state = gap_detected

        if gap_detected:
            self.get_logger().warn(
                "GAP DETECTED",
                throttle_duration_sec=self.status_print_period_sec,
            )
        else:
            self.get_logger().info(
                "NO GAP DETECTED",
                throttle_duration_sec=self.status_print_period_sec,
            )

    @staticmethod
    def find_true_runs(mask: np.ndarray) -> list[Tuple[int, int]]:
        runs: list[Tuple[int, int]] = []
        start = -1
        for idx, val in enumerate(mask):
            if val and start < 0:
                start = idx
            elif not val and start >= 0:
                runs.append((start, idx - 1))
                start = -1
        if start >= 0:
            runs.append((start, len(mask) - 1))
        return runs

    def has_gap_run(
        self,
        is_gap_mask: np.ndarray,
        expected_gap_px: int,
        expected_block_px: int,
    ) -> bool:
        runs = self.find_true_runs(is_gap_mask)
        min_gap_px = max(1, int(round(expected_gap_px * 0.5)))
        max_gap_px = max(min_gap_px, int(round(expected_gap_px * 3.0)))
        support_span = max(2, int(round(expected_block_px * 0.25)))

        for start, end in runs:
            width = end - start + 1
            if width < min_gap_px or width > max_gap_px:
                continue

            before_clear = start >= support_span
            after_clear = (is_gap_mask.size - 1 - end) >= support_span
            if before_clear and after_clear:
                return True

        return False

    def detect_dark_seam_gap(
        self,
        rgb_image: np.ndarray,
        row_slice: slice,
        col_slice: slice,
    ) -> Tuple[bool, Optional[float]]:
        roi = rgb_image[row_slice, col_slice]
        if roi.size == 0:
            return False, None

        # Grayscale luminance.
        gray = (
            0.299 * roi[:, :, 0] + 0.587 * roi[:, :, 1] + 0.114 * roi[:, :, 2]
        ).astype(np.float32)

        red = roi[:, :, 0].astype(np.float32)
        green = roi[:, :, 1].astype(np.float32)
        blue = roi[:, :, 2].astype(np.float32)
        red_dominance = red - np.maximum(green, blue)

        dark = gray < float(self.dark_pixel_threshold)
        low_red = red_dominance < self.red_dominance_threshold

        # Keep only seam pixels that also lie on image edges to suppress flat regions.
        gx = np.abs(np.diff(gray, axis=1, prepend=gray[:, :1]))
        gy = np.abs(np.diff(gray, axis=0, prepend=gray[:1, :]))
        edge_mask = (gx > self.edge_threshold) | (gy > self.edge_threshold)

        seam_like = (dark | low_red) & edge_mask

        seam_ratio = float(np.count_nonzero(seam_like)) / float(seam_like.size)
        if seam_ratio < self.dark_min_ratio:
            return False, None

        min_col_run_px = max(1, int(round(seam_like.shape[0] * self.min_run_ratio)))
        min_row_run_px = max(1, int(round(seam_like.shape[1] * self.min_run_ratio)))

        col_has_long = any(
            self.longest_true_run(seam_like[:, c]) >= min_col_run_px
            for c in range(seam_like.shape[1])
        )
        row_has_long = any(
            self.longest_true_run(seam_like[r, :]) >= min_row_run_px
            for r in range(seam_like.shape[0])
        )

        detected = bool(col_has_long or row_has_long)
        center_px = self.center_error_px(seam_like)
        return detected, center_px

    @staticmethod
    def longest_true_run(mask: np.ndarray) -> int:
        best = 0
        cur = 0
        for val in mask:
            if bool(val):
                cur += 1
                if cur > best:
                    best = cur
            else:
                cur = 0
        return best


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GapDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
