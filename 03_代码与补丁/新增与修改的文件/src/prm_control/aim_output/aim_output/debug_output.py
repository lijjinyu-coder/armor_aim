# -*- coding: utf-8 -*-
"""
debug_output.py — 调试输出适配器

职责：把 AimResult 以日志/画面形式输出，便于现场调试（任务书 §四要求调试画面能看到
装甲板候选、最终目标、yaw/pitch/distance、处理 FPS 和目标状态）。

与 ROS2 云台输出彼此独立：两者都只消费 AimResult。
"""
import time

import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import KeyPoints

from .aim_result import AimResult


class DebugOutput:
    def __init__(self, node: Node,
                 image_topic: str = "/image_raw",
                 keypoints_topic: str = "/key_points",
                 debug_image_topic: str = "/debug_image",
                 show_window: bool = False,
                 publish_debug_image: bool = True,
                 render_hz: float = 15.0,
                 log_period_s: float = 1.0,
                 detector_width: int = 1280,
                 detector_height: int = 1024):
        self.node = node
        self.bridge = CvBridge()
        self.show_window = show_window
        self.publish_debug_image = publish_debug_image
        self.log_period_s = log_period_s
        # /key_points 的坐标是在检测器的输入尺寸下的；叠加到原始图像上需要按比例缩放。
        # 仿真模式下检测器尺寸 = 图像尺寸（1440x1080），此时比例为 1。
        self.detector_width = max(1, int(detector_width))
        self.detector_height = max(1, int(detector_height))

        self.latest_image = None
        self.latest_keypoints = None
        self.latest_result = AimResult()
        self.latest_angles = None

        self.image_recv_count = 0
        self.processed_count = 0
        self._fps_window_start = time.time()
        self._image_fps = 0.0
        self._process_fps = 0.0
        self._last_log = 0.0

        self.image_sub = node.create_subscription(Image, image_topic, self._on_image, 1)
        self.keypoints_sub = node.create_subscription(KeyPoints, keypoints_topic, self._on_keypoints, 5)
        self.debug_pub = node.create_publisher(Image, debug_image_topic, 5) if publish_debug_image else None
        self.timer = node.create_timer(1.0 / max(1.0, render_hz), self._render)

    # ---------------- callbacks ----------------
    def _on_image(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self.node.get_logger().warn(f"debug: cv_bridge failed: {e}")
            return
        self.image_recv_count += 1

    def _on_keypoints(self, msg: KeyPoints):
        self.latest_keypoints = msg

    def update_result(self, result: AimResult, angles=None):
        self.latest_result = result
        self.latest_angles = angles
        self.processed_count += 1

    # ---------------- rendering ----------------
    def _render(self):
        now = time.time()
        dt = now - self._fps_window_start
        if dt >= 1.0:
            self._image_fps = self.image_recv_count / dt
            self._process_fps = self.processed_count / dt
            self.image_recv_count = 0
            self.processed_count = 0
            self._fps_window_start = now

        frame = None if self.latest_image is None else self.latest_image.copy()

        if frame is not None:
            self._draw_keypoints(frame)
            self._draw_text(frame)
            if self.debug_pub is not None:
                try:
                    out = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    out.header.stamp = self.node.get_clock().now().to_msg()
                    out.header.frame_id = "debug_image"
                    self.debug_pub.publish(out)
                except Exception as e:  # noqa: BLE001
                    self.node.get_logger().warn(f"debug: publish failed: {e}")
            if self.show_window:
                try:
                    cv2.imshow("auto-aim debug", frame)
                    cv2.waitKey(1)
                except Exception as e:  # noqa: BLE001
                    self.node.get_logger().warn(f"debug: imshow failed: {e}")
                    self.show_window = False

        self._maybe_log(now)

    def _draw_keypoints(self, frame):
        kp = self.latest_keypoints
        if kp is None or len(kp.points) != 8:
            return
        # 检测器把图像缩放到固定尺寸再检测，/key_points 的坐标是在该尺寸下的，
        # 显示时按"检测尺寸 -> 当前画面尺寸"缩放（仿真模式下两者相同，比例=1）。
        h, w = frame.shape[:2]
        sx = w / float(self.detector_width)
        sy = h / float(self.detector_height)
        pts = [(int(kp.points[i] * sx), int(kp.points[i + 1] * sy)) for i in range(0, 8, 2)]
        if all(p == (0, 0) for p in pts):
            return
        color = (0, 255, 0) if self.latest_result.target_valid else (0, 165, 255)
        for p in pts:
            cv2.circle(frame, p, 3, color, -1)
        cv2.line(frame, pts[0], pts[1], color, 1)
        cv2.line(frame, pts[2], pts[3], color, 1)
        cv2.line(frame, pts[0], pts[2], color, 1)
        cv2.line(frame, pts[1], pts[3], color, 1)

    def _draw_text(self, frame):
        r = self.latest_result
        lines = [
            f"status: {r.status}   valid: {r.target_valid}",
            f"yaw_rel: {r.yaw_rel_deg:+.2f} deg   pitch_rel: {r.pitch_rel_deg:+.2f} deg",
            f"distance: {r.distance_m:.2f} m   fire: {r.fire}",
        ]
        if self.latest_angles is not None:
            yaw_cmd, pitch_cmd = self.latest_angles
            lines.append(f"cmd yaw: {yaw_cmd:+.2f} deg   cmd pitch: {pitch_cmd:.2f} deg (90=level)")
        else:
            lines.append("cmd: -  (target lost / open loop)")
        lines.append(f"recv FPS: {self._image_fps:.1f}   process FPS: {self._process_fps:.1f}")
        y = 22
        for line in lines:
            cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            y += 22

    def _maybe_log(self, now):
        if self.log_period_s <= 0.0:
            return
        if now - self._last_log < self.log_period_s:
            return
        self._last_log = now
        r = self.latest_result
        if self.latest_angles is not None:
            yaw_cmd, pitch_cmd = self.latest_angles
            cmd = f"cmd=({yaw_cmd:+.2f},{pitch_cmd:.2f})deg"
        else:
            cmd = "cmd=-"
        self.node.get_logger().info(
            f"[aim] {r.status:<9} valid={int(r.target_valid)} "
            f"yaw_rel={r.yaw_rel_deg:+6.2f} pitch_rel={r.pitch_rel_deg:+6.2f} "
            f"dist={r.distance_m:5.2f}m {cmd} "
            f"recv_fps={self._image_fps:4.1f} proc_fps={self._process_fps:4.1f}")
