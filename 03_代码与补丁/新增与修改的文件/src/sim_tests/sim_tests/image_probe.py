#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
image_probe.py — 图像订阅测试（任务书 §三）
订阅仿真器 /image_raw 与 /camera_info，打印宽高/编码/时间戳/接收 FPS，并显示画面。
证明 "仿真器 → 自己的程序" 已连通。

用法:
  ros2 run sim_tests image_probe [--display 0|1] [--topic /image_raw]
"""
import sys
import time
import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class ImageProbe(Node):
    def __init__(self, topic: str, display: bool):
        super().__init__("image_probe")
        self.bridge = CvBridge()
        self.display = display
        self.count = 0
        self.last_log = time.time()
        self.window_log = 0.0
        self.sub = self.create_subscription(Image, topic, self.image_cb, 5)
        self.ci_sub = self.create_subscription(CameraInfo, "/camera_info", self.camera_info_cb, 5)
        self.get_logger().info(f"subscribing {topic} and /camera_info ...")

    def camera_info_cb(self, msg: CameraInfo):
        # 只打印一次内参（内参不变）
        if getattr(self, "_ci_printed", False):
            return
        self._ci_printed = True
        fx, fy = msg.k[0], msg.k[4]
        cx, cy = msg.k[2], msg.k[5]
        self.get_logger().info(
            f"[camera_info] {msg.width}x{msg.height} fx={fx:.2f} fy={fy:.2f} "
            f"cx={cx:.2f} cy={cy:.2f} dist={msg.distortion_model} d={list(msg.d)}"
        )

    def image_cb(self, msg: Image):
        self.count += 1
        now = time.time()
        if now - self.last_log >= 2.0:
            fps = self.count / (now - self.window_log)
            stamp = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
            self.get_logger().info(
                f"[image] {msg.width}x{msg.height} encoding={msg.encoding} "
                f"step={msg.step} frame_id={msg.header.frame_id} stamp={stamp} "
                f"recv_fps={fps:.1f}"
            )
            self.last_log = now
            self.window_log = now
            self.count = 0
        if self.display:
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                import cv2
                cv2.imshow("image_probe", cv_img)
                cv2.waitKey(1)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"display failed: {e}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/image_raw")
    parser.add_argument("--display", type=int, default=0, choices=[0, 1])
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    node = ImageProbe(args.topic, bool(args.display))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
