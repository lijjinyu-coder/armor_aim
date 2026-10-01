#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""保存一帧 /image_raw 到文件（用于检查仿真器相机画面内容）。

用法: python3 save_frame.py <输出路径> [话题]
"""
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class SaveFrame(Node):
    def __init__(self, out, topic):
        super().__init__("save_frame")
        self.bridge = CvBridge()
        self.out = out
        self.saved = False
        self.create_subscription(Image, topic, self.cb, 5)

    def cb(self, msg):
        if self.saved:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"convert failed: {e}")
            return
        cv2.imwrite(self.out, img)
        print(f"saved {self.out} {img.shape} encoding_in={msg.encoding}", flush=True)
        self.saved = True


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/frame.jpg"
    topic = sys.argv[2] if len(sys.argv) > 2 else "/image_raw"
    rclpy.init()
    node = SaveFrame(out, topic)
    while rclpy.ok() and not node.saved:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
