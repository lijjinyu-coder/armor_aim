#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""读取 /tf 中若干 frame 的 RPY 并退出（用于验证云台命令方向）。

用法: python3 tf_rpy.py [target_frame] [source_frame]
默认 odom <- gimbal_link
"""
import sys
import math

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion


class TfRpy(Node):
    def __init__(self, target, source):
        super().__init__("tf_rpy")
        self.target = target
        self.source = source
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.create_timer(0.2, self.tick)
        self.done = False

    def tick(self):
        if self.done:
            return
        try:
            tf = self.buf.lookup_transform(self.target, self.source, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"lookup failed: {e}", throttle_duration_sec=2.0)
            return
        q = tf.transform.rotation
        roll, pitch, yaw = [math.degrees(a) for a in
                            euler_from_quaternion([q.x, q.y, q.z, q.w])]
        t = tf.transform.translation
        print(f"{self.target} <- {self.source}: "
              f"roll={roll:+.2f} pitch={pitch:+.2f} yaw={yaw:+.2f} deg | "
              f"xyz=({t.x:+.3f},{t.y:+.3f},{t.z:+.3f})", flush=True)
        self.done = True


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else "odom"
    source = sys.argv[2] if len(sys.argv) > 2 else "gimbal_link"
    rclpy.init()
    node = TfRpy(target, source)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
