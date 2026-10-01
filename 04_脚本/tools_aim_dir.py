#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""打印某 frame 的世界姿态，以及其 +X 轴（瞄准方向）在 odom 下的方位角/仰角。

用法: python3 aim_dir.py [frame] [world]
输出: yaw/pitch（云的瞄准方向）+ 等效的仿真器绝对角（yaw_deg, pitch_from_vertical）
"""
import math
import sys

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

import numpy as np


class AimDir(Node):
    def __init__(self, frame, world):
        super().__init__("aim_dir")
        self.frame, self.world = frame, world
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.create_timer(0.2, self.tick)
        self.done = False

    def tick(self):
        if self.done:
            return
        try:
            tf = self.buf.lookup_transform(self.world, self.frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return
        q = tf.transform.rotation
        r, p, y = [math.degrees(a) for a in euler_from_quaternion([q.x, q.y, q.z, q.w])]
        # 旋转矩阵 -> +X 轴方向（ROS: x 前 / y 左 / z 上）
        x, yq, z, w = q.x, q.y, q.z, q.w
        m = np.array([
            [1 - 2 * (yq * yq + z * z), 2 * (x * yq - z * w), 2 * (x * z + yq * w)],
            [2 * (x * yq + z * w), 1 - 2 * (x * x + z * z), 2 * (yq * z - x * w)],
            [2 * (x * z - yq * w), 2 * (yq * z + x * w), 1 - 2 * (x * x + yq * yq)],
        ])
        d = m @ np.array([1.0, 0.0, 0.0])
        yaw_deg = math.degrees(math.atan2(d[1], d[0]))
        theta = math.degrees(math.acos(max(-1.0, min(1.0, d[2]))))  # 与 +Z 夹角
        pitch_from_vertical = 180.0 - theta                          # 90=水平
        t = tf.transform.translation
        print(f"{self.world}<-{self.frame} rpy=({r:+.2f},{p:+.2f},{y:+.2f}) "
              f"aim_dir=({d[0]:+.3f},{d[1]:+.3f},{d[2]:+.3f}) "
              f"aim_yaw={yaw_deg:+.2f} aim_pitch_from_vertical={pitch_from_vertical:+.2f} "
              f"pos=({t.x:+.3f},{t.y:+.3f},{t.z:+.3f})", flush=True)
        self.done = True


def main():
    frame = sys.argv[1] if len(sys.argv) > 1 else "camera_link"
    world = sys.argv[2] if len(sys.argv) > 2 else "odom"
    rclpy.init()
    node = AimDir(frame, world)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
