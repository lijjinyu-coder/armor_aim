#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""列出仿真器中所有装甲板位置，并计算从"己方云台"看过去的 yaw/仰角。

用于闭环演示时把云台先对准某个敌人（也可用于目标选择调试）。

用法: python3 list_armors.py [持续时间秒]
"""
import math
import sys

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker


class ArmorList(Node):
    def __init__(self, duration):
        super().__init__("list_armors")
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.armors = {}
        self.create_subscription(Marker, "/simulator/marker", self.cb, 10)
        self.duration = duration
        self.t0 = self.get_clock().now()

    def cb(self, m: Marker):
        if m.ns != "armors" or m.action != Marker.ADD:
            return
        p = m.pose.position
        self.armors[m.id] = (p.x, p.y, p.z, m.header.frame_id)

    def report(self):
        # 己方位置：map -> odom 的平移即云台位置
        try:
            t = self.buf.lookup_transform("map", "odom", rclpy.time.Time())
            px, py, pz = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
        except Exception:  # noqa: BLE001
            px = py = pz = 0.0
        print(f"己方云台位置 (map): ({px:+.3f}, {py:+.3f}, {pz:+.3f})", flush=True)
        print(f"装甲板数量: {len(self.armors)}", flush=True)
        rows = []
        for i, (x, y, z, frame) in self.armors.items():
            dx, dy, dz = x - px, y - py, z - pz
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            yaw = math.degrees(math.atan2(dy, dx))
            elev = math.degrees(math.asin(dz / dist)) if dist > 0 else 0.0
            rows.append((dist, i, x, y, z, yaw, elev))
        rows.sort()
        for dist, i, x, y, z, yaw, elev in rows[:12]:
            print(f"  armor id={i:<3} pos=({x:+.2f},{y:+.2f},{z:+.2f}) dist={dist:5.2f}m "
                  f"-> 需要 yaw={yaw:+7.2f} pitch(仰角)={elev:+6.2f}", flush=True)
        # 机器可读行：最近的“非己方”（>0.5m）装甲板，供脚本直接取用
        for dist, i, x, y, z, yaw, elev in rows:
            if dist > 0.5:
                print(f"TARGET yaw={yaw:.2f} pitch={elev:.2f} dist={dist:.2f} id={i}", flush=True)
                break


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
    rclpy.init()
    node = ArmorList(duration)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        if (node.get_clock().now() - node.t0).nanoseconds / 1e9 > duration:
            break
    node.report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
