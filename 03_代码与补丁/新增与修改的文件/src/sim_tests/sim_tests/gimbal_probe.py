#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gimbal_probe.py — 云台发布测试（任务书 §三）
向 /rm_gimbal/cmd 发布小幅、可解释的固定命令，观察仿真器云台实际响应，
验证字段单位与 yaw/pitch 正负方向。证明 "自己的程序 → 仿真器" 已连通。

关键语义（源码核实）:
  - yaw/pitch 是枪口绝对指向，单位【度】；pitch 从竖直轴量起（90=水平）
  - distance == -1.0 是"无解"哨兵，会停止云台追踪（保持姿态）
  - 仿真器里需要先按 F5 开启自瞄订阅

用法:
  ros2 run sim_tests gimbal_probe --yaw 0 --pitch 90            # 指向前方
  ros2 run sim_tests gimbal_probe --yaw 10 --pitch 90 --hold    # 偏航 +10°，持续发送
  ros2 run sim_tests gimbal_probe --yaw 0 --pitch 100 --hold    # 仰角 +10°
  ros2 run sim_tests gimbal_probe --stop                        # 发 distance=-1 停止追踪
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rm_interfaces.msg import GimbalCmd


class GimbalProbe(Node):
    def __init__(self, yaw, pitch, dist, hold, stop, rate):
        super().__init__("gimbal_probe")
        # 仿真器订阅端为 best-effort(sensor_data)，发布端必须匹配
        self.pub = self.create_publisher(GimbalCmd, "/rm_gimbal/cmd", qos_profile_sensor_data)
        self.yaw, self.pitch, self.dist = yaw, pitch, dist
        self.hold, self.stop, self.rate = hold, stop, rate
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f"publishing /rm_gimbal/cmd: yaw={yaw}deg pitch={pitch}deg "
            f"distance={dist} hold={hold} stop={stop} rate={rate}Hz"
        )

    def tick(self):
        msg = GimbalCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        if self.stop:
            msg.distance = -1.0  # 无解哨兵：停止追踪
        else:
            msg.yaw = float(self.yaw)
            msg.pitch = float(self.pitch)
            msg.yaw_diff = 0.0
            msg.pitch_diff = 0.0
            msg.distance = float(self.dist)
            msg.fire_advice = False
        self.pub.publish(msg)
        if not self.hold and not self.stop:
            self.get_logger().info("sent one command, exiting")
            self.timer.cancel()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--yaw", type=float, default=0.0, help="绝对 yaw（度）")
    parser.add_argument("--pitch", type=float, default=90.0, help="绝对 pitch（度，90=水平）")
    parser.add_argument("--distance", type=float, default=3.0, help="距离（米），-1=无解")
    parser.add_argument("--rate", type=float, default=10.0, help="发布频率 Hz")
    parser.add_argument("--hold", action="store_true", help="持续发送直到 Ctrl-C")
    parser.add_argument("--stop", action="store_true", help="发送 distance=-1 停止追踪")
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    node = GimbalProbe(args.yaw, args.pitch, args.distance, args.hold, args.stop, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
