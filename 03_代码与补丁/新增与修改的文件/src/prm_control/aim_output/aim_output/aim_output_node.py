#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
aim_output_node.py — 输出抽象节点（任务书 §五、§六）

数据流：
    /predicted_armor ──► AimResult ──┬──► 调试输出（日志 + /debug_image）
                                    └──► ROS2 输出（/rm_gimbal/cmd，绝对角/度）

设计要点：
  - 核心算法（检测器/PnP）零改动，本节点只做"结果 -> 通信消息"的适配
  - 每帧用最新 TF（world ← camera_optical_frame）把目标方向转到世界系，再转成
    仿真器要求的绝对云台角，因此误差不累积
  - 看门狗：超过 watchdog_timeout_ms 没有新的 predicted_armor，或解为全 0，
    发 distance=-1（仿真器无解哨兵）→ 停止追踪并保持姿态，不使用过期目标
"""
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from vision_msgs.msg import PredictedArmor

from .aim_result import AimResult, aim_result_from_armor_xyz, quaternion_to_matrix
from .debug_output import DebugOutput
from .gimbal_output import GimbalOutput

# 浮点参数用 dynamic_typing：`ros2 launch ... x:=20` 会被当成 INTEGER，
# 若参数声明为 double 会直接抛 InvalidParameterTypeException 把节点打死（踩过的坑）。
_DYNAMIC = ParameterDescriptor(dynamic_typing=True)


def _f(node, name: str) -> float:
    """读取数值参数并强制转成 float（兼容 INTEGER/DOUBLE 两种传参）。"""
    return float(node.get_parameter(name).value)


class AimOutputNode(Node):
    def __init__(self):
        super().__init__("aim_output")

        # ---- parameters ----
        self.declare_parameter("input_topic", "/predicted_armor")
        self.declare_parameter("gimbal_cmd_topic", "/rm_gimbal/cmd")
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("camera_frame", "camera_optical_frame")
        self.declare_parameter("publish_command", True)
        self.declare_parameter("watchdog_timeout_ms", 200, _DYNAMIC)
        self.declare_parameter("yaw_sign", 1.0, _DYNAMIC)
        self.declare_parameter("yaw_offset_deg", 0.0, _DYNAMIC)
        self.declare_parameter("pitch_offset_deg", 0.0, _DYNAMIC)
        # 指令速率限制（度/秒），0 = 关闭。视觉链路有延迟，限制变化率可避免过冲振荡。
        self.declare_parameter("max_yaw_rate_deg_s", 60.0, _DYNAMIC)
        self.declare_parameter("max_pitch_rate_deg_s", 45.0, _DYNAMIC)
        # 控制增益（0~1）与死区（度）：延迟主导的闭环里减小增益可避免过冲
        self.declare_parameter("control_gain", 0.4, _DYNAMIC)
        self.declare_parameter("deadband_deg", 1.5, _DYNAMIC)
        self.declare_parameter("debug_image", True)
        self.declare_parameter("show_window", False)
        self.declare_parameter("log_period_s", 1.0, _DYNAMIC)
        self.declare_parameter("publish_color_set", "")
        # 检测器输入尺寸（用于把 /key_points 角点正确叠加到原始图像上）
        self.declare_parameter("detector_width", 1280)
        self.declare_parameter("detector_height", 1024)

        p = self.get_parameter
        self.input_topic = p("input_topic").value
        self.world_frame = p("world_frame").value
        self.camera_frame = p("camera_frame").value
        self.watchdog_timeout_ms = _f(self, "watchdog_timeout_ms")
        self.publish_command = bool(p("publish_command").value)

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- output adapters ----
        self.gimbal_output = GimbalOutput(
            self,
            topic=p("gimbal_cmd_topic").value,
            publish_command=self.publish_command,
            yaw_sign=_f(self, "yaw_sign"),
            yaw_offset_deg=_f(self, "yaw_offset_deg"),
            pitch_offset_deg=_f(self, "pitch_offset_deg"),
            max_yaw_rate_deg_s=_f(self, "max_yaw_rate_deg_s"),
            max_pitch_rate_deg_s=_f(self, "max_pitch_rate_deg_s"),
            control_gain=_f(self, "control_gain"),
            deadband_deg=_f(self, "deadband_deg"),
        )
        self.debug_output = DebugOutput(
            self,
            debug_image_topic="/debug_image",
            show_window=bool(p("show_window").value),
            publish_debug_image=bool(p("debug_image").value),
            log_period_s=_f(self, "log_period_s"),
            detector_width=int(p("detector_width").value),
            detector_height=int(p("detector_height").value),
        )

        # ---- input ----
        self.sub = self.create_subscription(PredictedArmor, self.input_topic,
                                            self.on_predicted_armor, 1)

        # ---- optional: tell the detector which enemy colour to look for ----
        color = str(p("publish_color_set").value or "")
        self.color_pub = None
        if color:
            self.color_pub = self.create_publisher(String, "color_set", 1)
            msg = String()
            msg.data = color
            self.color_pub.publish(msg)
            self.get_logger().info(f"published color_set='{color}'")

        # ---- watchdog ----
        self.last_msg_time = None
        self.stop_sent = False
        self.last_status = "IDLING"
        self.create_timer(0.05, self.watchdog)

        self.get_logger().info(
            f"aim_output started: {self.input_topic} -> {self.gimbal_output.publisher.topic_name} | "
            f"tf {self.world_frame} <- {self.camera_frame} | "
            f"publish_command={self.publish_command} watchdog={self.watchdog_timeout_ms:.0f}ms")

    # ------------------------------------------------------------------
    def _camera_to_world(self):
        """相机光学系 -> 世界系的旋转矩阵（3x3）。取不到 TF 时返回 None。"""
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.world_frame, self.camera_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF {self.world_frame} <- {self.camera_frame} unavailable: {e}",
                                   throttle_duration_sec=5.0)
            return None
        q = tf.transform.rotation
        return quaternion_to_matrix(q.x, q.y, q.z, q.w)

    def on_predicted_armor(self, msg: PredictedArmor):
        self.last_msg_time = self.get_clock().now()

        status = "NO_ARMOR" if (msg.x == 0.0 and msg.y == 0.0 and msg.z == 0.0) else "TRACKING"
        if msg.fire:
            status = "FIRE"
        self.last_status = status

        result = aim_result_from_armor_xyz(msg.x, msg.y, msg.z, msg.fire, status)
        angles = None

        if result.target_valid:
            # 检测/解算有效；只有"命令通路"取决于 TF 是否可用
            rot = self._camera_to_world()
            if rot is not None:
                angles = self.gimbal_output.publish_result(result, rot, header=msg.header)
                if angles is None:
                    self.last_status = "CMD_INVALID"
                    result.status = "CMD_INVALID"
            else:
                # 无 TF 时无法得到绝对角：不发命令（避免误驱动云台），仅调试输出
                self.last_status = "TF_MISSING"
                result.status = "TF_MISSING"
        else:
            self.gimbal_output.publish_stop(header=msg.header)

        self.stop_sent = not result.target_valid
        self.debug_output.update_result(result, angles)

    def watchdog(self):
        """超时保护：目标丢失/图像中断时不持续使用过期结果。"""
        if self.last_msg_time is None:
            return
        age_ms = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e6
        if age_ms > self.watchdog_timeout_ms:
            if not self.stop_sent:
                self.get_logger().warn(
                    f"no predicted_armor for {age_ms:.0f} ms -> sending 'no solution' (distance=-1)")
                self.gimbal_output.publish_stop()
                self.stop_sent = True
                self.last_status = "TIMEOUT"
                stale = AimResult(0.0, 0.0, 0.0, False, False, "TIMEOUT")
                self.debug_output.update_result(stale, None)


def main(args=None):
    rclpy.init(args=args)
    node = AimOutputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
