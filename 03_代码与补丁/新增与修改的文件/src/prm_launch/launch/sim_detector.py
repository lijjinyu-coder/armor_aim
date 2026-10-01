"""
sim_detector.py — 仿真实时图像 → 原自瞄算法 → 统一 AimResult → 调试输出 + ROS2 云台输出

对应任务书 §三/§四/§五/§六：仿真器直接发布 /image_raw（rgb8），本 launch 不启动任何
相机/视频节点，图像由输入层（仿真器）提供，核心算法与真机完全一致。

默认内参与仿真器匹配（避免沿用旧视频参数）：
  仿真器 config.toml: [camera].fov = 45°（垂直视场角）, [capture.color] = 1440x1080
  fx = fy = H / (2*tan(fov_y/2)) = 1080 / (2*tan(22.5°)) ≈ 1303.70
  cx = W/2 = 720, cy = H/2 = 540, 零畸变（仿真器 /camera_info 为 plumb_bob 且 d 全 0）
  检测器也把图像缩放到 1440x1080（image.width/height），因此不做任何非等比缩放。

启动前请先在仿真器窗口按 F5 打开"自瞄订阅"开关（否则云台命令不生效）。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 与仿真器 config.toml 对应的默认值
SIM_WIDTH = 1440
SIM_HEIGHT = 1080
SIM_FOV_Y_DEG = 45.0

import math  # noqa: E402

_FY = SIM_HEIGHT / (2.0 * math.tan(math.radians(SIM_FOV_Y_DEG) / 2.0))


def generate_launch_description():
    args = [
        DeclareLaunchArgument("publish_command", default_value="true",
                              description="true=闭环(发云台命令); false=开环(仅调试输出)"),
        DeclareLaunchArgument("show_window", default_value="false",
                              description="在显示器上弹出 OpenCV 调试窗口"),
        DeclareLaunchArgument("debug_image", default_value="true",
                              description="是否发布 /debug_image 标注图"),
        DeclareLaunchArgument("target_red", default_value="true",
                              description="检测红色(true)还是蓝色(false)装甲板"),
        # 检测器调参（仿真画面比真机偏暗/对比度低，必要时放宽阈值）
        DeclareLaunchArgument("hue_range_limit", default_value="30",
                              description="HSV 色相容差"),
        DeclareLaunchArgument("saturation_lower_limit", default_value="100",
                              description="HSV 饱和度下限（仿真偏暗时可降到 50~70）"),
        DeclareLaunchArgument("value_lower_limit", default_value="150",
                              description="HSV 亮度下限（仿真偏暗时可降到 80~120）"),
        DeclareLaunchArgument("max_missed_frames", default_value="1",
                              description="连续丢失多少帧后重置搜索区（调大可提高连续性）"),
        DeclareLaunchArgument("reduce_search_area", default_value="true"),
        DeclareLaunchArgument("color_set", default_value="red",
                              description="发布一次 color_set 告知检测器目标颜色（red/blue）"),
        DeclareLaunchArgument("image_width", default_value=str(SIM_WIDTH)),
        DeclareLaunchArgument("image_height", default_value=str(SIM_HEIGHT)),
        DeclareLaunchArgument("fx", default_value=f"{_FY:.3f}"),
        DeclareLaunchArgument("fy", default_value=f"{_FY:.3f}"),
        DeclareLaunchArgument("cx", default_value=str(SIM_WIDTH / 2.0)),
        DeclareLaunchArgument("cy", default_value=str(SIM_HEIGHT / 2.0)),
        DeclareLaunchArgument("world_frame", default_value="odom",
                              description="TF 世界系（仿真器 map→odom 仅平移，旋转一致）"),
        DeclareLaunchArgument("camera_frame", default_value="camera_optical_frame"),
        DeclareLaunchArgument("yaw_sign", default_value="1.0"),
        DeclareLaunchArgument("yaw_offset_deg", default_value="0.0"),
        DeclareLaunchArgument("pitch_offset_deg", default_value="0.0"),
        DeclareLaunchArgument("watchdog_timeout_ms", default_value="200"),
        DeclareLaunchArgument("max_yaw_rate_deg_s", default_value="60.0",
                              description="指令 yaw 速率限制(度/秒)，0=关闭；避免视觉延迟导致过冲"),
        DeclareLaunchArgument("max_pitch_rate_deg_s", default_value="45.0",
                              description="指令 pitch 速率限制(度/秒)，0=关闭"),
        DeclareLaunchArgument("control_gain", default_value="0.4",
                              description="闭环控制增益(0~1)：延迟大时减小可避免过冲"),
        DeclareLaunchArgument("deadband_deg", default_value="1.5",
                              description="瞄准死区(度)：误差小于该值即保持不动"),
    ]

    detector = Node(
        package="opencv_armor_detector",
        executable="OpenCVArmorDetectorNode",
        name="opencv_armor_detector",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "_target_red": LaunchConfiguration("target_red"),
            "_hue_range_limit": LaunchConfiguration("hue_range_limit"),
            "_saturation_lower_limit": LaunchConfiguration("saturation_lower_limit"),
            "_value_lower_limit": LaunchConfiguration("value_lower_limit"),
            "_max_missed_frames": LaunchConfiguration("max_missed_frames"),
            "_reduce_search_area": LaunchConfiguration("reduce_search_area"),
            "image.width": LaunchConfiguration("image_width"),
            "image.height": LaunchConfiguration("image_height"),
        }],
    )

    pose_estimator = Node(
        package="pose_estimator",
        executable="PoseEstimatorNode",
        name="pose_estimator",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "camera.fx": LaunchConfiguration("fx"),
            "camera.fy": LaunchConfiguration("fy"),
            "camera.cx": LaunchConfiguration("cx"),
            "camera.cy": LaunchConfiguration("cy"),
            "camera.k1": 0.0,
            "camera.k2": 0.0,
            "camera.p1": 0.0,
            "camera.p2": 0.0,
            # 仿真器相机与枪口基本同轴，先按 0 处理；实测有偏差再调
            "cam_barrel_x": 0.0,
            "cam_barrel_y": 0.0,
            "cam_barrel_z": 0.0,
            "cam_barrel_roll": 0.0,
            "cam_barrel_pitch": 0.0,
            "cam_barrel_yaw": 0.0,
        }],
    )

    aim_output = Node(
        package="aim_output",
        executable="aim_output_node",
        name="aim_output",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "input_topic": "/predicted_armor",
            "gimbal_cmd_topic": "/rm_gimbal/cmd",
            "world_frame": LaunchConfiguration("world_frame"),
            "camera_frame": LaunchConfiguration("camera_frame"),
            "publish_command": LaunchConfiguration("publish_command"),
            "yaw_sign": LaunchConfiguration("yaw_sign"),
            "yaw_offset_deg": LaunchConfiguration("yaw_offset_deg"),
            "pitch_offset_deg": LaunchConfiguration("pitch_offset_deg"),
            "watchdog_timeout_ms": LaunchConfiguration("watchdog_timeout_ms"),
            "max_yaw_rate_deg_s": LaunchConfiguration("max_yaw_rate_deg_s"),
            "max_pitch_rate_deg_s": LaunchConfiguration("max_pitch_rate_deg_s"),
            "control_gain": LaunchConfiguration("control_gain"),
            "deadband_deg": LaunchConfiguration("deadband_deg"),
            "debug_image": LaunchConfiguration("debug_image"),
            "show_window": LaunchConfiguration("show_window"),
            "publish_color_set": LaunchConfiguration("color_set"),
            # 调试叠加层需要知道检测器输入尺寸，才能把 /key_points 画到原始图像上
            "detector_width": LaunchConfiguration("image_width"),
            "detector_height": LaunchConfiguration("image_height"),
        }],
    )

    return LaunchDescription(args + [detector, pose_estimator, aim_output])
