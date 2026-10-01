"""
autoaim.py — 单入口自瞄链路，**用启动参数切换输入/输出模式**

对应任务书 §五：
    视频文件输入 ────┐                  ┌── 调试／原模拟通信输出
                    ├── 自瞄核心算法 ──┤
    ROS2 图像输入 ───┘                  └── ROS2 云台输出

用法：
    # 仿真模式：图像来自仿真器 /image_raw，内参按仿真画面，输出到 /rm_gimbal/cmd
    ros2 launch prm_launch autoaim.py input:=ros2 publish_command:=true

    # 视频模式：图像来自视频文件，内参沿用真机旧相机默认值，默认只做调试输出
    ros2 launch prm_launch autoaim.py input:=video video_path:=/path/to/xx.avi

关键参数：
    input             ros2 | video          输入源（切换输入层，核心算法不变）
    publish_command   true|false            是否发 /rm_gimbal/cmd（false = 开环，只算不发）
    target_red/color_set                     目标颜色
    image_width/height, fx/fy/cx/cy          ros2 模式的检测尺寸与内参（须与仿真器一致）
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

import math

SIM_WIDTH = 1440
SIM_HEIGHT = 1080
SIM_FOV_Y_DEG = 45.0
_FY = SIM_HEIGHT / (2.0 * math.tan(math.radians(SIM_FOV_Y_DEG) / 2.0))


def generate_launch_description():
    is_video = PythonExpression(["'", LaunchConfiguration("input"), "' == 'video'"])
    is_ros2 = PythonExpression(["'", LaunchConfiguration("input"), "' == 'ros2'"])

    args = [
        DeclareLaunchArgument("input", default_value="ros2",
                              description="输入源: ros2（仿真器图像）或 video（视频文件）"),
        DeclareLaunchArgument("publish_command", default_value="true",
                              description="true=闭环(发云台命令); false=开环(仅调试输出)"),
        DeclareLaunchArgument("video_path", default_value="",
                              description="video 模式下的视频文件路径"),
        DeclareLaunchArgument("fps", default_value="30", description="video 模式播放帧率"),
        DeclareLaunchArgument("target_red", default_value="true",
                              description="红色(true) / 蓝色(false) 装甲板"),
        DeclareLaunchArgument("color_set", default_value="red"),
        # --- 检测器（两种模式共用；仿真模式用更大的检测尺寸以匹配捕获分辨率）---
        DeclareLaunchArgument("image_width", default_value=str(SIM_WIDTH)),
        DeclareLaunchArgument("image_height", default_value=str(SIM_HEIGHT)),
        DeclareLaunchArgument("hue_range_limit", default_value="30"),
        DeclareLaunchArgument("saturation_lower_limit", default_value="100"),
        DeclareLaunchArgument("value_lower_limit", default_value="150"),
        DeclareLaunchArgument("max_missed_frames", default_value="1"),
        DeclareLaunchArgument("reduce_search_area", default_value="true"),
        # --- 仿真相机内参（仅 ros2 模式使用）---
        DeclareLaunchArgument("fx", default_value=f"{_FY:.3f}"),
        DeclareLaunchArgument("fy", default_value=f"{_FY:.3f}"),
        DeclareLaunchArgument("cx", default_value=str(SIM_WIDTH / 2.0)),
        DeclareLaunchArgument("cy", default_value=str(SIM_HEIGHT / 2.0)),
        # --- 输出层 ---
        DeclareLaunchArgument("debug_image", default_value="true"),
        DeclareLaunchArgument("show_window", default_value="false"),
        DeclareLaunchArgument("world_frame", default_value="odom"),
        DeclareLaunchArgument("camera_frame", default_value="camera_optical_frame"),
        DeclareLaunchArgument("watchdog_timeout_ms", default_value="200"),
        DeclareLaunchArgument("max_yaw_rate_deg_s", default_value="60.0"),
        DeclareLaunchArgument("max_pitch_rate_deg_s", default_value="45.0"),
        DeclareLaunchArgument("yaw_sign", default_value="1.0"),
        DeclareLaunchArgument("yaw_offset_deg", default_value="0.0"),
        DeclareLaunchArgument("pitch_offset_deg", default_value="0.0"),
    ]

    # ---- 输入层：视频文件（仅 video 模式） ----
    video_publisher = Node(
        package="webcam_publisher", executable="VideoCaptureNode", name="video_capture",
        output="screen", emulate_tty=True, condition=IfCondition(is_video),
        parameters=[{"source": LaunchConfiguration("video_path"),
                     "fps": LaunchConfiguration("fps"),
                     "frame_id": "video"}],
    )

    # ---- 核心算法层：检测 ----
    detector = Node(
        package="opencv_armor_detector", executable="OpenCVArmorDetectorNode",
        name="opencv_armor_detector", output="screen", emulate_tty=True,
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

    # ---- 核心算法层：PnP（ros2 模式用仿真内参；video 模式用节点默认=真机旧内参） ----
    pose_ros2 = Node(
        package="pose_estimator", executable="PoseEstimatorNode", name="pose_estimator",
        output="screen", emulate_tty=True, condition=IfCondition(is_ros2),
        parameters=[{
            "camera.fx": LaunchConfiguration("fx"), "camera.fy": LaunchConfiguration("fy"),
            "camera.cx": LaunchConfiguration("cx"), "camera.cy": LaunchConfiguration("cy"),
            "camera.k1": 0.0, "camera.k2": 0.0, "camera.p1": 0.0, "camera.p2": 0.0,
            # 仿真器相机与枪口基本同轴
            "cam_barrel_x": 0.0, "cam_barrel_y": 0.0, "cam_barrel_z": 0.0,
            "cam_barrel_roll": 0.0, "cam_barrel_pitch": 0.0, "cam_barrel_yaw": 0.0,
        }],
    )
    pose_video = Node(
        package="pose_estimator", executable="PoseEstimatorNode", name="pose_estimator",
        output="screen", emulate_tty=True, condition=IfCondition(is_video),
    )

    # ---- 输出抽象层 ----
    aim_output = Node(
        package="aim_output", executable="aim_output_node", name="aim_output",
        output="screen", emulate_tty=True,
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
            "debug_image": LaunchConfiguration("debug_image"),
            "show_window": LaunchConfiguration("show_window"),
            "publish_color_set": LaunchConfiguration("color_set"),
            "detector_width": LaunchConfiguration("image_width"),
            "detector_height": LaunchConfiguration("image_height"),
        }],
    )

    return LaunchDescription(args + [video_publisher, detector, pose_ros2, pose_video, aim_output])
