"""
video2detector.py — 视频文件 → 自瞄算法 → 统一 AimResult → 调试输出（原链路保留）

对应任务书 §五验收第一条链路：
    视频输入 → 自瞄算法 → 调试／原模拟通信输出

默认不开云台输出（publish_command=false），即原有的"离线视频调试"行为；
需要时可用 publish_command:=true 让它也发 /rm_gimbal/cmd。
相机内参默认沿用真机旧相机值（在 pose_estimator 内部硬编码为默认参数），
仿真模式请用 sim_detector.py（那里会覆盖为与仿真画面对应的内参）。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "video_path",
            default_value="/opt/ws/auto-aiming/src/prm_vision/opencv_armor_detector/"
                          "test/resources/close_video/close.avi",
            description="输入视频文件路径"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("frame_id", default_value="video"),
        DeclareLaunchArgument("target_red", default_value="true"),
        DeclareLaunchArgument("publish_command", default_value="false",
                              description="true=同时发布 /rm_gimbal/cmd（仿真用）"),
        DeclareLaunchArgument("show_window", default_value="false"),
        DeclareLaunchArgument("debug_image", default_value="true"),
    ]

    video_publisher = Node(
        package="webcam_publisher",
        executable="VideoCaptureNode",
        name="video_capture",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "source": LaunchConfiguration("video_path"),
            "fps": LaunchConfiguration("fps"),
            "frame_id": LaunchConfiguration("frame_id"),
        }],
    )

    detector = Node(
        package="opencv_armor_detector",
        executable="OpenCVArmorDetectorNode",
        name="opencv_armor_detector",
        output="screen",
        emulate_tty=True,
        parameters=[{"_target_red": LaunchConfiguration("target_red")}],
    )

    pose_estimator = Node(
        package="pose_estimator",
        executable="PoseEstimatorNode",
        name="pose_estimator",
        output="screen",
        emulate_tty=True,
    )

    # 输出层：视频模式下默认只做调试输出（不驱动云台）
    aim_output = Node(
        package="aim_output",
        executable="aim_output_node",
        name="aim_output",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "publish_command": LaunchConfiguration("publish_command"),
            "debug_image": LaunchConfiguration("debug_image"),
            "show_window": LaunchConfiguration("show_window"),
            "log_period_s": 1.0,
        }],
    )

    return LaunchDescription(args + [video_publisher, detector, pose_estimator, aim_output])
