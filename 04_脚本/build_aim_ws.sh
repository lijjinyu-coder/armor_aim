#!/usr/bin/env bash
# 构建自瞄工作空间（仅仿真所需包）。
# 用法: bash sim/build_aim_ws.sh [工作空间路径]
#
# 为什么用白名单：仓库内 src/vision_opencv 是 Foxy 时代的 cv_bridge（Boost.Python），
# 与 Humble 系统包同名冲突；mv_publisher / prm_autobot_2023 / rplidar_ros /
# livox_ros_driver2 依赖硬件 SDK 或 navigation2，仿真用不到。
set -eo pipefail   # 注意：不能加 -u，ROS2 setup.bash 会引用未定义变量

WS="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
source /opt/ros/humble/setup.bash
cd "$WS"

PACKAGES="vision_msgs rm_interfaces opencv_armor_detector pose_estimator \
          webcam_publisher aim_output sim_tests prm_launch"

colcon build --symlink-install --packages-select $PACKAGES \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "[OK] 构建完成。使用前请先 source:"
echo "     source /opt/ros/humble/setup.bash && source $WS/install/setup.bash"
