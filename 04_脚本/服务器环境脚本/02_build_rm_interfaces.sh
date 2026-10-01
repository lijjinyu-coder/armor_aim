#!/usr/bin/env bash
# ============================================================
# 02_build_rm_interfaces.sh
# 用 colcon 构建仿真器自定义消息包 rm_interfaces。
# 必须在编译仿真器 (ros2 feature) 之前执行，因为 r2r_msg_gen
# 需要从 AMENT_PREFIX_PATH 中找到 rm_interfaces 的
# .msg 文件 / 生成的头文件 / 动态库。
# ============================================================
# 注意：不能用 set -u —— ROS2 的 setup.bash 会引用未定义变量，开启 nounset 会直接失败
set -eo pipefail

SIM_ROOT="${1:-/opt/ws/bevy_robomaster_simulator}"
INTERFACES_DIR="$SIM_ROOT/rm_interfaces"
WS="$SIM_ROOT/rm_ws"

source /opt/ros/humble/setup.bash

if [ ! -f "$INTERFACES_DIR/package.xml" ]; then
    echo "[ERR] 找不到 $INTERFACES_DIR/package.xml"
    exit 1
fi

cd "$INTERFACES_DIR"
colcon build --base-paths . --build-base "$WS/build" \
             --install-base "$WS/install" --merge-install

echo "[OK] rm_interfaces 已构建。编译仿真器前执行:"
echo "     source $WS/install/setup.bash"
