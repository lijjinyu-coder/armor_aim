#!/usr/bin/env bash
# ============================================================
# 03_build_simulator.sh
# 编译 bevy_robomaster_simulator（ros2 feature）。
# 前置条件: 已执行 02_build_rm_interfaces.sh 并 source 其 install。
#
# r2r_msg_gen 构建期要求（均已在本脚本内满足）:
#   - ROS_DISTRO / AMENT_PREFIX_PATH   (source ROS2 + rm_interfaces)
#   - libclang (bindgen 需要)
#   - IDL_PACKAGE_FILTER 限定消息生成范围，避免为全部安装包生成绑定
# ============================================================
# 注意：不能用 set -u —— ROS2 的 setup.bash 会引用未定义变量，开启 nounset 会直接失败
set -eo pipefail

SIM_ROOT="${1:-/opt/ws/bevy_robomaster_simulator}"
WS="$SIM_ROOT/rm_ws"

export PATH="/root/.cargo/bin:$PATH"
source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

export IDL_PACKAGE_FILTER="rm_interfaces;std_msgs;sensor_msgs;geometry_msgs;tf2_msgs;visualization_msgs;nav_msgs"

cd "$SIM_ROOT"

# 仅启用 ros2，关闭默认的 talos/ffmpeg 依赖
cargo build --no-default-features --features ros2

echo "[OK] 仿真器编译完成，运行:"
echo "     source /opt/ros/humble/setup.bash && source $WS/install/setup.bash"
echo "     cd $SIM_ROOT && DISPLAY=:0 cargo run --no-default-features --features ros2"
