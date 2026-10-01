#!/usr/bin/env bash
# 启动仿真器（需要已构建，且 ROS2 环境与 rm_interfaces 可用）。
# 用法: bash sim/run_sim.sh [仿真器目录] [DISPLAY]
#
# 提醒：启动后必须在仿真器窗口按 F5 打开"自瞄订阅"开关，否则 /rm_gimbal/cmd 不会被处理。
set -eo pipefail

SIM="${1:-/opt/ws/bevy_robomaster_simulator}"
export DISPLAY="${2:-:0}"
export PATH="/root/.cargo/bin:$PATH"

source /opt/ros/humble/setup.bash
if [ -f "$SIM/rm_ws/install/setup.bash" ]; then
    source "$SIM/rm_ws/install/setup.bash"
fi
export IDL_PACKAGE_FILTER="rm_interfaces;std_msgs;sensor_msgs;geometry_msgs;tf2_msgs;visualization_msgs;nav_msgs"

cd "$SIM"
echo "[..] 启动仿真器 (DISPLAY=$DISPLAY)；按 F5 开启自瞄订阅"
exec cargo run --no-default-features --features ros2
