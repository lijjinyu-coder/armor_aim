#!/usr/bin/env bash
# 启动自瞄链路（仿真模式）。图像由仿真器提供，本脚本不启动任何相机/视频节点。
# 用法: bash sim/run_aim_sim.sh [工作空间路径] [额外 launch 参数...]
#   bash sim/run_aim_sim.sh                       # 闭环：发 /rm_gimbal/cmd
#   bash sim/run_aim_sim.sh <ws> publish_command:=false   # 开环：只算不发
set -eo pipefail

WS="${1:-/opt/ws/auto-aiming}"
shift || true
export DISPLAY="${DISPLAY:-:0}"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

exec ros2 launch prm_launch sim_detector.py "$@"
