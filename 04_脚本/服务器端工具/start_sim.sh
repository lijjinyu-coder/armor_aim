#!/usr/bin/env bash
# 启动仿真器（后台，日志到 /opt/ws/sim_run.log），并等待 ROS2 话题出现。
SIM=/opt/ws/bevy_robomaster_simulator
source /opt/ros/humble/setup.bash
source "$SIM/rm_ws/install/setup.bash"
export DISPLAY=:0
export PATH=/root/.cargo/bin:$PATH
export IDL_PACKAGE_FILTER="rm_interfaces;std_msgs;sensor_msgs;geometry_msgs;tf2_msgs;visualization_msgs;nav_msgs"

pkill -f 'target/debug/daedalus' 2>/dev/null || true
sleep 1

cd "$SIM"
# 用 cargo run 启动（bevy 开了 dynamic_linking，直接执行二进制可能找不到 Bevy 动态库）
nohup cargo run --no-default-features --features ros2 > /opt/ws/sim_run.log 2>&1 &
echo "simulator launcher pid=$!"
sleep 20
echo "=== 进程 ==="
pgrep -af 'target/debug/daedalus' | head -3
echo "=== 日志 ==="
tail -20 /opt/ws/sim_run.log
echo "=== Vulkan ==="
vulkaninfo --summary 2>/dev/null | grep -E 'deviceName|driverName|apiVersion' | head -6
echo "=== 话题 ==="
timeout 10 ros2 topic list 2>/dev/null | head -20
