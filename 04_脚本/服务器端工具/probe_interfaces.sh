#!/usr/bin/env bash
# 仿真器接口实测：话题/类型/QoS/频率/内参/图像编码（与 docs/sim_interface_record.md 交叉核对）
source /opt/ros/humble/setup.bash
source /opt/ws/bevy_robomaster_simulator/rm_ws/install/setup.bash
export DISPLAY=:0

echo "=== 节点 ==="
timeout 10 ros2 node list 2>/dev/null
echo
echo "=== 话题列表 ==="
timeout 10 ros2 topic list 2>/dev/null
echo
echo "=== /image_raw 信息 ==="
timeout 10 ros2 topic info /image_raw --verbose 2>/dev/null | head -20
echo
echo "=== /rm_gimbal/cmd 信息（QoS 应为 BEST_EFFORT）==="
timeout 10 ros2 topic info /rm_gimbal/cmd --verbose 2>/dev/null | head -20
echo
echo "=== /camera_info（内参，取一帧）==="
timeout 15 ros2 topic echo /camera_info --once 2>/dev/null | head -20
echo
echo "=== /image_raw 第一帧头部 ==="
timeout 20 ros2 topic echo /image_raw --once --field header 2>/dev/null | head -10
timeout 20 ros2 topic echo /image_raw --once --field encoding 2>/dev/null | head -3
echo
echo "=== 频率（各 6 秒）==="
for t in /image_raw /camera_info /tf; do
  echo "--- $t ---"
  timeout 8 ros2 topic hz "$t" 2>/dev/null | head -3
done
