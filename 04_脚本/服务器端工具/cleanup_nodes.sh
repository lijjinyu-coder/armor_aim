#!/usr/bin/env bash
# 清理所有自瞄侧残留 ROS 节点（ros2 launch 的子节点不会被 pkill launch 进程带走）
echo "=== 清理前 ==="
pgrep -af 'aim_output_node|opencv_armor_detector|pose_estimator|video2detector|image_probe|gimbal_probe|ros2 launch|sim_detector' | head -12

pkill -f 'aim_output_node' 2>/dev/null
pkill -f 'opencv_armor_detector' 2>/dev/null
pkill -f 'PoseEstimatorNode' 2>/dev/null
pkill -f 'sim_detector.py' 2>/dev/null
pkill -f 'image_probe' 2>/dev/null
pkill -f 'gimbal_probe' 2>/dev/null
sleep 3
# 二次确认并强杀
for pat in aim_output_node opencv_armor_detector PoseEstimatorNode sim_detector.py; do
  pgrep -f "$pat" | while read -r p; do kill -9 "$p" 2>/dev/null; done
done
sleep 1

echo "=== 清理后 ==="
LEFT=$(pgrep -af 'aim_output_node|opencv_armor_detector|PoseEstimatorNode|sim_detector' | wc -l)
echo "剩余自瞄节点进程数: $LEFT"
pgrep -af 'daedalus' | head -2

echo "=== 等仿真器恢复帧率（10s）==="
sleep 10
tail -3 /opt/ws/sim_run.log | grep -E 'fps|frame_time' || true
