#!/usr/bin/env bash
# run_scenarios_v2.sh —— 运动场景测试（任务书 §七 第2、3类）
#   A) 目标运动：假人横向移动（J/L）
#   B) 己方运动：底盘 WASD 移动 + Q/E 旋转
source /opt/ros/humble/setup.bash
source /opt/ws/auto-aiming/install/setup.bash
source /opt/ws/bevy_robomaster_simulator/rm_ws/install/setup.bash
export DISPLAY=:0
REC=/opt/ws/recordings
mkdir -p "$REC"

pgrep -x openbox >/dev/null || { openbox --sm-disable >/dev/null 2>&1 & sleep 2; }
WIN=$(xdotool search --name daedalus | head -1)
[ -n "$WIN" ] && xdotool windowfocus --sync "$WIN" 2>/dev/null

kill_aim_nodes() {
  pkill -f 'aim_output_node' 2>/dev/null; pkill -f 'opencv_armor_detector' 2>/dev/null
  pkill -f 'PoseEstimatorNode' 2>/dev/null; pkill -f 'sim_detector.py' 2>/dev/null
  pkill -f 'gimbal_probe' 2>/dev/null
  sleep 2
  for pat in aim_output_node opencv_armor_detector PoseEstimatorNode sim_detector.py; do
    pgrep -f "$pat" | while read -r p; do kill -9 "$p" 2>/dev/null; done
  done
  sleep 1
}
yawv() { python3 /tmp/tf_rpy.py odom gimbal_link 2>/dev/null | tail -1 | grep -o 'yaw=[^ ]*' | tr -d ' ' | sed 's/yaw=//'; }
auto_on() {
  local cur target after
  cur=$(yawv); [ -z "$cur" ] && cur=0
  target=$(python3 -c "print(round((($cur + 50 + 180) % 360) - 180, 2))")
  timeout 5 ros2 run sim_tests gimbal_probe --yaw "$target" --pitch 0 --distance 1.4 --hold --rate 20 >/dev/null 2>&1 &
  sleep 3.5; after=$(yawv)
  timeout 2 ros2 run sim_tests gimbal_probe --stop >/dev/null 2>&1; sleep 0.6
  python3 -c "import sys;t=float('$target');a=float('$after' or 0);print(0 if abs((a-t+180)%360-180)<2 else 1)" >/dev/null 2>&1
}
ensure_auto_on() {
  for i in 1 2 3; do
    if auto_on; then echo "[OK] 自瞄已开启"; return 0; fi
    xdotool key --window "$WIN" --clearmodifiers F5 2>/dev/null || xdotool key --clearmodifiers F5
    sleep 2
  done
  echo "[!!] 自瞄未开启"; return 1
}
aim_at() {
  timeout 6 ros2 run sim_tests gimbal_probe --yaw "$1" --pitch "$2" --distance 1.4 --hold --rate 20 >/dev/null 2>&1 &
  sleep 5; timeout 2 ros2 run sim_tests gimbal_probe --stop >/dev/null 2>&1; sleep 1
}
ratio() { local t f; t=$(grep -c '\[aim\]' "$1" 2>/dev/null||true); t=${t:-0}; f=$(grep -c 'valid=1' "$1" 2>/dev/null||true); f=${f:-0}; [ "$t" -eq 0 ] && echo 0 || echo $(( f*100/t )); }
start_aim() { nohup ros2 launch prm_launch sim_detector.py target_red:=false publish_command:=true \
    show_window:=true debug_image:=false color_set:=blue control_gain:=0.4 deadband_deg:=1.5 > "$1" 2>&1 & }

# 目标方位
kill_aim_nodes
python3 /tmp/list_armors.py 6 > /tmp/armors.txt 2>&1
LINE=$(grep '^TARGET ' /tmp/armors.txt | head -1)
TYAW=$(echo "$LINE" | sed -n 's/.*yaw=\([-+0-9.]*\).*/\1/p'); TYAW=${TYAW:--141.55}
TPITCH=$(echo "$LINE" | sed -n 's/.*pitch=\([-+0-9.]*\).*/\1/p'); TPITCH=${TPITCH:--22.39}
echo "[OK] 目标方位 yaw=$TYAW pitch=$TPITCH"
ensure_auto_on

echo
echo "########## A) 目标运动（假人横向移动 J/L）##########"
kill_aim_nodes; aim_at "$TYAW" "$TPITCH"
timeout 45 ffmpeg -y -f x11grab -video_size 1920x1080 -framerate 15 -i :0 -t 35 \
    -c:v libx264 -preset veryfast -pix_fmt yuv420p "$REC/scenario_target_moving.mp4" >/dev/null 2>&1 &
RP=$!; sleep 2; start_aim /opt/ws/scenA.log
sleep 4
for i in $(seq 1 25); do xdotool key --window "$WIN" l; sleep 0.12; done
for i in $(seq 1 25); do xdotool key --window "$WIN" j; sleep 0.12; done
sleep 6
echo "--- 目标运动日志（末 6 行）---"; grep -E '\[aim\]' /opt/ws/scenA.log | tail -6
echo "--- 检出率: $(ratio /opt/ws/scenA.log)% ---"
kill_aim_nodes; wait $RP 2>/dev/null

echo
echo "########## B) 己方运动（底盘 WASD + Q/E 旋转）##########"
kill_aim_nodes; aim_at "$TYAW" "$TPITCH"
timeout 50 ffmpeg -y -f x11grab -video_size 1920x1080 -framerate 15 -i :0 -t 40 \
    -c:v libx264 -preset veryfast -pix_fmt yuv420p "$REC/scenario_chassis_moving.mp4" >/dev/null 2>&1 &
RP2=$!; sleep 2; start_aim /opt/ws/scenB.log
sleep 4
for k in w w d d s s a a; do
  for i in $(seq 1 6); do xdotool key --window "$WIN" "$k"; sleep 0.08; done
  sleep 0.3
done
for i in $(seq 1 10); do xdotool key --window "$WIN" e; sleep 0.08; done
for i in $(seq 1 10); do xdotool key --window "$WIN" q; sleep 0.08; done
sleep 6
echo "--- 己方运动日志（末 6 行）---"; grep -E '\[aim\]' /opt/ws/scenB.log | tail -6
echo "--- 检出率: $(ratio /opt/ws/scenB.log)% ---"
kill_aim_nodes; wait $RP2 2>/dev/null

echo
echo "=== 录制品 ==="; ls -la "$REC"/scenario*.mp4 2>/dev/null
