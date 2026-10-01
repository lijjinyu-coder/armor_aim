#!/usr/bin/env bash
# ============================================================
# run_all_v2.sh —— 一键收尾：目标选择 → 阈值标定 → 最小闭环录制 → 目标丢失测试
# 用法: bash run_all_v2.sh [录制秒数, 默认 40]
#
# 注意（踩过的坑）：
#   * ros2 launch 被 kill 后子节点仍在运行，会持续发 distance=-1 使对准命令失效，
#     因此必须用 kill_aim_nodes 按节点名清理。
#   * F5 是切换开关：先查状态，必要时只按一次。
# ============================================================
source /opt/ros/humble/setup.bash
source /opt/ws/auto-aiming/install/setup.bash
source /opt/ws/bevy_robomaster_simulator/rm_ws/install/setup.bash
export DISPLAY=:0
REC=/opt/ws/recordings
mkdir -p "$REC"
DUR="${1:-40}"

pgrep -x openbox >/dev/null || { openbox --sm-disable >/dev/null 2>&1 & sleep 2; }
WIN=$(xdotool search --name daedalus | head -1)
[ -n "$WIN" ] && xdotool windowfocus --sync "$WIN" 2>/dev/null

kill_aim_nodes() {
  pkill -f 'aim_output_node' 2>/dev/null
  pkill -f 'opencv_armor_detector' 2>/dev/null
  pkill -f 'PoseEstimatorNode' 2>/dev/null
  pkill -f 'sim_detector.py' 2>/dev/null
  pkill -f 'video2detector' 2>/dev/null
  pkill -f 'gimbal_probe' 2>/dev/null
  sleep 2
  for pat in aim_output_node opencv_armor_detector PoseEstimatorNode sim_detector.py; do
    pgrep -f "$pat" | while read -r p; do kill -9 "$p" 2>/dev/null; done
  done
  sleep 1
}

yawv() { python3 /tmp/tf_rpy.py odom gimbal_link 2>/dev/null | tail -1 | grep -o 'yaw=[^ ]*' | tr -d ' ' | sed 's/yaw=//'; }
# 自瞄状态检测：命令"当前 yaw + 50°（绕圈）"，只有命令真正生效时才会到位。
# 这样即使云台恰好停在校验角度上也不会误判。
auto_on() {
  local cur target
  cur=$(yawv); [ -z "$cur" ] && cur=0
  target=$(python3 -c "print(round((($cur + 50 + 180) % 360) - 180, 2))")
  timeout 5 ros2 run sim_tests gimbal_probe --yaw "$target" --pitch 0 --distance 1.4 --hold --rate 20 >/dev/null 2>&1 &
  sleep 3.5
  local after; after=$(yawv)
  timeout 2 ros2 run sim_tests gimbal_probe --stop >/dev/null 2>&1
  sleep 0.6
  python3 - "$target" "$after" <<'PY'
import sys
try:
    t = float(sys.argv[1]); a = float(sys.argv[2])
except Exception:
    sys.exit(1)
d = abs((a - t + 180.0) % 360.0 - 180.0)
sys.exit(0 if d < 2.0 else 1)
PY
}
ensure_auto_on() {  # 最多尝试 3 次，避免切换开关时把状态弄反
  local i
  for i in 1 2 3; do
    if auto_on; then echo "[OK] 自瞄已开启（第 $i 次检测）"; return 0; fi
    echo "[..] 自瞄未生效，按一次 F5（第 $i 次）"
    xdotool key --window "$WIN" --clearmodifiers F5 2>/dev/null || xdotool key --clearmodifiers F5
    sleep 2
  done
  echo "[!!] 三次尝试后自瞄仍未开启，后续命令可能无效"
  return 1
}
aim_at() {
  timeout 6 ros2 run sim_tests gimbal_probe --yaw "$1" --pitch "$2" --distance 1.4 --hold --rate 20 >/dev/null 2>&1 &
  sleep 5
  timeout 2 ros2 run sim_tests gimbal_probe --stop >/dev/null 2>&1
  sleep 1
}
valid_ratio() {
  local tot fire
  tot=$(grep -c '\[aim\]' "$1" 2>/dev/null || true); tot=${tot:-0}
  fire=$(grep -c 'valid=1' "$1" 2>/dev/null || true); fire=${fire:-0}
  if [ "$tot" -eq 0 ]; then echo 0; else echo $(( fire * 100 / tot )); fi
}

echo "########## 0) 清理残留节点 ##########"
kill_aim_nodes
echo "[OK] 残留节点已清理（仿真器进程数 $(pgrep -c -f 'target/debug/daedalus' || echo 0)）"

echo
echo "########## 1) 选择目标 ##########"
python3 /tmp/list_armors.py 6 > /tmp/armors.txt 2>&1
grep -E 'armor id=|^TARGET' /tmp/armors.txt | head -9
LINE=$(grep '^TARGET ' /tmp/armors.txt | head -1)
TYAW=$(echo "$LINE" | sed -n 's/.*yaw=\([-0-9.]*\).*/\1/p')
TPITCH=$(echo "$LINE" | sed -n 's/.*pitch=\([-0-9.]*\).*/\1/p')
if [ -z "$TYAW" ] || [ -z "$TPITCH" ]; then
  echo "[!!] 未解析到目标，使用默认值"; TYAW=-141.55; TPITCH=-22.39
fi
echo "[OK] 目标方位: yaw=$TYAW pitch=$TPITCH"

echo
echo "########## 2) 确定性开启自瞄 + 阈值标定 ##########"
ensure_auto_on || true

BEST="150 100 1"; BEST_RATIO=-1
for combo in "150 100 1" "100 70 3" "80 60 5"; do
  set -- $combo; V="$1"; S="$2"; M="$3"
  LOG=/tmp/tune_${V}_${S}_${M}.log
  kill_aim_nodes
  aim_at "$TYAW" "$TPITCH"
  timeout 14 ros2 launch prm_launch sim_detector.py target_red:=false publish_command:=false \
      show_window:=false debug_image:=false color_set:=blue \
      value_lower_limit:=$V saturation_lower_limit:=$S max_missed_frames:=$M > "$LOG" 2>&1 &
  sleep 11
  kill_aim_nodes
  R=$(valid_ratio "$LOG")
  echo "  value>=$V sat>=$S missed=$M  -> 检出率 ${R}%"
  if [ "$R" -gt "$BEST_RATIO" ]; then BEST_RATIO=$R; BEST="$V $S $M"; fi
done
set -- $BEST; BV="$1"; BS="$2"; BM="$3"
echo "[OK] 采用阈值: value>=$BV sat>=$BS max_missed_frames=$BM (检出率 ${BEST_RATIO}%)"

echo
echo "########## 3) 最小闭环 + 录制（${DUR}s）##########"
kill_aim_nodes
echo "[..] 起点：目标方位偏 5 度（目标在画面内但偏离中心）"
START_YAW=$(python3 -c "print($TYAW + 5)")
aim_at "$START_YAW" "$TPITCH"
python3 /tmp/aim_dir.py muzzle odom 2>/dev/null | tail -1

timeout $((DUR + 20)) ffmpeg -y -f x11grab -video_size 1920x1080 -framerate 15 -i :0 -t "$DUR" \
    -c:v libx264 -preset veryfast -pix_fmt yuv420p "$REC/closed_loop.mp4" > /dev/null 2>&1 &
RECPID=$!
sleep 2
LOG=/opt/ws/closed_loop.log
: > "$LOG"
nohup ros2 launch prm_launch sim_detector.py target_red:=false publish_command:=true \
    show_window:=true debug_image:=false color_set:=blue \
    value_lower_limit:=$BV saturation_lower_limit:=$BS max_missed_frames:=$BM \
    control_gain:=0.4 deadband_deg:=1.5 > "$LOG" 2>&1 &
sleep $((DUR - 10))
timeout 20 python3 /tmp/save_frame.py "$REC/after.jpg" >/dev/null 2>&1
sleep 8
echo "--- 闭环日志（末 10 行）---"
grep -E '\[aim\]' "$LOG" | tail -10
echo "--- 闭环检出率: $(valid_ratio "$LOG")% ---"
echo "--- 最后 5 条有效帧（相对偏差应收敛到接近 0）---"
grep -E 'valid=1' "$LOG" | tail -5
python3 /tmp/aim_dir.py muzzle odom 2>/dev/null | tail -1
kill_aim_nodes
wait $RECPID 2>/dev/null

echo
echo "########## 4) 目标丢失测试 + 录制 ##########"
LOG2=/opt/ws/lost.log
: > "$LOG2"
timeout 45 ffmpeg -y -f x11grab -video_size 1920x1080 -framerate 15 -i :0 -t 35 \
    -c:v libx264 -preset veryfast -pix_fmt yuv420p "$REC/target_lost.mp4" > /dev/null 2>&1 &
REC2=$!
sleep 2
nohup ros2 launch prm_launch sim_detector.py target_red:=false publish_command:=true \
    show_window:=true debug_image:=false color_set:=blue \
    value_lower_limit:=$BV saturation_lower_limit:=$BS max_missed_frames:=$BM \
    control_gain:=0.4 deadband_deg:=1.5 > "$LOG2" 2>&1 &
sleep 12
echo "[..] 云台转离目标（模拟目标离开视野）"
timeout 6 ros2 run sim_tests gimbal_probe --yaw 60 --pitch 0 --distance 1 --hold --rate 20 >/dev/null 2>&1 &
sleep 9
echo "--- 丢失后日志 ---"
grep -E '\[aim\]' "$LOG2" | tail -6
echo "--- 云台是否保持姿态 ---"
python3 /tmp/aim_dir.py muzzle odom 2>/dev/null | tail -1
kill_aim_nodes
wait $REC2 2>/dev/null

echo
echo "########## 5) 产物 ##########"
ls -la "$REC"/*.mp4 "$REC"/*.jpg 2>/dev/null
echo "闭环检出率: ${BEST_RATIO}% (value>=$BV sat>=$BS missed=$BM)"
