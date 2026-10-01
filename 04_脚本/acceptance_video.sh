#!/usr/bin/env bash
# ============================================================
# acceptance_video.sh —— 任务书 §六 验收录像（一次性连续镜头）
#
# 目标时间轴：
#   0~8s   目标偏离中心（自瞄关闭；调试层 yaw_rel≈偏离角、valid=True）
#   8s     按 F5 开启自瞄
#   8~26s  云台自主转向并稳定在中心附近
#   26~40s 目标丢失（敌方离开视野 → 停止图像输入）
#   40~52s 云台保持姿态、不再使用过期目标
#
# 三次踩坑后的关键做法：
#   1) 偏离基准必须用**算法实际能锁定的装甲板方位**（从开环输出 cmd yaw 读），
#      不能用 /simulator/marker 的最近装甲板（两者能差近 30°，会导致偏离期目标不在画面）。
#   2) 判断自瞄开关**不能**用"当前角+50°"这种大探针 —— 它本身会把云台移走。
#      改用 1.5° 微探针：开则动、关则不动，扰动可忽略。
#   3) 浮点参数一律写小数（如 20.0），节点侧也已做 dynamic_typing 兼容。
#
# 用法: bash acceptance_video.sh [输出文件] [偏离角度, 默认 10]
# ============================================================
set -o pipefail
source /opt/ros/humble/setup.bash
source /opt/ws/auto-aiming/install/setup.bash
source /opt/ws/bevy_robomaster_simulator/rm_ws/install/setup.bash
export DISPLAY=:0

OUT="${1:-/opt/ws/recordings/acceptance_closed_loop.mp4}"
OFFSET="${2:-10}"
mkdir -p "$(dirname "$OUT")"

pgrep -x openbox >/dev/null || { openbox --sm-disable >/dev/null 2>&1 & sleep 2; }
WIN=$(xdotool search --name daedalus 2>/dev/null | head -1)
echo "[..] daedalus 窗口 id: ${WIN:-（未找到，F5 将无法定向发送）}"
[ -n "$WIN" ] && xdotool windowfocus --sync "$WIN" 2>/dev/null

press_f5() {
  # 关键：xdotool key --window 走 XSendEvent，winit 常忽略；必须让窗口获得输入焦点后
  # 用 XTEST（不带 --window）发送，才可靠（这是前面 F5 时而有效的根因）。
  xdotool windowactivate --sync "$WIN" 2>/dev/null || xdotool windowfocus --sync "$WIN" 2>/dev/null
  sleep 0.4
  xdotool key --clearmodifiers F5
  sleep 0.4
}

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
# 注意：数值可能带 '+' 号（如 pitch=+17.38），字符类必须包含 [+]，否则解析为空
pitchv() { python3 /tmp/tf_rpy.py odom gimbal_link 2>/dev/null | tail -1 | sed -n 's/.*pitch=\([-+0-9.]*\).*/\1/p'; }
# 命令 pitch 的语义是"目标仰角"（90=水平），不是 gimbal_link 的 RPY pitch。
# 微探针必须用实际瞄准仰角，否则会把云台抬头/低头，导致目标离开视野（踩过的坑）。
elev() {
  python3 /tmp/aim_dir.py muzzle odom 2>/dev/null | tail -1 \
    | sed -n 's/.*aim_pitch_from_vertical=\([-+0-9.]*\).*/\1/p' \
    | awk '{printf "%.2f", $1-90}'
}
aimv() { python3 /tmp/aim_dir.py muzzle odom 2>/dev/null | tail -1; }
last_aim()  { grep -E '\[aim\]' "$1" 2>/dev/null | tail -1; }
last_valid(){ grep -E 'valid=1' "$1" 2>/dev/null | tail -1; }

# 1.5° 微探针：自瞄开启则云台移动 ~1.5°，关闭则不动
auto_on() {
  local before after delta
  before=$(yawv); [ -z "$before" ] && return 1
  local tgt; tgt=$(python3 -c "print(round($before + 1.5, 2))")
  timeout 4 ros2 run sim_tests gimbal_probe --yaw "$tgt" --pitch "$(elev)" --distance 1.4 --hold --rate 20 >/dev/null 2>&1 &
  sleep 3
  after=$(yawv)
  timeout 2 ros2 run sim_tests gimbal_probe --stop >/dev/null 2>&1; sleep 0.5
  python3 - "$before" "$after" <<'PY'
import sys
try:
    b, a = float(sys.argv[1]), float(sys.argv[2])
except Exception:
    sys.exit(1)
# 移动超过 0.8° 认为命令生效（自瞄开启）
sys.exit(0 if abs((a - b + 180.0) % 360.0 - 180.0) > 0.8 else 1)
PY
}
set_auto_aim() {  # on|off，最多 3 次尝试
  local want="$1" i
  for i in 1 2 3; do
    if [ "$want" = "on" ]; then
      auto_on && { echo "[OK] 自瞄=ON（第 $i 次判定）"; return 0; }
    else
      auto_on || { echo "[OK] 自瞄=OFF（第 $i 次判定）"; return 0; }
    fi
    press_f5; sleep 1.5
  done
  echo "[!!] 自瞄状态未能切换到 $want"; return 1
}
aim_at() {
  timeout 6 ros2 run sim_tests gimbal_probe --yaw "$1" --pitch "$2" --distance 1.4 --hold --rate 20 >/dev/null 2>&1 &
  sleep 5; timeout 2 ros2 run sim_tests gimbal_probe --stop >/dev/null 2>&1; sleep 1
}
layout_windows() {
  local dbg sim
  sim=$(xdotool search --name daedalus 2>/dev/null | head -1)
  dbg=$(xdotool search --name 'auto-aim debug' 2>/dev/null | head -1)
  [ -n "$sim" ] && { xdotool windowmove "$sim" 940 0 2>/dev/null; xdotool windowsize "$sim" 980 1080 2>/dev/null; }
  [ -n "$dbg" ] && { xdotool windowmove "$dbg" 0 0 2>/dev/null; xdotool windowsize "$dbg" 940 1080 2>/dev/null; }
}
start_pipeline() {  # $1=log  $2=publish_command
  nohup ros2 launch prm_launch sim_detector.py target_red:=false publish_command:=$2 \
      show_window:=true debug_image:=false color_set:=blue control_gain:=0.4 deadband_deg:=1.5 \
      max_yaw_rate_deg_s:=20.0 max_pitch_rate_deg_s:=15.0 > "$1" 2>&1 &
  local i n=0
  for i in $(seq 1 12); do sleep 3; n=$(grep -c '\[aim\]' "$1" 2>/dev/null || echo 0); [ "$n" -gt 0 ] && break; done
  echo "$n"
}

echo "########## 阶段 0：确认自瞄状态并测定算法实际目标方位 ##########"
kill_aim_nodes
set_auto_aim on || true
python3 /tmp/list_armors.py 6 > /tmp/armors.txt 2>&1
TYAW=$(grep '^TARGET ' /tmp/armors.txt | sed -n 's/.*yaw=\([-+0-9.]*\).*/\1/p'); TYAW=${TYAW:--141.55}
TPITCH=$(grep '^TARGET ' /tmp/armors.txt | sed -n 's/.*pitch=\([-+0-9.]*\).*/\1/p'); TPITCH=${TPITCH:--22.39}
echo "[..] marker 最近装甲板: yaw=$TYAW pitch=$TPITCH；先对准它再让算法给出实际锁定方位"
aim_at "$TYAW" "$TPITCH"
PL=/tmp/eff.log; : > "$PL"
start_pipeline "$PL" false >/dev/null
layout_windows
LINE=$(last_valid "$PL")
echo "    开环输出: ${LINE:-（无有效帧）}"
EFF_YAW=$(echo "$LINE" | sed -n 's/.*cmd=(\([-+0-9.]*\),.*/\1/p'); EFF_YAW=${EFF_YAW:-$TYAW}
EFF_PITCH=$(echo "$LINE" | sed -n 's/.*cmd=([-+0-9.]*,\([-+0-9.]*\)).*/\1/p'); EFF_PITCH=${EFF_PITCH:-$TPITCH}
echo "[OK] 算法实际目标方位: yaw=$EFF_YAW pitch=$EFF_PITCH"
kill_aim_nodes

echo
echo "########## 阶段 1：摆到'目标偏离中心'姿态并确认可检测 ##########"
OK_OFFSET=""
for off in "$OFFSET" 6 4; do
  set_auto_aim on >/dev/null || true
  kill_aim_nodes
  oy=$(python3 -c "print(round($EFF_YAW + $off, 2))")
  aim_at "$oy" "$EFF_PITCH"
  VL=/tmp/verify.log; : > "$VL"
  start_pipeline "$VL" false >/dev/null
  L=$(last_valid "$VL"); YR=$(echo "$L" | sed -n 's/.*yaw_rel= *\([-+0-9.]*\).*/\1/p')
  echo "  偏离 ${off}° -> yaw_rel=${YR:-n/a}"
  if [ -n "$YR" ]; then OK_OFFSET="$off"; kill_aim_nodes; break; fi
  kill_aim_nodes
done
if [ -z "$OK_OFFSET" ]; then echo "[!!] 找不到可检测的偏离姿态，改用 4°"; OK_OFFSET=4; fi
echo "[OK] 起始偏离角 = ${OK_OFFSET}°"

set_auto_aim on >/dev/null || true
STAGE_YAW=$(python3 -c "print(round($EFF_YAW + $OK_OFFSET, 2))")
aim_at "$STAGE_YAW" "$EFF_PITCH"
echo -n "    摆好后的云台: "; aimv

echo
echo "########## 阶段 2：关闭自瞄（微探针校验）并启动算法 ##########"
set_auto_aim off || echo "[!!] 关闭自瞄失败"
echo -n "    关闭后云台: "; aimv
LOG=/opt/ws/acceptance.log; : > "$LOG"
N=$(start_pipeline "$LOG" true)
layout_windows
sleep 1
echo "    链路输出 $N 行；偏离姿态下的结果:"
last_aim "$LOG"

echo
echo "########## 阶段 3：连续录制 ##########"
timeout 75 ffmpeg -y -f x11grab -video_size 1920x1080 -framerate 15 -i :0 -t 52 \
    -c:v libx264 -preset veryfast -pix_fmt yuv420p "$OUT" > /dev/null 2>&1 &
RECPID=$!
sleep 8
echo "[$(date +%T)] 按 F5 开启自瞄，并用算法反馈确认是否生效（yaw_rel 应开始收缩）"
YAW_REL() { last_valid "$LOG" | sed -n 's/.*yaw_rel= *\([-+0-9.]*\).*/\1/p'; }
ABS() { python3 -c "print(abs(float('$1')))" 2>/dev/null || echo 999; }
ok=0
for k in 1 2 3; do
  b=$(YAW_REL); b=${b:-0}
  press_f5
  sleep 6
  a=$(YAW_REL); a=${a:-0}
  echo "  第 $k 次 F5: yaw_rel ${b} -> ${a}"
  if [ "$(ABS "$a")" -lt "$(ABS "$b")" ] 2>/dev/null || [ "$(python3 -c "print(1 if abs(float('$a')) < 3 else 0)" 2>/dev/null)" = "1" ]; then
    ok=1; break
  fi
  sleep 1
done
echo "[$(echo "$([ $ok = 1 ] && echo OK || echo WARN)")] 自瞄已生效并开始收敛"
sleep 12
echo "[$(date +%T)] 转向后:"; last_aim "$LOG"
echo -n "    云台: "; aimv
echo "[$(date +%T)] 目标丢失：敌方横向离开视野"
for i in $(seq 1 22); do xdotool key --window "$WIN" l; sleep 0.12; done
sleep 2
echo "[$(date +%T)] 并停止图像输入（杀检测节点）"
pkill -f 'opencv_armor_detector' 2>/dev/null
sleep 9
echo "--- 丢失后日志 ---"; grep -E '\[aim\]' "$LOG" | tail -4
echo -n "--- 云台是否保持姿态: "; aimv
wait $RECPID 2>/dev/null
kill_aim_nodes
echo
echo "########## 完成 ##########"; ls -la "$OUT"
