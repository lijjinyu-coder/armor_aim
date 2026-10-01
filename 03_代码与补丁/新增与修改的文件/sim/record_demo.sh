#!/usr/bin/env bash
# 用 ffmpeg 录制虚拟显示（Xvfb :0）的画面，用于生成演示材料。
# 用法: bash sim/record_demo.sh <输出文件> <时长秒> [DISPLAY] [分辨率] [帧率]
#   例: bash sim/record_demo.sh /opt/ws/recordings/closed_loop.mp4 30
set -eo pipefail

OUT="${1:?用法: record_demo.sh <输出文件> <时长秒> [DISPLAY] [分辨率] [帧率]}"
DUR="${2:-30}"
DISP="${3:-:0}"
SIZE="${4:-1920x1080}"
FPS="${5:-15}"

mkdir -p "$(dirname "$OUT")"
echo "[..] 录制 $DISP -> $OUT (${DUR}s, $SIZE@$FPS)"
ffmpeg -y -f x11grab -video_size "$SIZE" -framerate "$FPS" -i "$DISP" -t "$DUR" \
    -c:v libx264 -preset veryfast -pix_fmt yuv420p "$OUT"
echo "[OK] $(ls -la "$OUT")"
