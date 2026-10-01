#!/usr/bin/env bash
# make_demo_video.sh —— 把分段录制的演示片段合成为一段带标题说明的视频（任务书 §八-4）
#
# 用法: bash sim/make_demo_video.sh <片段目录> <输出文件>
#   片段目录内按文件名排序取 *.mp4；每个片段前插入一张标题卡（片段文件名作为标题）。
#
# 依赖: ffmpeg（含 libx264 与 drawtext 字体）
set -eo pipefail

SRC="${1:?用法: make_demo_video.sh <片段目录> <输出文件>}"
OUT="${2:?用法: make_demo_video.sh <片段目录> <输出文件>}"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

FONT=""
for f in /usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf \
         /usr/share/fonts/truetype/dejavu/DejaVuSans.ttf; do
  [ -f "$f" ] && { FONT="$f"; break; }
done
[ -n "$FONT" ] || { echo "[WARN] 未找到字体，标题卡将不含文字"; }

i=0
: > "$WORK/list.txt"
for clip in "$SRC"/*.mp4; do
  [ -f "$clip" ] || continue
  name="$(basename "$clip" .mp4)"
  i=$((i + 1))
  card="$WORK/card_$(printf '%02d' "$i").mp4"
  if [ -n "$FONT" ]; then
    ffmpeg -y -f lavfi -i "color=c=black:s=1920x1080:d=2.5" \
      -vf "drawtext=fontfile=$FONT:text='${i}. ${name}':fontcolor=white:fontsize=56:x=(w-text_w)/2:y=(h-text_h)/2" \
      -c:v libx264 -pix_fmt yuv420p -r 15 "$card" >/dev/null 2>&1
  else
    ffmpeg -y -f lavfi -i "color=c=black:s=1920x1080:d=2.5" \
      -c:v libx264 -pix_fmt yuv420p -r 15 "$card" >/dev/null 2>&1
  fi
  # 统一片段参数，便于拼接
  norm="$WORK/norm_$(printf '%02d' "$i").mp4"
  ffmpeg -y -i "$clip" -vf "scale=1920:1080,fps=15,format=yuv420p" \
    -c:v libx264 -preset veryfast "$norm" >/dev/null 2>&1
  echo "file '$card'" >> "$WORK/list.txt"
  echo "file '$norm'" >> "$WORK/list.txt"
done

[ -s "$WORK/list.txt" ] || { echo "[ERR] 目录中没有 mp4 片段"; exit 1; }

ffmpeg -y -f concat -safe 0 -i "$WORK/list.txt" -c:v libx264 -preset veryfast -pix_fmt yuv420p "$OUT"
echo "[OK] 已合成 $OUT"
ls -la "$OUT"
