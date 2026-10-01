#!/usr/bin/env bash
# 从验收录像抽取若干帧（裁出调试叠加层区域），供本地拼时间轴图
cd /opt/ws/recordings || exit 1
rm -f /tmp/acc_*.jpg
for t in 1 5 9 13 22 34 48; do
  ffmpeg -y -ss "$t" -i acceptance_closed_loop.mp4 -vframes 1 \
      -vf 'crop=760:250:0:0' -q:v 3 "/tmp/acc_${t}.jpg" >/dev/null 2>&1
done
ls -la /tmp/acc_*.jpg
mkdir -p /tmp/accframes && rm -f /tmp/accframes/*
cp /tmp/acc_*.jpg /tmp/accframes/ 2>/dev/null
echo "--- 全帧截图（便于看画面）---"
for t in 9 13 22; do
  ffmpeg -y -ss "$t" -i acceptance_closed_loop.mp4 -vframes 1 -vf 'scale=960:-1' \
      -q:v 4 "/tmp/accfull_${t}.jpg" >/dev/null 2>&1
done
ls -la /tmp/accfull_*.jpg
