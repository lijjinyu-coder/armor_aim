#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""本地合成演示视频：用 PIL 渲染中文标题卡（避开 drawtext 的 Windows 路径转义问题），
再用 ffmpeg 拼接分段录制。"""
import os
import shutil
import subprocess
import tempfile

from PIL import Image, ImageDraw, ImageFont

ART = r"C:\rm\deploy\artifacts"
FFMPEG = shutil.which("ffmpeg") or r"C:\msys64\ucrt64\bin\ffmpeg.exe"
CJK_FONTS = [r"C:\Windows\Fonts\msyhbd.ttc", r"C:\Windows\Fonts\msyh.ttc",
             r"C:\Windows\Fonts\simhei.ttf"]

SEGMENTS = [
    ("video_mode_test.mp4",
     "1. 视频模式：原链路仍可运行（检测 + PnP + 调试叠加层）", 12.0),
    ("acceptance_closed_loop.mp4",
     "2. 验收镜头：目标偏离 13.8° → 开启自瞄 → 云台自主转向 → 稳定居中 → 目标丢失", 52.0),
    ("scenario_target_moving.mp4", "3. 目标运动：检测连续，云台跟随（检出率 100%）", 35.0),
    ("scenario_chassis_moving.mp4", "4. 己方运动：短暂丢失后自动重捕获（检出率 83%）", 40.0),
]


def run(args):
    p = subprocess.run(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if p.returncode != 0:
        raise RuntimeError("ffmpeg failed: " + " ".join(args[:8]))


def make_card(path, title, subtitle=""):
    img = Image.new("RGB", (1920, 1080), (10, 12, 16))
    draw = ImageDraw.Draw(img)
    font_path = next((f for f in CJK_FONTS if os.path.exists(f)), None)
    f_title = ImageFont.truetype(font_path, 64) if font_path else ImageFont.load_default()
    f_sub = ImageFont.truetype(font_path, 34) if font_path else ImageFont.load_default()

    bbox = draw.textbbox((0, 0), title, font=f_title)
    draw.text(((1920 - (bbox[2] - bbox[0])) / 2, 470), title, font=f_title,
              fill=(235, 240, 245))
    if subtitle:
        bbox = draw.textbbox((0, 0), subtitle, font=f_sub)
        draw.text(((1920 - (bbox[2] - bbox[0])) / 2, 570), subtitle, font=f_sub,
                  fill=(150, 165, 180))
    draw.text((60, 1010), "auto-aiming + bevy_robomaster_simulator  |  sim-closed-loop-v1",
              font=f_sub, fill=(90, 100, 112))
    img.save(path)


def main():
    if not os.path.exists(FFMPEG):
        print(f"[ERR] ffmpeg not found: {FFMPEG}")
        return 1
    work = tempfile.mkdtemp(prefix="demo_")
    entries = []
    try:
        for i, (name, title, _dur) in enumerate(SEGMENTS, start=1):
            src = os.path.join(ART, name)
            if not os.path.exists(src):
                print(f"[skip] {name} (缺失)")
                continue
            png = os.path.join(work, f"card_{i:02d}.png")
            make_card(png, title, f"segment {i} / {len(SEGMENTS)}")
            card = os.path.join(work, f"card_{i:02d}.mp4")
            norm = os.path.join(work, f"norm_{i:02d}.mp4")
            run([FFMPEG, "-y", "-loop", "1", "-i", png, "-t", "3", "-r", "15",
                 "-c:v", "libx264", "-pix_fmt", "yuv420p", card])
            run([FFMPEG, "-y", "-i", src, "-vf", "scale=1920:1080,fps=15,format=yuv420p",
                 "-c:v", "libx264", "-preset", "veryfast", norm])
            entries.extend([card, norm])
            print(f"[ok] {i}. {name}")

        list_file = os.path.join(work, "list.txt")
        with open(list_file, "w", encoding="utf-8") as fh:
            for e in entries:
                fh.write("file '%s'\n" % e.replace("\\", "/"))
        out = os.path.join(ART, "demo_all.mp4")
        run([FFMPEG, "-y", "-f", "concat", "-safe", "0", "-i", list_file,
             "-c:v", "libx264", "-preset", "veryfast", "-pix_fmt", "yuv420p", out])
        print(f"[DONE] {out} ({os.path.getsize(out)/1024/1024:.2f} MB)")
        return 0
    finally:
        shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
