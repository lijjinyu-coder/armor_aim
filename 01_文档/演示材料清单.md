# 演示材料（任务书 §八-4）

## 一、成品视频

| 文件 | 内容 | 时长 |
| --- | --- | --- |
| **`demo_all.mp4`** | **整合成品**：4 段带中文标题卡的演示（推荐直接观看） | ≈2 分 40 秒 |
| **`acceptance_closed_loop.mp4`** | **验收镜头（一镜到底，核心证据）**：<br>目标偏离中心 13.8° → 画面中按 F5 开启自瞄 → 云台自主转向 → 稳定居中（0.26°）→ 目标丢失 → 保持姿态 | 52 s |
| `video_mode_test.mp4` | 视频模式：原检测 + PnP + 调试叠加层（原链路仍可运行） | 12 s |
| `scenario_target_moving.mp4` | 目标运动：检测连续、云台跟随（检出率 100%） | 35 s |
| `scenario_chassis_moving.mp4` | 己方运动：短暂丢失后自动重捕获（检出率 83%） | 40 s |
| `closed_loop.mp4` / `target_lost.mp4` | 早期分段录制（闭环 / 丢失），验收镜头已覆盖并更完整 | 40 s / 35 s |

### 验收镜头时间轴（`acceptance_timeline.png` 为逐帧对照图）

| 时刻 | 调试叠加层 | 说明 |
| --- | --- | --- |
| t=1s / 5s | `FIRE valid=True yaw_rel=-13.80° pitch_rel=+1.67° dist=1.92m cmd yaw=-121.18°` | **目标偏离中心 13.8°**；自瞄关闭，云台冻结在 -127.02° |
| t=9s | `NO_ARMOR`（短暂） | 画面中按 F5 开启自瞄，转向过程中目标短暂离开视野 |
| t=13s | `FIRE valid=True yaw_rel=-0.71° pitch_rel=+0.52°` | **云台自主转向到位**（云台 -127.02° → -119.02° → -114.49°） |
| t=22s | `FIRE valid=True yaw_rel=+0.26° pitch_rel=-0.89°` | **稳定居中** |
| t=34s | `TIMEOUT valid=False cmd=-` | **目标丢失**（敌方离开视野 + 停止图像输入），看门狗发 `distance=-1` |
| t=48s | `TIMEOUT valid=False`，云台姿态不变 | **云台保持姿态，不再使用过期目标** |

> 该段为**连续一次录制、无剪辑**；录制期间云台只由算法输出驱动 —— 没有发送任何探针/手工命令，
> 也没有按方向键（自瞄开启时方向键本就失效，可作为反证）。

## 二、如何复现录制

```bash
# 服务器端（图形界面跑在 Xvfb :0，经 noVNC 查看）
bash sim/build_aim_ws.sh <本仓库>                       # 构建（白名单）
bash sim/run_sim.sh /opt/ws/bevy_robomaster_simulator   # 启动仿真器
bash sim/acceptance_video.sh /opt/ws/recordings/acceptance_closed_loop.mp4 10
bash sim/run_closed_loop_demo.sh 40                     # 自动选目标/标定阈值/闭环/丢失
bash sim/run_scenarios.sh                               # 运动场景
# 本地合成带标题卡的成品（PIL 渲染中文标题，避开 ffmpeg drawtext 的 Windows 路径转义问题）
python deploy/tools/make_demo_local.py
```

**踩坑记录（复现时会遇到）**：
1. `ros2 launch` 被杀后**子节点仍在运行**，会持续发 `distance=-1` 移除云台跟踪器 → 用
   `sim/cleanup_nodes.sh` 按节点名彻底清理。
2. F5 是**切换**开关；`xdotool key --window` 走 XSendEvent **winit 会忽略**，
   必须 `windowactivate` 后用 XTEST（不带 `--window`）发送。
3. 偏离基准要用**算法实际能锁定的装甲板方位**（开环输出的 `cmd yaw`），不能用 `/simulator/marker`
   的最近装甲板（两者可差近 30°，会导致偏离期目标不在画面）。
4. 解析日志数值时字符类要含 `+`（`pitch=+17.38`）；命令 pitch 的语义是**目标仰角**，
   不能用 `gimbal_link` 的 RPY pitch。

## 三、录屏与查看要点

- **查看**：本地 `ssh -L 6080:127.0.0.1:6080 root@<公网IP>` → 浏览器开 `http://127.0.0.1:6080/vnc.html`
- **录制**：`bash sim/record_demo.sh <输出> <秒数>`（内部 `ffmpeg -f x11grab -i :0`）

## 四、演示时值得指出的界面元素

1. 左下角仿真器自带叠加层（验收镜头中可看到 `auto-aim=ON`，以及 `total/accurate` 统计）
2. 调试窗口左上角：`status / valid / yaw_rel / pitch_rel / distance / cmd yaw / cmd pitch / recv FPS / process FPS`
3. 画面中的绿色框：检测到的敌方装甲板
4. 云台从偏离 13.8° 到 0.26° 的自主转向过程（画面视角随之转动）
