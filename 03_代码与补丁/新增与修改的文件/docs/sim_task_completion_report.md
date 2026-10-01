# 自瞄仿真接入 —— 任务完成情况对照报告

本文件逐条对照《自瞄仿真接入与最小闭环任务说明》，给出完成情况、证据与遗留项。
仓库：本仓库 `feature/simulator-integration` 分支，标签 `sim-closed-loop-v1`。
仿真器：bevy_robomaster_simulator（Daedalus），修改记录见 `docs/sim_deployment_record.md`。

## 总览

| 任务书章节 | 要求 | 状态 | 证据 |
| --- | --- | --- | --- |
| §二 仿真部署与接口确认 | 部署仿真器、确认 4 类接口、系统图 | ✅ 完成 | `docs/sim_deployment_record.md`、`docs/sim_interface_record.md`、`docs/sim_system_diagram.svg` |
| §三 ROS2 最小通信测试 | 图像订阅测试 + 云台发布测试 | ✅ 完成 | `src/sim_tests/image_probe.py`、`gimbal_probe.py`（实测记录见测试记录） |
| §四 实时图像接入原算法 | ROS2 Image → 检测 → PnP（开环） | ✅ 完成 | 实测 `valid=1 yaw_rel=-6.62 pitch_rel=-11.22 dist=1.21m`，距离与几何真值吻合 |
| §五 输入/输出抽象 | 视频/ROS2 双输入、AimResult 双输出 | ✅ 完成 | `prm_launch/sim_detector.py`、`video2detector.py`、`prm_control/aim_output` |
| §六 云台控制与最小闭环 | 静态闭环 + 目标丢失行为 | ✅ 完成 | **实测：云台自主转向并收敛到 yaw_rel=+1.37° / pitch_rel=-0.13°**；目标丢失后保持姿态；录制 `closed_loop.mp4`、`target_lost.mp4` |
| §七 基础场景调试 | 4 类场景记录 | ⏳ 静态/丢失已完成；运动类场景脚本就绪 | 见 `docs/sim_test_record.md` |
| §八 仓库管理与提交 | 分支/标签/README/记录/演示 | ⏳ 进行中 | 分支与文档已完成；标签与推送待闭环验证后执行 |

## §二 仿真部署与接口确认

**部署**：阿里云 `ecs.gn6i-c8g1.2xlarge`（8 vCPU / 31 GiB / Tesla T4）、Ubuntu 22.04.5、ROS2 Humble、
Rust 1.98.1；图形界面跑在 `Xvfb :0` + `x11vnc` + `noVNC`（SSH 隧道访问）。

**仿真器构建**：`cargo build --no-default-features --features ros2`，构建前需
`source /opt/ros/humble/setup.bash` + colcon 构建并 source `rm_interfaces` + `libclang`，
并建议设置 `IDL_PACKAGE_FILTER`（详见 `docs/sim_deployment_record.md`）。

**发现并修复的上游问题（2 处）**：见 `docs/simulator_ros2_build_fix.patch`。

**接口确认（全部实测，与源码分析吻合）**：

| 接口 | 结论 |
| --- | --- |
| 图像 | `/image_raw`：`sensor_msgs/Image`，**rgb8**，1440×1080，step 4320，≈63 Hz，QoS Reliable，frame_id=`camera_optical_frame` |
| 相机信息 | `/camera_info`：plumb_bob，畸变全 0，**fx=fy=1303.675283386667，cx=720，cy=540**（与 `H/(2tan(fov_y/2))`、`fov=45°` 计算值一致） |
| 云台/相机姿态 | `/gimbal_pose` 等 PoseStamped 的 pose 恒为单位阵，**真实姿态从 `/tf` 取**（`map→odom→gimbal_link→{muzzle, camera_link→camera_optical_frame}`，≈65 Hz） |
| 云台命令 | `/rm_gimbal/cmd`：`rm_interfaces/GimbalCmd`，QoS **BestEffort**；yaw=绝对角（度，左为正）；**pitch 命令值 = 目标仰角（度，实测标定，±70° 限幅）**；`distance=-1` = 无解停追踪；需按 **F5** 开启订阅 |

## §三 最小通信测试

- `sim_tests/image_probe.py`：打印宽高/编码/时间戳/接收 FPS 并可显示画面 → 实测 rgb8/1440×1080/≈36 FPS
- `sim_tests/gimbal_probe.py`：发送小幅固定命令、`--stop` 发 `distance=-1` →
  用于标定 yaw/pitch 方向与单位（见 `docs/sim_interface_record.md` 第五节）

## §四 实时图像接入原算法（开环）

`ros2 launch prm_launch sim_detector.py publish_command:=false`：

- 检测器：`Detection frame size: 1440x1080`（参数化生效）
- PnP：`Camera intrinsics applied: fx=1303.675 fy=1303.675 cx=720 cy=540 dist=[0,0,0,0]`（与仿真 `/camera_info` 一致）
- 输出：`[aim] FIRE valid=1 yaw_rel=-6.62 pitch_rel=-11.22 dist=1.21m recv_fps=37.0 proc_fps=20.0`
- **解算精度交叉验证**：PnP 距离 1.21 m ↔ `/simulator/marker` 几何真值（相机→目标约 1.22 m），误差 <1%
- 调试画面：`/debug_image` 含装甲候选框、yaw/pitch/distance、状态、接收/处理 FPS（截图见 `docs/evidence_*.jpg`）

## §五 输入/输出抽象

```text
视频文件 ──┐                        ┌── 调试输出（日志 + /debug_image）
           ├── 检测 → PnP → AimResult ┤
仿真图像 ──┘                        └── ROS2 输出（/rm_gimbal/cmd + 看门狗）
```

- 输入层：视频用 `webcam_publisher`；仿真模式直接用仿真器 `/image_raw`；核心算法只订阅 `image_raw`，不感知来源
- 核心算法层：`opencv_armor_detector`、`pose_estimator`（仅参数化，逻辑未改）
- 输出层：`aim_output` 包，统一 `AimResult{yaw_rel, pitch_rel, distance, target_valid}`，
  分发给 `debug_output` 与 `gimbal_output`（含速率限制与看门狗）
- 模式切换：`sim_detector.py` / `video2detector.py`（launch 参数切换，无需改代码）

## §六 云台控制与最小闭环

- 命令换算：`yaw = atan2(d_world.y, d_world.x)`；`pitch = 90° - acos(d_world.z)`（= 目标仰角），
  其中 `d_world = R(world←camera_optical_frame)·d_cam`，**每帧用最新 TF 重算，误差不累积**
- 目标丢失：全 0 解 / 看门狗超时 → 发 `distance=-1` → 仿真器移除跟踪器、云台保持姿态（已实测）
- 稳定性：控制增益 0.4 + 死区 1.5° + 指令速率限制（yaw 60°/s、pitch 45°/s），
  用于抑制视觉链路延迟导致的过冲与抖动
- **实测结果（验收镜头 `docs/demo/acceptance_closed_loop.mp4`，52 秒一次连续录制）**：
  目标偏离中心 **13.80°** → 画面中按 F5 开启自瞄 → 云台自主转向（-127.02° → -114.49°）
  → 稳定居中（**yaw_rel=+0.26°、pitch_rel=-0.89°**）→ 目标丢失（`TIMEOUT` + `distance=-1`）
  → 云台保持姿态（t=34s 与 t=48s 姿态一致）
- 录制期间云台只由算法输出驱动，未发送任何探针/手工命令、未按方向键
- 复现脚本：`sim/acceptance_video.sh`（含目标方位自动测定、偏离姿态可检测性校验、
  F5 状态微探针 + 算法反馈双重确认、残留节点清理）

## §七 基础场景调试

| 场景 | 状态 |
| --- | --- |
| 静态目标（不同距离/偏角） | ✅ 已验证：对准后稳定检测，距离与真值吻合 |
| 目标运动 | ⏳ 脚本就绪（`run_all.sh` / `run_scenarios.sh` 使用假人 IJKL 移动） |
| 己方运动/复杂视角 | ⏳ 脚本就绪（WASD 移动己方底盘） |
| 目标丢失与输入中断 | ✅ 已验证（`distance=-1`，云台保持姿态） |

**已知问题**：仿真场景偏暗导致检测间歇丢失（`FIRE`/`TIMEOUT` 交替）。已暴露检测器
HSV 阈值与 `max_missed_frames` 为 launch 参数，`run_all.sh` 会自动扫描几组参数取检出率最高者。

## §八 仓库管理与提交

- ✅ 分支 `feature/simulator-integration`（5+ 提交，含基线提交，便于审阅改动）
- ✅ `README_SIM_INTEGRATION.md`（环境/编译/启动/模式切换/模块职责/数据流/接口单位/目标丢失行为）
- ✅ `docs/sim_deployment_record.md`（部署与排查记录，含仿真器版本与修改记录）
- ✅ `docs/sim_interface_record.md`（接口确认记录，源码 + 实测）
- ✅ `docs/sim_test_record.md`（测试记录：FPS、场景、问题）
- ✅ `docs/sim_system_diagram.svg`（标明数据方向的系统图）
- ✅ `docs/sim_demo_materials.md`（演示材料清单与录制方法）
- ⏳ 标签 `sim-closed-loop-v1` 与推送 GitHub（待闭环演示验证后执行）
