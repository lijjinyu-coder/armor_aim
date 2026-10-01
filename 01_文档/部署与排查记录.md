# 仿真部署与排查记录

本文件记录仿真环境的搭建过程、版本信息与遇到的问题（任务书 §八-3 要求）。

## 一、环境记录

| 项 | 值 |
| --- | --- |
| 云服务器 | 阿里云 ECS `ecs.gn6i-c8g1.2xlarge`（8 vCPU / 31 GiB / 100 GB ESSD） |
| GPU | NVIDIA Tesla T4，驱动 580.126.09（镜像预装） |
| 操作系统 | Ubuntu 22.04.5 LTS |
| ROS2 | Humble（`ros-humble-ros-base` + cv_bridge / image_transport / camera_info_manager / nav_msgs / tf_transformations / rqt） |
| Rust | stable 1.98.1（rustc 1.98.1） |
| 仿真器 | bevy_robomaster_simulator（Daedalus），源码 SHA256 已与上游 `Blackjack200/bevy_robomaster_simulator@master` 全量比对一致 |
| 自瞄仓库 | 本仓库 `feature/simulator-integration` 分支，标签 `sim-closed-loop-v1`；源码与上游 `RoboMaster-Club/auto-aiming@main` 差异仅为本次有意修改/新增的文件 |
| 图形环境 | `Xvfb :0`（1920×1080）+ `x11vnc` + `noVNC`（6080），全部经 SSH 隧道访问，安全组只放行 22 |
| 录像工具 | ffmpeg 4.4.2（`x11grab` 录制虚拟屏） |

### 构建命令（可复现）

```bash
# 仿真器消息包
cd <sim>/rm_interfaces && source /opt/ros/humble/setup.bash
colcon build --base-paths . --build-base ../rm_ws/build --install-base ../rm_ws/install --merge-install

# 仿真器本体
cd <sim>
source /opt/ros/humble/setup.bash && source rm_ws/install/setup.bash
export IDL_PACKAGE_FILTER="rm_interfaces;std_msgs;sensor_msgs;geometry_msgs;tf2_msgs;visualization_msgs;nav_msgs"
cargo build --no-default-features --features ros2

# 自瞄工作空间
bash sim/build_aim_ws.sh <本仓库>
```

## 二、遇到的问题与处理

### 1. r2r 构建期依赖（任务书预告的"ROS2 feature／构建配置硬编码类问题"）

**现象**：直接 `cargo build --features ros2`（或未 source 环境时）失败：
- 未 source ROS2 → build.rs panic `ROS_DISTRO not set: Source your ROS!`
- 未构建 rm_interfaces → 找不到 `rm_interfaces/msg/*.h` / 链接不到 `librm_interfaces__rosidl_typesupport_c.so`
- 不设 `IDL_PACKAGE_FILTER` 时，r2r 会为 `AMENT_PREFIX_PATH` 下**所有**消息包生成绑定，构建极慢

**根因**：`r2r_msg_gen` 的 bindgen 生成依赖 `AMENT_PREFIX_PATH`/`CMAKE_PREFIX_PATH` 扫描（r2r_common 0.9.5 `lib.rs:143-150,170-203,205-282`），并需要 libclang。

**处理**：先 colcon 构建 `rm_interfaces` 并 source，安装 `libclang-dev`，设置 `IDL_PACKAGE_FILTER` 白名单。
**是否修改仿真器代码：否**（仅构建方式）。

### 2. 上游文档与源码不符

**现象**：仿真器 README 称订阅 `/armor_solver/cmd_gimbal`，源码实际订阅 `/rm_gimbal/cmd`（`src/ros2/topic.rs:159-161`）。
**处理**：以源码为准，接口记录见 `docs/sim_interface_record.md`。

### 3. 国内网络：多处直连失败（部署中最耗时的问题）

| 失败点 | 现象 | 处理 |
| --- | --- | --- |
| `raw.githubusercontent.com` | SSL 连接被重置（ROS2 GPG key、单文件下载） | 改用清华镜像取 key；资源改用 `codeload.github.com`（实测 8.1 MB/s） |
| `static.rust-lang.org` | rustup 下载长时间卡死（15 分钟无进展） | 改用 `rsproxy.cn` 镜像安装 |
| `crates.io` | 直接拉取慢且易断 | 配置 sparse 镜像 `sparse+https://rsproxy.cn/index/` |
| `packages.ros.org` | 可用但慢 | 使用 `mirrors.aliyun.com/ros2/ubuntu` |
| VNC/GUI | 云服务器无显示器 | `Xvfb :0` + `x11vnc` + `noVNC`，SSH 隧道访问 |

### 4. 自瞄仓库在 Humble 上的构建问题

| 问题 | 现象 | 处理 |
| --- | --- | --- |
| vendored cv_bridge | `src/vision_opencv` 是 Foxy 时代 3.0.7（Boost.Python），与系统 `ros-humble-cv-bridge` 冲突 | 用白名单构建（`sim/build_aim_ws.sh`），不编译该目录 |
| 硬件/导航包 | `mv_publisher`（MindVision SDK）、`prm_autobot_2023`（navigation2）、`rplidar_ros`、`livox_ros_driver2` 缺 SDK/依赖 | 白名单排除 |
| `webcam_publisher` 安装失败 | `ament_cmake_symlink_install_directory() can't find .../resources`（该目录只含可选样片） | 修改 CMakeLists：`resources` 存在才安装（已提交） |

### 5. 仿真器源码修改记录（2 处，均为 ros2 feature 构建/运行阻塞问题）

任务书允许"为解决部署问题做必要修改"，本次共修改 1 个文件、2 处，补丁见
`docs/simulator_ros2_build_fix.patch`（基于上游 `master` 基线）。

| # | 文件:位置 | 现象 | 原因 | 修改 |
| --- | --- | --- | --- | --- |
| 1 | `src/ros2/plugin.rs`（导入区） | `cargo build --no-default-features --features ros2` 在最后的 daedalus crate 报 `E0599: no variant ... named bevy_default` | `TextureFormat::bevy_default()` 需要 `BevyDefault` trait 在作用域内；默认的 `talos` feature 恰好从别的路径引入了它，**只开 ros2 时会缺失**（即任务书预告的"ROS2 feature／构建配置硬编码类问题"） | 增加 `use bevy::image::BevyDefault;`（bevy 把 bevy_image 重导出为 `image`） |
| 2 | `src/ros2/plugin.rs:217,226`（`capture_rune`） | 运行时 panic：`called Option::unwrap() on a None value`，随后整机崩溃（`Encountered a panic in system capture_rune`） | `capture_rune` 为每块装甲板构建 TF/marker 时无条件 `unwrap()` 其 `CENTER` 子节点；场景加载过程中存在没有 CENTER 的装甲根节点 | 取不到 CENTER 时回退使用装甲根自身变换（不再 panic） |

修复后实测：仿真器正常启动并在 Tesla T4 上以 **≈65 FPS** 运行，全部接口按文档发布/订阅。

### 6. 其他说明

本次**未修改**仿真器的场景、物理、PID 等行为参数，仅做了上述两处阻塞性修复。

## 三、如需复现（从零到跑通）

```bash
# 1) 基础环境（Ubuntu 22.04 + GPU 驱动已预装）
sudo apt install -y build-essential cmake llvm-dev libclang-dev clang \
  ros-humble-ros-base ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-camera-info-manager ros-humble-nav-msgs ros-humble-tf-transformations \
  libopencv-dev libeigen3-dev python3-opencv python3-colcon-common-extensions \
  xvfb x11vnc novnc websockify ffmpeg
# 2) Rust（国内用 rsproxy 镜像）
export RUSTUP_DIST_SERVER=https://rsproxy.cn RUSTUP_UPDATE_ROOT=https://rsproxy.cn/rustup
curl -sSf https://rsproxy.cn/rustup/dist/x86_64-unknown-linux-gnu/rustup-init -o /tmp/r && \
  chmod +x /tmp/r && /tmp/r -y --default-toolchain stable --profile minimal
# 3) 虚拟显示
Xvfb :0 -screen 0 1920x1080x24 & x11vnc -display :0 -forever -shared -nopw -localhost -rfbport 5900 &
websockify --web=/usr/share/novnc 6080 localhost:5900 &
# 4) 按上面"构建命令"构建仿真器与自瞄工作空间，再按 README_SIM_INTEGRATION.md 启动
```
