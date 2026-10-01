# 自瞄仿真接入与最小闭环 —— 使用说明

本文件说明如何把本仓库的自瞄算法接入 [bevy_robomaster_simulator](https://github.com/Blackjack200/bevy_robomaster_simulator)（Daedalus），
完成"仿真图像 → 原算法 → 云台命令 → 仿真云台运动 → 新图像"的实时闭环。

对应的任务批次：视觉组培训任务《自瞄仿真接入与最小闭环任务说明》。
分支：`feature/simulator-integration`；标签：`sim-closed-loop-v1`。

---

## 一、环境与依赖

| 项 | 版本/说明 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 LTS |
| ROS2 | Humble（`ros-humble-ros-base` 及以上） |
| 仿真器 | bevy_robomaster_simulator（Rust + Bevy 0.19，ROS2 集成走 r2r 0.9.5） |
| 编译器 | g++ / cmake / colcon；Rust stable ≥ 1.85（Bevy 0.19 需要 edition 2024） |
| 其他 | OpenCV (`libopencv-dev`)、Eigen3、`ros-humble-cv-bridge`、`ros-humble-image-transport`、`ros-humble-camera-info-manager`、`ros-humble-tf-transformations` |

在云服务器上可用（本仓库不含脚本，见部署记录）：

```bash
sudo apt install -y build-essential cmake \
  ros-humble-ros-base ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-camera-info-manager ros-humble-nav-msgs ros-humble-tf-transformations \
  libopencv-dev libeigen3-dev python3-opencv python3-colcon-common-extensions
```

> 仿真器在无显示器环境下需要虚拟 X（`Xvfb :0` + `x11vnc`/`noVNC`）才能看到画面。

---

## 二、编译

### 2.1 仿真器（含自定义消息 rm_interfaces）

仿真器的 ROS2 集成通过 r2r 的 bindgen 生成消息绑定，**构建期要求**：

1. 已经 `source /opt/ros/humble/setup.bash`（`ROS_DISTRO`、`AMENT_PREFIX_PATH` 必须存在，否则 r2r 直接 panic）；
2. `rm_interfaces` 已用 colcon 构建并 source（bindgen 需要它生成的头文件与 `librm_interfaces__rosidl_typesupport_c.so`）；
3. 安装 `libclang-dev`（bindgen 依赖）；
4. 建议设置 `IDL_PACKAGE_FILTER` 限定消息范围，否则会为环境中所有消息包生成绑定（极慢）。

```bash
# 1) 构建消息包
cd <仿真器目录>/rm_interfaces
source /opt/ros/humble/setup.bash
colcon build --base-paths . --build-base rm_ws/build --install-base rm_ws/install --merge-install

# 2) 构建仿真器（只用 ros2 feature，关闭默认的 talos）
cd <仿真器目录>
source /opt/ros/humble/setup.bash
source rm_ws/install/setup.bash
export IDL_PACKAGE_FILTER="rm_interfaces;std_msgs;sensor_msgs;geometry_msgs;tf2_msgs;visualization_msgs;nav_msgs"
cargo build --no-default-features --features ros2
```

### 2.2 本仓库（自瞄）

```bash
source /opt/ros/humble/setup.bash
cd <本仓库>
colcon build --symlink-install --packages-select \
  vision_msgs rm_interfaces opencv_armor_detector pose_estimator \
  webcam_publisher aim_output sim_tests prm_launch \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**为什么不直接 `colcon build` 全量构建？** 仓库内的 `src/vision_opencv` 是 Foxy 时代的
cv_bridge（Boost.Python 实现），与 Humble 系统包同名冲突；`mv_publisher`（MindVision 私有 SDK）、
`prm_autobot_2023`（navigation2）、`rplidar_ros`、`livox_ros_driver2` 依赖硬件 SDK 或导航栈，
仿真用不到。因此仿真链路由上面的白名单构建。
（`rm_interfaces` 是仿真器的消息包，已复制进本仓库，使自瞄侧能独立编译 `/rm_gimbal/cmd` 的类型支持。）

---

## 三、启动方式与模式切换

### 3.0 单入口模式切换（推荐）

`prm_launch/autoaim.py` 用一个启动参数切换输入源与输出行为，核心算法完全不变：

```bash
# 仿真模式（图像来自仿真器，内参按仿真画面，闭环输出）
ros2 launch prm_launch autoaim.py input:=ros2 publish_command:=true

# 视频模式（图像来自视频文件，内参沿用真机旧相机默认值，只做调试输出）
ros2 launch prm_launch autoaim.py input:=video video_path:=/path/to/video.avi
```

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `input` | `ros2` | **输入方式**：`ros2`（仿真器 `/image_raw`）或 `video`（`webcam_publisher` 读文件） |
| `publish_command` | `true` | `false` = 开环（只算不发），满足任务书 §四 验收 |
| `image_width`/`image_height` | `1440`/`1080` | 检测输入尺寸，须与仿真器 `[capture.color]` 一致 |
| `fx`/`fy`/`cx`/`cy` | `1303.675`/`1303.675`/`720`/`540` | 仿真内参（仅 `input:=ros2` 使用） |
| `target_red` / `color_set` | `true` / `red` | 目标颜色（仿真场景中敌方为蓝色时设 `false`/`blue`） |
| `max_yaw_rate_deg_s`/`max_pitch_rate_deg_s` | `60`/`45` | 指令速率限制（0=关闭），用于抑制视觉延迟导致的过冲 |
| `show_window`/`debug_image` | `false`/`true` | OpenCV 窗口 / `/debug_image` 标注图 |

### 3.1 仿真模式（ROS2 实时图像 → 闭环）

```bash
# 终端 A：仿真器（需要 DISPLAY）
source /opt/ros/humble/setup.bash && source <仿真器目录>/rm_ws/install/setup.bash
cd <仿真器目录> && DISPLAY=:0 cargo run --no-default-features --features ros2
#   ↑ 启动后在仿真器窗口按 F5 打开"自瞄订阅"开关，否则云台命令不会被处理

# 终端 B：自瞄链路（图像由仿真器提供，不启动任何相机/视频节点）
source /opt/ros/humble/setup.bash && source <本仓库>/install/setup.bash
ros2 launch prm_launch sim_detector.py               # 默认闭环（发 /rm_gimbal/cmd）
ros2 launch prm_launch sim_detector.py publish_command:=false   # 开环：只算不发
```

`sim_detector.py` 常用参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `publish_command` | `true` | `false` 时只做检测/解算/调试输出（任务书 §四 开环验收） |
| `target_red` | `true` | 目标颜色 |
| `image_width`/`image_height` | `1440`/`1080` | 检测输入尺寸，必须与仿真器 `[capture.color]` 一致 |
| `fx`/`fy`/`cx`/`cy` | `1303.675`/`1303.675`/`720`/`540` | 相机内参，对应上面的分辨率与 `[camera].fov=45°` |
| `yaw_sign`/`yaw_offset_deg`/`pitch_offset_deg` | `1.0`/`0`/`0` | 现场微调用的符号与偏置 |
| `watchdog_timeout_ms` | `200` | 超过该时间没有新解算结果即认为目标丢失 |
| `show_window` / `debug_image` | `false` / `true` | 是否弹出 OpenCV 窗口 / 是否发布 `/debug_image` |

> 若修改仿真器 `config.toml` 的 `[capture.color]` 或 `[camera].fov`，必须同步修改
> `image.width/height` 与 `fx/fy/cx/cy`（`fy = H / (2*tan(fov_y/2))`，`fx = fy`，`cx=W/2`，`cy=H/2`），
> 否则 PnP 解算的距离与角度会系统性偏差。

### 3.2 视频模式（原链路保留）

```bash
ros2 launch prm_launch video2detector.py video_path:=/path/to/video.avi
#  默认只做调试输出（不发云台命令），相机内参沿用真机旧相机默认值
```

### 3.3 最小通信测试（任务书 §三）

```bash
ros2 run sim_tests image_probe --display 1     # 订阅 /image_raw：宽高/编码/时间戳/接收 FPS
ros2 run sim_tests gimbal_probe --yaw 0 --pitch 90 --hold    # 向 /rm_gimbal/cmd 发固定命令
ros2 run sim_tests gimbal_probe --stop                        # 发 distance=-1 停止追踪
```

---

## 四、代码结构与模块职责

```
输入层（图像来源，可切换，核心算法不感知）
  webcam_publisher/VideoCaptureNode   视频文件/USB 相机 → /image_raw (bgr8)
  （仿真模式）仿真器自身发布            → /image_raw (rgb8)，由 cv_bridge 统一为 bgr8

核心算法层（与真机一致，未因仿真而改动检测/解算逻辑）
  opencv_armor_detector/OpenCVArmorDetectorNode
        /image_raw → resize(image.width×image.height) → 颜色/轮廓 → /key_points
  pose_estimator/PoseEstimatorNode
        /key_points → solvePnP(IPPE) + yaw 优化 + 有效性过滤 → /predicted_armor

输出抽象层（只消费统一结果 AimResult，不碰核心算法）
  aim_output/aim_output_node
        /predicted_armor → AimResult(yaw_rel, pitch_rel, distance, target_valid)
          ├─ debug_output  : RCLCPP 日志 + /debug_image（装甲候选、角度、距离、FPS、状态）
          └─ gimbal_output : /rm_gimbal/cmd（绝对角/度，sensor-data QoS）+ 目标丢失看门狗

接口包
  vision_msgs   ：KeyPoints(8 角点 + is_large_armor)、PredictedArmor(x/y/z mm, fire)
  rm_interfaces ：仿真器消息定义（GimbalCmd 等），复制自仿真仓库
```

### 数据流

```
[仿真器虚拟相机] --/image_raw(rgb8)--> [检测器] --/key_points--> [PnP 解算] --/predicted_armor-->
        ^                                                                              |
        |                                                                       [aim_output]
        |                                                                       /          \
   云台运动 <----- [仿真器云台 PID] <---- /rm_gimbal/cmd(绝对角/度) ---- ROS2输出      调试输出(日志//debug_image)
```

---

## 五、接口与单位（已按仿真器源码核实）

| 接口 | 话题 | 类型/单位 |
| --- | --- | --- |
| 图像 | `/image_raw` | `sensor_msgs/Image`，**encoding=rgb8**，默认 1440×1080，frame_id=`camera_optical_frame`，QoS=Reliable |
| 相机信息 | `/camera_info` | `sensor_msgs/CameraInfo`，plumb_bob，**畸变全 0**，`fx=fy=H/(2tan(fov_y/2))`，`cx=W/2`，`cy=H/2` |
| 检测结果 | `/key_points` | `vision_msgs/KeyPoints`，8 个 float32（4 角点 x,y），`is_large_armor` |
| 解算结果 | `/predicted_armor` | `vision_msgs/PredictedArmor`，`x/y/z` **mm**（相机系），`fire` |
| 姿态 | `/tf` | 树：`map→odom→gimbal_link→{muzzle, camera_link→camera_optical_frame}`；**`/gimbal_pose` 等 PoseStamped 的 pose 恒为单位阵，姿态必须从 /tf 取** |
| 云台命令 | `/rm_gimbal/cmd` | `rm_interfaces/GimbalCmd`，QoS=**sensor_data(BestEffort)**；`yaw`/`pitch` 为**绝对角、单位度**；`distance=-1.0` 表示无解；`yaw_diff`/`pitch_diff` 仿真器不读取 |

**`pitch` 的实测语义（务必注意，已用实验标定）**：命令值**等于目标仰角**（相对水平面，向上为正），
实测对应关系见 `docs/sim_interface_record.md` 第五节（`-20→-19.98`、`0→0.00`、`60→+60.00`，约 ±70° 后限幅）。
只读源码会以为它是"从竖直轴量起（90=水平）"，**以实测为准**。

内部统一结果 `AimResult`：

| 字段 | 单位 | 含义 |
| --- | --- | --- |
| `yaw_rel_deg` | 度 | 目标相对相机光轴方位角（视线法 `atan2(x,z)`，右为正） |
| `pitch_rel_deg` | 度 | 目标相对水平面仰角（`atan2(-y,√(x²+z²))`，上为正） |
| `distance_m` | 米 | `√(x²+y²+z²)` |
| `target_valid` | bool | 解算有效（全 0 解、超时、TF 不可用均为 false） |

**命令换算式**（`aim_output` 每帧执行，误差不累积）：

```
d_cam   = 由 AimResult 还原的相机系单位方向
d_world = R(world←camera_optical_frame) · d_cam          # 旋转取自 /tf
yaw_cmd   = atan2(d_world.y, d_world.x) × yaw_sign + yaw_offset_deg    # 绝对角，左转为正
pitch_cmd = 90° - acos(d_world.z) + pitch_offset_deg                   # = 目标仰角
```

> 实测验证：命令 `yaw=0, pitch=0` 后 `muzzle` 的实际瞄准方向为 `(+1.000, 0.000, -0.000)`（恰好水平朝前）。

---

## 六、目标丢失与异常处理

| 情形 | 上游行为 | 本工程行为 |
| --- | --- | --- |
| 无装甲 / 距离无效（NO_ARMOR） | 发布全 0 `predicted_armor` | 发 `distance=-1`（仿真器移除跟踪器、云台保持姿态） |
| PnP 无效（STOPPING） | 发布全 0 | 同上 |
| PnP 无效（IDLING） | **不发布任何消息** | 看门狗 `watchdog_timeout_ms` 超时 → 发 `distance=-1` |
| 仿真器/图像中断 | 无新消息 | 同上（看门狗） |
| 取不到 TF | — | 视为无效，只做调试输出、**不发命令**避免误驱动云台 |

> 仿真器的"无解"处理是移除 `GimbalAimTracker` 组件，PID 随即停止输出并保持当前姿态；
> 这也是"不使用过期目标"的落点（`src/ros2/plugin.rs` 中 `distance == -1.0` 分支）。

---

## 七、已知问题

1. 仿真器 README 写的订阅话题 `/armor_solver/cmd_gimbal` 与源码不符，**实际是 `/rm_gimbal/cmd`**。
2. 仿真器自瞄订阅默认关闭，**必须按 F5**；关闭期间最后一条命令会被保留，重新开启时会立即重发该陈旧命令（本工程按帧持续发布，正常使用不会触发）。
3. 仿真器所有时间戳为系统墙钟（RCL_SYSTEM_TIME），**不要启用 `use_sim_time`**。
4. 仿真器运行期热改 `config.toml` 不会影响已创建的图像捕获配置，需重启。
5. `webcam_publisher` 的 `frame_id` 原先每帧追加序号（已修复），否则 TF 查询会因 frame 名不断变化而失败。
