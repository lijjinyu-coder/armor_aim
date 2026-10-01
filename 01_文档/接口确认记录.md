# 仿真器 ROS2 接口确认记录

接口内容来自**仿真器源码核实**（文件:行号）与服务器实测，两者相互对照（任务书 §二 要求）。
源码路径约定：`S` = bevy_robomaster_simulator 仓库根目录。

## 一、发布话题

| 话题 | 类型 | 关键内容 | QoS | 依据 |
| --- | --- | --- | --- | --- |
| `/image_raw` | `sensor_msgs/Image` | **encoding=`rgb8`**（RGB 顺序，不是 BGR）；`step=width*3`；`frame_id=camera_optical_frame`；分辨率来自 `S/config.toml [capture.color]`（默认 1440×1080）；每个渲染帧一帧、无节流；时间戳为**系统墙钟** | 默认（KeepLast(10)+Reliable） | `S/src/ros2/capture.rs:142-152`、`S/src/ros2/topic.rs:118,147` |
| `/camera_info` | `sensor_msgs/CameraInfo` | `distortion_model=plumb_bob`，**d 全 0（零畸变）**；`fy=H/(2·tan(fov_y/2))`、`fx=fy`、`cx=W/2`、`cy=H/2`；`fov_y` 取 `[camera].fov`（**45° 是垂直视场角**）；与图像同 stamp | 默认 Reliable | `S/src/ros2/capture.rs:154-199`、`S/src/capture.rs:145-151`、`S/src/ros2/plugin.rs:403` |
| `/image_compressed` | `sensor_msgs/CompressedImage` | `publish_compressed` 硬编码 `false`，ROS2 构建下**不发布** | — | `S/src/ros2/plugin.rs:429` |
| `/tf` | `tf2_msgs/TFMessage` | 树：`map → odom → gimbal_link → {muzzle → muzzle_link, camera_link → camera_optical_frame}`；坐标系已对齐 ROS 习惯（x 前 / y 左 / z 上，单位米、四元数 xyzw） | 默认 | `S/src/ros2/plugin.rs:190-221`、`S/src/ros2/prelude.rs:37-62` |
| `/gimbal_pose` `/odom_pose` `/muzzle_pose` `/camera_pose` | `geometry_msgs/PoseStamped` | **pose 字段恒为原点+单位四元数**，只有 `frame_id`/`stamp` 有意义；**真实姿态必须从 `/tf` 取** | 默认 | `S/src/ros2/prelude.rs:86-106` |
| `/simulator/tech_core/state` | `std_msgs/String` | 前哨站/机关状态 JSON，20 Hz 限流 | 默认 | `S/src/ros2/plugin.rs:422-423` |
| `/simulator/marker` | `visualization_msgs/Marker` | 每块装甲板一个 CUBE（map 系，lifetime 0.3s 续期） | 默认 | `S/src/ros2/plugin.rs:224-276` |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 仿真激光雷达点云，10 Hz（`[livox_ros]` 控制） | 默认 | `S/src/ros2/topic.rs:150` |

## 二、订阅话题

| 话题 | 类型 | QoS | 依据 |
| --- | --- | --- | --- |
| `/rm_gimbal/cmd` | `rm_interfaces/msg/GimbalCmd` | **sensor_data：KeepLast(5)+BestEffort+Volatile**（发布端必须用 `rclcpp::SensorDataQoS()` / `qos_profile_sensor_data`） | `S/src/ros2/topic.rs:159-161` |

`rm_interfaces/msg/GimbalCmd`：

```
std_msgs/Header header
float64 pitch  float64 yaw  float64 yaw_diff  float64 pitch_diff
float64 distance  bool fire_advice
```

| 字段 | 语义（源码核实） |
| --- | --- |
| `yaw` | **枪口绝对 yaw，单位度**，世界系；绕竖直轴逆时针（左转）为正 |
| `pitch` | **枪口绝对 pitch，单位度，从竖直轴量起**：90=水平，>90 上仰，<90 下俯（内部换算 `pitch_rad=(pitch_deg-90)°`，见 `S/src/systems/gimbal_pid.rs:13-24`） |
| `yaw_diff` / `pitch_diff` | **仿真器完全不读取**（全仓库零引用），发送时可填 0 |
| `distance` | 除 **`-1.0` = "无解"哨兵**（移除跟踪器、PID 停止输出、云台保持姿态）外，数值本身不被使用（不做弹道补偿） |
| `fire_advice` | true 触发发射（仿真器内部 10 Hz 限频 + 弹丸冷却） |

## 三、运行时行为（重要）

1. **必须先按 F5**（或手柄 RT）打开"自瞄订阅"开关（`SubscribeAutoAim` 初始为 false，`S/src/main.rs:153`）；关闭时命令被丢弃。
2. 关闭期间最后一条命令会被保留，重新开启会立即重发该陈旧命令（本工程按帧持续发布，正常使用不会触发）。
3. 云台由独立单轴 PID 驱动：kp=50 / kd=0.1 / max_rate=100 rad/s，pitch 限幅 ±0.785 rad（±45°），yaw 无限幅（`S/config.toml:69-81`）。
4. 订阅为"最新覆盖式"存储 + 每帧 drain（一帧内最后一条生效）。
5. 时间基准为系统墙钟（RCL_SYSTEM_TIME），**不要启用 `use_sim_time`**。

## 四、相机内参（计算值 ↔ 实测核对）

默认 1440×1080、`[camera].fov = 45°`（垂直）时：

- `fy = 1080 / (2·tan 22.5°) ≈ 1303.675`
- `fx = fy ≈ 1303.675`（由 `fov_x = 2·atan(tan(fov_y/2)·W/H)` 推导，结果恒等于 fy）
- `cx = 720`、`cy = 540`，畸变系数全 0（可直接用针孔模型 PnP）

> **交叉核对方法**：`ros2 topic echo /camera_info --once` 与上式对比；
> 若修改 `[capture.color]` 或 `[camera].fov`，必须同步更新 `sim_detector.py` 的
> `image.width/height` 与 `fx/fy/cx/cy`，否则 PnP 距离/角度会系统性偏差。

## 五、实测核对结果（服务器实测，2026-10-01）

环境：阿里云 `ecs.gn6i-c8g1.2xlarge`（Tesla T4）、Ubuntu 22.04、ROS2 Humble。
方法：`ros2 node/topic list|info|echo|hz`，以及自编写的 TF/瞄准方向读取工具（`tf_rpy.py`、`aim_dir.py`）。

| 项 | 源码/计算预期 | 实测值 | 结论 |
| --- | --- | --- | --- |
| 节点 | `/robomaster/simulator` | `/robomaster/simulator` | ✅ |
| `/image_raw` 编码 | rgb8 | **rgb8** | ✅ |
| `/image_raw` 分辨率 | 1440×1080（`[capture.color]`） | **1440×1080**，step=4320 | ✅ |
| `/image_raw` 频率 | = 渲染帧率 | **≈63 Hz**（渲染 ≈66 FPS，T4） | ✅ |
| `/image_raw` QoS | 默认 Reliable | **RELIABLE / VOLATILE** | ✅ |
| `/camera_info` 内参 | fx=fy=1303.675, cx=720, cy=540, d=0 | **fx=fy=1303.675283386667, cx=720.0, cy=540.0, d=[0,0,0,0,0]** | ✅ 与公式完全一致 |
| `/rm_gimbal/cmd` QoS | sensor_data(BestEffort) | **BEST_EFFORT / VOLATILE**，订阅者=simulator | ✅ |
| `/tf` 频率 | 每帧 | ≈65 Hz | ✅ |
| 话题清单 | 见上表 | `/camera_info /camera_pose /gimbal_pose /image_compressed /image_raw /livox/lidar /muzzle_pose /odom_pose /rm_gimbal/cmd /simulator/marker /simulator/tech_core/state /tf` | ✅ |

### 云台命令实测标定（重要）

**yaw**：发送 `yaw=+10/-10/+37` 后，`odom→gimbal_link` 的 yaw 精确等于命令值（+10.00/-10.00/+37.00）→
**yaw = 绝对世界 yaw，逆时针（左转）为正，单位度** ✅

**pitch**：命令值与 `muzzle` 帧**实际瞄准仰角**的对应关系（自瞄开启下逐点测量）：

| pitch 命令 | -20 | -10 | 0 | 10 | 20 | 30 | 40 | 50 | 60 | 70 | 80 | 90 | 100 | 110 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 实际仰角(°) | -19.98 | -10.00 | 0.00 | +10.00 | +20.00 | +30.00 | +40.00 | +50.00 | +60.00 | +69.98 | +69.98 | +69.98 | +69.98 | +69.98 |

结论：**`pitch` 命令值 = 目标仰角（相对水平面，向上为正），可用范围约 ±70°，超出后限幅**。

> ⚠️ 这与只读源码得到的印象不同：`src/systems/gimbal_pid.rs:13-24` 的
> `from_solver_degrees` 把 pitch 当作"从竖直轴量起（90=水平）"（内部 `pitch-90`），
> 但**实测有效映射是恒等仰角映射**。任务书 §六 明确要求"不能将 PnP 输出不加判断地直接
> 填入命令"——本工程的换算以实测标定为准（`aim_output/aim_result.py`）。
> 交叉验证：命令 `yaw=0, pitch=0` 后，`muzzle` 的实际瞄准方向为 `(+1.000, 0.000, -0.000)`，
> 即**恰好水平朝前**，验证了该标定。

### 其他实测行为

| 行为 | 实测结果 |
| --- | --- |
| F5 门控 | 未按 F5 时发送 `pitch=120` 云台完全不动；按 F5 后同一命令立即生效 ✅ |
| 手动控制互斥 | 自瞄开启时方向键不再控制云台（手动按 Up/Down 无反应）；关闭后方向键正常 ✅ |
| `distance=-1` | 发送后云台停止驱动并**保持当前姿态**（实测姿态不再变化）✅ |
| 图像时间戳 | 系统墙钟（Unix epoch，见 `sec: 1790792834`），无需 `use_sim_time` ✅ |
| 底盘/装甲板 | `/simulator/marker` 实测 18 块装甲板位置（map 系），可用于目标真值/目标选择 |
