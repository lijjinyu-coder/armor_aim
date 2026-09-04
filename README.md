用 C++ + OpenCV 写的简化版装甲板自瞄程序，用 CMake 构建。

## 一、环境依赖

- 系统  Windows（MSYS2 UCRT64 环境）
- 编译器  g++（C++17）
- 构建  CMake ≥ 3.16 + Ninja
- 依赖  OpenCV 5.x

在 MSYS2 UCRT64 终端里安装依赖：

```
pacman -S mingw-w64-ucrt64-x86_64-gcc mingw-w64-ucrt64-x86_64-cmake \
           mingw-w64-ucrt64-x86_64-ninja mingw-w64-ucrt64-x86_64-opencv
```

## 二、如何编译

所有命令都在 MSYS2 UCRT64 终端里执行。

第一次编译：

```
cmake -B build
cmake --build build
```

以后改完代码再编译，只需要执行 `cmake --build build`。
编译产物是 `build/armor_aim.exe`。

## 三、如何运行

```
./build/armor_aim.exe                # 读 config/config.yaml 里填的视频，跑完整流程
./build/armor_aim.exe --video xx.mp4 # 临时指定一个视频
./build/armor_aim.exe --config xx.yaml  # 指定配置文件
```

运行中按 Q 或 ESC 退出。

## 四，输出结果

所有运行结果都在 output/ 目录

## 五、项目有哪些模块、每个模块负责什么

- VideoInput  `VideoInput.h/.cpp`  打开视频，一帧一帧读取，不管检测
- ArmorDetector  `ArmorDetector.h/.cpp`  预处理、找灯条并筛选、左右灯条匹配成装甲板
- TargetSelector  `TargetSelector.h/.cpp`  从多个候选中选一个目标，维护目标状态（发现/跟踪/丢失）
- PoseSolver  `PoseSolver.h/.cpp`  用四个角点 solvePnP，算出 yaw / pitch / distance
- DebugDrawer  `DebugDrawer.h/.cpp`  画候选框、目标框、写文字信息
- ConfigLoader  `ConfigLoader.h/.cpp`  读 config.yaml 里的阈值和参数
- DataPacker  `DataPacker.h/.cpp`  把 yaw/pitch/distance/状态打包成 8 字节报文
- DataParser  `DataParser.h/.cpp`  解析 8 字节报文、校验校验位、还原数据
- Common  `Common.h`  公共类型（目标状态、灯条、装甲板、位姿等）

主流程在 main.cpp：把上面这些模块串起来，逐帧处理并显示。

## 六、使用了哪些主要参数

参数集中在 `config/config.yaml`，主要的几类：

- 视频输入  `video_path`、`save_video`、`show_debug`
- 敌方颜色 / 预处理  `enemy_color`（BLUE/RED）、`preprocess_mode`（HSV/BGR/GRAY）、HSV 阈值、灰度阈值、通道差分阈值
- 灯条筛选  `lightbar_min_area`（面积）、`lightbar_min_ratio`（长宽比）、`lightbar_max_angle`（角度）
- 装甲板匹配  `armor_min_ratio` / `armor_max_ratio`（宽高比）、`armor_angle_sum_max`（内八）、`armor_length_diff_ratio`、`armor_y_offset_ratio`
- 目标跟踪  `lost_threshold`（连续丢失多少帧算没目标）
- 装甲板尺寸  `armor_width_mm` / `armor_height_mm`

约定：数值填 `0` 表示"没配"，程序会用内置的示例默认值。
相机内参（fx=fy=800, cx=320, cy=240）因为是近似值，直接写在 `PoseSolver.cpp` 里，没有放到配置里，也没有做标定。

## 七、当前检测效果如何

用默认参数跑给定测试视频（`videos/2607071529162881.mp4`，共 625 帧）：

- 跟踪到目标（TRACKING）：465 帧（74%）；
- 短暂丢失（LOST）：60 帧；未发现目标（NO_TARGET）：100 帧；
- 有目标时 distance 范围 **37 ~ 1943 mm**，平均约 596 mm，无明显跳变；
- yaw 范围约 -11.8° ~ +18.5°，pitch 约 -15.3° ~ +13.8°；
- 625 帧报文全部 `checksum=OK`，没有出错帧。

结果视频在 `output/output.avi`，报文记录在 `output/comm.log`。

## 八、存在的问题

1. 相机内参用的是近似值，没有标定，所以 distance 会有偏差；
2. 视频路径和各阈值要按自己的视频填，内置示例默认值不保证检测效果；
3. 背景复杂（画面里有多块同色区域）时可能误检；

## 九、阶段二：报文格式说明

### 1. 发送了哪些数据

每处理一帧，就把视觉程序输出的 yaw、pitch、distance、目标状态 打包成一包 8 字节报文（报文 ID `0x301`），在终端打印并在 `output/comm.log` 里记录。然后模拟接收方把报文解析回来，还原出这四个量。

- 0-1  yaw  有符号整数，小端序
- 2-3  pitch  有符号整数，小端序
- 4-5  distance  无符号整数，小端序
- 6  目标状态  0=未发现 1=检测到 2=跟踪中 3=短暂丢失
- 7  校验位  前 7 个字节相加后取低 8 位

### 2. 每个字段的单位是什么

- yaw / pitch 单位是**度**，打包时先 **× 100** 取整，接收端再 ÷ 100 还原（精度 0.01°）；
- distance 单位是**毫米**，直接取整存 uint16（量程 0~65535mm）；
- 目标状态是 0~3 的数字。

### 3. 为什么需要数据缩放

报文里只能存整数，而角度是带小数的（比如 7.73°）。所以约定乘 100 存整数、接收端除 100 还原，收发两边规则一致，精度不会丢。

### 4. 为什么需要规定字节序

一个 int16 占 2 个字节，先发哪一字节必须约定。本项目用**小端序**（低位在前），不规定的话发送方和接收方读出来的数会完全不同。

### 5. 为什么不直接发送 float

float 在内存里的格式复杂，直接看字节根本看不懂含义，也没法做数值校验。"整数 × 缩放"的格式每个字节含义明确、直观、容易调试和校验。

### 6. 无目标时发送什么

没有目标时也照常发一包完整数据：yaw=0、pitch=0、distance=0、state=0（NO_TARGET）。接收端靠 `state=0` 判断当前没有目标。

### 7. 接收端如何判断数据是否有效

把报文前 7 个字节加起来取低 8 位，和第 8 字节比对：一样 → `checksum=OK`，数据有效；不一样 → `checksum=FAIL`，说明这包数据在传输中出错。
