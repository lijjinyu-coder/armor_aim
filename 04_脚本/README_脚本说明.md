# 脚本说明

| 脚本 | 用途 |
| --- | --- |
| `build_aim_ws.sh` | 构建自瞄工作空间（白名单 colcon，避开 Foxy 版 cv_bridge 与硬件/导航包） |
| `run_sim.sh` | 启动仿真器（需先 source ROS2 与 rm_interfaces；启动后按 F5 开启自瞄订阅） |
| `run_aim_sim.sh` | 只启动自瞄链路（仿真模式） |
| `run_closed_loop_demo.sh` | 一键收尾：自动选目标 → 检测阈值自标定 → 最小闭环录制 → 目标丢失测试 |
| `acceptance_video.sh` | **验收镜头录制**：目标偏离中心 → 按 F5 开启自瞄 → 自主转向 → 稳定居中 → 目标丢失 |
| `run_scenarios.sh` | 运动场景测试（目标运动 / 己方运动）并录制 |
| `cleanup_nodes.sh` | 清理残留 ROS 节点（`ros2 launch` 被 kill 后子节点仍会运行） |
| `record_demo.sh` | 用 ffmpeg 录制 Xvfb 画面 |
| `make_demo_video.sh` | 服务器端把分段录制合并为带标题卡的成品（依赖 drawtext） |
| `make_demo_local.py` | 本地合并（用 PIL 渲染中文标题卡，避开 drawtext 的 Windows 路径问题） |
| `test_aim_output_logic.sh` | 运行 aim_output 的纯逻辑单元测试（20 项，无需 ROS） |
| `extract_acc_frames.sh` | 从验收录像抽取关键帧（供制作逐帧对照图） |
| `服务器端工具/` | 服务器上使用的诊断/取数脚本：`tf_rpy.py`（读 TF 姿态）、`aim_dir.py`（读实际瞄准方向）、`save_frame.py`（保存图像帧）、`list_armors.py`（列出装甲板真值方位）、`start_sim.sh`、`probe_interfaces.sh`、`run_all_v2.sh`、`run_scenarios_v2.sh` |
| `服务器环境脚本/` | 从零搭建服务器环境：`01_bootstrap_aliyun.sh`（apt/ROS2/Rust/VNC）、`02_build_rm_interfaces.sh`、`03_build_simulator.sh`、`04_git_init.sh` |
| `ssh_run.py` | 本地到服务器的 SSH/SFTP 助手（**已脱敏，需自行填入地址与凭据或改用密钥**） |
| `resume_and_finish.py` | 一键收尾：等待服务器 → 上传改动 → 重新编译 → 跑闭环与场景 → 回传视频 |

> 开发过程中还写过若干一次性诊断脚本（F5 状态、pitch 映射标定、窗口布局等），
> 未收录于此；结论已写入 `01_文档/接口确认记录.md` 与 `01_文档/部署与排查记录.md`。
