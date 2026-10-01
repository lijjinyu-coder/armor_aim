# 版本信息

## 自瞄仓库（auto-aiming）

- 分支：`feature/simulator-integration`
- 标签：`sim-closed-loop-v1`
- HEAD：`ebe6f89b6e47ffe8ac02e420e2f421c010586829`
- 上游基线提交（用于比对差异）：`ab46a87`
- 基线来源：RoboMaster-Club/auto-aiming @ main（已用 SHA256 全量比对确认一致）

```
* feature/simulator-integration ebe6f89 feat(demo): dedicated acceptance recording showing the full closed-loop narrative
  master                        ab46a87 chore: baseline of auto-aiming before simulator integration
```

```
sim-closed-loop-v1 自瞄仿真接入与最小闭环 v1（含验收镜头）
```

## 仿真器（bevy_robomaster_simulator）

- HEAD：`3a84d07ad46a004442dcc6d1344786192cb90db3`
- 上游基线：`0536224`（Blackjack200/bevy_robomaster_simulator @ master，已比对一致）
- 本次修改：1 个文件 2 处（见 03_代码与补丁/simulator_ros2_build_fix.patch）

```
3a84d07 fix(ros2): make the ros2 feature build and stop capture_rune panicking
0536224 chore: upstream baseline of bevy_robomaster_simulator
```

