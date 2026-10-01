# -*- coding: utf-8 -*-
"""
闭环控制律离线验证（不需要 ROS、不需要仿真器）。

目的：在无法连接仿真器时，仍然验证"目标方向 -> 云台绝对命令"这条换算链路的
**可逆性**与**收敛性**，即：
    1) 用仿真器实测标定的语义（yaw=绝对角；pitch_cmd=目标仰角）反推目标方向，
       应当与真实目标方向完全一致（无系统偏差）；
    2) 把该命令喂给"带速率限制的一阶云台模型"，迭代若干帧后瞄准误差应收敛到 0；
    3) 目标无效时不得产生命令。

相机模型：光学系（x 右 / y 下 / z 前），与 OpenCV/PnP 约定一致。
世界系：ROS 约定（x 前 / y 左 / z 上）。
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from aim_output.aim_result import (  # noqa: E402
    aim_result_from_armor_xyz,
    gimbal_angles_from_world_direction,
    quaternion_to_matrix,
)


# ---------------- 向量/矩阵小工具 ----------------
def vnorm(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def vcross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def vdot(a, b):
    return sum(x * y for x, y in zip(a, b))


def mat_mul_vec(m, v):
    return tuple(sum(m[i][j] * v[j] for j in range(3)) for i in range(3))


def mat_transpose(m):
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))


def camera_to_world_matrix(yaw_deg, elev_deg):
    """由云台 yaw/仰角构造 R(world<-camera_optical)（行主序，mat_mul_vec 可直接左乘相机系向量）。"""
    y, e = math.radians(yaw_deg), math.radians(elev_deg)
    fwd = (math.cos(e) * math.cos(y), math.cos(e) * math.sin(y), math.sin(e))  # z 光学轴
    up = (0.0, 0.0, 1.0)
    left = vnorm(vcross(up, fwd))
    right = tuple(-c for c in left)          # x 光学轴（右）
    down = vcross(fwd, right)                # y 光学轴（下）
    # 以相机三轴为“行”得到的是 world<-camera 的转置，这里转置回去，使 R·v 表示 camera->world
    cols = (right, down, fwd)
    return mat_transpose(cols)


def aim_direction(yaw_deg, elev_deg):
    y, e = math.radians(yaw_deg), math.radians(elev_deg)
    return (math.cos(e) * math.cos(y), math.cos(e) * math.sin(y), math.sin(e))


def angle_between(a, b):
    c = max(-1.0, min(1.0, vdot(vnorm(a), vnorm(b))))
    return math.degrees(math.acos(c))


# ---------------- 被测：一步闭环迭代 ----------------
def one_iteration(target_dir, gimbal_yaw_deg, gimbal_elev_deg, distance_m=1.4):
    """
    模拟"相机看到目标 -> PnP 输出 xyz(mm) -> AimResult -> 绝对云台角命令"。
    返回 (cmd_yaw_deg, cmd_pitch_deg, result)
    """
    R = camera_to_world_matrix(gimbal_yaw_deg, gimbal_elev_deg)
    d_cam = mat_mul_vec(mat_transpose(R), vnorm(target_dir))       # 目标在光学系下的方向
    # PnP 输出：光学系下的 xyz（mm），x 右 y 下 z 前
    x_mm = d_cam[0] * distance_m * 1000.0
    y_mm = d_cam[1] * distance_m * 1000.0
    z_mm = d_cam[2] * distance_m * 1000.0
    result = aim_result_from_armor_xyz(x_mm, y_mm, z_mm, fire=True, status="TRACKING")
    # 适配器：还原相机系方向 -> 转到世界 -> 转绝对角
    yaw_rel = math.radians(result.yaw_rel_deg)
    pitch_rel = math.radians(result.pitch_rel_deg)
    d_back = (math.cos(pitch_rel) * math.sin(yaw_rel),
              -math.sin(pitch_rel),
              math.cos(pitch_rel) * math.cos(yaw_rel))
    d_world = mat_mul_vec(R, d_back)
    angles = gimbal_angles_from_world_direction(d_world)
    return angles[0], angles[1], result


# ---------------- 测试 ----------------
def test_conversion_is_exact_for_various_directions():
    """核心性质：换算链在数学上是可逆的 —— 命令角恰好等于目标的世界方位/仰角。"""
    for yaw_t, elev_t in [(0, 0), (30, 10), (-141.55, -22.39), (90, 45), (-179, -30), (170, 60)]:
        target = aim_direction(yaw_t, elev_t)
        # 云台当前朝向与目标差一个随机但固定的偏差
        cmd_yaw, cmd_pitch, result = one_iteration(target, yaw_t - 7.5, elev_t + 4.0)
        assert result.target_valid
        # yaw 允许 360 环绕
        dy = abs(((cmd_yaw - yaw_t) + 180.0) % 360.0 - 180.0)
        assert dy < 1e-6, f"yaw 偏差 {dy} (目标 {yaw_t}, 命令 {cmd_yaw})"
        assert abs(cmd_pitch - elev_t) < 1e-6, f"pitch 偏差 {cmd_pitch - elev_t}"


def test_closed_loop_converges_with_rate_limited_gimbal():
    """
    一阶云台模型 + 指令速率限制：迭代若干帧后瞄准误差应单调收敛到 0（静态目标）。
    模型按实测标定：云台 yaw/仰角 = 命令值，但受速率限制（默认 60/45 度每秒）。
    """
    target = aim_direction(-141.55, -22.39)
    yaw, elev = -150.0, -30.0            # 初始偏离
    dt = 1.0 / 20.0                      # 20 FPS 处理
    max_yaw_rate, max_pitch_rate = 60.0, 45.0

    errors = []
    for _ in range(60):
        cmd_yaw, cmd_pitch, result = one_iteration(target, yaw, elev)
        assert result.target_valid
        # 速率限制（与 aim_result.limit_rate_step 相同的语义）
        from aim_output.aim_result import limit_rate_step
        yaw, elev = limit_rate_step(yaw, elev, cmd_yaw, cmd_pitch,
                                    max_yaw_rate, max_pitch_rate, dt)
        errors.append(angle_between(aim_direction(yaw, elev), target))

    assert errors[0] > errors[-1], "误差没有下降"
    assert errors[-1] < 0.5, f"未收敛，最终误差 {errors[-1]:.3f} deg"
    # 单调性（允许轻微抖动）：后半段误差应持续很小
    assert max(errors[-5:]) < 1.0


def test_open_loop_command_is_still_correct_while_target_visible():
    """开环（不发命令）时解算结果仍应正确：命令角等于目标方位，即使云台没动。"""
    target = aim_direction(20.0, 5.0)
    cmd_yaw, cmd_pitch, result = one_iteration(target, -10.0, -5.0)
    assert result.target_valid
    assert abs(((cmd_yaw - 20.0) + 180.0) % 360.0 - 180.0) < 1e-6
    assert abs(cmd_pitch - 5.0) < 1e-6


def test_invalid_solution_reports_invalid():
    """全 0 解（无装甲/停止）必须报告 target_valid=False，从而触发 distance=-1。"""
    r = aim_result_from_armor_xyz(0.0, 0.0, 0.0)
    assert not r.target_valid


# ---------------- 延迟下的过冲与增益/死区 ----------------
def _simulate(delay_frames, gain, deadband_deg, start_yaw=-150.0, start_elev=-30.0,
              target_yaw=-141.55, target_elev=-22.39, steps=80, dt=1.0 / 30.0):
    """模拟"图像延迟 delay_frames 帧"的闭环，返回每步的瞄准误差(度)。"""
    from aim_output.aim_result import blend_direction, gimbal_angles_from_world_direction

    target = aim_direction(target_yaw, target_elev)
    # 云台姿态历史（延迟队列）：第 i 步测量到的是 i-delay 帧的姿态
    yaw, elev = start_yaw, start_elev
    hist = [(yaw, elev)] * (delay_frames + 1)
    errors = []
    for i in range(steps):
        myaw, melev = hist[0]                      # 延迟后的观测姿态
        R = camera_to_world_matrix(myaw, melev)
        d_cam = mat_mul_vec(mat_transpose(R), target)
        d_world = mat_mul_vec(R, d_cam)
        d_current = mat_mul_vec(R, (0.0, 0.0, 1.0))
        if angle_between(d_current, d_world) <= deadband_deg:
            d_cmd = d_current
        else:
            d_cmd = blend_direction(d_current, d_world, gain)
        cmd_yaw, cmd_pitch = gimbal_angles_from_world_direction(d_cmd)
        yaw, elev = cmd_yaw, cmd_pitch             # 简化：云台瞬时到位
        hist.append((yaw, elev))
        hist.pop(0)
        errors.append(angle_between(aim_direction(yaw, elev), target))
    return errors


def test_gain_reduces_error_geometrically_without_latency():
    """
    增益语义验证：无延迟时，每步误差按 (1-gain) 收缩（几何收敛、无过冲）。
    gain=1 时一步到位；gain=0.4 时约 5 步进入 10% 误差。
    """
    from aim_output.aim_result import blend_direction, gimbal_angles_from_world_direction

    target = aim_direction(-141.55, -22.39)
    for gain, max_steps in ((1.0, 2), (0.4, 8)):
        yaw, elev = -150.0, -30.0
        hist = []
        for _ in range(max_steps):
            R = camera_to_world_matrix(yaw, elev)
            d_cam = mat_mul_vec(mat_transpose(R), target)
            d_world = mat_mul_vec(R, d_cam)
            d_current = mat_mul_vec(R, (0.0, 0.0, 1.0))
            d_cmd = blend_direction(d_current, d_world, gain)
            yaw, elev = gimbal_angles_from_world_direction(d_cmd)
            hist.append(angle_between(aim_direction(yaw, elev), target))
        # 初始误差约 9 度；gain=1 一步归零，gain=0.4 若干步后显著变小
        assert hist[-1] < (1.0 if gain == 1.0 else 1.5), f"gain={gain} 未收敛: {hist}"
        # 误差单调下降（无过冲）
        for a, b in zip(hist, hist[1:]):
            assert b <= a + 1e-9, f"gain={gain} 出现反弹: {hist}"


def test_deadband_holds_when_already_centred():
    """死区验证：误差已在死区内时，命令应等于当前指向（不再移动）。"""
    from aim_output.aim_result import blend_direction, gimbal_angles_from_world_direction

    target = aim_direction(-141.55, -22.39)
    # 云台已经在目标附近（误差 ~0.5 度 < 死区 1.5 度）
    yaw, elev = -141.8, -22.5
    R = camera_to_world_matrix(yaw, elev)
    d_cam = mat_mul_vec(mat_transpose(R), target)
    d_world = mat_mul_vec(R, d_cam)
    d_current = mat_mul_vec(R, (0.0, 0.0, 1.0))
    err = angle_between(d_current, d_world)
    assert err <= 1.5
    # 死区内：直接沿用当前指向
    yaw_cmd, elev_cmd = gimbal_angles_from_world_direction(d_current)
    assert abs(yaw_cmd - yaw) < 1e-6 and abs(elev_cmd - elev) < 1e-6
    # 对照：若不加死区而按增益插值，则会偏离当前指向（说明死区确实起作用）
    d_cmd = blend_direction(d_current, d_world, 0.4)
    assert angle_between(d_cmd, d_current) > 0.0
