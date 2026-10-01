#!/usr/bin/env bash
# 运行 aim_output 的纯逻辑单元测试（不需要 ROS，可在任意机器上执行）
# 用法: bash sim/test_aim_output_logic.sh
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$HERE/../src/prm_control/aim_output"
python3 -m pytest -q "$PKG/test/test_aim_result.py" 2>/dev/null || \
python3 - "$PKG" <<'PY'
# 没有 pytest 时的等价检查（纯逻辑，无 ROS 依赖）
import math, sys, os
pkg = sys.argv[1]
sys.path.insert(0, os.path.join(pkg, "aim_output", ".."))
sys.path.insert(0, pkg)
from aim_output.aim_result import (aim_result_from_armor_xyz, gimbal_angles_from_world_direction,
                                   quaternion_to_matrix, rotate_vector, limit_rate_step,
                                   shortest_angle_delta)
checks = []
def chk(name, cond): checks.append((name, bool(cond)))

chk("zero solution invalid", not aim_result_from_armor_xyz(0, 0, 0).target_valid)
r = aim_result_from_armor_xyz(0, 0, 3000)
chk("straight ahead 3m", r.target_valid and abs(r.distance_m - 3.0) < 1e-9)
r = aim_result_from_armor_xyz(1000, -1000, 1000)
chk("right and above", abs(r.yaw_rel_deg - 45) < 1e-6 and abs(r.pitch_rel_deg - 35.264) < 1e-2)
y, p = gimbal_angles_from_world_direction((1.0, 0.0, 0.0))
chk("level -> pitch 0", abs(y) < 1e-9 and abs(p) < 1e-9)
y, _ = gimbal_angles_from_world_direction((0.0, 1.0, 0.0))
chk("left -> yaw +90", abs(y - 90) < 1e-9)
_, p = gimbal_angles_from_world_direction((1.0, 0.0, 1.0))
chk("up 45 -> pitch +45", abs(p - 45) < 1e-6)
_, p = gimbal_angles_from_world_direction((1.0, 0.0, -1.0))
chk("down 45 -> pitch -45", abs(p + 45) < 1e-6)
m = quaternion_to_matrix(0, 0, 0, 1)
chk("identity rotation", rotate_vector(m, (1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0))
s = math.sin(math.radians(45))
m = quaternion_to_matrix(0, 0, s, s)
x, yy, z = rotate_vector(m, (1.0, 0.0, 0.0))
chk("rotZ90 maps x->y", abs(x) < 1e-9 and abs(yy - 1.0) < 1e-9)
chk("delta wraps +20", abs(shortest_angle_delta(10, 350) - 20) < 1e-9)
y, p = limit_rate_step(0, 0, 90, 90, 60, 45, 0.05)
chk("rate clamp", abs(y - 3) < 1e-9 and abs(p - 2.25) < 1e-9)
y, p = limit_rate_step(0, 0, 120, -80, 0, 0, 0.05)
chk("rate limit disabled", abs(y - 120) < 1e-9 and abs(p + 80) < 1e-9)
y, _ = limit_rate_step(350, 0, 10, 0, 60, 0, 0.05)
chk("short way around", abs(y - 353) < 1e-9)

fails = [n for n, ok in checks if not ok]
for n, ok in checks:
    print(("PASS " if ok else "FAIL ") + n)
print(f"--- {len(checks) - len(fails)}/{len(checks)} passed ---")
sys.exit(1 if fails else 0)
PY
echo "[OK] 逻辑测试通过"
