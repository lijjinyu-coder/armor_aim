#!/usr/bin/env bash
# ============================================================
# 04_git_init.sh
# 在服务器上为两个项目初始化版本控制（任务书 §八）：
#   - auto-aiming: feature/simulator-integration 分支 + 后续 sim-closed-loop-v1 标签
#   - 仿真器: 记录"原始版本"基线提交，后续必要修改单独提交（作为修改记录）
# 用法: bash 04_git_init.sh <GitHub仓库URL(可选)>
# ============================================================
set -euo pipefail

REMOTE_URL="${1:-}"

# ---------- auto-aiming ----------
AIM=/opt/ws/auto-aiming
cd "$AIM"
if [ ! -d .git ]; then
    git init -b master
    git config user.name "sim-integration"
    git config user.email "sim@localhost"
    git add -A
    git commit -m "chore: baseline of auto-aiming before simulator integration"
fi
# 建立任务要求的特性分支
git checkout -B feature/simulator-integration
if [ -n "$REMOTE_URL" ]; then
    git remote add origin "$REMOTE_URL" 2>/dev/null || git remote set-url origin "$REMOTE_URL"
    echo "[OK] 远端已设置: $REMOTE_URL (推送前请确认账号/密钥)"
fi
echo "[OK] auto-aiming: 当前分支 $(git branch --show-current)"

# ---------- 仿真器 ----------
SIM=/opt/ws/bevy_robomaster_simulator
cd "$SIM"
if [ ! -d .git ]; then
    git init -b master
    git config user.name "sim-integration"
    git config user.email "sim@localhost"
    git add -A
    git commit -m "chore: upstream baseline of bevy_robomaster_simulator"
fi
echo "[OK] 仿真器: 基线提交已建立 (后续部署修正以独立提交记录)"

echo
echo "下一步（完成闭环后）:"
echo "  cd $AIM && git tag -a sim-closed-loop-v1 -m 'sim closed loop v1'"
echo "  git push -u origin feature/simulator-integration --tags"
