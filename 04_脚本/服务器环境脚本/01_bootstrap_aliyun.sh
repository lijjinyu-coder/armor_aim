#!/usr/bin/env bash
# ============================================================
# 01_bootstrap_aliyun.sh
# 阿里云 GPU 实例（Ubuntu 22.04）一键环境引导脚本
# 覆盖: 基础工具 / Rust / ROS2 Humble / 仿真器系统依赖 / VNC 远程桌面
# 用法: sudo bash 01_bootstrap_aliyun.sh   (root 或 sudo 执行)
# ============================================================
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "[ERR] 请用 root 或 sudo 运行"
    exit 1
fi

# 目标用户（若你以非 root 登录部署，改成你的用户名）
TARGET_USER="${1:-root}"
TARGET_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"
[ -n "$TARGET_HOME" ] || TARGET_HOME="/root"

log()  { echo -e "\033[1;32m[OK]\033[0m $*"; }
warn() { echo -e "\033[1;33m[..]\033[0m $*"; }

# ------------------------------------------------------------
# 1. 基础包 + 编译/渲染/远程桌面依赖
# ------------------------------------------------------------
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y \
    curl wget git rsync htop tmux \
    build-essential cmake pkg-config ninja-build \
    python3-pip python3-venv \
    llvm-dev libclang-dev clang \
    libssl-dev \
    libvulkan1 vulkan-tools mesa-utils \
    libasound2-dev libudev-dev libxkbcommon-dev libxkbcommon-x11-0 \
    libx11-dev libxcb1-dev libxcb-render-util0-dev libxcb-xkb-dev \
    libxrandr-dev libxi-dev libxfixes-dev libxcursor-dev \
    xvfb x11vnc x11-apps xauth xdotool \
    novnc websockify \
    ffmpeg
log "基础依赖安装完成"

# ------------------------------------------------------------
# 2. Rust 工具链（rustup, stable）
# ------------------------------------------------------------
export PATH="/root/.cargo/bin:$TARGET_HOME/.cargo/bin:$PATH"
if ! command -v cargo >/dev/null 2>&1; then
    # 直连 static.rust-lang.org 在国内链路上会卡死，改用 rsproxy 镜像
    export RUSTUP_DIST_SERVER="https://rsproxy.cn"
    export RUSTUP_UPDATE_ROOT="https://rsproxy.cn/rustup"
    su - "$TARGET_USER" -c 'curl -sSf --max-time 30 https://rsproxy.cn/rustup/dist/x86_64-unknown-linux-gnu/rustup-init -o /tmp/rustup-init && chmod +x /tmp/rustup-init && /tmp/rustup-init -y --default-toolchain stable --profile minimal'
    log "rustup 安装完成"
else
    log "cargo 已存在: $(cargo --version)"
fi

# ------------------------------------------------------------
# 3. ROS2 Humble（阿里云镜像源，国内速度快）
# ------------------------------------------------------------
if [ ! -d /opt/ros/humble ]; then
    apt-get install -y software-properties-common gnupg2
    # 注意：raw.githubusercontent.com 在国内链路上不可达，改用清华镜像取 key
    ok_key=0
    for key_url in \
        "https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key" \
        "https://mirrors.ustc.edu.cn/rosdistro/ros.key" \
        "http://packages.ros.org/ros.key" ; do
        if curl -sSf --max-time 20 "$key_url" -o /usr/share/keyrings/ros-archive-keyring.gpg; then
            ok_key=1
            log "ROS2 key 来自 $key_url"
            break
        fi
    done
    [ "$ok_key" = "1" ] || { echo "[ERR] 无法获取 ROS2 GPG key"; exit 1; }

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.aliyun.com/ros2/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros2.list
    apt-get update
    # 若镜像不可用，把上面地址换回官方: http://packages.ros.org/ros2/ubuntu
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ros-humble-ros-base \
        ros-humble-cv-bridge \
        ros-humble-image-transport \
        ros-humble-image-transport-plugins \
        ros-humble-camera-info-manager \
        ros-humble-nav-msgs \
        ros-humble-tf-transformations \
        ros-humble-ament-cmake-gtest \
        ros-humble-rqt \
        ros-humble-rqt-graph \
        ros-humble-rqt-image-view \
        ros-humble-rqt-tf-tree \
        libopencv-dev libeigen3-dev python3-opencv \
        python3-colcon-common-extensions \
        python3-rosdep
    rosdep init || true
    log "ROS2 Humble 安装完成"
else
    log "ROS2 Humble 已存在: $(ls /opt/ros/humble/setup.bash)"
fi

# ------------------------------------------------------------
# 4. 验证 GPU 驱动
# ------------------------------------------------------------
if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi || warn "nvidia-smi 存在但执行失败，检查驱动"
else
    warn "未找到 nvidia-smi：确认购买时勾选了『预装 NVIDIA 驱动』"
    warn "若没有 GPU 驱动，仿真器将回退软件渲染（帧率很低）"
fi

# ------------------------------------------------------------
# 5. 工作目录
# ------------------------------------------------------------
mkdir -p /opt/ws
chown "$TARGET_USER":"$TARGET_USER" /opt/ws
log "工作目录 /opt/ws 就绪"

# ------------------------------------------------------------
# 6. VNC 远程桌面（Xvfb :0 + x11vnc + noVNC 网页版）
#    启动: systemctl start vncserver  访问: http://<公网IP>:6080/vnc.html
# ------------------------------------------------------------
cat > /etc/systemd/system/xvfb.service <<'EOF'
[Unit]
Description=Xvfb virtual display :0
After=network.target

[Service]
ExecStart=/usr/bin/Xvfb :0 -screen 0 1920x1080x24
Restart=always

[Install]
WantedBy=multi-user.target
EOF

cat > /etc/systemd/system/x11vnc.service <<'EOF'
[Unit]
Description=x11vnc server on :0
After=xvfb.service
Requires=xvfb.service

[Service]
ExecStart=/usr/bin/x11vnc -display :0 -forever -shared -nopw -listen localhost -rfbport 5900 -xkb
Restart=always

[Install]
WantedBy=multi-user.target
EOF

cat > /etc/systemd/system/novnc.service <<'EOF'
[Unit]
Description=noVNC web client on :6080
After=x11vnc.service
Requires=x11vnc.service

[Service]
ExecStart=/usr/bin/websockify --web=/usr/share/novnc 6080 localhost:5900
Restart=always

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable xvfb.service x11vnc.service novnc.service
systemctl start xvfb.service x11vnc.service novnc.service
log "VNC 服务已启动（noVNC 端口 6080，仅监听公开网卡，安全组别放行 6080，请走 SSH 隧道）"

echo
log "引导完成。下一步:"
echo "  1) 上传两个项目到 /opt/ws"
echo "  2) bash deploy/scripts/02_build_rm_interfaces.sh  (colcon 构建 rm_interfaces)"
echo "  3) bash deploy/scripts/03_build_simulator.sh      (cargo 构建仿真器 ros2 feature)"
echo "  4) 本地执行:  ssh -L 6080:127.0.0.1:6080 <用户>@<公网IP>  然后浏览器打开 http://127.0.0.1:6080/vnc.html"
