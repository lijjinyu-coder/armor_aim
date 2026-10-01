#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
resume_and_finish.py —— 服务器恢复后的一键收尾（本地运行）

流程：
  1) 等待服务器 SSH 恢复（轮询，默认最多 60 分钟）
  2) 上传本轮新增/修改的自瞄侧文件（aim_output / prm_launch / sim_tests）
  3) 在服务器上重新编译自瞄工作空间（白名单 colcon）
  4) 上传远程工具脚本（tf_rpy / aim_dir / save_frame / list_armors / run_all）
  5) 运行 run_all.sh：自动选目标 → 检测阈值自标定 → 最小闭环 + 录制 → 目标丢失测试
  6) 回传演示视频与截图到 deploy/artifacts/

用法:
  python resume_and_finish.py [--wait-minutes 60] [--record-seconds 40]
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ssh_run  # noqa: E402

REPO = r"C:\rm\auto-aiming-main\auto-aiming-main"
TOOLS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "remote")
ART = r"C:\rm\deploy\artifacts"

# 需要同步到服务器的自瞄侧目录（本轮改动）
SYNC = [
    (os.path.join(REPO, r"src\prm_control\aim_output"), "/opt/ws/auto-aiming/src/prm_control/aim_output",
     "build,install,log,__pycache__"),
    (os.path.join(REPO, r"src\prm_launch"), "/opt/ws/auto-aiming/src/prm_launch", "build,install,log,__pycache__"),
    (os.path.join(REPO, r"src\sim_tests"), "/opt/ws/auto-aiming/src/sim_tests", "build,install,log,__pycache__"),
    (os.path.join(REPO, r"src\prm_vision"), "/opt/ws/auto-aiming/src/prm_vision",
     "blue,easy,far_back,build,install,log,__pycache__"),
]

REMOTE_TOOLS = ["tf_rpy.py", "aim_dir.py", "save_frame.py", "list_armors.py", "run_all.sh"]


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def wait_for_server(minutes):
    deadline = time.time() + minutes * 60
    attempt = 0
    while time.time() < deadline:
        attempt += 1
        try:
            client = ssh_run.connect()
            code, out, _ = ssh_run.run(client, "echo ok; uptime | cut -d, -f1", timeout=30)
            client.close()
            if code == 0 and "ok" in out:
                log(f"服务器已恢复（第 {attempt} 次尝试）: {out.strip().splitlines()[-1]}")
                return True
        except Exception as e:  # noqa: BLE001
            log(f"第 {attempt} 次尝试失败: {type(e).__name__}")
        time.sleep(30)
    return False


def upload_all():
    for local, remote, ex in SYNC:
        log(f"上传 {local} -> {remote}")
        up = ssh_run.Uploader()
        try:
            files, uploaded, skipped, total = up.put_dir(local, remote)
            log(f"  {files} 文件（{uploaded} 上传 / {skipped} 跳过），{total/1024/1024:.2f} MB")
        finally:
            up.client.close()
    up = ssh_run.Uploader()
    try:
        for name in REMOTE_TOOLS:
            lp = os.path.join(TOOLS, name)
            if os.path.exists(lp):
                up.put_file(lp, f"/tmp/{name}")
        log("远程工具已上传")
    finally:
        up.client.close()


def build_and_run(record_seconds):
    client = ssh_run.connect()
    try:
        log("重新编译自瞄工作空间（白名单）")
        code, out, err = ssh_run.run(client, "bash /tmp/build_aim_ws.sh 2>&1 | tail -6", timeout=1800)
        log(out.strip()[-500:] or err.strip()[-500:])
        if code != 0:
            log("!! 编译失败，请检查日志")
            return False

        log(f"运行一键闭环与场景测试（录制 {record_seconds} 秒）")
        cmd = f"bash /tmp/run_all.sh {record_seconds} 2>&1 | tail -80"
        code, out, err = ssh_run.run(client, cmd, timeout=3600)
        print(out)
        if err.strip():
            print(err[-1000:], file=sys.stderr)
        return code == 0
    finally:
        client.close()


def fetch_artifacts():
    os.makedirs(ART, exist_ok=True)
    up = ssh_run.Uploader()
    try:
        code, out, _ = ssh_run.run(up.client, "ls /opt/ws/recordings/ 2>/dev/null", timeout=60)
        names = [n for n in out.split() if n]
        log(f"服务器产物 {len(names)} 个: {names}")
        for n in names:
            remote = f"/opt/ws/recordings/{n}"
            local = os.path.join(ART, n)
            try:
                up.sftp.get(remote, local)
                log(f"已下载 {n} ({os.path.getsize(local)} B)")
            except Exception as e:  # noqa: BLE001
                log(f"下载失败 {n}: {e}")
    finally:
        up.client.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wait-minutes", type=int, default=60)
    ap.add_argument("--record-seconds", type=int, default=40)
    ap.add_argument("--skip-wait", action="store_true")
    args = ap.parse_args()

    if not args.skip_wait and not wait_for_server(args.wait_minutes):
        log("等待超时，服务器仍不可达；稍后重试")
        return 2

    upload_all()
    ok = build_and_run(args.record_seconds)
    fetch_artifacts()
    log("完成" if ok else "闭环脚本返回非零，请检查上面的输出")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
