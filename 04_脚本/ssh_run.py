#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ssh_run.py — 本地到阿里云服务器的 SSH/SFTP 操作助手（仅本地使用，勿上传）

用法:
  python ssh_run.py "命令"                     # 远程执行命令
  python ssh_run.py --timeout 60 "命令"        # 指定超时（秒）
  python ssh_run.py --put <本地文件> <远程路径>
  python ssh_run.py --putdir <本地目录> <远程目录> [--exclude pat1,pat2]
  python ssh_run.py --get <远程文件> <本地路径>
"""
import argparse
import io
import os
import posixpath
import stat
import sys
import time

import paramiko

HOST = "<你的服务器公网IP>"
PORT = 22
USER = "<登录用户名>"
PASSWORD = "<你的服务器密码或改用密钥>"

# Files larger than this are uploaded in pieces (long transfers get cut by the link).
# Empirically the link drops single transfers above ~512 KB, so keep chunks at/below that.
CHUNK_SIZE = 256 * 1024
# Socket timeout: a stalled transfer raises after this instead of hanging forever.
SOCKET_TIMEOUT = 30.0

_EXCLUDES = ()


def connect(socket_timeout=None):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(HOST, port=PORT, username=USER, password=PASSWORD,
                   timeout=30, banner_timeout=30, auth_timeout=30)
    transport = client.get_transport()
    transport.set_keepalive(30)
    # A socket timeout makes a stalled transfer raise instead of hanging forever,
    # which is essential on this link (long transfers get frozen/reset).
    # Only applied for bulk transfers: command sessions may legitimately stay silent.
    if socket_timeout:
        transport.sock.settimeout(socket_timeout)
    return client


def run(client, command, timeout=600):
    chan = client.get_transport().open_session()
    chan.settimeout(timeout)
    chan.exec_command(command)
    out, err = [], []
    while True:
        if chan.recv_ready():
            out.append(chan.recv(65536))
        if chan.recv_stderr_ready():
            err.append(chan.recv_stderr(65536))
        if chan.exit_status_ready() and not chan.recv_ready() and not chan.recv_stderr_ready():
            break
        time.sleep(0.02)
    code = chan.recv_exit_status()
    return code, b"".join(out).decode("utf-8", "replace"), b"".join(err).decode("utf-8", "replace")


def is_excluded(relpath):
    parts = relpath.replace("\\", "/").split("/")
    for pat in _EXCLUDES:
        if not pat:
            continue
        for part in parts:
            if pat in part:
                return True
    return False


def make_dirs(sftp, remote_dir):
    parts = remote_dir.strip("/").split("/")
    cur = ""
    for p in parts:
        cur += "/" + p
        try:
            sftp.stat(cur)
        except IOError:
            sftp.mkdir(cur)


class Uploader:
    """Resilient recursive uploader: reconnects on failure and skips files whose
    remote size already matches the local size (resume support)."""

    def __init__(self):
        self.client = connect(socket_timeout=SOCKET_TIMEOUT)
        self.sftp = self.client.open_sftp()

    def reconnect(self):
        try:
            self.sftp.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            self.client.close()
        except Exception:  # noqa: BLE001
            pass
        time.sleep(3)
        self.client = connect(socket_timeout=SOCKET_TIMEOUT)
        self.sftp = self.client.open_sftp()

    def remote_size(self, path):
        try:
            return self.sftp.stat(path).st_size
        except IOError:
            return -1

    def put_file(self, local, remote, retries=5):
        """Upload one file. Files larger than CHUNK_SIZE are split into pieces and
        reassembled remotely, because long single transfers are cut by the network."""
        lsize = os.path.getsize(local)
        if self.remote_size(remote) == lsize:
            return "skipped"
        if lsize > CHUNK_SIZE:
            return self.put_file_chunked(local, remote, lsize)
        for attempt in range(retries):
            try:
                make_dirs(self.sftp, posixpath.dirname(remote))
                self.sftp.put(local, remote)
                if self.remote_size(remote) != lsize:
                    raise IOError("size mismatch after upload")
                return "uploaded"
            except Exception as e:  # noqa: BLE001
                print(f"  [RETRY {attempt+1}/{retries}] {remote}: {e}", flush=True)
                self.reconnect()
        raise IOError(f"failed to upload {local}")

    def put_file_chunked(self, local, remote, lsize):
        """Split a large file into CHUNK_SIZE pieces, upload each, then `cat` them."""
        chunk_dir = f"/tmp/_chunks_{os.getpid()}_{os.path.basename(local)}"
        code, _, _ = run(self.client, f"mkdir -p {chunk_dir} && rm -f {chunk_dir}/*")
        n_chunks = (lsize + CHUNK_SIZE - 1) // CHUNK_SIZE
        with open(local, "rb") as fh:
            for idx in range(n_chunks):
                data = fh.read(CHUNK_SIZE)
                remote_chunk = f"{chunk_dir}/part_{idx:05d}"
                if self.remote_size(remote_chunk) == len(data):
                    continue
                ok = False
                for attempt in range(6):
                    try:
                        self.sftp.putfo(io.BytesIO(data), remote_chunk)
                        if self.remote_size(remote_chunk) != len(data):
                            raise IOError("chunk size mismatch")
                        ok = True
                        break
                    except Exception as e:  # noqa: BLE001
                        print(f"  [CHUNK RETRY {attempt+1}/6] {os.path.basename(local)} "
                              f"part {idx+1}/{n_chunks}: {e}", flush=True)
                        self.reconnect()
                if not ok:
                    raise IOError(f"failed chunk {idx} of {local}")
                if (idx + 1) % 5 == 0:
                    print(f"  ... {os.path.basename(local)}: {idx+1}/{n_chunks} chunks", flush=True)
        cmd = (f"mkdir -p $(dirname {remote}) && cat {chunk_dir}/part_* > {remote} && "
               f"rm -rf {chunk_dir} && stat -c %s {remote}")
        code, out, err = run(self.client, cmd, timeout=600)
        if code != 0 or out.strip() != str(lsize):
            raise IOError(f"reassembly failed for {remote}: {out.strip()} != {lsize} ({err.strip()})")
        return "uploaded"

    def put_dir(self, local_dir, remote_dir):
        files = uploaded = skipped = 0
        total = 0
        for root, dirs, names in os.walk(local_dir):
            rel_root = os.path.relpath(root, local_dir)
            rel_root = "" if rel_root == "." else rel_root.replace("\\", "/")
            dirs[:] = [d for d in dirs if not is_excluded((rel_root + "/" + d).strip("/"))]
            rdir = posixpath.join(remote_dir, rel_root) if rel_root else remote_dir
            for name in names:
                rel = (rel_root + "/" + name).strip("/")
                if is_excluded(rel):
                    continue
                lp = os.path.join(root, name)
                rp = posixpath.join(rdir, name)
                try:
                    res = self.put_file(lp, rp)
                except Exception as e:  # noqa: BLE001
                    print(f"  [WARN] giving up on {rel}: {e}", flush=True)
                    continue
                files += 1
                if res == "skipped":
                    skipped += 1
                else:
                    uploaded += 1
                    total += os.path.getsize(lp)
                if files % 25 == 0:
                    print(f"  ... {files} files ({uploaded} uploaded, {skipped} skipped), "
                          f"{total/1024/1024:.1f} MB sent", flush=True)
        return files, uploaded, skipped, total


def main():
    global _EXCLUDES
    parser = argparse.ArgumentParser()
    parser.add_argument("command", nargs="?", default=None)
    parser.add_argument("--timeout", type=int, default=600)
    parser.add_argument("--put", nargs=2, metavar=("LOCAL", "REMOTE"))
    parser.add_argument("--putdir", nargs=2, metavar=("LOCAL", "REMOTE"))
    parser.add_argument("--get", nargs=2, metavar=("REMOTE", "LOCAL"))
    parser.add_argument("--exclude", default="")
    args = parser.parse_args()

    if args.exclude:
        _EXCLUDES = tuple(args.exclude.split(","))

    client = None
    try:
        if args.put:
            up = Uploader()
            client = up.client
            local, remote = args.put
            res = up.put_file(local, remote)
            print(f"[OK] {res} {local} -> {remote} ({os.path.getsize(local)} B)")
        elif args.putdir:
            up = Uploader()
            client = up.client
            local, remote = args.putdir
            files, uploaded, skipped, total = up.put_dir(local, remote)
            print(f"[OK] {local} -> {remote}: {files} files "
                  f"({uploaded} uploaded, {skipped} skipped), {total/1024/1024:.2f} MB sent")
        elif args.get:
            client = connect()
            sftp = client.open_sftp()
            remote, local = args.get
            sftp.get(remote, local)
            print(f"[OK] downloaded {remote} -> {local}")
        elif args.command:
            client = connect()
            code, out, err = run(client, args.command, args.timeout)
            if out:
                print(out, end="")
            if err:
                print(err, file=sys.stderr, end="")
            print(f"\n[exit code: {code}]", file=sys.stderr)
        else:
            parser.print_help()
    finally:
        if client is not None:
            client.close()


if __name__ == "__main__":
    main()
