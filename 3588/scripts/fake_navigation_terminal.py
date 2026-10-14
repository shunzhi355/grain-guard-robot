#!/usr/bin/env python3
"""3588 bench-test fake navigation: approve each stationary goal with Enter."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import socket
import stat
import sys

DEFAULT_SOCKET = "/run/grain-robot/fake_navigation.sock"


def serve(path: Path) -> int:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        if stat.S_ISSOCK(path.stat().st_mode):
            raise RuntimeError(f"套接字已存在，请先确认旧终端是否仍在运行：{path}")
        raise RuntimeError(f"目标路径不是套接字，拒绝覆盖：{path}")
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    inode = None
    try:
        listener.bind(str(path))
        os.chmod(path, 0o600)
        inode = path.stat().st_ino
        listener.listen(1)
        print(f"假导航终端已就绪：{path}", flush=True)
        print("仅供架空履带/静止台架联调；不发送任何底盘速度。", flush=True)
        while True:
            conn, _ = listener.accept()
            with conn:
                conn.settimeout(5.0)
                with conn.makefile("rb") as stream:
                    raw = stream.readline(4097)
                if len(raw) > 4096 or not raw.endswith(b"\n"):
                    continue
                try:
                    request = json.loads(raw)
                except (UnicodeError, json.JSONDecodeError):
                    continue
                if not isinstance(request, dict) or request.get("type") != "fake_nav_goal":
                    continue
                goal_id = request.get("goal_id")
                if not isinstance(goal_id, str) or not goal_id:
                    continue
                print(f"\n收到假导航目标：X={request.get('x_m')}  Y={request.get('y_m')}", flush=True)
                print("确认底盘静止、机构周围无人、取样管位置安全。", flush=True)
                answer = input("按 Enter 模拟到位；输入 n 拒绝：").strip().lower()
                approved = answer == ""
                reply = {"goal_id": goal_id, "approved": approved}
                try:
                    conn.sendall(json.dumps(reply).encode("utf-8") + b"\n")
                except OSError:
                    print("UI 已取消或断开，本次确认无效。", flush=True)
                else:
                    print("已发送人工确认，UI 将复核底盘安全状态。" if approved
                          else "已拒绝到位。", flush=True)
    except KeyboardInterrupt:
        print("\n假导航终端已退出。", flush=True)
        return 0
    finally:
        listener.close()
        if (inode is not None and path.exists() and stat.S_ISSOCK(path.stat().st_mode)
                and path.stat().st_ino == inode):
            path.unlink()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", default=os.getenv("GRAIN_FAKE_NAV_SOCKET", DEFAULT_SOCKET))
    args = parser.parse_args(argv)
    try:
        return serve(Path(args.socket))
    except (OSError, RuntimeError, EOFError) as exc:
        print(f"假导航终端失败：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
