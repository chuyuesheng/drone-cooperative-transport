#!/usr/bin/env python3
"""在 WSL localhost 与 Windows Webots 之间双向转发 ArduPilot UDP 数据。"""

import argparse
import math
import socket
import struct
import threading
import time


def motor_summary(data, actuator_count=4):
    """解析 SITL 电机数据包，并返回指定数量执行器的简短文本。"""
    # SITL 固定发送 16 个 float；数据不足时不尝试解包。
    if len(data) < 64:
        return "n/a"
    values = struct.unpack("f" * 16, data[:64])
    return ",".join(f"{v:.3f}" for v in values[:actuator_count])


def fdm_summary(data):
    """解析 Webots FDM 数据包，返回姿态、速度和位置摘要。"""
    # FDM 包由 16 个 double 组成，总长度应至少为 128 字节。
    if len(data) < 128:
        return "n/a"
    values = struct.unpack("d" * 16, data[:128])
    timestamp = values[0]
    angular_velocity = values[1:4]
    acceleration = values[4:7]
    attitude = values[7:10]
    velocity = values[10:13]
    position = values[13:16]
    return (
        f"t={timestamp:.3f} "
        f"rpy=({attitude[0]:.3f},{attitude[1]:.3f},{attitude[2]:.3f}) "
        f"gyro=({angular_velocity[0]:.2f},{angular_velocity[1]:.2f},{angular_velocity[2]:.2f}) "
        f"accel=({acceleration[0]:.2f},{acceleration[1]:.2f},{acceleration[2]:.2f}) "
        f"vel=({velocity[0]:.2f},{velocity[1]:.2f},{velocity[2]:.2f}) "
        f"pos=({position[0]:.2f},{position[1]:.2f},{position[2]:.2f})"
    )


def forward_loop(
    name, bind_addr, bind_port, target_addr, target_port, describe,
    require_monotonic_timestamp=False,
):
    """持续接收一个方向的 UDP 数据，可过滤乱序 FDM 包后再转发。"""
    # 接收和发送使用不同套接字，避免转发数据被本循环再次接收。
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    recv_sock.bind((bind_addr, bind_port))
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (target_addr, target_port)
    print(f"[Bridge:{name}] {bind_addr}:{bind_port} -> {target_addr}:{target_port}", flush=True)

    count = 0
    dropped = 0
    last_timestamp = None
    reset_guard_until = 0.0
    last_report = time.monotonic()
    while True:
        data, peer = recv_sock.recvfrom(4096)
        count += 1

        # ArduPilot Webots 后端把包首时间戳当作仿真时钟。旧包会使时钟倒退，
        # 并破坏惯性/位置估计，因此 FDM 方向必须保证时间戳单调递增。
        if require_monotonic_timestamp:
            if len(data) < 8:
                dropped += 1
                continue
            timestamp = struct.unpack("d", data[:8])[0]
            now = time.monotonic()
            # Webots Reset 会主动把时间归零。检测到足够大的回跳时，将其视为
            # 新一轮仿真，而不是等待时间重新追上上一轮的结束值。
            if (
                math.isfinite(timestamp)
                and last_timestamp is not None
                and last_timestamp >= 30.0
                and timestamp <= 5.0
                and timestamp < last_timestamp - 20.0
            ):
                print(
                    f"[Bridge:{name}] Webots time reset detected "
                    f"last={last_timestamp:.3f} new={timestamp:.3f}; accepting new epoch",
                    flush=True,
                )
                last_timestamp = None
                reset_guard_until = now + 3.0

            # 新一轮开始后的短暂保护窗口内，丢弃上一轮延迟到达的大时间戳包。
            delayed_old_epoch = (
                last_timestamp is not None
                and now < reset_guard_until
                and timestamp > last_timestamp + 5.0
            )
            if not math.isfinite(timestamp) or delayed_old_epoch or (
                last_timestamp is not None and timestamp <= last_timestamp
            ):
                dropped += 1
                if dropped <= 5 or dropped % 100 == 0:
                    print(
                        f"[Bridge:{name}] dropped stale/invalid FDM packet "
                        f"t={timestamp!r} last={last_timestamp!r} dropped={dropped}",
                        flush=True,
                    )
                continue
            last_timestamp = timestamp

        # 数据通过过滤后，原样转发到目标端点。
        send_sock.sendto(data, target)

        now = time.monotonic()
        if count <= 5 or now - last_report >= 1.0:
            print(
                f"[Bridge:{name}] packets={count} from={peer[0]}:{peer[1]} "
                f"bytes={len(data)} dropped={dropped} {describe(data)}",
                flush=True,
            )
            last_report = now


def main():
    """解析网络端点，并启动电机与 FDM 两个方向的转发线程。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, required=True, help="Webots motor UDP port, e.g. 9002")
    parser.add_argument("--windows-ip", required=True)
    parser.add_argument("--wsl-ip", required=True)
    parser.add_argument(
        "--actuator-count",
        type=int,
        default=4,
        choices=range(1, 17),
        metavar="1..16",
        help="Number of actuator values shown in bridge summaries (default: 4)",
    )
    args = parser.parse_args()

    # 约定：基础端口接收 SITL 电机输出，基础端口 + 1 传输 Webots FDM。
    fdm_port = args.port + 1
    threads = [
        threading.Thread(
            target=forward_loop,
            args=(
                "motors",
                "127.0.0.1",
                args.port,
                args.windows_ip,
                args.port,
                lambda data: motor_summary(data, args.actuator_count),
            ),
            daemon=True,
        ),
        threading.Thread(
            target=forward_loop,
            args=("fdm", args.wsl_ip, fdm_port, "127.0.0.1", fdm_port, fdm_summary, True),
            daemon=True,
        ),
    ]

    # 两个线程均为守护线程；主线程常驻以维持进程。
    for thread in threads:
        thread.start()
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    main()
