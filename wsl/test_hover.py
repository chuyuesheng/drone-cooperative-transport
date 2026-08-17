#!/usr/bin/env python3
"""通过 MAVLink 执行单机 GUIDED 起飞，并统计定高悬停性能。"""

import argparse
import math
import statistics
import time

from pymavlink import mavutil


def connect_with_retry(master, timeout=15.0):
    """在超时时间内反复尝试连接可能仍在启动的 MAVLink 端点。"""
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        try:
            return mavutil.mavlink_connection(master, autoreconnect=True)
        except (ConnectionError, OSError) as error:
            # SITL 和 MAVProxy 启动需要时间，短暂失败时等待后重试。
            last_error = error
            time.sleep(0.5)
    raise SystemExit(f"ERROR: cannot connect to {master}: {last_error}")


def wait_command_ack(connection, command, timeout=5.0):
    """等待指定 MAVLink 命令的确认消息，超时则返回 None。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = connection.recv_match(blocking=True, timeout=0.5)
        if message and message.get_type() == "COMMAND_ACK" and message.command == command:
            return message.result
    return None


def main():
    # 命令行参数可覆盖连接地址、目标高度和测试持续时间。
    parser = argparse.ArgumentParser()
    parser.add_argument("--master", default="tcp:127.0.0.1:5762")
    parser.add_argument("--altitude", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--land-after", action="store_true")
    parser.add_argument("--max-alt-error", type=float, default=0.5)
    parser.add_argument("--max-alt-stddev", type=float, default=0.1)
    parser.add_argument("--max-distance", type=float, default=2.0)
    parser.add_argument("--max-tilt", type=float, default=10.0)
    args = parser.parse_args()

    # 建立连接并等待飞控心跳；心跳到达后才能可靠地获取系统编号。
    vehicle = connect_with_retry(args.master)
    if vehicle.wait_heartbeat(timeout=15) is None:
        raise SystemExit("ERROR: no MAVLink heartbeat")
    target_system = vehicle.target_system
    target_component = vehicle.target_component or 1
    print(f"CONNECTED system={target_system} component={target_component}", flush=True)

    # 请求飞控以 20 Hz 发送全部常用遥测数据，供后面的统计与诊断使用。
    vehicle.mav.request_data_stream_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        20,
        1,
    )

    # GUIDED 模式允许脚本通过 MAVLink 下发起飞目标。
    guided = vehicle.mode_mapping().get("GUIDED")
    if guided is None:
        raise SystemExit("ERROR: GUIDED mode is unavailable")
    vehicle.mav.set_mode_send(
        target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        guided,
    )
    time.sleep(1)

    # 强制解锁仅用于无人的软件在环仿真，真实飞行不可使用该 magic 值绕过检查。
    vehicle.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        21196,  # ArduPilot 强制解锁 magic 值，仅限仿真。
        0,
        0,
        0,
        0,
        0,
    )
    arm_result = wait_command_ack(vehicle, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    print(f"ARM_ACK result={arm_result}", flush=True)
    time.sleep(1)

    # 向 ArduPilot 发送相对起飞高度，水平位置由 GUIDED 控制器保持。
    vehicle.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        args.altitude,
    )
    takeoff_result = wait_command_ack(vehicle, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
    print(f"TAKEOFF_ACK result={takeoff_result} target={args.altitude:.2f}m", flush=True)

    # 使用单调时钟计时，避免系统时间调整影响测试时长和采样窗口。
    started = time.monotonic()
    next_report = started
    latest_altitude = None
    latest_x = None
    latest_y = None
    latest_horizontal_speed = None
    latest_roll = 0.0
    latest_pitch = 0.0
    nav_roll = 0.0
    nav_pitch = 0.0
    # 只统计测试后 40% 的数据，以排除起飞爬升阶段的瞬态响应。
    steady_samples = []
    steady_horizontal_speeds = []
    steady_distances = []
    steady_tilts = []
    while time.monotonic() - started < args.duration:
        message = vehicle.recv_match(blocking=True, timeout=0.5)
        now = time.monotonic()
        if message:
            message_type = message.get_type()
            if message_type == "GLOBAL_POSITION_INT":
                # relative_alt 的单位是毫米，转换为米。
                latest_altitude = message.relative_alt / 1000.0
                if now - started >= args.duration * 0.6:
                    steady_samples.append(latest_altitude)
            elif message_type == "LOCAL_POSITION_NED":
                # NED 坐标中的 x/y 为水平位置，vx/vy 为水平速度。
                latest_x = message.x
                latest_y = message.y
                latest_horizontal_speed = math.hypot(message.vx, message.vy)
                if now - started >= args.duration * 0.6:
                    steady_horizontal_speeds.append(latest_horizontal_speed)
                    steady_distances.append(math.hypot(latest_x, latest_y))
            elif message_type == "ATTITUDE":
                # MAVLink 姿态角为弧度；日志中转换成角度便于观察。
                latest_roll = math.degrees(message.roll)
                latest_pitch = math.degrees(message.pitch)
                if now - started >= args.duration * 0.6:
                    steady_tilts.append(max(abs(latest_roll), abs(latest_pitch)))
            elif message_type == "NAV_CONTROLLER_OUTPUT":
                nav_roll = message.nav_roll
                nav_pitch = message.nav_pitch
            elif message_type == "STATUSTEXT":
                print(f"STATUS {message.text}", flush=True)
        # 每秒输出一次最新状态，便于定位失稳发生的具体时刻。
        if now >= next_report:
            altitude_text = "n/a" if latest_altitude is None else f"{latest_altitude:.3f}m"
            position_text = (
                "n/a" if latest_x is None
                else f"({latest_x:.2f},{latest_y:.2f})m vxy={latest_horizontal_speed:.2f}m/s"
            )
            print(
                f"SAMPLE t={now-started:.1f}s mode={vehicle.flightmode} "
                f"armed={vehicle.motors_armed()} alt={altitude_text} "
                f"pos={position_text} roll={latest_roll:.2f}deg pitch={latest_pitch:.2f}deg "
                f"nav_roll={nav_roll:.2f}deg nav_pitch={nav_pitch:.2f}deg",
                flush=True,
            )
            next_report += 1.0

    # 汇总稳态高度误差、波动、水平速度和相对起点的最大距离。
    if not steady_samples:
        raise SystemExit("ERROR: no steady-state altitude samples received")
    mean_altitude = statistics.fmean(steady_samples)
    altitude_stddev = statistics.pstdev(steady_samples)
    max_error = max(abs(value - args.altitude) for value in steady_samples)
    mean_horizontal_speed = (
        statistics.fmean(steady_horizontal_speeds) if steady_horizontal_speeds else float("nan")
    )
    max_distance = max(steady_distances) if steady_distances else float("nan")
    max_tilt = max(steady_tilts) if steady_tilts else float("nan")
    print(
        f"RESULT samples={len(steady_samples)} mean_alt={mean_altitude:.3f}m "
        f"stddev={altitude_stddev:.3f}m max_error={max_error:.3f}m "
        f"mean_vxy={mean_horizontal_speed:.3f}m/s max_distance={max_distance:.3f}m "
        f"max_tilt={max_tilt:.3f}deg",
        flush=True,
    )

    failures = []
    if max_error > args.max_alt_error:
        failures.append(f"max altitude error {max_error:.3f}m > {args.max_alt_error:.3f}m")
    if altitude_stddev > args.max_alt_stddev:
        failures.append(f"altitude stddev {altitude_stddev:.3f}m > {args.max_alt_stddev:.3f}m")
    if not math.isfinite(max_distance) or max_distance > args.max_distance:
        failures.append(f"max distance {max_distance:.3f}m > {args.max_distance:.3f}m")
    if not math.isfinite(max_tilt) or max_tilt > args.max_tilt:
        failures.append(f"max tilt {max_tilt:.3f}deg > {args.max_tilt:.3f}deg")

    # 默认测试结束后保持当前模式；显式传入 --land-after 才切换 LAND。
    if args.land_after:
        land = vehicle.mode_mapping()["LAND"]
        vehicle.mav.set_mode_send(
            target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            land,
        )
        print("LAND commanded", flush=True)

    if failures:
        raise SystemExit("FAIL: " + "; ".join(failures))
    print("PASS: hover acceptance criteria satisfied", flush=True)


if __name__ == "__main__":
    main()
