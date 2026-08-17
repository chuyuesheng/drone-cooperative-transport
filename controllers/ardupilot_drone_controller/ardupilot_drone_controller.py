#!/usr/bin/env python3
"""DJI Mavic 2 PRO 的 ArduPilot-Webots 控制器入口。

本模块通过 UDP 将 Webots 传感器数据发送给 ArduPilot SITL，并把 SITL
返回的电机指令施加到 Mavic 2 PRO 模型的四个螺旋桨上。

电机到 ArduPilot SERVO 的映射（四旋翼 X 架型）：
  "front right propeller" -> m1 (SERVO1)
  "rear left propeller"   -> m2 (SERVO2)
  "front left propeller"  -> m3 (SERVO3)
  "rear right propeller"  -> m4 (SERVO4)

Mavic2Pro.proto 的四个螺旋桨均采用轴向正推力约定，因此可以直接应用
ArduPilot 输出的归一化电机指令。

本程序由 Webots 世界 worlds/mavic_2_pro.wbt 内的机器人控制器启动。
"""

import re
import os
import subprocess
import sys

from webots_vehicle import WebotsArduVehicle


def detect_wsl_ip():
    """控制器运行在 Windows 时，返回 WSL 虚拟机的 IPv4 地址。"""
    # 用户显式提供的 WSL_IP 优先，便于网络地址变化时覆盖自动检测。
    env_ip = os.environ.get("WSL_IP")
    if env_ip:
        return env_ip.strip()
    if not sys.platform.startswith("win"):
        return None
    # Windows 侧调用 wsl.exe 查询当前 WSL 网卡地址。
    try:
        output = subprocess.check_output(
            ["wsl.exe", "hostname", "-I"],
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2,
        )
    except Exception:
        return None
    for token in output.split():
        if re.match(r'^\d+\.\d+\.\d+\.\d+$', token):
            return token
    return None


def get_instance():
    """从 Webots 机器人名称或命令行参数获取无人机实例编号。"""
    # 方法 1：读取机器人名称末尾的数字，例如 drone_0 -> 0。
    name = os.environ.get("WEBOTS_ROBOT_NAME", "")
    m = re.search(r'(\d+)$', name)
    if m:
        return int(m.group(1))
    # 方法 2：读取独立整数 controllerArg，例如 "0" 表示实例 0。
    # 只接受纯整数，避免把 --motor-cap 或 --sitl-address 的值误认为实例号。
    for a in sys.argv[1:]:
        if re.fullmatch(r'\d+', a):
            return int(a)
    return 0


def main():
    """解析参数，创建车辆桥接对象并等待 Webots 退出。"""
    instance = get_instance()
    # 限制最大角速度，给吊载状态下的姿态差动控制保留余量。
    motor_cap = 220.0                      # 单位：rad/s
    sitl_address = detect_wsl_ip() or "127.0.0.1"
    camera_name = None                     # 悬停验证不需要摄像头，保持轻量

    # 解析 Webots controllerArgs 中的可选覆盖参数。
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        arg = args[i]
        if arg in ("--motor-cap", "-m") and i + 1 < len(args):
            motor_cap = float(args[i + 1])
            i += 2
        elif arg in ("--sitl-address", "-s") and i + 1 < len(args):
            sitl_address = args[i + 1]
            i += 2
        elif arg == "--camera":
            camera_name = "camera"
            i += 1
        else:
            i += 1

    motors = ["front right propeller", "rear left propeller",
              "front left propeller", "rear right propeller"]

    print(f"[Drone I{instance}] ========================================")
    print(f"[Drone I{instance}] Motors:      {motors}")
    print(f"[Drone I{instance}] UDP port:    {9002 + 10 * instance}")
    print(f"[Drone I{instance}] Motor cap:   {motor_cap} rad/s")
    print(f"[Drone I{instance}] SITL addr:   {sitl_address}")
    print(f"[Drone I{instance}] Camera:      {camera_name}")
    print(f"[Drone I{instance}] ========================================")

    # 实例号同时决定 UDP 端口：9002 + 10 * instance。
    vehicle = WebotsArduVehicle(
        motor_names=motors,
        accel_name="accelerometer",
        imu_name="inertial unit",
        gyro_name="gyro",
        gps_name="gps",
        camera_name=camera_name,
        instance=instance,
        motor_velocity_cap=motor_cap,
        reversed_motors=None,
        bidirectional_motors=False,
        uses_propellers=True,
        sitl_address=sitl_address,
        launch_lock_threshold=0.40,
        level_attitude_lock=True,
    )

    name = vehicle.robot.getName()
    print(f"[Drone I{instance}] Robot name:  {name}")
    print(f"[Drone I{instance}] Bridge running, waiting for SITL...")

    # SITL 通信在后台线程运行；主线程仅维持控制器进程生命周期。
    while vehicle.webots_connected():
        import time
        time.sleep(1)

    print(f"[Drone I{instance}] Webots disconnected, exiting.")


if __name__ == "__main__":
    main()
