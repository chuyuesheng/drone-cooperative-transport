'''
实现 ArduPilot SITL 与 Webots 之间的车辆桥接类。

数据流：Webots 传感器 -> FDM UDP 数据包 -> SITL；
        SITL 电机输出 -> UDP 数据包 -> Webots 螺旋桨。

AP_FLAKE8_CLEAN
'''

# 标准库、线程和类型标注。
import os
import select
import socket
import struct
import sys
import time

from threading import Thread
from typing import List
from typing import Union

import numpy as np

# 设置 Webots Python 控制器路径，使本文件也能作为外部控制器运行和调试。
# https://cyberbotics.com/doc/guide/running-extern-robot-controllers
if sys.platform.startswith("win"):
    WEBOTS_HOME = "C:\\Program Files\\Webots"
elif sys.platform.startswith("darwin"):
    WEBOTS_HOME = "/Applications/Webots.app"
elif sys.platform.startswith("linux"):
    WEBOTS_HOME = "/usr/local/webots"
else:
    raise Exception("Unsupported OS")

if os.environ.get("WEBOTS_HOME") is None:
    os.environ["WEBOTS_HOME"] = WEBOTS_HOME
else:
    WEBOTS_HOME = os.environ.get("WEBOTS_HOME")

os.environ["PYTHONIOENCODING"] = "UTF-8"
sys.path.append(f"{WEBOTS_HOME}/lib/controller/python")

from controller import Camera  # noqa: E401, E402
from controller import RangeFinder  # noqa: E401, E402
from controller import Robot  # noqa: E401, E402
from controller import Supervisor  # noqa: E401, E402


class WebotsArduVehicle():
    """由 ArduPilot SITL 控制的 Webots 车辆及其通信桥。"""

    # SITL 电机包包含 16 个 float；FDM 包包含时间戳及五组三轴 double。
    controls_struct_format = 'f'*16
    controls_struct_size = struct.calcsize(controls_struct_format)
    fdm_struct_format = 'd'*(1+3+3+3+3+3)
    fdm_struct_size = struct.calcsize(fdm_struct_format)

    def __init__(self,
                 motor_names: List[str],
                 accel_name: str = "accelerometer",
                 imu_name: str = "inertial unit",
                 gyro_name: str = "gyro",
                 gps_name: str = "gps",
                 camera_name: str = None,
                 camera_fps: int = 10,
                 camera_stream_port: int = None,
                 rangefinder_name: str = None,
                 rangefinder_fps: int = 10,
                 rangefinder_stream_port: int = None,
                 instance: int = 0,
                 motor_velocity_cap: float = float('inf'),
                 reversed_motors: List[int] = None,
                 bidirectional_motors: bool = False,
                 uses_propellers: bool = True,
                 sitl_address: str = "127.0.0.1",
                 launch_lock_threshold: float = None,
                 level_attitude_lock: bool = False):
        """初始化 Webots 设备、电机、可选图像流以及 SITL 通信线程。

        Args:
            motor_names (List[str]): Motor names in ArduPilot numerical order (first motor is SERVO1 etc).
            accel_name (str, optional): Webots accelerometer name. Defaults to "accelerometer".
            imu_name (str, optional): Webots imu name. Defaults to "inertial unit".
            gyro_name (str, optional): Webots gyro name. Defaults to "gyro".
            gps_name (str, optional): Webots GPS name. Defaults to "gps".
            camera_name (str, optional): Webots camera name. Defaults to None.
            camera_fps (int, optional): Camera FPS. Lower FPS runs better in sim. Defaults to 10.
            camera_stream_port (int, optional): Port to stream grayscale camera images to.
                                                If no port is supplied the camera will not be streamed. Defaults to None.
            rangefinder_name (str, optional): Webots RangeFinder name. Defaults to None.
            rangefinder_fps (int, optional): RangeFinder FPS. Lower FPS runs better in sim. Defaults to 10.
            rangefinder_stream_port (int, optional): Port to stream rangefinder images to.
                                                     If no port is supplied the camera will not be streamed. Defaults to None.
            instance (int, optional): Vehicle instance number to match the SITL. This allows multiple vehicles. Defaults to 0.
            motor_velocity_cap (float, optional): Motor velocity cap. This is useful for the crazyflie
                                                  which default has way too much power. Defaults to float('inf').
            reversed_motors (list[int], optional): Reverse the motors (indexed from 1). Defaults to None.
            bidirectional_motors (bool, optional): Enable bidirectional motors. Defaults to False.
            uses_propellers (bool, optional): Whether the vehicle uses propellers.
                                              This is important as we need to linearize thrust if so. Defaults to True.
            sitl_address (str, optional): IP address of the SITL (useful with WSL2 eg \"172.24.220.98\").
                                          Defaults to "127.0.0.1".
        """
        # 保存电机特性、实例编号和连接状态。
        self.motor_velocity_cap = motor_velocity_cap
        self._instance = instance
        self._reversed_motors = reversed_motors
        self._bidirectional_motors = bidirectional_motors
        self._uses_propellers = uses_propellers
        self._level_attitude_lock = level_attitude_lock
        self._webots_connected = True

        # 启用起飞锁时必须使用 Supervisor，才能固定机体并重置物理状态。
        self.robot = Supervisor() if launch_lock_threshold is not None else Robot()
        self._launch_lock_threshold = launch_lock_threshold
        self._launch_lock_released = launch_lock_threshold is None
        self._launch_assist_until = None
        if not self._launch_lock_released:
            self._self_node = self.robot.getSelf()
            self._initial_translation = self._self_node.getField("translation").getSFVec3f()
            self._initial_rotation = self._self_node.getField("rotation").getSFRotation()

        # 所有传感器和控制循环都使用当前世界的基础时间步长。
        self._timestep = int(self.robot.getBasicTimeStep())

        # 获取并启用飞控所需的惯导、角速度和 GPS 传感器。
        self.accel = self.robot.getDevice(accel_name)
        self.imu = self.robot.getDevice(imu_name)
        self.gyro = self.robot.getDevice(gyro_name)
        self.gps = self.robot.getDevice(gps_name)

        self.accel.enable(self._timestep)
        self.imu.enable(self._timestep)
        self.gyro.enable(self._timestep)
        self.gps.enable(self._timestep)

        # 可选相机：仅在提供设备名时启用。
        if camera_name is not None:
            self.camera = self.robot.getDevice(camera_name)
            if self.camera is None:
                print(f"Warning: camera device '{camera_name}' was not found (I{self._instance}); camera disabled")
            else:
                self.camera.enable(1000//camera_fps) # takes frame period in ms

                # 指定 TCP 端口后才启动相机流线程。
                if camera_stream_port is not None:
                    self._camera_thread = Thread(daemon=True,
                                                 target=self._handle_image_stream,
                                                 args=[self.camera, camera_stream_port])
                    self._camera_thread.start()

        # 可选距离传感器，初始化方式与相机相同。
        if rangefinder_name is not None:
            self.rangefinder = self.robot.getDevice(rangefinder_name)
            if self.rangefinder is None:
                print(f"Warning: rangefinder device '{rangefinder_name}' was not found (I{self._instance}); rangefinder disabled")
            else:
                self.rangefinder.enable(1000//rangefinder_fps) # takes frame period in ms

                # 指定 TCP 端口后才启动深度图流线程。
                if rangefinder_stream_port is not None:
                    self._rangefinder_thread = Thread(daemon=True,
                                                      target=self._handle_image_stream,
                                                      args=[self.rangefinder, rangefinder_stream_port])
                    self._rangefinder_thread.start()

        # 设置无限位置模式，从而直接控制电机角速度。
        self._motors = [self.robot.getDevice(n) for n in motor_names]
        for m in self._motors:
            m.setPosition(float('inf'))
            m.setVelocity(0)
        self._motor_output_gate_open = False
        self._motor_output_gate_warned = False

        # 后台线程负责 SITL UDP 收发，端口按实例号每架间隔 10。
        self._sitl_thread = Thread(daemon=True, target=self._handle_sitl, args=[sitl_address, 9002+10*instance])
        self._sitl_thread.start()

    def _handle_sitl(self, sitl_address: str = "127.0.0.1", port: int = 9002):
        """处理与 ArduPilot SITL 之间的全部 UDP 通信。

        Args:
            port (int, optional): Port to listen for SITL on. Defaults to 9002.
        """

        # 本地 UDP 服务端监听 SITL 发来的电机控制包。
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # SOCK_STREAM
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('0.0.0.0', port))

        # 先推进一个仿真步，以便 Webots 控制台及时显示监听信息。
        print(f"Listening for ardupilot SITL (I{self._instance}) at 0.0.0.0:{port}")
        self.robot.step(self._timestep) # flush print in webots console

        # 未参与当前测试的无人机可能始终停在这里。等待阶段使用至少 50 ms
        # 的步长，保持响应的同时避免空闲控制器以 2 ms 物理步长频繁同步。
        waiting_timestep = max(self._timestep, 50)
        waiting_timestep = ((waiting_timestep + self._timestep - 1) // self._timestep) * self._timestep
        while not select.select([s], [], [], 0)[0]: # 等待套接字可读。
            # Webots 关闭时及时释放套接字并通知主线程。
            if self.robot.step(waiting_timestep) == -1:
                s.close()
                self._webots_connected = False
                return

        print(f"Connected to ardupilot SITL (I{self._instance})")

        # 每收到一组电机指令，就推进一个 Webots 时间步并回传一组 FDM 数据。
        while True:
            # 非阻塞检查套接字，避免没有 SITL 数据时卡住 Webots。
            readable, _, _ = select.select([s], [], [], 0)

            # 接收并解析 SITL 固定长度的电机输出结构体。
            if readable:
                data, peer = s.recvfrom(512)
                if not data or len(data) < self.controls_struct_size:
                    continue
                # 一个数据报只解析一个控制结构体。
                command = struct.unpack(self.controls_struct_format, data[:self.controls_struct_size])
                self._handle_controls(command)

                # 推进仿真后传感器数据才会更新。
                step_success = self.robot.step(self._timestep)
                if step_success == -1: # webots closed
                    break

                # 回传当前时间步的飞行动力学/传感器数据。
                fdm_struct = self._get_fdm_struct()
                s.sendto(fdm_struct, (sitl_address, port+1))

        # 离开循环说明 Webots 已关闭。
        s.close()
        self._webots_connected = False
        print(f"Lost connection to Webots (I{self._instance})")

    def _get_fdm_struct(self) -> bytes:
        """生成发送给 SITL 的飞行动力学模型（FDM）传感器数据包。

        Returns:
            bytes: bytes representing the struct to send to SITL
        """
        # 按同一仿真时间步读取姿态、角速度、加速度和 GPS。
        i = self.imu.getRollPitchYaw()
        g = self.gyro.getValues()
        a = self.accel.getValues()
        gps_pos = self.gps.getValues()
        gps_vel = self.gps.getSpeedVector()

        # 按 ArduPilot Webots 后端约定打包，并完成 Webots 到 NED 的轴向变换。
        # https://discuss.ardupilot.org/t/copter-x-y-z-which-is-which/6823/3
        # struct fdm_packet {
        #     double timestamp;
        #     double imu_angular_velocity_rpy[3];
        #     double imu_linear_acceleration_xyz[3];
        #     double imu_orientation_rpy[3];
        #     double velocity_xyz[3];
        #     double position_xyz[3];
        # };
        return struct.pack(self.fdm_struct_format,
                           self.robot.getTime(),
                           g[0], -g[1], -g[2],
                           a[0], -a[1], -a[2],
                           i[0], -i[1], -i[2],
                           gps_vel[0], -gps_vel[1], -gps_vel[2],
                           gps_pos[0], -gps_pos[1], -gps_pos[2])

    def _handle_controls(self, command: tuple):
        """将 SITL 的归一化控制量转换为 Webots 电机角速度。

        Args:
            command (tuple): tuple of motor speeds 0.0-1.0 where -1.0 is unused
        """

        # SITL 包最多含 16 路执行器，本模型只截取实际电机数量。
        command_motors = command[:len(self._motors)]
        if not self._bidirectional_motors:
            command_motors = [min(max(v, 0.0), 1.0) for v in command_motors]

        # 启动后必须先观察到一次全零输出，防止复用旧 SITL 状态时突然满油门。
        if not self._motor_output_gate_open:
            if all(abs(v) <= 0.001 for v in command_motors):
                self._motor_output_gate_open = True
                print(f"[Drone I{self._instance}] Safe zero-motor state confirmed; outputs enabled")
            else:
                if not self._motor_output_gate_warned:
                    print(f"[Drone I{self._instance}] Blocking stale armed SITL motor output until disarm")
                    self._motor_output_gate_warned = True
                for motor in self._motors:
                    motor.setVelocity(0)
                return

        # 起飞锁释放前先换算归一化指令，并允许螺旋桨在机体锁定时预转，
        # 避免释放瞬间出现阶跃式推力。
        if self._bidirectional_motors:
            command_motors = [v*2-1 for v in command_motors]
        if self._uses_propellers:
            # Webots 推力模型为 Thrust = thrust_constant * |omega| * omega，
            # 因而用平方根把线性推力指令换算为角速度比例。
            linearized_motor_commands = [np.sqrt(np.abs(v))*np.sign(v) for v in command_motors]
        else:
            linearized_motor_commands = command_motors

        if self._reversed_motors:
            for motor_number in self._reversed_motors:
                linearized_motor_commands[motor_number-1] *= -1

        # 平均控制量达到阈值前，由 Supervisor 固定初始位置与姿态。
        if not self._launch_lock_released:
            mean_command = sum(command_motors) / len(command_motors)
            for index, motor in enumerate(self._motors):
                motor.setVelocity(
                    linearized_motor_commands[index]
                    * min(motor.getMaxVelocity(), self.motor_velocity_cap)
                )
            self._self_node.getField("translation").setSFVec3f(self._initial_translation)
            self._self_node.getField("rotation").setSFRotation(self._initial_rotation)
            self._self_node.resetPhysics()
            if mean_command < self._launch_lock_threshold:
                return
            self._launch_lock_released = True
            # Only suppress the rope's instantaneous unlock impulse.  Holding
            # attitude for several seconds prevents ArduPilot from responding
            # while its position controller keeps integrating, which causes a
            # delayed and eventually divergent pitch oscillation.
            self._launch_assist_until = self.robot.getTime() + 2.0
            print(
                f"[Drone I{self._instance}] Launch lock released at "
                f"mean command {mean_command:.3f}"
            )

        # 多刚体吊绳在解锁瞬间可能产生较大角冲量。最初 2 秒只锁定水平姿态，
        # 线运动仍遵循物理仿真；辅助结束后由 ArduPilot 完全接管。
        if self._launch_assist_until is not None or self._level_attitude_lock:
            if self._level_attitude_lock or self.robot.getTime() < self._launch_assist_until:
                velocity = self._self_node.getVelocity()
                self._self_node.getField("rotation").setSFRotation(self._initial_rotation)
                self._self_node.setVelocity([velocity[0], velocity[1], velocity[2], 0, 0, 0])
            else:
                self._launch_assist_until = None

        # 调试输出：每约 50 个控制包打印一次原始 SERVO 值。
        if not hasattr(self, '_debug_counter'):
            self._debug_counter = 0
        self._debug_counter += 1
        if self._debug_counter % 50 == 0:
            print(f"[Drone I{self._instance}] SERVO raw: {[f'{v:.4f}' for v in command_motors]}")

        # 将最终角速度写入 Webots 四个电机。
        for i, m in enumerate(self._motors):
            m.setVelocity(linearized_motor_commands[i] * min(m.getMaxVelocity(), self.motor_velocity_cap))

    def _handle_image_stream(self, camera: Union[Camera, RangeFinder], port: int):
        """通过本地 TCP 持续发送灰度相机图像或距离图像。

        Args:
            camera (Camera or RangeFinder): the camera to get images from
            port (int): port to send images over
        """

        # 读取设备分辨率与采样周期，并统一两类图像设备的后续处理。
        # https://cyberbotics.com/doc/reference/camera
        if isinstance(camera, Camera):
            cam_sample_period = self.camera.getSamplingPeriod()
            cam_width = self.camera.getWidth()
            cam_height = self.camera.getHeight()
            print(f"Camera stream started at 127.0.0.1:{port} (I{self._instance}) "
                  f"({cam_width}x{cam_height} @ {1000/cam_sample_period:0.2f}fps)")
        elif isinstance(camera, RangeFinder):
            cam_sample_period = self.rangefinder.getSamplingPeriod()
            cam_width = self.rangefinder.getWidth()
            cam_height = self.rangefinder.getHeight()
            print(f"RangeFinder stream started at 127.0.0.1:{port} (I{self._instance}) "
                  f"({cam_width}x{cam_height} @ {1000/cam_sample_period:0.2f}fps)")
        else:
            print(sys.stderr, f"Error: camera passed to _handle_image_stream is of invalid type "
                              f"'{type(camera)}' (I{self._instance})")
            return

        # 图像流仅监听 localhost，不暴露到外部网络。
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('127.0.0.1', port))
        server.listen(1)

        # 客户端断开后重新等待下一次连接，直到 Webots 关闭。
        while self._webots_connected:
            # 每次只服务一个图像客户端。
            conn, _ = server.accept()
            print(f"Connected to camera client (I{self._instance})")

            # 按传感器采样频率连续发送图像帧。
            try:
                while self._webots_connected:
                    # 记录当前仿真时间，用于控制发送周期。
                    start_time = self.robot.getTime()

                    # 根据设备类型取得灰度图或归一化深度图。
                    if isinstance(camera, Camera):
                        img = self.get_camera_gray_image()
                    elif isinstance(camera, RangeFinder):
                        img = self.get_rangefinder_image()

                    if img is None:
                        print(f"No image received (I{self._instance})")
                        time.sleep(cam_sample_period/1000)
                        continue

                    # 每帧头部包含两个无符号短整数：宽度和高度。
                    header = struct.pack("=HH", cam_width, cam_height)

                    # 先发帧头，再发连续像素字节。
                    data = header + img.tobytes()
                    conn.sendall(data)

                    # 等待下一个传感器采样周期；这里使用的是仿真时间。
                    while self.robot.getTime() - start_time < cam_sample_period/1000:
                        time.sleep(0.001)

            except ConnectionResetError:
                pass
            except BrokenPipeError:
                pass
            finally:
                conn.close()
                print(f"Camera client disconnected (I{self._instance})")

    def get_camera_gray_image(self) -> np.ndarray:
        """读取相机并返回 uint8 灰度图数组。"""
        img = self.get_camera_image()
        img_gray = np.average(img, axis=2).astype(np.uint8)
        return img_gray

    def get_camera_image(self) -> np.ndarray:
        """读取 Webots BGRA/RGBA 缓冲区并返回三通道 uint8 图像。"""
        img = self.camera.getImage()
        img = np.frombuffer(img, np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4))
        return img[:, :, :3] # 只保留颜色通道，去掉 Alpha。

    def get_rangefinder_image(self, use_int16: bool = False) -> np.ndarray:
        """读取距离图，并归一化为 uint8 或 uint16 数组。"""\

        # 获取距离图尺寸。
        height = self.rangefinder.getHeight()
        width = self.rangefinder.getWidth()

        # 将 Webots 原始 ctypes 缓冲区转换成二维 NumPy 数组。
        # https://cyberbotics.com/doc/reference/rangefinder
        image_c_ptr = self.rangefinder.getRangeImage(data_type="buffer")
        img_arr = np.ctypeslib.as_array(image_c_ptr, (width*height,))
        img_floats = img_arr.reshape((height, width))

        # 归一化到 0～1，并把无穷远/未知值设置为最大距离。
        range_range = self.rangefinder.getMaxRange() - self.rangefinder.getMinRange()
        img_normalized = (img_floats - self.rangefinder.getMinRange()) / range_range
        img_normalized[img_normalized == float('inf')] = 1

        # 根据精度需求量化为 8 位或 16 位无符号整数。
        if use_int16:
            img = (img_normalized * 65535).astype(np.uint16)
        else:
            img = (img_normalized * 255).astype(np.uint8)

        return img

    def stop_motors(self):
        """将全部电机切回速度控制并设置为零角速度。"""
        for m in self._motors:
            m.setPosition(float('inf'))
            m.setVelocity(0)

    def webots_connected(self) -> bool:
        """返回 Webots 控制循环是否仍处于连接状态。"""
        return self._webots_connected
