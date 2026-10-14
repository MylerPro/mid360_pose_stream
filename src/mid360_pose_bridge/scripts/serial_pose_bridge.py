#!/usr/bin/env python3
"""Stream the robot's current pose to the STM32 over USB CDC at a fixed rate.

Wire format (15 bytes, unchanged):
  AA 55 | x(float32 LE) | y(float32 LE) | yaw(float32 LE) | sum8

Pose source
-----------
The default source is the **odometry** chain ``odom -> base_link`` (FAST-LIO,
which fuses the MID360 IMU at 200 Hz with the point cloud). This deliberately
needs no map, no AMCL and no initial pose: it works as soon as the lidar runs.
Set ``map_frame``/``base_frame`` to change the source.

Origin (零点)
-------------
The transmitted values are the current pose expressed **relative to a movable
origin**, using the exact same 2D rigid transform as ``serial_goal_bridge``::

    dx = x - x0 ; dy = y - y0
    x_rel =  cos(yaw0)*dx + sin(yaw0)*dy
    y_rel = -sin(yaw0)*dx + cos(yaw0)*dy
    yaw_rel = normalize_angle(yaw - yaw0)

The origin is locked automatically on the first valid pose (``auto_origin``)
and can be re-locked at any time through the ``refresh_origin`` service. A
refresh immediately transmits one ``(0, 0, 0)`` frame so the STM32 re-zeros
at the same instant (this is the "inject the origin pose" convention).

No navigation goal and no ``/cmd_vel`` is ever serialised by this node.
"""
import math
import struct
import time

import rclpy
import serial
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


FRAME_HEADER = b"\xAA\x55"
FRAME_LENGTH = 15


def pack_pose_frame(x_m, y_m, yaw_rad):
    values = (float(x_m), float(y_m), float(yaw_rad))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("x, y and yaw must be finite float32 values")
    body = FRAME_HEADER + struct.pack("<fff", *values)
    packet = body + bytes([sum(body) & 0xFF])
    if len(packet) != FRAME_LENGTH:
        raise AssertionError(f"pose frame must contain {FRAME_LENGTH} bytes")
    return packet


def normalize_angle(angle):
    """Wrap an angle into [-pi, pi]. Same helper as serial_goal_bridge."""
    return math.atan2(math.sin(angle), math.cos(angle))


class SerialPoseBridge(Node):
    def __init__(self):
        super().__init__("serial_pose_bridge")
        defaults = {
            "serial_port": (
                "/dev/serial/by-id/"
                "usb-STMicroelectronics_STM32_Virtual_ComPort_3154365A3233-if00"
            ),
            "baudrate": 115200,
            # 默认与 launch 的 send_rate 一致；launch 会用参数覆盖这里。
            "send_rate": 60.0,
            # ── 发散看门狗 ──────────────────────────────────────────────
            # 2026-10-14 事故：FAST-LIO 发散到 1000+ 米（等效 163 m/s），
            # 桥毫不知情地按 60 Hz 把这个位姿发了 1 分多钟给底盘 ✗
            # 现在按【物理可能的速度】判据拦截：整车不可能跑得比 max_speed 快。
            # 连续 divergence_frames 帧超限就【锁死停发】，并每轮统计打印 ERROR。
            # 清除方式：调用 refresh_origin 服务（表示操作员已检查并重新归零）。
            # 设 max_speed 为 0 或负数可关闭这个保护（不建议）。
            # 2026-10-14 实测：FAST-LIO 的正常匹配修正会出现单帧约 7.5 cm 的
            # 跳变（等效 4.49 m/s），设 3.0 会让 5 次连续跳变误锁死停发。
            # 放宽到 5.0 —— 仍远低于发散时的量级（上次实测 163 m/s，537 m/s）。
            "max_speed": 5.0,          # m/s，整车物理上限
            "divergence_frames": 5,    # 连续超限多少帧后锁死（60Hz 下仅 83 ms）
            # 默认取里程计链 odom -> base_link（FAST-LIO，融合 MID360 的 IMU 与点云），
            # 不依赖地图/AMCL/初始位姿。要改用全局定位就把 map_frame 设成 "map"。
            "map_frame": "odom",
            "base_frame": "base_link",
            "pose_topic": "/chassis/pose",
            "tx_pose_topic": "/chassis/serial_tx_pose",
            "origin_topic": "/chassis/origin",
            # 用 "~/..." 私有名，解析成 /serial_pose_bridge/refresh_origin，
            # 与 serial_goal_bridge 的 "~/set_origin" 保持同一约定。
            "refresh_service": "~/refresh_origin",
            # 必须大于 FAST-LIO 的启动积压，否则 current_pose() 每次抛异常、
            # 一帧都发不出去（而且是静默的）。实测 map->base_link 年龄 1.29~1.64s。
            "max_pose_age": 5.0,
            # 启动后收到第一帧有效位姿就自动锁零点：底盘上电时机器人没动，
            # 两边零点天然重合，所以开机即用、无需手动操作。
            "auto_origin": True,
            "log_frames": True,
            "stats_period": 5.0,
            "reconnect_period": 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.port_name = str(self.get_parameter("serial_port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.max_pose_age = max(float(self.get_parameter("max_pose_age").value), 0.0)
        self.auto_origin = bool(self.get_parameter("auto_origin").value)
        # 发散看门狗（见 defaults 里的说明）
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.divergence_frames = max(
            int(self.get_parameter("divergence_frames").value), 1)
        self.origin_topic = str(self.get_parameter("origin_topic").value)
        self.log_frames = bool(self.get_parameter("log_frames").value)
        self.reconnect_period = max(
            float(self.get_parameter("reconnect_period").value), 0.2)

        rate = float(self.get_parameter("send_rate").value)
        if not 0.0 < rate <= 100.0:
            raise ValueError("send_rate must be in (0, 100] Hz")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("pose_topic").value), 10)
        self.tx_pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("tx_pose_topic").value), 10)
        self.origin_pub = self.create_publisher(PoseStamped, self.origin_topic, 10)

        self.origin = None          # (x0, y0, yaw0) in the source frame
        self.origin_locks = 0
        self.zero_frames_sent = 0
        self.refresh_srv = self.create_service(
            Trigger, str(self.get_parameter("refresh_service").value),
            self.handle_refresh_origin)

        self.port = None
        self.tx_frames = 0
        self.tx_bytes = 0
        self.tx_errors = 0
        self.last_values = None
        # 发散看门狗状态
        self._wd_prev = None           # (t, x, y) 上一帧用于算速度
        self._wd_strikes = 0           # 连续超限计数
        self._wd_latched = False       # 锁死标记
        self._wd_worst_speed = 0.0     # 观察到的最快等效速度
        self._wd_at = None             # 锁死时刻
        self.wait_reason = "waiting for localization TF"
        self.last_reconnect_attempt_ns = 0
        self._open_port(initial=True)

        self.send_timer = self.create_timer(1.0 / rate, self.send_current_pose)
        stats_period = max(float(self.get_parameter("stats_period").value), 0.0)
        if stats_period > 0.0:
            self.stats_timer = self.create_timer(stats_period, self.report_stats)

    def _open_port(self, initial=False):
        try:
            self.port = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.0,
                write_timeout=0.2,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
                # 【独占】走 TIOCEXCL：本进程持有期间，别的进程再 open 同一个
                # 设备会直接失败（EBUSY）。这是防止"两个 writer 交错写 15 字节帧"
                # 最根本的一道保护 —— 因为帧交错会让帧头/校验全乱，而现象是
                # STM32 侧 g_pl_frame_bad 上涨，很难反查到 NUC 这边有两个进程。
                # 曾经真实发生过：同时跑两套 launch 时 /serial_goal/tx_pose
                # 有 2 个发布者。
                exclusive=True,
            )
            verb = "Opened" if initial else "Re-opened"
            self.get_logger().info(
                f"{verb} {self.port_name}; {self.baudrate} 8N1; "
                "protocol=AA55/x-f32/y-f32/yaw-f32/sum8; frame_len=15; exclusive=ON")
        except (serial.SerialException, OSError) as error:
            self.port = None
            message = f"Cannot open {self.port_name}: {error}"
            if initial:
                self.get_logger().error(message)
            else:
                self.get_logger().warning(message)

    def _try_reconnect(self):
        now_ns = self.get_clock().now().nanoseconds
        period_ns = int(self.reconnect_period * 1e9)
        if now_ns - self.last_reconnect_attempt_ns >= period_ns:
            self.last_reconnect_attempt_ns = now_ns
            self._open_port()

    @staticmethod
    def yaw_from_quaternion(quaternion):
        values = (
            quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("non-finite orientation quaternion")
        norm = math.sqrt(sum(value * value for value in values))
        if norm < 1e-6:
            raise ValueError("zero orientation quaternion")
        x, y, z, w = (value / norm for value in values)
        return math.atan2(2.0 * (w * z + x * y),
                          1.0 - 2.0 * (y * y + z * z))

    def current_pose(self):
        transform = self.tf_buffer.lookup_transform(
            self.map_frame, self.base_frame, rclpy.time.Time())
        stamp = transform.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self.max_pose_age > 0.0 and stamp_ns > 0:
            age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
            if age > self.max_pose_age or age < -self.max_pose_age:
                raise ValueError(f"localization TF age is {age:.3f}s")

        pose = PoseStamped()
        pose.header = transform.header
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        yaw = self.yaw_from_quaternion(pose.pose.orientation)
        values = (pose.pose.position.x, pose.pose.position.y, yaw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("non-finite localization pose")
        return pose, values

    def publish_origin(self, reference_pose=None):
        """Publish the locked origin so it is observable from the outside."""
        pose = PoseStamped()
        if reference_pose is not None:
            pose.header = reference_pose.header
        else:
            pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        x0, y0, yaw0 = self.origin
        pose.pose.position.x = x0
        pose.pose.position.y = y0
        pose.pose.orientation.z = math.sin(yaw0 * 0.5)
        pose.pose.orientation.w = math.cos(yaw0 * 0.5)
        self.origin_pub.publish(pose)

    def relative_pose(self, values):
        """Express ``values`` relative to the locked origin.

        Uses the exact same 2D rigid transform as ``serial_goal_bridge``:
        translate to the origin, then rotate by ``-yaw0``, and wrap yaw into
        [-pi, pi].  Identical maths means the two nodes can never disagree.
        """
        if self.origin is None:
            raise ValueError("origin is not locked yet")
        x0, y0, yaw0 = self.origin
        x, y, yaw = values
        dx = x - x0
        dy = y - y0
        cos_yaw = math.cos(yaw0)
        sin_yaw = math.sin(yaw0)
        # ROS base frame convention: +x forward, +y left, yaw CCW positive.
        x_rel = cos_yaw * dx + sin_yaw * dy
        y_rel = -sin_yaw * dx + cos_yaw * dy
        yaw_rel = normalize_angle(yaw - yaw0)
        return (x_rel, y_rel, yaw_rel)

    def lock_origin(self, reason, reference_pose=None):
        """Record the current pose as the new zero point."""
        pose, values = self.current_pose()
        self.origin = values
        self.origin_locks += 1
        self.publish_origin(pose if reference_pose is None else reference_pose)
        self.get_logger().info(
            f"ORIGIN_LOCKED reason={reason} x0={values[0]:.4f}m "
            f"y0={values[1]:.4f}m yaw0={values[2]:.4f}rad "
            f"frame={self.map_frame}->{self.base_frame}")
        return values

    def write_frame(self, values, note=""):
        """Write one 15-byte frame. Returns True when the bytes left the host."""
        if self.port is None:
            self._try_reconnect()
            if self.port is None:
                self.wait_reason = "serial port is not open"
                return False
        packet = pack_pose_frame(*values)
        try:
            written = self.port.write(packet)
            if written != len(packet):
                raise serial.SerialException(
                    f"short serial write: {written}/{len(packet)} bytes")
            self.port.flush()
        except (serial.SerialException, OSError) as error:
            self.tx_errors += 1
            self.get_logger().error(f"Serial pose write failed: {error}")
            try:
                self.port.close()
            except Exception:
                pass
            self.port = None
            self.wait_reason = f"write failed: {error}"
            return False
        self.tx_frames += 1
        self.tx_bytes += written
        self.last_values = values
        self.wait_reason = ""
        if self.log_frames:
            suffix = f" {note}" if note else ""
            self.get_logger().info(
                f"TX pose x={values[0]:.4f}m y={values[1]:.4f}m "
                f"yaw={values[2]:.4f}rad hex={packet.hex(' ')}{suffix}")
        return True

    def handle_refresh_origin(self, request, response):
        # 刷新零点同时解除发散锁死（表示操作员已检查过并重新归零）
        if self._wd_latched:
            self.get_logger().warn(
                "refresh_origin 解除发散锁死，恢复发送（看门狗计数已清零）")
            self._wd_latched = False
            self._wd_strikes = 0
            self._wd_worst_speed = 0.0
            self._wd_prev = None
        """Re-lock the zero point and immediately tell the STM32 to re-zero.

        The ``(0, 0, 0)`` frame is the "inject the origin pose" convention: it
        makes the STM32 treat wherever it is right now as its zero.  Because the
        new origin IS the current pose, the regular stream would also produce
        (0, 0, 0) on the very next tick -- sending it here just makes the
        re-zero take effect immediately instead of up to one period later.
        """
        del request
        try:
            values = self.lock_origin("refresh_service")
        except (TransformException, ValueError) as error:
            response.success = False
            response.message = (
                f"cannot refresh origin: {error} "
                f"(frame={self.map_frame}->{self.base_frame})")
            self.get_logger().warning(response.message)
            return response

        sent = self.write_frame((0.0, 0.0, 0.0), note="[origin zero frame]")
        if sent:
            self.zero_frames_sent += 1
        response.success = True
        response.message = (
            f"origin refreshed at x={values[0]:.4f}, y={values[1]:.4f}, "
            f"yaw={values[2]:.4f}; zero frame "
            f"{'sent' if sent else 'NOT sent (serial unavailable)'}")
        return response

    def send_current_pose(self):
        try:
            pose, values = self.current_pose()
        except (TransformException, ValueError) as error:
            self.wait_reason = str(error)
            return

        self.pose_pub.publish(pose)

        if self.origin is None:
            if not self.auto_origin:
                self.wait_reason = (
                    "origin is not locked; call the refresh_origin service")
                return
            try:
                self.lock_origin("auto_first_pose", reference_pose=pose)
            except (TransformException, ValueError) as error:
                self.wait_reason = str(error)
                return

        # ── 发散看门狗：在发送前拦截 ──
        # 用【绝对位姿】的速度判断，与零点无关；整车不可能超过 max_speed。
        if self._wd_latched:
            self.wait_reason = (f"DIVERGENCE LATCHED at {self._wd_at} "
                                f"(peak {self._wd_worst_speed:.1f} m/s) — "
                                f"call refresh_origin to clear")
            return
        if self.max_speed > 0.0:
            now_m = time.monotonic()
            if self._wd_prev is not None:
                t_prev, x_prev, y_prev = self._wd_prev
                dt = now_m - t_prev
                if dt > 1e-6:
                    speed = math.hypot(values[0] - x_prev, values[1] - y_prev) / dt
                    self._wd_worst_speed = max(self._wd_worst_speed, speed)
                    if speed > self.max_speed:
                        self._wd_strikes += 1
                    else:
                        self._wd_strikes = 0
                    if self._wd_strikes >= self.divergence_frames:
                        self._wd_latched = True
                        self._wd_at = time.strftime("%H:%M:%S")
                        self.get_logger().error(
                            f"DIVERGENCE DETECTED — 位姿源在发散，已【停止发送】！"
                            f" 等效速度 {speed:.1f} m/s > 上限 {self.max_speed:.1f} m/s，"
                            f"连续 {self._wd_strikes} 帧超限；峰值 {self._wd_worst_speed:.1f} m/s。"
                            f" 底盘不会收到任何字节（避免乱跑）。"
                            f" 查因：① lidar_type 是否 1（CustomMsg 真实时间戳）"
                            f" ② 静置时 /odom 是否漂移  ③ frame_rate 是否过高。"
                            f" 处理完后调用 refresh_origin 服务解除。")
                        return
            self._wd_prev = (now_m, values[0], values[1])

        try:
            relative = self.relative_pose(values)
        except ValueError as error:
            self.wait_reason = str(error)
            return

        if self.write_frame(relative):
            self.tx_pose_pub.publish(pose)

    def report_stats(self):
        state = "open" if self.port is not None and self.port.is_open else "closed"
        detail = f" waiting={self.wait_reason}" if self.wait_reason else ""
        if self.origin is None:
            detail += " origin=UNLOCKED"
        else:
            x0, y0, yaw0 = self.origin
            detail += (f" origin=({x0:.4f},{y0:.4f},{yaw0:.4f})"
                       f" locks={self.origin_locks}")
        if self.last_values is not None:
            x_m, y_m, yaw_rad = self.last_values
            detail += f" last_tx=({x_m:.4f},{y_m:.4f},{yaw_rad:.4f})"
        if self._wd_latched:
            detail += " WATCHDOG=LATCHED(stopped)"
        elif self._wd_strikes:
            detail += f" WATCHDOG=strikes{self._wd_strikes}"
        elif self.max_speed > 0.0:
            detail += f" peak_speed={self._wd_worst_speed:.2f}m/s"
        self.get_logger().info(
            f"serial pose TX: port={state} frames={self.tx_frames} "
            f"bytes={self.tx_bytes} errors={self.tx_errors}"
            f" zero_frames={self.zero_frames_sent}{detail}")

    def destroy_node(self):
        if self.port is not None:
            try:
                self.port.close()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = SerialPoseBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
