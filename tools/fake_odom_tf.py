#!/usr/bin/env python3
"""发布一条假的 odom -> base_link TF，用于【无硬件】离线调试整条链路。

位姿是脚本化的、量值已知，方便对着看板核对：
    静止 → 直线 +1.0 m → 静止 → 原地 +90° → 静止（循环）

用法:
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 tools/fake_odom_tf.py          # 默认 50 Hz 循环
    python3 tools/fake_odom_tf.py 20       # 20 Hz
"""
import os
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# ── 路径解析：从本文件位置推出仓库根，保证 clone 到任何地方都能用 ──
_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_LOGS = os.path.join(_REPO, "logs")
_INSTALL_SETUP = os.path.join(_REPO, "install", "setup.bash")


X0, Y0, YAW0 = 10.0, 5.0, 0.30
LINEAR, TURN = 1.000, math.pi / 2.0
T_B, T_C, T_D, T_E, T_LOOP = 3.0, 5.0, 7.0, 9.0, 12.0


class FakeOdom(Node):
    def __init__(self, rate):
        super().__init__("fake_odom_tf_pub")
        self.br = TransformBroadcaster(self)
        self.t0 = None
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f"发布假 odom->base_link @ {rate:.0f} Hz；"
            f"轨迹: 静止→+{LINEAR} m→静止→+{math.degrees(TURN):.0f}°→静止 (每 {T_LOOP:.0f}s 循环)")

    @staticmethod
    def pose_at(t):
        t = t % T_LOOP
        xe, ye = X0 + LINEAR * math.cos(YAW0), Y0 + LINEAR * math.sin(YAW0)
        if t < T_B:
            return X0, Y0, YAW0
        if t < T_C:
            s = LINEAR * (t - T_B) / (T_C - T_B)
            return X0 + s * math.cos(YAW0), Y0 + s * math.sin(YAW0), YAW0
        if t < T_D:
            return xe, ye, YAW0
        if t < T_E:
            return xe, ye, YAW0 + TURN * (t - T_D) / (T_E - T_D)
        return xe, ye, YAW0 + TURN

    def tick(self):
        now = time.time()
        if self.t0 is None:
            self.t0 = now
        x, y, yaw = self.pose_at(now - self.t0)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = math.sin(yaw * 0.5)
        t.transform.rotation.w = math.cos(yaw * 0.5)
        self.br.sendTransform(t)


def main():
    rate = float(sys.argv[1]) if len(sys.argv) > 1 else 50.0
    rclpy.init()
    node = FakeOdom(rate)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass


if __name__ == "__main__":
    main()
