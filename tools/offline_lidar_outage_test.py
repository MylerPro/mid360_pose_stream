#!/usr/bin/env python3
"""离线测试：雷达断流保护（不需要任何硬件）。

模拟 2026-10-15 的现场故障：车体调试断电 → 雷达掉电 → 位姿漂移 → 恢复后发散。
验证桥内两道防线：

  ① 雷达断流 > max_lidar_gap(1.0s) → 【停发】（位姿退化成纯 IMU 推算，不可信）
  ② 雷达断流 > lidar_relock_gap(5.0s) → 【锁死】，必须调 refresh_origin 才恢复
     （车体断电时底盘也忘了零点，所以必须强制重新建立两边零点）

用法:
    source install/setup.bash
    python3 tools/offline_lidar_outage_test.py
"""
import math
import os
import pty
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster
from std_srvs.srv import Trigger

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BRIDGE = os.path.join(REPO, "install", "mid360_pose_bridge", "lib",
                      "mid360_pose_bridge", "serial_pose_bridge.py")

OK, BAD, WARN = "✓", "✗", "!"
results = []


def check(name, ok, detail=""):
    results.append((name, ok))
    print(f"  {OK if ok else BAD} {name:38s} {detail}", flush=True)


def make_cloud(node, n=20):
    msg = PointCloud2()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "livox_frame"
    msg.height, msg.width = 1, n
    msg.fields = [PointField(name="x", offset=0, datatype=7, count=1),
                  PointField(name="y", offset=4, datatype=7, count=1),
                  PointField(name="z", offset=8, datatype=7, count=1)]
    msg.point_step, msg.row_step = 12, 12 * n
    msg.is_dense = True
    msg.data = b"\x00" * (12 * n)
    return msg


class Sim(Node):
    """假雷达 + 假 TF + 断流开关。"""

    def __init__(self):
        super().__init__("lidar_outage_sim")
        q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PointCloud2, "/livox/lidar", q)
        self.tfb = TransformBroadcaster(self)
        self.emit_lidar = True
        self.t0 = time.time()

    def tick(self):
        now = time.time() - self.t0
        if self.emit_lidar:
            self.pub.publish(make_cloud(self))
        m = TransformStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "odom"
        m.child_frame_id = "base_link"
        m.transform.translation.x = 0.1 * math.sin(now * 0.5)
        m.transform.rotation.w = 1.0
        self.tfb.sendTransform(m)


def main():
    if not os.path.exists(BRIDGE):
        print(f"  {BAD} 找不到桥: {BRIDGE}\n     先 colcon build --symlink-install")
        return 1

    rclpy.init()
    master, slave = pty.openpty()
    port = os.ttyname(slave)
    sim = Sim()

    bridge = subprocess.Popen(
        ["python3", BRIDGE, "--ros-args", "-r", "__node:=serial_pose_bridge",
         "-p", f"serial_port:={port}", "-p", "send_rate:=20.0",
         "-p", "map_frame:=odom", "-p", "base_frame:=base_link",
         "-p", "auto_origin:=true", "-p", "max_pose_age:=5.0",
         "-p", "log_frames:=false", "-p", "stats_period:=2.0",
         "-p", "max_lidar_gap:=1.0", "-p", "lidar_relock_gap:=5.0"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    lines = []
    threading.Thread(
        target=lambda: [lines.append(l.rstrip()) for l in bridge.stdout],
        daemon=True).start()

    os.set_blocking(master, False)
    raw = bytearray()

    def drain(seconds):
        t_end = time.time() + seconds
        while time.time() < t_end:
            try:
                d = os.read(master, 4096)
                if d:
                    raw.extend(d)
            except (BlockingIOError, OSError):
                pass
            sim.tick()
            rclpy.spin_once(sim, timeout_sec=0.005)

    try:
        print("=" * 74)
        print(" 雷达断流保护测试（模拟车体断电导致雷达掉电）")
        print("=" * 74)

        # 阶段 1：一切正常
        drain(5.0)
        n1 = len(raw) // 15
        check("阶段1 正常时在发送", n1 > 20, f"发 {n1} 帧")

        # 阶段 2：雷达断流 2 秒（短抖动，应停发但【不】锁死）
        sim.emit_lidar = False
        # 断流后桥还有 max_lidar_gap(1.0s) 的"宽限期" —— 它无法在断流瞬间就知道。
        # 所以要先放掉宽限期再取基线，否则测到的是设计行为而不是故障。
        drain(1.6)
        mark = len(raw)
        drain(2.5)
        stopped = len(raw) == mark
        check("阶段2 断流后停止发送", stopped, f"宽限期后新增 {len(raw) - mark} 字节")
        check("阶段2 短断流未锁死",
              not any("LIDAR OUTAGE" in l for l in lines), "（应自动恢复）")

        # 阶段 3：恢复 → 应自动续发
        sim.emit_lidar = True
        drain(3.0)
        check("阶段3 恢复后自动续发", len(raw) > mark, f"发 {len(raw) // 15} 帧")

        # 阶段 4：长断流 8 秒（> lidar_relock_gap=5）→ 应【锁死】
        sim.emit_lidar = False
        drain(1.6)          # 放掉宽限期
        mark2 = len(raw)
        drain(8.0)
        latched = any("LIDAR OUTAGE" in l for l in lines)
        check("阶段4 长断流触发锁死", latched)
        check("阶段4 锁死期间不发字节", len(raw) == mark2, f"新增 {len(raw) - mark2} 字节")

        # 阶段 5：恢复数据也不该自动发（必须人工刷零点）
        sim.emit_lidar = True
        mark3 = len(raw)
        drain(4.0)
        check("阶段5 恢复后仍不发（需刷零点）", len(raw) == mark3,
              f"新增 {len(raw) - mark3} 字节")

        # 阶段 6：调 refresh_origin → 应解除并恢复
        cli = subprocess.run(
            ["ros2", "service", "call", "/serial_pose_bridge/refresh_origin",
             "std_srvs/srv/Trigger", "{}"],
            capture_output=True, text=True, timeout=15)
        ok_srv = "success=True" in cli.stdout
        check("阶段6 refresh_origin 成功", ok_srv,
              cli.stdout.strip().splitlines()[-1][:60] if cli.stdout else "")
        mark4 = len(raw)
        drain(3.0)
        check("阶段6 刷零点后恢复发送", len(raw) > mark4,
              f"新增 {len(raw) - mark4} 字节 = {(len(raw) - mark4) // 15} 帧")

    finally:
        bridge.kill()
        sim.destroy_node()
        rclpy.shutdown()

    print("=" * 74)
    passed = sum(1 for _, ok in results if ok)
    if passed == len(results):
        print(f" ✅ 全部通过（{passed}/{len(results)}）—— 雷达断流保护有效")
        print("=" * 74)
        return 0
    print(f" ❌ {len(results) - passed}/{len(results)} 项未通过")
    print("=" * 74)
    return 1


if __name__ == "__main__":
    sys.exit(main())
