#!/usr/bin/env python3
"""离线验证【launch 集成】—— 不需要雷达、不需要 STM32。

做法：
  * 用 pty 造伪串口，通过 launch 的 serial_port 参数喂给 pose bridge
  * 本脚本发布假的 odom -> base_link TF（脚本化、量值已知）
  * 真正启动 ros2 launch（with_lidar:=true 但雷达没接，驱动会报错退出，
    不影响我们验证串口桥那一路）
  * 从 pty 另一头读帧，校验频率、零点、换算

用法:
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 tools/offline_launch_integration_test.py
"""
import math
import os
import pty
import signal
import struct
import subprocess
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
T_B, T_C, T_D, T_E = 3.0, 5.0, 7.0, 9.0
DURATION = 15.0
SEND_RATE = 60.0          # 必须与下面 launch 传的 send_rate 一致
LOG = "logs/offline_launch_test.log"


def emit(s):
    print(s)
    sys.stdout.flush()


class ScriptedOdom(Node):
    def __init__(self):
        super().__init__("fake_odom_pub")
        self.br = TransformBroadcaster(self)
        self.t0 = None
        self.create_timer(0.02, self.tick)

    @staticmethod
    def pose_at(t):
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


def parse_frames(buf):
    frames, bad = [], 0
    while len(buf) >= 15:
        if buf[0] != 0xAA or buf[1] != 0x55 or (sum(buf[:14]) & 0xFF) != buf[14]:
            bad += 1
            buf = buf[1:]
            continue
        frames.append(struct.unpack("<fff", buf[2:14]))
        buf = buf[15:]
    return frames, bad, buf


def main():
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    os.set_blocking(master, False)
    emit(f"伪串口: {slave_name}  → 通过 launch 的 serial_port 参数传入")

    rclpy.init()
    fake = ScriptedOdom()

    cmd = [
        "ros2", "launch", "mid360_pose_bridge", "pose_stream.launch.py",
        "pose_source_frame:=odom", "auto_origin:=true",
        "frame_rate:=60", f"send_rate:={SEND_RATE}",
        f"serial_port:={slave_name}", "rviz:=false",
        "max_tf_age:=5.0",
    ]
    logf = open(LOG, "wb")
    emit("启动 launch ...")
    proc = subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT,
                            preexec_fn=os.setsid)

    frames, raw, bad_total = [], b"", 0
    t_start = time.time()
    try:
        while time.time() - t_start < DURATION:
            rclpy.spin_once(fake, timeout_sec=0.004)
            try:
                data = os.read(master, 65536)
            except (BlockingIOError, OSError):
                data = b""
            if data:
                raw += data
                got, bad, raw = parse_frames(raw)
                bad_total += bad
                pt = (time.time() - fake.t0) if fake.t0 is not None else -1.0
                for f in got:
                    frames.append((pt, f[0], f[1], f[2]))
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            time.sleep(2.0)
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
        logf.close()

    log = ""
    try:
        log = open(LOG, "rb").read().decode("utf-8", "replace")
    except Exception:
        pass

    emit("")
    emit("=" * 78)
    emit("launch 集成验证结果")
    emit("=" * 78)
    ok = True

    started = "serial_pose_bridge.py" in log and "process started" in log
    emit(f"  pose bridge 被 launch 启动 : {'✓' if started else '★ 没有'}")
    ok &= started

    locked = "ORIGIN_LOCKED" in log
    emit(f"  零点自动锁定（ORIGIN_LOCKED）: {'✓' if locked else '★ 没有'}")
    if locked:
        for ln in log.splitlines():
            if "ORIGIN_LOCKED" in ln:
                emit(f"      {ln.split('] ', 1)[-1]}")
                break
    ok &= locked

    bad_excl = "both write to the same serial port" in log
    emit(f"  互斥检查未误触发          : {'✓' if not bad_excl else '★ 误触发'}")
    ok &= not bad_excl

    emit("")
    emit(f"  收到帧数                  : {len(frames)}")
    emit(f"  坏帧(帧头/校验)           : {bad_total}  {'✓' if bad_total == 0 else '★'}")
    ok &= bad_total == 0
    if len(frames) < 20:
        emit("  ★ 帧太少。launch 日志尾部：")
        for ln in log.strip().splitlines()[-15:]:
            emit("      " + ln)
        emit("=" * 78)
        return 1

    span = frames[-1][0] - frames[0][0]
    hz = (len(frames) - 1) / span if span > 0 else 0.0
    emit(f"  实测频率                  : {hz:.2f} Hz  (期望 {SEND_RATE:.0f})  "
         f"{'✓' if abs(hz - SEND_RATE) <= SEND_RATE * 0.1 else '★'}")
    ok &= abs(hz - SEND_RATE) <= SEND_RATE * 0.1

    def last_in(a, b):
        sel = [f for f in frames if a <= f[0] <= b]
        return sel[-1] if sel else None

    pa, pc, pe = last_in(1.5, T_B), last_in(6.0, T_D), last_in(13.0, DURATION)
    if pa:
        m = math.hypot(pa[1], pa[2])
        emit(f"  自动锁零点值              : ({pa[1]:+.6f}, {pa[2]:+.6f})  "
             f"|xy|={m*1000:.3f} mm  {'✓' if m < 0.001 else '★'}")
        ok &= m < 0.001
    if pc:
        e = math.hypot(pc[1] - LINEAR, pc[2])
        emit(f"  走 1.000m 后              : ({pc[1]:+.6f}, {pc[2]:+.6f})  "
             f"误差 {e*1000:.3f} mm  {'✓' if e < 0.001 else '★'}")
        ok &= e < 0.001
    if pe:
        e = math.hypot(pe[1] - LINEAR, pe[2])
        ddeg = math.degrees(pe[3] - TURN)
        emit(f"  再转 +90° 后              : ({pe[1]:+.6f}, {pe[2]:+.6f}, "
             f"{math.degrees(pe[3]):+.4f}°)  误差 {e*1000:.3f} mm / {ddeg:+.4f}°  "
             f"{'✓' if e < 0.001 and abs(ddeg) < 0.1 else '★'}")
        ok &= e < 0.001 and abs(ddeg) < 0.1

    emit("=" * 78)
    emit("  ✅ launch 集成验证通过 —— 参数接线、零点、60Hz 位姿流全对"
         if ok else "  ❌ 有项目未通过，见上面标 ★ 的项")
    emit(f"  （完整 launch 日志: {LOG}）")
    emit("=" * 78)
    return 0 if ok else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
