#!/usr/bin/env python3
"""离线验证位姿流桥 —— 不需要任何硬件。

做法：
  * 用 pty 造一对伪终端，slave 端当"串口"给桥用，master 端由本脚本读取
  * 本脚本发布一条【脚本化、量值精确已知】的 odom -> base_link TF
  * 于是可以逐帧校验：帧格式、频率、相对换算的【比例与角度】、零点、刷新服务

脚本化轨迹（量值精确已知，不依赖计时精度）：
    阶段 A  t∈[0,3)  静止在 (10.0, 5.0, yaw=+0.30)
    阶段 B  t∈[3,5)  沿航向【直线前进 1.000 m】，不转向
    阶段 C  t∈[5,7)  静止
    阶段 D  t∈[7,9)  原地【转 +90.000°】
    阶段 E  t∈[9,..) 静止

由此可以做的精确判据：
  * B 段前后位移的模长必须 = 1.000 m          （校验比例/尺度）
  * D 段前后 yaw 变化必须 = +90.000°          （校验角度与归一化）
  * 刷新服务后必须重新归零，并出现精确 (0,0,0) 帧

用法:
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 tools/offline_pose_stream_test.py
"""
import math
import os
import pty
import struct
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

# ── 路径解析：从本文件位置推出仓库根，保证 clone 到任何地方都能用 ──
_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_LOGS = os.path.join(_REPO, "logs")
_INSTALL_SETUP = os.path.join(_REPO, "install", "setup.bash")


BRIDGE = os.path.join(_REPO, "install", "mid360_pose_bridge", "lib",
                            "mid360_pose_bridge", "serial_pose_bridge.py")
X0, Y0, YAW0 = 10.0, 5.0, 0.30
LINEAR = 1.000          # 阶段 B 的直线位移 [m]
TURN = math.pi / 2.0    # 阶段 D 的转角 [rad]
SEND_RATE = 20.0
T_A, T_B, T_C, T_D, T_E = 0.0, 3.0, 5.0, 7.0, 9.0
DURATION = 14.5
T_REFRESH = 11.0        # 刷新必须发生在阶段E取值【之后】


def emit(s):
    print(s)
    sys.stdout.flush()


class ScriptedOdom(Node):
    """按脚本阶段发布量值精确已知的 odom -> base_link。"""

    def __init__(self):
        super().__init__("fake_odom_pub")
        self.br = TransformBroadcaster(self)
        self.t0 = None
        self.create_timer(0.02, self.tick)

    @staticmethod
    def pose_at(t):
        if t < T_B:                                   # 阶段 A：静止
            return X0, Y0, YAW0
        if t < T_C:                                   # 阶段 B：直线前进 1.000 m
            s = LINEAR * (t - T_B) / (T_C - T_B)
            return X0 + s * math.cos(YAW0), Y0 + s * math.sin(YAW0), YAW0
        if t < T_D:                                   # 阶段 C：静止
            return X0 + LINEAR * math.cos(YAW0), Y0 + LINEAR * math.sin(YAW0), YAW0
        if t < T_E:                                   # 阶段 D：原地转 +90°
            yaw = YAW0 + TURN * (t - T_D) / (T_E - T_D)
            return X0 + LINEAR * math.cos(YAW0), Y0 + LINEAR * math.sin(YAW0), yaw
        return (X0 + LINEAR * math.cos(YAW0), Y0 + LINEAR * math.sin(YAW0),
                YAW0 + TURN)                          # 阶段 E：静止

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
    frames, bad, junk = [], 0, 0
    while len(buf) >= 15:
        if buf[0] != 0xAA or buf[1] != 0x55:
            bad += 1
            buf = buf[1:]
            continue
        chunk = buf[:15]
        if (sum(chunk[:14]) & 0xFF) != chunk[14]:
            bad += 1
            buf = buf[1:]
            continue
        frames.append(struct.unpack("<fff", chunk[2:14]))
        buf = buf[15:]
    return frames, bad, buf


def settled(frames, t_a, t_b, phase):
    """取窗口内【最后一帧】。

    注意：不能取"最后 N 帧的中位数" —— 20 Hz 下最后 6 帧跨 250 ms，
    中位落在窗口结束前约 150 ms，会把运动段的取值截短约 7%（实测踩过）。
    各阶段末尾都是静止保持，所以最后一帧就是该阶段的稳定值。
    """
    sel = [f for f in frames if t_a <= f[0] <= t_b]
    if not sel:
        emit(f"      ({phase} 阶段没有帧)")
        return None
    last = sel[-1]
    return last[1], last[2], last[3]


def main():
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    os.set_blocking(master, False)
    emit(f"伪串口: {slave_name}")

    rclpy.init()
    fake = ScriptedOdom()
    cli = fake.create_client(Trigger, "/serial_pose_bridge/refresh_origin")

    proc = subprocess.Popen(
        [sys.executable, BRIDGE, "--ros-args",
         "-p", f"serial_port:={slave_name}",
         "-p", "map_frame:=odom", "-p", "base_frame:=base_link",
         "-p", f"send_rate:={SEND_RATE}", "-p", "auto_origin:=true",
         "-p", "log_frames:=false", "-p", "stats_period:=0.0",
         "-p", "max_pose_age:=5.0",
        # 离线测试没有雷达，关掉雷达断流看门狗（它由 offline_lidar_outage_test 单独测）
        "-p", "max_lidar_gap:=0.0", "-r", "__node:=serial_pose_bridge"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    emit(f"脚本轨迹: 静止→直线 {LINEAR} m→静止→转 {math.degrees(TURN):.1f}°→静止")
    frames = []          # (阶段时间, x_rel, y_rel, yaw_rel)
    raw = b""
    bad_total = 0
    t_start = time.time()
    refresh = {"done": False, "ok": None, "at": None}

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
            # 关键：阶段时间必须相对【假发布器自己的 t0】，
            # 否则阶段窗口和轨迹会差一个启动偏移（实测差 0.14s → 7% 误差）。
            pt = (time.time() - fake.t0) if fake.t0 is not None else -1.0
            for f in got:
                frames.append((pt, f[0], f[1], f[2]))

        if not refresh["done"] and fake.t0 is not None \
                and time.time() - fake.t0 > T_REFRESH:
            refresh["done"] = True
            if cli.wait_for_service(timeout_sec=5.0):
                fut = cli.call_async(Trigger.Request())
                end = time.time() + 5.0
                while not fut.done() and time.time() < end:
                    rclpy.spin_once(fake, timeout_sec=0.02)
                if fut.done() and fut.result() is not None:
                    refresh["ok"] = fut.result()
                    refresh["at"] = time.time() - fake.t0

    elapsed = time.time() - t_start
    try:
        proc.terminate()
        proc.wait(timeout=5)
    except Exception:
        proc.kill()
    try:
        out = proc.stdout.read() or ""
    except Exception:
        out = ""

    emit("")
    emit("=" * 78)
    emit(f"离线验证结果   采样 {elapsed:.1f}s")
    emit("=" * 78)
    ok = True

    emit(f"  收到帧数        : {len(frames)}")
    emit(f"  坏帧(帧头/校验) : {bad_total}  {'✓' if bad_total == 0 else '★ 有坏帧!'}")
    ok &= bad_total == 0
    if len(frames) < 20:
        emit("  ★ 帧太少，桥可能没在发。桥的日志尾部：")
        for ln in out.strip().splitlines()[-12:]:
            emit("      " + ln)
        emit("=" * 78)
        return 1

    span = frames[-1][0] - frames[0][0]
    hz = (len(frames) - 1) / span if span > 0 else 0.0
    emit(f"  实测发送频率    : {hz:.2f} Hz  (协议要求 {SEND_RATE})  "
         f"{'✓' if abs(hz - SEND_RATE) <= 1.5 else '★ 偏差大'}")
    ok &= abs(hz - SEND_RATE) <= 1.5

    f0 = frames[0]
    emit(f"  首帧(自动锁零点): ({f0[1]:+.4f}, {f0[2]:+.4f}, {f0[3]:+.4f})  "
         f"{'✓' if math.hypot(f0[1], f0[2]) < 0.02 else '★ 离零点太远'}")
    ok &= math.hypot(f0[1], f0[2]) < 0.02

    # ---- 阶段取值：只用各阶段末尾的【静止保持段】----
    # 这样取值完全不受 20 Hz 采样延迟影响，可以做精确比对。
    # 阶段 A 静止时自动锁零点 → 原点 = (X0, Y0, YAW0)。
    # 因此走 1.000 m + 转 +90° 后的相对值应精确等于 (1.0, 0.0, +90°)：
    #     dx = 1.0·cos(y0), dy = 1.0·sin(y0)
    #     x_rel =  cos·dx + sin·dy = 1.0
    #     y_rel = -sin·dx + cos·dy = 0.0      ← 与 y0 无关
    pa = settled(frames, 1.5, T_B, "A 静止")
    pc = settled(frames, 6.5, T_D, "C 静止")
    pe = settled(frames, 9.5, T_REFRESH, "E 静止")

    emit("")
    if pa:
        m = math.hypot(pa[0], pa[1])
        emit(f"  ① 自动锁零点    : 首段静止值 ({pa[0]:+.6f}, {pa[1]:+.6f}, {pa[2]:+.6f})  "
             f"|xy|={m*1000:.3f} mm  {'✓' if m < 0.001 else '★'}")
        ok &= m < 0.001
    else:
        ok = False

    if pc:
        ex, ey, ew = LINEAR, 0.0, 0.0
        dxm = (pc[0] - ex) * 1000.0
        dym = (pc[1] - ey) * 1000.0
        ddeg = math.degrees(pc[2] - ew)
        emit(f"  ② 走 1.000m 后  : 发出 ({pc[0]:+.6f}, {pc[1]:+.6f}, {pc[2]:+.6f})")
        emit(f"     期望 (1.000000, 0.000000, +0.000000) "
             f"→ 误差 {dxm:+.3f} mm / {dym:+.3f} mm / {ddeg:+.4f}°  "
             f"{'✓' if abs(dxm) < 1 and abs(dym) < 1 and abs(ddeg) < 0.1 else '★ 比例或方向不对'}")
        ok &= abs(dxm) < 1 and abs(dym) < 1 and abs(ddeg) < 0.1
    else:
        ok = False

    if pe:
        ex, ey, ew = LINEAR, 0.0, TURN
        dxm = (pe[0] - ex) * 1000.0
        dym = (pe[1] - ey) * 1000.0
        ddeg = math.degrees(pe[2] - ew)
        emit(f"  ③ 再转 +90° 后  : 发出 ({pe[0]:+.6f}, {pe[1]:+.6f}, {math.degrees(pe[2]):+.4f}°)")
        emit(f"     期望 (1.000000, 0.000000, +90.0000°) "
             f"→ 误差 {dxm:+.3f} mm / {dym:+.3f} mm / {ddeg:+.4f}°  "
             f"{'✓' if abs(dxm) < 1 and abs(dym) < 1 and abs(ddeg) < 0.1 else '★ 转角或归一化不对'}")
        ok &= abs(dxm) < 1 and abs(dym) < 1 and abs(ddeg) < 0.1
    else:
        ok = False

    emit("")
    if refresh["ok"] is None:
        emit("  refresh_origin  : ★ 服务没有响应")
        ok = False
    else:
        r = refresh["ok"]
        emit(f"  refresh_origin  : success={r.success}")
        emit(f"      '{r.message}'")
        ok &= bool(r.success)
        after = [f for f in frames if refresh["at"] and f[0] > refresh["at"] + 0.04]
        if after:
            m = max(math.hypot(f[1], f[2]) for f in after[:8])
            emit(f"      刷新后前 8 帧 |xy| 最大 {m*1000:.2f} mm  "
                 f"{'✓ 已重新归零' if m < 0.03 else '★ 没有归零'}")
            ok &= m < 0.03

    zero = [f for f in frames if f[1] == 0.0 and f[2] == 0.0 and f[3] == 0.0]
    emit(f"  精确 (0,0,0) 帧 : {len(zero)} 个  "
         f"{'✓' if len(zero) >= 2 else '★ 少于 2 个（首帧+刷新帧）'}")
    ok &= len(zero) >= 2

    emit("=" * 78)
    emit("  ✅ 离线验证全部通过：帧格式/20Hz/位移比例/转角/零点/刷新服务 全部正确"
         if ok else "  ❌ 有项目未通过，见上面标 ★ 的项")
    emit("=" * 78)
    return 0 if ok else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
