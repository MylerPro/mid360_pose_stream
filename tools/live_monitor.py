#!/usr/bin/env python3
"""实机实时监测 —— 每 N 秒打一行状态，异常时打 [ALERT]，同时写 CSV 便于复盘。

只读：不发布任何话题，不碰串口。

用法:
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 tools/live_monitor.py            # 默认 3 秒一行
    python3 tools/live_monitor.py 2          # 2 秒一行
    python3 tools/live_monitor.py 3 /tmp/m.csv

监控内容:
  * 五个数据话题的频率是否掉零
  * /odom 的 xy 幅值（千米级发散）与 z 漂移
  * /cloud_registered 时间戳是否落在未来（H-1 合成时间失控）
  * TF 链是否完整
  * /scan 有效光束比例（突然接近 100% = 扫到地面）
  * 串口桥的发布者数量（>1 = 有重复节点，会重复发帧/争串口）
  * 事件行：两种模式都支持
      - 位姿流模式：/chassis/serial_tx_pose、/chassis/origin（20 Hz 位姿）
      - 目标式模式：/goal_pose、/serial_goal/tx_pose、/serial_goal/origin
"""
import os
import csv
import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

# 路径解析：从本文件位置推出仓库根
_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_LOGS = os.path.join(_REPO, "logs")

# 阈值（可按现场调整）
ODOM_XY_WARN = 100.0        # |xy| 超过就认为发散
ODOM_XY_ALERT = 500.0
ODOM_Z_WARN = 5.0           # odom 的 z 漂移
STAMP_FUTURE_ALERT = 0.5    # /cloud_registered 时间戳落在未来多少秒算失控
SCAN_FIN_HIGH = 0.95        # 有效光束比例上限
SCAN_FIN_LOW = 0.02
SILENT_AFTER = 3.0          # 超过这么久没数据算断流
# 期望频率。lidar/cloud/odom 现在是 60 Hz（frame_rate:=60）；
# imu 200 Hz（留足余量写成 50）；scan 只在启动 map_server+AMCL 时才存在，
# 见下面的"无发布者就不告警"逻辑。
EXPECTED = {"lidar": 60.0, "imu": 50.0, "scan": 10.0, "cloud": 60.0, "odom": 60.0}
# 话题 -> 话题名，用于检测"有没有发布者"（没有就不该告警）
TOPIC_OF = {"lidar": "/livox/lidar", "imu": "/imu/data", "scan": "/scan",
            "cloud": "/cloud_registered", "odom": "/odom"}


def emit(s):
    print(s)
    sys.stdout.flush()


class Monitor(Node):
    def __init__(self):
        super().__init__("live_monitor")
        self.t = {}
        self.n = {k: 0 for k in EXPECTED}
        self.last = {}
        self.scan_msg = None
        self.odom_msg = None
        self.cloud_stamp = None
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.alerts = set()

        q = qos_profile_sensor_data
        self.create_subscription(PointCloud2, "/livox/lidar", lambda m: self._hit("lidar", m), q)
        self.create_subscription(Imu, "/imu/data", lambda m: self._hit("imu", m), q)
        self.create_subscription(LaserScan, "/scan", self._scan, q)
        self.create_subscription(PointCloud2, "/cloud_registered", self._cloud, q)
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        # 事件话题
        self.create_subscription(PoseStamped, "/goal_pose", self._goal, 10)
        self.create_subscription(PoseStamped, "/serial_goal/tx_pose", self._tx, 10)
        self.create_subscription(PoseStamped, "/serial_goal/origin", self._origin, 10)
        self.create_subscription(PoseStamped, "/chassis/serial_tx_pose", self._tx, 10)
        self.create_subscription(PoseStamped, "/chassis/origin", self._origin, 10)

    # ---------- 回调 ----------
    def _hit(self, key, msg):
        self.n[key] += 1
        self.t[key] = time.time()
        self.last[key] = msg

    def _scan(self, m):
        self._hit("scan", m)
        self.scan_msg = m

    def _cloud(self, m):
        self._hit("cloud", m)
        s = m.header.stamp
        self.cloud_stamp = s.sec + s.nanosec * 1e-9

    def _odom(self, m):
        self._hit("odom", m)
        self.odom_msg = m

    def _goal(self, m):
        p = m.pose.position
        emit(f"  >>> GOAL_POSE 收到  frame={m.header.frame_id} x={p.x:.4f} y={p.y:.4f}")

    def _tx(self, m):
        p = m.pose.position
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z)
        emit(f"  <<< TX_POSE  发出  x={p.x:.4f} y={p.y:.4f} yaw={yaw:.4f}")

    def _origin(self, m):
        p = m.pose.position
        emit(f"  *** ORIGIN   锁定  x0={p.x:.4f} y0={p.y:.4f}")


def tf_probe(node):
    out = {}
    for parent, child in (("odom", "livox_frame"), ("map", "odom"),
                          ("livox_frame", "base_link"), ("base_link", "base_footprint"),
                          ("map", "base_link")):
        try:
            t = node.tf.lookup_transform(parent, child, rclpy.time.Time())
            tr = t.transform.translation
            out[f"{parent}->{child}"] = (tr.x, tr.y, tr.z)
        except Exception:
            out[f"{parent}->{child}"] = None
    return out


def main():
    interval = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    csv_path = sys.argv[2] if len(sys.argv) > 2 else f"/tmp/live_monitor_{time.strftime('%H%M%S')}.csv"
    rclpy.init()
    node = Monitor()
    emit("=" * 100)
    emit(f"实机实时监测  每 {interval:.0f}s 一行   CSV -> {csv_path}")
    emit("列含义: hz[lidar/imu/scan/cloud/odom]  pose[map坐标系下的odom xy/z]  tf  scanFin  txPub")
    emit("Ctrl-C 结束。出现 [ALERT] 就把那一行连同前后几行发给我。")
    emit("=" * 100)

    f = open(csv_path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "lidar_hz", "imu_hz", "scan_hz", "cloud_hz", "odom_hz",
                "x", "y", "z", "tf_ok", "scan_fin_ratio", "tx_pub", "alerts"])
    f.flush()

    t0 = time.time()
    prev = {k: 0 for k in EXPECTED}
    try:
        while True:
            end = time.time() + interval
            while time.time() < end:
                try:
                    rclpy.spin_once(node, timeout_sec=0.05)
                except (ExternalShutdownException, KeyboardInterrupt):
                    raise KeyboardInterrupt
                except Exception:
                    # 上下文被信号处理器关掉时 spin_once 会抛，按正常退出处理
                    raise KeyboardInterrupt
            now = time.time()
            dt = now - t0
            hz = {k: (node.n[k] - prev[k]) / dt for k in EXPECTED}
            prev = dict(node.n)
            t0 = now

            # ---- 断流检测 ----
            alerts = []
            for k, want in EXPECTED.items():
                last = node.t.get(k)
                if last is None:
                    # 只在"这个话题本该有发布者"时才告警。
                    # 例：nav2_localization:=false 时 /scan 不存在（pc2ls 没起），
                    # 那不是故障，不该刷告警。
                    if node.count_publishers(TOPIC_OF[k]) > 0:
                        alerts.append(f"{k} 从未收到数据")
                elif now - last > SILENT_AFTER and hz[k] < 0.05:
                    alerts.append(f"{k} 已 {now - last:.1f}s 无数据")
                elif node.t.get(k) and hz[k] < want * 0.4:
                    alerts.append(f"{k} 频率掉到 {hz[k]:.1f}Hz（期望 ~{want:.0f}）")

            # ---- odom 健康 ----
            x = y = z = float("nan")
            if node.odom_msg is not None:
                p = node.odom_msg.pose.pose.position
                x, y, z = p.x, p.y, p.z
                mag = math.hypot(x, y)
                if mag > ODOM_XY_ALERT:
                    alerts.append(f"/odom |xy|={mag:.1f}m 已严重发散")
                elif mag > ODOM_XY_WARN:
                    alerts.append(f"/odom |xy|={mag:.1f}m 偏大，怀疑发散")
                if abs(z) > ODOM_Z_WARN:
                    alerts.append(f"/odom z={z:+.2f}m 漂移偏大（会让 /scan 高度带滑动）")
                if node.odom_msg.child_frame_id != "livox_frame":
                    alerts.append(f"/odom child_frame_id={node.odom_msg.child_frame_id}（应为 livox_frame）")
            else:
                alerts.append("/odom 从未收到")

            # ---- H-1：合成时间失控 ----
            if node.cloud_stamp is not None:
                d = node.cloud_stamp - now
                if d > STAMP_FUTURE_ALERT:
                    alerts.append(f"/cloud_registered 时间戳在未来 {d:+.2f}s ← H-1 合成时间失控")

            # ---- /scan 质量 ----
            fin = float("nan")
            if node.scan_msg is not None and node.scan_msg.ranges:
                fin = sum(1 for r in node.scan_msg.ranges if math.isfinite(r)) / len(node.scan_msg.ranges)
                if fin >= SCAN_FIN_HIGH:
                    alerts.append(f"/scan 有效光束 {fin*100:.0f}% 过高 ← 可能扫到地面/车体")
                elif fin <= SCAN_FIN_LOW and node.t.get("scan"):
                    alerts.append(f"/scan 有效光束 {fin*100:.0f}% 过低 ← 高度带可能不匹配")

            # ---- TF ----
            chain = tf_probe(node)
            bad = [k for k, v in chain.items() if v is None]
            tf_ok = not bad
            if bad:
                alerts.append("TF 缺失: " + ", ".join(bad))
            for k in ("livox_frame->base_link", "base_link->base_footprint"):
                v = chain.get(k)
                if v is not None:
                    want = (-0.40, 0.0, -0.581) if k.startswith("livox") else (0.0, 0.0, -0.14)
                    if max(abs(a - b) for a, b in zip(v, want)) > 0.01:
                        alerts.append(f"{k}={v} 与期望 {want} 不符 ← URDF 被改过？")

            # ---- 重复节点 ----
            tx_pub = (node.count_publishers("/chassis/serial_tx_pose")
                      + node.count_publishers("/serial_goal/tx_pose"))
            if tx_pub > 1:
                alerts.append(f"串口桥有 {tx_pub} 个发布者 ← 重复节点，会重复发帧/争串口！")
            pose_bridge = any(n.strip("/") == "serial_pose_bridge"
                              for n in node.get_node_names())
            if pose_bridge:
                try:
                    _t = node.tf.lookup_transform("odom", "base_link", rclpy.time.Time())
                    _st = _t.header.stamp
                    _lag = time.time() - (_st.sec + _st.nanosec * 1e-9)
                    if _lag > 0.80:
                        alerts.append(
                            f"位姿源滞后 {_lag*1000:.0f} ms（>800ms）← 闭环会震荡；"
                            f"先查是不是两套栈在抢（看 Publisher count 和进程数）")
                except Exception:
                    pass

            # ---- 打印 ----
            ts = time.strftime("%H:%M:%S")
            emit(f"[{ts}] hz lidar={hz['lidar']:5.1f} imu={hz['imu']:6.1f} scan={hz['scan']:5.1f} "
                 f"cloud={hz['cloud']:5.1f} odom={hz['odom']:5.1f} | "
                 f"pose x={x:9.3f} y={y:8.3f} z={z:+6.3f} | "
                 f"tf={'OK ' if tf_ok else 'BAD'} | scanFin={fin*100 if fin == fin else -1:3.0f}% | txPub={tx_pub} | mode={'pose' if pose_bridge else 'goal'}")
            for a in alerts:
                if a not in node.alerts:
                    emit(f"  [ALERT {ts}] {a}")
            node.alerts = set(alerts)

            w.writerow([f"{now:.2f}"] + [f"{hz[k]:.2f}" for k in EXPECTED] +
                       [f"{x:.4f}", f"{y:.4f}", f"{z:.4f}", int(tf_ok),
                        f"{fin:.4f}", tx_pub, " | ".join(alerts)])
            f.flush()
    except KeyboardInterrupt:
        emit("\n监测结束。CSV: " + csv_path)
    finally:
        f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
