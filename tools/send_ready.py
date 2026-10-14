#!/usr/bin/env python3
"""发送就绪自检 —— 一帧要真的离开串口，这些条件必须【同时】成立。

昨晚的教训：发送路径本身没问题，但三个前置条件从来没有同时满足过，
而且每一个失败都是【静默】的（不报错、不发送）。所以做成一条命令。

用法:
    python3 tools/send_ready.py [launch日志路径]

不带参数时默认找 logs/launch_*.log 里最新的一个。
"""
import glob
import math
import os
import re
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

# ── 路径解析：从本文件位置推出仓库根，保证 clone 到任何地方都能用 ──
_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_LOGS = os.path.join(_REPO, "logs")
_INSTALL_SETUP = os.path.join(_REPO, "install", "setup.bash")


SERIAL_BY_ID = ("/dev/serial/by-id/"
                "usb-STMicroelectronics_STM32_Virtual_ComPort_3154365A3233-if00")
OK, BAD, WARN, INFO = "[ OK ]", "[FAIL]", "[WARN]", "[info]"


def emit(s):
    print(s)
    sys.stdout.flush()


def serial_holders():
    hits = []
    try:
        real = os.path.realpath(SERIAL_BY_ID)
    except Exception:
        return hits
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            for fd in os.listdir(f"/proc/{pid}/fd"):
                link = os.readlink(f"/proc/{pid}/fd/{fd}")
                if link == real or "ttyACM" in link:
                    cmd = open(f"/proc/{pid}/cmdline", "rb").read().replace(
                        b"\0", b" ").decode(errors="replace")[:60]
                    hits.append((pid, link, cmd))
                    break
        except Exception:
            continue
    return hits


def count_proc(pat):
    import subprocess
    out = subprocess.run(["ps", "-eo", "cmd"], capture_output=True, text=True).stdout
    return sum(1 for ln in out.splitlines() if pat in ln and "awk" not in ln and "grep" not in ln)


def tail_has(path, pat):
    try:
        with open(path, "rb") as f:
            data = f.read()
        return pat.encode() in data
    except Exception:
        return False


def topic_hz(topic, seconds=6.0):
    """用 `ros2 topic hz` 测频率（绕开本节点的 Python 订阅瓶颈）。

    为什么要另起进程：本节点的 /livox/lidar 订阅在 60 Hz × 3360 点的
    PointCloud2 下【读不过来】，实测一直读到 0.0 Hz 而权威值是 60.015 Hz ✗
    —— 一个会误报 FAIL 的看板比没有更糟。

    为什么走【外部 timeout + 文件重定向】而不是 subprocess.run(capture_output)：
    在 rclpy 已经初始化的进程里抓管道输出实测拿不到东西（一直"测不到"），
    而落到文件里稳定可靠（实测 60.015 Hz）。
    """
    tmp = "/tmp/.dsh_topic_hz.txt"
    try:
        import subprocess
        subprocess.run(
            f"timeout -k 1 {int(seconds)} ros2 topic hz {topic} > {tmp} 2>&1",
            shell=True, timeout=seconds + 5, check=False)
        with open(tmp, "r", errors="replace") as fh:
            out = fh.read()
        rates = re.findall(r"average rate: ([\d.]+)", out)
        if rates:
            return float(rates[-1])
    except Exception:
        pass
    return None


def ros_param(node, name, cast=float, default=None):
    """读一个 ROS 参数（ros2 param get 会卡，所以统一带 timeout）。"""
    try:
        import subprocess
        r = subprocess.run(["ros2", "param", "get", node, name],
                           capture_output=True, text=True, timeout=6)
        m = re.search(r"([-\d.]+)\s*$", r.stdout.strip())
        if m:
            return cast(m.group(1))
    except Exception:
        pass
    return default


def tail_text(path, limit=4000):
    """读日志尾部若干字符（用于判断看门狗是否锁死）。"""
    if not path or not os.path.exists(path):
        return ""
    try:
        with open(path, "rb") as fh:
            fh.seek(0, os.SEEK_END)
            size = fh.tell()
            fh.seek(max(0, size - limit))
            return fh.read().decode("utf-8", errors="replace")
    except Exception:
        return ""


def recent_events(log, limit=7):
    """从 launch 日志里抽出关键业务事件，给看板当"事件流"用。"""
    pats = ("ORIGIN_LOCKED", "SET_ORIGIN_REJECTED", "GOAL_REJECTED", "GOAL_MAP",
            "TX_WRITE_OK", "TX_WRITE_ERROR", "SERIAL_OPEN_ERROR", "Opened /dev",
            "Re-opened", "Serial pose write failed", "serial pose TX:")
    if not log:
        return []
    try:
        with open(log, "rb") as f:
            data = f.read().decode("utf-8", "replace")
    except Exception:
        return []
    out = []
    for ln in data.splitlines():
        if any(p in ln for p in pats):
            m = re.search(r"\[(\d{10}\.\d+)\]", ln)
            ts = (time.strftime("%H:%M:%S", time.localtime(float(m.group(1))))
                  if m else "--:--:--")
            msg = ln.split("]: ", 1)[-1]
            # 只保留关键字段，别让整行 hex 把屏幕撑爆
            if "TX_WRITE_OK" in msg:
                # 15 字节 = 44 个字符（"aa 55 ... a1"），定长截取避免把后面的
                # "frames=1" 也吃进 hex 里
                m2 = re.search(r"hex=([0-9a-f ]{44})", msg)
                m3 = re.search(r"tx_relative_to_origin=\(([^)]*)\)", msg)
                msg = (f"TX_WRITE_OK bytes=15  x/y/yaw=({m3.group(1) if m3 else '?'})  "
                       f"hex={m2.group(1) if m2 else '?'}")
            if "serial pose TX:" in msg:
                # 5 秒一次的统计行，保留但压缩
                msg = msg.split("serial pose TX:", 1)[1].strip()
                msg = "serial pose TX:" + msg[:120]
            out.append((ts, msg[:150]))
    return out[-limit:]


def measure_tf_lag(src, base, seconds=3.0):
    """用一个【只订阅 /tf 的轻量节点】测 TF 年龄。

    为什么要单独一个节点：主 Probe 同时订阅 60 Hz 的大点云 + odom + imu + tx，
    单线程 Python 读不过来，它自己的 TF 缓冲会落后好几秒 ——
    实测因此把 10 ms 的真实滞后误报成 6199 ms。只订阅 /tf 就没有这个问题。
    """
    best = None
    probe = None
    try:
        probe = Node("tf_lag_probe")
        buf = Buffer()
        TransformListener(buf, probe)
        t0 = time.time()
        samples = []
        while time.time() - t0 < seconds:
            rclpy.spin_once(probe, timeout_sec=0.02)
            try:
                t = buf.lookup_transform(src, base, rclpy.time.Time())
                st = t.header.stamp
                samples.append(time.time() - (st.sec + st.nanosec * 1e-9))
            except Exception:
                pass
        if samples:
            samples.sort()
            best = samples[len(samples) // 2]
    except Exception:
        best = None
    finally:
        if probe is not None:
            try:
                probe.destroy_node()
            except Exception:
                pass
    return best


class Probe(Node):
    def __init__(self):
        super().__init__("send_ready")
        self.t = {}
        self.n = {"lidar": 0, "imu": 0, "scan": 0, "odom": 0, "tx": 0}
        self.odom = None
        self.tf = Buffer()
        TransformListener(self.tf, self)
        q = qos_profile_sensor_data
        # 轨迹与逐级时间戳：2026-10-14 事故暴露的两个缺失检查
        self.track = []            # [(t, x, y)] 来自 /odom 话题（可选，兜底）
        self.tf_track = []         # [(t, x, y)] 来自 TF —— 与桥同源，优先用它
        self.stamp_age = {}        # 话题 -> 最新一条消息的时间戳年龄

        def _aged(key, count=None):
            def cb(m):
                if count:
                    self._h(count)
                st = m.header.stamp
                self.stamp_age[key] = time.time() - (st.sec + st.nanosec * 1e-9)
            return cb

        self.create_subscription(PointCloud2, "/livox/lidar", _aged("/livox/lidar", "lidar"), q)
        self.create_subscription(Imu, "/imu/data", _aged("/imu/data", "imu"), q)
        self.create_subscription(LaserScan, "/scan", lambda m: self._h("scan"), q)
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        # 两种桥的"已发送"话题都数上（位姿流用 /chassis/serial_tx_pose）
        self.create_subscription(PoseStamped, "/chassis/serial_tx_pose",
                                 lambda m: self._h("tx"), 10)
        self.create_subscription(PoseStamped, "/serial_goal/tx_pose",
                                 lambda m: self._h("tx"), 10)

    def _h(self, k):
        self.n[k] += 1
        self.t[k] = time.time()

    def _odom(self, m):
        self.n["odom"] += 1
        now = time.time()
        self.t["odom"] = now
        self.odom = m.pose.pose.position
        st = m.header.stamp
        self.stamp_age["/odom"] = now - (st.sec + st.nanosec * 1e-9)
        self.track.append((now, self.odom.x, self.odom.y))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    watch = "--watch" in sys.argv[1:]

    if watch:
        # 看板模式：清屏后反复跑自检，用于第二个终端常驻盯着。
        # 每次都开新进程 → 频率统计不会累积，读数始终反映"最近这几秒"。
        import subprocess
        period = 20.0
        for a in sys.argv[1:]:
            if a.startswith("--period="):
                try:
                    period = max(float(a.split("=", 1)[1]), 3.0)
                except ValueError:
                    pass
        me = os.path.abspath(__file__)
        try:
            while True:
                os.system("clear")
                subprocess.run([sys.executable, me] + args)
                print(f"\n  ── 看板模式：每 {period:.0f} 秒刷新 ──  Ctrl-C 退出 ──\n")
                time.sleep(period)
        except KeyboardInterrupt:
            print("\n  看板已退出。")
        return

    log = args[0] if args else None
    if log is None:
        cands = sorted(glob.glob("logs/launch_*.log"), key=os.path.getmtime)
        log = cands[-1] if cands else None

    rclpy.init()
    node = Probe()
    W = 8.0
    t0 = time.time()
    next_tf_sample = 0.0
    while time.time() - t0 < W:
        rclpy.spin_once(node, timeout_sec=0.05)
        # 按 TF 采轨迹与时间戳年龄 —— 必须跟位姿流桥读【同一个源】(TF odom->base_link)。
        # 不能用 /odom 话题：两者不同源时，话题缺失会让检查静默消失（实测踩过：
        # 假 TF 只发 TF 不发 /odom，于是漂移检查一条都不打印）。
        now_m = time.time()
        if now_m >= next_tf_sample:
            next_tf_sample = now_m + 0.1
            try:
                tr = node.tf.lookup_transform("odom", "base_link", rclpy.time.Time())
                st = tr.header.stamp
                node.stamp_age["TF odom->base_link"] = now_m - (st.sec + st.nanosec * 1e-9)
                node.tf_track.append((now_m, tr.transform.translation.x,
                                      tr.transform.translation.y))
            except Exception:
                pass

    emit("=" * 76)
    emit(f"发送就绪自检   {time.strftime('%Y-%m-%d %H:%M:%S')}   采样 {W:.0f}s")
    emit("  ⓘ 本节点是单线程 Python，60 Hz 大点云下读不满 —— 下面几个频率是")
    emit("    【下限】。权威值请用: ros2 topic hz <话题>")
    emit("=" * 76)
    bad = 0

    # ---- 前置条件 1: dry_run ----
    # 主判据用日志 banner（serial_mode=...），因为它不受 ros2 CLI / daemon 抖动影响，
    # 而且是 bridge 自己在启动时打的、唯一权威的那一行。参数查询只作交叉验证。
    banner = None
    if log:
        try:
            with open(log, "rb") as f:
                _d = f.read().decode("utf-8", "replace")
            if "serial_mode=DRY_RUN" in _d:
                banner = "DRY_RUN"
            elif "serial_mode=SERIAL_TX" in _d:
                banner = "SERIAL_TX"
        except Exception:
            pass

    dry_param = None
    for _try in range(3):
        try:
            import subprocess
            r = subprocess.run(["ros2", "param", "get", "/serial_goal_bridge", "dry_run"],
                               capture_output=True, text=True, timeout=8)
            m = re.search(r"(True|False)", r.stdout)
            if m:
                dry_param = (m.group(1) == "True")
                break
        except Exception:
            pass
        time.sleep(0.4)

    # ---- 先判定当前是哪种桥：位姿流（新默认）还是点击发目标（旧）----
    names = []
    try:
        names = node.get_node_names()
    except Exception:
        pass
    pose_mode = any(n.strip("/") == "serial_pose_bridge" for n in names)
    goal_mode = any(n.strip("/") == "serial_goal_bridge" for n in names)

    _rate_txt = ""
    if pose_mode:
        try:
            import subprocess as _sp2
            _r = _sp2.run(["ros2", "param", "get", "/serial_pose_bridge", "send_rate"],
                          capture_output=True, text=True, timeout=6)
            _m = re.search(r"([\d.]+)", _r.stdout)
            if _m:
                _rate_txt = f"（{float(_m.group(1)):.0f} Hz 连续发位姿）"
        except Exception:
            pass
    mode_txt = (f"位姿流 serial_pose_bridge{_rate_txt or '（连续发位姿）'}" if pose_mode else
                "目标式 serial_goal_bridge（点一次发一帧）" if goal_mode else
                "未检测到任何串口桥")
    emit(f"  {OK if (pose_mode or goal_mode) else BAD} 当前模式        : {mode_txt}")
    if not (pose_mode or goal_mode):
        bad += 1

    if pose_mode:
        # 位姿流桥没有 dry_run 参数：它一旦起来就会开串口。
        # 真正要看的只有"串口有没有被它打开"和"零点锁没锁"。
        emit(f"  {INFO} 前置1 dry_run      : 位姿流桥无此参数（不需要 dry_run:=false）")
    elif goal_mode:
        if banner == "DRY_RUN" or dry_param is True:
            emit(f"  {BAD} 前置1 dry_run      : True ← 串口不会被打开，一帧都发不出去")
            emit(f"       （启动时漏了 dry_run:=false）")
            bad += 1
        elif banner == "SERIAL_TX" or dry_param is False:
            emit(f"  {OK} 前置1 dry_run      : False（会真写串口）")
        else:
            emit(f"  {WARN} 前置1 dry_run      : 判定不了（日志无 serial_mode 行，参数也查不到）")
            bad += 1

    # ---- 前置条件 2: 串口 ----
    dev = os.path.exists(SERIAL_BY_ID)
    hold = serial_holders()
    if not dev:
        emit(f"  {BAD} 前置2 串口设备     : {SERIAL_BY_ID} 不存在（STM32 没插或没上电）")
        bad += 1
    elif not hold:
        hint = ("位姿流桥会自动重连，检查 SERIAL_OPEN_ERROR" if pose_mode
                else "dry_run=True 或启动时打不开")
        emit(f"  {BAD} 前置2 串口被打开   : 设备在，但【没有进程】打开它 ← {hint}")
        bad += 1
    else:
        emit(f"  {OK} 前置2 串口被打开   : " + ", ".join(f"pid{p}" for p, _, _ in hold))

    # ---- 前置条件 3: 零点 ----
    has_goal_pub = node.count_publishers("/serial_goal/tx_pose")
    # count_publishers 偶尔在发现完成前返回 0；而"收到了 tx 数据"就等价于有 1 个发布者
    has_pose_pub = max(node.count_publishers("/chassis/serial_tx_pose"),
                       1 if node.n["tx"] > 0 else 0)
    locked_from_log = tail_has(log, "ORIGIN_LOCKED") if log else False
    # 位姿流模式下有个【更强的直接证据】：/chassis/serial_tx_pose 在发，
    # 就说明零点一定锁上了 —— 因为桥在 origin is None 时根本不发送。
    # 这样即使 launch 没加 tee 落盘（看板读不到日志），也不会误报。
    stream_live = pose_mode and node.n["tx"] > 0
    if pose_mode:
        # 位姿流模式【只信数据流】：日志可能属于上一次 launch（实测踩过 ——
        # 看板读到 1 小时前的 ORIGIN_LOCKED 而误报"已锁定"）。
        # 而"桥在发"是铁证：origin is None 时桥根本不发送。
        locked_from_log = stream_live
    if locked_from_log:
        label = "零点已锁定" if pose_mode else "serial_origin"
        if pose_mode and stream_live:
            emit(f"  {OK} 前置3 {label:11s}: ✓ 位姿流正在发送（桥在 origin 未锁时不会发，"
                 f"所以这等价于零点已锁）")
        else:
            emit(f"  {OK} 前置3 {label:11s}: 日志里已出现 ORIGIN_LOCKED"
                 f"（{os.path.basename(log) if log else '无日志'}）")
    else:
        if pose_mode:
            emit(f"  {BAD} 前置3 零点          : 没锁上（数据流也没在发）")
            emit(f"       （位姿源不可用。检查 odom->base_link 是否存在、"
                 f"TF 年龄是否超过 max_tf_age）")
        else:
            emit(f"  {BAD} 前置3 serial_origin: 日志里没有 ORIGIN_LOCKED ← 还没成功调 set_origin")
            emit(f"       （目标会被 GOAL_REJECTED，在打包之前就返回，什么都不发）")
        bad += 1

    # ---- 重复实例（会重复发帧 / 两个桥争同一个串口）----
    n_launch = count_proc("ros2 launch mid360_pose_bridge")
    n_bridge = count_proc("serial_goal_bridge") + count_proc("serial_pose_bridge")
    pubs = has_goal_pub + has_pose_pub
    if n_launch > 1 or n_bridge > 1 or pubs > 1:
        emit(f"  {BAD} 重复实例       : launch={n_launch} 串口桥={n_bridge} "
             f"tx发布者={pubs} ← 会重复发帧/争串口！")
        bad += 1
    else:
        emit(f"  {OK} 重复实例       : launch={n_launch} 串口桥={n_bridge} "
             f"位姿流发布者={has_pose_pub} 目标发布者={has_goal_pub}")

    # ---- 数据流 ----
    emit("  " + "-" * 72)
    # 期望频率随配置变：frame_rate 提到 60 后，lidar/odom 是 60 Hz；
    # /scan 只在启动 map_server+AMCL 时存在（nav2_localization:=true）。
    # 注意：本节点是单线程 Python，60 Hz 的大点云可能读不满 —— 绝对值会偏低，
    # 这里只判"有没有在流"，权威频率请用 `ros2 topic hz`。
    checks = [("imu", 50, "/imu/data"), ("odom", 60, "/odom")]
    if node.count_publishers("/scan") > 0:
        checks.append(("scan", 10, "/scan"))
    # /livox/lidar 本节点测不准（60 Hz × 3360 点读不过来），
    # 试过另起进程测但在这个已初始化 rclpy 的进程里拿不到输出（试了 3 种写法）。
    # 不再假装能测 —— 直接告诉操作员用权威命令，避免误报。
    emit(f"  [info] /livox/lidar  : 请用  ros2 topic hz /livox/lidar  （期望 ~frame_rate）")

    for k, want, name in checks:
        hz = node.n[k] / W
        if hz < want * 0.25:
            emit(f"  {BAD} {name:14s}: {hz:6.1f} Hz（期望 ~{want}）")
            bad += 1
        else:
            emit(f"  {OK} {name:14s}: {hz:6.1f} Hz")

    # ---- 位姿流：实际发送频率（这是新模式的核心指标）----
    if pose_mode:
        emit("  " + "-" * 72)
        tx_hz = node.n["tx"] / W
        # 目标频率取实际 send_rate 参数（协议本身不规定频率，是我们配的）
        want_hz = 60.0
        try:
            import subprocess as _sp3
            _r = _sp3.run(["ros2", "param", "get", "/serial_pose_bridge", "send_rate"],
                          capture_output=True, text=True, timeout=6)
            _m = re.search(r"([\d.]+)", _r.stdout)
            if _m:
                want_hz = float(_m.group(1))
        except Exception:
            pass
        # 本节点是单线程 Python，60 Hz 下可能读不满，所以阈值放到 0.5 倍
        if tx_hz >= want_hz * 0.5:
            emit(f"  {OK} 位姿流发送频率 : {tx_hz:5.1f} Hz（配置 send_rate={want_hz:.0f}）"
                 f"  ← /chassis/serial_tx_pose")
        else:
            emit(f"  {BAD} 位姿流发送频率 : {tx_hz:5.1f} Hz（配置 send_rate={want_hz:.0f}）"
                 f"  ← 偏低或没在发，看 launch 日志里 serial pose TX 行的 waiting= 原因")
            bad += 1

    # ---- /odom 是否 ≈ 0（决定 serial_origin 会不会等于 AMCL 原点）----
    if node.odom is not None:
        mag = math.hypot(node.odom.x, node.odom.y)
        if mag < 0.5:
            emit(f"  {OK} /odom 位置     : ({node.odom.x:+.3f},{node.odom.y:+.3f}) |xy|={mag:.3f} "
                 f"→ 机器人没动过，serial_origin 会等于 AMCL 原点 ✓")
        else:
            emit(f"  {WARN} /odom 位置     : |xy|={mag:.2f} m → 机器人动过了！"
                 f"现在锁 origin 会和底盘原点差这么多")
    else:
        emit(f"  {BAD} /odom 位置     : 无数据")
        bad += 1

    # ---- 逐级时间戳（定位"延迟出在哪一级"）----
    # 2026-10-14 事故里 /livox/lidar 只落后 17 ms 而 /odom 落后 1445 ms，
    # 说明延迟是 FAST-LIO 加的、不是驱动。没有这一项就只能猜。
    if pose_mode and node.stamp_age:
        keys = [k for k in ("/livox/lidar", "/imu/data", "/odom",
                            "TF odom->base_link") if k in node.stamp_age]
        if keys:
            worst = max(node.stamp_age[k] for k in keys)
            tag = OK if worst < 0.15 else (WARN if worst < 0.5 else BAD)
            emit(f"  {tag} 逐级时间戳   : "
                 + "  ".join(f"{k}={node.stamp_age[k] * 1000:.0f}ms" for k in keys)
                 + ("   ✓ 各级接近" if worst < 0.15
                    else "   ← 哪一级突然变大就是哪一级的问题"))
            if worst >= 0.5:
                bad += 1

    # ---- 静置漂移 / 等效速度（2026-10-14 事故的直接判据）----
    # 上次里程计漂到 1000+ 米（等效 163 m/s），当时没有任何检查发现它。
    # 判据用【物理可能的速度】：整车不可能超过 max_speed（与看门狗同源）。
    track = node.tf_track if len(node.tf_track) >= 20 else node.track
    src_txt = "TF" if track is node.tf_track else "/odom"
    if pose_mode and len(track) >= 20:
        t0s, x0s, y0s = track[0]
        t1s, x1s, y1s = track[-1]
        span = max(t1s - t0s, 1e-6)
        dist = math.hypot(x1s - x0s, y1s - y0s)
        eff = dist / span
        # 逐采样点的【瞬时】速度 —— 平均值会把单次大跳变稀释掉（8 秒平均
        # 会把一个 500 m/s 的单帧跳变摊成 0.13 m/s，正好绕过判据）。
        peak_inst = 0.0
        jump = 0.0
        for i in range(1, len(track)):
            j = math.hypot(track[i][1] - track[i - 1][1],
                           track[i][2] - track[i - 1][2])
            dt_i = max(track[i][0] - track[i - 1][0], 1e-6)
            jump = max(jump, j)
            peak_inst = max(peak_inst, j / dt_i)
        limit = ros_param("/serial_pose_bridge", "max_speed", float, 3.0) or 3.0
        info = (f"漂移({src_txt},{span:.0f}s): 位移 {dist:.3f} m  平均 {eff:.2f} m/s  "
                f"瞬时峰值 {peak_inst:.2f} m/s  单次最大跳变 {jump:.3f} m")
        if peak_inst > limit or eff > limit:
            emit(f"  {BAD} {info}   ★ 超过物理上限 {limit:.1f} m/s —— 里程计在发散！"
                 f"（看门狗应已停发，查 launch 日志里的 DIVERGENCE DETECTED）")
            bad += 1
        elif peak_inst > 1.0 or eff > 0.5:
            emit(f"  {WARN} {info}   （在移动 / 有跳变；静止时应位移 <0.05 m）")
        else:
            emit(f"  {OK} {info}   ✓ 静止或正常速度范围内")

    elif pose_mode:
        emit(f"  {WARN} 漂移         : 样本不足"
             f"（TF {len(node.tf_track)} 条 / /odom {len(node.track)} 条）"
             " ← 位姿源没起来？")

    # ---- 发散看门狗是否锁死 ----
    if pose_mode:
        txt = tail_text(log, 4000) if log else ""
        if "DIVERGENCE DETECTED" in txt or "WATCHDOG=LATCHED" in txt:
            emit(f"  {BAD} 发散看门狗   : ★ 已锁死停发 —— 位姿源发过散，"
                 f"底盘收不到字节；处理完调 refresh_origin 解除")
            bad += 1

    # ---- 位姿源滞后（位姿流模式下这是最关键的指标：闭环反馈延迟）----
    if pose_mode:
        src = "odom"
        try:
            import subprocess as _sp
            r = _sp.run(["ros2", "param", "get", "/serial_pose_bridge", "map_frame"],
                        capture_output=True, text=True, timeout=6)
            m = re.search(r"String value is:\s*(\S+)", r.stdout)
            if m:
                src = m.group(1)
        except Exception:
            pass
        lag = measure_tf_lag(src, "base_link")
        try:
            if lag is None:
                raise RuntimeError("no TF samples")
            if lag < 0.30:
                emit(f"  {OK} 位姿源滞后     : {src}->base_link {lag*1000:5.0f} ms  "
                     f"← 闭环可用（目标 <300 ms）")
            elif lag < 0.80:
                emit(f"  {WARN} 位姿源滞后   : {src}->base_link {lag*1000:5.0f} ms  "
                     f"← 偏高，闭环带宽受限")
            else:
                emit(f"  {BAD} 位姿源滞后     : {src}->base_link {lag*1000:5.0f} ms  "
                     f"← ★ 闭环会震荡（FAST-LIO 积压没排空）")
                emit(f"       修法：确认 fastlio_mid360_point_filter 补丁已编译，"
                     f"见 RUNBOOK 第 7 节")
                bad += 1
        except Exception as e:
            emit(f"  {BAD} 位姿源滞后     : {src}->base_link 测不到（{type(e).__name__}）")
            bad += 1

    # ---- TF: map 是否存在、年龄是否可接受 ----
    emit("  " + "-" * 72)
    # 位姿流模式下没有 map（AMCL 未启动），只检查里程计链
    pairs = ([("odom", "livox_frame"), ("base_link", "base_footprint")] if pose_mode
             else [("odom", "livox_frame"), ("map", "odom"), ("base_link", "base_footprint")])
    chain_ok = True
    for p, c in pairs:
        try:
            node.tf.lookup_transform(p, c, rclpy.time.Time())
        except Exception:
            emit(f"  {BAD} TF {p:12s}-> {c:16s}: 不可用")
            chain_ok = False
    if chain_ok:
        emit(f"  {OK} TF 链          : " + ", ".join(f"{p}->{c}" for p, c in pairs) + " 都在")

    age = None
    if pose_mode:
        # 位姿流模式用 odom->base_link，不需要 map 帧（那个 FAIL 是旧流程的判据）
        age = measure_tf_lag("odom", "base_link", seconds=2.0)
        if age is None:
            emit(f"  {BAD} odom->base_link : 测不到 → 位姿源没起来")
            bad += 1
    else:
        try:
            t = node.tf.lookup_transform("map", "base_link", rclpy.time.Time())
            s = t.header.stamp
            age = time.time() - (s.sec + s.nanosec * 1e-9)
        except Exception as e:
            emit(f"  {BAD} map->base_link : 查不到（{type(e).__name__}）"
                 f"→ 先在 RViz 点 2D Pose Estimate 建立 map 帧")
            bad += 1

    if age is not None:
        limit = 3.0
        try:
            import subprocess
            r = subprocess.run(["ros2", "param", "get", "/serial_goal_bridge", "max_tf_age"],
                               capture_output=True, text=True, timeout=8)
            m = re.search(r"([\d.]+)", r.stdout)
            if m:
                limit = float(m.group(1))
        except Exception:
            pass
        if abs(age) < limit:
            emit(f"  {OK} map->base_link : 年龄 {age:+.2f}s < max_tf_age {limit:.2f}s → set_origin 能过")
        else:
            emit(f"  {BAD} map->base_link : 年龄 {age:+.2f}s ≥ max_tf_age {limit:.2f}s "
                 f"→ set_origin 会被拒（位姿样本全被丢）")
            bad += 1

    # ---- 最近事件（原点锁定 / 目标 / 发送）----
    events = recent_events(log, 7)
    if events:
        emit("  " + "-" * 72)
        src_note = os.path.basename(log) if log else '无日志'
        if pose_mode:
            src_note += "  ⚠️ 位姿流模式请以「位姿流发送频率」和「位姿源滞后」为准；" \
                        "这里可能是旧 launch 的日志（当前 launch 若没加 tee 就不会落盘）"
        emit(f"  最近事件（{src_note}）:")
        for ts, msg in events:
            emit(f"    [{ts}] {msg}")

    # ---- 结论 ----
    emit("=" * 76)
    if bad == 0:
        if pose_mode:
            emit("  ✅ 全部就绪 —— 位姿流正在以目标频率发送；要归零就 bash tools/refresh_origin.sh")
        else:
            emit("  ✅ 全部就绪 —— 现在点 2D Goal Pose 就会真的发帧（看 TX_WRITE_OK）")
    else:
        if pose_mode:
            emit(f"  ❌ 有 {bad} 项不满足 —— 位姿流可能没在正常发送，先解决上面标 FAIL 的项")
        else:
            emit(f"  ❌ 有 {bad} 项不满足 —— 现在点目标【不会】发出任何字节，先解决上面标 FAIL 的项")
    emit("=" * 76)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
