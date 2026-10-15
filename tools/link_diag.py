#!/usr/bin/env python3
"""雷达数据链路诊断 —— 定位"雷达跑着跑着不报点"到底断在哪一层。

背景（2026-10-15）：MID360 运行中间歇性停止送数据（空洞 8s→6s→332s→45s→18s），
应用层完全没有报错。现场网卡是 **Intel I225-V**，这颗芯片有著名的"接收通路
间歇性停摆但链路状态仍显示正常"的缺陷。

光看 /livox/lidar 的频率【无法区分】是雷达没发、网卡没收、还是驱动没出：
    rx_packets 不涨  →  包根本没到主机    →  雷达 或 网卡/网线
    rx_packets 在涨  →  包到了但没变成话题 →  驱动 或 DDS

本工具每秒同时记录两侧，把断点定位到具体一层。

用法:
    # 正常采样（跟着实机跑，事后看 logs/link_diag_*.csv）
    python3 tools/link_diag.py --iface enp89s0

    # 只看雷达 UDP 端口有没有包进来（需要 root 或 CAP_NET_RAW）
    python3 tools/link_diag.py --iface enp89s0 --udp

输出:
    屏幕每 5 秒一行 + CSV（logs/link_diag_<时间>.csv），事后可逐秒对齐分析。
"""
import argparse
import csv
import os
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STATS = "/sys/class/net/{}/statistics/{}"
FIELDS = ["rx_packets", "rx_bytes", "rx_errors", "rx_dropped",
          "rx_missed_errors", "rx_fifo_errors", "tx_packets", "tx_errors"]


def read_stat(iface, name):
    try:
        with open(STATS.format(iface, name)) as fh:
            return int(fh.read().strip())
    except Exception:
        return None


def read_text(path):
    try:
        with open(path) as fh:
            return fh.read().strip()
    except Exception:
        return "?"


def ping_lidar(ip, timeout=1):
    """ping 雷达。

    这是区分两种断流的关键：
      网卡停摆      → ping 也失败（所有接收都死了）
      雷达数据流停了 → ping 还通（控制面活着，只是不送点云）
    少了这一项，rx_pps 掉 0 时无法判断是网卡还是雷达 ✗
    """
    try:
        r = subprocess.run(["ping", "-c", "1", "-W", str(timeout), ip],
                           capture_output=True, text=True, timeout=timeout + 3)
        return r.returncode == 0
    except Exception:
        return None


def lidar_udp_bytes(iface, seconds=1.0):
    """用 tcpdump 抓 1 秒，数雷达 UDP 端口进来的字节。

    这是【最硬】的证据：它绕开驱动、绕开 ROS、绕开 DDS，
    直接看网卡有没有把雷达的包交给内核。
    """
    try:
        r = subprocess.run(
            ["timeout", str(int(seconds)), "tcpdump", "-i", iface, "-n", "-q",
             "udp", "and", "portrange", "56300-56500"],
            capture_output=True, text=True, timeout=seconds + 4)
        # 统计 "N packets captured" 行
        for line in r.stderr.splitlines():
            if "packets captured" in line:
                return int(line.split()[0])
        return 0
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="enp89s0")
    ap.add_argument("--udp", action="store_true",
                    help="额外用 tcpdump 数雷达端口的包（需要 root）")
    ap.add_argument("--period", type=float, default=1.0)
    ap.add_argument("--lidar-ip", default="192.168.1.102")
    args = ap.parse_args()

    os.makedirs(os.path.join(REPO, "logs"), exist_ok=True)
    out = os.path.join(REPO, "logs", f"link_diag_{time.strftime('%H%M%S')}.csv")
    fh = open(out, "w", newline="")
    w = csv.writer(fh)
    w.writerow(["t", "wall", "carrier", "speed", "operstate"] + FIELDS
               + ["rx_pps", "rx_KBps", "udp_pkts", "ping", "note"])

    print(f"链路诊断启动  iface={args.iface}  CSV={out}")
    print("  判读（rx_pps 掉到 0 时看 ping）:")
    print("        ping 也不通 → 网卡/网线层（现场是 Intel I225-V，有已知停摆缺陷）")
    print("        ping 还通   → 雷达数据流停了（雷达自身或心跳），网络是好的")
    print("        rx_pps 正常但 /livox/lidar 没数据 → 驱动/DDS 层")
    print(f"  {'时刻':8s} {'状态':>6s} {'速率':>7s} {'rx_pps':>8s} {'rx_KB/s':>9s} "
          f"{'err':>5s} {'drop':>5s} {'miss':>5s} {'udp':>6s}  备注")
    if args.udp:
        print("  ⓘ --udp 已开：需要 root 才能用 tcpdump")

    prev = {k: read_stat(args.iface, k) for k in FIELDS}
    t0 = time.time()
    next_print = 0.0
    last_rx = None
    stall_start = None
    try:
        while True:
            time.sleep(args.period)
            now = time.time()
            cur = {k: read_stat(args.iface, k) for k in FIELDS}
            carrier = read_text(f"/sys/class/net/{args.iface}/carrier")
            speed = read_text(f"/sys/class/net/{args.iface}/speed")
            oper = read_text(f"/sys/class/net/{args.iface}/operstate")

            rx_pps = rx_kbps = None
            if prev["rx_packets"] is not None and cur["rx_packets"] is not None:
                rx_pps = (cur["rx_packets"] - prev["rx_packets"]) / args.period
                rx_kbps = (cur["rx_bytes"] - prev["rx_bytes"]) / args.period / 1024.0

            udp = lidar_udp_bytes(args.iface) if args.udp else None
            ping_ok = ping_lidar(args.lidar_ip)

            note = ""
            if rx_pps is not None:
                if rx_pps < 5 and last_rx is not None and last_rx >= 5:
                    stall_start = now
                    if ping_ok is False:
                        note = "★ 接收停摆 + ping 也不通"
                        verdict = "网卡/网线层（I225-V 停摆嫌疑最大）"
                    elif ping_ok is True:
                        note = "★ 接收停摆但 ping 通"
                        verdict = "雷达数据流停了（雷达自身或心跳丢失），网络是好的"
                    else:
                        note = "★ 接收停摆"
                        verdict = "未知（ping 也没结果）"
                    print(f"\n  ★★ {time.strftime('%T')} 接收通路停摆！"
                          f"（carrier={carrier} speed={speed} oper={oper} ping={ping_ok}）"
                          f"\n      → 判断: {verdict}\n", flush=True)
                elif stall_start and rx_pps >= 5:
                    print(f"\n  ✓ {time.strftime('%T')} 接收恢复，"
                          f"停摆 {now - stall_start:.1f} s\n", flush=True)
                    note = f"恢复(断了{now - stall_start:.1f}s)"
                    stall_start = None
                last_rx = rx_pps

            if now - t0 >= next_print:
                next_print = now - t0 + 5.0
                print(f"  {time.strftime('%T'):8s} {oper:>6s} {str(speed) + 'M':>7s} "
                      f"{rx_pps if rx_pps is not None else -1:8.0f} "
                      f"{rx_kbps if rx_kbps is not None else -1:9.1f} "
                      f"{cur['rx_errors']:5d} {cur['rx_dropped']:5d} "
                      f"{cur['rx_missed_errors']:5d} "
                      f"{udp if udp is not None else -1:6d} "
                      f"{('通' if ping_ok else '不通') if ping_ok is not None else '?':>4s}  "
                      f"{note}", flush=True)

            w.writerow([f"{now - t0:.1f}", time.strftime("%H:%M:%S"), carrier, speed,
                        oper] + [cur[k] for k in FIELDS]
                       + [rx_pps, rx_kbps, udp, ping_ok, note])
            fh.flush()
            prev = cur
    except KeyboardInterrupt:
        print(f"\n  已停止。CSV: {out}")
    finally:
        fh.close()


if __name__ == "__main__":
    sys.exit(main())
