#!/bin/bash
# 一键起栈（自动检查硬件 + 落盘日志）
#
#   bash start.sh                       # 默认 60 Hz
#   bash start.sh frame_rate:=10 send_rate:=20.0
#   bash start.sh rviz:=true
set -o pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "======================================================================"
echo " mid360_pose_stream 启动检查   $(date '+%F %T')"
echo "======================================================================"

# ── ① 硬件检查（只警告，不阻塞：位姿流桥会自动重连）──
NET="${NET:-enp89s0}"
warn=0
carr="$(cat /sys/class/net/$NET/carrier 2>/dev/null || echo '?')"
if [ "$carr" = "1" ]; then echo "  ✓ 雷达网口 $NET carrier=1"
else echo "  ✗ 雷达网口 $NET carrier=$carr —— 雷达没插/没上电，位姿源会起不来"; warn=1; fi

if timeout 4 ping -c1 -W2 192.168.1.102 >/dev/null 2>&1; then
  echo "  ✓ 雷达 192.168.1.102 可达"
else
  echo "  ✗ ping 不通 192.168.1.102"; warn=1
fi

if ls /dev/serial/by-id/ 2>/dev/null | grep -q STM32; then
  echo "  ✓ STM32 串口在线"
else
  echo "  ⚠ 没看到 STM32（可以先起，位姿流桥每 2 秒自动重连，插上就通）"
fi

# ── ② 防止两套实例抢雷达 UDP 和串口 ──
# 检测【真实的节点进程】而不是 launch 命令行 —— 后者会被自己的 shell 命令行
# 误匹配（脚本里出现 "ros2 launch" 字样就会被 ps 抓到）。
running=$(ps -eo cmd | grep -cE "[f]astlio_mapping|[l]ivox_ros_driver2_node|[s]erial_pose_bridge")
if [ "$running" -gt 0 ]; then
  echo
  echo "  ✗ 已有 $running 个本项目节点在跑！必须先停掉："
  echo "     两个 livox 驱动会抢雷达 UDP（雷达数据被瓜分 → 频率掉一半）"
  echo "     两个 bridge 会抢串口（帧交错 → MCU 侧 g_pl_frame_bad 上涨，极难反查）"
  echo
  echo "     停掉：  for p in \$(ps -eo pid,cmd | awk '/[f]astlio_mapping|[l]ivox_ros_driver2_node|[s]erial_pose_bridge/ {print \$1}'); do kill -9 \$p; done"
  echo "     或者：  Ctrl-C 原来那个终端"
  exit 1
fi
echo "  ✓ 没有其它实例在跑"

# ── ③ 环境 ──
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$REPO/install/setup.bash"
mkdir -p "$REPO/logs"
LOG="$REPO/logs/launch_$(date +%H%M%S).log"

echo "======================================================================"
echo " 启动中...  日志: $LOG"
echo " 另开一个终端看板:  python3 $REPO/tools/send_ready.py --watch --period=15"
echo " 需要归零时:        bash $REPO/tools/refresh_origin.sh"
echo "======================================================================"
echo

exec ros2 launch mid360_pose_bridge pose_stream.launch.py "$@" 2>&1 | tee "$LOG"
