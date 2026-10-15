#!/bin/bash
# 离线回归测试 —— 不需要雷达、不需要 STM32。
#
# 用 pty 伪造串口 + 脚本化假 TF，校验帧格式/频率/量值精度/零点/刷新服务，
# 以及完整 launch 的参数接线。
#
# 用法:
#   source install/setup.bash
#   bash verify.sh
set -o pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "======================================================================"
echo " mid360_pose_stream 离线回归测试   $(date '+%F %T')"
echo "======================================================================"

# ── 前置检查 ──
fail=0
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "  ✗ 找不到 /opt/ros/humble/setup.bash —— 需要 ROS 2 Humble"; exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

if [ ! -f "$REPO/install/setup.bash" ]; then
  echo "  ✗ 还没构建。先跑: colcon build --symlink-install"; exit 1
fi
# shellcheck disable=SC1091
source "$REPO/install/setup.bash"

BRIDGE="$REPO/install/mid360_pose_bridge/lib/mid360_pose_bridge/serial_pose_bridge.py"
if [ ! -e "$BRIDGE" ]; then
  echo "  ✗ 找不到位姿流桥: $BRIDGE"; exit 1
fi
echo "  ✓ 环境就绪（ROS_DOMAIN_ID=$ROS_DOMAIN_ID）"
echo

# ── 测试 1：位姿流桥单元级 ──
echo "----------------------------------------------------------------------"
echo " 测试 1/3  位姿流桥（帧格式 / 频率 / 量值精度 / 零点 / 刷新服务）"
echo "----------------------------------------------------------------------"
timeout -k 5 90 python3 -u tools/offline_pose_stream_test.py
rc1=$?
echo

# ── 测试 2：launch 集成 ──
echo "----------------------------------------------------------------------"
echo " 测试 2/3  launch 集成（参数接线 / 零点 / 位姿流量值）"
echo "----------------------------------------------------------------------"
timeout -k 5 90 python3 -u tools/offline_launch_integration_test.py
rc2=$?

# ── 测试 3：雷达断流保护 ──
echo
echo "----------------------------------------------------------------------"
echo " 测试 3/3  雷达断流保护（模拟车体断电导致雷达掉电）"
echo "----------------------------------------------------------------------"
timeout -k 5 120 python3 -u tools/offline_lidar_outage_test.py
rc3=$?

# 清理测试残留（只针对本仓库起的进程）
for p in $(ps -eo pid,cmd | awk '/mid360_pose_bridge\/lib|mid360_pose_bridge\/launch|[f]ake_odom_tf/ && !/awk/ {print $1}'); do
  kill -9 "$p" 2>/dev/null
done
rm -f /dev/shm/fastrtps_* 2>/dev/null

echo
echo "======================================================================"
if [ $rc1 -eq 0 ] && [ $rc2 -eq 0 ] && [ $rc3 -eq 0 ]; then
  echo " ✅ 全部通过 —— 代码在无硬件条件下行为正确"
  echo "    下一步：接上雷达和 STM32，按 docs/TERMINALS.md 起实机"
  echo "======================================================================"
  exit 0
else
  echo " ❌ 有测试未通过（测试1 rc=$rc1，测试2 rc=$rc2，测试3 rc=$rc3）"
  echo "======================================================================"
  exit 1
fi
