#!/bin/bash
# 刷新位姿流的零点（"让底盘认为它现在在原点"）。
#
# 做三件事：
#   1. 把机器人【当前位姿】记成新的零点
#   2. 立刻发一帧 (0,0,0) 给 STM32，让它同步归零
#   3. 之后继续按新零点以【当前 send_rate】发相对位姿（现为 60 Hz）
#
# 用法:
#   bash tools/refresh_origin.sh
#
# 两个服务都试：位姿流桥的 refresh_origin 优先，退回旧目标桥的 set_origin。
# （本脚本只读参数并调用服务，不发运动指令。）

set -o pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
source install/setup.bash

POSE_SRV="/serial_pose_bridge/refresh_origin"
GOAL_SRV="/serial_goal_bridge/set_origin"

echo "======================================================================"
echo " 刷新零点 / Refreshing pose-stream origin"
echo "======================================================================"

# ⚠️ 不要用 `ros2 node list` 判断节点在不在 —— ros2cli daemon 会卡住并返回空列表，
#    导致误判"没有找到 bridge"（2026-10-13 真实踩过）。
#    改成用 `ros2 service list` 探测（更可靠），并且即使探测不到也【照样直接调用】，
#    让真实的错误信息说话，而不是被探测逻辑挡在门外。
services="$(timeout 15 ros2 service list 2>/dev/null)"
if grep -qx "$POSE_SRV" <<<"$services"; then
  echo "  使用位姿流桥: $POSE_SRV"
  TARGET="$POSE_SRV"
elif grep -qx "$GOAL_SRV" <<<"$services"; then
  echo "  位姿流桥不在，退回旧目标桥: $GOAL_SRV"
  echo "  （注意：旧桥是【点击发目标】模式，刷的是 serial_origin，不发零帧）"
  TARGET="$GOAL_SRV"
else
  # 服务列表也可能为空/不全（daemon 卡住）→ 不据此拒绝，直接试调用，让错误信息说话
  echo "  服务列表里没看到 bridge（可能是 ros2cli daemon 卡住）。"
  echo "  仍然直接尝试调用 $POSE_SRV ..."
  TARGET="$POSE_SRV"
fi

echo "----------------------------------------------------------------------"
OUT="$(timeout 20 ros2 service call "$TARGET" std_srvs/srv/Trigger {} 2>&1)"
RC=$?
echo "$OUT" | sed 's/^/  /'
echo "----------------------------------------------------------------------"

if [ $RC -ne 0 ] || grep -qiE "could not|not found|does not exist|unavailable|Failed to" <<<"$OUT"; then
  echo "  ✗ 调用没成功。逐项查："
  echo "     · launch 起了吗？      python3 "$REPO/tools/ps_ros.py""
  echo "     · 服务在列表里吗？     ros2 service list | grep -i origin"
  echo "     · ros2 CLI 卡住了吗？  ros2 daemon stop   （不影响正在跑的节点）"
  echo "     · 位姿源可用吗？       看 launch 日志 serial pose TX 行的 waiting= 字段"
  exit 1
fi

if ! grep -q "success=True" <<<"$OUT"; then
  echo "  ✗ 服务返回 success=False，看上面 message 里的具体原因。"
  exit 1
fi

if grep -qi "NOT sent" <<<"$OUT"; then
  echo "  ⚠️ 零点在 NUC 侧刷新了，但【零帧没发出去】—— 底盘不会跟着归零，两边会静默错开！"
  echo "     先修串口，然后【再刷一次】。"
  exit 1
fi

echo "  ✓ 刷新成功（zero frame sent）。约定：底盘现在应认为【当前位置 = 原点】。"
echo
echo "  再去 launch 日志确认这三样（缺一不可）："
echo "    1) ORIGIN_LOCKED reason=refresh_service x0=.. y0=.. yaw0=.."
echo "    2) TX pose x=0.0000m y=0.0000m yaw=0.0000rad hex=aa 55 00 ... 00 ff [origin zero frame]"
echo "    3) 统计行里 locks +1、zero_frames +1、last_tx≈(0,0,0)"
echo
echo "  电控侧对应：应收到一帧 aa 55 00 ... 00 ff，且 g_pl_dbg_x/y/yaw 变成 0。"
echo "======================================================================"
