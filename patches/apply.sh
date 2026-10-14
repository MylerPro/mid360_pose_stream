#!/bin/bash
# 把两个必需补丁打到 dependencies.repos 拉下来的干净代码上。
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."

for spec in "src/FAST_LIO_ROS2:patches/fastlio_local_changes.patch" \
            "src/livox_ros_driver2:patches/livox_driver_local_changes.patch"; do
  dir="${spec%%:*}"; patch_file="${spec##*:}"
  if [ ! -d "$dir" ]; then
    echo "★ 找不到 $dir —— 先跑: mkdir -p src && vcs import src < dependencies.repos"
    exit 1
  fi
  if git -C "$dir" apply --reverse --check "$patch_file" 2>/dev/null; then
    echo "  · $dir 补丁已应用，跳过"
  else
    git -C "$dir" apply "$patch_file"
    echo "  ✓ 已应用 $patch_file 到 $dir"
  fi
done

echo
echo "补丁就绪。接下来："
echo "  1) 确认已装 Livox-SDK2（驱动依赖，非 ROS 包）："
echo "       ls /usr/local/lib/liblivox_sdk_shared.so"
echo "  2) colcon build --symlink-install"
echo "  3) bash verify.sh          # 离线回归测试，不需要硬件"
