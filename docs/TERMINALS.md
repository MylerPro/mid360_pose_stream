# 终端命令速查

> 一页纸，所有命令可直接粘贴。`docs/RUNBOOK.md` 讲"为什么"，这里只讲"敲什么"。

```bash
REPO=/home/myler/mid360_pose_stream      # 换成你的实际路径
NET=enp89s0                              # 连雷达的网口
```

---

## 【A】上机前（30 秒）

```bash
cat /sys/class/net/$NET/carrier           # 期望 1
ping -c3 192.168.1.102                    # 期望通
ls /dev/serial/by-id/                     # 期望 usb-STMicroelectronics_STM32_...-if00
```

---

## 【B】终端 1 —— 起栈（**只起一套**）

```bash
# 推荐：一键脚本（硬件检查 + 拒绝重复实例 + 日志落盘）
bash $REPO/start.sh
bash $REPO/start.sh frame_rate:=10 send_rate:=20.0      # 传参透传给 ros2 launch

# 等价手工写法
cd $REPO
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch mid360_pose_bridge pose_stream.launch.py \
  frame_rate:=60 send_rate:=60.0 \
  2>&1 | tee logs/launch_$(date +%H%M%S).log
```

**常用变体**

```bash
... frame_rate:=60 send_rate:=60.0 rviz:=true       # 带可视化
... frame_rate:=30 send_rate:=30.0                  # 降到 30 Hz
... frame_rate:=10 send_rate:=20.0                  # 回退到 10 Hz
... pose_source_frame:=map                          # 改用地图定位（需装 AMCL）
... serial_port:=/dev/ttyACM1                       # 临时换串口
```

---

## 【C】终端 2 —— 看板

```bash
cd $REPO
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 tools/send_ready.py --watch --period=15     # 常驻，每 15 秒刷新
python3 tools/send_ready.py                         # 单次
```

**核心三行**

```
[ OK ] 位姿流发送频率 :  60.0 Hz（配置 send_rate=60）
[ OK ] 位姿源滞后     : odom->base_link    10 ms  ← 闭环可用（目标 <300 ms）
[ OK ] TF 链          : odom->livox_frame, base_link->base_footprint 都在
```

---

## 【D】刷新零点

```bash
bash $REPO/tools/refresh_origin.sh
```

等价写法：

```bash
ros2 service call /serial_pose_bridge/refresh_origin std_srvs/srv/Trigger {}
```

**刷新前先记下当前读数**：

```bash
grep -a "last_tx=" $REPO/logs/launch_*.log | tail -1
```

**成功判据**：`success=True` 且含 **`zero frame sent`**（不是 `NOT sent`）。

---

## 【E】查看运行状态

```bash
cd $REPO

# 发送统计（每 5 秒一行；60 Hz 时 frames 应每 5 秒 +300，errors=0）
grep -a "serial pose TX" logs/launch_*.log | tail -3

# 零点事件
grep -a "ORIGIN_LOCKED" logs/launch_*.log | tail -3

# 关键事件
grep -a -E "ORIGIN_LOCKED|zero frame|Opened /dev|Re-opened|Cannot open|write failed" \
     logs/launch_*.log | tail -10

# 实时频率（权威值用这个，别信 Python 看板的绝对值）
ros2 topic hz /livox/lidar                # 期望 = frame_rate（默认 60）
ros2 topic hz /imu/data                   # 期望 ~200
ros2 topic hz /odom                       # 期望 = frame_rate
ros2 topic hz /chassis/serial_tx_pose     # 期望 = send_rate

# 节点与发布者（发布者必须为 1）
ros2 node list | grep serial_pose_bridge
ros2 topic info /chassis/serial_tx_pose

# 串口被谁占用
for p in $(pgrep -f serial_pose_bridge); do ls -l /proc/$p/fd 2>/dev/null | grep ttyACM; done

# 关键参数
ros2 param get /serial_pose_bridge map_frame      # odom
ros2 param get /serial_pose_bridge send_rate      # 60.0
ros2 param get /serial_pose_bridge max_pose_age   # 5.0
ros2 param get /livox_mid360 publish_freq         # = frame_rate
ros2 param get /fastlio_mapping preprocess.scan_rate   # = frame_rate
```

---

## 【F】失败排查

```bash
# F-1 位姿源 TF 到底有没有（必须加 timeout，否则会一直等）
timeout 8 ros2 run tf2_ros tf2_echo odom base_link
timeout 8 ros2 run tf2_ros tf2_echo livox_frame base_link

# F-2 雷达链
cat /sys/class/net/$NET/carrier
ping -c3 192.168.1.102

# F-3 串口设备 + 独占锁
ls -l /dev/serial/by-id/
ls -l /proc/$(pgrep -f serial_pose_bridge | head -1)/fd 2>/dev/null | grep ttyACM

# F-4 单实例检查（都应为 1）
ps -eo cmd | grep -c "[r]os2 launch mid360_pose_bridge"
ps -eo cmd | grep -c "[s]erial_pose_bridge"
ps -eo cmd | grep -c "[f]astlio_mapping"
ps -eo cmd | grep -c "[l]ivox_ros_driver2"

# F-5 RViz 单独开（Fixed Frame 应为 odom）
source /opt/ros/humble/setup.bash && source $REPO/install/setup.bash
rviz2 -d $REPO/install/mid360_pose_bridge/share/mid360_pose_bridge/config/pose_stream.rviz

# F-6 综合监测（写 CSV，方便复盘）
python3 - <<'EOF'
# 这里只给最直接的一条：直接看三个 TF 和三个话题
EOF
ros2 topic hz /odom & sleep 3; kill %1

# F-7 一次看全部关键状态
for p in map_frame base_frame send_rate max_pose_age; do
  printf "%-14s " "$p"; ros2 param get /serial_pose_bridge $p 2>&1 | tail -1
done
```

---

## 【G】离线测试（**不需要任何硬件**）

```bash
cd $REPO
source /opt/ros/humble/setup.bash && source install/setup.bash

bash verify.sh                                    # 两套测试一起跑

python3 tools/offline_pose_stream_test.py         # 只跑单元级
python3 tools/offline_launch_integration_test.py  # 只跑 launch 集成

python3 tools/fake_odom_tf.py 50                  # 起假 TF 手动调试
```

---

## 【H】构建 / 依赖 / 补丁

```bash
cd $REPO

# 首次
sudo apt install python3-vcstool
mkdir -p src && vcs import src < dependencies.repos
bash patches/apply.sh
colcon build --symlink-install
source install/setup.bash

# 只改了本项目代码（不用重拉依赖）
colcon build --packages-select mid360_pose_bridge

# 补丁状态检查（反向能应用 = 已打过）
git -C src/FAST_LIO_ROS2 apply --reverse --check patches/fastlio_local_changes.patch && echo 已应用
git -C src/livox_ros_driver2 apply --reverse --check patches/livox_driver_local_changes.patch && echo 已应用

# 回退补丁
git -C src/FAST_LIO_ROS2 apply -R patches/fastlio_local_changes.patch
colcon build --packages-select fast_lio
```

---

## 【I】清理

```bash
cd $REPO

# 清进程（[x] 写法避免匹配到自己的 shell）
for p in $(ps -eo pid,cmd | awk '/[r]os2 launch mid360_pose_bridge|[m]id360_pose_bridge\/lib|[l]ivox_ros|[f]ast_lio\/lib/ {print $1}'); do kill -9 $p; done
sleep 2
rm -f /dev/shm/fastrtps_*

# 确认（应为 0）
ps -eo cmd | grep -cE "[m]id360_pose_bridge/lib|[f]ast_lio/lib|[l]ivox_ros"
```

---

## 常用组合

```bash
# ── 开机 ──
cat /sys/class/net/$NET/carrier && ls /dev/serial/by-id/    # 都对？
# 终端1: 【B】起栈        终端2: 【C】看板
# 等 20 秒 → 看板全绿、位姿源滞后 < 300 ms

# ── 搬动过机器人之后 ──
# 摆到期望零点、确认静止
bash $REPO/tools/refresh_origin.sh
# 看板确认出现 "zero frame" 那行

# ── 收工 ──
# 终端1 Ctrl-C，然后【I】清理
```

---

## 环境备忘

| 项 | 值 |
|---|---|
| NUC 网口 IP | `192.168.1.50/24`（静态，NetworkManager 持久化） |
| 雷达 IP | `192.168.1.102` |
| STM32 by-id | `/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_<序列号>-if00` |
| 串口参数 | `115200 8N1`，无流控，`exclusive=ON` |
| 帧格式 | `AA 55 \| x f32 LE \| y f32 LE \| yaw f32 LE \| sum8` = 15 字节 |
| 默认频率 | 雷达 60 Hz / 发送 60 Hz（`frame_rate` + `send_rate`） |
| `ROS_DOMAIN_ID` | 默认 0；脚本里要显式 `export` |
| 日志目录 | `$REPO/logs/`（`/tmp` 重启会清空） |
| 零点语义 | 只在桥的内存里，launch 重启即丢（`auto_origin` 会重锁一次） |
