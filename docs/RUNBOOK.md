# 实机运行规范

> 每次上机按同一套顺序走，每一步都有客观判据。
> 命令速查见 [`TERMINALS.md`](TERMINALS.md)；协议见 [`PROTOCOL.md`](PROTOCOL.md)；
> 遇到怪事先搜 [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md)。

```bash
REPO=/home/myler/mid360_pose_stream      # 换成你的实际路径
```

---

## 0. 上机前（30 秒）

```bash
cat /sys/class/net/<网口>/carrier     # 期望 1
ping -c3 192.168.1.102                # 期望通（雷达 IP 持久化，不用重配）
ls /dev/serial/by-id/                 # 期望有 STM32 那一项
```

⚠️ **别插错设备**：`ICWorkshop_PowerDebugger_Wireless_RX` 是无线调试探针，**不是** STM32。
认名字里有 `STM32` 的那个。

⚠️ **机器人摆放**：摆到你想当**零点**的位置和朝向上。
零点只在两处建立：① 启动时第一帧有效位姿（自动）② 你调 `refresh_origin` 时（手动）。

---

## 1. 两个终端

### 终端 1 —— 跑栈（**只起一套**）

```bash
# 推荐：一键脚本（自动硬件检查 + 拒绝重复实例 + 日志落盘）
bash $REPO/start.sh frame_rate:=60 send_rate:=60.0

# 等价的手工写法
cd $REPO
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch mid360_pose_bridge pose_stream.launch.py \
  frame_rate:=60 send_rate:=60.0 \
  2>&1 | tee logs/launch_$(date +%H%M%S).log
```

| 参数 | 作用 | 备注 |
|---|---|---|
| `frame_rate:=60` | **同时**驱动雷达 `publish_freq` 与 FAST-LIO 的 `preprocess.scan_rate` | 两者必须一致，故绑成一个参数。回退写 `10` |
| `send_rate:=60.0` | 串口发送频率 | 回退写 `20.0` |
| `pose_source_frame:=odom` | 位姿参考帧（默认） | 改成 `map` 才需要 AMCL |
| `auto_origin:=true` | 第一帧有效位姿自动锁零点（默认） | 开机即用 |
| `max_tf_age:=5.0` | 允许的位姿时间戳最大年龄 | 设小了会**静默**锁不上零点 |
| `rviz:=false` | 纯可视化，默认不启 | 跑在同一台机器上会占 CPU；要看时再开 |
| `serial_port:=...` | 换 STM32 板子时改这里 | 默认按 by-id |

### 终端 2 —— 看板（每 15 秒刷新）

```bash
cd $REPO
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 tools/send_ready.py --watch --period=15
```

---

## 2. 流程

### 步骤 ① 起栈 → 等 20 秒

**等 20 秒**是为了让 FAST-LIO 的启动瞬态过去。看板应显示：

```
[ OK ] 当前模式        : 位姿流 serial_pose_bridge（60 Hz 连续发位姿）
[ OK ] 前置2 串口被打开 : pidNNNNN
[ OK ] 前置3 零点已锁定 : ✓ 位姿流正在发送
[ OK ] 重复实例         : launch=1 串口桥=1 ...
[ OK ] /livox/lidar 60.0 Hz   /imu/data 200.1 Hz   /odom 59.9 Hz
[ OK ] 位姿流发送频率   :  60.0 Hz（配置 send_rate=60）
[ OK ] 位姿源滞后       : odom->base_link    10 ms  ← 闭环可用
[ OK ] TF 链            : odom->livox_frame, base_link->base_footprint 都在
```

**三个必要条件**（缺一个就不发）：

| # | 条件 | 不满足的表现 |
|---|---|---|
| 1 | 串口被打开 | 看板 `前置2` FAIL；桥每 2 秒自动重连 |
| 2 | 零点已锁定 | 看板 `前置3` FAIL |
| 3 | 位姿源可用（TF 存在且年龄 < `max_tf_age`） | 看板 `位姿源滞后` FAIL |

### 步骤 ② 需要时刷新零点

**开机不需要刷**（`auto_origin` 已自动锁）。只在下面情况刷：

| 情况 | 不刷的后果 | 机器人摆在哪 |
|---|---|---|
| 搬动 / 推动 / 抬起了机器人 | 里程计跟着变，MCU 不知道 → 两边错开 | 摆到期望零点 |
| **NUC 或 launch 重启过**（MCU 没重启） | **`odom` 从 0 重算**，两边错开 → MCU 以为机器人瞬移 | 摆回原零点 |
| 里程计漂移积累多了 | 读数整体偏 | 摆回已知参考点 |

**操作**：

```bash
# 1) 先看一眼刷新前的读数（好对比）
grep -a "last_tx=" $REPO/logs/launch_*.log | tail -1

# 2) 确认机器人【已静止】、已摆到期望的零点位置和朝向

# 3) 刷新
bash $REPO/tools/refresh_origin.sh
```

**成功判据（两处都要看）**：

```
① 命令输出:  success=True  且含  zero frame sent
② 看板事件:  [HH:MM:SS] TX pose x=0.0000m y=0.0000m yaw=0.0000rad hex=aa 55 00 ... 00 ff
```

之后几秒的 `last_tx` 应贴着 `(0,0,0)`。

**失败处理**：

| 现象 | 原因 | 处理 |
|---|---|---|
| `success=False`，`cannot refresh origin: ...` | 位姿源不可用 | 修位姿源（雷达/`odom->base_link`）；或把 `max_tf_age` 提到 8.0 |
| `success=True` 但 `zero frame NOT sent` | 串口没开 | 检查 STM32；桥会自动重连，接好后**再刷一次** |
| 命令报找不到服务 | launch 没起 | 先起终端 1 |

**三个必须记住的点**：
1. **刷新时机器人必须静止** —— 否则读位姿和 MCU 收到零帧之间的位移会让两边差一截
2. **刷新只改 NUC 侧，靠那一帧 `(0,0,0)` 通知 MCU** —— 没发出去就会**静默错开**
3. **刷新是幂等的** —— 不确定就再刷一次，不会累积误差

---

## 3. 失败速查

| 看板 / 日志现象 | 原因 | 处理 |
|---|---|---|
| `当前模式: 未检测到任何串口桥` | launch 没起 | 起终端 1 |
| `前置2 串口设备: ... 不存在` | STM32 没插/插成调试探针 | 见第 0 节 |
| `前置2 串口被打开: 设备在，但没有进程打开它` | 打开失败或被占用 | 看日志 `Cannot open`；`exclusive=ON` 会挡住第二个进程 |
| `前置3 零点没锁上` | 位姿源不可用 | 看统计行 `waiting=` 字段 |
| `位姿源滞后` > 800 ms | FAST-LIO 积压没排空 | 见 `TROUBLESHOOTING.md` 第 1、2 节；确认 `frame_rate` 与 `scan_rate` 一致 |
| `位姿流发送频率` 偏低 | 零点没锁 / 串口没开 / 位姿源断 | 依上表逐项查；统计行 `waiting=` 直接给出原因 |
| `重复实例: launch=2` | 起了两套 | 清成一套 |
| `Serial pose write failed` | 串口掉了 | 修好线即可，桥会自动重连继续发 |
| 驱动报 `bind failed` / `Init lds lidar fail!` | 雷达网络或 SDK 库路径 | 见 `TROUBLESHOOTING.md` 第 4 节与 `patches/README.md` |

---

## 4. 铁律

1. **`frame_rate` 与 `scan_rate` 必须一致** —— 本仓库已绑成一个参数，别绕过它改 YAML
2. **`max_tf_age` 必须大于位姿源延迟** —— 设小了零点会**静默**锁不上
3. **NUC / launch 重启后必须重锁零点**（或两边一起重启）
4. **只起一套 launch**，串口 writer 只有一个
5. **RViz 别和 launch 绑在一起** —— 它挂掉会带走整套；用 `rviz:=true` 或单独起都行，但知道这个风险
6. **刷新零点后确认 `zero frame sent`** —— 否则两边静默错开

---

## 5. NUC 侧能证明什么、不能证明什么

| 看板 `位姿流发送频率 60.0 Hz` + 日志 `errors=0` **证明了** | **不能**证明 |
|---|---|
| 每秒 60 帧、每帧 15 字节写进了串口 | MCU 固件读了它们 |
| 帧头与校验正确（构建时保证） | 字节被解析成 x/y/yaw |
| STM32 的 USB 设备栈活着 | 底盘用这个位姿做了闭环 |

**协议没有 ACK 回传**，所以"电控是否收到"只能读 MCU 侧调试变量 —— 见 `PROTOCOL.md` 第 4 节。

---

## 6. 相关文件

| 文件 | 用途 |
|---|---|
| `tools/send_ready.py` | 看板 + 发送就绪自检（含位姿源滞后判据） |
| `tools/refresh_origin.sh` | 刷新零点 |
| `verify.sh` | 离线回归测试（不需要硬件） |
| `tools/offline_*_test.py` | 上面两个测试的实现 |
| `tools/fake_odom_tf.py` | 假 TF，手动调试用 |
| `docs/PROTOCOL.md` | 帧格式 + 电控侧核对 |
| `docs/TROUBLESHOOTING.md` | 踩过的坑 |

---

## 7. ✅ 2026-10-14 实机验证结果（六项判据全过）

```
[ OK ] 位姿流发送频率 :  60.1 Hz（配置 send_rate=60）
[ OK ] 逐级时间戳     : /imu/data=1ms  /odom=9ms  TF odom->base_link=13ms   ✓ 各级接近
[ OK ] 漂移(TF,8s)   : 位移 0.010 m  平均 0.00 m/s  瞬时峰 0.09 m/s   ✓
[ OK ] 位姿源滞后     : odom->base_link     9 ms  ← 闭环可用
[ OK ] /imu/data 200.1 Hz   /odom 60.0 Hz
[ OK ] No Effective Points = 0 次（静置 35 秒漂移 0.0027 m）
```

| # | 判据 | 通过标准 | 修复前（失败那次）|
|---|---|---|---|
| 1 | 静置漂移 | 60 s < 0.05 m | 2720 m / 16.6 s ✗ |
| 2 | 逐级时间戳 | 各级差 < 30 ms | 17 ms vs 1445 ms ✗ |
| 3 | TF 滞后 | < 100 ms | 1445 ms ✗ |
| 4 | 频率 | = frame_rate / 200 / = send_rate | 正常 |
| 5 | **`No Effective Points`** | **必须 0 次** | **1016 次** ✗ |
| 6 | 发散看门狗 | 未锁死 | 无此防线 |

**第 5 项是根因指标**：旧配置（`lidar_type=4` 用 yaw 猜时间）会在开机几秒就开始刷它，
然后必然发散 —— 详见 `TROUBLESHOOTING.md` 第 10 节。

## 8. 环境备忘

- 雷达网口 IP **持久化**（NetworkManager 静态配置）：`192.168.1.50/24`
- 零点**只存在桥的内存里**，launch 重启即丢（`auto_origin` 会重锁一次）
- `odom` 也**从 0 重新开始** —— 重启后两边是否对齐取决于机器人那时在不在零点
- 日志一律写 `$REPO/logs/`（`/tmp` 重启会清空）
- `ROS_DOMAIN_ID` 默认 0；脚本里要显式 `export`
