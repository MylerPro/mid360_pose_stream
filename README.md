# mid360_pose_stream

把 **Livox MID360** 的实时位姿通过串口连续送给 **STM32**，用于底盘闭环定位。

ROS 2 Humble · 15 字节定长帧 · 默认 60 Hz · 实测位姿延迟 10 ms

---

## 它怎么工作

整条链路只有 4 个节点，依次是：

**① 雷达出数据**
MID360 用网线把点云和 IMU 数据以 UDP 发给主机。

**② `livox_ros_driver2` 收数据**
发布两个话题：`/livox/lidar`（点云，默认 60 Hz）和 `/imu/data`（IMU，200 Hz）。

**③ `fast_lio` 算位姿**
订阅上面两路，融合出里程计，发布：

- TF：`odom` → `livox_frame`
- 话题：`/odom`

**④ `robot_state_publisher` 补出车体**
读 URDF，把雷达帧接到车体上：

- TF：`livox_frame` → `base_link`（雷达在车上的安装位置）
- TF：`base_link` → `base_footprint`（车体中心到地面）

**⑤ `serial_pose_bridge` 发串口**（本仓库的核心，约 400 行）

1. 查 TF `odom` → `base_link`，得到机器人在里程计系下的位姿 `(x, y, yaw)`
2. 减去**零点**，算出相对位姿
3. 打包成 15 字节帧，按 `send_rate` 连续写串口

**⑥ STM32 使用**
收到 `x/y/yaw`，当作**机器人当前位姿**用于底盘闭环（不是导航目标，见 `docs/PROTOCOL.md`）。

> 关键点：**位姿源是纯里程计**，所以整条链路不需要地图、不需要 AMCL、
> 不需要 Nav2、也不需要人工点初始位姿 —— 雷达一上电就有位姿。

---

## 特点

| 特点 | 说明 |
|---|---|
| **不依赖地图/定位** | 位姿源是 FAST-LIO 里程计（融合 MID360 的 200 Hz IMU 与点云），开机即用 |
| **20 ~ 60 Hz 可配** | 实测 60 Hz 下位姿时间戳年龄中位 **10 ms**，闭环可用 |
| **零点可运行时刷新** | 调服务后立刻补发一帧 `(0,0,0)`，让 MCU 在同一瞬间归零 |
| **串口独占锁** | `flock` 保证同一时刻只有一个进程能持有串口，杜绝帧交错 |
| **串口自动重连** | 线掉了会自己重开，不用重启 launch |
| **全程可离线验证** | 伪串口 + 假 TF 的回归测试，精度到 1.000000 m / +90.0000°，不需要任何硬件 |

---

## 1. 硬件要求

| 项 | 说明 |
|---|---|
| 计算单元 | x86_64 或 arm64，Ubuntu 22.04 + ROS 2 Humble（本项目在 NUC11PAHi7 上验证） |
| 雷达 | Livox MID360，网线直连主机的独立网口 |
| 主机网口 IP | `192.168.1.50/24`，静态配置。必须与 `config/MID360_hardware_config.json` 里的 `host_net_info` 一致 |
| 雷达 IP | `192.168.1.102`（MID360 出厂默认） |
| STM32 | 通过 USB CDC 接入，`115200 8N1` |

⚠️ **别把无线调试探针当成 STM32**。串口路径按 `by-id` 写死为
`/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_<序列号>-if00`。
换板子要改 `launch/pose_stream.launch.py` 里的 `SERIAL_BY_ID`，或起栈时传
`serial_port:=/dev/ttyACM0`。

---

## 2. 软件依赖

### 2.1 ROS 2 与系统包

```bash
sudo apt install ros-humble-desktop ros-humble-tf2-ros python3-colcon-common-extensions
sudo apt install libpcl-dev libeigen3-dev          # FAST-LIO 需要
```

### 2.2 Livox-SDK2

**注意：这是驱动的系统级依赖，不是 ROS 包**，apt 装不到，必须手动编译安装：

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j"$(nproc)" && sudo make install && sudo ldconfig

# 确认装好了
ls /usr/local/lib/liblivox_sdk_shared.so
```

### 2.3 本仓库与第三方依赖

本仓库**不包含**第三方源码（原因见第 9 节许可证），用 vcstool 按钉死的 commit 拉取：

```bash
git clone git@github.com:MylerPro/mid360_pose_stream.git
cd mid360_pose_stream

sudo apt install python3-vcstool
mkdir -p src && vcs import src < dependencies.repos   # 拉 FAST_LIO_ROS2 与 livox_ros_driver2
bash patches/apply.sh                                 # 打必需补丁，不打编译不过
colcon build --symlink-install
source install/setup.bash
```

> `patches/apply.sh` 打的两个补丁是**必需的**，不是可选优化：
> 一个是 FAST-LIO 的编译修复 + 点云抽稀，一个是驱动的构建修复。
> 详见 [`patches/README.md`](patches/README.md)。

---

## 3. 快速开始

```bash
# ① 物理检查
cat /sys/class/net/<网口>/carrier     # 期望 1
ping -c3 192.168.1.102                # 期望通
ls /dev/serial/by-id/                 # 期望有 STM32 那一项

# ② 起栈（推荐用 start.sh：自动硬件检查 + 拒绝重复实例 + 日志落盘）
bash start.sh
#   回退到低频：  bash start.sh frame_rate:=10 send_rate:=20.0
#   要可视化：    bash start.sh rviz:=true

# ③ 看板（另开一个终端）—— 全绿就说明在正常发
python3 tools/send_ready.py --watch --period=15

# ④ 需要归零时
bash tools/refresh_origin.sh
```

**不用 `start.sh` 的等价写法**：

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch mid360_pose_bridge pose_stream.launch.py frame_rate:=60 send_rate:=60.0
```

**看板的关键三行**（全绿就对了）：

```
[ OK ] 位姿流发送频率 :  60.0 Hz（配置 send_rate=60）
[ OK ] 位姿源滞后     : odom->base_link    10 ms  ← 闭环可用（目标 <300 ms）
[ OK ] TF 链          : odom->livox_frame, base_link->base_footprint 都在
```

详细的逐步判据、失败速查、操作铁律见 [`docs/RUNBOOK.md`](docs/RUNBOOK.md)；
所有终端命令速查见 [`docs/TERMINALS.md`](docs/TERMINALS.md)。

---

## 4. 串口帧格式

**固定 15 字节**，无帧尾：

| 字节偏移 | 长度 | 字段 | 类型 | 说明 |
|---|---|---|---|---|
| 0 | 1 | 帧头 1 | `uint8` | 固定 `0xAA` |
| 1 | 1 | 帧头 2 | `uint8` | 固定 `0x55` |
| 2 | 4 | `x` | `float32` 小端 | 米，相对零点 |
| 6 | 4 | `y` | `float32` 小端 | 米，相对零点 |
| 10 | 4 | `yaw` | `float32` 小端 | 弧度，相对零点，已归一化到 `[-π, π]` |
| 14 | 1 | 校验和 | `uint8` | `sum(前 14 字节) & 0xFF` |

校验和的两种等价写法：

```c
// 写法 A
uint8_t s = 0; for (int i = 0; i < 14; i++) s += buf[i];

// 写法 B（等价，因 0xAA + 0x55 == 0xFF）
uint8_t s = (0xAA + 0x55 + sum_of_12_payload_bytes) & 0xFF;
```

**相对零点的换算**（`x0, y0, yaw0` 是锁定的零点）：

```
dx = x - x0 ,  dy = y - y0
x_rel   =  cos(yaw0)·dx + sin(yaw0)·dy
y_rel   = -sin(yaw0)·dx + cos(yaw0)·dy
yaw_rel =  normalize_angle(yaw - yaw0)      归一化到 [-π, π]，即最短转角
```

坐标系约定：`+x` 前、`+y` 左、`yaw` 逆时针为正。

**带宽**：60 Hz × 15 字节 = 900 B/s，115200 波特率下占用约 8%。

完整的协议说明、电控侧应对照的调试变量、排错阶梯见 [`docs/PROTOCOL.md`](docs/PROTOCOL.md)。

---

## 5. ⚠️ 两个频率必须一起改

`frame_rate` 这一个参数**同时**驱动两处：

| 驱动到哪 | 作用 |
|---|---|
| `livox_ros_driver2` 的 `publish_freq` | 雷达多久攒一帧 |
| FAST-LIO 的 `preprocess.scan_rate` | 用 yaw 合成每个点时间的角速度基准 |

**这两者不一致会出错。** FAST-LIO 的 `mid360_handler` 用
`omega_l = 0.361 * SCAN_RATE` 从 yaw 合成每个点的相对时间。
若雷达是 60 Hz（16.7 ms/帧）而 `scan_rate` 还是 10（基准 100 ms/帧），
合成时间会被拉长 6 倍，`lidar_end_time` 落到未来约 83 ms
→ 去畸变的时间基准错 → **位姿退化甚至发散**。

本仓库把两者绑成一个 `frame_rate` 参数，**从结构上不可能配错**。

### 帧率的反直觉结论

| 帧率 | 每帧点数 | 抽稀后 | 单帧耗时 vs 输入间隔 | 结果 |
|---|---|---|---|---|
| 10 Hz | ~20000 | ~6700 | ≈100 ms ≈ 100 ms | 积压排不掉 → 滞后 **1.4 s** |
| **60 Hz** | ~3300 | **~1100** | **≪ 16.7 ms** | **无积压 → 滞后 10 ms** |

原因：`publish_freq` 只是驱动侧的**聚合窗口**，雷达本身是 20 万点/秒的连续流，
所以**总点率不变**，只是每帧变轻。
**提高帧率反而解决了低帧率时的积压问题。**

---

## 6. 零点（origin）

发送的数值都是**相对零点**的。零点在两处建立：

1. **启动时自动**（`auto_origin:=true`，默认）—— 第一帧有效位姿即锁为零点，开机即用
2. **运行时手动** —— `bash tools/refresh_origin.sh`

刷新是**原子操作**：读当前位姿设为新零点 → **立刻发一帧 `(0,0,0)`** →
之后按新零点继续发。那一帧 `(0,0,0)` 的含义是"**你现在就在原点**"
（让 MCU 在同一瞬间归零），不是"目标是原点"。

| 什么时候要刷新 | 原因 |
|---|---|
| 搬动 / 推动 / 抬起了机器人 | 里程计跟着变了，MCU 不知道 |
| **NUC 或 launch 重启过**（MCU 没重启） | `odom` 会从 0 重算，两边零点错开 → MCU 以为机器人瞬移 |
| 里程计漂移积累多了 | 无回环校正 |

⚠️ **刷新时机器人必须静止**。
⚠️ **刷新后要确认那一帧真的发出去了**（输出里要有 `zero frame sent`），
否则 NUC 归零了而 MCU 没归零 —— **两边静默错开，且不报错**。

---

## 7. 离线验证（不需要硬件）

```bash
source install/setup.bash
bash verify.sh
```

两套测试都用 **pty 伪串口 + 脚本化假 TF**：

| 测试 | 校验内容 |
|---|---|
| `tools/offline_pose_stream_test.py` | 帧格式与校验和、发送频率、**位移精确到 1.000000 m**、**转角精确到 +90.0000°**、自动锁零点、`refresh_origin` 服务与零帧 |
| `tools/offline_launch_integration_test.py` | 走完整 launch：参数接线、零点、位姿流频率与量值 |

手动调试可以起一个假 TF：

```bash
python3 tools/fake_odom_tf.py 50     # 轨迹：静止→+1 m→静止→+90°→静止，12 秒循环
```

---

## 8. 目录结构

| 路径 | 说明 |
|---|---|
| `start.sh` | 一键起栈（硬件检查 + 拒绝重复实例 + 日志落盘） |
| `verify.sh` | 离线回归测试入口 |
| `dependencies.repos` | 第三方依赖清单，commit 已钉死 |
| `patches/` | 必需补丁 + `apply.sh` + 说明 |
| `docs/PROTOCOL.md` | 15 字节帧 + MCU 侧对接与调试变量 |
| `docs/RUNBOOK.md` | 实机运行规范：逐步判据 / 失败速查 / 操作铁律 |
| `docs/TERMINALS.md` | 所有终端命令速查 |
| `docs/TROUBLESHOOTING.md` | 踩过的坑与结论（最值钱的部分） |
| `tools/send_ready.py` | 看板：发送频率、位姿源滞后、TF 链、串口占用 |
| `tools/refresh_origin.sh` | 刷新零点 |
| `tools/fake_odom_tf.py` | 离线假 TF，手动调试用 |
| `tools/offline_pose_stream_test.py` | 离线单元级回归测试 |
| `tools/offline_launch_integration_test.py` | 离线 launch 集成测试 |
| `src/mid360_pose_bridge/launch/pose_stream.launch.py` | 唯一入口 |
| `src/mid360_pose_bridge/scripts/serial_pose_bridge.py` | 核心：零点 + 打包 + 发送 + 重连 |
| `src/mid360_pose_bridge/urdf/diffbot_mid360_hardware.urdf` | TF 链（根是 `livox_frame`） |
| `src/mid360_pose_bridge/config/MID360_hardware_config.json` | 雷达网络配置（改 IP 处） |
| `src/mid360_pose_bridge/config/fastlio_mid360.yaml` | FAST-LIO 参数 |
| `src/mid360_pose_bridge/config/pose_stream.rviz` | 可视化配置（Fixed Frame = `odom`） |

---

## 9. 许可证

本仓库自有代码见 [`LICENSE`](LICENSE)（MIT）。

第三方**不 vendor**，由 `dependencies.repos` 按钉死的 commit 拉取 + 补丁修改：

| 包 | 许可证 | 说明 |
|---|---|---|
| `Ericsii/FAST_LIO_ROS2` | **GPL-2.0** | ⚠️ **不要 vendor 进本仓库** —— 那会把整个项目拖成 GPL。用"钉死 commit + 补丁"引用，许可边界清晰 |
| `Livox-SDK/livox_ros_driver2` | MIT | 宽松 |
| `Livox-SDK/Livox-SDK2` | MIT | 宽松（系统级依赖） |
