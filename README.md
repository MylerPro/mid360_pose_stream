# mid360_pose_stream

把一个 **Livox MID360** 的实时位姿通过串口送给 **STM32**，用来做底盘闭环定位。

```
 MID360 ──UDP──> livox_ros_driver2 ──/livox/lidar + /imu/data──> FAST-LIO
                                                                   │
                                            TF: odom -> livox_frame│
                                                                   ▼
                                        robot_state_publisher (URDF)
                                        livox_frame -> base_link -> base_footprint
                                                                   │
                                          serial_pose_bridge ◄─────┘
                                          (相对"零点"的 x/y/yaw)
                                                   │ 15 字节帧 @ 60 Hz
                                                   ▼
                                          USB CDC ──> STM32
```

**特点**

- **不需要地图、不需要 AMCL、不需要 Nav2、不需要点初始位姿** —— 位姿源是纯里程计
  （FAST-LIO 融合 MID360 的 200 Hz IMU 与点云），雷达一起来就有位姿
- **20 ~ 60 Hz 可配**，实测 60 Hz 下 `odom` 时间戳年龄仅 **10 ms**（闭环可用）
- **零点可运行时刷新**：服务调用后立刻补发一帧 `(0,0,0)`，让 MCU 在同一瞬间归零
- **串口独占锁**（`flock`）：同一时刻只有一个进程能持有串口，杜绝帧交错
- **串口自动重连**：线掉了会自己重开，不用重启 launch
- **全程可离线验证**：假串口 + 假 TF 的回归测试，不需要任何硬件

---

## 1. 硬件要求

| 项 | 说明 |
|---|---|
| 计算单元 | x86_64 或 arm64，跑 Ubuntu 22.04 + ROS 2 Humble（本项目在 NUC11PAHi7 上验证） |
| 雷达 | Livox MID360，网线直连主机的独立网口 |
| 主机网口 IP | **`192.168.1.50/24`**（静态，要和 `config/MID360_hardware_config.json` 里的 `host_net_info` 一致） |
| 雷达 IP | **`192.168.1.102`**（默认） |
| STM32 | 通过 USB CDC 接入，`115200 8N1` |

> ⚠️ **别把无线调试探针当成 STM32**。本项目的串口路径是按 `by-id` 写死的：
> `/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_<序列号>-if00`
> 换板子要改 `launch/pose_stream.launch.py` 里的 `SERIAL_BY_ID`。

---

## 2. 软件依赖

### 2.1 ROS 2 与系统包

```bash
sudo apt install ros-humble-desktop ros-humble-tf2-ros python3-colcon-common-extensions
sudo apt install libpcl-dev libeigen3-dev          # FAST-LIO 需要
```

### 2.2 Livox-SDK2（**驱动的系统级依赖，不是 ROS 包**）

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j"$(nproc)" && sudo make install && sudo ldconfig
ls /usr/local/lib/liblivox_sdk_shared.so   # 应该存在
```

### 2.3 本仓库与第三方

```bash
git clone <本仓库> mid360_pose_stream && cd mid360_pose_stream
sudo apt install python3-vcstool
mkdir -p src && vcs import src < dependencies.repos    # 拉 FAST_LIO_ROS2 / livox_ros_driver2
bash patches/apply.sh                                  # 打必需补丁（不加编译不过，详见 patches/README.md）
colcon build --symlink-install
source install/setup.bash
```

---

## 3. 快速开始

```bash
# ① 物理检查
cat /sys/class/net/<你的网口>/carrier      # 1
ping -c3 192.168.1.102                     # 通
ls /dev/serial/by-id/                       # 有 STM32 那一项

# ② 起栈（推荐用 start.sh：自动做硬件检查 + 拒绝重复实例 + 日志落盘）
bash start.sh
#   回退到低频：  bash start.sh frame_rate:=10 send_rate:=20.0
#   要可视化：    bash start.sh rviz:=true

# 等价的手工写法
source install/setup.bash
ros2 launch mid360_pose_bridge pose_stream.launch.py

# ③ 看板（另开一个终端）—— 全绿就说明在正常发
python3 tools/send_ready.py --watch --period=15

# ④ 需要归零时
bash tools/refresh_origin.sh
```

看板关键三行：

```
[ OK ] 位姿流发送频率 :  60.0 Hz（配置 send_rate=60）
[ OK ] 位姿源滞后     : odom->base_link    10 ms  ← 闭环可用（目标 <300 ms）
[ OK ] TF 链          : odom->livox_frame, base_link->base_footprint 都在
```

---

## 4. 串口帧格式

```
AA 55 | x (float32 小端) | y (float32 小端) | yaw (float32 小端) | sum8
 2B   |      4B          |      4B          |       4B           |  1B
                                                共 15 字节
校验和 = sum(前 14 字节) & 0xFF
              ≡ (0xAA + 0x55 + sum(12 字节载荷)) & 0xFF
```

- `x`、`y` 单位 **米**，`yaw` 单位 **弧度**（已归一化到 `[-π, π]`，即最短转角）
- 数值是**相对零点**的：`x_rel = cos(y0)·dx + sin(y0)·dy`，`y_rel = -sin(y0)·dx + cos(y0)·dy`，
  `yaw_rel = normalize(yaw - y0)`（`+x` 前、`+y` 左、`yaw` 逆时针为正）
- 满速 60 Hz × 15 B = **900 B/s**，115200 波特率下占用约 8%

详见 [`docs/PROTOCOL.md`](docs/PROTOCOL.md)（含电控侧应对照的调试变量）。

---

## 5. ⚠️ 两个频率必须一起改

`frame_rate` 这一个参数**同时**驱动两处：

| 驱动到哪 | 作用 |
|---|---|
| `livox_ros_driver2` 的 `publish_freq` | 雷达多久攒一帧 |
| FAST-LIO 的 `preprocess.scan_rate` | 用 yaw 合成每个点时间的角速度基准 |

**这两者不一致会出错。** `mid360_handler` 用
`omega_l = 0.361 * SCAN_RATE`、`curvature = (yaw_fp - yaw_angle + 360k) / omega_l`
来合成每个点的相对时间。若雷达 60 Hz（16.7 ms/帧）而 `scan_rate` 还是 10
（基准 100 ms/帧），合成时间会被拉长 6 倍 → `lidar_end_time` 落到未来约 83 ms
→ **去畸变时间基准错 → 位姿退化甚至发散**。

所以本仓库把两者绑成一个 `frame_rate` 参数，**不可能配错**。

### 帧率的反直觉结论

| 帧率 | 每帧点数 | 抽稀后 | 单帧耗时 vs 输入间隔 | 结果 |
|---|---|---|---|---|
| 10 Hz | ~20000 | ~6700 | ≈100 ms ≈ 100 ms | 积压卡死 → 滞后 **1.4 s** |
| **60 Hz** | ~3300 | **~1100** | **≪ 16.7 ms** | **无积压 → 滞后 10 ms** |

`publish_freq` 只是驱动侧的聚合窗口，**总点率不变**（20 万点/秒）。
**提高帧率反而解决了低帧率时的积压问题。**

---

## 6. 零点（origin）

发送的数值是**相对零点**的。零点在两处建立：

1. **启动时自动**（`auto_origin:=true`）—— 第一帧有效位姿即锁为零点，开机即用
2. **运行时手动** —— `bash tools/refresh_origin.sh`

刷新是**原子操作**：读当前位姿设为新零点 → **立刻发一帧 `(0,0,0)`** →
之后按新零点继续发。那一帧 `(0,0,0)` 的约定是"**你现在就在原点**"
（让 MCU 在同一瞬间归零），不是"目标是原点"。

| 什么时候要刷 | 为什么 |
|---|---|
| 搬动 / 推动 / 抬起了机器人 | 里程计跟着变了，MCU 不知道 |
| **NUC 或 launch 重启过**（MCU 没重启） | **`odom` 会从 0 重新开始**，两边错开 → MCU 以为机器人瞬移 |
| 里程计漂移积累多了 | 无回环校正 |

⚠️ **刷新时机器人必须静止**；⚠️ **刷新后一定要确认那一帧真的发出去了**
（输出里要有 `zero frame sent`），否则 NUC 归零了而 MCU 没归零 —— **两边静默错开，不报错**。

---

## 7. 离线验证（不需要硬件）

```bash
source install/setup.bash
bash verify.sh
```

`verify.sh` 跑两套测试，都用 **pty 伪串口 + 脚本化假 TF**：

| 测试 | 校验内容 |
|---|---|
| `tools/offline_pose_stream_test.py` | 帧格式/校验和、发送频率、**位移比例精确到 1.000 m**、**转角精确到 +90.000°**、自动锁零点、`refresh_origin` 服务与零帧 |
| `tools/offline_launch_integration_test.py` | 走完整 launch：参数接线、零点、位姿流频率与量值 |

想手动调试可以起假 TF：
```bash
python3 tools/fake_odom_tf.py 50    # 轨迹：静止→+1 m→静止→+90°→静止，12 s 循环
```

---

## 8. 目录结构

```
├── README.md                  本文件
├── dependencies.repos         第三方依赖（commit 已钉死）
├── verify.sh                  离线回归测试入口
├── patches/                   必需补丁 + 说明 + apply.sh
├── docs/
│   ├── PROTOCOL.md            15 字节帧 + MCU 侧对接与调试变量
│   ├── RUNBOOK.md             实机运行规范（逐步判据 / 失败速查 / 铁律）
│   ├── TERMINALS.md           所有终端命令速查
│   └── TROUBLESHOOTING.md     踩过的坑与结论（★ 最值钱的部分）
├── tools/
│   ├── send_ready.py          看板（自动识别模式 + 位姿源滞后判据）
│   ├── refresh_origin.sh      刷新零点
│   ├── fake_odom_tf.py        离线假 TF
│   └── offline_*_test.py      离线回归测试
└── src/mid360_pose_bridge/
    ├── launch/pose_stream.launch.py   唯一入口
    ├── scripts/serial_pose_bridge.py  ★ 核心
    ├── urdf/diffbot_mid360_hardware.urdf
    └── config/
        ├── MID360_hardware_config.json   雷达网络配置（改 IP 处）
        ├── fastlio_mid360.yaml           FAST-LIO 参数（含 scan_rate）
        └── pose_stream.rviz              Fixed Frame = odom
```

---

## 9. 许可证

本仓库自有代码：见 `LICENSE`。

**第三方**（不 vendor，由 `dependencies.repos` 拉取）：

| 包 | 许可证 | 影响 |
|---|---|---|
| `Ericsii/FAST_LIO_ROS2` | **GPL-2.0** | ⚠️ **不要 vendor 进本仓库**，否则整个项目会被 GPL 覆盖。用钉死 commit + 补丁的方式引用，许可边界清晰 |
| `Livox-SDK/livox_ros_driver2` | MIT | 宽松 |
| `Livox-SDK/Livox-SDK2` | MIT | 宽松 |
