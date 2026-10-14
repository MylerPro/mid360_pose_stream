# 串口协议（NUC → STM32）

## 1. 物理层

| 项 | 值 |
|---|---|
| 接口 | USB CDC（STM32 Virtual COM Port） |
| 设备路径 | `/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_<序列号>-if00` |
| 参数 | **115200 8N1**，无流控（`xonxoff=False, rtscts=False, dsrdtr=False`） |
| 独占 | `exclusive=True`（`flock`）—— 同一时刻只有一个进程能持有串口 |
| 发送频率 | 可配，默认 **60 Hz**（`send_rate`；协议本身不规定频率） |
| 带宽占用 | 60 Hz × 15 B = 900 B/s ≈ 115200 波特率的 **8%** |

> ⚠️ `exclusive` 是 **`flock` 建议性锁**，只对其他同样用 `flock` 的打开者有效 ——
> 普通 `open()`（`cat`、旧脚本）**仍然能打开**。所以仓库内所有 writer 都必须开这个标志。

## 2. 帧格式（固定 15 字节）

```
偏移  长度  字段      类型            说明
 0     1    header    0xAA
 1     1    header2   0x55
 2     4    x         float32 小端     米，相对零点
 6     4    y         float32 小端     米，相对零点
10     4    yaw       float32 小端    弧度，相对零点，已归一化到 [-π, π]
14     1    checksum  uint8           sum(前 14 字节) & 0xFF
```

### 校验和的两种等价写法

```c
// 写法 A（NUC 侧实现）
uint8_t sum = 0;
for (int i = 0; i < 14; i++) sum += buf[i];
uint8_t checksum = sum;

// 写法 B（等价，电控侧常用）
uint8_t checksum = (0xAA + 0x55 + sum_of_12_payload_bytes) & 0xFF;
```

两者数学等价（实测逐字节一致）。因为 `0xAA + 0x55 = 0xFF`，
**全零载荷的帧校验和恰好是 `0xFF`**。

### 参考帧（可直接和 `g_pl_last` 对比）

| 相对位姿 | 15 字节 |
|---|---|
| `(0, 0, 0)` ← 刷新零点时发的那帧 | `aa 55 00 00 00 00 00 00 00 00 00 00 00 00 ff` |
| `(1.0, 0, 0)` | `aa 55 00 00 80 3f 00 00 00 00 00 00 00 00 be` |
| `(0.1, 0, 0)` | `aa 55 cd cc cc 3d 00 00 00 00 00 00 00 00 a1` |
| `(1.234, 5.678, 0.901)` | `aa 55 b6 f3 9d 3f 2d b2 b5 40 f0 a7 66 3f 94` |

## 3. 数值语义

### 3.1 是"位姿"不是"目标"

帧里的 `x/y/yaw` 是**机器人当前的位姿**（相对零点），喂给
`R2_Pose_Set_Origin(x, y, yaw)` 即可 —— **不是导航目标**。

> 这个区别很关键：如果按"目标"解释，那么连续发送当前位姿等于一直告诉底盘
> "目标就是你所在的位置"，底盘永远不会动；而且刷新零点那一帧 `(0,0,0)`
> 会被理解成"目标是原点"→ 底盘会开回原点。

### 3.2 相对零点的换算

```
dx = x - x0 ;  dy = y - y0            （x0,y0,yaw0 = 锁定的零点）
x_rel  =  cos(yaw0)·dx + sin(yaw0)·dy
y_rel  = -sin(yaw0)·dx + cos(yaw0)·dy
yaw_rel = normalize_angle(yaw - yaw0)  （归一化到 [-π, π] = 最短转角）
```

坐标系约定：`+x` 前、`+y` 左、`yaw` 逆时针为正（右手系，z 轴向上）。

`yaw_rel` 归一化后给的是**最短转角**：例如零点在 −170°、机器人转到 +170°，
发出的是 **−20°**，而不是 +340°。

### 3.3 零点的建立与刷新

| 时机 | 行为 |
|---|---|
| 启动（`auto_origin:=true`） | 第一帧有效位姿自动锁为零点 → 开机即用 |
| 手动 `refresh_origin` 服务 | 读当前位姿设为新零点 → **立刻发一帧 `(0,0,0)`** → 之后按新零点发 |

`(0,0,0)` 那一帧的约定是「**你现在就在原点**」，让 MCU 在同一瞬间归零。
所以刷新是**两端同时归零**的原子操作。

⚠️ 如果那一帧没发出去（串口不可用），**NUC 归零了而 MCU 没归零 → 两边静默错开**。
NUC 侧会打印 `zero frame NOT sent`，**必须重刷**。

## 4. 电控侧核对清单

协议**没有 ACK 回传**（`pose_link.c` 只实现 RX），所以 NUC 侧
`port.write()` 返回 15 **只证明字节离开了主机**，不证明 MCU 解析了。
要闭环确认只能读 MCU 的调试变量：

| 变量 | 期望值（60 Hz 时） |
|---|---|
| `g_pl_rx_bytes` | 约 **+900 / 秒**（60 × 15） |
| `g_pl_frame_ok` | 约 **+60 / 秒** |
| `g_pl_frame_bad` | **不应增长**（增长 = 帧头/校验不符，通常是两个 writer 抢串口） |
| `g_pl_dbg_x/y/yaw` | 应等于帧内解码值**加上** `POSE_LINK_OFFSET_*`（当前默认全 0） |
| `g_pl_last[16]` | 末 15 字节应等于最近一帧 |

### 排错阶梯

| MCU 侧现象 | 含义 | 查什么 |
|---|---|---|
| `g_pl_rx_bytes` 完全不动 | 一个字节都没到 | ① 固件是否把 USB CDC 回调接到 `PoseLink_RxData()` ② 波特率/线 |
| `rx_bytes` 涨、`frame_ok`/`frame_bad` 都不涨 | 字节到了但凑不成完整帧 | 帧长/帧头；看 `g_pl_last` 里有没有 `aa 55` |
| `frame_bad` 涨 | 帧头对了但校验错 | 对比 `g_pl_last` 与上面参考帧；查是不是有两个进程在写 |
| `frame_ok` 涨但 `dbg_*` 值不对 | 解析成功但值不符 | float 端序、字段顺序、`POSE_LINK_OFFSET_*` 是否被设过 |

### 两个容易误判的点

1. **`g_pl_dbg_*` 是加了偏移之后的值**（`pose_link.c` 里累加
   `POSE_LINK_OFFSET_X/Y/YAW`）。默认全 `0.0f`，所以应精确等于解码值。
   若不等，先问电控有没有设偏移，别急着怀疑协议。
2. **低通滤波**（`POSE_LINK_ENABLE_LPF`）。若被打开，`g_pl_dbg_*` 会是平滑后的值，
   不会精确等于单帧解码值 —— 那也不是协议问题。

## 5. 相关源文件

| 文件 | 内容 |
|---|---|
| `src/mid360_pose_bridge/scripts/serial_pose_bridge.py` | 帧打包（`pack_pose_frame`）+ 零点 + 发送/重连 |
| `src/mid360_pose_bridge/scripts/serial_pose_bridge.py::normalize_angle` | yaw 归一化 |
| `tools/refresh_origin.sh` | 刷新零点 |
