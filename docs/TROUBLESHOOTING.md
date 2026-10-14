# 踩过的坑与结论

> 这份文档是本项目最值钱的部分。代码才 150 KB，但这些坑花了两天的实机调试。
> **每条都写了"症状 → 机理 → 结论"，遇到问题先来这里搜。**

---

## 1. ⚠️ 两个频率必须一致（时基耦合）

**症状**：雷达帧率改高之后，位姿开始漂/退化甚至发散；点云看起来"糊"了。

**机理**：`mid360_handler` **不用驱动的真实逐点时间戳**，而是用 yaw 自己合成：

```cpp
double omega_l = 0.361 * SCAN_RATE;                              // 角速度基准
added_pt.curvature = (yaw_fp - yaw_angle + 360k) / omega_l;      // 合成相对时间
```

`SCAN_RATE` 是 FAST-LIO 的 `preprocess.scan_rate` 参数，**默认 10**。
若雷达实际 60 Hz（16.7 ms/帧）而 `scan_rate` 还是 10（基准 100 ms/帧）：

- 合成的逐点时间被拉长 **6 倍**
- `lidar_end_time = lidar_beg_time + ~100ms`（应为 ~17ms）→ 时间戳落到**未来 83 ms**
- **去畸变的时间基准错 → 运动补偿失效 → 位姿退化/发散**

**结论**：把雷达 `publish_freq` 和 FAST-LIO 的 `preprocess.scan_rate`
**绑成同一个参数**（本仓库的 `frame_rate`），从结构上不可能配错。
YAML 里也留了同样的值做兜底（单独起 FAST-LIO 调试时用）。

---

## 2. 帧率提高**反而**消除了积压（反直觉）

**症状（改之前）**：`odom` 的时间戳比当前时间旧 **1.4 秒**。底盘用这个位姿做闭环会震荡。
表现为 TF 年龄大、`/cloud_registered` 落后 1.4 s，但**数值本身是准确的**（自洽），只是延迟。

**机理**：FAST-LIO 的处理由 **100 Hz 定时器**驱动（`laserMapping.cpp`），
本可以处理 100 帧/秒，积压是**可以排空**的。但：

- 它按输入速率取**最旧**的缓冲帧，FIFO、不丢旧帧
- 10 Hz 时每帧要处理约 20000 个点 → 单帧耗时 ≈100 ms
- **≈100 ms 恰好等于输入间隔** → 积压既不增长也不减少，**永远卡在 13 帧**

**解法**：把帧率提到 **60 Hz**。`publish_freq` 只是驱动侧的**聚合窗口**，
MID360 本身是 20 万点/秒的连续流，所以**总点率不变**，但每帧点数降到：

| 帧率 | 每帧点数 | 抽稀后 | 单帧耗时 vs 输入间隔 | 结果 |
|---|---|---|---|---|
| 10 Hz | ~20000 | ~6700 | ≈100 ms ≈ 100 ms | 积压卡死 → 滞后 **1.4 s** |
| **60 Hz** | ~3300 | **~1100** | **≪ 16.7 ms** | **无积压 → 滞后 10 ms** |

**实测**（5279 样本）：`odom->base_link` 年龄 中位 **10 ms**、P90 18 ms、最大 112 ms。
**改善 140 倍。**

**结论**：遇到"处理不过来"，先看**每帧工作量**而不是总吞吐量。
聚合窗口调小反而让每帧变轻。

---

## 3. `point_filter_num` 在 MID360 路径上被漏掉了

**症状**：和上面同一个 —— 10 Hz 时积压排不掉。

**机理**：`preprocess.cpp` 里四个 handler，`avia` / `oust64` / `velodyne`
**都实现了** `i % point_filter_num` 抽稀，**只有 `mid360_handler` 没有**：

```cpp
// 原代码：所有点都进
if (x*x + y*y + z*z > blind*blind) pl_surf.push_back(std::move(added_pt));
```

配置里明明写着 `point_filter_num: 3`，但这个路径不生效。

**结论**：补丁**逐字采用 `velodyne_handler` 的写法** ——
`is_first` / `yaw_fp` / `time_last` 这些用 yaw 合成点时间的状态
**对每个点照常更新**，只在入队前按 `i % point_filter_num` 过滤。

> ⚠️ **千万不要图省事在循环开头写 `continue`**！那会让 yaw 累加器只看到 1/3 的点，
> 把合成时间算坏 —— 修了性能，坏了精度。`velodyne_handler` 的写法是唯一安全位置。

**附带结论**：MID360 的四条线点数**恰好均匀**（实测 `{0:5064, 1:5064, 2:5064, 3:5064}`），
所以按索引取模 3 是沿方位角**均匀抽稀，不会产生固定角度盲区**。

---

## 4. 两个进程同时写串口 → 帧交错，且极难反查

**症状**：MCU 侧 `g_pl_frame_bad` 上涨、`g_pl_frame_ok` 不涨。
NUC 侧**一切正常**（每个进程都 report 写成功）。

**机理**：15 字节帧被两个 writer 交错（比如 `A1A2 B1B2 A3A4 ...`），
接收端的帧头/校验全乱。现象只出现在 MCU 侧，NUC 侧看不出任何异常。

**结论**：**所有**串口 writer 都加 `serial.Serial(..., exclusive=True)`。
实测任一持有期间另一个 `open()` 必定失败：
`SerialException: [Errno 11] Could not exclusively lock port`。

> ⚠️ 限制：`exclusive` 是 `flock` **建议性锁**，只对同样用 `flock` 的打开者有效。
> 普通 `open()`（`cat`、旧脚本）仍能打开。所以必须**全部** writer 都开。

---

## 5. 零点只在内存里，重启即丢 —— 而且会**静默错开**

**症状**：重启 launch 后，底盘动作方向/幅度完全不对，但没有任何报错。

**机理**：
- 零点只存在桥的内存里，launch 重启就没了（`auto_origin` 会重锁一次）
- **同时 `odom` 也从 0 重新开始**
- 如果重启时机器人不在零点位置，NUC 的新零点和 MCU 记着的旧零点就**错开了**

**结论**：
1. **NUC / launch 重启后必须重锁零点**：把机器人摆回零点 → `refresh_origin.sh`
2. 或者**两边一起重启**（MCU 也重新上电归零）
3. 刷新后一定确认输出里有 **`zero frame sent`** —— 没有就说明 MCU 没收到，
   两边会静默错开

---

## 6. 滞后测量会骗人：监控节点自己就是瓶颈

**症状**：专用探针实测滞后 **10 ms**，但看板报 **6199 ms**，还标成 FAIL。

**机理**：看板主节点同时订阅 60 Hz 的大点云 + odom + imu + tx，
**单线程 Python 读不过来** → 它自己的 TF 缓冲落后好几秒 →
`lookup_transform(Time())` 返回的 "最新" 其实是 6 秒前的。
（同一个原因，频率也被读成 44 Hz 而不是 60。）

**结论**：**测 TF 年龄必须用只订阅 `/tf` 的轻量探针**，不要和重载荷共用一个节点。
本仓库 `tools/send_ready.py` 里的 `measure_tf_lag()` 就是为此单独建的节点。

---

## 7. `map` 帧相关的一连串失败（已通过架构规避）

旧架构（地图 + AMCL）下踩过的连锁坑，现在位姿源改成 `odom` 后**全部消失**：

| 症状 | 机理 |
|---|---|
| RViz 报 `Fixed Frame [map] does not exist`，点云也画不出来 | `map` 帧由 AMCL 发布，而 AMCL 没收到 `/initialpose` 就不发 |
| RViz 的 2D Goal Pose 点了不产生任何消息 | Fixed Frame 无效时工具无法把点击换算到地图坐标 |
| `set_origin` 永远报"位姿样本不足：0 个样本" | `max_tf_age=1.0` 小于实际滞后 1.4 s → 采样全被丢弃，**静默** |
| `Failed to transform initial pose in time` | AMCL 用 `now` 查 TF，而 FAST-LIO 滞后 1.4 s → 外推到未来 |

**结论**：位姿源用 `odom` 时，地图/AMCL/初始位姿全都不需要。
详见 `RUNBOOK.md` 的"三个必要条件"。

---

## 8. `dry_run` 默认 `true` —— 静默不发

**症状**：定位正常、目标也点了，但电控什么都收不到，NUC 侧也没有任何报错。

**机理**：旧的目标式桥的 launch 参数 `dry_run` **默认 `true`**，
而代码里 `_open_port()` 被 `if not self.dry_run and not self.local_test` 保护 ——
串口**根本不会打开**，只打印一行 `serial_mode=DRY_RUN (serial is not opened)`。

**结论**：位姿流桥**没有** `dry_run` 参数（它一启动就真发），
所以本仓库不存在这个陷阱。旧桥的教训是：**默认值不该是"静默不工作"**。

---

## 9. 其他值得记住的小坑

| 坑 | 结论 |
|---|---|
| `pkill -f "xxx"` 会杀掉调用它的 shell | 用 `ps + awk '/[x]xx/'` 的方括号写法避免自匹配 |
| `timeout` 杀不掉 `rclpy` 节点（吞 SIGTERM） | 用 `timeout -k`，或在节点里捕获 `ExternalShutdownException` |
| `ros2 node list` 会因 CLI daemon 卡住而返回空 | 用 `ros2 service list` 探测更可靠；或直接调用，让真实错误说话 |
| `ros2 topic echo` 默认 RELIABLE | 看 BEST_EFFORT 话题要加 `--qos-reliability best_effort`；`/tf_static` 要加 `--qos-durability transient_local` |
| XML 注释里不能有 `--` | URDF 注释写 `--` 会报 `not well-formed (invalid token)` |
| `robot_state_publisher` 的 KDL 不支持根 link 带 `<inertial>` | URDF 根节点（`livox_frame`）必须去掉 `<inertial>` |
| `/tmp` 重启会清空 | 日志一律写仓库内 `logs/` |
| `set -u` 会让 ROS 的 `setup.bash` 直接退出 | 脚本里别用 `set -u`（或用 `${VAR:-}`） |
| 静态 TF 必须挂在 URDF **根**节点 | 挂在 `base_footprint` 会让它有**两个父节点**，所有 `tf2` 查询**静默失败** |
| MID360 雷达本身固定 10 Hz 扫描？ | **不是**。它是 20 万点/秒的连续流，`publish_freq` 只决定聚合窗口 |
