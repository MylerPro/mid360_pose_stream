# 补丁

这两个补丁是**必需的**，不是可选优化 —— 没有它们，从 `dependencies.repos`
拉下来的干净代码**编不过或跑不起来**。补丁均可双向校验（`git apply --reverse --check`
通过 ⇒ 能正向应用到钉死的 commit 上）。

## 应用

```bash
mkdir -p src && vcs import src < dependencies.repos
bash patches/apply.sh          # 一次性把两个补丁打上
```

## `fastlio_local_changes.patch`（192 行）

对上游 `Ericsii/FAST_LIO_ROS2 @ 2fffc57` 的本地改动：

| 文件 | 改动 | 为什么必需 |
|---|---|---|
| `CMakeLists.txt` | `pcl_ros` → `tf2_ros`；PCL 组件加 `filters kdtree` | **编译修复**：上游在 Humble 下 `pcl_ros` 依赖链会让 `find_package` 失败 |
| `package.xml` | 同上，依赖换成 `tf2_ros` + `libpcl-all-dev` | 同上 |
| `src/laserMapping.cpp` | 新增 `odom_topic` / `odom_frame` / `base_frame` 三个参数（原先帧名硬编码 `camera_init` / `body`），并发布里程计 | 帧名可配，才能和 URDF 对上 |
| `src/preprocess.cpp` | **`mid360_handler` 补上 `point_filter_num` 抽稀** | 见下 |

### 为什么必须补 `point_filter_num`

`mid360_handler` 原本是唯一一个**没有**应用 `point_filter_num` 的 handler
（`avia_handler` / `oust64_handler` / `velodyne_handler` 三个都实现了），
于是每帧硬吃约 15280 个点。后果很隐蔽：

- 单帧耗时 ≈100 ms ≈ 输入间隔（10 Hz）→ 启动阶段形成的约 13 帧积压**永远排不掉**
- `odom` 时间戳比现在旧 **1.4 s** → 底盘拿这个位姿做闭环会震荡

补丁**逐字采用 `velodyne_handler` 的写法**：`is_first` / `yaw_fp` / `time_last`
这些用 yaw 合成点时间的状态**对每个点照常更新**，只在入队前按 `i % point_filter_num` 过滤
—— 所以合成的点时间完全不受影响。

> ⚠️ 千万不要图省事在循环开头写 `continue`：那会让 yaw 累加器只看到 1/3 的点，
> 把合成时间算坏 → 去畸变时间基准错 → 位姿退化甚至发散。

## `livox_driver_local_changes.patch`（79 行）

对上游 `Livox-SDK/livox_ros_driver2 @ 4a1def9` 的本地改动：

| 文件 | 改动 | 为什么必需 |
|---|---|---|
| `CMakeLists.txt` | 加 `INSTALL_RPATH` 指向 Livox SDK 库目录 | **不装就跑不起来**：否则 `livox_ros_driver2_node` 找不到 `liblivox_sdk_shared.so`（症状是 `bind failed` / `Init lds lidar fail!`） |
| `config/MID360_config.json` | 主机 IP `192.168.1.5` → `192.168.1.50` | 上游默认 IP 与本机网口不符 |
| `launch_ROS2/rviz_MID360_launch.py` | 设置 `LD_LIBRARY_PATH` | 同上（该 launch 本项目不用，但保持能跑） |

> 本项目的实际网络配置用的是包内的 `config/MID360_hardware_config.json`，
> 不依赖驱动自带的那份；这份改动是为了让驱动自身的示例也能用。
