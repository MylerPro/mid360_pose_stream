#!/usr/bin/env python3
"""MID360 位姿流 —— 唯一入口。

启动链：
    livox_ros_driver2  ->  /livox/lidar + /imu/data
    fast_lio           ->  odom -> livox_frame   (TF)  +  /odom
    robot_state_publisher (URDF)  ->  livox_frame -> base_link -> base_footprint
    serial_pose_bridge ->  以 send_rate 连续把 (odom->base_link 相对零点) 的
                           x/y/yaw 打包成 15 字节帧写串口给 STM32

不需要地图 / AMCL / Nav2 / 初始位姿：雷达一起来就有位姿。

用法：
    ros2 launch mid360_pose_bridge pose_stream.launch.py
    ros2 launch mid360_pose_bridge pose_stream.launch.py frame_rate:=10 send_rate:=20.0
    ros2 launch mid360_pose_bridge pose_stream.launch.py rviz:=true

ⓘ frame_rate 同时驱动 驱动的 publish_freq 和 FAST-LIO 的 preprocess.scan_rate。
  这两者【必须一致】—— 见 README 的「两个频率必须一起改」。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

SERIAL_BY_ID = (
    "/dev/serial/by-id/"
    "usb-STMicroelectronics_STM32_Virtual_ComPort_3154365A3233-if00"
)


def _validate(context, *args, **kwargs):
    """启动前把明显配错的参数拒掉，避免运行到一半才发现。"""
    del args, kwargs
    frame_rate = float(LaunchConfiguration("frame_rate").perform(context))
    send_rate = float(LaunchConfiguration("send_rate").perform(context))
    source = LaunchConfiguration("pose_source_frame").perform(context).strip()
    if not 0.5 <= frame_rate <= 100.0:
        raise RuntimeError(
            f"frame_rate must be in [0.5, 100] Hz (driver clamps outside this), "
            f"got {frame_rate}")
    if not 0.0 < send_rate <= 100.0:
        raise RuntimeError(f"send_rate must be in (0, 100] Hz, got {send_rate}")
    xfer = int(LaunchConfiguration("xfer_format").perform(context))
    ltype = int(LaunchConfiguration("lidar_type").perform(context))
    if (xfer, ltype) not in ((1, 1), (0, 4)):
        raise RuntimeError(
            f"xfer_format={xfer} 与 lidar_type={ltype} 不配对，FAST-LIO 会收不到数据。"
            f"正确组合：xfer_format=1 + lidar_type=1（推荐，真实时间戳）；"
            f"或 xfer_format=0 + lidar_type=4（旧，仅 10 Hz 正确）")
    if source not in ("odom", "map"):
        raise RuntimeError(
            f"pose_source_frame must be 'odom' (odometry) or 'map' (needs AMCL), "
            f"got '{source}'")
    return []


def generate_launch_description():
    share = get_package_share_directory("mid360_pose_bridge")

    urdf_path = os.path.join(share, "urdf", "diffbot_mid360_hardware.urdf")
    with open(urdf_path, "r", encoding="utf-8") as stream:
        robot_description = stream.read()

    livox_config = os.path.join(share, "config", "MID360_hardware_config.json")
    fastlio_config = os.path.join(share, "config", "fastlio_mid360.yaml")
    rviz_config = os.path.join(share, "config", "pose_stream.rviz")

    frame_rate = LaunchConfiguration("frame_rate")
    xfer_format = LaunchConfiguration("xfer_format")
    lidar_type = LaunchConfiguration("lidar_type")
    send_rate = LaunchConfiguration("send_rate")
    pose_source_frame = LaunchConfiguration("pose_source_frame")
    max_tf_age = LaunchConfiguration("max_tf_age")
    auto_origin = LaunchConfiguration("auto_origin")
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    log_frames = LaunchConfiguration("log_frames")
    max_speed = LaunchConfiguration("max_speed")
    divergence_frames = LaunchConfiguration("divergence_frames")
    max_lidar_gap = LaunchConfiguration("max_lidar_gap")
    lidar_relock_gap = LaunchConfiguration("lidar_relock_gap")
    max_lidar_backlog = LaunchConfiguration("max_lidar_backlog")
    rviz = LaunchConfiguration("rviz")

    # 雷达驱动：MID360 本身是 20 万点/秒的连续流，publish_freq 只决定多久攒一帧。
    # 60 Hz -> 每帧约 3300 点（总点率不变），FAST-LIO 单帧轻松跑完、不产生积压。
    livox = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_mid360",
        output="screen",
        parameters=[{
            # ⚠️ 1 = CustomMsg —— 必须与 FAST-LIO 的 preprocess.lidar_type=1 配对。
            # CustomMsg 带【驱动测得的真实逐点 offset_time】，FAST-LIO 的 avia_handler
            # 直接使用它，不做任何 yaw 合成，因此与雷达帧率无关。
            # 若改回 0（PointCloud2）就必须同时把 lidar_type 改回 4，否则收不到数据。
            "xfer_format": ParameterValue(xfer_format, value_type=int),
            "multi_topic": 0,
            "data_src": 0,
            "publish_freq": ParameterValue(frame_rate, value_type=float),
            "output_data_type": 0,
            "frame_id": "livox_frame",
            "lvx_file_path": "",
            "user_config_path": livox_config,
            "cmdline_input_bd_code": "livox0000000001",
        }],
        remappings=[("livox/imu", "/imu/data")],
    )

    # FAST-LIO：位姿源。
    # ⚠️ scan_rate 只在 lidar_type=4（mid360_handler 的 yaw 合成）下有用，
    #    而它在 publish_freq=60 下【到底该填多少无法确定】：实测同样填 60，
    #    一次完全正常、一次从开机就 "No Effective Points" 并最终发散
    #    （两次的代码/配置/编译参数/速率核对为相同 → 失败不是确定性的）。
    #    所以不要依赖它 —— 用 lidar_type=1（CustomMsg 真实时间戳），
    #    curvature 直接来自驱动的 offset_time，这个值就不参与位姿计算了。
    fastlio = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[fastlio_config, {
            # 1 = avia/CustomMsg（用驱动的真实 offset_time），4 = mid360/PointCloud2
            # （用 yaw 合成时间，60 Hz 下会算坏 —— 详见 YAML 里的说明）
            "preprocess.lidar_type": ParameterValue(lidar_type, value_type=int),
            # 固定 10，【不要】改成 frame_rate —— 它是雷达的一圈方位角周期，见上。
            # 只有 lidar_type=4 会用到；lidar_type=1 时不参与位姿计算。
            "preprocess.scan_rate": 10,
            # 治那 1.4 秒启动积压：FAST-LIO 初始化期间驱动堆下的帧永不排空
            # （sync_packages 严格 FIFO、从不丢弃）。超过此值就丢最旧的帧。
            "max_lidar_backlog": ParameterValue(max_lidar_backlog, value_type=int),
        }],
    )

    # TF 链：URDF 的根是 livox_frame（FAST-LIO 发布的帧），
    # livox_frame -> base_link -> base_footprint。
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    # 唯一的串口 writer。exclusive=True（flock）保证同一时刻只有一个进程能持有串口。
    bridge = Node(
        package="mid360_pose_bridge",
        executable="serial_pose_bridge.py",
        name="serial_pose_bridge",
        output="screen",
        parameters=[{
            "serial_port": serial_port,
            "baudrate": ParameterValue(baudrate, value_type=int),
            "send_rate": ParameterValue(send_rate, value_type=float),
            "map_frame": pose_source_frame,
            "base_frame": "base_link",
            "pose_topic": "/chassis/pose",
            "tx_pose_topic": "/chassis/serial_tx_pose",
            "origin_topic": "/chassis/origin",
            "refresh_service": "~/refresh_origin",
            # 必须大于位姿源的时间戳年龄，否则一帧都发不出去，而且完全静默。
            "max_pose_age": ParameterValue(max_tf_age, value_type=float),
            "auto_origin": ParameterValue(auto_origin, value_type=bool),
            "log_frames": ParameterValue(log_frames, value_type=bool),
            # 发散看门狗：位姿源发散时【停发】，避免底盘收到乱跑的位姿
            "max_speed": ParameterValue(max_speed, value_type=float),
            "divergence_frames": ParameterValue(divergence_frames, value_type=int),
            # 雷达断流看门狗：超时停发（雷达一断，位姿就退化成纯 IMU 推算）
            "max_lidar_gap": ParameterValue(max_lidar_gap, value_type=float),
            "lidar_relock_gap": ParameterValue(lidar_relock_gap, value_type=float),
            "stats_period": 5.0,
            "reconnect_period": 2.0,
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_pose_stream",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_type", default_value="1",
            description="FAST-LIO 的 preprocess.lidar_type：1=avia/CustomMsg（默认，用驱动"
                        "真实时间戳，与帧率无关）；4=mid360/PointCloud2（用 yaw 合成时间，"
                        "仅在 10 Hz 下正确）。必须与 xfer_format 配对。"),
        DeclareLaunchArgument(
            "xfer_format", default_value="1",
            description="Livox 驱动的消息格式：1=CustomMsg（默认，带真实逐点时间戳）；"
                        "0=PointCloud2。必须与 lidar_type 配对（1↔1，0↔4）。"),
        DeclareLaunchArgument(
            "frame_rate", default_value="60",
            description="雷达发布帧率[Hz] → 驱动的 publish_freq。"
                        "注意：它【不】驱动 preprocess.scan_rate（那是雷达的一圈"
                        "方位角周期，固定 10，见 launch 内注释）。回退写 10。"),
        DeclareLaunchArgument(
            "send_rate", default_value="60.0",
            description="串口发送频率[Hz]。回退写 20.0。"),
        DeclareLaunchArgument(
            "pose_source_frame", default_value="odom",
            description="位姿参考帧：odom（里程计，默认，不依赖地图）或 map（需 AMCL）"),
        DeclareLaunchArgument(
            "max_tf_age", default_value="5.0",
            description="允许的位姿时间戳最大年龄[s]。设小了会静默锁不上零点。"),
        DeclareLaunchArgument(
            "auto_origin", default_value="true",
            description="启动后第一帧有效位姿自动锁零点；之后可用 refresh_origin 重锁"),
        DeclareLaunchArgument("serial_port", default_value=SERIAL_BY_ID),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("log_frames", default_value="true"),
        DeclareLaunchArgument(
            "max_speed", default_value="5.0",
            description="发散看门狗的速度上限[m/s]。整车物理上跑不了这么快，"
                        "连续超限就停发。设 0 关闭（不建议）。"),
        DeclareLaunchArgument(
            "divergence_frames", default_value="5",
            description="连续多少帧超速后锁死停发。60Hz 下 5 帧仅 83 ms。"),
        DeclareLaunchArgument(
            "max_lidar_gap", default_value="1.0",
            description="雷达断流看门狗：超过这个秒数没收到 /livox/lidar 就停发。"
                        "雷达一断，FAST-LIO 退化成纯 IMU 推算，位姿会漂但时间戳"
                        "依然新鲜（max_pose_age 抓不到）。设 0 关闭。"),
        DeclareLaunchArgument(
            "lidar_relock_gap", default_value="5.0",
            description="雷达断流超过这个秒数就【锁死】，必须调 refresh_origin 才恢复。"
                        "车体调试断电会让雷达掉电，此时底盘也忘了零点，两边都不可信，"
                        "必须重新建立零点。短于此值的小抖动会自动恢复。"),
        DeclareLaunchArgument(
            "max_lidar_backlog", default_value="5",
            description="FAST-LIO 最多保留几帧雷达数据（治那 1.4 秒启动积压）。"
                        "延迟上限 ≈ 该值 / frame_rate。0 = 关闭。"),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="是否启动 RViz。它是纯可视化，挂掉不影响发送，"
                        "但跑在同一台机器上会占 CPU —— 建议按需再开。"),
        OpaqueFunction(function=_validate),
        state_publisher,
        livox,
        fastlio,
        bridge,
        rviz_node,
    ])
