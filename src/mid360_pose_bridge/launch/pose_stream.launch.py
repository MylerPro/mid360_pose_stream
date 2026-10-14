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
    send_rate = LaunchConfiguration("send_rate")
    pose_source_frame = LaunchConfiguration("pose_source_frame")
    max_tf_age = LaunchConfiguration("max_tf_age")
    auto_origin = LaunchConfiguration("auto_origin")
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    log_frames = LaunchConfiguration("log_frames")
    rviz = LaunchConfiguration("rviz")

    # 雷达驱动：MID360 本身是 20 万点/秒的连续流，publish_freq 只决定多久攒一帧。
    # 60 Hz -> 每帧约 3300 点（总点率不变），FAST-LIO 单帧轻松跑完、不产生积压。
    livox = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_mid360",
        output="screen",
        parameters=[{
            "xfer_format": 0,
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

    # FAST-LIO：位姿源。scan_rate 必须等于雷达实际帧率 —— mid360_handler 用
    # omega_l = 0.361 * SCAN_RATE 从 yaw 合成每个点的时间；两者不一致时合成时间
    # 会被拉伸/压缩（10 vs 60 会错 6 倍）-> 去畸变时间基准错 -> 位姿退化甚至发散。
    fastlio = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[fastlio_config, {
            "preprocess.scan_rate": ParameterValue(frame_rate, value_type=int),
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
            "frame_rate", default_value="60",
            description="雷达帧率[Hz]，同时驱动 publish_freq 与 preprocess.scan_rate。"
                        "两者必须一致，故绑成一个参数。回退写 10。"),
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
