cmake_minimum_required(VERSION 3.8)
project(mid360_pose_bridge)

# 自研代码只有位姿流桥一个脚本 + launch/urdf/config。
# 第三方（livox_ros_driver2 / fast_lio）由 dependencies.repos 拉取，不在这里安装。

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME}
  # __pycache__ 是 colcon build 的副产物，别装进去
  PATTERN "__pycache__" EXCLUDE
)

install(PROGRAMS
  scripts/serial_pose_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
