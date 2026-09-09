cmake_minimum_required(VERSION 3.8)
project(arm_grasp_sim)

find_package(ament_cmake REQUIRED)

# 资源文件安装到 share/${PROJECT_NAME}（FindPackageShare 才能解析）
install(DIRECTORY urdf config worlds launch
  DESTINATION share/${PROJECT_NAME})

# 可执行节点安装到 lib/${PROJECT_NAME}（ros2 run 用）
install(PROGRAMS
  scripts/grasp_controller.py
  scripts/grasp_controller_real.py
  scripts/grasp_interface.py
  scripts/send_grasp_goal.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
