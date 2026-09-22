cmake_minimum_required(VERSION 3.5)
project(gazebo_mimic_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

# Specify additional locations of header files
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

# Declare the mimic model plugin library
add_library(gazebo_mimic SHARED src/MimicPlugin.cpp)

ament_target_dependencies(gazebo_mimic
  gazebo_dev
  gazebo_ros
)

# Install targets
install(TARGETS gazebo_mimic
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(gazebo_mimic)
ament_export_dependencies(gazebo_ros)

ament_package()
