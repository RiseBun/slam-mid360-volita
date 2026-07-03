cmake_minimum_required(VERSION 3.8)
project(pose_guard_mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)

add_executable(pose_guard_mapper_node src/pose_guard_mapper_node.cpp)

target_include_directories(pose_guard_mapper_node PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(pose_guard_mapper_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(pose_guard_mapper_node
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  pcl_conversions
  livox_ros_driver2
)

install(TARGETS pose_guard_mapper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
