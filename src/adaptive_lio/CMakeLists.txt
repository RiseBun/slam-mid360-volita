cmake_minimum_required(VERSION 3.8)
project(adaptive_lio)

# Build type
set(CMAKE_BUILD_TYPE "Release")

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")

# ========================
# ROS2 dependencies
# ========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(std_srvs REQUIRED)

# ========================
# System dependencies
# ========================
# Custom cmake modules for CSparse/Cholmod
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Glog REQUIRED)
find_package(OpenMP QUIET)
find_package(CSparse REQUIRED)
find_package(Cholmod REQUIRED)

if(OpenMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif()

# Processor count for multi-threading
include(ProcessorCount)
ProcessorCount(N)
if(N GREATER 4)
  add_definitions(-DMP_EN)
  add_definitions(-DMP_PROC_NUM=3)
elseif(N GREATER 3)
  add_definitions(-DMP_EN)
  add_definitions(-DMP_PROC_NUM=2)
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

# ROOT_DIR macro for config file path
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# ========================
# Include directories
# ========================
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/sophus
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Glog_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
)
link_directories(${PCL_LIBRARY_DIRS})

# ========================
# Robin map (tessil)
# ========================
include(FetchContent)
FetchContent_Declare(
  tessil
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/tessil-src
)
if(NOT tessil_POPULATED)
  set(BUILD_TESTING OFF)
  FetchContent_Populate(tessil)
  add_library(robin_map INTERFACE)
  add_library(tsl::robin_map ALIAS robin_map)
  target_include_directories(robin_map INTERFACE
    "$<BUILD_INTERFACE:${tessil_SOURCE_DIR}/include>")
endif()

# ========================
# Collect non-ROS third party libs
# ========================
set(third_party_libs
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  glog gflags
  ${yaml-cpp_LIBRARIES}
  yaml-cpp
  robin_map
)

# ========================
# Build sub-libraries
# ========================
# --- common ---
add_library(${PROJECT_NAME}.common
  src/common/timer/timer.cc
  src/common/utility.cpp
  src/common/utils_trans.cpp
  src/common/utils.cpp
  src/common/global_flags.cc
  src/common/map.cpp
  src/common/config.cpp
)
target_link_libraries(${PROJECT_NAME}.common
  ${third_party_libs}
)

# --- algo ---
add_library(${PROJECT_NAME}.algo
  src/algo/static_imu_init.cc
)
target_link_libraries(${PROJECT_NAME}.algo
  glog gflags ${PROJECT_NAME}.common
)

# --- lio ---
add_library(${PROJECT_NAME}.lio
  src/lio/poseParameterization.cpp
  src/lio/lidarFactor.cpp
  src/lio/lio_utils.cpp
  src/lio/lidarodom.cpp
)
target_link_libraries(${PROJECT_NAME}.lio
  ${PROJECT_NAME}.common
  ${PROJECT_NAME}.algo
  ${third_party_libs}
)

# --- preprocess ---
add_library(${PROJECT_NAME}.pre
  src/preprocess/cloud_convert/cloud_convert2.cc
)
target_link_libraries(${PROJECT_NAME}.pre
  ${third_party_libs}
)
ament_target_dependencies(${PROJECT_NAME}.pre
  sensor_msgs
  pcl_conversions
  livox_ros_driver2
)

# ========================
# Main executable
# ========================
add_executable(adaptive_lio_node
  src/apps/main_ros2.cpp
  src/apps/common_utility.cpp
)
target_link_libraries(adaptive_lio_node
  ${PROJECT_NAME}.common
  ${PROJECT_NAME}.pre
  ${PROJECT_NAME}.lio
  ${PROJECT_NAME}.algo
  ${third_party_libs}
)
ament_target_dependencies(adaptive_lio_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  pcl_conversions
  livox_ros_driver2
  std_srvs
)

# ========================
# Install
# ========================
install(TARGETS adaptive_lio_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
