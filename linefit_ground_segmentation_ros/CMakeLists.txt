cmake_minimum_required(VERSION 3.5)
project(linefit_ground_segmentation_ros)

# C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# for debugging
set(CMAKE_CXX_FLAGS "-g"${CMAKE_CXX_FLAGS})

find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(linefit_ground_segmentation REQUIRED)
find_package(pcl_conversions REQUIRED)

set(deps_ros
  rclcpp
  rclcpp_components
  tf2_ros
  sensor_msgs
  geometry_msgs
  pcl_conversions
  linefit_ground_segmentation
)

set(deps
  ${PCL_LIBRARIES}
)

include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${linefit_ground_segmentation_ros_INCLUDE_DIRS}
)

add_executable(ground_segmentation_node src/ground_segmentation_node.cpp)
target_link_libraries(ground_segmentation_node ${deps})
ament_target_dependencies(ground_segmentation_node
  ${deps_ros}
)

install(
  TARGETS ground_segmentation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ground_segmentation_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)

ament_package()

