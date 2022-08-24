cmake_minimum_required(VERSION 3.5)
project(linefit_ground_segmentation)

# C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# for debugging
set(CMAKE_CXX_FLAGS "-g" ${CMAKE_CXX_FLAGS})

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

set(deps
  ${PCL_LIBRARIES}
)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Vanilla libs
add_library(linefit_ground_segmentation_lib SHARED src/ground_segmentation.cc src/segment.cc src/bin.cc)
target_link_libraries(linefit_ground_segmentation_lib ${deps})

# install
install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS linefit_ground_segmentation_lib
  DESTINATION lib/${project_name}
)

install(
  TARGETS linefit_ground_segmentation_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(
  linefit_ground_segmentation_lib
)

ament_package()

