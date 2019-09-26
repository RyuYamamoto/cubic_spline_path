cmake_minimum_required(VERSION 2.8.3)
project(cubic_spline_path)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  tf2
  tf2_ros
  geometry_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cubic_spline_path_node src/cubic_spline_path_node.cpp src/cubic_spline_path.cpp)
target_link_libraries(cubic_spline_path_node
  yaml-cpp
  ${catkin_LIBRARIES}
)