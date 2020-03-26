cmake_minimum_required(VERSION 2.8.3)
project(safety)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(safety_node src/safety_node.cpp)

target_link_libraries(safety_node
  ${catkin_LIBRARIES}
)