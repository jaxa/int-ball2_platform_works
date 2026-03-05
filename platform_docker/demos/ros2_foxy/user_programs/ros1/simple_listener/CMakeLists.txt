cmake_minimum_required(VERSION 2.8.3)
project(simple_listener)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES simple_listener
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(simple_listener_node src/simple_listener_node.cpp src/simple_listener.cpp)
target_link_libraries(simple_listener_node ${catkin_LIBRARIES})

