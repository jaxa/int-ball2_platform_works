cmake_minimum_required(VERSION 2.8.3)
project(simple_talker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES simple_talker
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(simple_talker_node src/simple_talker_node.cpp src/simple_talker.cpp)
target_link_libraries(simple_talker_node ${catkin_LIBRARIES})

