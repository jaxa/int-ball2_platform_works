cmake_minimum_required(VERSION 2.8.3)

project(audio_capture)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   audio_common_msgs
   ib2_msgs
)

find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
   CATKIN_DEPENDS roscpp ib2_msgs
)

include_directories(
   ${catkin_INCLUDE_DIRS}
   ${Boost_INCLUDE_DIRS}
   ${GST1.0_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/audio_capture.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GST1.0_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
