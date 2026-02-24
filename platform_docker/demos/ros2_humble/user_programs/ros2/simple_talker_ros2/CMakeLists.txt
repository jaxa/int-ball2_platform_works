cmake_minimum_required(VERSION 3.8)
project(simple_talker_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(publisher SHARED src/publisher.cpp)
target_compile_definitions(publisher PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(publisher rclcpp rclcpp_components std_msgs)
rclcpp_components_register_nodes(publisher "composition::MinimalPublisher")

add_executable(pub_sub_node src/simple_talker_ros2.cpp)
ament_target_dependencies(pub_sub_node rclcpp rclcpp_components std_msgs)
target_link_libraries(pub_sub_node publisher)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  pub_sub_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
