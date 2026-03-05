#include <rclcpp/rclcpp.hpp>

#include "simple_talker_ros2/publisher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  const auto publish = std::make_shared<composition::MinimalPublisher>(rclcpp::NodeOptions());

  exec.add_node(publish);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
