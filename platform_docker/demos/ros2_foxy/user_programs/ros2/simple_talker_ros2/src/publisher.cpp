#include <simple_talker_ros2/publisher.hpp>

#include <rclcpp_components/register_node_macro.hpp>

namespace composition
{

MinimalPublisher::MinimalPublisher(const rclcpp::NodeOptions & options) : Node("minimal_publisher", options), count_(0)
{
  this->declare_parameter("pub_topic", "topic");
  pub_topic_ = this->get_parameter("pub_topic").as_string();

  publisher_ = this->create_publisher<std_msgs::msg::String>(pub_topic_, 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
}

MinimalPublisher::~MinimalPublisher()
{
}

void MinimalPublisher::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

} // namespace composition

RCLCPP_COMPONENTS_REGISTER_NODE(composition::MinimalPublisher)