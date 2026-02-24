#include <simple_listener/simple_listener.h>

SimpleListener::SimpleListener(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh)
{
  pnh_.param<std::string>("sub_topic", sub_topic_, "sub");;
  listener_sub_ = nh_.subscribe(sub_topic_, 10, &SimpleListener::MsgCallback, this);

}

SimpleListener::~SimpleListener()
{
}

void SimpleListener::MsgCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}