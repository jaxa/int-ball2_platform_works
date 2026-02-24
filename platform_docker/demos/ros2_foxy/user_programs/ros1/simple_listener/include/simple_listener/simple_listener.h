#pragma once

#include <iostream> 
#include <math.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>

class SimpleListener
{
public:
  SimpleListener(ros::NodeHandle nh,ros::NodeHandle pnh);
  ~SimpleListener();

  void MsgCallback(const std_msgs::String::ConstPtr& msg);

private:

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string sub_topic_;

  ros::Subscriber listener_sub_;

};
