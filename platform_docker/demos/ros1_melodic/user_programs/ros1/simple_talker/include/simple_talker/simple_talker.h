#pragma once

#include <iostream> 
#include <math.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>

class SimpleTalker
{
public:
  SimpleTalker(ros::NodeHandle nh,ros::NodeHandle pnh);
  ~SimpleTalker();
private:
  void timerCallback(void);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string pub_topic_;

  ros::Publisher talker_pub_;

};
