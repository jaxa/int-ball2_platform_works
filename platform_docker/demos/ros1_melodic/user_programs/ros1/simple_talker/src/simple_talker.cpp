#include <simple_talker/simple_talker.h>

SimpleTalker::SimpleTalker(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh)
{
  pnh_.param<std::string>("pub_topic", pub_topic_, "pub");
  talker_pub_ = nh_.advertise<std_msgs::String>(pub_topic_, 10);

  boost::thread publish_thread(boost::bind(&SimpleTalker::timerCallback, this));
}

SimpleTalker::~SimpleTalker()
{
}

void SimpleTalker::timerCallback(void)
{
  int data = 0;

  ros::Rate loop_rate(1);
  while(ros::ok())
  {
    std_msgs::String msg;
    msg.data = "Hello!: You are No. " + std::to_string(data);
    talker_pub_.publish(msg);

    data ++;
    loop_rate.sleep();
  }
}