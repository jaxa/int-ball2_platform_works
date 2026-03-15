#include <ros/ros.h>
#include <simple_talker/simple_talker.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "simple_talker_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    SimpleTalker simple_talker(nh,pnh);
    ros::spin();
    return 0;
}
