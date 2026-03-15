#include <ros/ros.h>
#include <simple_listener/simple_listener.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "simple_listener_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    SimpleListener simple_listener(nh,pnh);
    ros::spin();
    return 0;
}
