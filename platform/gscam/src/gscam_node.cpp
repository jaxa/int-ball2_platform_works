
#include <ros/ros.h>
#include <gscam/gscam.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "gscam_publisher");
  ros::NodeHandle nh("~");

  gscam::GSCam gscam_driver(nh);
  gscam_driver.run();

  return 0;
}

