#include <ros/ros.h>
#include "ros_package_template/imu_listener.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_listener");
  ros::NodeHandle nodeHandle("~");

  imu_listener::ImuListener imu_listener(nodeHandle);

  ros::spin();
  return 0;
}
