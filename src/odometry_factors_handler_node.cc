#include <ros/ros.h>
#include "odometry_factors_handler/odometry_factors_handler.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_listener");
  gtsam::NonlinearFactorGraph::shared_ptr graph;
  ros::NodeHandle node;
  ROS_INFO("Initialized!");
  tf::TransformListener listener;
  x_view_slam::OdometryFactorsHandler odom_handler(graph);
  ros::Rate rate(10.0);
  listener.waitForTransform("/odom", "/odometry/filtered",ros::Time(), ros::Duration(4.0));
  tf::StampedTransform transform;
  listener.lookupTransform("/odom", "/odometry/filtered",  
                  ros::Time::now(), transform);
  odom_handler.handleTransform(transform);
  ros::spin();
  return 0;
}
