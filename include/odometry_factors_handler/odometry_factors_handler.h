#ifndef X_VIEW_SLAM_ODOMETRY_FACTORS_HANDLER_H_
#define X_VIEW_SLAM_ODOMETRY_FACTORS_HANDLER_H_

// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>


// GTSAM headers
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ExpressionFactor.h>

namespace x_view_slam {

class OdometryFactorsHandler {
 private:
  tf::StampedTransform current_transform_;
  gtsam::NonlinearFactorGraph graph_;
  int odometry_counter_;
  gtsam::noiseModel::Diagonal::shared_ptr odom_model_;
 public:
  OdometryFactorsHandler(gtsam::NonlinearFactorGraph graph);
  void createOdomFactor(tf::Transform&);
  tf::Transform getRelativeTransormation(tf::StampedTransform&,
                                         tf::StampedTransform&);
  void handleTransform(tf::StampedTransform&);
};

} // namespace x_view_slam

#endif // X_VIEW_SLAM_ODOMETRY_FACTORS_HANDLER_H_
