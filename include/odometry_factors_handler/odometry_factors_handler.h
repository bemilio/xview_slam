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
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>

#include <mincurves/DiscreteSE3Curve.hpp>


namespace x_view_slam {


typedef kindr::minimal::QuatTransformationTemplate<double> SE3; // A frame transformation built from a quaternion and a point
typedef SE3::Rotation SO3;

class OdometryFactorsHandler {
 private:
	tf::StampedTransform current_transform_;
	gtsam::NonlinearFactorGraph::shared_ptr& graph_;
	int odometry_counter_;
	gtsam::noiseModel::Diagonal::shared_ptr noise_model_;
 public:
  OdometryFactorsHandler(gtsam::NonlinearFactorGraph::shared_ptr&);
  gtsam::ExpressionFactor<SE3> createOdomFactor(tf::StampedTransform&, tf::StampedTransform&);
  tf::Transform getRelativeTransormation(tf::StampedTransform&,
                                         tf::StampedTransform&);
  void handleTransform(tf::StampedTransform&);
};

} // namespace x_view_slam

#endif // X_VIEW_SLAM_ODOMETRY_FACTORS_HANDLER_H_
