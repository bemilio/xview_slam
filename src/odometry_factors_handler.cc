#include "odometry_factors_handler/odometry_factors_handler.h"

namespace x_view_slam {

OdometryFactorsHandler::OdometryFactorsHandler(
    gtsam::NonlinearFactorGraph graph) : graph_(graph), odometry_counter_(0) {
  Eigen::Matrix<double, 6, 1> noise;
  noise << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
  odom_model_ = gtsam::noiseModel::Diagonal::Sigmas(noise);
  current_transform_.setIdentity();
};

void OdometryFactorsHandler::createOdomFactor(
    tf::Transform& relative_odom_transform) {
  Vector3 t = relative_odom_transform.getOrigin();
  Quaternion orientation = relative_odom_transform.getRotation();
  tf::Matrix3x3 R = tf::Matrix3x3(orientation);
  Pose3_ relative_pose_ = Pose3_(R, t);  //expression because of underscore at the end (?)
  if (odometry_counter_ == 0) {

  }
  else{
    gtsam::Expression<Pose3> x1_(Symbol('x', odometry_counter_));
    gtsam::Expression<Pose3> x2_(Symbol('x', odometry_counter_ + 1));
    graph_.addExpressionFactor(between(x1_, x2_), relative_pose_, odom_model_);
    odometry_counter_++;
  }
};

tf::Transform OdometryFactorsHandler::getRelativeTransormation(
    tf::StampedTransform& T_w_a, tf::StampedTransform& T_w_b) {
  tf::Transform T_a_w = T_w_a.inverse();
  tf::Transform T_a_b = T_a_w * T_w_b;
  return T_a_b;
};

void OdometryFactorsHandler::handleTransform(tf::StampedTransform& transform) {
  tf::Transform relative_odom_transform =
      OdometryFactorsHandler::getRelativeTransormation(current_transform_,
                                                       transform);
  OdometryFactorsHandler::createOdomFactor(relative_odom_transform);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_listener");
  gtsam::NonlinearFactorGraph graph;
  ros::NodeHandle node;
  ROS_INFO("Initialized!");
  tf::TransformListener listener;
  OdometryFactorsHandler odom_handler(graph);
  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/odom", "/odometry/filtered",
                               ros::Time::now(), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    bool loop_closure_flag = 0;
    if(loop_closure_flag == 0){
      odom_handler.handleTransform(transform);
    }
  }
  ros::spin();
  return 0;
}

} // namespace x_view_slam
