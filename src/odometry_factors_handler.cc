#include "odometry_factors_handler/odometry_factors_handler.h"

namespace x_view_slam {

OdometryFactorsHandler::OdometryFactorsHandler(gtsam::NonlinearFactorGraph::shared_ptr& input_graph)
    : graph_(input_graph), odometry_counter_(0)
     {
  Eigen::Matrix<double,6,1> noise;
  noise(0) = 0.0000001;
  noise(1) = 0.0000001;
  noise(2) = 0.0000001;
  noise(3) = 0.0000001;
  noise(4) = 0.0000001;
  noise(5) = 0.0000001;
  noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(noise);
};

gtsam::ExpressionFactor<SE3> OdometryFactorsHandler::createOdomFactor(
    tf::StampedTransform& transform_a, tf::StampedTransform& transform_b) {
  Eigen::Quaternion<double> q_w_a_temp(transform_a.getRotation());
  Eigen::Quaternion<double> q_w_b_temp(transform_b.getRotation());
  SE3::Rotation q_w_a(q_w_a_temp);
  SE3::Position p_w_a(transform_a.getOrigin());
  SE3::Rotation q_w_b(q_w_b_temp);
  SE3::Position p_w_b(transform_b.getOrigin());
  SE3 T_w_a(q_w_a, p_w_a);
  SE3 T_w_b(q_w_b, p_w_b);
  gtsam::Expression<SE3> T_w_a_(T_w_a);
  gtsam::Expression<SE3> T_w_b_(T_w_b);
  gtsam::Expression<SE3> T_a_w_(kindr::minimal::inverse(T_w_a_));
  gtsam::Expression<SE3> relative_expression(kindr::minimal::compose(T_a_w_, T_w_b_));
  SE3 relative_measurement = T_w_a.inverse().operator*(T_w_b);

  if (odometry_counter_ == 0) {

  }
  else{
    odometry_counter_++;
    return(gtsam::ExpressionFactor<SE3>(noise_model_,relative_measurement, relative_expression));
      // notes on usage:
      // ExpressionFactor(const SharedNoiseModel& noiseModel, 
      // const T& measurement, const Expression<T>& expression) 
  }

};

void OdometryFactorsHandler::handleTransform(tf::StampedTransform& transform) {
  OdometryFactorsHandler::createOdomFactor(current_transform_, transform);
}

} // namespace x_view_slam
