#include "imu_listener/imu_listener.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"


//GTSAM headers
//#include <gtsam/slam/expressions.h>
//#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
//#include <gtsam/nonlinear/Values.h>
//#include <gtsam/inference/Symbol.h>
//#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
//#include <gtsam/nonlinear/Marginals.h>
//#include <gtsam/geometry/Pose3.h>

using namespace std;
using namespace gtsam;



class OdometryFactorsHandler {
private:
	tf::StampedTransform current_transform;
	ExpressionFactorGraph::shared_ptr& graph;
	int odometry_counter = 0;
	noiseModel::Diagonal::shared_ptr odomModel = noiseModel::Diagonal::Sigmas((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4));
public:
	OdometryFactorsHandler(ExpressionFactorGraph::shared_ptr&);
	void createOdomFactor(tf::Transform&);
	tf::Transform getRelativeTransormation(tf::StampedTransform&, tf::StampedTransform&);
	void handleTransform(tf::StampedTransform&);
};

OdometryFactorsHandler::OdometryFactorsHandler(ExpressionFactorGraph::shared_ptr& input_graph){
	current_transform.setIdentity();
	graph =  input_graph;
};

void OdometryFactorsHandler::createOdomFactor(tf::Transform& relative_odom_transform){
	Vector3 t = relative_odom_transform.getOrigin();
	Quaternion orientation = relative_odom_transform.getRotation();
	tf::Matrix3x3 R = tf::Matrix3x3(orientation);
	Pose3_ relative_pose_ = Pose3_(R, t); //expression because of underscore at the end (?)
	if(odometry_counter==0){

	}
	else{
		Expression<Pose3> x1_(Symbol('x', odometry_counter));
		Expression<Pose3> x2_(Symbol('x', odometry_counter+1));
		graph.addExpressionFactor(between(x1_, x2_), relative_pose_, odomModel);
		odometry_counter++;
	}
};

tf::Transform OdometryFactorsHandler::getRelativeTransormation(tf::StampedTransform& T_w_a,
			 tf::StampedTransform& T_w_b){
	tf::Transform T_a_w = T_w_a.inverse();
	tf::Transform T_a_b = T_a_w * T_w_b;
	return T_a_b; 
};

void OdometryFactorsHandler::handleTransform(tf::StampedTransform& transform){
	tf::Transform relative_odom_transform = OdometryFactorsHandler::getRelativeTransormation(current_transform, transform);
	OdometryFactorsHandler::createOdomFactor(relative_odom_transform);

}


int main(int argc, char **argv){
	ros::init(argc, argv, "imu_listener");
	ExpressionFactorGraph graph;
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
