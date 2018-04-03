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

void callback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Qualcuno ha detto [%s] ?!", msg->data.c_str());
};


class ImuComposer
{
private:
	geometry_msgs::Quaternion imu_composed ();
	
public:
    void callback_imu(const sensor_msgs::Imu::ConstPtr& imu_msg){
	ROS_INFO("Orientation:");
	ROS_INFO_STREAM(imu_msg->orientation);
	
	//for(int i=0;i<9;i++){
	//	ROS_INFO_STREAM(imu_msg->orientation_covariance[i]);

    	//WARNING: if this function does not return void I get error
    	//when using it as a callback
    	//return(imu_msg->orientation);

	}
};





// void callback_joints(const sensor_msgs::JointState::ConstPtr& joints_msg){
	
// };

// void callback_odom(const nav_msgs::Odometry::ConstPtr& odom_msg){
// 	//geometry_msgs::PoseWithCovariance odom_with_cov_msg = odom_msg->pose;
// 	//geometry_msgs::Pose odom_pose_msg = odom_with_cov_msg.pose;
// 	//geometry_msgs::Point odom_posit_msg = odom_pose_msg.position;
// 	// geometry_msgs::Quaternion odom_orientation_msg = odom_pose_msg.orientation;
// 	// tf::Quaternion odom_orientation  = tf::Quaternion(odom_orientation_msg.x, odom_orientation_msg.y, odom_orientation_msg.z, odom_orientation_msg.w);
// };

// void callback_localization(const nav_msgs::Odometry::ConstPtr& odom_msg){
// 	geometry_msgs::PoseWithCovariance::ConstPtr& odom_with_cov_msg = odom_msg->pose;
// 	geometry_msgs::Pose::ConstPtr& odom_pose_msg = odom_with_cov_msg->pose;
// 	geometry_msgs::Point::ConstPtr& odom_posit_msg = odom_pose_msg->position;
// 	geometry_msgs::Quaternion::ConstPtr& odom_orientation_msg = odom_pose_msg->orientation;
// 	tf::Quaternion odom_orientation  = tf::Quaternion(odom_orientation_msg->x,
// 					odom_orientation_msg->y, odom_orientation_msg->z, odom_orientation_msg->w);
// 	ROS_INFO("robot_localization output:");
// 	ROS_INFO_STREAM(odom_orientation);
// };

class OdometryFactorsHandler{
private:
	tf::StampedTransform current_transform;
public:
	OdometryFactorsHandler();
	void createOdomFactor(tf::Transform&);
	tf::Transform getRelativeTransormation(tf::StampedTransform&, tf::StampedTransform&);
	void handleTransform(tf::StampedTransform&);
};

OdometryFactorsHandler::OdometryFactorsHandler(){
	//declare constructor here
	current_transform.setIdentity();
};

void OdometryFactorsHandler::createOdomFactor(tf::Transform& relative_odom_transform){
	;
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
	ImuComposer imuComposer;
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("random_topic", 1000, callback);
	ROS_INFO("Initialized!");
	tf::TransformListener listener;
	OdometryFactorsHandler odom_handler;
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
	//ros::Subscriber imu_sub = node.subscribe("/imu/data", 1000, &ImuComposer::callback_imu, &imuComposer);
	//ros::Subscriber odometry_sub = node.subscribe("/husky_velocity_controller/odom ", 1000, callback_odom);
	//ros::Subscriber imu_sub = node.subscribe("/joint_states", 1000, callback_joints);
	ROS_INFO("tf listener started");;
	ros::spin();
	return 0;
}
