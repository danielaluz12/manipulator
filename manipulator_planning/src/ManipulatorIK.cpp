#include "manipulator_planning/ManipulatorIK.h"

namespace manipulator_ik {

ManipulatorIK::ManipulatorIK (ros::NodeHandle& nh) {

	// ROS node handle
	nodeHandle_ = nh;

	// URDF parameter for TracIK
	nodeHandle_.param("urdf_param", urdf_param_, std::string("/robot_description"));

	// Get parameters from config file
	nodeHandle_.getParam("/manipulator_planning/chain_start", chain_start_);
	nodeHandle_.getParam("/manipulator_planning/chain_end", chain_end_);
	nodeHandle_.getParam("/manipulator_planning/timeout", timeout_);
	nodeHandle_.getParam("/manipulator_planning/num_samples", num_samples_);
	nodeHandle_.getParam("/manipulator_planning/joints_names", jointNames_);
	
	// Publishers
	jointCommandPublisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("/manipulator_planning/desired_joint_positions", 100);
	jointStatePublisher_ = nodeHandle.advertise<sensor_msgs::JointState>("/manipulator_planning/joint_states", 100);

	// Subscriber
	desiredEEPosSubscriber_ = nodeHandle_.subscribe("/manipulator_vision/desired_ee_pos", 100, &ManipulatorIK::desiredEEPosCallBack, this);

	// RViz
	pointMarker_ = getSphereMarker("base_link","gripper_pos", 0, 0.02, 1.0, 0.8, 0.4, 1.0);
	pointMarkerPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("/manipulator_planning/gripper_position_vis", 10);

}

void ManipulatorIK::publishMarker (const geometry_msgs::PoseStamped &pos) {

	pointMarker_.pose.position.x = pos.pose.position.x;
	pointMarker_.pose.position.y = pos.pose.position.y;
	pointMarker_.pose.position.z = pos.pose.position.z;

	pointMarkerPublisher_.publish(pointMarker_);
}

sensor_msgs::JointState ManipulatorIK::jointAnglesConverter (const std::vector<double> &joint_angles)
{
	sensor_msgs::JointState joint_msg;

	joint_msg.header.stamp = ros::Time::now();

	for (int i = 0; i < joint_angles.size(); ++i) {
		joint_msg.name.push_back(jointNamesArray[i]);
		joint_msg.position.push_back(joint_angles[i]);

	}

	return joint_msg;
}

void ManipulatorIK::publishJointCommand () {
	jointCommandPublisher_.publish(jointCommand_);
	jointStatePublisher_.publish(jointCommand_);   // visualization for testing!
}

}
