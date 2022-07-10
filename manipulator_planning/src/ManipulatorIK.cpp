#include "manipulator_planning/ManipulatorIK.h"

namespace manipulator_ik {

ManipulatorIK::ManipulatorIK(ros::NodeHandle& nh) {

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
	jointCommandCMPublisher_ = nodeHandle_.advertise<trajectory_msgs::JointTrajectory>("/arm_position_controller/command", 100);
	// jointCommandPublisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("/arm_position_controller/command", 100); 
	// jointStatePublisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("/manipulator_planning/joint_states", 100);

	// Subscriber
	desiredEEPosSubscriber_ = nodeHandle_.subscribe("/manipulator_vision/desired_ee_pos", 100, &ManipulatorIK::desiredEEPosCallBack, this);

	// RViz
	pointMarker_ = getSphereMarker ("base_link","gripper_pos", 0, 0.02, 1.0, 0.8, 0.4, 1.0);
	pointMarkerPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("/manipulator_planning/gripper_position_vis", 10);

//	std::cout << chain_start_ << std::endl;
//	std::cout << chain_end_ << std::endl;
//	std::cout << timeout_ << std::endl;
//	std::cout << timeout_<< std::endl;

//
//	for (int i = 0; i < jointNames_.size(); ++i) {
//		// joint_msg.name.push_back(jointNames_);
//		std::cout << jointNames_[i] << std::endl;
//	}
}

visualization_msgs::Marker ManipulatorIK::getSphereMarker(const std::string& frame_id, const std::string& ns, const int id, const double scale, const float red, const float green, const float blue, const float a){
	visualization_msgs::Marker marker;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.scale.x = scale;
	marker.scale.y = scale;
	marker.scale.z = scale;
	marker.color.a = a;
	marker.color.r = red;
	marker.color.g = green;
	marker.color.b = blue;
	marker.id = id;
	marker.ns = ns;
	marker.header.stamp = ros::Time::now();
	marker.header.frame_id = frame_id;

	return marker;
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
		// joint_msg.name.push_back(jointNames_);
		joint_msg.position.push_back(joint_angles[i]);

	}

	return joint_msg;
}

void ManipulatorIK::publishJointCommand() {

	//função pra converter joinstates p trajectory(jointCommandCM_)
	
	jointCommandCMPublisher_.publish(jointCommandCM_);
	// convertCommandCM();

	// jointCommandPublisher_.publish(jointCommand_);

	// jointStatePublisher_.publish(jointCommand_);   // visualization for testing!
}

void ManipulatorIK::findIKSolution(const KDL::Frame &end_effector_pose) {
	TRAC_IK::TRAC_IK tracik_solver(chain_start_, chain_end_, urdf_param_, timeout_, eps_);

	KDL::Chain chain;
	KDL::JntArray ll, ul;                           // lower joint limits, upper joint limits

	bool valid = tracik_solver.getKDLChain(chain);
	if (!valid) {
		ROS_ERROR("There was no valid KDL chain found");
		return;
	}

	valid = tracik_solver.getKDLLimits(ll,ul);
	if (!valid) {
		ROS_ERROR("There were no valid KDL joint limits found");
		return;
	}

	assert(chain.getNrOfJoints() == ll.data.size());
	assert(chain.getNrOfJoints() == ul.data.size());

	ROS_INFO ("Using %d joints",chain.getNrOfJoints());


	// Create Nominal chain configuration midway between all joint limits
	KDL::JntArray nominal(chain.getNrOfJoints());
	for (uint j=0; j<nominal.data.size(); j++) {
		nominal(j) = (ll(j)+ul(j))/2.0;
	}

	KDL::JntArray result;
	int rc;

	rc=tracik_solver.CartToJnt(nominal,end_effector_pose,result);
	ROS_INFO("Found %d solution", rc);

	if (rc > 0) {

		jointNames_.resize(chain.getNrOfJoints());

		// jointCommand_.header.stamp = ros::Time::now();

		// // jointCommand_.name = jointNames_;
		// jointCommand_.name.push_back("joint_base_mancal");
    // jointCommand_.name.push_back("joint_mancal_link1");
    // jointCommand_.name.push_back("joint_link1_link2");
    // jointCommand_.name.push_back("joint_link2_link3");
    // jointCommand_.name.push_back("joint_link3_gripper");

		jointCommand_.position.resize(chain.getNrOfJoints());
		for( std::size_t j = 0; j < chain.getNrOfJoints(); ++j){
				jointCommand_.position[j] = result(j);
				// printf("\n%f\n",jointCommand_.position[j]);
		}

		// jointCommandCM_.joint_names= jointCommand_.name;
		jointCommandCM_.header.stamp = ros::Time::now();
		jointCommandCM_.joint_names.push_back("joint_base_mancal");
    jointCommandCM_.joint_names.push_back("joint_mancal_link1");
    jointCommandCM_.joint_names.push_back("joint_link1_link2");
    jointCommandCM_.joint_names.push_back("joint_link2_link3");
    jointCommandCM_.joint_names.push_back("joint_link3_gripper");


    // We will have two waypoints in this goal trajectory
    jointCommandCM_.points.resize(2);

    // First trajectory point
    // Positions
    int ind = 0;
		for( std::size_t j = 0; j < 5; ++j){
				jointCommandCM_.points[ind].positions.resize(5);
				jointCommandCM_.points[ind].positions[j] = 0.0;
		}
   
    // Velocities
    jointCommandCM_.points[ind].velocities.resize(5);
    for (size_t j = 0; j < 5; ++j)
    {
      jointCommandCM_.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    jointCommandCM_.points[ind].time_from_start = ros::Duration(1.0);

    // Second trajectory point
    // Positions
    ind += 1;
		for( std::size_t j = 0; j < 5; ++j){
				jointCommandCM_.points[ind].positions.resize(5);
				jointCommandCM_.points[ind].positions[j] = jointCommand_.position[j];
				// printf("\n%f\n",jointCommandCM_.points[ind].positions[j]);
		}
    
    // Velocities
    jointCommandCM_.points[ind].velocities.resize(5);
    for (size_t j = 0; j < 5; ++j)
    {
      jointCommandCM_.points[ind].velocities[j] = 0.0;
    }
    // To be reached 2 seconds after starting along the trajectory
    jointCommandCM_.points[ind].time_from_start = ros::Duration(2.0);

	}

}

// void ManipulatorIK::convertCommandCM(){

// 		jointCommandCM_.joint_names= jointNames_;
// 		jointCommandCM_.header.stamp = ros::Time::now();
//     // goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
//     // goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
//     // goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
//     // goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
//     // goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
//     // goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

//     // We will have two waypoints in this goal trajectory
//     jointCommandCM_.points.resize(2);

//     // First trajectory point
//     // Positions
//     int ind = 0;
// 		for( std::size_t j = 0; j < 5; ++j){
// 				jointCommandCM_.points[ind].positions.resize(5);
// 				jointCommandCM_.points[ind].positions[j] = 0.0;
// 		}
   
//     // Velocities
//     jointCommandCM_.points[ind].velocities.resize(5);
//     for (size_t j = 0; j < 5; ++j)
//     {
//       jointCommandCM_.points[ind].velocities[j] = 0.0;
//     }
//     // To be reached 1 second after starting along the trajectory
//     jointCommandCM_.points[ind].time_from_start = ros::Duration(1.0);

//     // Second trajectory point
//     // Positions
//     ind += 1;
// 		for( std::size_t j = 0; j < 5; ++j){
// 				jointCommandCM_.points[ind].positions.resize(5);
// 				jointCommandCM_.points[ind].positions[j] = jointCommand_.position[j];
// 				// printf("\n%f\n",jointCommandCM_.points[ind].positions[j]);
// 		}
    
//     // Velocities
//     jointCommandCM_.points[ind].velocities.resize(5);
//     for (size_t j = 0; j < 5; ++j)
//     {
//       jointCommandCM_.points[ind].velocities[j] = 0.0;
//     }
//     // To be reached 2 seconds after starting along the trajectory
//     jointCommandCM_.points[ind].time_from_start = ros::Duration(3.0);

// }


void ManipulatorIK::desiredEEPosCallBack(const geometry_msgs::PoseStamped &msg) {
  // ROS_INFO("Callbacking...");
  KDL::Frame end_effector_pose;
  tf::poseMsgToKDL(msg.pose, end_effector_pose);

  // Compute the IK Solution and publish the solution from within the method
  findIKSolution(end_effector_pose);
  publishMarker(msg);

}


}
