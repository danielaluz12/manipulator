#include "manipulator_planning/ManipulatorIK.h"

int main(int argc, char** argv)
{
	// Init ROS node
	ros::init(argc, argv, "manipulator_ik_test");
	ros::NodeHandle nodeHandle;

	manipulator_ik::ManipulatorIK robotIK (nodeHandle);

	ros::Rate loop_rate(500);
	while (ros::ok()) {

		robotIK.publishJointCommand();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
