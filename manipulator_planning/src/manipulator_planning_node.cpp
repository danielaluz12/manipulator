#include "manipulator_planning/ManipulatorIK.h"

int main(int argc, char** argv)
{
	// Init ROS node
	ros::init(argc, argv, "manipulator_planning");
	ros::NodeHandle nodeHandle;

	ManipulatorIK robotIK (nodeHandle);

	ros::Rate loop_rate(500);
	while (ros::ok()) {

		robotIK.publishJointCommand();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
