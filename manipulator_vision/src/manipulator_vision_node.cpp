#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseStamped>

#include "manipulator_vision/RequestEEPos.h"

geometry_msgs::PoseStamped ee1;
geometry_msgs::PoseStamped ee2;
geometry_msgs::PoseStamped ee3;

ros::Publisher publish_ee_pos;

void initPoseMessages () {

	// First example position
	ee1.header.stamp = rospy.Time.now()
	ee1.header.frame_id = "base_link"

	ee1.pose.position.x = 0.134411
	ee1.pose.position.y = -0.154806
	ee1.pose.position.z = 0.473348

	ee1.pose.orientation.x = -0.408715
	ee1.pose.orientation.y = 0.143526
	ee1.pose.orientation.z = 0.289469
	ee1.pose.orientation.w = 0.853558

	// Second example position
	ee2.header.stamp = rospy.Time.now()
	ee2.header.frame_id = "base_link"

	ee2.pose.position.x = -0.308071
	ee2.pose.position.y = -0.382763
	ee2.pose.position.z = 0.00767814

	ee2.pose.orientation.x = 0.888864
	ee2.pose.orientation.y = -0.177634
	ee2.pose.orientation.z = 0.205209
	ee2.pose.orientation.w = 0.369131

	// Third example position
	ee3.header.stamp = rospy.Time.now()
	ee3.header.frame_id = "base_link"

	ee3.pose.position.x = 0.0775239
	ee3.pose.position.y = -0.147995
	ee3.pose.position.z = 0.510453

	ee3.pose.orientation.x = 0.32089
	ee3.pose.orientation.y = 0.418266
	ee3.pose.orientation.z = 0.425309
	ee3.pose.orientation.w = 0.73566
}

bool publishEEPos(std_srvs::Trigger::Request  &req,
	   	   	   	  std_srvs::Trigger::Response &res)
{
	if (req.data == 1)	
		publish_ee_pos.publish(ee1);
	if (req.data == 2)	
		publish_ee_pos.publish(ee2);
	if (req.data == 3)	
		publish_ee_pos.publish(ee3);

	res.success = true;

	return true;
}

int main(int argc, char** argv)
{
	// Init ROS node
	ros::init(argc, argv, "manipulator_vision");
	ros::NodeHandle nodeHandle;

	initPoseMessages();

	ros::ServiceServer publishEEPosService_ = nodeHandle.advertiseService("request_ee_pos", publishEEPos);

	publish_ee_pos = nodeHandle.advertise<geometry_msgs::PoseStamped>("/manipulator_vision/desired_ee_pos", 100);

	ros::spin();

//	ros::Rate loop_rate(500);
//	while (ros::ok()) {
//
//		ros::spinOnce();
//		loop_rate.sleep();
//	}

	return 0;
}
