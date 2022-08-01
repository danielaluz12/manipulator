#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "manipulator_vision/RequestEEPos.h"

geometry_msgs::PoseStamped ee1;
geometry_msgs::PoseStamped ee2;
geometry_msgs::PoseStamped ee3;

ros::Publisher publish_ee_pos;
ros::Subscriber JevoisTransform;
tf::TransformListener listener;

geometry_msgs::TransformStamped transformStamped;
tf::Transform btTrans;

tf::StampedTransform stampedTF;
// static tf2_ros::TransformBroadcaster br;
//tf2_ros::TransformBroadcaster tf_broadcaster;


tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);
geometry_msgs::PoseStamped received_pose;

void initPoseMessages () {

	// First example position
	ee1.header.stamp = ros::Time::now();
	ee1.header.frame_id = "base_link";

	ee1.pose.position.x = 0.134411;
	ee1.pose.position.y = -0.154806;
	ee1.pose.position.z = 0.473348;

	ee1.pose.orientation.x = -0.408715;
	ee1.pose.orientation.y = 0.143526;
	ee1.pose.orientation.z = 0.289469;
	ee1.pose.orientation.w = 0.853558;

	// Second example position
	ee2.header.stamp = ros::Time::now();
	ee2.header.frame_id = "base_link";

	ee2.pose.position.x = -0.308071;
	ee2.pose.position.y = -0.382763;
	ee2.pose.position.z = 0.00767814;

	ee2.pose.orientation.x = 0.888864;
	ee2.pose.orientation.y = -0.177634;
	ee2.pose.orientation.z = 0.205209;
	ee2.pose.orientation.w = 0.369131;

	// Third example position
	ee3.header.stamp = ros::Time::now();
	ee3.header.frame_id = "base_link";

	ee3.pose.position.x = 0.0775239;
	ee3.pose.position.y = -0.147995;
	ee3.pose.position.z = 0.510453;

	ee3.pose.orientation.x = 0.32089;
	ee3.pose.orientation.y = 0.418266;
	ee3.pose.orientation.z = 0.425309;
	ee3.pose.orientation.w = 0.73566;
}

bool publishEEPos(manipulator_vision::RequestEEPos::Request  &req,
	   	   	   	  	manipulator_vision::RequestEEPos::Response &res)
{
	if (req.data == 1)	{
		ee1.header.stamp = ros::Time::now();
		publish_ee_pos.publish(ee1);
	}
	if (req.data == 2)	
		ee1.header.stamp = ros::Time::now();
		publish_ee_pos.publish(ee2);
	if (req.data == 3)	
		ee1.header.stamp = ros::Time::now();
		publish_ee_pos.publish(ee3);

	res.success = true;

	return true;
}

 void JevoisTransformCallBack(const geometry_msgs::PoseStamped &msg)
 {

	received_pose.header.stamp = msg.header.stamp;
	received_pose.header.frame_id = msg.header.frame_id;
	received_pose.pose = msg.pose;
 }



int main(int argc, char** argv)
{
	// Init ROS node
	ros::init(argc, argv, "manipulator_vision_node");
	ros::NodeHandle nodeHandle;

	initPoseMessages();

	ros::ServiceServer publishEEPosService_ = nodeHandle.advertiseService("request_ee_pos", publishEEPos);

	JevoisTransform = nodeHandle.subscribe("/end_effector_jevois_pose", 100, &JevoisTransformCallBack);

	ros::Publisher PoseTransPub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/manipulator_vision/tf_ee_pos", 100);

	publish_ee_pos = nodeHandle.advertise<geometry_msgs::PoseStamped>("/manipulator_vision/desired_ee_pos", 100);

	ros::spin();
	ros::Rate rate(10.0);
	while (nodeHandle.ok()){
       
			try{
				geometry_msgs::PoseStamped pose_transformed ;
				transformStamped = tfBuffer.lookupTransform("base", "mancal",ros::Time(0));
				tf2::doTransform(received_pose.pose,pose_transformed.pose,transformStamped);
			}
			catch (tf2::TransformException &ex) {
				ROS_WARN("%s",ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}

			

			// pos_msg.pose.position.x = transformStamped.transform.translation.x;
			// pos_msg.pose.position.y = transformStamped.transform.translation.y;
			// pos_msg.pose.position.z = transformStamped.transform.translation.z;

			// pos_msg.pose.orientation.x = transformStamped.transform.rotation.x;
			// pos_msg.pose.orientation.y = transformStamped.transform.rotation.y;
			// pos_msg.pose.orientation.z = transformStamped.transform.rotation.z;
			// pos_msg.pose.orientation.w = transformStamped.transform.rotation.w;

	
      // PoseTransPub.publish(pos_msg);
   
			rate.sleep();
     }
//	ros::Rate loop_rate(500);
//	while (ros::ok()) {
//
//		ros::spinOnce();
//		loop_rate.sleep();
//	}

	return 0;
}
