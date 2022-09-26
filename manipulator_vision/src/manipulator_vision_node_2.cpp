#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf/message_filter.h"
#include <visualization_msgs/Marker.h>

ros::Subscriber PoseSubs;
ros::Publisher TransPosePub;
visualization_msgs::Marker pointMarker_;
ros::Publisher pointMarkerPublisher_;

geometry_msgs::PoseStamped objectPoseInBaseFrame;
geometry_msgs::PoseStamped objectPoseInCameraFrame;


visualization_msgs::Marker getSphereMarker(const std::string& frame_id, const std::string& ns, const int id, const double scale, const float red, const float green, const float blue, const float a){
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

void publishMarker() {

	pointMarker_.pose.position.x = objectPoseInBaseFrame.pose.position.x;
	pointMarker_.pose.position.y = objectPoseInBaseFrame.pose.position.y;
	pointMarker_.pose.position.z = objectPoseInBaseFrame.pose.position.z;
		
	pointMarkerPublisher_.publish(pointMarker_);
}


//  Callback 
void PoseCallBack(const geometry_msgs::PoseStamped& pose_in)
{
	objectPoseInCameraFrame.header.frame_id = "/camera";
	objectPoseInCameraFrame.header.stamp = ros::Time::now();
	objectPoseInCameraFrame.pose.position.x = pose_in.pose.position.x/1000;
	objectPoseInCameraFrame.pose.position.y = pose_in.pose.position.y/1000;
	objectPoseInCameraFrame.pose.position.z = pose_in.pose.position.z/1000;

	objectPoseInCameraFrame.pose.orientation.x = pose_in.pose.orientation.x;
	objectPoseInCameraFrame.pose.orientation.y = pose_in.pose.orientation.y;
	objectPoseInCameraFrame.pose.orientation.z = pose_in.pose.orientation.z;
	objectPoseInCameraFrame.pose.orientation.w = pose_in.pose.orientation.w;


}

void transformAndPusblishObjectPose () {

	static tf::TransformBroadcaster br;
	tf::Transform transformCameraToObject;
	tf::Transform transformBaseToObject;
	tf::Quaternion objectOrientationInCameraFrame;
	tf::Quaternion objectOrientationInBaseFrame;

	try 
	{
		tf::TransformListener tf_;
		const std::string target_frame_ ="/base_link";


		tf_.waitForTransform("/base_link","/camera", ros::Time(), ros::Duration(5.0));
		tf_.transformPose(target_frame_, objectPoseInCameraFrame, objectPoseInBaseFrame);
	}
	catch (tf2::TransformException &ex) 
	{
		ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
	}

		// object in camera frame
		objectOrientationInCameraFrame.setX(objectPoseInCameraFrame.pose.orientation.x);
    objectOrientationInCameraFrame.setY(objectPoseInCameraFrame.pose.orientation.y);
    objectOrientationInCameraFrame.setZ(objectPoseInCameraFrame.pose.orientation.z);
    objectOrientationInCameraFrame.setW(objectPoseInCameraFrame.pose.orientation.w);

		tf::Vector3 positionCameraToObject(objectPoseInCameraFrame.pose.position.x,
																			 objectPoseInCameraFrame.pose.position.y,
																			 objectPoseInCameraFrame.pose.position.z);

		transformCameraToObject.setRotation(objectOrientationInCameraFrame);
    transformCameraToObject.setOrigin(positionCameraToObject);
    br.sendTransform(tf::StampedTransform(transformCameraToObject, ros::Time::now(), "camera", "object1" ));

		objectPoseInBaseFrame.pose.orientation.x = 0;
		objectPoseInBaseFrame.pose.orientation.y = 0;
		objectPoseInBaseFrame.pose.orientation.z = 0;
		objectPoseInBaseFrame.pose.orientation.w = 1;

		// object in base frame
		objectOrientationInBaseFrame.setX(objectPoseInBaseFrame.pose.orientation.x);
    objectOrientationInBaseFrame.setY(objectPoseInBaseFrame.pose.orientation.y);
    objectOrientationInBaseFrame.setZ(objectPoseInBaseFrame.pose.orientation.z);
    objectOrientationInBaseFrame.setW(objectPoseInBaseFrame.pose.orientation.w);

		tf::Vector3 positionBaseToObject(objectPoseInBaseFrame.pose.position.x, 
																		 objectPoseInBaseFrame.pose.position.y,
																		 objectPoseInBaseFrame.pose.position.z);

		transformBaseToObject.setRotation(objectOrientationInBaseFrame);
    transformBaseToObject.setOrigin(positionBaseToObject);
    br.sendTransform(tf::StampedTransform(transformBaseToObject, ros::Time::now(), "base_link", "object2" ));

		objectPoseInBaseFrame.header.stamp = ros::Time::now();
		TransPosePub.publish(objectPoseInBaseFrame);

	  publishMarker();
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "manipulator_vision_node_2"); //Init ROS
	ros::NodeHandle n_;

	PoseSubs = n_.subscribe("/end_effector_jevois_pose", 100, &PoseCallBack);
	TransPosePub = n_.advertise<geometry_msgs::PoseStamped>("/manipulator_vision/transformed_pose", 100);
	// PoseDrawer pd; //Construct class
	pointMarker_ = getSphereMarker ("base_link","gripper_pos", 0, 0.02, 1.0, 0.8, 0.4, 1.0);
	pointMarkerPublisher_ = n_.advertise<visualization_msgs::Marker>("/manipulator_planning/gripper_position_vis", 10);
	
	ros::Rate loop_rate(150);

	while (ros::ok()) {

		transformAndPusblishObjectPose();
		ros::spinOnce();
		loop_rate.sleep();
	}

	
	return 0;
};