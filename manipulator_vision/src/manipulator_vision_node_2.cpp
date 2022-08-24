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
geometry_msgs::PoseStamped pose_out;
visualization_msgs::Marker pointMarker_;
ros::Publisher pointMarkerPublisher_;


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

	pointMarker_.pose.position.x = pose_out.pose.position.x/1000;
	pointMarker_.pose.position.y = pose_out.pose.position.y/1000;
	pointMarker_.pose.position.z = pose_out.pose.position.z/1000;
		
	pointMarkerPublisher_.publish(pointMarker_);
}


//  Callback 
void PoseCallBack(const geometry_msgs::PoseStamped& pose_in)
{
	
	static tf::TransformBroadcaster br;
	tf::Transform o_transform_c;
	tf::Transform o_transform_b;
	tf::Quaternion cameraOrientobj;
	tf::Quaternion baseOrientobj;



	// geometry_msgs::PoseStamped pose_out;

	try 
	{
		tf::TransformListener tf_;
		const std::string target_frame_ ="/base_link";


		tf_.waitForTransform("/base_link","/camera", ros::Time(), ros::Duration(5.0));
		tf_.transformPose(target_frame_,pose_in, pose_out);



		pose_out.pose.position.x = pose_out.pose.position.x/1000;
		pose_out.pose.position.y = pose_out.pose.position.y/1000;
		pose_out.pose.position.z = pose_out.pose.position.z/1000;
		

		// pose_out.pose.orientation.x=0.0;
		// pose_out.pose.orientation.y=0.0;
		// pose_out.pose.orientation.z =0.0;
		// pose_out.pose.orientation.w =1.0;


		cameraOrientobj.setX(pose_in.pose.orientation.x);
    cameraOrientobj.setY(pose_in.pose.orientation.y);
    cameraOrientobj.setZ(pose_in.pose.orientation.z);
    cameraOrientobj.setW(pose_in.pose.orientation.w);

		tf::Vector3 positionCameraToObject(pose_in.pose.position.x/1000, pose_in.pose.position.y/1000,pose_in.pose.position.z/1000);


		
		baseOrientobj.setX(pose_out.pose.orientation.x);
    baseOrientobj.setY(pose_out.pose.orientation.y);
    baseOrientobj.setZ(pose_out.pose.orientation.z);
    baseOrientobj.setW(pose_out.pose.orientation.w);

		tf::Vector3 positionBaseToObject(pose_out.pose.position.x, pose_out.pose.position.y,pose_out.pose.position.z);

		o_transform_c.setRotation(cameraOrientobj);
    o_transform_c.setOrigin(positionCameraToObject);
    br.sendTransform(tf::StampedTransform(o_transform_c, ros::Time::now(), "camera", "object1" ));


		o_transform_b.setRotation(baseOrientobj);
    o_transform_b.setOrigin(positionBaseToObject);
    br.sendTransform(tf::StampedTransform(o_transform_b, ros::Time::now(), "base_link", "object2" ));

		TransPosePub.publish(pose_out);

	  publishMarker();


	}
	catch (tf2::TransformException &ex) 
	{
		ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
	}
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
	ros::spin(); // Run until interupted 
	return 0;
};