#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf/message_filter.h"

ros::Subscriber PoseSubs;
ros::Publisher TransPosePub;
	
//  Callback 
void PoseCallBack(const geometry_msgs::PoseStamped& pose_in)
{
	geometry_msgs::PoseStamped pose_out;
	try 
	{
		tf::TransformListener tf_;
		const std::string target_frame_ ="/base_link";


		tf_.waitForTransform("/base_link","/camera", ros::Time(), ros::Duration(5.0));
		tf_.transformPose(target_frame_,pose_in, pose_out);


		
		
		// printf("point Position(x:%f y:%f z:%f) Orientation(x:%f y:%f z:%f w:%f)\n", 
					// pose_out.pose.position.x,
					// pose_out.pose.position.y,
					// pose_out.pose.position.z,
					// pose_out.pose.orientation.x,
					// pose_out.pose.orientation.y,
					// pose_out.pose.orientation.z,
					// pose_out.pose.orientation.w);

		pose_out.pose.position.x = pose_out.pose.position.x/10;
		pose_out.pose.position.y = pose_out.pose.position.y/100;
		pose_out.pose.position.z = pose_out.pose.position.z/10;
		

		pose_out.pose.orientation.x=0.0;
		pose_out.pose.orientation.y=0.0;
		pose_out.pose.orientation.z =0.0;
		pose_out.pose.orientation.w =1.0;

	
		TransPosePub.publish(pose_out);


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
	ros::spin(); // Run until interupted 
	return 0;
};