#include <vector>
#include <iostream>

#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_conversions/kdl_msg.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

namespace manipulator_ik {

class ManipulatorIK 
{
	public:

		// Constructor
		ManipulatorIK (ros::NodeHandle& nh);

		// Destructor
		~ManipulatorIK() = default;

		// Compute the inverse kinematics
		sensor_msgs::JointState findIKSolution(const KDL::Frame &end_effector_pose);

		// Publish joint command
		void publishJointCommand ();

	private:

		// ROS node handle
		ros::NodeHandle nodeHandle_;

		// Joints names
		std::vector<std::string> jointNames_;

		// Publishs marker for end-effector position visualization in RViz
		visualization_msgs::Marker pointMarker_;
		ros::Publisher pointMarkerPublisher_;
		void publishMarker (const geometry_msgs::PoseStamped &pos);
		visualization_msgs::Marker getSphereMarker (const std::string& frame_id, const std::string& ns, const int id, const double scale, const float red, const float green, const float blue, const float a);

		// Publishs joint command from the inverse kinematics
		sensor_msgs::JointState jointCommand_;
		ros::Publisher jointCommandPublisher_;
		
		// Converts joint angles' vector in a joint message
		sensor_msgs::JointState jointAnglesConverter (const std::vector<double> &joint_angles);

		// Set initial joint angles
		void jointInitAngles();
		
		// Gets the desired end-effector position from the Jevois Camera
		ros::Subscriber desiredEEPosSubscriber_;
		void desiredEEPosCallBack(const geometry_msgs::PoseStamped &msg); 

		// Publishes joint_states for RViz visualization (used only for testing)
		ros::Publisher jointStatePublisher_;

		// TracIK variables
		double eps_ = 1e-5;
		std::string chain_start_;
		std::string chain_end_;
		std::string urdf_param_;
		int num_samples_;
		double timeout_;

 
};

}
