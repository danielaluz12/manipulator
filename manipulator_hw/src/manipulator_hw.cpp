#include "manipulator_hw/manipulator_hw.h"
#include <pluginlib/class_list_macros.h>

namespace manipulator_hw
{

manipulatorHW::manipulatorHW(){

}

manipulatorHW::~manipulatorHW(){

}

bool manipulatorHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
{
  ROS_INFO("Initializing Manipulator hardware interface");
  
  nh_ = robot_hw_nh;

  // Get joint names and num of joint
  nh_.getParam("/hardware_interface/joints", joint_names_);
  num_joints_ = joint_names_.size();
  
  // get joint names
  // if (!nh_.getParam("joints", joint_names_))
  // {
  //     ROS_ERROR("No joints given in the namespace: %s.", nh_.getNamespace().c_str());
  //     return false;
  // }

	// Print for debug
	std::cout << "[Manipulator HW] Joint names: ";
	for (int i = 0; i < num_joints_; i++) {
	 	 std::cout << joint_names_[i] << " ";
	}
	std::cout << "\n" << std::endl;

  // Resize vectors
  joint_position_state_.resize(num_joints_);
  joint_velocity_state_.resize(num_joints_);
  joint_effort_state_.resize(num_joints_);
  joint_pos_command_.resize(num_joints_);

  // Register handles
    for(int i = 0; i < num_joints_; i++) {
        //State
        hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_state_[i], &joint_velocity_state_[i], &joint_effort_state_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        //Command
        hardware_interface::JointHandle JointHandle(jointStateHandle, &joint_pos_command_[i]);
        joint_pos_interface_.registerHandle(JointHandle);
    }
  
  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&joint_pos_interface_);

  raspberry_sub_ = root_nh.subscribe("/manipulator/raspberry/joint_states", 100, &manipulatorHW::raspberryDataCallback, this);
  raspberry_pub_ = root_nh.advertise<sensor_msgs::JointState>("/manipulator/raspberry/joint_command", 100);

  return true;
}

void manipulatorHW::raspberryDataCallback (const sensor_msgs::JointState &msg) {
	raspberry_state_msg_ = msg;
}

void manipulatorHW::read(const ros::Time &time, const ros::Duration &period)
{

	// Using data from RaspberryPi
  // std::cout << "joint_pos_command_ = [";
	for (int i = 0; i < num_joints_; i++) {
    // std::cout << joint_pos_command_[i] << ", ";
		//joint_position_state_[i] = raspberry_state_msg_.position[i]; // with raspberry
		joint_position_state_[i] = joint_pos_command_[i]; // without raspberry
		joint_velocity_state_[i] = 0;
		joint_effort_state_[i] = 0;
	}
  // std::cout << "]" << std::endl;
    
	// Print for debug
	// std::cout << "[Manipulator HW] Joint_positions: ";
	// for (int i = 0; i < num_joints_; i++) {
	// 	 std::cout << joint_position_state_[i] << " ";
	// }
	// std::cout << "\n" << std::endl;

}

void manipulatorHW::write(const ros::Time &time, const ros::Duration &period)
{
	sensor_msgs::JointState cmd;

	cmd.header.stamp = ros::Time::now();

	for (int i = 0; i < num_joints_; ++i) {
		cmd.name.push_back(joint_names_[i]);
		cmd.position.push_back(joint_pos_command_[i]);
		cmd.velocity.push_back(0.0);
		cmd.effort.push_back(0.0);
	}

	raspberry_pub_.publish(cmd);
}

}

PLUGINLIB_EXPORT_CLASS(manipulator_hw::manipulatorHW, hardware_interface::RobotHW)
