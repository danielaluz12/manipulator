#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <iostream>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

namespace manipulator_hw
{

class normalHW : public hardware_interface::RobotHW
{
  public:
    // Constructor
    normalHW();

    // Destructor
    ~normalHW();

    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);
    void read(const ros::Time & time, const ros::Duration &period);

    void write(const ros::Time & time, const ros::Duration &period);

  private:

    ros::NodeHandle nh_;
  
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface joint_pos_interface_;
  
    int num_joints_;
    std::vector<std::string> joint_names_;

    // states
    std::vector<double> joint_position_state_;
    std::vector<double> joint_velocity_state_;
    std::vector<double> joint_effort_state_;

    // commands
    std::vector<double> joint_pos_command_;

    // subscribe to RaspberryPi to get joint positions
    sensor_msgs::JointState raspberry_state_msg_;
    ros::Subscriber raspberry_sub_;
    void raspberryDataCallback (const sensor_msgs::JointState &msg);

    // publish joint commands to RaspberryPi
    ros::Publisher raspberry_pub_;
 
};

}
