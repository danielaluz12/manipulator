#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <iostream>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

namespace myrobot_hw
{

class normalHW : public hardware_interface::RobotHW
{
  public:
    //Constructor
    normalHW();

    //Destructor
    ~normalHW();

    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);
    void read(const ros::Time & time, const ros::Duration &period);

    void write(const ros::Time & time, const ros::Duration &period);
    

  private:

    ros::NodeHandle nh_;
  
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface joint_pos_interface;
  
    int num_joints;
    std::vector<std::string> joint_name;

    //states
    std::vector<double> joint_position_state;
    std::vector<double> joint_velocity_state;
    std::vector<double> joint_effort_state;

    //given setpoints
    std::vector<double> joint_pos_command;  
 
};
}
