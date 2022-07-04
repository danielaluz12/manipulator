

#include "my_roscpp_library/my_serial_library.h"
#include "normal_hi/normal_hw.h"

namespace myrobots_hardware_interface
{

bool normalHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
{

  
  ROS_INFO("Initializing GPG3 hardware interface");
  
  nh_ = robot_hw_nh;

  //get joint names and num of joint
  nh_.getParam("joints", joint_name);
  num_joints = joint_name.size();

  //resize vectors
  joint_position_state.resize(num_joints);
  joint_velocity_state.resize(num_joints);
  joint_effort_state.resize(num_joints);
  joint_pos_command.resize(num_joints);

  //Register handles
    for(int i=0; i<num_joints; i++){
        //State
        hardware_interface::JointStateHandle jointStateHandle(joint_name[i], &joint_position_state[i], &joint_velocity_state[i], &joint_effort_state[i]);
        joint_state_interface.registerHandle(jointStateHandle);

        //Command
        hardware_interface::JointHandle JointHandle(jointStateHandle, &joint_pos_command[i]);
        joint_pos_interface.registerHandle(JointHandle);
    }
  
  //Register interfaces
  registerInterface(&joint_state_interface);
  registerInterface(&joint_pos_interface);


  return true;
}

void normalHW::read(const ros::Time &time, const ros::Duration &period)
{
    //read in rad 
    // serial.ReadPack(rmsg);  

    // pos_[0] = rmsg[0]* 0.01 -3; //pulse to rad
    // pos_[1] = rmsg[1]* 0.006 -1.2; //pulse to rad
    // pos_[2] = rmsg[2]* 0.007 -1.4; //pulse to rad
    // pos_[3] = rmsg[3]* 0.0157 -6.28; //pulse to rad
    // pos_[4] =  rmsg[4]* 0.0026 -0.52;
    // pos_[5] =  rmsg[5]* (-0.0026) +0.52; 

    
    // std::cout << " joint_position : ";
    // for (int i = 0; i < 6; i++) {
    //         std::cout << pos_[i] << " ";
    // }
    // std::cout << "\n" << std::endl; 

     
}

void normalHW::write(const ros::Time &time, const ros::Duration &period)
{
    //send in rad also
    // cmd_2[0]=int(cmd_[0]*100.0 + 300.0);  //j_base_mancal
    // cmd_2[1]=int(cmd_[1]*166.666 + 200.0); //j_mancal_elo1
    // cmd_2[2]=int(cmd_[2]*142.857 +200.0); // j_elo1_elo2
    // cmd_2[3]=int(cmd_[3]*63.694 + 399.999); // por mais  duas joint
    // cmd_2[4]=int(cmd_[4]*384.615+ 200.0);
    // cmd_2[5]=int(cmd_[5]*(-384.615)+ 200.0);


    // serial.SendPack(cmd_2);

    // std::cout << "joint command ";
    // for (int i = 0; i < 6; i++) {
    //         std::cout << cmd_[i] << " ";
    // }
    // std::cout << "\n" << std::endl; 

  
}
}
PLUGINLIB_EXPORT_CLASS(normalHW, hardware_interface::RobotHW)