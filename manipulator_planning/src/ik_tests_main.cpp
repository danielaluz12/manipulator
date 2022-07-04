#include "tracik_planning/ik_tests.h"

int main(int argc, char** argv)
{
  srand(1);
  armIK robot;
  ros::init(argc, argv, "trac_ik_test");

  ros::NodeHandle nodeHandle;
  subscriber= nodeHandle.subscribe("end_effector_jevois_pose", 1, &armIK::callBack);
  
  
  jointCommandPublisher = nodeHandle.advertise<sensor_msgs::JointState>("end_effector_command_solution", 1);
  pointMarkerPublisher = nodeHandle.advertise<visualization_msgs::Marker>("gripper_position_vis", 1);
  jointStatePublisher = nodeHandle.advertise<sensor_msgs::JointState>("/manipulator/joint_states", 1000);
  robot.jointInitAngles();
  
  ros::Rate loop_rate(500);
  while (ros::ok()) {

    robot.jointStatePublisher.publish(jointCommand);

    ros::spinOnce();
    loop_rate.sleep();
    
  }
 
  return 0;
}