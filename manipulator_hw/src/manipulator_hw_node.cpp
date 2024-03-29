#include "manipulator_hw/manipulator_hw.h"
#include <iostream>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "manipulator_hw_node");
  ros::NodeHandle nh;

  ROS_INFO("Initializing Manipulator hw node");

  manipulator_hw::manipulatorHW robot;
  if (!robot.init(nh, nh))
  {
    ROS_FATAL("Manipulator initialization failed");
    return 1;
  }
  
  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::Rate rate(500);
  ros::Time ts = ros::Time::now();
  
  ROS_INFO("Spinning");

  while(ros::ok())
  {
    ros::Duration d = ros::Time::now() - ts;
    ts = ros::Time::now();
  
    robot.read(ts, d);
    cm.update(ts, d);
    robot.write(ts, d);
        
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
