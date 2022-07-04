#include <iostream>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "normal_hi/normal_hw.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "normal_hw");
  ros::NodeHandle nh;

  ROS_INFO("Initializing GPG3 remote node");

  normalHW robot;
  if (!robot.init(nh, nh))
  {
    ROS_FATAL("GPG3 initialization failed");
    return 1;
  }
  
  
  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::Rate rate(200);
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