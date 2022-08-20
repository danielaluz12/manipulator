#include <iostream> 
#include <ros/ros.h>
#include <cstdio>
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <stdint.h>
#include <unistd.h> // UNIX standard function definitions
#include <boost/locale.hpp>
#include <codecvt>
#include <locale>
#include <string>
#include <cassert>
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls
#include <vector>

// ros::Publisher publish_ee_pos;

serial::Serial my_serial("/dev/ttyACM0",115200);


 void config_camera(){

	if(my_serial.isOpen())
    std::cout << " Yes." << std::endl;
  else
    std::cout << " No." << std::endl;

	std::string test_string="setpar serout USB\r\n";

	size_t bytes_wrote = my_serial.write(test_string);

	std::string result = my_serial.read(2);

	std::cout << "Bytes wrote: "  << bytes_wrote ;

	std::cout << "length" << result.length() << ", String read: " << result << std::endl;


  }



	
	

// 	# Initialize DemoArUco module
// 	ser.write("setmapping2 YUYV 640 480 20.0 JeVois DemoArUco\r".encode())
// 	out = ser.readline().rstrip()
// 	print(out)

// 	# Set 3D pose messages
// 	ser.write("setpar dopose true\r".encode())
// 	out = ser.readline().rstrip()
// 	print(out)

// 	# Set 3D pose messages
// 	ser.write("setpar markerlen 53\r".encode())
// 	out = ser.readline().rstrip()
// 	print(out)

// 	# Set 3D pose messages
// 	ser.write("setpar serprec 4\r".encode())
// 	out = ser.readline().rstrip()
// 	print(out)

// 	# Set Detail style messages
// 	ser.write("setpar serstyle Detail\r".encode())
// 	out = ser.readline().rstrip()
// 	print(out)

// 	# Set Detail style messages
// 	ser.write("setpar serstamp FrameTime\r".encode())
// 	out = ser.readline().rstrip()
// 	print(out)

// 	# # Start image stream
// 	ser.write("streamon\r".encode())
// 	out = ser.readline().rstrip()
// 	print(out)

// 	time.sleep(1)



// void initPoseMessages () {

// 	// First example position
// 	ee1.header.stamp = ros::Time::now();
// 	ee1.header.frame_id = "base_link";

// 	ee1.pose.position.x = 0.134411;
// 	ee1.pose.position.y = -0.154806;
// 	ee1.pose.position.z = 0.473348;

// 	ee1.pose.orientation.x = -0.408715;
// 	ee1.pose.orientation.y = 0.143526;
// 	ee1.pose.orientation.z = 0.289469;
// 	ee1.pose.orientation.w = 0.853558;

// 	// Second example position
// 	ee2.header.stamp = ros::Time::now();
// 	ee2.header.frame_id = "base_link";

// 	ee2.pose.position.x = -0.308071;
// 	ee2.pose.position.y = -0.382763;
// 	ee2.pose.position.z = 0.00767814;

// 	ee2.pose.orientation.x = 0.888864;
// 	ee2.pose.orientation.y = -0.177634;
// 	ee2.pose.orientation.z = 0.205209;
// 	ee2.pose.orientation.w = 0.369131;

// 	// Third example position
// 	ee3.header.stamp = ros::Time::now();
// 	ee3.header.frame_id = "base_link";

// 	ee3.pose.position.x = 0.0775239;
// 	ee3.pose.position.y = -0.147995;
// 	ee3.pose.position.z = 0.510453;

// 	ee3.pose.orientation.x = 0.32089;
// 	ee3.pose.orientation.y = 0.418266;
// 	ee3.pose.orientation.z = 0.425309;
// 	ee3.pose.orientation.w = 0.73566;
// }

// bool publishEEPos(manipulator_vision::RequestEEPos::Request  &req,
// 	   	   	   	  	manipulator_vision::RequestEEPos::Response &res)
// {
// 	if (req.data == 1)	{
// 		ee1.header.stamp = ros::Time::now();
// 		publish_ee_pos.publish(ee1);
// 	}
// 	if (req.data == 2)	
// 		ee1.header.stamp = ros::Time::now();
// 		publish_ee_pos.publish(ee2);
// 	if (req.data == 3)	
// 		ee1.header.stamp = ros::Time::now();
// 		publish_ee_pos.publish(ee3);

// 	res.success = true;

// 	return true;
// }

int main(int argc, char** argv)
{
	// Init ROS node
	ros::init(argc, argv, "manipulator_vision_trans");
	ros::NodeHandle nodeHandle;


	config_camera();

	// initPoseMessages();

	// publish_ee_pos = nodeHandle.advertise<geometry_msgs::PoseStamped>("/manipulator_vision/desired_ee_pos", 100);

	ros::spin();

//	ros::Rate loop_rate(500);
//	while (ros::ok()) {
//
//		ros::spinOnce();
//		loop_rate.sleep();
//	}

	return 0;
}
