#include "tracik_planning/ik_tests.h"


#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_conversions/kdl_msg.h>
// #include <baxter_core_msgs/JointCommand.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

std::string chain_start, chain_end, urdf_param;
double timeout = 0.005;
double eps = 1e-5;
ros::Publisher jointCommandPublisher;
ros::Publisher pointMarkerPublisher;
ros::Publisher jointStatePublisher;
sensor_msgs::JointState init_pos;
sensor_msgs::JointState jointCommand;
std::string jointNamesArray[] ={"joint_base_mancal","joint_mancal_link1","joint_link1_link2","joint_link2_link3","joint_link3_gripper","joint_gripper_air"};
  
void jointInitAngles()
{
  jointCommand.header.stamp = ros::Time::now();

  for (int i = 0; i < 4; ++i) {
    jointCommand.name.push_back(jointNamesArray[i]);
    jointCommand.position.push_back(0.0);

  }
}

sensor_msgs::JointState jointAnglesConverter (const std::vector<double> &joint_angles)
{
  sensor_msgs::JointState joint_msg;

  joint_msg.header.stamp = ros::Time::now();

  for (int i = 0; i < joint_angles.size(); ++i) {
    joint_msg.name.push_back(jointNamesArray[i]);
    joint_msg.position.push_back(joint_angles[i]);

  }

  return joint_msg;
}

// Computes IK with limits considerations. 
// Inputs: KDL::Frame        pose
//         const std::string limbName 
// Output: publishes solution.
void findIKSolution(const KDL::Frame &end_effector_pose,ros::NodeHandle& nh, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param)
{

  // This constructor parses the URDF loaded in rosparm urdf_param into the
  // needed KDL structures.  We then pull these out to compare against the KDL
  // IK solver.
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

  KDL::Chain chain;
  KDL::JntArray ll, ul;                           // lower joint limits, upper joint limits

  bool valid = tracik_solver.getKDLChain(chain);
  if (!valid) {
    ROS_ERROR("There was no valid KDL chain found");
    return;
  }

  valid = tracik_solver.getKDLLimits(ll,ul);
  if (!valid) {
    ROS_ERROR("There were no valid KDL joint limits found");
    return;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  ROS_INFO ("Using %d joints",chain.getNrOfJoints());


  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());
  for (uint j=0; j<nominal.data.size(); j++) {
    nominal(j) = (ll(j)+ul(j))/2.0;
  }

  KDL::JntArray result;
  // result count
  int rc;

  rc=tracik_solver.CartToJnt(nominal,end_effector_pose,result);
  ROS_INFO("Found %d solution", rc);

  if (rc > 0) {
      /* In the solution, the order of joint angles is s->e->w,
       * while baxter_core_msgs::JointCommand requires the order of e->s->w.
       * So we have to make some conversions.
      */
      //std::string jointNamesArray[] = {limbName + "_s0", limbName + "_s1", limbName + "_e0", limbName + "_e1",
      //    limbName + "_w0", limbName + "_w1", limbName + "_w2"};
  
     
      std::vector<std::string> jointNamesVector(chain.getNrOfJoints());
      std::cout << chain.getNrOfJoints() << std::endl;
      for( std::size_t j = 0; j < chain.getNrOfJoints(); ++j)
          jointNamesVector[j] = jointNamesArray[j];

      jointCommand.header.stamp = ros::Time::now();
      jointCommand.name = jointNamesVector;
      jointCommand.position.resize(chain.getNrOfJoints());
      for( std::size_t j = 0; j < chain.getNrOfJoints(); ++j){
          jointCommand.position[j] = result(j);
          printf("\n%f\n",result(j));
      }
      // The conversion has done, publishing the joint command
      jointCommandPublisher.publish(jointCommand);
  }
}

visualization_msgs::Marker getSphereMarker (const std::string& frame_id,
                                            const std::string& ns,
                                            const int id,
                                            const double scale,
                                            const float red,
                                            const float green,
                                            const float blue,
                                            const float a) {
	visualization_msgs::Marker marker;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.scale.x = scale;
	marker.scale.y = scale;
	marker.scale.z = scale;
	marker.color.a = a;
	marker.color.r = red;
	marker.color.g = green;
	marker.color.b = blue;
	marker.id = id;
	marker.ns = ns;
	marker.header.stamp = ros::Time::now();
	marker.header.frame_id = frame_id;

	return marker;
}

//----------------------------------------------------------------------------------------------
// Compute the Inverse Kinematics 
//----------------------------------------------------------------------------------------------
void callBack(const geometry_msgs::PoseStamped &callBackData) {
  // ROS_INFO("Callbacking...");
  KDL::Frame end_effector_pose;
  tf::poseMsgToKDL(callBackData.pose, end_effector_pose);

  // Compute the IK Solution and publish the solution from within the method

  ros::NodeHandle nh;

  int num_samples;

  nh.param("chain_start", chain_start, std::string("base_link"));
  nh.param("chain_end", chain_end, std::string("air"));
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));
  //nh.param("num_samples", num_samples, 1000);
  nh.param("timeout", timeout, 0.005);
  
  findIKSolution(end_effector_pose,nh, chain_start, chain_end, timeout,urdf_param);

  visualization_msgs::Marker pointMarker = getSphereMarker("base_link","gripper_pos", 0, 0.02, 1.0, 0.8, 0.4, 1.0);

  pointMarker.pose.position.x = callBackData.pose.position.x;
  pointMarker.pose.position.y = callBackData.pose.position.y;
  pointMarker.pose.position.z = callBackData.pose.position.z;

  pointMarkerPublisher.publish(pointMarker);

}



int main(int argc, char** argv)
{
  srand(1);
  ros::init(argc, argv, "trac_ik_test");

  ros::NodeHandle nodeHandle;
  ros::Subscriber subscriber = nodeHandle.subscribe("end_effector_jevois_pose", 1, callBack);
  
  
  jointCommandPublisher = nodeHandle.advertise<sensor_msgs::JointState>("end_effector_command_solution", 1);
  pointMarkerPublisher = nodeHandle.advertise<visualization_msgs::Marker>("gripper_position_vis", 1);
  jointStatePublisher = nodeHandle.advertise<sensor_msgs::JointState>("/manipulator/joint_states", 1000);
  jointInitAngles();
  
  ros::Rate loop_rate(500);
  while (ros::ok()) {

	  jointStatePublisher.publish(jointCommand);

	  ros::spinOnce();
    loop_rate.sleep();
    
  }
 
  return 0;
}