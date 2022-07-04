#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_conversions/kdl_msg.h>
// #include <baxter_core_msgs/JointCommand.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher jointCommandPublisher;
ros::Publisher pointMarkerPublisher;
ros::Publisher jointStatePublisher;
sensor_msgs::JointState init_pos;
sensor_msgs::JointState jointCommand;
ros::Subscriber subscriber; 

void callBack(const geometry_msgs::PoseStamped &callBackData);

std::string jointNamesArray[] ={"joint_base_mancal","joint_mancal_link1","joint_link1_link2",
                                "joint_link2_link3","joint_link3_gripper","joint_gripper_air"};

namespace arm_ik {

std::string chain_start, chain_end, urdf_param;
double timeout = 0.005;
double eps = 1e-5;


class armIK 
{
  public:

     /*!
      * Constructor.
      */
    // armIK(ros::NodeHandle& nh, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param)

    // Destructor.
    // virtual ~armIK();

    void jointInitAngles();

    
    virtual void findIKSolution(const KDL::Frame &end_effector_pose);
    
    virtual sensor_msgs::JointState jointAnglesConverter (const std::vector<double> &joint_angles);

    

    visualization_msgs::Marker getSphereMarker (const std::string& frame_id,
                                            const std::string& ns,
                                            const int id,
                                            const double scale,
                                            const float red,
                                            const float green,
                                            const float blue,
                                            const float a);
    
 
};

}