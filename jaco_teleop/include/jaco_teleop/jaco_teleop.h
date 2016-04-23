#ifndef JACO_TELEOP_H
#define JACO_TELEOP_H

#include <ros/ros.h>
#include <jaco_msgs/FullJointVelocity.h>
#include <jaco_msgs/FullVelocity.h>
#include <jaco_msgs/HomeArm.h>
#include <jaco_msgs/Start.h>
#include <jaco_msgs/Stop.h>
#include <sensor_msgs/Joy.h>

//Control modes
#define CARTESIAN_CONTROL 0 
#define JOINT_CONTROL 1


#define MAX_TRANS_VEL .175
#define MAX_ANG_VEL 1.047
#define MAX_ROT_VEL_WRIST 12.000
#define MAX_ROT_VEL_ARM 8.000
#define MAX_FINGER_VEL 1500.0


class jaco_teleop
{
  public:

  jaco_teleop();

  void publish_velocity();

  private:

  void joy_cback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle node; 

  ros::Publisher joint_cmd_; 
  ros::Publisher cartesian_cmd_; 
  ros::Publisher finger_cmd_;
  ros::Subscriber joy_sub_; 

  ros::ServiceClient stop_client_;
  ros::ServiceClient start_client_;
  ros::ServiceClient homing_client_;
  
  jaco_msgs::FullVelocity cartesian_vel;
  jaco_msgs::FullJointVelocity joint_vel; 

  
  jaco_msgs::FullVelocity last_cartesian_vel;
  jaco_msgs::FullJointVelocity last_joint_vel; 

  int mode;


  double linear_throttle_factor; 
  double angular_throttle_factor; 
  double finger_throttle_factor; 


};


int main(int argc, char **argv);


#endif