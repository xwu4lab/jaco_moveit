#include <jaco_teleop/jaco_teleop.h>

using namespace std;

  jaco_teleop::jaco_teleop()
  {
	  ros::NodeHandle private_nh("~");
	
	  joint_cmd_ = node.advertise<jaco_msgs::FullJointVelocity>("jaco_arm_driver/in/full_joint_velocity",1);
	  cartesian_cmd_ = node.advertise<jaco_msgs::FullVelocity>("jaco_arm_driver/in/full_cartesian_velocity", 1);
	  joy_sub_ = node.subscribe<sensor_msgs::Joy>("joy", 1, &jaco_teleop::joy_cback, this);
	
	  stop_client_ = node.serviceClient<jaco_msgs::Stop>("jaco_arm_driver/in/stop");
	  start_client_ = node.serviceClient<jaco_msgs::Start>("jaco_arm_driver/in/start"); 
	  homing_client_ = node.serviceClient<jaco_msgs::HomeArm>("jaco_arm_driver/in/home_arm");;
	
	  private_nh.param<double>("linear_throttle_factor", linear_throttle_factor, 1.0);
	  private_nh.param<double>("angular_throttle_factor", angular_throttle_factor, 1.0);
	  private_nh.param<double>("finger_throttle_factor", finger_throttle_factor, 1.0);
	
	  mode = JOINT_CONTROL;
	
	  ROS_INFO("JACO joystick started");
  }

  void jaco_teleop::joy_cback(const sensor_msgs::Joy::ConstPtr& joy)
  {
	    if (joy->buttons.at(6) == 1)
	    {
		    jaco_msgs::Stop srv_stop;
		    srv_stop.request;
		    if (stop_client_.call(srv_stop))
		    {
			    ROS_INFO("%s" , srv_stop.response.stop_result.c_str());
		    }
		    else 
		    {
			    ROS_INFO("Failed to call STOP service");
		    }
	    }
	    if (joy->buttons.at(7) == 1)
	    {
		    jaco_msgs::Start srv_start;
		    srv_start.request;
		    if (start_client_.call(srv_start))
		    {
			    ROS_INFO("%s" , srv_start.response.start_result.c_str());
		    }
		    else 
		    {
			    ROS_INFO("Failed to call START service");
		    }
	    }	
		
	    if (joy->buttons.at(3) == 1)
	    {
		    jaco_msgs::HomeArm srv_homing;
		    srv_homing.request;
		    if (homing_client_.call(srv_homing))
		    {
			    ROS_INFO("%s" , srv_homing.response.homearm_result.c_str());
		    }
		    else 
		    {
			    ROS_INFO("Failed to call HomeArm service");
		    }
	    }
	   
		
		
	    switch (mode)

	    {
		    case CARTESIAN_CONTROL:
		
		    cartesian_vel.velocity.linear.x = joy->axes.at(0) * MAX_TRANS_VEL * linear_throttle_factor;
		    cartesian_vel.velocity.linear.y = joy->axes.at(1) * MAX_TRANS_VEL * linear_throttle_factor;
		    cartesian_vel.velocity.linear.z = joy->axes.at(3) * MAX_TRANS_VEL * linear_throttle_factor;
		
		    cartesian_vel.velocity.angular.x = joy->axes.at(6) * MAX_ANG_VEL * angular_throttle_factor;
		    cartesian_vel.velocity.angular.y = joy->axes.at(7) * MAX_ANG_VEL * angular_throttle_factor;
		    cartesian_vel.velocity.angular.z = joy->axes.at(4) * MAX_ANG_VEL * angular_throttle_factor;
		    
		    if (joy->buttons.at(5) == 0)
				{
					cartesian_vel.finger1 = joy->buttons.at(4) * MAX_FINGER_VEL * finger_throttle_factor;
					cartesian_vel.finger2 = joy->buttons.at(4) * MAX_FINGER_VEL * finger_throttle_factor;
					cartesian_vel.finger3 = joy->buttons.at(4) * MAX_FINGER_VEL * finger_throttle_factor;
				}
			else if (joy->buttons.at(5) ==1)
				{
					cartesian_vel.finger1 = -joy->buttons.at(5) * MAX_FINGER_VEL * finger_throttle_factor;
					cartesian_vel.finger2 = -joy->buttons.at(5) * MAX_FINGER_VEL * finger_throttle_factor;
					cartesian_vel.finger3 = -joy->buttons.at(5) * MAX_FINGER_VEL * finger_throttle_factor;
				}

//		    ROS_INFO("Cartesian velocity is : %f, %f, %f, %f, %f, %f", 
//                  cartesian_vel.twist.linear.x, cartesian_vel.twist.linear.y, cartesian_vel.twist.linear.z,
//                  cartesian_vel.twist.angular.x, cartesian_vel.twist.angular.y, cartesian_vel.twist.angular.z);
		
		    if (joy->buttons.at(JOINT_CONTROL) == 1)
		    {
				cartesian_vel.velocity.linear.x = 0.0;
				cartesian_vel.velocity.linear.y = 0.0;
				cartesian_vel.velocity.linear.z = 0.0;
		
				cartesian_vel.velocity.angular.x = 0.0;
				cartesian_vel.velocity.angular.y = 0.0;
				cartesian_vel.velocity.angular.z = 0.0;
				
				
				cartesian_vel.finger1 = 0.0;
				cartesian_vel.finger2 = 0.0;
				cartesian_vel.finger3 = 0.0;
            
				cartesian_cmd_.publish(cartesian_vel);
				mode = JOINT_CONTROL;
			
				ROS_INFO("Activated JOINT control mode");
			}
			
			break;
			
			case JOINT_CONTROL:
		
			joint_vel.joint1 = joy->axes.at(0) * MAX_ROT_VEL_ARM * angular_throttle_factor;
			joint_vel.joint2 = joy->axes.at(1) * MAX_ROT_VEL_ARM * angular_throttle_factor;
			joint_vel.joint3 = joy->axes.at(3) * MAX_ROT_VEL_ARM * angular_throttle_factor;
			joint_vel.joint4 = joy->axes.at(4) * MAX_ROT_VEL_WRIST * angular_throttle_factor;
			joint_vel.joint5 = joy->axes.at(6) * MAX_ROT_VEL_WRIST * angular_throttle_factor;
			joint_vel.joint6 = joy->axes.at(7) * MAX_ROT_VEL_WRIST * angular_throttle_factor;
			
			if (joy->buttons.at(5) == 0 )
				{
					joint_vel.finger1 = joy->buttons.at(4) * MAX_FINGER_VEL * finger_throttle_factor;
					joint_vel.finger2 = joy->buttons.at(4) * MAX_FINGER_VEL * finger_throttle_factor;
					joint_vel.finger3 = joy->buttons.at(4) * MAX_FINGER_VEL * finger_throttle_factor;
					
					
				}
			else if (joy->buttons.at(5) ==1)
				{
					joint_vel.finger1 = -joy->buttons.at(5) * MAX_FINGER_VEL * finger_throttle_factor;
					joint_vel.finger2 = -joy->buttons.at(5) * MAX_FINGER_VEL * finger_throttle_factor;
					joint_vel.finger3 = -joy->buttons.at(5) * MAX_FINGER_VEL * finger_throttle_factor;
					
				}

				
//			ROS_INFO("Joint velocity is : %f, %f, %f, %f, %f, %f", 
//                  joint_vel.joint1, joint_vel.joint2, joint_vel.joint3,
//                  joint_vel.joint4, joint_vel.joint5, joint_vel.joint6);
                  
		    
			if (joy->buttons.at(CARTESIAN_CONTROL) == 1)
			{
				joint_vel.joint1 = 0.0;
				joint_vel.joint2 = 0.0;
				joint_vel.joint3 = 0.0;
				joint_vel.joint4 = 0.0;
				joint_vel.joint5 = 0.0;
				joint_vel.joint6 = 0.0;
				
				joint_vel.finger1 = 0.0;
				joint_vel.finger2 = 0.0;
				joint_vel.finger3 = 0.0;
				joint_cmd_.publish(joint_vel);
				mode = CARTESIAN_CONTROL;
			
				ROS_INFO("Activated CARTESIAN control mode");
			}
		
			break;
			
		}
	}

	void jaco_teleop::publish_velocity()
	{
		
		switch (mode)
		{
			case CARTESIAN_CONTROL:
			cartesian_cmd_.publish(cartesian_vel);
			break;
		
			case JOINT_CONTROL:
			joint_cmd_.publish(joint_vel);
			break;
			
		}
	}

	int main(int argc, char **argv)
	{
		ros::init(argc, argv, "jaco_teleop");
		jaco_teleop joystick;
		ros::Rate loop_rate(60);
	
		while (ros::ok())
		{
			joystick.publish_velocity();
			ros::spinOnce();
			loop_rate.sleep();
		}
	
		return 0;
	}

