#ifndef JACO_DRIVER_JACO_TRAJECTORY_ACTION_H_
#define JACO_DRIVER_JACO_TRAJECTORY_ACTION_H_

#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <boost/thread/thread.hpp>

#include "jaco_driver/jaco_comm.h"

#define DTR 0.0174532925
#define RTD 57.295779513

const double DEFAULT_GOAL_THRESHOLD = 0.03;

namespace jaco
{
	class JacoTrajectoryActionServer
	{
		typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JTAS;
		typedef JTAS::GoalHandle JointGoalHandle;
		
		public:
		
			JacoTrajectoryActionServer(JacoComm &, const ros::NodeHandle &n);
			~JacoTrajectoryActionServer();

			bool is_jointSpaceTrajectory_finished(const std::vector<double> &currentvalue, const std::vector<double> &targetvalue);
			bool is_allJointSpaceTrajectory_finished(const std::vector<double> &currentvalue, const std::vector<double> &targetvalue);
            void calculate_error_dervError(const std::vector<double> &currentvalue, const std::vector<double> &targetvalue); 
            bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b);	
	        double setTwoEqual(const double &value);
			double velocitySatuation(const double &currentvalue, const double &targetvalue);                       
						
			void update();	
			
		private:
			ros::NodeHandle node_handle_;
			JacoComm &arm_comm_;
			
			void watchdog(const ros::TimerEvent&);
			ros::Timer watchdog_timer;
			double watchdog_interval_seconds;
			
			double vel_factor;
			double err_factor;
			
			control_msgs::FollowJointTrajectoryResult jtaction_res;
            control_msgs::FollowJointTrajectoryFeedback jtaction_fb;

            std::vector<std::string> joint_names_; 
            std::vector<double> current_jtangles, desired_jtangles, final_jtangles, nextTraj_jtangles, velocity;
            std::vector<double> error_jtangles, old_err_jtangles, dervErr_jtangles, old_dervErr_jtangles;
            double old_time, current_time, time_diff, trajectory_start_time, trajectory_duration;
	
			void joint_goalCB(JointGoalHandle gh);
 			void joint_cancelCB(JointGoalHandle gh);			

            int num_jointTrajectory;
            int num_activeTrajectory; 
            
  			JTAS action_server_;
			JointGoalHandle joint_active_goal;
			
			bool move_joint;
			bool movejoint_done;
			bool next_point;							
			bool stop_jaco;			
			bool has_active_goal;	
			
			ros::Time last_cmd_time;
			double timeout_seconds;
			double default_timeout_seconds;
	
			void outerloopcontroller_jointSpace(const std::vector<double> &currentvalue, const std::vector<double> &targetvalue, const double error_factor);
			double error_factor;

	};
}

#endif // JACO_DRIVER_JACO_TRAJECTORY_ACTION_H
