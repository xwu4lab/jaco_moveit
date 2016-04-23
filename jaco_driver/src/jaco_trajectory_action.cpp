#include <kinova/KinovaTypes.h>

#include "jaco_driver/jaco_trajectory_action.h"

#include "jaco_driver/jaco_types.h"

#define PI 3.1415926
namespace jaco
{

	JacoTrajectoryActionServer::JacoTrajectoryActionServer(JacoComm &arm_comm, const ros::NodeHandle &nh)
		: node_handle_(nh), arm_comm_(arm_comm),
		  action_server_(node_handle_, "jaco_arm_controller/joint_trajectory_action",
						 boost::bind(&JacoTrajectoryActionServer::joint_goalCB,  this, _1), boost::bind(&JacoTrajectoryActionServer::joint_cancelCB, this, _1),false),
		  has_active_goal(false)
	{
		ros::NodeHandle pn("~");
		
		node_handle_.param<double>("status_interval_seconds", watchdog_interval_seconds, 0.01);
		node_handle_.param<double>("velocity_factor", vel_factor, 150.0);
		node_handle_.param<double>("error_factor", err_factor, 1.0);
		node_handle_.param<double>("timeout_seconds", default_timeout_seconds, 0.40);
		
		joint_names_.resize(6, "");
		joint_names_[0] = "jaco_joint_1";
		joint_names_[1] = "jaco_joint_2";
		joint_names_[2] = "jaco_joint_3";
		joint_names_[3] = "jaco_joint_4";
		joint_names_[4] = "jaco_joint_5";
		joint_names_[5] = "jaco_joint_6";
		  
		current_jtangles.resize(6, 0.0);
		final_jtangles.resize(6, 0.0);               
		nextTraj_jtangles.resize(6, 0.0);
		error_jtangles.resize(6, 0.0);
		dervErr_jtangles.resize(6, 0.0);
		old_dervErr_jtangles.resize(6, 0.0);
		old_err_jtangles.resize(6, 0.0);
		velocity.resize(6, 0.0);
		
		stop_jaco  = false;
		move_joint = false;
		next_point = false;
		
		num_jointTrajectory = 0;
		num_activeTrajectory = 0;
		error_factor = 1;
		current_time = 0.0;
		old_time = 0.0;
		trajectory_start_time = 0.0;
		trajectory_duration = 0.0;
		time_diff = 0.01;  	
		timeout_seconds = default_timeout_seconds;
		
		jtaction_fb.joint_names.resize(6,"");
		jtaction_fb.desired.positions.resize(6,0.0);
		jtaction_fb.actual.positions.resize(6,0.0);
		jtaction_fb.error.positions.resize(6,0.0);

   
		action_server_.start();
		watchdog_timer = node_handle_.createTimer(ros::Duration(watchdog_interval_seconds),
											   &JacoTrajectoryActionServer::watchdog, this);

	}
    
    JacoTrajectoryActionServer::~JacoTrajectoryActionServer()
    {
		watchdog_timer.stop();
	}
	
	void JacoTrajectoryActionServer::update()
	{
		if ( num_jointTrajectory != 0)
		{
			if (move_joint)
			{
				if(num_jointTrajectory == num_activeTrajectory)
				{
					trajectory_start_time = old_time;
					for (int j = 0; j < 6; j++)
					{                                
						nextTraj_jtangles[j] = joint_active_goal.getGoal()->trajectory.points[num_jointTrajectory-num_activeTrajectory].positions[j];
					}
				}
				else
				{
					for (int j = 0; j < 6; j++)
					{                                
						nextTraj_jtangles[j] = joint_active_goal.getGoal()->trajectory.points[(num_jointTrajectory-num_activeTrajectory)-1].positions[j];
					}
				}
				
				JacoAngles current_angle;
				arm_comm_.getJointAngles(current_angle);
				
				current_jtangles[0] = (current_angle.Actuator1 - 180.0) * DTR;
				current_jtangles[1] = (current_angle.Actuator2 - 270.0) * DTR;
				current_jtangles[2] = (current_angle.Actuator3 - 90.0 ) * DTR;
				current_jtangles[3] = (current_angle.Actuator4 - 180.0) * DTR;
				current_jtangles[4] = (current_angle.Actuator5 - 180.0) * DTR;
				current_jtangles[5] = (current_angle.Actuator6 - 270.0 ) * DTR;
				
				if (next_point || num_activeTrajectory == 0)
				{
					for (int k = 0; k < 6; k++)
					{
						velocity[k] =  velocitySatuation(current_jtangles[k], nextTraj_jtangles[k]) * vel_factor ;
						last_cmd_time = ros::Time().now();
						next_point = false;
					}
				}
				
				AngularPosition target;
				target.Actuators.Actuator1 = velocity[0] ;
				target.Actuators.Actuator2 = velocity[1] ;
				target.Actuators.Actuator3 = velocity[2] ;
				target.Actuators.Actuator4 = velocity[3] ;
				target.Actuators.Actuator5 = velocity[4] ;
				target.Actuators.Actuator6 = velocity[5] ;
				target.Fingers.Finger1 = 0.0;
				target.Fingers.Finger2 = 0.0;
				target.Fingers.Finger3 = 0.0;
				
				arm_comm_.setFullJointVelocities(target);

//				Still not available
//				outerloopcontroller_jointSpace(current_jtangles, nextTraj_jtangles, err_factor);

				old_time = ros::Time::now().toSec();

				calculate_error_dervError(current_jtangles, nextTraj_jtangles);

				jtaction_fb.header.stamp = ros::Time::now();
				jtaction_fb.joint_names = joint_names_;


				for(int i = 0; i< 6; i++)
				{
					jtaction_fb.desired.positions[i] = nextTraj_jtangles[i];
					jtaction_fb.actual.positions[i]= current_jtangles[i];
					jtaction_fb.error.positions[i] = fabs(error_jtangles[i]);
					
				}

				joint_active_goal.publishFeedback(jtaction_fb);
				
				if (num_activeTrajectory == 0)
				{
					
					if (is_allJointSpaceTrajectory_finished(current_jtangles, nextTraj_jtangles))
					{
						AngularPosition target_finish;
						target_finish.Actuators.Actuator1 = 0.0 ;
						target_finish.Actuators.Actuator2 = 0.0 ;
						target_finish.Actuators.Actuator3 = 0.0 ;
						target_finish.Actuators.Actuator4 = 0.0 ;
						target_finish.Actuators.Actuator5 = 0.0 ;
						target_finish.Actuators.Actuator6 = 0.0 ;
						target_finish.Fingers.Finger1 = 0.0;
						target_finish.Fingers.Finger2 = 0.0;
						target_finish.Fingers.Finger3 = 0.0;
						
						arm_comm_.setFullJointVelocities(target_finish);
						
						if (!arm_comm_.isStopped())
						{
							if(ros::Time::now().toSec() > (trajectory_start_time + trajectory_duration))
							{
								jtaction_res.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;

								joint_active_goal.setSucceeded(jtaction_res);
								
								ROS_INFO(" !!!!!!!!I AM THE WINNER!!!!!!!! ");
								ROS_INFO(" Desired angle in degree : ");
								for(int i = 0; i< 6; i++)
								{
									ROS_INFO(" %i joint: %f ", i, setTwoEqual(nextTraj_jtangles[i]) * RTD);
								}
								ROS_INFO("-------------------------------------");
								ROS_INFO(" Final angles in degree : ");
								for(int i = 0; i< 6; i++)
								{
									ROS_INFO(" %i joint: %f ", i, setTwoEqual(current_jtangles[i]) * RTD);
								}
								ROS_INFO("-------------------------------------");
								ROS_INFO(" Error in degree : ");
								for(int i = 0; i< 6; i++)
								{
									ROS_INFO(" %i joint: %f ", i, setTwoEqual(nextTraj_jtangles[i] - current_jtangles[i]) * RTD);
								}
								error_factor = 1;
								has_active_goal = false;		
							}
							else
							{
								ROS_INFO("No active trajectories but TIMEOUT");
								jtaction_res.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
								joint_active_goal.setSucceeded(jtaction_res);
							}
						}
						else
						{
							ROS_INFO("No active trajectories but API is not in control. Aborted!");
							jtaction_res.error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
							joint_active_goal.setAborted(jtaction_res);
						}

						num_jointTrajectory = 0;
						num_activeTrajectory = 0;
					
						move_joint = false;
						next_point = false;
						
						timeout_seconds = default_timeout_seconds;
					
						arm_comm_.stopAPI();
						arm_comm_.startAPI();
					}
				}
				else
				{
					double elapsed_time_seconds = ros::Time().now().toSec() - last_cmd_time.toSec();

					if (elapsed_time_seconds > timeout_seconds)
					{
						timeout_seconds = elapsed_time_seconds;
						num_activeTrajectory--;
						next_point = true;
						ROS_INFO(" TIMEOUT AND GO FOR NEXT TRAJECTORY POINT, ELAPSED TIME: %f", elapsed_time_seconds);
					}
						
					if (is_jointSpaceTrajectory_finished(current_jtangles, nextTraj_jtangles))
					{
						num_activeTrajectory--;
						next_point = true;
						ROS_INFO (" ACTIVE TRAJECTORY: %i", num_activeTrajectory);
					}
				}
			}

			if (stop_jaco)
			{
				ROS_INFO(" Stopping the api control of Jaco arm...");
				arm_comm_.stopAPI();
				arm_comm_.startAPI();
				stop_jaco = false;
			}
		}
	}


	void JacoTrajectoryActionServer::joint_goalCB(JointGoalHandle gh)
	{
		ROS_INFO("Received goal: goalCB");
		if (!setsEqual(joint_names_, gh.getGoal()->trajectory.joint_names))
		{
			ROS_ERROR("Joints on incoming goal don't match our joints");
			gh.setRejected();
			return;
		}

		if (has_active_goal)
		{
			ROS_DEBUG("Received new goal, canceling current goal");

			joint_active_goal.setCanceled();
			has_active_goal = false;
			
			num_jointTrajectory = 0;
			num_activeTrajectory = 0;
			
		}

		ROS_DEBUG("Publishing trajectory");                

		int ct = 0;
		num_jointTrajectory = 0;
		num_activeTrajectory = 0;
		
		timeout_seconds = default_timeout_seconds;
		
		num_jointTrajectory = gh.getGoal()->trajectory.points.size();
		ROS_INFO("Tajectory Points: %i", num_jointTrajectory);
		num_activeTrajectory = num_jointTrajectory;
		
		if (num_jointTrajectory > 100)
		{
			ROS_ERROR("Trajectory longer than 100. Rejected!");
			gh.setRejected();
			num_jointTrajectory = 0;
			num_activeTrajectory = 0;
			return;
		}

		gh.setAccepted();
		joint_active_goal = gh;
		has_active_goal = true;


		desired_jtangles.resize(num_jointTrajectory * 6);

		ROS_INFO("num_jointTrajectory: %i", num_jointTrajectory);

		for(unsigned int i = 0 ; i < gh.getGoal()->trajectory.points.size(); i++)
		{
			for (int j = 0; j < 6; j++)
			{
				desired_jtangles[ct] = gh.getGoal()->trajectory.points[i].positions[j];
				ct++;                                
			}
		}

		trajectory_duration = gh.getGoal()->trajectory.points[num_jointTrajectory - 1].time_from_start.toSec();

		move_joint = true;  
		next_point = true;
		ROS_INFO("RECEIVE GOAL");
		
	}

	void JacoTrajectoryActionServer::joint_cancelCB(JointGoalHandle gh)
	{
		ROS_DEBUG("Received action cancel request");
		if (joint_active_goal == gh)
		{
			stop_jaco = true;

			joint_active_goal.setCanceled();
			has_active_goal = false;
			
			num_jointTrajectory = 0;
			num_activeTrajectory = 0;
			
			timeout_seconds = default_timeout_seconds;
			
			arm_comm_.stopAPI();
			arm_comm_.startAPI();
			
			ROS_INFO("Joint goal canceled");
		}
	}
	
	bool JacoTrajectoryActionServer::is_jointSpaceTrajectory_finished(const std::vector<double> &currentvalue, const std::vector<double> &targetvalue)
	{
		double tol_jtag = 10.0 * DTR; 
		
		if 	((fabs(setTwoEqual(currentvalue[0] - targetvalue[0])) < tol_jtag) && 
			(fabs(setTwoEqual(currentvalue[1] - targetvalue[1])) < tol_jtag) &&
			(fabs(setTwoEqual(currentvalue[2] - targetvalue[2])) < tol_jtag) &&
			(fabs(setTwoEqual(currentvalue[3] - targetvalue[3])) < tol_jtag) &&
			(fabs(setTwoEqual(currentvalue[4] - targetvalue[4])) < tol_jtag) &&
			(fabs(setTwoEqual(currentvalue[5] - targetvalue[5])) < tol_jtag) )
			
			return true;

		return false;

	}
	bool JacoTrajectoryActionServer::is_allJointSpaceTrajectory_finished(const std::vector<double> &currentvalue, const std::vector<double> &targetvalue)
	{
		double tol_jtag = 1.0 * DTR; 


		if 	((fabs(setTwoEqual(currentvalue[0] - targetvalue[0])) < tol_jtag) && 
			(fabs(setTwoEqual(currentvalue[1] - targetvalue[1])) < tol_jtag) &&
			(fabs(setTwoEqual(currentvalue[2] - targetvalue[2])) < tol_jtag) &&
			(fabs(setTwoEqual(currentvalue[3] - targetvalue[3])) < tol_jtag) &&
			(fabs(setTwoEqual(currentvalue[4] - targetvalue[4])) < tol_jtag) &&
			(fabs(setTwoEqual(currentvalue[5] - targetvalue[5])) < tol_jtag) )
			
			return true;

		return false;

	}
	
	void JacoTrajectoryActionServer::outerloopcontroller_jointSpace(const std::vector<double> &currentvalue, const std::vector<double> &targetvalue, const double error_factor)
	{	
		AngularPosition new_target;
		new_target.Actuators.Actuator1 = ((velocitySatuation(current_jtangles[0], nextTraj_jtangles[0])) - error_factor * old_dervErr_jtangles[0]) * vel_factor ;
		new_target.Actuators.Actuator2 = ((velocitySatuation(current_jtangles[1], nextTraj_jtangles[1])) - error_factor * old_dervErr_jtangles[1]) * vel_factor ;
		new_target.Actuators.Actuator3 = ((velocitySatuation(current_jtangles[2], nextTraj_jtangles[2])) - error_factor * old_dervErr_jtangles[2]) * vel_factor ;
		new_target.Actuators.Actuator4 = ((velocitySatuation(current_jtangles[3], nextTraj_jtangles[3])) - error_factor * old_dervErr_jtangles[3]) * vel_factor ;
		new_target.Actuators.Actuator5 = ((velocitySatuation(current_jtangles[4], nextTraj_jtangles[4])) - error_factor * old_dervErr_jtangles[4]) * vel_factor ;
		new_target.Actuators.Actuator6 = ((velocitySatuation(current_jtangles[5], nextTraj_jtangles[5])) - error_factor * old_dervErr_jtangles[5]) * vel_factor ;
		new_target.Fingers.Finger1 = 0.0;
		new_target.Fingers.Finger2 = 0.0;
		new_target.Fingers.Finger3 = 0.0;
		
        arm_comm_.setFullJointVelocities(new_target);
        ROS_INFO("THE ERROR COMPENSATION ARE: %f %f %f %f %f %f", error_factor * old_dervErr_jtangles[0] * vel_factor, 
			error_factor * old_dervErr_jtangles[1] * vel_factor, error_factor * old_dervErr_jtangles[2] * vel_factor, 
			error_factor * old_dervErr_jtangles[3] * vel_factor, error_factor * old_dervErr_jtangles[4] * vel_factor, 
			error_factor * old_dervErr_jtangles[5] * vel_factor);

	}
	
	
	void JacoTrajectoryActionServer::calculate_error_dervError(const std::vector<double> &currentvalue, const std::vector<double> &targetvalue)
	{

		for(int i = 0; i< 6; i++)
		{
			 error_jtangles[i] = setTwoEqual(targetvalue[i] - currentvalue[i]);
		}
		
		old_time = current_time;


		for(int i = 0; i < 6; i++)
		{
			old_err_jtangles[i] = error_jtangles[i];
		}

	}

	bool JacoTrajectoryActionServer::setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
	{
		if (a.size() != b.size())
		return false;

		for (size_t i = 0; i < a.size(); ++i)
		{
			if (count(b.begin(), b.end(), a[i]) != 1)
			return false;
		}
		for (size_t i = 0; i < b.size(); ++i)
		{
			if (count(a.begin(), a.end(), b[i]) != 1)
			return false;
		}

		return true;
	}
	
	void JacoTrajectoryActionServer::watchdog(const ros::TimerEvent&)
	{
		update();
	}
	
	double JacoTrajectoryActionServer::setTwoEqual(const double &value)
	{
		double value_ = value;
		while ( value_ > PI || value_ < -PI )
		{
			if (value_ > PI)
			{
				value_ -= 2*PI;
			}
			if (value_ < -PI)
			{
				value_ += 2*PI;
			}
		}
		return value_;
	}
	
	double JacoTrajectoryActionServer::velocitySatuation(const double &currentvalue, const double &targetvalue)
	{
		double diff = targetvalue - currentvalue;
		int count_plus = 0;
		int count_minus = 0;
		while ( diff > PI || diff < -PI )
		{
			if (diff > PI)
			{
				diff -= 2*PI;
				count_minus++;
			}
			if (diff < -PI)
			{
				diff += 2*PI;
				count_plus++;
			}
		}
		double normal_targetvalue = targetvalue + 2*PI*count_plus - 2*PI*count_minus;
		return normal_targetvalue - currentvalue;
			
	}
	
} //namespace jaco
