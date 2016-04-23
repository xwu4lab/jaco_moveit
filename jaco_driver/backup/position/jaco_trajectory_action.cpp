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
		
		stop_jaco       = false;
		// used for joint trajectory action
		move_joint          = false;
		movejoint_done      = false;
		num_jointTrajectory = 0;
		num_activeTrajectory = 0;
		
		pn.param("constraints/goal_time", goal_time_constraint, 0.0);
		// Gets the constraints for each joint.
		for (size_t i = 0; i < 6; ++i)
		{
			std::string ns = std::string("constraints/") + joint_names_[i];
			double g, t;
			pn.param(ns + "/goal", g, DEFAULT_GOAL_THRESHOLD);
			pn.param(ns + "/trajectory", t, -1.0);
			goal_constraints[joint_names_[i]] = g;
			trajectory_constraints[joint_names_[i]] = t;
		}
			pn.param("constraints/stopped_velocity_tolerance", stopped_velocity_tolerance, 0.01);
			
			node_handle_.param<double>("status_interval_seconds", watchdog_interval_seconds, 0.1);
			
			watchdog_timer = node_handle_.createTimer(ros::Duration(watchdog_interval_seconds),
                                           &JacoTrajectoryActionServer::watchdog, this);

			//pub_controller_command 	= jtacn.advertise<trajectory_msgs::JointTrajectory>("command", 1);
			//sub_controller_state   	= jtacn.subscribe("feedback_states", 1, &JacoActionController::controllerStateCB, this);

			// starting all the action server
			action_server_.start();

			// temporary outer control loop
			error_factor = 1;
			control_counter = 0;


			// feedback
			jtaction_fb.joint_names.resize(6,"");
			jtaction_fb.desired.positions.resize(6,0.0);
			jtaction_fb.actual.positions.resize(6,0.0);
			jtaction_fb.error.positions.resize(6,0.0);

			current_time = 0.0;
			old_time = 0.0;
			trajectory_start_time = 0.0;
			trajectory_duration = 0.0;
			time_diff = 0.001;
			derv_counter = 0;           


	}
    
    JacoTrajectoryActionServer::~JacoTrajectoryActionServer()
    {
		pub_controller_command.shutdown();
		sub_controller_state.shutdown();
		watchdog_timer.stop();
	}
	
	void JacoTrajectoryActionServer::update()
	{
		if ( num_jointTrajectory != 0)
		{
			if (movejoint_done)
			{
				JacoAngles current_angle;
				arm_comm_.getJointAngles(current_angle);
				
				current_jtangles[0] = (current_angle.Actuator1 - 180.0) * DTR;
				current_jtangles[1] = (current_angle.Actuator2 - 270.0) * DTR;
				current_jtangles[2] = (current_angle.Actuator3 - 90.0) * DTR;
				current_jtangles[3] = (current_angle.Actuator4 - 180.0) * DTR;
				current_jtangles[4] = (current_angle.Actuator5 - 180.0) * DTR;
				current_jtangles[5] = (current_angle.Actuator6 + 90.0) * DTR;


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

				calculate_error_dervError(current_jtangles, nextTraj_jtangles);

				// feedback
				jtaction_fb.header.stamp = ros::Time::now();
				jtaction_fb.joint_names = joint_names_;


				for(int i = 0; i< 6; i++)
				{
					jtaction_fb.desired.positions[i] = nextTraj_jtangles[i];
					jtaction_fb.actual.positions[i]= current_jtangles[i];
					jtaction_fb.error.positions[i] = fabs(error_jtangles[i]);
					
				}

				joint_active_goal.publishFeedback(jtaction_fb);
				
				//if all trajectories have been executed
				if (num_activeTrajectory == 0)
				{
					if (!arm_comm_.isStopped())
					{
						if(ros::Time::now().toSec() > (trajectory_start_time + trajectory_duration))
						{
							jtaction_res.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
							movejoint_done = false;
							joint_active_goal.setSucceeded(jtaction_res);
							
							ROS_INFO(" I am the winner ");
							ROS_INFO(" Final angles in degree (degree) : ");
							for(int i = 0; i< 6; i++)
							{
								ROS_INFO(" %i joint: %f ", i, current_jtangles[i] * RTD );
							}
							ROS_INFO("-------------------------------------");
							ROS_INFO(" Final angles in degree (rad) : ");
							for(int i = 0; i< 6; i++)
							{
								ROS_INFO(" %i joint: %f ", i, current_jtangles[i]);
							}
							error_factor = 1;
							control_counter = 0;
							has_active_goal = false;
							
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
					
					arm_comm_.stopAPI();
					arm_comm_.startAPI();
				}
				if (is_jointSpaceTrajectory_finished(current_jtangles, nextTraj_jtangles))
				{
						num_activeTrajectory--;
						ROS_INFO (" ACTIVE TRAJECTORY: %i", num_activeTrajectory);
						move_joint = true;
				}
			}
				
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

				ROS_INFO("I get here, and the joint is %f %f %f %f %f %f:", nextTraj_jtangles[0], nextTraj_jtangles[1], nextTraj_jtangles[2], nextTraj_jtangles[3], nextTraj_jtangles[4], nextTraj_jtangles[5]);
				JacoAngles target;
				target.Actuator1 = nextTraj_jtangles[0] * RTD + 180.0;
				target.Actuator2 = nextTraj_jtangles[1] * RTD + 269.5;
				target.Actuator3 = nextTraj_jtangles[2] * RTD + 90.0;
				target.Actuator4 = nextTraj_jtangles[3] * RTD + 180.0;
				target.Actuator5 = nextTraj_jtangles[4] * RTD + 180.0;
				target.Actuator6 = nextTraj_jtangles[5] * RTD - 90.0;
				
				arm_comm_.setJointAngles(target);
								
				ROS_INFO("Joint trajectory sent to Jaco arm");

				old_time = ros::Time::now().toSec();

				movejoint_done = true;
				move_joint = false;
				
				JacoAngles current_angle;
				arm_comm_.getJointAngles(current_angle);
				
				current_jtangles[0] = (current_angle.Actuator1 - 180.0) * DTR;
				current_jtangles[1] = (current_angle.Actuator2 - 270.0) * DTR;
				current_jtangles[2] = (current_angle.Actuator3 - 90.0) * DTR;
				current_jtangles[3] = (current_angle.Actuator4 - 180.0) * DTR;
				current_jtangles[4] = (current_angle.Actuator5 - 180.0) * DTR;
				current_jtangles[5] = (current_angle.Actuator6 + 90.0) * DTR;
								

				calculate_error_dervError(current_jtangles, nextTraj_jtangles);

				// feedback
				jtaction_fb.header.stamp = ros::Time::now();
				jtaction_fb.joint_names = joint_names_;
								
				for(int i = 0; i< 6; i++)
				{
					jtaction_fb.desired.positions[i] = nextTraj_jtangles[i];
					jtaction_fb.actual.positions[i]= current_jtangles[i];
					jtaction_fb.error.positions[i] = fabs(error_jtangles[i]);
				}

				joint_active_goal.publishFeedback(jtaction_fb);
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
		// Ensures that the joints in the goal match the joints we are commanding.
		ROS_INFO("Received goal: goalCB");
		if (!setsEqual(joint_names_, gh.getGoal()->trajectory.joint_names))
		{
			ROS_ERROR("Joints on incoming goal don't match our joints");
			gh.setRejected();
			return;
		}

		// Cancels the currently active goal.
		if (has_active_goal)
		{
			ROS_DEBUG("Received new goal, canceling current goal");

			// Marks the current goal as canceled.
			joint_active_goal.setCanceled();
			has_active_goal = false;
			
			num_jointTrajectory = 0;
			num_activeTrajectory = 0;
			
		}


		// Sends the trajectory along to the controller
		ROS_DEBUG("Publishing trajectory");                

		int ct = 0;
		num_jointTrajectory = 0;
		num_activeTrajectory = 0;
		
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

		//save the estimated duration of the trajectory
		trajectory_duration = gh.getGoal()->trajectory.points[num_jointTrajectory - 1].time_from_start.toSec();

		move_joint = true;  
		ROS_INFO("RECEIVE GOAL");
		
	}

	void JacoTrajectoryActionServer::joint_cancelCB(JointGoalHandle gh)
	{
		ROS_DEBUG("Received action cancel request");
		if (joint_active_goal == gh)
		{
			// Stops the controller.
			stop_jaco = true;

			// Marks the current goal as canceled.
			joint_active_goal.setCanceled();
			has_active_goal = false;
			
			num_jointTrajectory = 0;
			num_activeTrajectory = 0;
			
			arm_comm_.stopAPI();
			arm_comm_.startAPI();
			
			ROS_INFO("Joint goal canceled");
		}
	}
	
	bool JacoTrajectoryActionServer::is_jointSpaceTrajectory_finished(const std::vector<double> &currentvalue, const std::vector<double> &targetvalue)
	{
		double tol_jtag = 2.0 * DTR; 	// tolerance = 2 degree

		for(int i = 0; i< 6; i++)
		ROS_WARN("Tolarence: %f", setTwoEqual(fabs(currentvalue[i] - targetvalue[i])));

		if 	((setTwoEqual(fabs(currentvalue[0] - targetvalue[0])) < tol_jtag) && 
			(setTwoEqual(fabs(currentvalue[1] - targetvalue[1])) < tol_jtag) &&
			(setTwoEqual(fabs(currentvalue[2] - targetvalue[2])) < tol_jtag) &&
			(setTwoEqual(fabs(currentvalue[3] - targetvalue[3])) < tol_jtag) &&
			(setTwoEqual(fabs(currentvalue[4] - targetvalue[4])) < tol_jtag) &&
			(setTwoEqual(fabs(currentvalue[5] - targetvalue[5])) < tol_jtag) )
			
			return true;

		return false;

	}
	
	void JacoTrajectoryActionServer::outerloopcontroller_jointSpace(const std::vector<double> &currentvalue, const std::vector<double> &targetvalue, const double error_factor)
	{

		std::vector<double> new_jtang(6,0.0);

		for(int i = 0; i< 6; i++)
			new_jtang[i] = targetvalue[i] + (error_factor *( targetvalue[i] - currentvalue[i]) );
		
		JacoAngles target;
		target.Actuator1 = new_jtang[0]* RTD + 180.0;
		target.Actuator2 = new_jtang[1]* RTD + 269.5;
		target.Actuator3 = new_jtang[2]* RTD + 90.0;
		target.Actuator4 = new_jtang[3]* RTD + 180.0;
		target.Actuator5 = new_jtang[4]* RTD + 180.0;
		target.Actuator6 = new_jtang[5]* RTD + 270.0;
		
        arm_comm_.setJointAngles(target);

	}
	
	bool JacoTrajectoryActionServer::is_trajectory_finished()
	{

		if (    (( fabs(dervErr_jtangles[0])< 0.000001 ) && ( fabs(old_dervErr_jtangles[0])< 0.000001 ) ) &&
				(( fabs(dervErr_jtangles[1])< 0.000001 ) && ( fabs(old_dervErr_jtangles[1])< 0.000001 ) ) &&
				(( fabs(dervErr_jtangles[2])< 0.000001 ) && ( fabs(old_dervErr_jtangles[2])< 0.000001 ) ) )
		{
			derv_counter = derv_counter+1;
		}
		else
			derv_counter = 0;


		if(derv_counter > 100)
		{
			derv_counter = 0;

			return true;
		}

		return false;

	}
	
	void JacoTrajectoryActionServer::calculate_error_dervError(const std::vector<double> &currentvalue, const std::vector<double> &targetvalue)
	{

		current_time = ros::Time::now().toSec();

		time_diff =  current_time - old_time;


		for(int i = 0; i< 6; i++)
		{
			 error_jtangles[i] = targetvalue[i] - currentvalue[i];
			 dervErr_jtangles[i] = (error_jtangles[i] - old_err_jtangles[i]) / time_diff;
		}
		
		old_time = current_time;


		for(int i = 0; i < 6; i++)
		{
			old_err_jtangles[i] = error_jtangles[i];
			old_dervErr_jtangles[i] = dervErr_jtangles[i];
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
	
} //namespace jaco
