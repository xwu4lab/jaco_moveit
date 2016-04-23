//============================================================================
// Name        : jaco_arm.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Jaco robotic manipulator arm
//============================================================================

#include "jaco_driver/jaco_arm.h"
#include <string>
#include <vector>

#define PI 3.14159265359


namespace 
{
    /// \brief Convert Kinova-specific angle degree variations (0..180, 360-181) to
    ///        a more regular representation (0..180, -180..0).
    inline void convertKinDeg(double& qd)
    {
        static const double PI_180 = (PI / 180.0);

        // Angle velocities from the API are 0..180 for positive values,
        // and 360..181 for negative ones, in a kind of 2-complement setup.
        if (qd > 180.0) {
            qd -= 360.0;
        }
        qd *= PI_180;
    }

    inline void convertKinDeg(std::vector<double>& qds)
    {
        for (size_t i = 0; i < qds.size(); ++i) {
            double& qd = qds[i];
            convertKinDeg(qd);
        }
    }

    inline void convertKinDeg(geometry_msgs::Vector3& qds)
    {
        convertKinDeg(qds.x);
        convertKinDeg(qds.y);
        convertKinDeg(qds.z);
    }
}

namespace jaco
{

JacoArm::JacoArm(JacoComm &arm, const ros::NodeHandle &nodeHandle)
    :  node_handle_(nodeHandle), jaco_comm_(arm)
{
    /* Set up Services */
    stop_service_ = node_handle_.advertiseService("in/stop", &JacoArm::stopServiceCallback, this);
    start_service_ = node_handle_.advertiseService("in/start", &JacoArm::startServiceCallback, this);
    homing_service_ = node_handle_.advertiseService("in/home_arm", &JacoArm::homeArmServiceCallback, this);

    set_force_control_params_service_ = node_handle_.advertiseService("in/set_force_control_params", &JacoArm::setForceControlParamsCallback, this);
    start_force_control_service_ = node_handle_.advertiseService("in/start_force_control", &JacoArm::startForceControlCallback, this);
    stop_force_control_service_ = node_handle_.advertiseService("in/stop_force_control", &JacoArm::stopForceControlCallback, this);
    
    /* Set up Publishers */
    joint_angles_publisher_ = node_handle_.advertise<jaco_msgs::JointAngles>("out/joint_angles", 2);
    joint_state_publisher_ = node_handle_.advertise<sensor_msgs::JointState>("out/joint_state", 2);
    joint_position_state_publisher_ = node_handle_.advertise<sensor_msgs::JointState>("out/joint_position_state", 2);
    tool_position_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("out/tool_position", 2);
    tool_wrench_publisher_ = node_handle_.advertise<geometry_msgs::WrenchStamped>("out/tool_wrench", 2);
    finger_position_publisher_ = node_handle_.advertise<jaco_msgs::FingerPosition>("out/finger_position", 2);

    /* Set up Subscribers*/
    joint_velocity_subscriber_ = node_handle_.subscribe("in/joint_velocity", 1,
                                                      &JacoArm::jointVelocityCallback, this);
    cartesian_velocity_subscriber_ = node_handle_.subscribe("in/cartesian_velocity", 1,
                                                          &JacoArm::cartesianVelocityCallback, this);
    finger_velocity_subscriber_ = node_handle_.subscribe("in/finger_velocity", 1,
                                                       &JacoArm::fingerVelocityCallback, this);
    full_joint_velocity_subscriber_ = node_handle_.subscribe("in/full_joint_velocity", 1,
                                                      &JacoArm::fullJointVelocityCallback, this);                                                   
    full_cartesian_velocity_subscriber_ = node_handle_.subscribe("in/full_cartesian_velocity", 1,
                                                          &JacoArm::fullCartesianVelocityCallback, this);                                                   
    joint_position_subscriber_ = node_handle_.subscribe("in/joint_position", 1,
                                                      &JacoArm::jointPositionCallback, this);
    cartesian_position_subscriber_ = node_handle_.subscribe("in/cartesian_position", 1,
                                                          &JacoArm::cartesianPositionCallback, this);
    
    node_handle_.param<double>("status_interval_seconds", status_interval_seconds_, 0.1);
    node_handle_.param<double>("joint_angular_vel_timeout", joint_vel_timeout_seconds_, 0.3);
    node_handle_.param<double>("cartesian_vel_timeout", cartesian_vel_timeout_seconds_, 0.25);
    node_handle_.param<double>("finger_vel_timeout", finger_vel_timeout_seconds_, 0.2);
    node_handle_.param<double>("full_joint_angular_vel_timeout", full_joint_vel_timeout_seconds_, 0.25);
    node_handle_.param<double>("full_cartesian_vel_timeout", full_cartesian_vel_timeout_seconds_, 0.25);
    node_handle_.param<double>("joint_angular_pos_timeout", joint_pos_timeout_seconds_, 0.25);
    node_handle_.param<double>("cartesian_pos_timeout", cartesian_pos_timeout_seconds_, 0.25);
    
    node_handle_.param<double>("joint_angular_vel_interval", joint_vel_interval_seconds_, 0.01);
    node_handle_.param<double>("cartesian_vel_interval", cartesian_vel_interval_seconds_, 0.01);
    node_handle_.param<double>("finger_vel_interval", finger_vel_interval_seconds_, 0.01);
    node_handle_.param<double>("full_joint_angular_vel_interval", full_joint_vel_interval_seconds_, 0.01);
    node_handle_.param<double>("full_cartesian_vel_interval", full_cartesian_vel_interval_seconds_, 0.01);
    node_handle_.param<double>("joint_angular_pos_interval", joint_pos_interval_seconds_, 0.01);
    node_handle_.param<double>("cartesian_pos_interval", cartesian_pos_interval_seconds_, 0.01);
	

    node_handle_.param<std::string>("tf_prefix", tf_prefix_, "jaco_");

    // Approximative conversion ratio from finger position (0..6000) to joint angle 
    // in radians (0..0.7).
    node_handle_.param("finger_angle_conv_ratio", finger_conv_ratio_, 0.7 / 5000.0);

    // Depending on the API version, the arm might return velocities in the
    // 0..360 range (0..180 for positive values, 181..360 for negative ones).
    // This indicates that the ROS node should convert them first before
    // updating the joint_state topic.
    node_handle_.param("convert_joint_velocities", convert_joint_velocities_, true);

    joint_names_.resize(JACO_JOINTS_COUNT);
    joint_names_[0] = tf_prefix_ + "joint_1";
    joint_names_[1] = tf_prefix_ + "joint_2";
    joint_names_[2] = tf_prefix_ + "joint_3";
    joint_names_[3] = tf_prefix_ + "joint_4";
    joint_names_[4] = tf_prefix_ + "joint_5";
    joint_names_[5] = tf_prefix_ + "joint_6";
    joint_names_[6] = tf_prefix_ + "joint_finger_1";
    joint_names_[7] = tf_prefix_ + "joint_finger_2";
    joint_names_[8] = tf_prefix_ + "joint_finger_3";
    
    joint_urdf_names_.resize(6);
    joint_urdf_names_[0] = tf_prefix_ + "joint_1";
    joint_urdf_names_[1] = tf_prefix_ + "joint_2";
    joint_urdf_names_[2] = tf_prefix_ + "joint_3";
    joint_urdf_names_[3] = tf_prefix_ + "joint_4";
    joint_urdf_names_[4] = tf_prefix_ + "joint_5";
    joint_urdf_names_[5] = tf_prefix_ + "joint_6";

    status_timer_ = node_handle_.createTimer(ros::Duration(status_interval_seconds_),
                                           &JacoArm::statusTimer, this);

    ROS_INFO("Angular Velocity: %.3f interval %.3f timeout", joint_vel_interval_seconds_, joint_vel_timeout_seconds_);
    joint_vel_timer_ = node_handle_.createTimer(ros::Duration(joint_vel_interval_seconds_),
                                              &JacoArm::jointVelocityTimer, this);
    joint_vel_timer_.stop();
    joint_vel_timer_flag_ = false;

	ROS_INFO("Cartesian Velocity: %.3f interval %.3f timeout", cartesian_vel_interval_seconds_, cartesian_vel_timeout_seconds_);
    cartesian_vel_timer_ = node_handle_.createTimer(ros::Duration(cartesian_vel_interval_seconds_),
                                                  &JacoArm::cartesianVelocityTimer, this);
    cartesian_vel_timer_.stop();
    cartesian_vel_timer_flag_ = false;
	
	ROS_INFO("Finger Velocity: %.3f interval %.3f timeout", finger_vel_interval_seconds_, finger_vel_timeout_seconds_);
    finger_vel_timer_ = node_handle_.createTimer(ros::Duration(finger_vel_interval_seconds_), 
											   &JacoArm::fingerVelocityTimer, this);
	finger_vel_timer_.stop();
	finger_vel_timer_flag_ = false;
	
	ROS_INFO("Full Angular Velocity: %.3f interval %.3f timeout", full_joint_vel_interval_seconds_, full_joint_vel_timeout_seconds_);
    full_joint_vel_timer_ = node_handle_.createTimer(ros::Duration(full_joint_vel_interval_seconds_),
                                              &JacoArm::fullJointVelocityTimer, this);
    full_joint_vel_timer_.stop();
    full_joint_vel_timer_flag_ = false;
    
    ROS_INFO("Full Cartesian Velocity: %.3f interval %.3f timeout", full_cartesian_vel_interval_seconds_, full_cartesian_vel_timeout_seconds_);
    full_cartesian_vel_timer_ = node_handle_.createTimer(ros::Duration(full_cartesian_vel_interval_seconds_),
                                                  &JacoArm::fullCartesianVelocityTimer, this);
    full_cartesian_vel_timer_.stop();
    full_cartesian_vel_timer_flag_ = false;

	ROS_INFO("Angular Position: %.3f interval %.3f timeout", joint_pos_interval_seconds_, joint_pos_timeout_seconds_);
    joint_pos_timer_ = node_handle_.createTimer(ros::Duration(joint_pos_interval_seconds_),
                                              &JacoArm::jointPositionTimer, this);
    joint_pos_timer_.stop();
    joint_pos_timer_flag_ = false;

	ROS_INFO("Cartesian Position: %.3f interval %.3f timeout", cartesian_pos_interval_seconds_, cartesian_pos_timeout_seconds_);
    cartesian_pos_timer_ = node_handle_.createTimer(ros::Duration(cartesian_pos_interval_seconds_),
                                                  &JacoArm::cartesianPositionTimer, this);
    cartesian_pos_timer_.stop();
    cartesian_pos_timer_flag_ = false;

    ROS_INFO("The arm is ready to use.");


}


JacoArm::~JacoArm()
{
}


bool JacoArm::homeArmServiceCallback(jaco_msgs::HomeArm::Request &req, jaco_msgs::HomeArm::Response &res)
{
    jaco_comm_.homeArm();
    jaco_comm_.initFingers();
    res.homearm_result = "JACO ARM HAS BEEN RETURNED HOME";
    return true;
}



/*!
 * \brief Handler for "stop" service.
 *
 * Instantly stops the arm and prevents further movement until start service is
 * invoked.
 */
bool JacoArm::stopServiceCallback(jaco_msgs::Stop::Request &req, jaco_msgs::Stop::Response &res)
{
    jaco_comm_.stopAPI();
    res.stop_result = "Arm stopped";
    ROS_DEBUG("Arm stop requested");
    return true;
}


/*!
 * \brief Handler for "start" service.
 *
 * Re-enables control of the arm after a stop.
 */
bool JacoArm::startServiceCallback(jaco_msgs::Start::Request &req, jaco_msgs::Start::Response &res)
{
    jaco_comm_.startAPI();
    res.start_result = "Arm started";
    ROS_DEBUG("Arm start requested");
    return true;
}

bool JacoArm::setForceControlParamsCallback(jaco_msgs::SetForceControlParams::Request &req, jaco_msgs::SetForceControlParams::Response &res)
{
    CartesianInfo inertia, damping, force_min, force_max;
    inertia.X      = req.inertia_linear.x;
    inertia.Y      = req.inertia_linear.y;
    inertia.Z      = req.inertia_linear.z;
    inertia.ThetaX = req.inertia_angular.x;
    inertia.ThetaY = req.inertia_angular.y;
    inertia.ThetaZ = req.inertia_angular.z;
    damping.X      = req.damping_linear.x;
    damping.Y      = req.damping_linear.y;
    damping.Z      = req.damping_linear.z;
    damping.ThetaX = req.damping_angular.x;
    damping.ThetaY = req.damping_angular.y;
    damping.ThetaZ = req.damping_angular.z;

    jaco_comm_.setCartesianInertiaDamping(inertia, damping);

    force_min.X      = req.force_min_linear.x;
    force_min.Y      = req.force_min_linear.y;
    force_min.Z      = req.force_min_linear.z;
    force_min.ThetaX = req.force_min_angular.x;
    force_min.ThetaY = req.force_min_angular.y;
    force_min.ThetaZ = req.force_min_angular.z;
    force_max.X      = req.force_max_linear.x;
    force_max.Y      = req.force_max_linear.y;
    force_max.Z      = req.force_max_linear.z;
    force_max.ThetaX = req.force_max_angular.x;
    force_max.ThetaY = req.force_max_angular.y;
    force_max.ThetaZ = req.force_max_angular.z;

    jaco_comm_.setCartesianForceMinMax(force_min, force_max);

    return true;
}

bool JacoArm::startForceControlCallback(jaco_msgs::Start::Request &req, jaco_msgs::Start::Response &res)
{
    jaco_comm_.startForceControl();
    res.start_result = "Start force control requested.";
    return true;
}

bool JacoArm::stopForceControlCallback(jaco_msgs::Stop::Request &req, jaco_msgs::Stop::Response &res)
{
    jaco_comm_.stopForceControl();
    res.stop_result = "Stop force control requested.";
    return true;
}


void JacoArm::jointVelocityCallback(const jaco_msgs::JointVelocityConstPtr& joint_vel)
{
    if (!jaco_comm_.isStopped())
    {
        joint_velocities_.Actuator1 = joint_vel->joint1;
        joint_velocities_.Actuator2 = joint_vel->joint2;
        joint_velocities_.Actuator3 = joint_vel->joint3;
        joint_velocities_.Actuator4 = joint_vel->joint4;
        joint_velocities_.Actuator5 = joint_vel->joint5;
        joint_velocities_.Actuator6 = joint_vel->joint6;
        last_joint_vel_cmd_time_ = ros::Time().now();
        
        if (joint_vel_timer_flag_ == false)
        {
            joint_vel_timer_.start();
            joint_vel_timer_flag_ = true;
        }
    } else {
        ROS_WARN("Velocity command received but comm is stopped");
    }
}

void JacoArm::jointVelocityTimer(const ros::TimerEvent&)
{
    double elapsed_time_seconds = ros::Time().now().toSec() - last_joint_vel_cmd_time_.toSec();

    if (elapsed_time_seconds > joint_vel_timeout_seconds_)
    {
        ROS_WARN("Joint vel timed out: %f", elapsed_time_seconds);
        joint_vel_timer_.stop();
        joint_vel_timer_flag_ = false;
    }
    else
    {
        // ROS_DEBUG("Joint vel timer (%f): %f, %f, %f, %f, %f, %f", elapsed_time_seconds,
        //           joint_velocities_.Actuator1, joint_velocities_.Actuator2, joint_velocities_.Actuator3,
        //           joint_velocities_.Actuator4, joint_velocities_.Actuator5, joint_velocities_.Actuator6);
        jaco_comm_.setJointVelocities(joint_velocities_);

    }
}



void JacoArm::cartesianVelocityCallback(const geometry_msgs::TwistStampedConstPtr& cartesian_vel)
{
    if (!jaco_comm_.isStopped())
    {
        cartesian_velocities_.X = cartesian_vel->twist.linear.x;
        cartesian_velocities_.Y = cartesian_vel->twist.linear.y;
        cartesian_velocities_.Z = cartesian_vel->twist.linear.z;
        cartesian_velocities_.ThetaX = cartesian_vel->twist.angular.x;
        cartesian_velocities_.ThetaY = cartesian_vel->twist.angular.y;
        cartesian_velocities_.ThetaZ = cartesian_vel->twist.angular.z;

        last_cartesian_vel_cmd_time_ = ros::Time().now();

        if (cartesian_vel_timer_flag_ == false)
        {
            cartesian_vel_timer_.start();
            cartesian_vel_timer_flag_ = true;
        }
    }
}



void JacoArm::cartesianVelocityTimer(const ros::TimerEvent&)
{
    double elapsed_time_seconds = ros::Time().now().toSec() - last_cartesian_vel_cmd_time_.toSec();

    if (elapsed_time_seconds > cartesian_vel_timeout_seconds_)
    {
        ROS_DEBUG("Cartesian vel timed out: %f", elapsed_time_seconds);
        cartesian_vel_timer_.stop();
        cartesian_vel_timer_flag_ = false;
    }
    else
    {
        ROS_DEBUG("Cart vel timer (%f): %f, %f, %f, %f, %f, %f", elapsed_time_seconds,
                  cartesian_velocities_.X, cartesian_velocities_.Y, cartesian_velocities_.Z,
                  cartesian_velocities_.ThetaX, cartesian_velocities_.ThetaY, cartesian_velocities_.ThetaZ);
        jaco_comm_.setCartesianVelocities(cartesian_velocities_);
        
    }
}


void JacoArm::fingerVelocityCallback(const jaco_msgs::FingerVelocityConstPtr& finger_vel)
{
    if (!jaco_comm_.isStopped())
    {

		finger_velocity_.Finger1 = finger_vel->finger1;
		finger_velocity_.Finger2 = finger_vel->finger2;
		finger_velocity_.Finger3 = finger_vel->finger3;
		
        last_finger_vel_cmd_time_ = ros::Time().now();

        if (finger_vel_timer_flag_ == false)
        {
            finger_vel_timer_.start();
            finger_vel_timer_flag_ = true;
        }
    } else {
        ROS_WARN("Finger command received but comm is stopped");
    }
}

void JacoArm::fingerVelocityTimer(const ros::TimerEvent&)
{
    double elapsed_time_seconds = ros::Time().now().toSec() - last_finger_vel_cmd_time_.toSec();

    if (elapsed_time_seconds > finger_vel_timeout_seconds_)
    {
        ROS_WARN("Finger vel timed out: %f", elapsed_time_seconds);
        finger_vel_timer_.stop();
        finger_vel_timer_flag_ = false;
    }
    else
    {
        // ROS_DEBUG("Finger vel timer (%f): %f, %f, %f", elapsed_time_seconds,
        //           finger_position_.Finger1, finger_position_.Finger2, finger_position_.Finger3);

        jaco_comm_.setFingerVelocities(finger_velocity_);

    }
}


void JacoArm::fullJointVelocityCallback(const jaco_msgs::FullJointVelocityConstPtr& full_joint_vel)
{
    if (!jaco_comm_.isStopped())
    {
        full_joint_velocities_.Actuators.Actuator1 = full_joint_vel->joint1;
        full_joint_velocities_.Actuators.Actuator2 = full_joint_vel->joint2;
        full_joint_velocities_.Actuators.Actuator3 = full_joint_vel->joint3;
        full_joint_velocities_.Actuators.Actuator4 = full_joint_vel->joint4;
        full_joint_velocities_.Actuators.Actuator5 = full_joint_vel->joint5;
        full_joint_velocities_.Actuators.Actuator6 = full_joint_vel->joint6;
        full_joint_velocities_.Fingers.Finger1     = full_joint_vel->finger1;
        full_joint_velocities_.Fingers.Finger2     = full_joint_vel->finger2;
        full_joint_velocities_.Fingers.Finger3     = full_joint_vel->finger3;
        last_full_joint_vel_cmd_time_ = ros::Time().now();

        if (full_joint_vel_timer_flag_ == false)
        {
            full_joint_vel_timer_.start();
            full_joint_vel_timer_flag_ = true;
        }
    } else {
        ROS_WARN("Velocity command received but comm is stopped");
    }
}

void JacoArm::fullJointVelocityTimer(const ros::TimerEvent&)
{
    double elapsed_time_seconds = ros::Time().now().toSec() - last_full_joint_vel_cmd_time_.toSec();

    if (elapsed_time_seconds > full_joint_vel_timeout_seconds_)
    {
        ROS_WARN("Full Joint vel timed out: %f", elapsed_time_seconds);
        full_joint_vel_timer_.stop();
        full_joint_vel_timer_flag_ = false;
    }
    else
    {
        // ROS_DEBUG("Full Joint vel timer (%f): %f, %f, %f, %f, %f, %f", elapsed_time_seconds,
        //           full_joint_velocities_.Actuators.Actuator1, full_joint_velocities_.Actuators.Actuator2, full_joint_velocities_.Actuators.Actuator3,
        //           full_joint_velocities_.Actuators.Actuator4, full_joint_velocities_.Actuators.Actuator5, full_joint_velocities_.Actuators.Actuator6,
        //           full_joint_velocities_.Fingers.Finger1, full_joint_velocities_.Fingers.Finger2, full_joint_velocities_.Fingers.Finger3 );
        jaco_comm_.setFullJointVelocities(full_joint_velocities_);
    }
}

void JacoArm::fullCartesianVelocityCallback(const jaco_msgs::FullVelocityConstPtr& full_cartesian_vel)
{
    if (!jaco_comm_.isStopped())
    {
        full_cartesian_velocities_.Coordinates.X = full_cartesian_vel->velocity.linear.x;
        full_cartesian_velocities_.Coordinates.Y = full_cartesian_vel->velocity.linear.y;
        full_cartesian_velocities_.Coordinates.Z = full_cartesian_vel->velocity.linear.z;
        full_cartesian_velocities_.Coordinates.ThetaX = full_cartesian_vel->velocity.angular.x;
        full_cartesian_velocities_.Coordinates.ThetaY = full_cartesian_vel->velocity.angular.y;
        full_cartesian_velocities_.Coordinates.ThetaZ = full_cartesian_vel->velocity.angular.z;
        full_cartesian_velocities_.Fingers.Finger1 = full_cartesian_vel->finger1;
        full_cartesian_velocities_.Fingers.Finger2 = full_cartesian_vel->finger2;
        full_cartesian_velocities_.Fingers.Finger3 = full_cartesian_vel->finger3;

        last_full_cartesian_vel_cmd_time_ = ros::Time().now();

        if (full_cartesian_vel_timer_flag_ == false)
        {
            full_cartesian_vel_timer_.start();
            full_cartesian_vel_timer_flag_ = true;
        }
    }
}



void JacoArm::fullCartesianVelocityTimer(const ros::TimerEvent&)
{
    double elapsed_time_seconds = ros::Time().now().toSec() - last_full_cartesian_vel_cmd_time_.toSec();

    if (elapsed_time_seconds > full_cartesian_vel_timeout_seconds_)
    {
        ROS_WARN("Full Cartesian vel timed out: %f", elapsed_time_seconds);
        full_cartesian_vel_timer_.stop();
        full_cartesian_vel_timer_flag_ = false;
    }
    else
    {
        //ROS_DEBUG("Full Cart vel timer (%f): %f, %f, %f, %f, %f, %f", elapsed_time_seconds,
        //          full_cartesian_velocities_.Coordinates.X, full_cartesian_velocities_.Coordinates.Y, full_cartesian_velocities_.Coordinates.Z,
        //          full_cartesian_velocities_.Coordinates.ThetaX, full_cartesian_velocities_.Coordinates.ThetaY, full_cartesian_velocities_.Coordinates.ThetaZ
        //          full_cartesian_velocities_.Fingers.Finger1, full_cartesian_velocities_.Fingers.Finger2, full_cartesian_velocities_.Fingers.Finger3);
        jaco_comm_.setFullCartesianVelocities(full_cartesian_velocities_);
    }
}

void JacoArm::jointPositionCallback(const jaco_msgs::JointAnglesConstPtr& joint_pos)
{
    if (!jaco_comm_.isStopped())
    {
        joint_position_.Actuator1 = joint_pos->joint1;
        joint_position_.Actuator2 = joint_pos->joint2;
        joint_position_.Actuator3 = joint_pos->joint3;
        joint_position_.Actuator4 = joint_pos->joint4;
        joint_position_.Actuator5 = joint_pos->joint5;
        joint_position_.Actuator6 = joint_pos->joint6;
        last_joint_pos_cmd_time_ = ros::Time().now();

        
        if (joint_pos_timer_flag_ == false)
        {
            joint_pos_timer_.start();
            joint_pos_timer_flag_ = true;
        }
    } else {
        ROS_WARN("Position command received but comm is stopped");
    }
}

void JacoArm::jointPositionTimer(const ros::TimerEvent&)
{
    double elapsed_time_seconds = ros::Time().now().toSec() - last_joint_pos_cmd_time_.toSec();

    if (elapsed_time_seconds > joint_pos_timeout_seconds_)
    {
        ROS_WARN("Joint postion timed out: %f", elapsed_time_seconds);
        joint_pos_timer_.stop();
        joint_pos_timer_flag_ = false;
    }
    else
    {
        // ROS_DEBUG("Joint pos timer (%f): %f, %f, %f, %f, %f, %f", elapsed_time_seconds,
        //           joint_position_.Actuator1, joint_position_.Actuator2, joint_position_.Actuator3,
        //           joint_position_.Actuator4, joint_position_.Actuator5, joint_position_.Actuator6);
        jaco_comm_.setJointAngles(joint_position_);

    }
}



void JacoArm::cartesianPositionCallback(const geometry_msgs::TwistStampedConstPtr& cartesian_pos)
{
    if (!jaco_comm_.isStopped())
    {
        cartesian_position_.X = cartesian_pos->twist.linear.x;
        cartesian_position_.Y = cartesian_pos->twist.linear.y;
        cartesian_position_.Z = cartesian_pos->twist.linear.z;
        cartesian_position_.ThetaX = cartesian_pos->twist.angular.x;
        cartesian_position_.ThetaY = cartesian_pos->twist.angular.y;
        cartesian_position_.ThetaZ = cartesian_pos->twist.angular.z;

        last_cartesian_pos_cmd_time_ = ros::Time().now();

        if (cartesian_pos_timer_flag_ == false)
        {
            cartesian_pos_timer_.start();
            cartesian_pos_timer_flag_ = true;
        }
    }
}



void JacoArm::cartesianPositionTimer(const ros::TimerEvent&)
{
    double elapsed_time_seconds = ros::Time().now().toSec() - last_cartesian_pos_cmd_time_.toSec();

    if (elapsed_time_seconds > cartesian_pos_timeout_seconds_)
    {
        ROS_DEBUG("Cartesian position timed out: %f", elapsed_time_seconds);
        cartesian_pos_timer_.stop();
        cartesian_pos_timer_flag_ = false;
    }
    else
    {
        ROS_DEBUG("Cart vel timer (%f): %f, %f, %f, %f, %f, %f", elapsed_time_seconds,
                  cartesian_position_.X, cartesian_position_.Y, cartesian_position_.Z,
                  cartesian_position_.ThetaX, cartesian_position_.ThetaY, cartesian_position_.ThetaZ);
        jaco_comm_.setCartesianPosition(cartesian_position_);
        
    }
}




/*!
 * \brief Publishes the current joint angles.
 *
 * Joint angles are published in both their raw state as obtained from the arm
 * (JointAngles), and transformed & converted to radians (joint_state) as per
 * the Jaco Kinematics PDF.
 *
 * Velocities and torques (effort) are only published in the JointStates
 * message, only for the first 6 joints as these values are not available for
 * the fingers.
 */
void JacoArm::publishJointAngles(void)
{
    FingerAngles fingers;
    jaco_comm_.getFingerPositions(fingers);

    // Query arm for current joint angles
    JacoAngles current_angles;
    jaco_comm_.getJointAngles(current_angles);
    jaco_msgs::JointAngles jaco_angles = current_angles.constructAnglesMsg();


    sensor_msgs::JointState joint_state;
    joint_state.name = joint_names_;
 //   joint_state.header.stamp = ros::Time::now();
    
    sensor_msgs::JointState joint_position_state;
    joint_position_state.name = joint_urdf_names_;
    joint_position_state.header.stamp = ros::Time::now();

    jaco_angles.joint1 = current_angles.Actuator1;
    jaco_angles.joint2 = current_angles.Actuator2;
    jaco_angles.joint3 = current_angles.Actuator3;
    jaco_angles.joint4 = current_angles.Actuator4;
    jaco_angles.joint5 = current_angles.Actuator5;
    jaco_angles.joint6 = current_angles.Actuator6;
    
    // Transform from physical angles to Kinova DH algorithm in radians , then place into vector array
    joint_state.position.resize(9);

    joint_state.position[0] = (jaco_angles.joint1 - 180.0) * (PI / 180.0);
    joint_state.position[1] = (jaco_angles.joint2 - 270.0) * (PI / 180.0);
    joint_state.position[2] = (jaco_angles.joint3 - 90.0) * (PI / 180.0);
    joint_state.position[3] = (jaco_angles.joint4 - 180.0 ) * (PI / 180.0);
    joint_state.position[4] = (jaco_angles.joint5 - 180.0) * (PI / 180.0);
    joint_state.position[5] = (jaco_angles.joint6 - 270.0) * (PI / 180.0);
    joint_state.position[6] = finger_conv_ratio_ * fingers.Finger1;
    joint_state.position[7] = finger_conv_ratio_ * fingers.Finger2;
    joint_state.position[8] = finger_conv_ratio_ * fingers.Finger3;
   
   // just for visualization
    joint_position_state.position.resize(6);
    
    joint_position_state.position[0] = (jaco_angles.joint1 - 180.0) * (PI / 180.0);
    joint_position_state.position[1] = (jaco_angles.joint2 - 270.0) * (PI / 180.0);
    joint_position_state.position[2] = (jaco_angles.joint3 - 90.0) * (PI / 180.0);
    joint_position_state.position[3] = (jaco_angles.joint4 - 180.0 ) * (PI / 180.0);
    joint_position_state.position[4] = (jaco_angles.joint5 - 180.0) * (PI / 180.0);
    joint_position_state.position[5] = (jaco_angles.joint6 - 270.0) * (PI / 180.0);


  
    // Joint velocities
    JacoAngles current_vels;
    jaco_comm_.getJointVelocities(current_vels);
    joint_state.velocity.resize(9);
    joint_state.velocity[0] = current_vels.Actuator1;
    joint_state.velocity[1] = current_vels.Actuator2;
    joint_state.velocity[2] = current_vels.Actuator3;
    joint_state.velocity[3] = current_vels.Actuator4;
    joint_state.velocity[4] = current_vels.Actuator5;
    joint_state.velocity[5] = current_vels.Actuator6;

    ROS_DEBUG_THROTTLE(0.1,
                       "Raw joint velocities: %f %f %f %f %f %f",
                       joint_state.velocity[0],
                       joint_state.velocity[1],
                       joint_state.velocity[2],
                       joint_state.velocity[3],
                       joint_state.velocity[4],
                       joint_state.velocity[5]);



    if (convert_joint_velocities_) {
        convertKinDeg(joint_state.velocity);
    }

    // No velocity for the fingers:
    joint_state.velocity[6] = 0.0;
    joint_state.velocity[7] = 0.0;
    joint_state.velocity[8] = 0.0;

    // Joint torques (effort)
    // NOTE: Currently invalid.
    JacoAngles joint_tqs;
    joint_state.effort.resize(9);
    joint_state.effort[0] = joint_tqs.Actuator1;
    joint_state.effort[1] = joint_tqs.Actuator2;
    joint_state.effort[2] = joint_tqs.Actuator3;
    joint_state.effort[3] = joint_tqs.Actuator4;
    joint_state.effort[4] = joint_tqs.Actuator5;
    joint_state.effort[5] = joint_tqs.Actuator6;
    joint_state.effort[6] = 0.0;
    joint_state.effort[7] = 0.0;
    joint_state.effort[8] = 0.0;

    ROS_DEBUG_THROTTLE(0.1,
                       "Raw joint torques: %f %f %f %f %f %f",
                       joint_state.effort[0],
                       joint_state.effort[1],
                       joint_state.effort[2],
                       joint_state.effort[3],
                       joint_state.effort[4],
                       joint_state.effort[5]);

    joint_angles_publisher_.publish(jaco_angles);
    joint_state_publisher_.publish(joint_state);
    joint_position_state_publisher_.publish(joint_position_state);
    ros::spinOnce();
}


/*!
 * \brief Publishes the current cartesian coordinates
 */
void JacoArm::publishToolPosition(void)
{
    JacoPose pose;
    geometry_msgs::PoseStamped current_position;

    jaco_comm_.getCartesianPosition(pose);

    current_position.pose            = pose.constructPoseMsg();
    current_position.header.stamp    = ros::Time::now();
    current_position.header.frame_id = tf_prefix_ + "link_base";

    tool_position_publisher_.publish(current_position);
}

/*!
 * \brief Publishes the current cartesian forces at the end effector. 
 */
void JacoArm::publishToolWrench(void)
{
    JacoPose wrench;
    geometry_msgs::WrenchStamped current_wrench;

    jaco_comm_.getCartesianForce(wrench);

    current_wrench.wrench          = wrench.constructWrenchMsg();
    current_wrench.header.stamp    = ros::Time::now();
    // TODO: Rotate wrench to fit the end effector frame.
    // Right now, the orientation of the wrench is in the API's (base) frame.
    current_wrench.header.frame_id = tf_prefix_ + "api_origin";


    // Same conversion issue as with velocities:
    if (convert_joint_velocities_) {
        convertKinDeg(current_wrench.wrench.torque);
    }

    tool_wrench_publisher_.publish(current_wrench);
}

/*!
 * \brief Publishes the current finger positions.
 */
void JacoArm::publishFingerPosition(void)
{
    FingerAngles fingers;
    jaco_comm_.getFingerPositions(fingers);
    finger_position_publisher_.publish(fingers.constructFingersMsg());
}


void JacoArm::statusTimer(const ros::TimerEvent&)
{
    publishJointAngles();
    publishToolPosition();
    publishToolWrench();
    publishFingerPosition();
}

}  // namespace jaco