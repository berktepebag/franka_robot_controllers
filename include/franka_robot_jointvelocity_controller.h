#ifndef FRANKA_ROBOT_JOINTPOSITONCONTROLLER_H_
#define FRANKA_ROBOT_JOINTPOSITONCONTROLLER_H_

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
//Added
#include <franka_hw/franka_state_interface.h>

namespace franka_robot_controllers{

    class JointVelocityController : public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface, franka_hw::FrankaStateInterface>
    {
    private:
        hardware_interface::VelocityJointInterface* velocity_joint_interface_;
        std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
        
        ros::Duration elapsed_time_;
        //std::array<double, 7> initial_pose_{};

        std::vector<double> initial_pose_;
        std::vector<double> joint_goal_duration;

        std::vector<double> joint_goal_velocities;
        std::vector<double> current_joint_goal_velocities;
        std::vector<double> joint_accelerations;

        std::vector<double> joint_commands;
        int newSeq;

        ros::Subscriber teleop_cmd_sub;

    public:
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;  

        void frankaRobotTeleopCallback(const sensor_msgs::JointState::ConstPtr& msg);

        void setJointVelocityDurations(std::vector<double> jointPositionGoalsMessage){
            joint_goal_duration = jointPositionGoalsMessage;}

        std::vector<double> getJointVelocityDurations() const 
            {return joint_goal_duration;}

        void setJointSpeedLimits(std::vector<double> jointVelocityMessage)
            {joint_goal_velocities = jointVelocityMessage;}    
        
        std::vector<double> getJointSpeedLimits(){return joint_goal_velocities;}

        void setSeq(int seqMessageReceived){newSeq = seqMessageReceived;}
        int getSeq(){return newSeq;}

        double deg2rad(double degree){
            return degree * M_PI / 180;}

        double rad2deg(double radian){
            return radian / M_PI * 180;}



        bool debugging = false;
        };
        
        int currentSeq;
        double goalDuration;
    }

#endif


