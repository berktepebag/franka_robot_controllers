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


namespace franka_robot_controllers{

    class JointPositionController : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface>
    {
    private:
        hardware_interface::PositionJointInterface* position_joint_interface_;
        std::vector<hardware_interface::JointHandle> position_joint_handles_;
        
        ros::Duration elapsed_time_;
        //std::array<double, 7> initial_pose_{};

        std::vector<double> initial_pose_;
        std::vector<double> joint_position_goals;
        std::vector<double> speed_mult;
        std::vector<double> joint_velocity_limits;
        std::vector<double> current_joint_velocity_limits;
        std::vector<double> joint_accelerations;

        std::vector<double> joint_commands;

        ros::Subscriber teleop_cmd_sub;




    public:
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;  

        void frankaRobotTeleopCallback(const sensor_msgs::JointState::ConstPtr& msg);

        void setJointPositionGoals(std::vector<double> jointPositionGoalsMessage){
            joint_position_goals = jointPositionGoalsMessage;}

        std::vector<double> getJointPositionGoals() const 
            {return joint_position_goals;}

        void setJointSpeedLimits(std::vector<double> jointVelocityMessage)
            {speed_mult = jointVelocityMessage;}    
            std::vector<double> getJointSpeedLimits(){return speed_mult;}

        double deg2rad(double degree){
            return degree * M_PI / 180;}

        double rad2deg(double radian){
            return radian / M_PI * 180;}

        bool debugging = false;
        };
    }

#endif


