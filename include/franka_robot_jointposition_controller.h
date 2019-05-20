#ifndef FRANKA_ROBOT_JOINTPOSITONCONTROLLER_H_
#define FRANKA_ROBOT_JOINTPOSITONCONTROLLER_H_

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace franka_robot_controllers{

    class JointPositionController : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface>
    {
    private:
        hardware_interface::PositionJointInterface* position_joint_interface_;
        std::vector<hardware_interface::JointHandle> position_joint_handles_;
    public:
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;     
    };
}



#endif


