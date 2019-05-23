#include <franka_robot_jointposition_controller.h>

#include <pluginlib/class_list_macros.h>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>

namespace franka_robot_controllers{

	bool JointPositionController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh){

		ROS_INFO_STREAM(" ******* \n Starting Franka Robot Joint Position Controller \n *******");

		position_joint_interface_ = robot_hw->get<hardware_interface::PositionJointInterface>();

		if (position_joint_interface_ == nullptr)
		{
			ROS_ERROR("Could not get joint position interface from hardware!");
			return false;
		}

		std::vector<std::string> joint_names;
		if (!nh.getParam("joint_names",joint_names))
		{
			ROS_ERROR("Could not get joint names.");
		}
		if (joint_names.size()!=7)
		{
			ROS_ERROR_STREAM("Did not recieve 7 joints, Franka robot is 7 DOF robot!");
			return false;
		}

		position_joint_handles_.resize(7);
		for (int i = 0; i < 7; ++i)
		{
			try{
				position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
			}									
			catch(const hardware_interface::HardwareInterfaceException& e){
				ROS_ERROR_STREAM("Could not get joint handles: " << e.what());
				return false;
			}
		}

		bool safety_check = true;

		if (safety_check)
		{
			std::array<double, 7> q_start{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};

			for (int i = 0; i < q_start.size(); ++i)
			{
				if (std::abs(position_joint_handles_[i].getPosition() - q_start[i] > 0.1))
				{

					ROS_ERROR_STREAM("Robot is not in the starting position. This is only for safety of the robot. If you do not want to use this function just turn safety_check to false. Run `roslaunch franka_example_controllers move_to_start.launch robot_ip:=<robot-ip> first to move Franka robot to starting position.");
					return false;
				}
			}
		}
		return true;
	}

	void JointPositionController::starting(const ros::Time&) {

		for (int i = 0; i < 7; ++i)
		{
			initial_pose_[i] = position_joint_handles_[i].getPosition();
		}		
		elapsed_time_ = ros::Duration(0.0);
	}
	void JointPositionController::update(const ros::Time&, const ros::Duration& period){

		elapsed_time_ += period;

		double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec())) * 0.2;

		for (int i = 0; i < 7; ++i)
		{
			if (i == 4)
			{
				position_joint_handles_[i].setCommand(initial_pose_[i] - delta_angle);
			}else{
				position_joint_handles_[i].setCommand(initial_pose_[i] + delta_angle);
			}
		}
	}
}

// Implementation name_of_your_controller_package::NameOfYourControllerClass,

PLUGINLIB_EXPORT_CLASS(franka_robot_controllers::JointPositionController,
	controller_interface::ControllerBase)