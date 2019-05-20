#include <franka_robot_controller_pi/franka_robot_jointposition_controller.h>

#include <pluginlib/class_list_macros.h>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>

namespace franka_robot_controllers{

    bool JointPositionController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh){

    	ROS_INFO_STREAM(" ******* \n Starting Franka Robot Joint Position Controller \n *******");

    	





        return true;
    }

    void JointPositionController::starting(const ros::Time&) {

    }
    void JointPositionController::update(const ros::Time&, const ros::Duration& period){

    }
    


}


// Implementation name_of_your_controller_package::NameOfYourControllerClass,
PLUGINLIB_EXPORT_CLASS(franka_robot_controllers::JointPositionController,
                       controller_interface::ControllerBase)