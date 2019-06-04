#include <teleop_cmd.h>

int main(int argc, char **argv)
{

	Franka_robot franka_robot;
	franka_robot.setJointNames(); // Sets the joint names to the default names.
	franka_robot.setJointStartPosition(); // Sets the joint positions to start position
	
	// Initialize the franka_robot_teleop node
	ros::init(argc,	argv, "franka_robot_teleop");

	ros::NodeHandle nh;

	ros::Publisher teleop_cmd_pub = nh.advertise<sensor_msgs::JointState>("franka_robot/teleop_cmd", 1000);
	ros::Rate loop_rate(50);

	sensor_msgs::JointState joint_state_msg;
	joint_state_msg.name = franka_robot.getJointNames();	
	joint_state_msg.position = franka_robot.getJointGoalPosition();

	franka_robot.setJointVelocities();
	joint_state_msg.velocity = franka_robot.getJointVelocities();

	while(ros::ok()){

		teleop_cmd_pub.publish(joint_state_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}