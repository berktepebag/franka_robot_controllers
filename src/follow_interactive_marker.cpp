#include <follow_interactive_marker.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>
// Kinematics
#include <moveit_msgs/GetPositionIK.h>

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

	//IK solver service begin
	ros::ServiceClient service_client = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

	moveit_msgs::GetPositionIK::Request service_request;
	moveit_msgs::GetPositionIK::Response service_response;

	while (!service_client.exists())
	{
		ROS_INFO("Waiting for service");
		sleep(1.0);
	}
	//IK solver service ends

	sensor_msgs::JointState joint_state_msg;
	joint_state_msg.name = franka_robot.getJointNames();

	franka_robot.setJointStartPosition();
	joint_state_msg.position = franka_robot.getJointGoalPositions();	

	//Set joint velocities to zero in order to prevent unexpected movements at the beginning.
	franka_robot.setJointVelocitiesZero();	

	//nh.getParam("panda_joi")	

	srand(time(NULL));
	
	while(ros::ok()){		

		franka_robot.setJointVelocitiesOne();

		service_request.ik_request.group_name = "panda_arm";
		service_request.ik_request.pose_stamped.pose.position.x = 0.088;
		service_request.ik_request.pose_stamped.pose.position.y = 0.0;
		service_request.ik_request.pose_stamped.pose.position.z = 0.0;

		service_request.ik_request.pose_stamped.pose.orientation.x = 0.623742313489;
		service_request.ik_request.pose_stamped.pose.orientation.y = -0.333084863602;
		service_request.ik_request.pose_stamped.pose.orientation.z = 0.333084863604;
		service_request.ik_request.pose_stamped.pose.orientation.w = 0.623742313492;

  		/* Call the service */
		service_client.call(service_request, service_response);
		ROS_INFO_STREAM(
			"Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ")
			<< service_response.error_code.val);

		std::vector<double> joint_group_positions;


		for (int i = 0; i < 7; ++i)
		{		
			ROS_INFO("Converted from Pose to Joint Space with IK-> Joint %d position: %f",i,service_response.solution.joint_state.position[i]);			
		}

		joint_state_msg.velocity = franka_robot.getJointVelocities();
		joint_state_msg.position = franka_robot.getJointGoalPositions();

		teleop_cmd_pub.publish(joint_state_msg);
		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}