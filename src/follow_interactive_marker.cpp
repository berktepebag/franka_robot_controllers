#include <follow_interactive_marker.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>
// Kinematics
#include <moveit_msgs/GetPositionIK.h>

geometry_msgs::Pose planned_pose;
std::vector<double> joint_group_positions = {0*M_PI/180, -M_PI_4*M_PI/180, 0*M_PI/180, -3 * M_PI_4*M_PI/180, 0*M_PI/180, M_PI_2*M_PI/180, M_PI_4*M_PI/180};

void frankaRobotTeleopCallback(const visualization_msgs::InteractiveMarkerUpdate::ConstPtr& msg) {

	ROS_INFO("Received message");

	planned_pose = msg->poses[0].pose;	
}



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

	//Subscribe to moveit's interactive marker pose
	ros::Subscriber marker_sub = nh.subscribe("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update", 1, &frankaRobotTeleopCallback);
	

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

		/*
		ROS_INFO("Pose x: %f", planned_pose.position.x);
		ROS_INFO("Pose y: %f", planned_pose.position.y);
		ROS_INFO("Pose z: %f", planned_pose.position.z);
		*/


		service_request.ik_request.group_name = "panda_arm";
		service_request.ik_request.pose_stamped.pose.position.x = planned_pose.position.x;
		service_request.ik_request.pose_stamped.pose.position.y = planned_pose.position.y;
		service_request.ik_request.pose_stamped.pose.position.z = planned_pose.position.z;

		service_request.ik_request.pose_stamped.pose.orientation.x = planned_pose.orientation.x;
		service_request.ik_request.pose_stamped.pose.orientation.y = planned_pose.orientation.y;
		service_request.ik_request.pose_stamped.pose.orientation.z = planned_pose.orientation.z;
		service_request.ik_request.pose_stamped.pose.orientation.w = planned_pose.orientation.w;

  		//Call the service
		service_client.call(service_request, service_response);
		ROS_INFO_STREAM(
			"Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ")
			<< service_response.error_code.val);

		


		for (int i = 0; i < 7; ++i)
		{		
			ROS_INFO("Converted from Pose to Joint Space with IK-> Joint %d position: %f",i,service_response.solution.joint_state.position[i]*180/M_PI);		
			joint_group_positions[i] = 	service_response.solution.joint_state.position[i];
		}
		

		joint_state_msg.velocity = {0.02,0.02,0.02,0.02,0.02,0.02,0.02};
		joint_state_msg.position = joint_group_positions;

		teleop_cmd_pub.publish(joint_state_msg);
		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}