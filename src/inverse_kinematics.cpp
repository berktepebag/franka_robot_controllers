
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// Kinematics
#include <moveit_msgs/GetPositionIK.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "panda_arm";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  ros::ServiceClient service_client = node_handle.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
  while (!service_client.exists())
  {
    ROS_INFO("Waiting for service");
    sleep(1.0);
  }

  moveit_msgs::GetPositionIK::Request service_request;
  moveit_msgs::GetPositionIK::Response service_response;

  eometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;

  service_request.ik_request.group_name = "panda_arm";
  //service_request.ik_request.pose_stamped.header.frame_id = "panda_joint8";
  service_request.ik_request.pose_stamped.pose.position.x = current_pose.position.x;//0.088;
  service_request.ik_request.pose_stamped.pose.position.y = current_pose.position.y;//0.0;
  service_request.ik_request.pose_stamped.pose.position.z = current_pose.position.z;//0.0;

  service_request.ik_request.pose_stamped.pose.orientation.x = current_pose.orientation.x;//0.623742313489;
  service_request.ik_request.pose_stamped.pose.orientation.y = current_pose.orientation.y;//-0.333084863602;
  service_request.ik_request.pose_stamped.pose.orientation.z = current_pose.orientation.z;//0.333084863604;
  service_request.ik_request.pose_stamped.pose.orientation.w = current_pose.orientation.w;//0.623742313492;

  /* Call the service */
  service_client.call(service_request, service_response);
  ROS_INFO_STREAM(
    "Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ")
    << service_response.error_code.val);

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;


  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  for (int i = 0; i < joint_group_positions.size(); ++i)
  {
    ROS_INFO("Reading from actuator sensors-> Joint %d position: %f",i,joint_group_positions[i]);
    ROS_INFO("Converted from Pose to Joint Space with IK-> Joint %d position: %f",i,service_response.solution.joint_state.position[i]);
  }
