#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

struct location {
	double x;
	double y;
	double theta;
};

location PICKUP = {-3.86, 1.14, 0.88 };
location DROPOFF = {2.36, -2.06, -1.1};

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  //Moving to pickup site

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = PICKUP.x;
  goal.target_pose.pose.position.y = PICKUP.y;
  goal.target_pose.pose.orientation.w = PICKUP.theta;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Robot is navigating to the pickup site");
  ac.sendGoal(goal);
  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached pickup site
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Robot picked up the virtual object");
  	ros::Duration(5).sleep();

  	// Moving to dropoff site

  	// Define a position and orientation for the robot to reach
  	goal.target_pose.pose.position.x = DROPOFF.x;
  	goal.target_pose.pose.position.y = DROPOFF.y;
  	goal.target_pose.pose.orientation.w = DROPOFF.theta;

  	// Send the goal position and orientation for the robot to reach
  	ROS_INFO("Robot is navigating to the dropoff site");
  	ac.sendGoal(goal);
  	// Wait an infinite time for the results
  	ac.waitForResult();

  	// Check if the robot reached dropoff site
  	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	  ROS_INFO("Robot dropped the virtual object");
	else
	  ROS_INFO("Robot was unable to reach dropoff site for some reason");

  } else {
    ROS_INFO("Robot was unable to reach pickup site for some reason");
  }
  return 0;
}
