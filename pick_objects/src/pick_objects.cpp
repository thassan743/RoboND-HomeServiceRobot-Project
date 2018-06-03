#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  ros::Publisher status_pub = n.advertise<std_msgs::String>("status", 1000);
  ros::Rate loop_rate(1);
  std_msgs::String msg;
  
  //publish status
  msg.data = "START";
  status_pub.publish(msg);
  ros::spinOnce();

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

  // Define position and orientation of the pickup location
  goal.target_pose.pose.position.x = 4.0;
  goal.target_pose.pose.position.y = 8.0;
  goal.target_pose.pose.orientation.w = 1.0;
  
  //update status
  msg.data = "MOVE: PICKUP";
  status_pub.publish(msg);
  ros::spinOnce();
  
  // Send the pickup location to the robot
  ROS_INFO("Sending pickup location...");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Robot arrived at pickup location");
  else
    ROS_INFO("Robot failed to arrive at pickup location");
  
  //update status
  msg.data = "PICKUP";
  status_pub.publish(msg);
  ros::spinOnce();
  
  sleep(5);
  
  goal.target_pose.header.stamp = ros::Time::now();

  // Define position and orientation of the drop-off location
  goal.target_pose.pose.position.x = -4.0;
  goal.target_pose.pose.position.y = 3.0;
  goal.target_pose.pose.orientation.w = 1.0;
  
  //update status
  msg.data = "MOVE: DROP";
  status_pub.publish(msg);
  ros::spinOnce();
  
  // Send the drop-off location to the robot
  ROS_INFO("Sending drop-off location...");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Robot arrived at drop-off location");
  else
    ROS_INFO("Robot failed to arrive at drop-off location");
  
  //update status
  msg.data = "DROP";
  status_pub.publish(msg);
  ros::spin();
  
  return 0;
}
