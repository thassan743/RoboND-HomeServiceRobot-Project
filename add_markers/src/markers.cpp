#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include <string>

geometry_msgs::Pose robotPose;
std::string status;

visualization_msgs::Marker create_marker(int action, geometry_msgs::Pose& pose)
{
  // Set shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  
  //Create marker
  visualization_msgs::Marker marker;
  
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "marker";
  marker.id = 0;

  // Set the marker type.
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = action;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose = pose;
  
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  
  return marker;
}

//Callback to handle robot status
void update_status(const std_msgs::String::ConstPtr& msg)
{
  status = msg->data;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  visualization_msgs::Marker marker;
  geometry_msgs::Pose markerPose;
    
  //publish marker
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  //subscribe to robot status
  ros::Subscriber status_sub = n.subscribe("status", 100, update_status);
  
  //Initialize marker pose
  markerPose.position.x = 0;
  markerPose.position.y = 0;
  markerPose.position.z = 0;
  markerPose.orientation.x = 0;
  markerPose.orientation.y = 0;
  markerPose.orientation.z = 0;
  markerPose.orientation.w = 0;
  
  // Wait for subscriber
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  
  // Publish marker at pickup location
  ROS_INFO("Publish pickup marker");
  markerPose.position.x = 4.0;
  markerPose.position.y = 8.0;
  marker = create_marker(visualization_msgs::Marker::ADD, markerPose);
  marker_pub.publish(marker);
  
  // Wait for robot to arrive at pickup location
  ROS_INFO("Waiting for pickup");
  while(status != "PICKUP")
  {
    ros::spinOnce();
  }
  
  //Hide pickup marker
  ROS_INFO("Marker picked up");
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
  
  //Move marker to drop-off location
  marker.pose.position.x = -4.0;
  marker.pose.position.y = 3.0;
  marker.action = visualization_msgs::Marker::ADD;
  
  // Wait for robot to reach drop-off location
  ROS_INFO("Waiting for drop-off");
  while(status != "DROP")
  {
    ros::spinOnce();
  }
  
  //Publish drop-off marker
  ROS_INFO("Publish drop-off marker");
  marker_pub.publish(marker);
  sleep(5);
  
  ros::spin();
  
  return 0;
}
