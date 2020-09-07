#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>


void posiCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  float posi_x, posi_y;  

  posi_x = (*msg).pose.position.x;
  posi_y = (*msg).pose.position.y;
  
  ROS_INFO("uav0_pose: x:%0.6f, y:%0.6f", posi_x, posi_y);
}

int main(int argc, char **argv)
{
  //init the position_publisher node
  ros::init(argc, argv, "position_subscriber");
  
  //create a node handle
  ros::NodeHandle n;

  //create a subscriber
  ros::Subscriber posi_sub = n.subscribe("/uav0/mavros/local_position/pose", 10, posiCallback); 
  ROS_INFO("subscriber starts!");
  //wait for callback 
  ros::spin();

  return 0;
}     
