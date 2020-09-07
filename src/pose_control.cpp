#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <iostream>
#include <string>

#define VEL_MAX 1
using namespace ros;
using namespace geometry_msgs;

int goal_flag_1 = false;
int goal_flag_2 = false;
Point uav1_goal;
Point uav2_goal;
Twist twist1;
Twist twist2;
Publisher vel_pub_1;
Publisher vel_pub_2;

//sign function
int sign(double x)
{
  if(x > 0)
  {
    return 1;
  }
  else if(x < 0)
  {
    return -1;
  }
  else 
  {
    return 0;
  }
}

//get the vel based on goalpose and nowpose
Twist velDisposal1(PoseStamped now)
{
  double delta_x = uav1_goal.x - now.pose.position.x;
  double delta_y = uav1_goal.y - now.pose.position.y;
  double delta_z = uav1_goal.z - now.pose.position.z;
  double P = 1;
  
  if(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2) > 0.01 && goal_flag_1)
  {
    //set the limitation of vel
    twist1.linear.x = (abs(P*delta_x) < VEL_MAX)? P*delta_x: sign(P*delta_x)*VEL_MAX;
    twist1.linear.y = (abs(P*delta_y) < VEL_MAX)? P*delta_y: sign(P*delta_y)*VEL_MAX;
    twist1.linear.z = (abs(P*delta_z) < VEL_MAX)? P*delta_z: sign(P*delta_z)*VEL_MAX;
  }
  else 
  {
    twist1.linear.x = 0;
    twist1.linear.y = 0;
    twist1.linear.z = 0;
  }
  
  return twist1;
}

//get the vel based on goalpose and nowpose
Twist velDisposal2(PoseStamped now)
{
  double delta_x = uav2_goal.x - now.pose.position.x;
  double delta_y = uav2_goal.y - now.pose.position.y;
  double delta_z = uav2_goal.z - now.pose.position.z;
  double P = 1;
  
  if(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2) > 0.01 && goal_flag_2)
  {
    //set the limitation of vel
    twist2.linear.x = (abs(P*delta_x) < VEL_MAX)? P*delta_x: sign(P*delta_x)*VEL_MAX;
    twist2.linear.y = (abs(P*delta_y) < VEL_MAX)? P*delta_y: sign(P*delta_y)*VEL_MAX;
    twist2.linear.z = (abs(P*delta_z) < VEL_MAX)? P*delta_z: sign(P*delta_z)*VEL_MAX;
  }
  else 
  {
    twist2.linear.x = 0;
    twist2.linear.y = 0;
    twist2.linear.z = 0;
  }
  
  return twist2;
}

void nowPoseCallback1(const PoseStamped::ConstPtr& msg)
{
   twist1 = velDisposal1(*msg);
   
   vel_pub_1.publish(twist1);
  
   //for debugging
   //ROS_INFO("uav1_vel: x:%0.6f, y:%0.6f, z:%0.6f", twist1.linear.x, twist1.linear.x, twist1.linear.z);
}

void nowPoseCallback2(const PoseStamped::ConstPtr& msg)
{
   twist2 = velDisposal2(*msg);
   
   vel_pub_2.publish(twist2);
  
   //for debugging
   //ROS_INFO("uav2_vel: x:%0.6f, y:%0.6f, z:%0.6f", twist2.linear.x, twist2.linear.x, twist2.linear.z);
}

//get the goal posi of uav1 as goal
void goalPoseCallback1(const Point::ConstPtr& msg)
{
  uav1_goal = *msg;
  goal_flag_1 = true;

  //for debugging
  //ROS_INFO("uav1_goal_pose: x:%0.6f, y:%0.6f", goal.x, goal.y);
}

void goalPoseCallback2(const Point::ConstPtr& msg)
{
  uav2_goal = *msg;
  goal_flag_2 = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_control");
  ros::NodeHandle n;
  
  //get the goal pose of uav1
  ros::Subscriber uav1_goal_pose_sub = n.subscribe("/uav1/pose_goal", 1000, goalPoseCallback1);
  ros::Subscriber uav2_goal_pose_sub = n.subscribe("/uav2/pose_goal", 1000, goalPoseCallback2);
  //get the now pose of uav1
  ros::Subscriber uav1_now_pose_sub = n.subscribe("/uav1/mavros/local_position/pose", 1000, nowPoseCallback1);
  ros::Subscriber uav2_now_pose_sub = n.subscribe("/uav2/mavros/local_position/pose", 1000, nowPoseCallback2);
  //pub the vel whitch is subscribed by ros_nav_quadrotor
  vel_pub_1 = n.advertise<Twist>("/uav1/ros_follower_quadrotor_node/cmd_follower_vel", 1000);
  vel_pub_2 = n.advertise<Twist>("/uav2/ros_follower_quadrotor_node/cmd_follower_vel", 1000);
  spin();
  return 0;
}
