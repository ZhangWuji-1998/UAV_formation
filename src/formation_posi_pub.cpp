#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

float d = 1;

ros::Publisher posepub1;
ros::Publisher posepub2;

geometry_msgs::Point uav1_relapose(geometry_msgs::PoseStamped uav0_now)
{
  geometry_msgs::Point uav1_goal;
  
  uav1_goal.x = uav0_now.pose.position.x + d;
  uav1_goal.y = uav0_now.pose.position.y;
  uav1_goal.z = uav0_now.pose.position.z;
  
  //used for debugging
  //ROS_INFO("uav1_pose: x:%0.6f, y:%0.6f", uav1_goal.x, uav1_goal.y);
  
  return uav1_goal;
}

geometry_msgs::Point uav2_relapose(geometry_msgs::PoseStamped uav0_now)
{
  geometry_msgs::Point uav2_goal;
  
  uav2_goal.x = uav0_now.pose.position.x - d;
  uav2_goal.y = uav0_now.pose.position.y;
  uav2_goal.z = uav0_now.pose.position.z;
  
  //used for debugging
  //ROS_INFO("uav1_pose: x:%0.6f, y:%0.6f", uav1_goal.x, uav1_goal.y);
  
  return uav2_goal;
}

void leaderPosiCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  geometry_msgs::Point uav1_goal;
  geometry_msgs::Point uav2_goal;
  uav1_goal = uav1_relapose(*msg);
  uav2_goal = uav2_relapose(*msg);
  
  //publish the pose_goal of uav1 (how to control the rate of publishing)
  posepub1.publish(uav1_goal);
  posepub2.publish(uav2_goal);

  //used for debugging
  //ROS_INFO("uav0_pose: x:%0.6f, y:%0.6f", (*msg).pose.position.x, (*msg).pose.position.y);


}

int main(int argc, char **argv)
{
  //init the position_publisher node
  ros::init(argc, argv, "formation_posi_pub");
  
  //create a node handle
  ros::NodeHandle n;

  //create a subscriber
  ros::Subscriber leader_sub = n.subscribe("/uav0/mavros/local_position/pose", 10, leaderPosiCallback);
 
  ROS_INFO("subscriber starts!");
  //create a publisher
  posepub1 = n.advertise<geometry_msgs::Point>("/uav1/pose_goal", 1000);
  posepub2 = n.advertise<geometry_msgs::Point>("/uav2/pose_goal", 1000);
  //wait for callback
  ros::spin();

  return 0;
}     
