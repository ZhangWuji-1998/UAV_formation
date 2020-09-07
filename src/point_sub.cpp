#include <ros/ros.h>
#include <formation/Point.h>

void pointInfoback(const formation::PointPtr& msg)
{
  float posi_x, posi_y;  
  posi_x = (*msg).position.x;
  posi_y = (*msg).position.y;
  
  ROS_INFO("subscribe point info: x:%0.6f, y:%0.6f", posi_x, posi_y);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_sub");

  ros::NodeHandle n;
  
  ros::Subscriber point_info_sub = n.subscribe("/point_info", 10, pointInfoback);

  ros::spin();
 
  return 0;
}
