#include <ros/ros.h>
#include <formation/Point.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_pub");

  ros::NodeHandle n;

  ros::Publisher point_info_pub = n.advertise<formation::Point>("/point_info", 10);

  ros::Rate loop_rate(1);

  int count = 0;
  while(ros::ok())
  {
    formation::Point point_msg;
    
    point_msg.position.x = 5;
    point_msg.position.y = 6;
    point_info_pub.publish(point_msg);

    ROS_INFO("publish point info: x:%0.6f, y:%0.6f", point_msg.position.x, point_msg.position.y);

    loop_rate.sleep();
  }
  return 0;
}
