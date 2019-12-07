#include "ros/ros.h"
#include "std_msgs/String.h"
#include <parallel_parking.h>
#include "drive_ros_msgs/EnvironmentModel.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  drive_ros_msgs::EnvironmentModel msg;
  ros::Publisher chatter_pub = n.advertise<drive_ros_msgs::EnvironmentModel>("chatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    msg.front_distance = 1.1;
    msg.current_lane = 2;
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}

