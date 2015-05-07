#include <ros/ros.h>

#include "gim30/gim30_node.hpp"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "gim30");
  ros::NodeHandle n;

  Gim30 gim30(n);

  // ros::Rate r(80.0);

  while(n.ok()){
    ROS_INFO("here");
    gim30.CalculateDatas();
    // r.sleep();
  }
  return 0;
}
