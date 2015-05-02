#include <ros/ros.h>

#include "gim30/gim30_node_get3d.hpp"

int main(int argc, char* argv[])
{
  // init ros
  ros::init(argc, argv, "gim30");

  //original class
  Gim30::Get3d gim30_get3d;
  
  ros::spin();

  return 0;
}
