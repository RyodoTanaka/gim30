#include <ros/ros.h>

#include "gim30/gim30_node_calculate.hpp"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "gim30");

  Gim30::Calculate gim30;

  ros::Rate r(40);

  while(gim30.n.ok()){
    gim30.CalculateDatas();
    r.sleep();
  }
  return 0;
}
