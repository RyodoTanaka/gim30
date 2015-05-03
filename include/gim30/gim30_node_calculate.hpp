#ifndef __GIM30_NODE_CALCULATE__
#define __GIM30_NODE_CALCULATE__

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "gim30_node_gimbal.hpp"
#include "gim30_node_urg.hpp"

#include <iostream>
#include <cmath>

using namespace std;

namespace Gim30{
  class Calculate : private Gim30::Gimbal, private Gim30::URG
  {
  public:
    Calculate();
    
    void CalculateDatas();

    ros::NodeHandle n;

  private:
    
    int GetDatas();

    string pc_pub_name;
    string pc2_pub_name;
    string frame_id;

    sensor_msgs::PointCloud pc_data;
    sensor_msgs::PointCloud2 pc2_data;

    ros::Publisher pc_pub;
    ros::Publisher pc2_pub;
  };
}

Gim30::Calculate::Calculate() :
  pc_pub_name("point_cloudino"),
  pc2_pub_name("point_cloud2"),
  frame_id("gim30")
{
  n.param("gim30/pc_pub_name", pc_pub_name, pc_pub_name);
  n.param("gim30/pc2_pub_name", pc2_pub_name, pc2_pub_name);
  n.param("gim30/frame_id", frame_id, frame_id);

  pc_data.header.frame_id = frame_id;
  pc_data.channels.resize(1);
  pc_data.channels[0].name = "intensities";
  pc_data.points.resize(step);
  pc_data.channels[0].values.resize(step);

  pc_pub = n.advertise<sensor_msgs::PointCloud>(pc_pub_name, 40);
  pc2_pub = n.advertise<sensor_msgs::PointCloud2>(pc2_pub_name, 40);
}


int Gim30::Calculate::GetDatas()
{ 
  if(!GetOldAngle())
    return 1;
  if(!GetURGData())
    return 2;
  if(!GetNewAngle())
    return 3;
 
  for(int i=0; i<step; i++){
    ranges[i] = (double)ranges_raw[i] / 1000.0;
    if(ranges[i] > range_max || ranges[i] < range_min)
      ranges[i] = 0;
  }

  return 0;
}

void Gim30::Calculate::CalculateDatas()
{
  double tmp;
  double alpha;
  double beta;
  
  if(!GetDatas()){
    for(int i=0; i<=step; i++){
      tmp = new_angle-old_angle;
      if(tmp > 180)
	tmp = 360 - tmp;
      tmp = (deg_max-deg_min)*tmp*i/(360.0*step) + old_angle + (180.0+deg_min)/360.0*tmp;
      alpha = atan(0.57734*sin(tmp*M_PI/180.0));
      beta = -atan(0.57734*cos(tmp*M_PI/180.0)*cos(alpha));
      tmp = (rad_max - rad_min)*i/step + rad_min;    
      pc_data.points[i].x = (double)ranges[i]/1000.0*cos(tmp)*cos(beta) + 0.039*sin(beta);
      pc_data.points[i].y = (double)ranges[i]/1000.0*cos(tmp)*sin(alpha)*sin(beta) + (double)ranges[i]/1000.0*sin(tmp)*cos(alpha) - 0.039*sin(alpha)*cos(beta);
      pc_data.points[i].z = -(double)ranges[i]/1000.0*cos(tmp)*cos(alpha)*sin(beta) + (double)ranges[i]/1000.0*sin(tmp)*sin(alpha) + 0.039*cos(alpha)*cos(beta);
      if(publish_intensity)
	pc_data.channels[0].values[i] = intensities[i];
    } 

    pc_data.header.stamp = ros::Time::now();

    sensor_msgs::convertPointCloudToPointCloud2(pc_data, pc2_data);

    pc_pub.publish(pc_data);
    pc2_pub.publish(pc2_data);
  }
  else{
    ROS_WARN("Anable to get Gim30 datas.");
  }
}

#endif
