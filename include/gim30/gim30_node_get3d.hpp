#ifndef __GIM30_NODE__
#define __GIM30_NODE__

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>

#include "gim30_node_control.hpp"

using namespace std;

namespace Gim30{
  class Get3d : private Gim30::Control
  {
  public:
    Get3d();

  private:
    void callback(const sensor_msgs::LaserScan::ConstPtr& laser);
  
    string pc_pub_name;
    string pc2_pub_name;
    string laser_sub_name;
    string frame_id;
    bool publish_intensity;

    double angle_min;
    double angle_max;
    double deg_min;
    double deg_max;
    double get_steps;

    ros::NodeHandle n;
    ros::Publisher pc_pub;
    ros::Publisher pc2_pub;
    ros::Subscriber sub;
  };
}

Gim30::Get3d::Get3d() :
  pc_pub_name("point_cloud"),
  pc2_pub_name("point_cloud2"),
  laser_sub_name("laser"),
  frame_id("gim30"),
  publish_intensity(true),
  angle_min(-M_PI/2.0),
  angle_max(M_PI/2.0)
{
  n.param("pc_pub_name", pc_pub_name, pc_pub_name);
  n.param("pc2_pub_name", pc2_pub_name, pc2_pub_name);
  n.param("laser_sub_name", laser_sub_name, laser_sub_name);
  n.param("frame_id", frame_id, frame_id);
  n.param("publish_intensity", publish_intensity, publish_intensity);
  n.param("angle_min", angle_min, angle_min);
  n.param("angle_max", angle_max, angle_max);

  deg_min = angle_min*180.0/M_PI;
  deg_max = angle_max*180.0/M_PI;
  get_steps = (deg_max - deg_min)*4.0;

  pc_pub = n.advertise<sensor_msgs::PointCloud>(pc_pub_name, 40);
  pc2_pub = n.advertise<sensor_msgs::PointCloud2>(pc2_pub_name, 40);
  sub = n.subscribe(laser_sub_name, 40, &Get3d::callback, this);
}

void Gim30::Get3d::callback(const sensor_msgs::LaserScan::ConstPtr& laser)
{

  GetAngle();

  sensor_msgs::PointCloud pc_data;
  sensor_msgs::PointCloud2 pc2_data;
 
  pc_data.header.frame_id = frame_id;
  pc_data.channels.resize(1);
  pc_data.channels[0].name = "intensities";
  pc_data.points.resize(laser->ranges.size());
  pc_data.channels[0].values.resize(laser->intensities.size());

  double tmp;
  double alpha;
  double beta;
  
  for(double i=0; i<=get_steps; i++){
    tmp = new_angle-old_angle;
    if(tmp > 180)
      tmp = 360 - tmp;
    tmp = (deg_max-deg_min)*tmp*i/(360.0*get_steps) + old_angle + (180.0+deg_min)/360.0*tmp;
    alpha = atan(0.57734*sin(tmp*M_PI/180.0));
    beta = -atan(0.57734*cos(tmp*M_PI/180.0)*cos(alpha));
    tmp = (angle_max - angle_min)*i/get_steps + angle_min;    
    pc_data.points[i].x = laser->ranges[i]*cos(tmp)*cos(beta) + 0.039*sin(beta);
    pc_data.points[i].y = laser->ranges[i]*cos(tmp)*sin(alpha)*sin(beta) + laser->ranges[i]*sin(tmp)*cos(alpha) - 0.039*sin(alpha)*cos(beta);
    pc_data.points[i].z = -laser->ranges[i]*cos(tmp)*cos(alpha)*sin(beta) + laser->ranges[i]*sin(tmp)*sin(alpha) + 0.039*cos(alpha)*cos(beta);
    if(publish_intensity)
      pc_data.channels[0].values[i] = laser->intensities[i];
  } 
  
  pc_data.header.stamp = ros::Time::now();

  sensor_msgs::convertPointCloudToPointCloud2(pc_data, pc2_data);

  pc_pub.publish(pc_data);
  pc2_pub.publish(pc2_data);
}

#endif 
