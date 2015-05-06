#ifndef __GIM30_NODE_URG__
#define __GIM30_NODE_URG__

#include <ros/ros.h>

#include <urg_sensor.h>
#include <urg_utils.h>

#include <iostream>

using namespace std;

namespace Gim30{
  class URG
  {
  public:
    URG();
     ~URG();

    bool GetURGData();
    
    bool publish_intensity;
    double deg_min;
    double deg_max;
    double rad_min;
    double rad_max;
    int skip;
    double range_min;
    double range_max;
    int step;
    long *ranges_raw;
    double *ranges;
    unsigned short *intensities;

    ros::NodeHandle n;
  
  private:
    void OpenURG();
    void CloseURG();
    
    string ip_address;
    int ip_port;
    string serial_port;
    int serial_baud;

    urg_t urg;
    long timestamp;
  };
}

Gim30::URG::URG() :
  ip_address("192.168.0.10"),
  ip_port(10940),
  serial_port("/dev/ttyACM0"),
  serial_baud(115200),
  publish_intensity(true),
  deg_min(-90.0),
  deg_max(90.0),
  skip(0),
  range_min(0.1),
  range_max(30.0)
{
  n.param("gim30/ip_address", ip_address, ip_address);
  n.param("gim30/ip_port", ip_port, ip_port);
  n.param("gim30/serial_port", serial_port, serial_port);
  n.param("gim30/serial_baud", serial_baud, serial_baud);
  n.param("gim30/publish_intensity", publish_intensity, publish_intensity);
  n.param("gim30/degree_minimum", deg_min, deg_min);
  n.param("gim30/degree_maximum", deg_max, deg_max);
  n.param("gim30/skip", skip, skip);
  n.param("gim30/range_minimum", range_min, range_min);
  n.param("gim30/range_maximum", range_max, range_max);

  OpenURG();

}

Gim30::URG::~URG()
{
  CloseURG();
}

void Gim30::URG::OpenURG()
{
  if(urg_open(&urg,URG_SERIAL,serial_port.c_str(),serial_baud) < 0){  
    if(urg_open(&urg,URG_ETHERNET,ip_address.c_str(),ip_port) < 0){
      ROS_WARN("URG on Gim30 open error.");
      ROS_BREAK();
    }
  }

  urg_set_scanning_parameter(&urg, urg_deg2step(&urg,deg_min), urg_deg2step(&urg,deg_max), skip);

  rad_min = deg_min*M_PI/180.0;
  rad_max = deg_max*M_PI/180.0;
  step = (deg_max - deg_min)*4.0;

  ranges_raw = new long[step+1];
  ranges = new double[step+1];
  intensities = new unsigned short[step+1];
}

void Gim30::URG::CloseURG()
{
  delete ranges_raw;
  delete ranges;
  delete intensities;
  urg_close(&urg);
}

bool Gim30::URG::GetURGData()
{
  if(publish_intensity){
    urg_start_measurement(&urg, URG_DISTANCE_INTENSITY, 1, 0);
    if(urg_get_distance_intensity(&urg, ranges_raw, intensities, &timestamp) <= 0){
      ROS_WARN("Disable to get URG data on Gim30.");
      return false;
    }
  }
  else{
    urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
    if(urg_get_distance(&urg, ranges_raw, &timestamp) <= 0){
      ROS_WARN("Disable to get URG data on Gim30.");
      return false;
    }
  }
  return true;
}
#endif
