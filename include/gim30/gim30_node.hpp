#ifndef __GIM30_NODE__
#define __GIM30_NODE__

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <iostream>
#include <cmath>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <urg_sensor.h>
#include <urg_utils.h>

using namespace std;

class Gim30{
public:
  Gim30(ros::NodeHandle &n);
  ~Gim30();

  void CalculateDatas();

private:
  // parameter server arguments /////////////////////
  string pc_pub_name;
  string pc2_pub_name;
  string frame_id;
  //////////////////////////////////////////////////

  // Publish //////////////////////////
  sensor_msgs::PointCloud pc_data;
  sensor_msgs::PointCloud2 pc2_data;
  ros::Publisher pc_pub;
  ros::Publisher pc2_pub;
  /////////////////////////////////////////////////

  // Calculate ////////////////////////////
  int GetDatas();
  /////////////////////////////////////////

  // Serial //////////////////////////////////////
  bool GetAngle();
  bool GetOldAngle();
  bool GetNewAngle();
  void OpenGim30();
  void CloseGim30();
  void StartGim30();
  void StopGim30();

  double old_angle;
  double new_angle;
  string portname;
  int fd;
  struct termios oldtio;
  struct termios newtio;
  string data;
  ////////////////////////////////////////////////

  // URG /////////////////////////////////////////
  void OpenURG();
  void CloseURG();
  bool GetURGData();

  string ip_address;
  int ip_port;
  string serial_port;
  int serial_baud;

  urg_t urg;
  long timestamp;
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
  /////////////////////////////////////////////////
};
#endif
