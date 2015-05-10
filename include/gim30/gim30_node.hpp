#ifndef __GIM30_NODE__
#define __GIM30_NODE__

#include <ros/ros.h>

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

  int GetDatas();

  // URG args //////////////////////////
  bool publish_intensity;
  long timestamp;
  double deg_min;
  double deg_max;
  double rad_min;
  double rad_max;
  int skip;
  double range_min;
  double range_max;
  int step;
  float **ranges;
  float **intensities;
  //////////////////////////////////////

  // Gim30 args ////////////////////////
  double old_angle;
  double new_angle;
  //////////////////////////////////////

private: 
  // Serial //////////////////////////////////////
  bool GetAngle();
  bool GetOldAngle();
  bool GetNewAngle();
  void OpenGim30();
  void CloseGim30();
  void StartGim30();
  void StopGim30();

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
  /////////////////////////////////////////////////
};
#endif
