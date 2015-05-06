#ifndef __GIM30_NODE_GIMBAL__
#define __GIM30_NODE_GIMBAL__

#include <ros/ros.h>

#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>

using namespace std;

namespace Gim30{
  class Gimbal{
  public:
    Gimbal();
    ~Gimbal();
    
    bool GetOldAngle();
    bool GetNewAngle();
    
    double old_angle;
    double new_angle;

  private:
    void OpenGim30();
    void CloseGim30();
    void StartGim30();
    void StopGim30();

    string portname;
    int fd;
    struct termios oldtio;
    struct termios newtio;
    string data;

    ros::NodeHandle n;
  };
}

Gim30::Gimbal::Gimbal() :
  portname("/dev/ttyUSB0"),
  fd(-1),
  old_angle(0.0),
  new_angle(1.0)
{
  n.param("gim30/portname", portname, portname);
  data.resize(8,'\0');
  OpenGim30();
  StartGim30();
}

Gim30::Gimbal::~Gimbal()
{
  cout << "here" << endl;
  StopGim30();
  CloseGim30();
}

void Gim30::Gimbal::OpenGim30()
{
  if(fd > 0){
    ROS_WARN("Gim30 is already open");
  }
  else if((fd = open(portname.c_str(), O_RDWR) ) < 0){
    ROS_WARN("Failed to open the port : Gime30");
    ROS_BREAK();
  } 
  
  //Get old termios for Gim30
  if(ioctl(fd, TCGETS, &oldtio) < 0){
    ROS_WARN("Gim30 ioctl TCGET error");
    ROS_BREAK();
  }
  //Copy old to new 
  newtio = oldtio;  

  //New serial settings of Gim30
  newtio.c_cflag = (B115200 | CS8 | CREAD | CLOCAL);
  newtio.c_iflag = (IGNPAR | ICRNL);
  newtio.c_oflag = 0;
  newtio.c_lflag = ~ICANON;

  //Set new serial settings of Gim30
  if(ioctl(fd, TCSETS, &newtio) < 0){
    ROS_WARN("Gim30 ioctl TCSETS error");
    ROS_BREAK();
  }
  ROS_INFO("Succeed to set Gim30.");
}

void Gim30::Gimbal::CloseGim30()
{
  if(fd < 0){
    ROS_WARN("Gim30 Port is already closed.");
    ROS_BREAK();
  }
  close(fd);
  fd = -1;
  ROS_INFO("Succeed to close Gim30.");
}

void Gim30::Gimbal::StartGim30()
{
  if(write(fd, "START\r\n", sizeof("START\r\n")) != sizeof("START\r\n")){
    ROS_WARN("Gim30 write error. Cannot start Gim30.");
    ROS_BREAK();
  }
  ROS_INFO("Succeed to start GIm30.");
}

void Gim30::Gimbal::StopGim30()
{
  if(write(fd, "STOP\r\n", sizeof("STOP\r\n")) != sizeof("STOP\r\n")){
    ROS_WARN("Gim30 write error. Cannot stop Gim30.");
    ROS_BREAK();
  }
  ROS_INFO("Succeed to stop Gim30.");
}

bool Gim30::Gimbal::GetOldAngle()
{
  int tmp;
  for(int i=0; i<8; i+=tmp){
    tmp = read(fd, &data[i], 8);
    if(tmp < 0)
      return false;
  }
  old_angle = -(atoi(&data[1]) / 100.0);
  return true;
}

bool Gim30::Gimbal::GetNewAngle()
{
  int tmp;
  for(int i=0; i<8; i+=tmp){
    tmp = read(fd, &data[i], 8);
    if(tmp < 0)
      return false;
  }
  new_angle = -(atoi(&data[1]) / 100.0);
  return true;
}

#endif
