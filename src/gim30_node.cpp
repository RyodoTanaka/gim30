#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>

using namespace std;

// This class is getting the 3d data from gim30 + hokuyo2d
// To get hokuyo2d data(2D data), we use urg libraries.
// So, you have to install that libraries. 
// For more info, Go "http://sourceforge.net/p/urgnetwork/wiki/top_jp/" 
// About gim30, Go "http://ci.nii.ac.jp/els/110007503932.pdf?id=ART0009334101&type=pdf&lang=jp&host=cinii&order_no=&ppv_type=0&lang_sw=&no=1430472704&cp="
namespace Gim30
{
  class Control{
  public:
    Control();
    ~Control();
    
    float old_angle;
    float new_angle;

  private:
    void SetGim30();
    void CloseGim30();
    void StartGim30();
    void StopGim30();

    void callback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& diag_msg);
    ros::Subscriber diag_msg_sub;

    string portname;
    int fd;
    struct termios oldtio;
    struct termios newtio;
    string data;
    float GIMangle_old;
    float GIMangle_new;
  
    ros::NodeHandle n;
    string diag_sub_name;
  };

  class Get3d : public nodelet::Nodelet, public Gim30::Control
  {
  public:
    Get3d() :
      pc_pub_name("gim30/point_cloud"),
      pc2_pub_name("gim30/point_cloud2"),
      laser_sub_name("gim30/laser"),
      frame_id("gim30")
    {}
  private:
    virtual void onInit();
    void callback(const sensor_msgs::LaserScan::ConstPtr& laser);
  
    string pc_pub_name;
    string pc2_pub_name;
    string laser_sub_name;
    string frame_id;

    ros::Publisher pc_pub;
    ros::Publisher pc2_pub;
    ros::Subscriber sub;
  };
}

Gim30::Control::Control() :
  portname("/dev/ttyUSB0"),
  diag_sub_name("diagnostic"),
  fd(-1),
  old_angle(0.0),
  new_angle(1.0)
{
  n.param("gim30/portname", portname, portname);
  n.param("gim30/fd", fd, fd);
  data.resize(8,'\0');
  SetGim30();
  StartGim30();
  
  diag_msg_sub = n.subscribe<diagnostic_msgs::DiagnosticStatus>(diag_sub_name, 40, &Gim30::Control::callback, this);
}

Gim30::Control::~Control()
{
  StopGim30();
  CloseGim30();
}

void Gim30::Control::SetGim30()
{
  if(fd > 0){
    ROS_FATAL("Gim30 is already open");
  }
  else if((fd = open(portname.c_str(), O_RDWR) ) < 0){
    ROS_FATAL("Failed to open the port : Gime30");
    ROS_BREAK();
  } 
  
  //Get old termios for Gim30
  if(ioctl(fd, TCGETS, &oldtio) < 0){
    ROS_FATAL("Gim30 ioctl TCGET error");
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
    ROS_FATAL("Gim30 ioctl TCSETS error");
    ROS_BREAK();
  }
  ROS_INFO("Succeed to set Gim30.");
}

void Gim30::Control::CloseGim30()
{
  if(fd < 0){
    ROS_FATAL("Gim30 Port is already closed.");
    ROS_BREAK();
  }
  close(fd);
  fd = -1;
  ROS_INFO("Succeed to close Gim30.");
}

void Gim30::Control::StartGim30()
{
  if(write(fd, "START\r\n", sizeof("START\r\n")) != sizeof("START\r\n")){
    ROS_FATAL("Gim30 write error. Cannot start Gim30.");
    ROS_BREAK();
  }
  ROS_INFO("Succeed to start GIm30.");
}

void Gim30::Control::StopGim30()
{
  if(write(fd, "STOP\r\n", sizeof("STOP\r\n")) != sizeof("STOP\r\n")){
    ROS_FATAL("Gim30 write error. Cannot stop Gim30.");
    ROS_BREAK();
  }
  ROS_INFO("Succeed to stop Gim30.");
}

void Gim30::Control::callback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& diag_msg)
{
  old_angle = new_angle;
  int tmp;
  for(int i=0; i<8; i+=tmp)
    tmp = read(fd, &data[i], 8);
  new_angle = -(atoi(&data[1]) / 100.0);
}

void Gim30::Get3d::onInit()
{
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  private_nh.param("gim30/pc_pub_name", pc_pub_name, pc_pub_name);
  private_nh.param("gim30/pc2_pub_name", pc2_pub_name, pc2_pub_name);
  private_nh.param("gim30/laser_sub_name", laser_sub_name, laser_sub_name);
  private_nh.param("gim30/frame_id", frame_id, frame_id);

  
  pc_pub = private_nh.advertise<sensor_msgs::PointCloud>(pc_pub_name, 40);
  pc2_pub = private_nh.advertise<sensor_msgs::PointCloud2>(pc2_pub_name, 40);
  sub = private_nh.subscribe(laser_sub_name, 40, &Get3d::callback, this);
}

void Gim30::Get3d::callback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
  ros::sensor_msgs::PointCloud pc_data;
  ros::sensor_msgs::PointCloud2 pc2_data;

  pc_data.header.frame_id = frame_id;
  pc_data.channels.resize(1);
  pc_data.channels[0].name = "intensities";
  pc_data.points.resize(laser->ranges.size());
  pc_data.channels[0].values.resize(laser->intensities.size());

}

int main(int argc, char* argv[])
{
  return 0;
}
