#include "gim30/gim30_node.hpp"

Gim30::Gim30(ros::NodeHandle &n) :
  pc_pub_name("point_cloudino"),
  pc2_pub_name("point_cloud2"),
  frame_id("gim30"),
  portname("/dev/ttyUSB0"),
  fd(-1),
  old_angle(0.0),
  new_angle(1.0),
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
  // Node Control /////////////////////////////////////////////////////
  n.param("gim30/pc_pub_name", pc_pub_name, pc_pub_name);
  n.param("gim30/pc2_pub_name", pc2_pub_name, pc2_pub_name);
  n.param("gim30/frame_id", frame_id, frame_id);

  pc_data.header.frame_id = frame_id;
  pc_data.channels.resize(1);
  pc_data.channels[0].name = "intensities";
  pc_data.points.resize(step);
  pc_data.channels[0].values.resize(step);

  pc_pub = n.advertise<sensor_msgs::PointCloud>(pc_pub_name, 1);
  pc2_pub = n.advertise<sensor_msgs::PointCloud2>(pc2_pub_name, 1);
  /////////////////////////////////////////////////////////////////////

  // Serial ////////////////////////////////////////////
  n.param("gim30/portname", portname, portname);
  data.resize(8,'\0');
  OpenGim30();
  StartGim30();
  /////////////////////////////////////////////////////

  // URG ///////////////////////////////////////////////////////////////////////
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
  //////////////////////////////////////////////////////////////////////////////

}


Gim30::~Gim30()
{
  // Serial //////////////////
  StopGim30();
  CloseGim30();
  ///////////////////////////

  // URG ////////////////////
  CloseURG();
  ///////////////////////////
}



int Gim30::GetDatas()
{ 
  if(!GetOldAngle())
    return 1;
  if(!GetURGData())
    return 2;
  if(!GetNewAngle())
    return 3;

  return 0;
}

void Gim30::CalculateDatas()
{
  double tmp;
  double alpha;
  double beta;
  
  //  GetOldAngle();
  ROS_INFO("start megering");
  urg_start_measurement(&urg, URG_DISTANCE_INTENSITY, 1, 0);
  if(urg_get_distance_intensity(&urg, ranges_raw, intensities, &timestamp) <= 0){
    ROS_WARN("Disable to get URG data on Gim30.");
  }
  GetAngle();
  ROS_INFO("stop megering");

  // if(!GetDatas()){
  for(int i=0; i<step; i++){
    ranges[i] = (double)ranges_raw[i] / 1000.0;
    if(ranges[i] > range_max || ranges[i] < range_min)
      ranges[i] = 0;
    tmp = new_angle-old_angle;
    if(tmp > 180)
      tmp = 360 - tmp;
    tmp = (deg_max-deg_min)*tmp*(double)i/(360.0*step) + old_angle + (180.0+deg_min)/360.0*tmp;
    alpha = atan(0.57734*sin(tmp*M_PI/180.0));
    beta = -atan(0.57734*cos(tmp*M_PI/180.0)*cos(alpha));
    tmp = (rad_max - rad_min)*(double)i/step + rad_min;    
    pc_data.points[i].x = ranges[i]*cos(tmp)*cos(beta) + 0.039*sin(beta);
    pc_data.points[i].y = ranges[i]*cos(tmp)*sin(alpha)*sin(beta) + ranges[i]*sin(tmp)*cos(alpha) - 0.039*sin(alpha)*cos(beta);
    pc_data.points[i].z = -ranges[i]*cos(tmp)*cos(alpha)*sin(beta) + ranges[i]*sin(tmp)*sin(alpha) + 0.039*cos(alpha)*cos(beta);
    if(publish_intensity)
      pc_data.channels[0].values[i] = intensities[i];
  } 
  
  ROS_INFO("culculate fhinish");

  pc_data.header.stamp = ros::Time::now();

  ROS_INFO("convert start");

  //sensor_msgs::convertPointCloudToPointCloud2(pc_data, pc2_data);

  ROS_INFO("end of gim30");


  // pc_pub.publish(pc_data);
  // pc2_pub.publish(pc2_data);
  // }
  // else{
  //   ROS_WARN("Anable to get Gim30 datas.");
  // }
}

void Gim30::OpenGim30()
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

void Gim30::CloseGim30()
{
  if(fd < 0){
    ROS_WARN("Gim30 Port is already closed.");
    ROS_BREAK();
    
  }
  close(fd);
  fd = -1;
  ROS_INFO("Succeed to close Gim30.");
}

void Gim30::StartGim30()
{
  if(write(fd, "START\r\n", sizeof("START\r\n")) != sizeof("START\r\n")){
    ROS_WARN("Gim30 write error. Cannot start Gim30.");
    ROS_BREAK();
    
  }
  ROS_INFO("Succeed to start Gim30.");
}

void Gim30::StopGim30()
{
  if(write(fd, "STOP\r\n", sizeof("STOP\r\n")) != sizeof("STOP\r\n")){
    ROS_WARN("Gim30 write error. Cannot stop Gim30.");
    ROS_BREAK();
    
  }
  ROS_INFO("Succeed to stop Gim30.");
}


bool Gim30::GetAngle()
{
  int tmp;
  old_angle = new_angle;
  for(int i=0; i<8; i+=tmp){
    tmp = read(fd, &data[i], 8);
    if(tmp < 0)
      return false;
  }
  new_angle = -(atoi(&data[1]) / 100.0);
  //cout << old_angle << endl;
  return true;
}

bool Gim30::GetOldAngle()
{
  int tmp;
  for(int i=0; i<8; i+=tmp){
    tmp = read(fd, &data[i], 8);
    if(tmp < 0)
      return false;
  }
  old_angle = -(atoi(&data[1]) / 100.0);
  //cout << old_angle << endl;
  return true;
}

bool Gim30::GetNewAngle()
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

void Gim30::OpenURG()
{
  // if(urg_open(&urg,URG_SERIAL,serial_port.c_str(),(long)serial_baud) < 0){  
  if(urg_open(&urg,URG_ETHERNET,ip_address.c_str(),(long)ip_port) < 0){
    ROS_WARN("URG on Gim30 open error.");
    ROS_BREAK();
      
  }
  // }

  urg_set_scanning_parameter(&urg, urg_deg2step(&urg,deg_min), urg_deg2step(&urg,deg_max), skip);

  rad_min = deg_min*M_PI/180.0;
  rad_max = deg_max*M_PI/180.0;
  step = (deg_max - deg_min)*4.0;

  ranges_raw = new long[step+1];
  ranges = new double[step+1];
  intensities = new unsigned short[step+1];
}

void Gim30::CloseURG()
{
  delete ranges_raw;
  delete ranges;
  delete intensities;
  urg_close(&urg);
}

bool Gim30::GetURGData()
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
