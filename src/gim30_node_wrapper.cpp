#include "gim30/gim30_node.hpp"

Gim30::Gim30(ros::NodeHandle &n) :
  portname("/dev/ttyUSB0"),
  fd(-1),
  old_angle(-720.0),
  new_angle(-720.0),
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

  try{
    OpenURG();
  }
  catch(runtime_error& e){
    CloseURG();
    throw;
  }
  //////////////////////////////////////////////////////////////////////////////

  // Serial ////////////////////////////////////////////
  n.param("gim30/portname", portname, portname);
  data.resize(8,'\0');
  
  try{
    OpenGim30();
    StartGim30();
  }
  catch(runtime_error& e){
    StopGim30();
    CloseGim30();
    CloseURG();
    throw;
  }
  /////////////////////////////////////////////////////
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



int Gim30::GetDatas(ros::Time& t)
{ 

  if(GetURGData(t) == false)
	return false;
  if(GetAngle() == false)
	return false;


  return true;
}

void Gim30::OpenGim30()
{
  if(fd > 0){
    ROS_WARN("Gim30 is already open");
  }
  else if((fd = open(portname.c_str(), O_RDWR) ) < 0){
    ROS_WARN("Failed to open the port : Gime30.");
    throw runtime_error("Failed to Open the port : Gim30.");    
  } 

  //Get old termios for Gim30
  if(ioctl(fd, TCGETS, &oldtio) < 0){
    ROS_WARN("Gim30 ioctl TCGET error.");
    throw runtime_error("Gim30 ioctl TCGET error.");
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
    ROS_WARN("Gim30 ioctl TCSETS error.");
    throw runtime_error("Gim30 ioctl TCSETS error.");
  }

  ROS_INFO("Gim30 is opened.");
}

void Gim30::CloseGim30()
{
  if(fd < 0){
    ROS_WARN("Gim30 Port is already closed.");
    throw runtime_error("Gim30 Port is already closed.");
  }
  close(fd);
  fd = -1;
  ROS_INFO("Gim30 closed.");
}

void Gim30::StartGim30()
{
  if(write(fd, "START\r\n", sizeof("START\r\n")) != sizeof("START\r\n")){
    ROS_WARN("Gim30 write error. Cannot start Gim30.");
    throw runtime_error("Gim30 write error to Start.");
  }

  ROS_INFO("Start the Gim30.");
}

void Gim30::StopGim30()
{
  if(write(fd, "STOP\r\n", sizeof("STOP\r\n")) != sizeof("STOP\r\n")){
    ROS_WARN("Gim30 write error. Cannot stop Gim30.");
    throw runtime_error("Gim30 write error to Stop.");
  }
  ROS_INFO("Stop the Gim30.");
}


bool Gim30::GetAngle()
{
  //cout << old_angle << ", " << new_angle << endl;
  int tmp;
  // 2回連続でangleが取れない時は失敗
  if(old_angle == new_angle){
	old_angle = new_angle;
  	ROS_WARN("failed to get angle");
  	for(int i=0; i<8; i+=tmp){
  	  tmp = read(fd, &data[i], 8);
  	  if(tmp < 0)
  		return false;
  	}
  	new_angle = -(atoi(&data[1]) / 100.0);
  	return false;
  }  
  else{
	old_angle = new_angle;
	for(int i=0; i<8; i+=tmp){
	  tmp = read(fd, &data[i], 8);
	  if(tmp < 0)
		return false;
	}
	//cout << "succeed to get angle" << endl;
	new_angle = -(atoi(&data[1]) / 100.0);
  }
  //  cout << new_angle << "\t" << old_angle << "\t" << new_angle - old_angle << endl;
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
  rad_min = deg_min*M_PI/180.0;
  rad_max = deg_max*M_PI/180.0;
  step = (deg_max - deg_min)*4.0 + 1;

  ranges = new double*[2];
  intensities = new double*[2];

  for(int j = 0; j<2; j++){
    for(int i = 0; i<step; i++){
      ranges[j] = new double[step];
      intensities[j] = new double[step];
      ranges[j][i] = 0;
      intensities[j][i] = 0;
    }
  }

  if(urg_open(&urg,URG_SERIAL,serial_port.c_str(),(long)serial_baud) < 0){  
    if(urg_open(&urg,URG_ETHERNET,ip_address.c_str(),(long)ip_port) < 0){
      ROS_WARN("URG on Gim30 open error.");
      throw runtime_error("URG on Gim30 open error.");
    }
  }

  urg_set_scanning_parameter(&urg, urg_deg2step(&urg,deg_min), urg_deg2step(&urg,deg_max), skip);

  if(publish_intensity)
    urg_start_measurement(&urg, URG_DISTANCE_INTENSITY, URG_SCAN_INFINITY, skip);
  else
    urg_start_measurement(&urg, URG_DISTANCE, URG_SCAN_INFINITY, skip);

}

void Gim30::CloseURG()
{
  for(int i=0; i<2; i++){
    delete ranges[i];
    delete intensities[i];
  }
  delete[] ranges;
  delete[] intensities;

  urg_close(&urg);
} 

bool Gim30::GetURGData(ros::Time& t)
{
  if(publish_intensity){
    for(int i=0; i<step; i++){
      ranges[0][i] = ranges[1][i];
      intensities[0][i] = intensities[1][i];
    }
    t = ros::Time::now();
    if(urg_get_distance_intensity(&urg, ranges[1], intensities[1], &timestamp) <= 0){
      ROS_WARN("Disable to get URG data on Gim30.");
      for(int i=0; i<step; i++){
		ranges[1][i] = 0;
		intensities[1][i] = 0;
      } 
      return false;
    }
  }
  else{
    for(int i=0; i<step; i++)
      ranges[0][i] = ranges[1][i];
    t = ros::Time::now();
    if(urg_get_distance(&urg, ranges[1], &timestamp) <= 0){
      ROS_WARN("Disable to get URG data on Gim30.");
      for(int i=0; i<step; i++)
		ranges[1][i] = 0;
      return false;
    }
  }
  return true;
}
