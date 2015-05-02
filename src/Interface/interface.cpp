#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <stdexcept>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "interface.h"

#define GIM30_DATASIZE 8

using namespace cirkit;
using namespace std;

Gim30Interface::Gim30Interface(
			       std::string new_GIMportname,
			       std::string new_LRFportname
			       )
{

  // Gim30 serial const------------------------------
  GIMportname = new_GIMportname;
  GIMdata.resize(GIM30_DATASIZE, '\0');
  GIMfd = -1;
  GIMangle_old = 0;
  GIMangle_new = 1;
  // -------------------------------------------------
  // LRF serial const---------------------------------
  LRFportname = new_LRFportname;
  timestamp = 0;
  LRFdata.header.frame_id = "gim30_lrf";
  LRFdata.header.stamp.sec = timestamp;
  //--------------------------------------------------


  // Open Gim30 & LRF port----------------------------
  if(!OpenSerialPort())
    ROS_INFO("Gim30 Open Succeed !!");
  else{
    ROS_FATAL("Gim Open Error");
    ROS_BREAK();
  }
  // --------------------------------------------------


  // Start Gim30---------------------------------------
  if(!StartGim30())
    ROS_INFO("Start Gim30.");
  else{
    ROS_FATAL("StartGim30() Failed.");
    ROS_BREAK();
  }
  // --------------------------------------------------


}

Gim30Interface::~Gim30Interface()
{
  //Stop Gim30
  if(!StopGim30())
    ROS_INFO("Stop Gim30.");
  else
    ROS_FATAL("StopGim30() Failed.");

  if(!CloseSerialPort())
    ROS_INFO("Gim30 Close Succeed");
  else 
    ROS_FATAL("Gim30 Close Error");
}

int Gim30Interface::OpenSerialPort()
{
  try{ SetSerialPort(); }
  catch(exception &e){
    cerr << e.what() << endl;
    return 1;
  }

  return 0;
}

int Gim30Interface::SetSerialPort()
{
  // Gim30 open----------------------------------------------------------------------------
  //Serial open
  if( GIMfd > 0)
    throw logic_error("Gim30 is already open");
  if( (GIMfd = open(GIMportname.c_str(), O_RDWR) ) < 0)
    throw logic_error ("Failed to open the port : Gime30");
  else 
    //Get old termios for Gim30
    if(ioctl(GIMfd, TCGETS, &GIMoldtio) < 0)
      throw logic_error("Gim30 ioctl TCGET error");

  //Copy old to new 
  GIMnewtio = GIMoldtio;  

  //New serial settings of Gim30
  GIMnewtio.c_cflag = (B115200 | CS8 | CREAD | CLOCAL);
  GIMnewtio.c_iflag = (IGNPAR | ICRNL);
  GIMnewtio.c_oflag = 0;
  GIMnewtio.c_lflag = ~ICANON;

  //Set new serial settings of Gim30
  if(ioctl(GIMfd, TCSETS, &GIMnewtio) < 0)
    throw logic_error("Gim30 ioctl TCSETS error");
  // End Gim30 open------------------------------------------------------------------------

 
  // LRF open------------------------------------------------------------------------------
  long min_range;
  long max_range;

  if(urg_open(&urg,URG_ETHERNET,LRFportname.c_str(),10940) < 0)
    throw logic_error("LRF on Gim30 urg_open error");
 
  urg_set_scanning_parameter(&urg, urg_deg2step(&urg, LRF_START_DEG), urg_deg2step(&urg, LRF_END_DEG), LRF_SKIP_STEP);

  urg_distance_min_max(&urg, &min_range, &max_range);
  LRFdata.range_min = (float)min_range/1000.0;
  LRFdata.range_max = (float)max_range/1000.0;	
  LRFdata.angle_min = LRF_START_DEG*M_PI/180.0;
  LRFdata.angle_max = LRF_END_DEG*M_PI/180.0;
  LRFdata.angle_increment = urg_step2rad(&urg, (LRF_SKIP_STEP+1));
  get_steps = (LRF_END_DEG - LRF_START_DEG)*4.0;
  start_index = urg_deg2index(&urg, LRF_START_DEG);
  end_index = urg_deg2index(&urg, LRF_END_DEG);

  ROS_INFO("%f", (LRF_END_DEG - LRF_START_DEG));

  LRFdata.ranges.resize(get_steps+1);
  LRFdata.intensities.resize(get_steps+1);
  ranges = new long[LRFdata.ranges.size()];
  intensities = new unsigned short[LRFdata.intensities.size()];



  // End LRF open--------------------------------------------------------------------------

  return 0;

}

int Gim30Interface::CloseSerialPort()
{
  //Close Gim30-------------------------------------------------
  if(GIMfd < 0){
    cerr << "Gim30 Port is already closed." << endl;
    return 1;
  } else {
    close(GIMfd);
    GIMfd = -1;
  }
  //End Close Gim30---------------------------------------------

  //Close LRF---------------------------------------------------
  delete ranges;
  delete intensities;
  urg_close(&urg);
  //End Close LRF-----------------------------------------------

  return 0;
}

int Gim30Interface::StartGim30()
{
  if(write(GIMfd, "START\r\n", sizeof("START\r\n")) != sizeof("START\r\n")){
    cerr << "write error" << endl;
    return 1;
  }
  else 
    return 0;
}

int Gim30Interface::StopGim30()
{
  if(write(GIMfd, "STOP\r\n", sizeof("STOP\r\n")) != sizeof("STOP\r\n")){
    cerr << "write error" << endl;
    return 1;
  }
  else
    return 0;
}

bool Gim30Interface::Get3DData(const bool intensity)
{
  int tmp;
  char buf[8];

  switch(intensity){
  case true :

    for(int i=0; i<GIM30_DATASIZE; i+=tmp)
      tmp = read(GIMfd, &GIMdata[i], GIM30_DATASIZE);
    GIMangle_old = -(std::atoi(&GIMdata[1]) / 100.0); 

    urg_start_measurement(&urg, URG_DISTANCE_INTENSITY, 1, 0);
    if(urg_get_distance_intensity(&urg, ranges, intensities, &timestamp) <= 0){
      cerr << "urg get distance & intensity error" << endl;
      LRFdata.header.stamp.sec = timestamp; 
      for(int i=0; i<LRFdata.ranges.size(); i++){
	LRFdata.intensities[i] = 0;	
	LRFdata.ranges[i] = 0;
      }
      for(int i=0; i<GIM30_DATASIZE; i+=tmp)
	tmp = read(GIMfd, &GIMdata[i], GIM30_DATASIZE);
      GIMangle_new = -(std::atoi(&GIMdata[1]) / 100.0);

      return true;
    }

    for(int i=0; i<GIM30_DATASIZE; i+=tmp)
      tmp = read(GIMfd, &GIMdata[i], GIM30_DATASIZE);
    GIMangle_new = -(std::atoi(&GIMdata[1]) / 100.0);

    LRFdata.header.stamp.sec = timestamp; 
    for(int i=0; i<LRFdata.ranges.size(); i++){
      LRFdata.ranges[i] = (double)ranges[i];
      LRFdata.intensities[i] = (double)intensities[i];
      if(LRFdata.ranges[i] > LRF_MAX_RANGE)
	LRFdata.ranges[i] = 0;
      if(LRFdata.ranges[i] < 100)
	LRFdata.ranges[i] = 0;
      LRFdata.ranges[i] /= 1000.0;
    }
    return true;
    break;

  default :
    for(int i=0; i<GIM30_DATASIZE; i+=tmp)
      tmp = read(GIMfd, &GIMdata[i], GIM30_DATASIZE);
    GIMangle_old = -(std::atoi(&GIMdata[1]) / 100.0);

    urg_start_measurement(&urg, URG_DISTANCE, 1, 0);  
    if(urg_get_distance(&urg, ranges, &timestamp) <= 0){
      cerr << "urg get distance error" << endl;
      LRFdata.header.stamp.sec = timestamp; 
      for(int i=0; i<LRFdata.ranges.size(); i++)
	LRFdata.ranges[i] = 0;

      for(int i=0; i<GIM30_DATASIZE; i+=tmp)
	tmp = read(GIMfd, &GIMdata[i], GIM30_DATASIZE);
      GIMangle_new = -(std::atoi(&GIMdata[1]) / 100.0); 

      return false;
    }

    for(int i=0; i<GIM30_DATASIZE; i+=tmp)
      tmp = read(GIMfd, &GIMdata[i], GIM30_DATASIZE);
    GIMangle_new = -(std::atoi(&GIMdata[1]) / 100.0); 

    LRFdata.header.stamp.sec = timestamp; 
    for(int i=0; i<LRFdata.ranges.size(); i++){
      LRFdata.ranges[i] = (double)ranges[i];
      if(LRFdata.ranges[i] > LRF_MAX_RANGE)
	LRFdata.ranges[i] = 0;
      if(LRFdata.ranges[i] < 100)
	LRFdata.ranges[i] = 0;
      LRFdata.ranges[i] /= 1000.0;
    }

    return false;
    break;
  }
}
