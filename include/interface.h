#ifndef __GIM30_3D__
#define __GIM30_3D__

#include <termios.h>
#include <urg_sensor.h>
#include <urg_utils.h>

#include <sensor_msgs/LaserScan.h>

#define LRF_MAX_RANGE 30000.0   //Max range of LRF [mm]
#define LRF_START_DEG -90.0
#define LRF_END_DEG 90.0
#define LRF_SKIP_STEP 0

namespace cirkit{
  class Gim30Interface
  {
  private :
    // Private Variable-----------------------------
    std::string GIMportname;                  //Gim30 port name
    std::string GIMdata;                      //Row Angle data from Gim30
    int GIMfd;                                //Gim30 file discriptor 
    struct termios GIMoldtio;                 //Gim30 old termios
    struct termios GIMnewtio;                 //Gim30 new termios

    std::string LRFportname;                  //LRF on Gim30 portname
    urg_t urg;                                //LRF on Gim30 urg_t struct
    long *ranges;
    long timestamp;
    unsigned short *intensities;


    // End Private Variable-------------------------

  public :

    // Prototype-------------------------
    // Constructor
    Gim30Interface( std::string new_GIMportname, std::string new_LRFportname);

    // Destructor
    ~Gim30Interface();

    //Open Serial Port
    virtual int OpenSerialPort();

    //Set Serial Port
    virtual int SetSerialPort();

    // Close Serial Port
    virtual int CloseSerialPort();

    // Start Gim30
    virtual int StartGim30();

    // Stop Gim30
    virtual int StopGim30();

    // Get LRF data
    virtual bool Get3DData(const bool intensity);

    // End Peototype----------------------

    // Variable----------------------------------
    sensor_msgs::LaserScan LRFdata;
    float GIMangle_old;
    float GIMangle_new;
    int get_steps;
    int start_index;
    int end_index;

    // End Variable-------------------------------

  };
}


#endif 
