#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h> 
#include <sensor_msgs/point_cloud_conversion.h>
#include <cmath>

#include "interface.h"

std::string Gim30portname;
std::string LRFportname;

cirkit::Gim30Interface *Gim30;

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "gim30_3d");
  ROS_INFO("Gim30 3D for ROS");

  ros::NodeHandle n;
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("/gim30_3d", 50);

  n.param<std::string>("gim30/Gim30portname",Gim30portname,"/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0");
  n.param<std::string>("gim30/LRFportname", LRFportname, "192.168.0.10");

  Gim30 = new cirkit::Gim30Interface(Gim30portname, LRFportname);

  sensor_msgs::PointCloud GIM3Ddata;
  // Gim30 3D points const----------------------------
  // GIM3Ddata.header.stamp = ros::Time::now();
  GIM3Ddata.header.frame_id = "/base_link";
  // Add intensity data channel
  GIM3Ddata.channels.resize(1);
  GIM3Ddata.channels[0].name = "intensities";

  //  GIM3Ddata.points.resize(1081);

  GIM3Ddata.points.resize(Gim30->LRFdata.ranges.size());
  GIM3Ddata.channels[0].values.resize(Gim30->LRFdata.intensities.size());

  sensor_msgs::PointCloud2 GIM3Ddata_pc2;
  ros::Publisher cloud_pub_pc2 = n.advertise<sensor_msgs::PointCloud2>("/gim30_3d_pc2", 50);

  double alpha, beta;
  double tmp;
  //--------------------------------------------------

  ros::Rate r(40.0);

  while(n.ok()){

    switch(Gim30->Get3DData(true)){
    case true :

      for(double i=0; i<=Gim30->get_steps; i++){
	if(Gim30->LRFdata.ranges[i] == 0){
	  GIM3Ddata.points[i].x = 0;
	  GIM3Ddata.points[i].y = 0;
	  GIM3Ddata.points[i].z = 0;
	}
	else{
	  tmp = Gim30->GIMangle_new-Gim30->GIMangle_old;
	  if(tmp > 180)
	    tmp = 360 - tmp;
	  tmp = (LRF_END_DEG-LRF_START_DEG)*tmp*i/(360.0*Gim30->get_steps) + Gim30->GIMangle_old + (180.0+LRF_START_DEG)/360.0*tmp;
	  alpha = atan(0.57734*sin(tmp*M_PI/180.0));
	  beta = -atan(0.57734*cos(tmp*M_PI/180.0)*cos(alpha));
	  tmp = (Gim30->LRFdata.angle_max - Gim30->LRFdata.angle_min)*i/Gim30->get_steps + Gim30->LRFdata.angle_min;    
	  GIM3Ddata.points[i].x = Gim30->LRFdata.ranges[i]*cos(tmp)*cos(beta) + 0.039*sin(beta);
	  GIM3Ddata.points[i].y = Gim30->LRFdata.ranges[i]*cos(tmp)*sin(alpha)*sin(beta) + Gim30->LRFdata.ranges[i]*sin(tmp)*cos(alpha) - 0.039*sin(alpha)*cos(beta);
	  GIM3Ddata.points[i].z = -Gim30->LRFdata.ranges[i]*cos(tmp)*cos(alpha)*sin(beta) + Gim30->LRFdata.ranges[i]*sin(tmp)*sin(alpha) + 0.039*cos(alpha)*cos(beta);

	  GIM3Ddata.channels[0].values[i] = Gim30->LRFdata.intensities[i];
	}
      } 

      break;
    default :
      for(double i=0; i<Gim30->get_steps; i++){
	if(Gim30->LRFdata.ranges[i] == 0){
	  GIM3Ddata.points[i].x = 0;
	  GIM3Ddata.points[i].y = 0;
	  GIM3Ddata.points[i].z = 0;
	  GIM3Ddata.channels[0].values[i] = 0;
	}
	else {
	  tmp = Gim30->GIMangle_new-Gim30->GIMangle_old;
	  if(tmp > 180)
	    tmp = 360 - tmp;
	  tmp = (LRF_END_DEG-LRF_START_DEG)*tmp*i/(360.0*Gim30->get_steps) + Gim30->GIMangle_old + (180.0+LRF_START_DEG)/360.0*tmp;
	  alpha = atan(0.57734*sin(tmp*M_PI/180.0));
	  beta = -atan(0.57734*cos(tmp*M_PI/180.0)*cos(alpha));
	  tmp = (Gim30->LRFdata.angle_max - Gim30->LRFdata.angle_min)*i/Gim30->get_steps + Gim30->LRFdata.angle_min;    
	  GIM3Ddata.points[i].x = Gim30->LRFdata.ranges[i]*cos(tmp)*cos(beta) + 0.039*sin(beta);
	  GIM3Ddata.points[i].y = Gim30->LRFdata.ranges[i]*cos(tmp)*sin(alpha)*sin(beta) + Gim30->LRFdata.ranges[i]*sin(tmp)*cos(alpha) - 0.039*sin(alpha)*cos(beta);
	  GIM3Ddata.points[i].z = -Gim30->LRFdata.ranges[i]*cos(tmp)*cos(alpha)*sin(beta) + Gim30->LRFdata.ranges[i]*sin(tmp)*sin(alpha) + 0.039*cos(alpha)*cos(beta);	}
      }

      break;
    }

    GIM3Ddata.header.stamp = ros::Time::now();


    sensor_msgs::convertPointCloudToPointCloud2(GIM3Ddata, GIM3Ddata_pc2);

    cloud_pub.publish(GIM3Ddata);
    cloud_pub_pc2.publish(GIM3Ddata_pc2);
    r.sleep();
  }
  
  delete Gim30;

}
