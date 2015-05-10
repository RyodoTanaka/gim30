#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <iostream>

#include "gim30/gim30_node.hpp"

using namespace std;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "gim30");
  ros::NodeHandle n;
  ros::NodeHandle param_n;
  bool err_flg = false;

  try{
    Gim30 gim30(param_n);

    // Publish args ///////////////////////////////////
    sensor_msgs::PointCloud pc_data;
    sensor_msgs::PointCloud2 pc2_data;
    ros::Publisher pc_pub;
    ros::Publisher pc2_pub;
    ///////////////////////////////////////////////////

    // parameter server arguments /////////////////////
    string pc_pub_name;
    string pc2_pub_name;
    string frame_id;
    //////////////////////////////////////////////////

    // Node Control /////////////////////////////////////////////////////
    n.param("gim30/pc_pub_name", pc_pub_name, pc_pub_name);
    n.param("gim30/pc2_pub_name", pc2_pub_name, pc2_pub_name);
    n.param("gim30/frame_id", frame_id, frame_id);

    pc_data.header.frame_id = frame_id;
    pc_data.channels.resize(1);
    pc_data.channels[0].name = "intensities[0]";
    pc_data.points.resize(gim30.step);
    pc_data.channels[0].values.resize(gim30.step);

    pc_pub = n.advertise<sensor_msgs::PointCloud>(pc_pub_name, 1);
    pc2_pub = n.advertise<sensor_msgs::PointCloud2>(pc2_pub_name, 1);
    /////////////////////////////////////////////////////////////////////

    double tmp;
    double alpha;
    double beta;

    while(ros::ok()){
      ROS_INFO("Start the loop.");

      //gim30.GetDatas();

      if(!gim30.GetDatas()){
      	ROS_INFO("Get the datas & start calculate.");
      	for(int i=0; i<gim30.step; i++){
      	  if(gim30.ranges[0][i] > gim30.range_max || gim30.ranges[0][i] < gim30.range_min){
      	    pc_data.points[i].x = 0;
      	    pc_data.points[i].y = 0;
      	    pc_data.points[i].z = 0;
	    if(gim30.publish_intensity)
	      pc_data.channels[0].values[0] = 0;
      	    continue;
      	  }
      	  tmp = gim30.new_angle-gim30.old_angle;
      	  if(tmp > 180)
      	    tmp = tmp - 360;
      	  tmp = (gim30.deg_max - gim30.deg_min)*tmp*(double)i/(360.0*gim30.step) + gim30.old_angle + (180.0 + gim30.deg_min)/360.0*tmp;
      	  alpha = atan(0.57734*sin(tmp*M_PI/180.0));
      	  beta = -atan(0.57734*cos(tmp*M_PI/180.0)*cos(alpha));
      	  tmp = (gim30.rad_max - gim30.rad_min)*(double)i/gim30.step + gim30.rad_min;    
      	  pc_data.points[i].x = gim30.ranges[0][i]*cos(tmp)*cos(beta) + 0.039*sin(beta);
      	  pc_data.points[i].y = gim30.ranges[0][i]*cos(tmp)*sin(alpha)*sin(beta) + gim30.ranges[0][i]*sin(tmp)*cos(alpha) - 0.039*sin(alpha)*cos(beta);
      	  pc_data.points[i].z = -gim30.ranges[0][i]*cos(tmp)*cos(alpha)*sin(beta) + gim30.ranges[0][i]*sin(tmp)*sin(alpha) + 0.039*cos(alpha)*cos(beta);
      	  if(gim30.publish_intensity)
      	    pc_data.channels[0].values[i] = gim30.intensities[0][i];
      	} 
      	ROS_INFO("Finish calculate");
      
      	pc_data.header.stamp = ros::Time::now();

      	ROS_INFO("convert start.");

      	sensor_msgs::convertPointCloudToPointCloud2(pc_data, pc2_data);

      	ROS_INFO("convert end.");

      	pc_pub.publish(pc_data);
      	pc2_pub.publish(pc2_data);
      }
      else{
      	ROS_WARN("Anable to get Gim30 datas.");
      }
      ROS_INFO("Finish the loop.");
    }
  }
  catch(runtime_error &e){
    cerr << e.what() << endl;
    return 1;
  }

  return 0;
}
