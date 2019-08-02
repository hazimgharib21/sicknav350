/*
 * sicknav350_map.cpp
 *
 *  Created on: Aug 1, 2019 
 *  Author: hazimgharib
 *
 * Based on the sicklms.cpp and sickld.cpp from the sicktoolbox_wrapper ROS package
 * and the sample code from the sicktoolbox manual.
 *
 * Released under BSD license.
 */


#include <iostream>
#include <sicktoolbox/SickNAV350.hh>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

namespace OperatingModes
{
  enum OperatingMode
  {
    POWERDOWN = 0,
    STANDBY = 1,
    MAPPING = 2,
    LANDMARK = 3,
    NAVIGATION = 4,
  };
}
typedef OperatingModes::OperatingMode OperatingMode;

namespace ReflectorTypes
{
  enum ReflectorType
  {
    FLAT = 1,
    CYLINDRICAL = 2,
  };
}
typedef ReflectorTypes::ReflectorType ReflectorType;

namespace Users
{
  enum User
  {
    OPERATOR = 2,
    AUTHORIZED_CLIENT = 3,
  };
}
typedef Users::User User;

using namespace std;
using namespace SickToolbox;

// odometry call back from robot
void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
  vx = msg->twist.twist.linear.x;
  vy = msg->twist.twist.linear.y;
  vth = msg->twist.twist.angular.z;
}

int main(int argc, char *argv[]){

  ros::init(argc, argv, "sicknav350_map");
  ros::NodeHandle nh;
  ros::NodeHandle nh_ns("~");


  int port;
  bool inverted;
  std::string ipaddress;
  std::string odometry;

  nh_ns.param("port", port, DEFAULT_SICK_TCP_PORT);
  nh_ns.param("ipaddress", ipaddress, (std::string)DEFAULT_SICK_IP_ADDRESS);
  nh_ns.param("inverted", inverted, false);
  
  // Robot odometry sub
  ros::Subscriber sub = nh.subscribe("odom", 10, OdometryCallback);

  // Define buffers for return values of scan data
  double range_values[SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};
  unsigned int intensity_values[SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};

  SickNav350 sick_nav350(ipaddress.c_str(), port);

  try{

    // Initialize the device
    sick_nav350.Initialize();
    sick_nav350.SetAccessMode((int)Users::AUTHORIZED_CLIENT);

    // Device setup
    sick_nav350.SetOperatingMode((int)OperatingModes::STANDBY);
    sick_nav350.SetCurrentLayer(1);
    sick_nav350.SetPoseDataFormat(1, 1);
    sick_nav350.SetScanDataFormat(1, 1);
    sick_nav350.SetLandmarkDataFormat(0,1,2);
    sick_nav350.SetReflectorWindow(500, 500, 500, 70000);
    sick_nav350.SetActionRadius(400, 70000);
    sick_nav350.SetReflectorSize(80);
    sick_nav350.SetReflectorType((int)ReflectorTypes::FLAT);
    sick_nav350.SetReflectorThreshold(15);
    sick_nav350.SetLandmarkMatching(0);
    sick_nav350.SetMappingConfiguration(5, 1, 0, 0, 0);
    sick_nav350.SetOperatingMode((int)OperatingModes::MAPPING);

    while(ros::ok()){

      //sick_nav350.DoMapping();
      if(sick_nav350.ReflectorData_.error == 99){

        if(sick_nav350.ReflectorData_.num_reflector == 0){
          sick_nav350.GetLayout();
        }else{
          std::cout << "Num Reflector : " << sick_nav350.ReflectorData_.num_reflector << std::endl;
          int temp[sick_nav350.ReflectorData_.num_reflector][7];
          for(int i = 0; i < sick_nav350.ReflectorData_.num_reflector; i++){
            std::cout << "X : " << sick_nav350.ReflectorData_.x[i] << std::endl;
            std::cout << "Y : " << sick_nav350.ReflectorData_.y[i] << std::endl;
            std::cout << "ID : " << sick_nav350.ReflectorData_.GlobalID[i] << std::endl;
            std::cout << std::endl;
            temp[i][0] = sick_nav350.ReflectorData_.x[i];
            temp[i][1] = sick_nav350.ReflectorData_.y[i];
            temp[i][2] = sick_nav350.ReflectorData_.type[i];
            temp[i][3] = sick_nav350.ReflectorData_.subtype[i];
            temp[i][4] = sick_nav350.ReflectorData_.size[i];
            temp[i][5] = 1;
            temp[i][6] = sick_nav350.ReflectorData_.LocalID[i];
          }

          sick_nav350.AddLandmark(sick_nav350.ReflectorData_.num_reflector, temp);
        }
      }
      sick_nav350.GetLayout();


      
    
    }

    sick_nav350.Uninitialize();

  
  }catch(...){

    ROS_ERROR("Error");
    return -1;

  }

}
