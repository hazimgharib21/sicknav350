/*
 * sicknav350_node.cpp
 *
 *  Created on: Aug 5, 2015
 *  Forked From : punithm/sicknav350
 *      Author: hazimgharib
 *
 * Based on the sicklms.cpp and sickld.cpp from the sicktoolbox_wrapper ROS package
 * and the sample code from the sicktoolbox manual.
 *
 * Released under BSD license.
 */

#include <iostream>
#include <sicktoolbox/SickNAV350.hh>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <deque>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Point.h"
#include "sicknav350/PointArray.h"
#include <vector>

#define DEG2RAD M_PI/180.0

struct Point
{
  float x;
  float y;
};

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

void publish_scan(ros::Publisher *pub, double *range_values,
                  uint32_t n_range_values, unsigned int *intensity_values,
                  uint32_t n_intensity_values, ros::Time start,
                  double scan_time, bool inverted, float angle_min,
                  float angle_max, std::string frame_id,
                  unsigned int sector_start_timestamp)
{
  static int scan_count = 0;
  sensor_msgs::LaserScan scan_msg;
  scan_msg.header.frame_id = frame_id;
  scan_count++;
  if (inverted)
  {
    scan_msg.angle_min = angle_max * DEG2RAD;
    scan_msg.angle_max = angle_min * DEG2RAD;
  }
  else
  {
    scan_msg.angle_min = angle_min * DEG2RAD;
    scan_msg.angle_max = angle_max * DEG2RAD;
  }
  scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(n_range_values - 1);
  scan_msg.scan_time = 0.125;//scan_time 125ms;
  scan_msg.time_increment = scan_msg.scan_time / n_range_values;
  scan_msg.range_min = 0.1;
  scan_msg.range_max = 250.;
  scan_msg.ranges.resize(n_range_values);
  scan_msg.header.stamp = start;
  for (size_t i = 0; i < n_range_values; i++)
  {
    scan_msg.ranges[i] = (float)range_values[i] / 1000;
  }
  scan_msg.intensities.resize(n_intensity_values);
  for (size_t i = 0; i < n_intensity_values; i++)
  {
    scan_msg.intensities[i] = 0;//(float)intensity_values[i];
  }
  pub->publish(scan_msg);

}

// odometry call back from the robot
void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  vx = msg->twist.twist.linear.x;
  vy = msg->twist.twist.linear.y;
  vth = msg->twist.twist.angular.z;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "sicknav350");
  int port;
  std::string ipaddress;
  std::string frame_id, fixed_frame_id;
  std::string odometry;
  std::string scan;
  bool inverted;
  int sick_motor_speed = 8;//10; // Hz
  double sick_step_angle = 1.5;//0.5;//0.25;
  double active_sector_start_angle = 0;
  double active_sector_stop_angle = 360;//269.75;
  std::string laser_frame_id, laser_child_frame_id, odom_frame_id;
  ros::NodeHandle nh;
  ros::NodeHandle nh_ns("~");
  nh_ns.param<std::string>("scan", scan, "scan");
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan, 100);

  nh_ns.param("port", port, DEFAULT_SICK_TCP_PORT);
  nh_ns.param("ipaddress", ipaddress, (std::string)DEFAULT_SICK_IP_ADDRESS);
  nh_ns.param("inverted", inverted, false);
  nh_ns.param<std::string>("frame_id", frame_id, "front_laser"); //laser frame for scan data
  nh_ns.param<std::string>("fixed_frame_id", fixed_frame_id, "front_mount"); // nav350 mount position frame on the robot

  nh_ns.param<std::string>("laser_frame_id", laser_frame_id, "map"); //global cooridnate frame measurement for navigation and position based on reflectors
  nh_ns.param<std::string>("laser_child_frame_id", laser_child_frame_id, "reflector");// a fixed frame eg: odom or base or reflector

  ros::Subscriber sub = nh.subscribe("odom", 10, OdometryCallback); // data from sensor fusion or wheel odometry of jackal robot

  ros::Publisher pub = nh.advertise<sicknav350::PointArray>("reflectors", 1);


  nh_ns.param("resolution", sick_step_angle, 1.0);
  nh_ns.param("start_angle", active_sector_start_angle, 0.);
  nh_ns.param("stop_angle", active_sector_stop_angle, 360.);
  nh_ns.param("scan_rate", sick_motor_speed, 5);

  /* Define buffers for return values */
  double range_values[SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};
  unsigned int intensity_values[SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};
  /* Define buffers to hold sector specific data */
  unsigned int num_measurements = {0};
  unsigned int sector_start_timestamp = {0};
  unsigned int sector_stop_timestamp = {0};
  double sector_step_angle = {0};
  double sector_start_angle = {0};
  double sector_stop_angle = {0};
  /* Instantiate the object */
  SickNav350 sick_nav350(ipaddress.c_str(), port);
  //  ros::Duration(50).sleep(); //timedelay for jackal robot startup jobs
  double last_time_stamp = 0;

  try
  {
    /* Initialize the device */
    sick_nav350.Initialize();
    sick_nav350.SetAccessMode((int)Users::AUTHORIZED_CLIENT);

    try
    {
      sick_nav350.SetOperatingMode((int)OperatingModes::STANDBY);
      sick_nav350.SetCurrentLayer(1);
      sick_nav350.SetPoseDataFormat(1, 0);
      sick_nav350.SetReflectorWindow(500, 500, 500, 70000);
      sick_nav350.SetActionRadius(400, 70000);
      sick_nav350.SetReflectorSize(80);
      sick_nav350.SetReflectorType((int)ReflectorTypes::FLAT);
      sick_nav350.SetLandmarkMatching(0);
      sick_nav350.SetOperatingMode((int)OperatingModes::NAVIGATION);
      sick_nav350.SetPose(0, 0, 0);
    }
    catch (...)
    {
      ROS_ERROR("Configuration error");
      return -1;
    }

    ros::Time last_start_scan_time;
    unsigned int last_sector_stop_timestamp = 0;
    ros::Rate loop_rate(8);

    while (ros::ok())
    {
      /* Get the scan and landmark measurements */
      sick_nav350.GetDataNavigation(1, 2);
      sick_nav350.GetSickMeasurements(range_values,
                                      &num_measurements,
                                      &sector_step_angle,
                                      &sector_start_angle,
                                      &sector_stop_angle,
                                      &sector_start_timestamp,
                                      &sector_stop_timestamp
                                     );

      std::cout << "\n==============================" << std::endl;
      std::cout << "Pose X : " << sick_nav350.PoseData_.x << std::endl;
      std::cout << "Pose Y : " << sick_nav350.PoseData_.y << std::endl;
      std::cout << "Pose phi : " << sick_nav350.PoseData_.phi << std::endl;

      Point myarray[sick_nav350.ReflectorData_.num_reflector];
      Point point;
      sicknav350::PointArray pointarray;

      for (int i = 0; i < sick_nav350.ReflectorData_.num_reflector; i++)
      {
        point.x = sick_nav350.ReflectorData_.x[i];
        point.y = sick_nav350.ReflectorData_.y[i];
        myarray[i] = point;
      }

      std::vector<Point> my_vector(myarray, myarray + sizeof(myarray) / sizeof(Point));

      pointarray.points.clear();
      pointarray.size = sick_nav350.ReflectorData_.num_reflector;

      int i = 0;
      for (std::vector<Point>::iterator it = my_vector.begin(); it != my_vector.end();  ++it)
      {
        geometry_msgs::Point point;
        point.x = (*it).x;
        point.y = (*it).y;
        point.z = 0;
        pointarray.points.push_back(point);
        i++;
      }
      pub.publish(pointarray);

      if (sector_start_timestamp < last_time_stamp)
      {
        loop_rate.sleep();
        ros::spinOnce();
        continue;
      }
      last_time_stamp = sector_start_timestamp;
      ros::Time end_scan_time = ros::Time::now();

      double scan_duration = 0.125;

      ros::Time start_scan_time = end_scan_time - ros::Duration(scan_duration);
      sector_start_angle -= 180;
      sector_stop_angle -= 180;
      publish_scan(&scan_pub, range_values, num_measurements, intensity_values,
                   num_measurements, start_scan_time, scan_duration, inverted,
                   (float)sector_start_angle, (float)sector_stop_angle, frame_id, sector_start_timestamp);

      last_start_scan_time = start_scan_time;
      last_sector_stop_timestamp = sector_stop_timestamp;

      sick_nav350.SetSpeed(vx, vy, vth, sector_start_timestamp, 0);

      loop_rate.sleep();
      ros::spinOnce();

    }
    /* Uninitialize the device */
    sick_nav350.Uninitialize();
  }
  catch (...)
  {
    ROS_ERROR("Error");
    return -1;
  }
  return 0;
}
