

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

#include <boost/bind.hpp>

extern "C" {
  #include "coord_system.h"
}

#define TO_RADIANS (M_PI/180)
#define TO_DEGREES (180/M_PI)

void initialize_datum(double datum_ecef[3],
                      const sensor_msgs::NavSatFixConstPtr fix_ptr,
                      const ros::Publisher& pub_datum)
{
  ros::NodeHandle n("~");
  sensor_msgs::NavSatFix datum_msg(*fix_ptr);
  if (n.hasParam("datum_latitude") &&
      n.hasParam("datum_longitude") && 
      n.hasParam("datum_altitude")) {
    n.getParam("datum_latitude", datum_msg.latitude);  
    n.getParam("datum_longitude", datum_msg.longitude);  
    n.getParam("datum_altitude", datum_msg.altitude);  
    ROS_INFO("Using datum provided by node parameters.");
  } else {
    ROS_INFO("Using initial position fix as datum.");
  }
  pub_datum.publish(datum_msg);

  double llh[3] = { datum_msg.latitude * TO_RADIANS,
                    datum_msg.longitude * TO_RADIANS,
                    datum_msg.altitude };
  wgsllh2ecef(llh, datum_ecef);
}


static void handle_fix(const sensor_msgs::NavSatFixConstPtr fix_ptr,
                       const ros::Publisher& pub_enu,
                       const ros::Publisher& pub_datum)
{
  static double ecef_datum[3];
  static bool have_datum = false;

  if (!have_datum) {
    initialize_datum(ecef_datum, fix_ptr, pub_datum);
    have_datum = true;
  }

  double llh[3] = { fix_ptr->latitude * TO_RADIANS,
                    fix_ptr->longitude * TO_RADIANS,
                    fix_ptr->altitude };
  double ecef[3];
  wgsllh2ecef(llh, ecef);
  
  double ned[3];
  wgsecef2ned_d(ecef, ecef_datum, ned);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = fix_ptr->header.stamp;
  odom_msg.pose.pose.position.x = ned[1];
  odom_msg.pose.pose.position.y = ned[0];
  odom_msg.pose.pose.position.z = -ned[2];
  odom_msg.pose.covariance[0] = fix_ptr->position_covariance[0];
  odom_msg.pose.covariance[7] = fix_ptr->position_covariance[4];
  odom_msg.pose.covariance[14] = fix_ptr->position_covariance[8];
  odom_msg.pose.covariance[21] = 1000;
  odom_msg.pose.covariance[28] = 1000;
  odom_msg.pose.covariance[35] = 1000;
  odom_msg.pose.pose.orientation.w = 1;
  pub_enu.publish(odom_msg); 
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "from_fix");
  ros::NodeHandle n;

  ros::Publisher pub_enu = n.advertise<nav_msgs::Odometry>("enu", 5);
  ros::Publisher pub_datum = n.advertise<sensor_msgs::NavSatFix>("enu_datum", 5, true);
  ros::Subscriber sub = n.subscribe<sensor_msgs::NavSatFix>("fix", 5, 
      boost::bind(handle_fix, _1, pub_enu, pub_datum));

  ros::spin();
  return 0;
}

