

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

#include <boost/bind.hpp>

extern "C" {
  #include "coord_system.h"
}

#define TO_RADIANS (M_PI/180)
#define TO_DEGREES (180/M_PI)


static void handle_enu(const nav_msgs::OdometryConstPtr odom_ptr,
                       const double ecef_datum[3],
                       const ros::Publisher& pub_fix)
{
  double ned[3] = { odom_ptr->pose.pose.position.y,
                    odom_ptr->pose.pose.position.x
                    -odom_ptr->pose.pose.position.z };

  double ecef[3];
  wgsned2ecef_d(ned, ecef_datum, ecef);
  
  double llh[3];
  wgsecef2llh(ecef, llh);

  sensor_msgs::NavSatFix fix_msg;
  fix_msg.header.stamp = odom_ptr->header.stamp;
  fix_msg.latitude = llh[0] * TO_DEGREES;
  fix_msg.longitude = llh[1] * TO_DEGREES;
  fix_msg.altitude = llh[2];
  fix_msg.position_covariance[0] = odom_ptr->pose.covariance[0];
  fix_msg.position_covariance[4] = odom_ptr->pose.covariance[7];
  fix_msg.position_covariance[8] = odom_ptr->pose.covariance[14];
  pub_fix.publish(fix_msg); 
}


static void handle_datum(const sensor_msgs::NavSatFixConstPtr datum_ptr,
                         ros::NodeHandle& n)
{
  double llh[3] = { datum_ptr->latitude * TO_RADIANS,
                    datum_ptr->longitude * TO_RADIANS,
                    datum_ptr->altitude };

  static double ecef_datum[3];
  wgsllh2ecef(llh, ecef_datum);

  static ros::Publisher pub_fix = n.advertise<sensor_msgs::NavSatFix>("fix", 5);
  static ros::Subscriber sub_enu = n.subscribe<nav_msgs::Odometry>("enu", 5, 
      boost::bind(handle_enu, _1, ecef_datum, boost::ref(pub_fix)));
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "to_fix");
  ros::NodeHandle n;

  ros::Subscriber sub_datum = n.subscribe<sensor_msgs::NavSatFix>("enu_datum", 5,
      boost::bind(handle_datum, _1, boost::ref(n)));

  ros::spin();
  return 0;
}

