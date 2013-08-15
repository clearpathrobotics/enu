/**
 *      _____
 *     /  _  \
 *    / _/ \  \
 *   / / \_/   \
 *  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
 *  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
 *   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
 *    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
 *     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
 *             ROBOTICS
 *`
 *  File: from_fix.cpp
 *  Desc: Node receives NavSatFix messages and publishes ENU Odometry messages.
 *  Auth: Mike Purvis
 *
 *  Copyright (c) 2013, Clearpath Robotics, Inc. 
 *  All Rights Reserved
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Please send comments, questions, or patches to skynet@clearpathrobotics.com 
 *
 */

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

#include <boost/bind.hpp>

extern "C" {
  // The backend of this node is an included C library called libswiftnav.
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

  // Local ENU coordinates are with respect to a plane which is 
  // perpendicular to a particular lat/lon. This logic decides 
  // whether to use a specific passed-in point (typical for 
  // repeated tests in a locality) or just an arbitrary starting
  // point (more ad-hoc type scenarios).
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

  // The datum point is stored as an ECEF, for mathematical reasons.
  // We convert it here, using the appropriate function from
  // libswiftnav.
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

  // Prepare the appropriate input vector to convert the input latlon
  // to an ECEF triplet.
  double llh[3] = { fix_ptr->latitude * TO_RADIANS,
                    fix_ptr->longitude * TO_RADIANS,
                    fix_ptr->altitude };
  double ecef[3];
  wgsllh2ecef(llh, ecef);
  
  // ECEF triplet is converted to north-east-down (NED), by combining it
  // with the ECEF-formatted datum point.
  double ned[3];
  wgsecef2ned_d(ecef, ecef_datum, ned);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = fix_ptr->header.stamp;
  odom_msg.pose.pose.position.x = ned[1];
  odom_msg.pose.pose.position.y = ned[0];
  odom_msg.pose.pose.position.z = -ned[2];

  // We only need to populate the diagonals of the covariance matrix; the
  // rest initialize to zero automatically, which is correct as the
  // dimensions of the state are independent.
  odom_msg.pose.covariance[0] = fix_ptr->position_covariance[0];
  odom_msg.pose.covariance[7] = fix_ptr->position_covariance[4];
  odom_msg.pose.covariance[14] = fix_ptr->position_covariance[8];
  
  // Do not use orientation dimensions from GPS.
  // (-1 is an invalid covariance and standard ROS practice to set as invalid.)
  odom_msg.pose.covariance[21] = -1;
  odom_msg.pose.covariance[28] = -1;
  odom_msg.pose.covariance[35] = -1;

  pub_enu.publish(odom_msg); 
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "from_fix");
  ros::NodeHandle n;

  // Initialize publishers, and pass them into the handler for 
  // the subscriber.
  ros::Publisher pub_enu = n.advertise<nav_msgs::Odometry>("enu", 5);
  ros::Publisher pub_datum = n.advertise<sensor_msgs::NavSatFix>("enu_datum", 5, true);
  ros::Subscriber sub = n.subscribe<sensor_msgs::NavSatFix>("fix", 5, 
      boost::bind(handle_fix, _1, pub_enu, pub_datum));

  ros::spin();
  return 0;
}

