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
 *
 *  File: to_fix.cpp
 *  Desc: Node which receives a latlon datum and ENU Odometry messages, and
          outputs a NavSatFix.
 *  Auth: Mike Purvis
 *
 *  Copyright (c) 2012, Clearpath Robotics, Inc. 
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


static void handle_enu(const nav_msgs::OdometryConstPtr odom_ptr,
                       const double ecef_datum[3],
                       const ros::Publisher& pub_fix)
{
  // Prepare NED vector from ENU coordinates, perform conversion in libswiftnav
  // library calls.
  double ned[3] = { odom_ptr->pose.pose.position.y,
                    odom_ptr->pose.pose.position.x
                    -odom_ptr->pose.pose.position.z };

  double ecef[3];
  wgsned2ecef_d(ned, ecef_datum, ecef);
  
  double llh[3];
  wgsecef2llh(ecef, llh);

  // Prepare output Fix message. Copy over timestamp from source message,
  // convert radian latlon output back to degrees.
  sensor_msgs::NavSatFix fix_msg;
  fix_msg.header.stamp = odom_ptr->header.stamp;
  fix_msg.latitude = llh[0] * TO_DEGREES;
  fix_msg.longitude = llh[1] * TO_DEGREES;
  fix_msg.altitude = llh[2];

  // We only need to populate the diagonals of the covariance matrix; the
  // rest initialize to zero automatically, which is correct as the
  // dimensions of the state are independent.
  fix_msg.position_covariance[0] = odom_ptr->pose.covariance[0];
  fix_msg.position_covariance[4] = odom_ptr->pose.covariance[7];
  fix_msg.position_covariance[8] = odom_ptr->pose.covariance[14];

  pub_fix.publish(fix_msg); 
}


static void handle_datum(const sensor_msgs::NavSatFixConstPtr datum_ptr,
                         ros::NodeHandle& n)
{
  // Convert received datum to latlon using function from libswiftnav.
  double llh[3] = { datum_ptr->latitude * TO_RADIANS,
                    datum_ptr->longitude * TO_RADIANS,
                    datum_ptr->altitude };
  static double ecef_datum[3];
  wgsllh2ecef(llh, ecef_datum);

  // Pass datum into the data subscriber. Subscriber, Publisher, and the datum
  // array are static, so that they stick around when this function has exited.
  static ros::Publisher pub_fix = n.advertise<sensor_msgs::NavSatFix>("fix", 5);
  static ros::Subscriber sub_enu = n.subscribe<nav_msgs::Odometry>("enu", 5, 
      boost::bind(handle_enu, _1, ecef_datum, boost::ref(pub_fix)));
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "to_fix");
  ros::NodeHandle n;

  // Initially just subscribe to the datum topic. Once we receive that, we'll
  // spin up the rest of the node's activities.
  ros::Subscriber sub_datum = n.subscribe<sensor_msgs::NavSatFix>("enu_datum", 5,
      boost::bind(handle_datum, _1, boost::ref(n)));

  ros::spin();
  return 0;
}

