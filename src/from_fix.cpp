/**
 *
 *  \file
 *  \brief Node receives NavSatFix messages and publishes ENU Odometry messages.
 *  \author Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \author Ryan Gariepy <rgariepy@clearpathrobotics.com>
 *
 *  \copyright Copyright (c) 2013, Clearpath Robotics, Inc. 
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

#include <boost/bind.hpp>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

#include "enu/enu.h"  // ROS wrapper for conversion functions


void initialize_datum(const sensor_msgs::NavSatFix& fix,
                      const ros::Publisher& pub_datum,
                      sensor_msgs::NavSatFix* datum_ptr) {
  ros::NodeHandle pnh("~");

  // Local ENU coordinates are with respect to a plane which is
  // perpendicular to a particular lat/lon. This logic decides
  // whether to use a specific passed-in point (typical for
  // repeated tests in a locality) or just an arbitrary starting
  // point (more ad-hoc type scenarios).
  if (pnh.hasParam("datum_latitude") &&
      pnh.hasParam("datum_longitude") &&
      pnh.hasParam("datum_altitude")) {
    pnh.getParam("datum_latitude", datum_ptr->latitude);
    pnh.getParam("datum_longitude", datum_ptr->longitude);
    pnh.getParam("datum_altitude", datum_ptr->altitude);
    ROS_INFO("Using datum provided by node parameters.");
  } else {
    datum_ptr->latitude = fix.latitude;
    datum_ptr->longitude = fix.longitude;
    datum_ptr->altitude = fix.altitude;
    ROS_INFO("Using initial position fix as datum.");
  }
  pub_datum.publish(*datum_ptr);
}


static void handle_fix(const sensor_msgs::NavSatFixConstPtr fix_ptr,
                       const ros::Publisher& pub_odom,
                       const ros::Publisher& pub_datum,
                       const std::string& output_tf_frame,
                       const double invalid_covariance_value,
                       const double scale_covariance,
                       const double lock_altitude) {
  static sensor_msgs::NavSatFix datum;
  static bool have_datum = false;

  if (!have_datum) {
    initialize_datum(*fix_ptr, pub_datum, &datum);
    have_datum = true;
  }

  // Convert the input latlon into north-east-down (NED) via an ECEF
  // transformation and an ECEF-formatted datum point
  nav_msgs::Odometry odom;
  enu::fix_to_point(*fix_ptr, datum, &odom.pose.pose.position);
  if (lock_altitude!=-1) 
    odom.pose.pose.position.z = lock_altitude;

  odom.header.stamp = fix_ptr->header.stamp;
  odom.header.frame_id = output_tf_frame;  // Name of output tf frame
  odom.child_frame_id = fix_ptr->header.frame_id;  // Antenna location

  //Initialize orientation to zero (GPS orientation is N/A)
  odom.pose.pose.orientation.x = 0;
  odom.pose.pose.orientation.y = 0;
  odom.pose.pose.orientation.z = 0;
  odom.pose.pose.orientation.w = 1;

  // We only need to populate the diagonals of the covariance matrix; the
  // rest initialize to zero automatically, which is correct as the
  // dimensions of the state are independent.
  odom.pose.covariance[0] = fix_ptr->position_covariance[0]*scale_covariance;
  odom.pose.covariance[7] = fix_ptr->position_covariance[4]*scale_covariance;
  odom.pose.covariance[14] = fix_ptr->position_covariance[8]*scale_covariance;

  // Do not use orientation dimensions from GPS.
  // (-1 is an invalid covariance and standard ROS practice to set as invalid.)
  odom.pose.covariance[21] = invalid_covariance_value;
  odom.pose.covariance[28] = invalid_covariance_value;
  odom.pose.covariance[35] = invalid_covariance_value;

  pub_odom.publish(odom);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "from_fix");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  std::string output_tf_frame;
  pnh.param<std::string>("output_frame_id", output_tf_frame, "map");
  double invalid_covariance_value;
  double scale_covariance;
  double lock_altitude;
  pnh.param<double>("invalid_covariance_value", invalid_covariance_value, -1.0);  // -1 is ROS convention.  1e6 is robot_pose_ekf convention
  pnh.param<double>("scale_covariance", scale_covariance,1.0);
  pnh.param<double>("lock_altitude", lock_altitude, -1.0); //if -1 then use ENU altitude, otherwise lock the altitude at the provided value

  // Initialize publishers, and pass them into the handler for
  // the subscriber.
  ros::Publisher pub_odom = n.advertise<nav_msgs::Odometry>("enu", 5);
  ros::Publisher pub_datum = n.advertise<sensor_msgs::NavSatFix>("enu_datum", 5, true);
  ros::Subscriber sub = n.subscribe<sensor_msgs::NavSatFix>("fix", 5,
      boost::bind(handle_fix, _1, pub_odom, pub_datum, output_tf_frame, invalid_covariance_value,scale_covariance, lock_altitude));

  ros::spin();
  return 0;
}

