/**
 *
 *  \file
 *  \brief Wrapping swiftnav with ROS message types to support LLH->ENU
 *         and LLH<->ECEF conversions
 *  \author Ryan Gariepy <rgariepy@clearpathrobotics.com>
 *
 *  \copyright Copyright (c) 2013, Clearpath Robotics, Inc. 
 *
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
 * Please send comments, questions, or patches to code@clearpathrobotics.com 
 *
 */

#include "enu/enu_ros.h"

#include <boost/bind.hpp>
#include <string>

extern "C" {
  // The backend of this node is an included C library called libswiftnav.
  #include "libswiftnav/include/coord_system.h"
}

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

#define TO_RADIANS (M_PI/180)
#define TO_DEGREES (180/M_PI)

void llh_to_ecef(const sensor_msgs::NavSatFix& llh_ptr, double ecef[3]) {
  // The datum point is stored as an ECEF, for mathematical reasons.
  // We convert it here, using the appropriate function from
  // libswiftnav.
  double llh_raw[3] = { llh_ptr.latitude * TO_RADIANS,
                        llh_ptr.longitude * TO_RADIANS,
                        llh_ptr.altitude };
  wgsllh2ecef(llh_raw, ecef);
}

void llh_to_enu(const sensor_msgs::NavSatFixConstPtr fix_ptr,
                const sensor_msgs::NavSatFix& datum,
                const std::string& output_tf_frame,
                double invalid_covariance_value,
                nav_msgs::Odometry& enu) {
  // Convert reference LLH-formatted datum to ECEF format
  double ecef_datum[3];
  llh_to_ecef(datum, ecef_datum);

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

  // Output data
  enu.header.stamp = fix_ptr->header.stamp;
  enu.header.frame_id = output_tf_frame;  // Name of output tf frame
  enu.child_frame_id = fix_ptr->header.frame_id;  // Antenna location
  enu.pose.pose.position.x = ned[1];
  enu.pose.pose.position.y = ned[0];
  enu.pose.pose.position.z = -ned[2];

  // We only need to populate the diagonals of the covariance matrix; the
  // rest initialize to zero automatically, which is correct as the
  // dimensions of the state are independent.
  enu.pose.covariance[0] = fix_ptr->position_covariance[0];
  enu.pose.covariance[7] = fix_ptr->position_covariance[4];
  enu.pose.covariance[14] = fix_ptr->position_covariance[8];

  // Do not use orientation dimensions from GPS.
  // (-1 is an invalid covariance and standard ROS practice to set as invalid.)
  enu.pose.covariance[21] = invalid_covariance_value;
  enu.pose.covariance[28] = invalid_covariance_value;
  enu.pose.covariance[35] = invalid_covariance_value;
}

void enu_to_llh(const nav_msgs::OdometryConstPtr odom_ptr,
                const sensor_msgs::NavSatFix& datum,
                sensor_msgs::NavSatFix& llh) {
  // Convert reference LLH-formatted datum to ECEF format
  double ecef_datum[3];
  llh_to_ecef(datum, ecef_datum);

  // Prepare NED vector from ENU coordinates, perform conversion in libswiftnav
  // library calls.
  double ned[3] = { odom_ptr->pose.pose.position.y,
                    odom_ptr->pose.pose.position.x
                    -odom_ptr->pose.pose.position.z };

  double ecef[3];
  wgsned2ecef_d(ned, ecef_datum, ecef);

  double llh_raw[3];
  wgsecef2llh(ecef, llh_raw);

  // Output Fix message. Copy over timestamp from source message,
  // convert radian latlon output back to degrees.
  llh.header.frame_id = odom_ptr->child_frame_id;
  llh.header.stamp = odom_ptr->header.stamp;
  llh.latitude = llh_raw[0] * TO_DEGREES;
  llh.longitude = llh_raw[1] * TO_DEGREES;
  llh.altitude = llh_raw[2];

  // We only need to populate the diagonals of the covariance matrix; the
  // rest initialize to zero automatically, which is correct as the
  // dimensions of the state are independent.
  llh.position_covariance[0] = odom_ptr->pose.covariance[0];
  llh.position_covariance[4] = odom_ptr->pose.covariance[7];
  llh.position_covariance[8] = odom_ptr->pose.covariance[14];
}
