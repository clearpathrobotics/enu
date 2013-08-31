/**
 *
 *  \file
 *  \brief Functions for conversion between LLH and ENU coordinates
 *         (given a defined datum) and LLH->ECEF (for datum generation)
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

#ifndef INCLUDE_ENU_ROS_H_

#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

/*
 * Converts an LLH coordinate into the corresponding ECEF coordinate. Primarily intended for datum generation for LLH<->ENU conversion. 
 * \param [in] llh_ptr NavSatFix with a valid latitude, longitude, and altitude
 * \param [out] ecef[3] ECEF array, in metres
 * \return N/A 
 */
void llh_to_ecef(const sensor_msgs::NavSatFix& llh_ptr, double ecef[3]);

/*
 * Converts an LLH coordinate into the corresponding ENU coordinate, when also
 * given a datum (in ECEF format). Use llh_to_ecef to determine the datum.
 * \param [in] fix_ptr NavSatFix with a valid latitude, longitude, and altitude
 * representing the current position
 * \param [in] Datum point in LLH format
 * \param [in] output_tf_frame The string to be used as the frame_id in the output
 * \param [in] invalid_covariance_value The value which represents invalid covariances
 * TODO: Deprecate invalid_covariance_value when it's fully standardized across ROS (rgariepy, 30 Aug 2013)
 * \param [out] ENU of current position relative to datum
 */
void llh_to_enu(const sensor_msgs::NavSatFixConstPtr fix_ptr,
                const sensor_msgs::NavSatFix& datum,
                const std::string& output_tf_frame,
                double invalid_covariance_value,
                nav_msgs::Odometry& enu);                 

/*
 * Converts an ENU coordinate into the corresponding LLH coordinate, when also
 * given a datum (in ECEF format). Use llh_to_ecef to determine the datum.
 * TODO: Should we work in LLH only and make ECEF invisible? (rgariepy, 30 Aug 2013)
 * \param [in] Odom_ptr ENU position with respect to datum point
 * \param [in] Datum point in LLH format
 * \param [out] LLH corresponding to ENU + datum combination
 */
void enu_to_llh(const nav_msgs::OdometryConstPtr odom_ptr,
                const sensor_msgs::NavSatFix& datum,
                sensor_msgs::NavSatFix& llh);

#endif  // INCLUDE_ENU_ROS_H_
