/**
 *
 *  \file
 *  \brief Node provides to_fix and from_fix conversion services.
 *  \author Mike Purvis <mpurvis@clearpathrobotics.com>
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
#include "enu/FromFix.h"
#include "enu/ToFix.h"
#include "enu/ToENU.h"
#include "enu/enu.h"  // ROS wrapper for conversion functions

bool from_fix_service(const enu::FromFix::Request& req, enu::FromFix::Response& resp) {
  enu::fix_to_point(req.llh, req.datum, &(resp.point));
  return true;
}

bool to_fix_service(const enu::ToFix::Request& req, enu::ToFix::Response& resp) {
  enu::point_to_fix(req.point, req.datum, &(resp.llh));
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "enu_services");
  ros::NodeHandle n;

  ros::ServiceServer from_fix_srv = 
    n.advertiseService<enu::FromFix::Request, enu::FromFix::Response> ("from_fix", from_fix_service);
  ros::ServiceServer to_fix_srv =
    n.advertiseService<enu::ToFix::Request, enu::ToFix::Response> ("to_fix", to_fix_service);

  ros::spin();
  return 0;
}

