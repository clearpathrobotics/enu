#!/usr/bin/env python

import roslib; roslib.load_manifest('enu')
import rospy

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3, Quaternion
from nav_msgs.msg import Odometry

from math import radians, cos, sin, sqrt

A_EARTH = 6378137.0
FLATTENING = 1.0 / 298.257223563
NAV_E_TWO = (2.0 - FLATTENING) * FLATTENING
      

class NavSatFixExtended(NavSatFix):
  def to_ecef(fix):
    if not hasattr(fix, '_ecef'):
      slat = sin(radians(fix.latitude))
      clat = cos(radians(fix.latitude))
      r_n = A_EARTH / sqrt(1 - NAV_E_TWO * slat * slat)

      fix._ecef = Vector3((r_n + fix.altitude) * clat * cos(radians(fix.longitude)),
                          (r_n + fix.altitude) * clat * sin(radians(fix.longitude)),
                          (r_n * (1 - NAV_E_TWO) + fix.altitude) * slat)
    return fix._ecef

  def to_enu(fix, fix_datum):
    ecef = fix.to_ecef()
    ecef_init = fix_datum.to_ecef()
    diff = Vector3(ecef.x - ecef_init.x, 
                   ecef.y - ecef_init.y, 
                   ecef.z - ecef_init.z)  
    lamb = radians(fix_datum.longitude)
    phi = radians(fix_datum.latitude)

    return Vector3(-1.0 * sin(lamb) * diff.x + cos(lamb) * diff.y, 
                   -1.0 * sin(phi) * cos(lamb) * diff.x - 
                          sin(phi) * sin(lamb) * diff.y + 
                          cos(phi) * diff.z,
                   cos(phi) * cos(lamb) * diff.x + 
                   cos(phi) * sin(lamb) * diff.y +
                   sin(phi) * diff.z)


class ENUPublisher:
  def __init__(self):
    self.odometry_msg = Odometry()
    self.odometry_msg.pose.pose.orientation = Quaternion(0,0,0,1)
    self.odometry_msg.pose.covariance[21] = 1000
    self.odometry_msg.pose.covariance[28] = 1000
    self.odometry_msg.pose.covariance[35] = 1000

    rospy.Subscriber('fix', NavSatFixExtended, self._cb)
    self.enu_pub = rospy.Publisher('enu', Odometry)
    self.datum_pub = rospy.Publisher('enu_datum', NavSatFix, latch=True)

    try:
      # If there was a datum parameterized in, use that.
      self.datum = NavSatFix(rospy.get_param('~datum_latitude', None),
                             rospy.get_param('~datum_longitude', None),
                             rospy.get_param('~datum_altitude', None))
      self.datum_pub.publish(self.datum) 
    except TypeError:
      # Otherwise, wait for the first fix.
      self.datum = None

  def _cb(self, fix):
    if not self.datum:
      self.datum = fix
      self.datum_pub.publish(self.datum)

    self.odometry_msg.pose.pose.position = fix.to_enu(self.datum)
    self.odometry_msg.pose.covariance[0] = fix.position_covariance[0]
    self.odometry_msg.pose.covariance[7] = fix.position_covariance[4]
    self.odometry_msg.pose.covariance[14] = fix.position_covariance[8]
    self.enu_pub.publish(self.odometry_msg) 


if __name__ == "__main__":
  rospy.init_node('enu_publisher')
  ENUPublisher()
  rospy.spin()
