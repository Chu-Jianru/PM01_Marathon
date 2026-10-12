#!/usr/bin/env python3
"""Publish Gazebo pose as the RTK fix GLIO subscribes to.

robosense.yaml reads gnss_ksxt as sensor_msgs/NavSatFix. Heading is carried
in position_covariance[6], and status 2 is the RTK fix the initializer accepts.
"""
import math

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus


EARTH_RADIUS = 6378137.0


class GnssKsxt(object):
    def __init__(self):
        self.lat0 = math.radians(float(rospy.get_param("~origin_latitude", 40.0)))
        self.lon0 = math.radians(float(rospy.get_param("~origin_longitude", 116.0)))
        self.pub = rospy.Publisher("/gnss_ksxt", NavSatFix, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.callback, queue_size=10)

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_deg = math.degrees(math.atan2(siny, cosy))
        fix = NavSatFix()
        fix.header = msg.header
        fix.header.frame_id = "gnss"
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = math.degrees(self.lat0 + y / EARTH_RADIUS)
        fix.longitude = math.degrees(self.lon0 + x / (EARTH_RADIUS * math.cos(self.lat0)))
        fix.altitude = z
        fix.position_covariance[0] = 0.0001
        fix.position_covariance[4] = 0.0001
        fix.position_covariance[6] = 90.0 - yaw_deg
        fix.position_covariance[8] = 0.0001
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub.publish(fix)


if __name__ == "__main__":
    rospy.init_node("gnss_ksxt")
    GnssKsxt()
    rospy.spin()
