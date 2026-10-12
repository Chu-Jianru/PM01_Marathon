#!/usr/bin/env python3
"""Publish /gnss_ksxt from /odom."""
import math

import rospy
from nav_msgs.msg import Odometry
from osgeo import osr
from sensor_msgs.msg import NavSatFix, NavSatStatus


_PI = math.pi
_EE = 0.00669342162296594323
_A = 6378245.0
_PROJ4 = "+proj=tmerc +lat_0=0 +lon_0=117 +k=1.0 +x_0=500000 +y_0=0 +ellps=GRS80 +units=m +no_defs"


def _transform_lat(lng, lat):
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * _PI) + 20.0 * math.sin(2.0 * lng * _PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat * _PI) + 40.0 * math.sin(lat / 3.0 * _PI)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * _PI) + 320.0 * math.sin(lat * _PI / 30.0)) * 2.0 / 3.0
    return ret


def _transform_lng(lng, lat):
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * _PI) + 20.0 * math.sin(2.0 * lng * _PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lng * _PI) + 40.0 * math.sin(lng / 3.0 * _PI)) * 2.0 / 3.0
    ret += (150.0 * math.sin(lng / 12.0 * _PI) + 300.0 * math.sin(lng / 30.0 * _PI)) * 2.0 / 3.0
    return ret


def _wgs84_to_gcj02(lng, lat):
    dlat = _transform_lat(lng - 105.0, lat - 35.0)
    dlng = _transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * _PI
    magic = 1.0 - _EE * math.sin(radlat) ** 2
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((_A * (1.0 - _EE)) / (magic * sqrtmagic) * _PI)
    dlng = (dlng * 180.0) / (_A / sqrtmagic * math.cos(radlat) * _PI)
    return lng + dlng, lat + dlat


def _invert_gcj02(gcj_lng, gcj_lat):
    lng, lat = gcj_lng, gcj_lat
    for _ in range(8):
        got_lng, got_lat = _wgs84_to_gcj02(lng, lat)
        lng -= got_lng - gcj_lng
        lat -= got_lat - gcj_lat
    return lng, lat


class GnssKsxt(object):
    def __init__(self):
        self.lat0 = float(rospy.get_param("~origin_latitude", 40.0))
        self.lon0 = float(rospy.get_param("~origin_longitude", 117.0))
        self.drop_start = float(rospy.get_param("~drop_start_m", 80.0))
        self.drop_length = float(rospy.get_param("~drop_length_m", 0.0))
        self.origin = None
        self.drop_done = False
        wgs = osr.SpatialReference()
        wgs.ImportFromEPSG(4326)
        wgs.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
        projected = osr.SpatialReference()
        projected.ImportFromProj4(_PROJ4)
        projected.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
        self._to_xy = osr.CoordinateTransformation(wgs, projected)
        self._to_lonlat = osr.CoordinateTransformation(projected, wgs)
        gcj_lon, gcj_lat = _wgs84_to_gcj02(self.lon0, self.lat0)
        self.anchor_x, self.anchor_y, _ = self._to_xy.TransformPoint(gcj_lon, gcj_lat)
        self.pub = rospy.Publisher("/gnss_ksxt", NavSatFix, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.callback, queue_size=10)

    def _latlon(self, x, y):
        gcj_lon, gcj_lat, _ = self._to_lonlat.TransformPoint(self.anchor_x + x, self.anchor_y + y)
        return _invert_gcj02(gcj_lon, gcj_lat)

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_deg = math.degrees(math.atan2(siny, cosy))
        if self.origin is None and math.hypot(x, y) > 1.0:
            self.origin = (x, y)
        status = NavSatStatus.STATUS_GBAS_FIX
        if self.origin is not None and not self.drop_done:
            traveled = math.hypot(x - self.origin[0], y - self.origin[1])
            if self.drop_start <= traveled < self.drop_start + self.drop_length:
                status = NavSatStatus.STATUS_NO_FIX
            elif traveled >= self.drop_start + self.drop_length:
                self.drop_done = True
        longitude, latitude = self._latlon(x, y)
        fix = NavSatFix()
        fix.header = msg.header
        fix.header.frame_id = "gnss"
        fix.status.status = status
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = latitude
        fix.longitude = longitude
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
