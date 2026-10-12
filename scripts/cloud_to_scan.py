#!/usr/bin/env python3
"""Publish /scan. Body-frame points with z < 0.25 are dropped."""
import math

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan, PointCloud2


class CloudToScan(object):
    def __init__(self):
        self.z_min = float(rospy.get_param("~z_min", 0.25))
        self.z_max = float(rospy.get_param("~z_max", 2.0))
        self.angle_min = float(rospy.get_param("~angle_min", -math.pi))
        self.angle_max = float(rospy.get_param("~angle_max", math.pi))
        self.angle_increment = float(rospy.get_param("~angle_increment", math.radians(1.0)))
        self.range_min = float(rospy.get_param("~range_min", 0.50))
        self.range_max = float(rospy.get_param("~range_max", 30.0))
        self.footprint_radius = float(rospy.get_param("~footprint_radius", 0.75))
        bins = int(round((self.angle_max - self.angle_min) / self.angle_increment))
        self.bins = max(1, bins)
        self.pub = rospy.Publisher("/scan", LaserScan, queue_size=1)
        rospy.Subscriber("/current_scan_body", PointCloud2, self.callback, queue_size=1)

    def callback(self, msg):
        ranges = np.full(self.bins, np.inf, dtype=np.float32)
        for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if z < self.z_min or z > self.z_max:
                continue
            if math.hypot(x, y) < self.footprint_radius:
                continue
            dist = math.hypot(x, y)
            if dist < self.range_min or dist > self.range_max:
                continue
            angle = math.atan2(y, x)
            if angle < self.angle_min or angle > self.angle_max:
                continue
            index = int((angle - self.angle_min) / self.angle_increment)
            if index < 0 or index >= self.bins:
                continue
            if dist < ranges[index]:
                ranges[index] = dist
        scan = LaserScan()
        scan.header = msg.header
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_min + self.bins * self.angle_increment
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [self.range_max + 1.0 if not math.isfinite(value) else float(value) for value in ranges]
        self.pub.publish(scan)


if __name__ == "__main__":
    rospy.init_node("cloud_to_scan")
    CloudToScan()
    rospy.spin()
