#!/usr/bin/env python3
"""Publish /rslidar_points and /current_scan_body."""
import math
import struct

import rospy
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
from std_msgs.msg import Header


POINT_STEP = 32


def _fields():
    # Layout matches the PCL RoboSense point: xyz, padding, intensity, ring, timestamp.
    return [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("intensity", 16, PointField.FLOAT32, 1),
        PointField("ring", 20, PointField.UINT16, 1),
        PointField("timestamp", 24, PointField.FLOAT64, 1),
    ]


class RsLidarPoints(object):
    def __init__(self):
        self.horizontal = int(rospy.get_param("~horizontal_samples", 360))
        self.vertical = int(rospy.get_param("~vertical_samples", 32))
        self.scan_period = float(rospy.get_param("~scan_period", 0.1))
        self.sensor_xyz = rospy.get_param("~sensor_xyz", [0.40, 0.0, 0.32])
        self.frame_lidar = rospy.get_param("~lidar_frame", "laser")
        self.frame_body = rospy.get_param("~body_frame", "body")
        self.pub_lidar = rospy.Publisher("/rslidar_points", PointCloud2, queue_size=1)
        self.pub_body = rospy.Publisher("/current_scan_body", PointCloud2, queue_size=1)
        self.pub_body_typo = rospy.Publisher("/cuerrent_scan_body", PointCloud2, queue_size=1)
        rospy.Subscriber("/rslidar_block", PointCloud, self.callback, queue_size=1)

    def callback(self, msg):
        count = len(msg.points)
        if count == 0:
            return
        horizontal = self.horizontal if count % self.vertical == 0 else max(1, count // self.vertical)
        stamp = msg.header.stamp.to_sec()
        sx, sy, sz = self.sensor_xyz
        lidar_blob = bytearray()
        body_blob = bytearray()
        kept = 0
        for index, point in enumerate(msg.points):
            if not (math.isfinite(point.x) and math.isfinite(point.y) and math.isfinite(point.z)):
                continue
            ring = index // horizontal
            if ring >= self.vertical:
                ring = self.vertical - 1
            azimuth = index % horizontal
            timestamp = stamp + self.scan_period * (float(azimuth) / float(horizontal))
            intensity = 100.0
            packed_lidar = struct.pack(
                "<fffffHxxd", point.x, point.y, point.z, 0.0, intensity, ring, timestamp)
            packed_body = struct.pack(
                "<fffffHxxd", point.x + sx, point.y + sy, point.z + sz, 0.0, intensity, ring, timestamp)
            lidar_blob.extend(packed_lidar)
            body_blob.extend(packed_body)
            kept += 1
        if kept == 0:
            return
        header = Header(stamp=msg.header.stamp, frame_id=self.frame_lidar)
        self.pub_lidar.publish(self._cloud(header, lidar_blob, kept))
        body_header = Header(stamp=msg.header.stamp, frame_id=self.frame_body)
        body_cloud = self._cloud(body_header, body_blob, kept)
        self.pub_body.publish(body_cloud)
        self.pub_body_typo.publish(body_cloud)

    def _cloud(self, header, blob, count):
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = count
        cloud.fields = _fields()
        cloud.is_bigendian = False
        cloud.point_step = POINT_STEP
        cloud.row_step = POINT_STEP * count
        cloud.data = bytes(blob)
        cloud.is_dense = True
        return cloud


if __name__ == "__main__":
    rospy.init_node("rslidar_points")
    RsLidarPoints()
    rospy.spin()
