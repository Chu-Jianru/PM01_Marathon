#!/usr/bin/env python3
"""Stack tilted lidar rings into one body-frame cloud.

Publishes the names used by the marathon tracker and by GLIO:
/current_scan_body, the README spelling /cuerrent_scan_body, and
/cloud_registered_body.
"""
import math

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header


SENSOR_X = 0.40
SENSOR_Z = 0.32


class ScanToCloud(object):
    def __init__(self):
        topics = rospy.get_param("~ring_topics", ["/scan_ring_0", "/scan_ring_1", "/scan_ring_2", "/scan_ring_3"])
        pitches = rospy.get_param("~ring_pitches_deg", [-10.0, -3.0, 0.0, 7.0])
        if len(topics) != len(pitches):
            raise rospy.ROSException("ring_topics and ring_pitches_deg must have the same length")
        self.rings = []
        for topic, pitch_deg in zip(topics, pitches):
            pitch = math.radians(float(pitch_deg))
            self.rings.append({"topic": topic, "pitch": pitch, "scan": None})
            rospy.Subscriber(topic, LaserScan, self._make_callback(len(self.rings) - 1), queue_size=1)
        self.publishers = [
            rospy.Publisher("/current_scan_body", PointCloud2, queue_size=1),
            rospy.Publisher("/cuerrent_scan_body", PointCloud2, queue_size=1),
            rospy.Publisher("/cloud_registered_body", PointCloud2, queue_size=1),
        ]
        rospy.Timer(rospy.Duration(0.1), self.publish)

    def _make_callback(self, index):
        def callback(msg):
            self.rings[index]["scan"] = msg
        return callback

    def publish(self, _event):
        points = []
        stamp = None
        for ring in self.rings:
            scan = ring["scan"]
            if scan is None:
                continue
            if stamp is None or scan.header.stamp > stamp:
                stamp = scan.header.stamp
            cosine = math.cos(ring["pitch"])
            sine = math.sin(ring["pitch"])
            angle = scan.angle_min
            for dist in scan.ranges:
                if not math.isfinite(dist) or dist < scan.range_min or dist > scan.range_max:
                    angle += scan.angle_increment
                    continue
                ray_x = dist * math.cos(angle)
                ray_y = dist * math.sin(angle)
                # Ry(pitch): positive pitch looks down.
                points.append((
                    ray_x * cosine + SENSOR_X,
                    ray_y,
                    -ray_x * sine + SENSOR_Z,
                ))
                angle += scan.angle_increment
        if stamp is None:
            return
        header = Header(stamp=stamp, frame_id="body")
        cloud = pc2.create_cloud_xyz32(header, points)
        for publisher in self.publishers:
            publisher.publish(cloud)


if __name__ == "__main__":
    rospy.init_node("scan_to_cloud")
    ScanToCloud()
    rospy.spin()
