#!/usr/bin/env python3
"""Turn /scan hits into obstacles expressed in the odometry frame."""
import math

import rospy
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def quat_rotate(qx, qy, qz, qw, vx, vy, vz):
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


class ScanObstacles(object):
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "camera_init")
        self.max_range = float(rospy.get_param("~max_range", 3.5))
        self.stride = int(rospy.get_param("~stride", 4))
        self.radius = float(rospy.get_param("~radius", 0.08))
        self.odom = None
        self.pub = rospy.Publisher("/obstacles_raw", ObstacleArrayMsg, queue_size=1)
        rospy.Subscriber("/odometry", Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.on_scan, queue_size=1)

    def on_odom(self, msg):
        self.odom = msg

    def on_scan(self, msg):
        out = ObstacleArrayMsg()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        if self.odom is None:
            self.pub.publish(out)
            return
        pose = self.odom.pose.pose
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        angle = msg.angle_min
        for index, dist in enumerate(msg.ranges):
            angle += msg.angle_increment if index else 0.0
            if index % self.stride:
                continue
            if not math.isfinite(dist) or dist < msg.range_min or dist > min(msg.range_max, self.max_range):
                continue
            lx = dist * math.cos(angle)
            ly = dist * math.sin(angle)
            wx, wy, _ = quat_rotate(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0), lx, ly, 0.0)
            point = Point32()
            point.x = pose.position.x + wx
            point.y = pose.position.y + wy
            obstacle = ObstacleMsg()
            obstacle.radius = self.radius
            obstacle.polygon.points.append(point)
            obstacle.orientation.w = 1.0
            out.obstacles.append(obstacle)
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("scan_obstacles")
    ScanObstacles()
    rospy.spin()
