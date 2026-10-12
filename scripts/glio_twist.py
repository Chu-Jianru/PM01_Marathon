#!/usr/bin/env python3
"""Publish /odometry_body from GLIO /Odometry."""
import math

import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry


def yaw_of(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def angle_delta(current, previous):
    delta = current - previous
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


class GlioTwist(object):
    def __init__(self):
        self.previous = None
        self.yaw = 0.0
        self.have_yaw = False
        self.pub = rospy.Publisher("/odometry_body", Odometry, queue_size=10)
        rospy.Subscriber("/Odometry", Odometry, self.callback, queue_size=10)

    def callback(self, msg):
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = "camera_init"
        out.child_frame_id = "body"
        out.pose.pose.position = msg.pose.pose.position
        speed = 0.0
        if not self.have_yaw:
            self.yaw = yaw_of(msg.pose.pose.orientation)
            self.have_yaw = True
        if self.previous is not None:
            dt = (msg.header.stamp - self.previous.header.stamp).to_sec()
            if dt > 1e-3:
                current = msg.pose.pose.position
                previous = self.previous.pose.pose.position
                vx = (current.x - previous.x) / dt
                vy = (current.y - previous.y) / dt
                speed = math.hypot(vx, vy)
                if speed > 0.8:
                    measured = math.atan2(vy, vx)
                    self.yaw = self.yaw + 0.35 * angle_delta(measured, self.yaw)
        half = 0.5 * self.yaw
        out.pose.pose.orientation = Quaternion(z=math.sin(half), w=math.cos(half))
        out.twist.twist.linear.x = speed
        self.previous = msg
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("glio_twist")
    GlioTwist()
    rospy.spin()
