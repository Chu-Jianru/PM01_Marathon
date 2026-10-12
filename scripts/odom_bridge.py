#!/usr/bin/env python3
"""Republish /odom as /odometry. Twist is expressed in the pose frame."""
import math

import rospy
from nav_msgs.msg import Odometry


def quat_rotate(qx, qy, qz, qw, vx, vy, vz):
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


class OdomBridge(object):
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "camera_init")
        self.child_frame_id = rospy.get_param("~child_frame_id", "body")
        self.pubs = [
            rospy.Publisher("/odometry", Odometry, queue_size=10),
        ]
        rospy.Subscriber("/odom", Odometry, self.callback, queue_size=10)

    def callback(self, msg):
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.frame_id
        out.child_frame_id = self.child_frame_id
        out.pose = msg.pose
        q = msg.pose.pose.orientation
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular
        lx, ly, lz = quat_rotate(q.x, q.y, q.z, q.w, lin.x, lin.y, lin.z)
        ax, ay, az = quat_rotate(q.x, q.y, q.z, q.w, ang.x, ang.y, ang.z)
        out.twist.twist.linear.x = lx
        out.twist.twist.linear.y = ly
        out.twist.twist.linear.z = lz
        out.twist.twist.angular.x = ax
        out.twist.twist.angular.y = ay
        out.twist.twist.angular.z = az
        for pub in self.pubs:
            pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("odom_bridge")
    OdomBridge()
    rospy.spin()
