#!/usr/bin/env python3
"""Move roadside pedestrians along the course."""
import math
import os
import sys

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Quaternion

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import course


def yaw_quat(yaw):
    return Quaternion(z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


class SidewalkTraffic(object):
    def __init__(self):
        self.points = course.centerline(step=1.0)
        self.length = course.course_length(self.points)
        self.walkers = (
            ("walker_0", 40.0, course.SIDEWALK, 1.3),
            ("walker_1", 180.0, course.SIDEWALK, 1.1),
        )
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.callback)

    def place(self, name, along, lateral, yaw_extra=0.0):
        along = along % self.length
        x, y, yaw = course.pose_at_distance(self.points, along)
        px = x + lateral * -math.sin(yaw)
        py = y + lateral * math.cos(yaw)
        state = ModelState()
        state.model_name = name
        state.pose = Pose()
        state.pose.position.x = px
        state.pose.position.y = py
        state.pose.position.z = course.ground_z(px) + 0.85
        state.pose.orientation = yaw_quat(yaw + yaw_extra)
        state.reference_frame = "world"
        try:
            self.set_state(state)
        except rospy.ServiceException:
            pass

    def callback(self, event):
        t = rospy.Time.now().to_sec()
        for name, start, lateral, speed in self.walkers:
            self.place(name, start + speed * t, lateral)


if __name__ == "__main__":
    rospy.init_node("sidewalk_traffic")
    SidewalkTraffic()
    rospy.spin()
