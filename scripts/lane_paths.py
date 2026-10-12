#!/usr/bin/env python3
"""Publish the marathon center line and both lane boundaries.

These paths stand in for the camera segmentation output. They follow the
same bends as the Gazebo road.
"""
import math
import os
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import course


def make_path(frame, samples):
    path = Path()
    path.header.frame_id = frame
    for x, y, yaw in samples:
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        path.poses.append(pose)
    return path


def main():
    rospy.init_node("lane_paths")
    frame = rospy.get_param("~frame_id", "camera_init")
    half_width = float(rospy.get_param("~half_width", course.HALF_WIDTH))
    center_samples = course.centerline()
    paths = {
        "/central/smoothed_path": center_samples,
        "/left/smoothed_path": course.offset_path(center_samples, half_width),
        "/right/smoothed_path": course.offset_path(center_samples, -half_width),
    }
    publishers = {
        topic: rospy.Publisher(topic, Path, queue_size=1, latch=True)
        for topic in paths
    }
    rospy.loginfo("marathon course length %.1f m", course.course_length(center_samples))
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        stamp = rospy.Time.now()
        for topic, samples in paths.items():
            path = make_path(frame, samples)
            path.header.stamp = stamp
            for pose in path.poses:
                pose.header.stamp = stamp
            publishers[topic].publish(path)
        rate.sleep()


if __name__ == "__main__":
    main()
