#!/usr/bin/env python3
"""Play a walk cycle on the legs and arms. Speed comes from /odom."""
import math
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty


def _clamp(value, low, high):
    return max(low, min(high, value))


class GaitPlayer(object):
    def __init__(self):
        self.phase = 0.0
        self.blend = 0.0
        self.speed = 0.0
        self.last = None
        self.pub = rospy.Publisher("/gait_controller/command", Float64MultiArray, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self._odom, queue_size=1)

    def _odom(self, msg):
        self.speed = max(0.0, msg.twist.twist.linear.x)

    def _leg(self, phase, hip_amp, knee_amp):
        # Positive hip pitch swings the foot toward -X, so the forward step uses the opposite sign.
        hip = -hip_amp * math.sin(phase)
        knee = knee_amp * max(0.0, math.sin(phase))
        ankle = -0.55 * hip + 0.30 * knee
        return (
            _clamp(hip, -0.9, 0.9),
            _clamp(knee, 0.0, 1.7),
            _clamp(ankle, -0.8, 0.8),
        )

    def step(self):
        now = rospy.Time.now()
        if self.last is None:
            self.last = now
            dt = 0.0
        else:
            dt = (now - self.last).to_sec()
            self.last = now
        if dt < 0.0 or dt > 0.2:
            dt = 0.0
        moving = 1.0 if self.speed > 0.12 else 0.0
        self.blend += (moving - self.blend) * min(1.0, 4.0 * dt)
        hip_amp = 0.50 * self.blend
        knee_amp = 1.05 * self.blend
        frequency = min(2.2, self.speed / 0.65) * self.blend
        self.phase = (self.phase + 2.0 * math.pi * frequency * dt) % (2.0 * math.pi)
        hip_l, knee_l, ankle_l = self._leg(self.phase, hip_amp, knee_amp)
        hip_r, knee_r, ankle_r = self._leg(self.phase + math.pi, hip_amp, knee_amp)
        shoulder_l = _clamp(-0.70 * hip_l, -1.2, 1.2)
        shoulder_r = _clamp(-0.70 * hip_r, -1.2, 1.2)
        msg = Float64MultiArray()
        msg.data = [
            hip_l, knee_l, ankle_l,
            hip_r, knee_r, ankle_r,
            shoulder_l, shoulder_r,
        ]
        self.pub.publish(msg)


def _wait_for_controller():
    deadline = time.time() + 60.0
    pub = rospy.Publisher("/gait_controller/command", Float64MultiArray, queue_size=1)
    while pub.get_num_connections() == 0 and time.time() < deadline and not rospy.is_shutdown():
        time.sleep(0.1)
    if pub.get_num_connections() == 0:
        rospy.logwarn("gait controller has no subscriber")
    pub.unregister()
    rospy.wait_for_service("/gazebo/unpause_physics", timeout=30.0)
    rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()


if __name__ == "__main__":
    rospy.init_node("gait_player")
    _wait_for_controller()
    player = GaitPlayer()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        player.step()
        rate.sleep()
