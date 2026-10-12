#!/usr/bin/env python3
"""Open the Gazebo window after the leg controllers are running.

Opening gzclient while the model is still being inserted stalls the physics
clock at a fraction of a second, so the robot never leaves the start."""
import os
import sys
import time

import rospy
from controller_manager_msgs.srv import ListControllers


def main():
    rospy.init_node("open_gazebo_gui", anonymous=True, disable_rostime=True)
    rospy.wait_for_service("/controller_manager/list_controllers", timeout=120.0)
    proxy = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
    deadline = time.time() + 120.0
    while time.time() < deadline and not rospy.is_shutdown():
        states = {item.name: item.state for item in proxy().controller}
        if (states.get("joint_state_controller") == "running"
                and states.get("gait_controller") == "running"):
            break
        time.sleep(0.3)
    else:
        rospy.logerr("腿部控制器没有启动，Gazebo 窗口不打开。")
        raise SystemExit(1)
    os.execvp(sys.argv[1], sys.argv[1:])


if __name__ == "__main__":
    main()
