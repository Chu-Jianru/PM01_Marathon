#!/usr/bin/env python3
"""Publish PM01 leg poses from the zqsa01 walking policy."""
import math
import os
import sys

import numpy as np
import yaml

POLICY_JOINTS = [
    "leg_l1_joint", "leg_l2_joint", "leg_l3_joint",
    "leg_l4_joint", "leg_l5_joint", "leg_l6_joint",
    "leg_r1_joint", "leg_r2_joint", "leg_r3_joint",
    "leg_r4_joint", "leg_r5_joint", "leg_r6_joint",
]
# Policy order mapped onto the education-edition leg chain.
PM01_JOINTS = [
    "J01_HIP_ROLL_L", "J02_HIP_YAW_L", "J00_HIP_PITCH_L",
    "J03_KNEE_PITCH_L", "J04_ANKLE_PITCH_L", "J05_ANKLE_ROLL_L",
    "J07_HIP_ROLL_R", "J08_HIP_YAW_R", "J06_HIP_PITCH_R",
    "J09_KNEE_PITCH_R", "J10_ANKLE_PITCH_R", "J11_ANKLE_ROLL_R",
]
LIMITS = np.array([
    [-0.436, 2.094],
    [-1.57, 4.014],
    [-3.141, 2.443],
    [-0.3491, 2.3911],
    [-0.6807, 0.7243],
    [-0.2618, 0.2618],
    [-2.094, 0.436],
    [-4.014, 1.57],
    [-3.141, 2.443],
    [-0.3491, 2.3911],
    [-0.6807, 0.7243],
    [-0.2618, 0.2618],
], dtype=np.float64)


class PolicyGait(object):
    def __init__(self, policy_path, config_path, signs):
        import onnxruntime as ort

        cfg = yaml.safe_load(open(config_path))["LeggedRobotRLConfig"]
        defaults = cfg["init_state"]["default_joint_angle"]
        self.default = np.array([float(defaults[name]) for name in POLICY_JOINTS], dtype=np.float64)
        self.action_scale = float(cfg["control"]["action_scale"])
        self.period = float(cfg["control"]["gait_period"])
        self.dof_vel_scale = float(cfg["normalization"]["obs_scales"]["dof_vel"])
        self.clip_obs = float(cfg["normalization"]["clip_scales"]["clip_observations"])
        self.clip_act = float(cfg["normalization"]["clip_scales"]["clip_actions"])
        self.frame = int(cfg["size"]["observations_size"])
        self.stack = int(cfg["size"]["observations_frame_size"])
        self.signs = np.array(signs, dtype=np.float64)
        if self.signs.shape != (12,):
            raise RuntimeError("signs must contain 12 values")
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        self.session = ort.InferenceSession(
            policy_path, sess_options=options, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.input_rank = len(self.session.get_inputs()[0].shape)
        self.history = None
        self.last_action = np.zeros(12, dtype=np.float64)
        self.q = self.default.copy()
        self.prev_q = self.default.copy()
        self.qdot = np.zeros(12, dtype=np.float64)
        self.phase = 0.0
        self.started = False

    def step(self, dt, vx, vy, yaw, standing):
        if dt <= 0.0 or dt > 0.2:
            dt = 0.01
        self.phase += dt
        if standing:
            command = np.array([0.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        else:
            cycle = self.phase / self.period
            command = np.array([
                math.sin(2.0 * math.pi * cycle),
                math.cos(2.0 * math.pi * cycle),
                vx, vy, yaw,
            ], dtype=np.float64)
        # Motor velocity is slower than a one-step target jump, so low-pass the difference.
        raw_velocity = (self.q - self.prev_q) / dt
        blend = min(1.0, dt / 0.05)
        self.qdot += (raw_velocity - self.qdot) * blend
        velocity = self.qdot
        obs = np.concatenate([
            command,
            (self.q - self.default),
            velocity * self.dof_vel_scale,
            self.last_action,
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
        ])
        if obs.shape[0] != self.frame:
            raise RuntimeError("observation length is %d" % obs.shape[0])
        if not self.started:
            obs = obs.copy()
            obs[29:41] = 0.0
            self.history = np.tile(obs, self.stack)
            self.started = True
        self.history = np.concatenate([self.history[self.frame:], obs])
        flat = np.clip(self.history, -self.clip_obs, self.clip_obs).astype(np.float32)
        feed = flat.reshape(1, -1) if self.input_rank >= 2 else flat
        action = np.array(self.session.run(
            [self.output_name], {self.input_name: feed})[0]).reshape(-1)[:12]
        action = np.clip(action, -self.clip_act, self.clip_act)
        self.last_action = action.astype(np.float64)
        self.prev_q = self.q.copy()
        q = self.last_action * self.action_scale + self.default
        self.q = np.clip(q, LIMITS[:, 0], LIMITS[:, 1])
        # Roll, yaw and ankle roll swing the thigh sideways on this kinematic
        # chain. Keep those at the standing angle and play the forward stride.
        shown = self.q.copy()
        shown[[0, 1, 5, 6, 7, 11]] = self.default[[0, 1, 5, 6, 7, 11]]
        return np.clip(self.signs * shown, LIMITS[:, 0], LIMITS[:, 1])


def _clip(value, limit):
    return max(-limit, min(limit, value))


def run_offline(policy_path, config_path):
    gait = PolicyGait(policy_path, config_path, np.ones(12))
    stand = None
    for _ in range(50):
        stand = gait.step(0.01, 0.0, 0.0, 0.0, True)
    samples = [gait.step(0.01, 1.0, 0.0, 0.0, False) for _ in range(200)]
    series = np.vstack(samples)
    pitch_corr = float(np.corrcoef(series[:, 2], series[:, 8])[0, 1])
    print("stand " + " ".join("%.3f" % value for value in stand))
    for index, name in enumerate(PM01_JOINTS):
        column = series[:, index]
        print("%s min %.3f max %.3f mean %.3f" % (name, column.min(), column.max(), column.mean()))
    print("hip_pitch_corr %.3f" % pitch_corr)
    print("knee_left_mean %.3f knee_right_mean %.3f" % (series[:, 3].mean(), series[:, 9].mean()))
    return 0


def main():
    policy_default = os.path.expanduser("~/engineai_policy/zqsa01_policy.onnx")
    config_default = os.path.expanduser("~/engineai_policy/zqsa01_rl.yaml")
    if "--offline" in sys.argv:
        return run_offline(policy_default, config_default)

    import rospy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float64MultiArray

    rospy.init_node("policy_gait")
    policy_path = os.path.expanduser(rospy.get_param("~policy_path", policy_default))
    config_path = os.path.expanduser(rospy.get_param("~config_path", config_default))
    signs = [float(value) for value in rospy.get_param("~signs", [1.0] * 12)]
    stand_speed = float(rospy.get_param("~stand_speed", 0.12))
    gait = PolicyGait(policy_path, config_path, signs)
    state = {"vx": 0.0, "vy": 0.0, "yaw": 0.0}

    def _odom(msg):
        state["vx"] = msg.twist.twist.linear.x
        state["vy"] = msg.twist.twist.linear.y
        state["yaw"] = msg.twist.twist.angular.z

    pub = rospy.Publisher("/gait_controller/command", Float64MultiArray, queue_size=1)
    rospy.Subscriber("/odom", Odometry, _odom, queue_size=1)
    rate = rospy.Rate(100)
    last = None
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        dt = 0.01 if last is None else (now - last).to_sec()
        last = now
        standing = (
            abs(state["vx"]) < stand_speed
            and abs(state["vy"]) < 0.10
            and abs(state["yaw"]) < 0.15
        )
        command = gait.step(
            dt,
            _clip(state["vx"], 1.0),
            _clip(state["vy"], 0.7),
            _clip(state["yaw"], 0.8),
            standing,
        )
        msg = Float64MultiArray()
        msg.data = [float(value) for value in command]
        pub.publish(msg)
        rate.sleep()
    return 0


if __name__ == "__main__":
    sys.exit(main())
