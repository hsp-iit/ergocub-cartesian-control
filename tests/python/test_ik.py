#!/usr/bin/env python3
import numpy as np
import os
import yarp

# Import the module (ensure it’s in PYTHONPATH or installed)
from pysquale import BimanualIK, PoseInput
from config_real import Config as cfg


# --------- User configuration (edit these to match your robot) ---------
# Path to the URDF file
# install ergocub-software to get the URDF
os.environ["YARP_ROBOT_NAME"] = "ergoCubSN002"
urdf_path = yarp.ResourceFinder().findFileByName('model.urdf')

# Joint names for each chain (as they appear in the URDF)
right_joint_names = [
    "r_shoulder_pitch",
    "r_shoulder_roll",
    "r_shoulder_yaw",
    "r_elbow",
    "r_wrist_yaw",
    "r_wrist_roll",
    "r_wrist_pitch",
]
left_joint_names  = [
    "l_shoulder_pitch",
    "l_shoulder_roll",
    "l_shoulder_yaw",
    "l_elbow",
    "l_wrist_yaw",
    "l_wrist_roll",
    "l_wrist_pitch",
]
torso_joint_names = ["torso_roll", "torso_pitch", "torso_yaw"]

# Frame names in the URDF
right_root_frame = "root_link"
right_ee_frame   = "r_hand_palm"
left_root_frame  = "root_link"
left_ee_frame    = "l_hand_palm"

# Control parameters
dt = 0.01     # control sampling time [s]
steps = 1     # number of IK steps to run
# ----------------------------------------------------------------------


def np_array(values):
    """Helper: float numpy array."""
    return np.asarray(values, dtype=float)


def main():
    ik = BimanualIK(**cfg.to_dict())

    # Initial state (q, dq)
    q0  = np.zeros(cfg.no_joints)
    dq0 = np.zeros(cfg.no_joints)
    ik.reset(q0, dq0)

    right_pose = PoseInput()
    right_pose.pos  = np_array([0.05, 0.0, 0.0])
    right_pose.quat = np_array([0.0, 0.0, 0.0, 1.0])

    left_pose = PoseInput()
    left_pose.pos  = np_array([0.05, 0.0, 0.0])
    left_pose.quat = np_array([0.0, 0.0, 0.0, 1.0])

    q = ik.solve_ik(right_pose=right_pose, left_pose=left_pose)
    print(f"{q=}")


if __name__ == "__main__":
    main()
