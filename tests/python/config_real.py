from typing import List
import numpy as np
import os
os.environ["YARP_ROBOT_NAME"] = "ergoCubSN002"

import yarp
from clearconf import BaseConfig, Hidden


class Config(BaseConfig):
    # Names and frames
    urdf: str = yarp.ResourceFinder().findFileByName('model.urdf')
    right_joints: List[str] = [
        "r_shoulder_pitch",
        "r_shoulder_roll",
        "r_shoulder_yaw",
        "r_elbow",
        "r_wrist_yaw",
        "r_wrist_roll",
        "r_wrist_pitch",
    ]
    left_joints: List[str] = [
        "l_shoulder_pitch",
        "l_shoulder_roll",
        "l_shoulder_yaw",
        "l_elbow",
        "l_wrist_yaw",
        "l_wrist_roll",
        "l_wrist_pitch",
    ]
    torso_joints: List[str] = ["torso_roll", "torso_pitch", "torso_yaw"]

    right_root_frame: str = "root_link"
    right_ee_frame: str   = "r_hand_palm"
    left_root_frame: str  = "root_link"
    left_ee_frame: str    = "l_hand_palm"

    # Timing
    sampling_time: float = 0.01  # 100 Hz

    # Derived sizes
    no_right: Hidden | int = len(right_joints)
    no_left: Hidden | int = len(left_joints)
    no_torso: Hidden | int = len(torso_joints)
    no_joints:  Hidden | int  = no_right + no_left + no_torso

    # IK weights/gains
    joint_acc_weight = np.array([2.5, 2.5, 2.5], dtype=float)  # [right, left, torso]
    joint_pos_weights = np.array([*[0.01]*no_right, *[0.01]*no_left, *[0.01]*no_torso], dtype=float)
    joint_pos_kp = np.array([*[3.0]*no_right, *[3.0]*no_left, *[15.0]*no_torso], dtype=float)
    joint_pos_kd = np.array([*[4.5]*no_right, *[4.5]*no_left, *[22.5]*no_torso], dtype=float)

    cart_pos_weight = np.array([20.0, 20.0], dtype=float)
    cart_pos_kp     = np.array([100.0, 100.0], dtype=float)
    cart_pos_kd     = np.array([25.0, 25.0], dtype=float)

    cart_ori_weight = np.array([0.01, 0.01], dtype=float)
    cart_ori_kp     = np.array([100.0, 100.0], dtype=float)
    cart_ori_kd     = np.array([25.0, 25.0], dtype=float)

    improve_manip_weight: float = 0.001

    # Joint references/limits
    q_home  = np.zeros(no_joints, dtype=float)
    q_lower = -np.pi * np.ones(no_joints, dtype=float)
    q_upper =  np.pi * np.ones(no_joints, dtype=float)

    # Limits gain per chain [R,L,T]
    limit_gains_rlT = np.array([0.90, 0.90, 0.90], dtype=float)