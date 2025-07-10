# code by LinCC111 Boxjod 2025.1.13 Box2AI-Robotics copyright 盒桥智能 版权所有

import os
import numpy as np
import time
import math
import sys
sys.path.append('/root/soarm/lerobot-kinematics-main')
from lerobot_kinematics import lerobot_IK, lerobot_FK, get_robot, feetech_arm
# For Feetech Motors
from lerobot_kinematics.lerobot.feetech import FeetechMotorsBus
import json

np.set_printoptions(linewidth=200)

# Set up the MuJoCo render backend
os.environ["MUJOCO_GL"] = "egl"

# Define joint names
JOINT_NAMES = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]

robot = get_robot('so100')

# Define joint control increment (in radians)
JOINT_INCREMENT = 0.005
POSITION_INSERMENT = 0.0008

# Define joint limits
control_qlimit = [[-1.5, -3.1, -0.0, -1.375,  -1.57, -0.15], 
                  [ 1.5,  0.0,  3.1,  1.475,   3.1,  1.5]]
control_glimit = [[0.125, -0.4,  0.046, -3.1, -0.75, -1.5], 
                  [0.340,  0.4,  0.23, 2.0,  1.57,  1.5]]

# Initialize target joint positions
init_qpos = np.array([0.0, -3.14, 3.14, 0.0, -1.57, 1.5])
target_qpos = init_qpos.copy()
init_gpos = lerobot_FK(init_qpos[1:5],robot=robot)
target_gpos = init_gpos.copy()
print(init_gpos)
target_gpos_last = init_gpos.copy()
target_qpos_last = init_qpos.copy()
# Connect to the robotic arm motors
motors = {"shoulder_pan": (1, "sts3215"),
          "shoulder_lift": (2, "sts3215"),
          "elbow_flex": (3, "sts3215"),
          "wrist_flex": (4, "sts3215"),
          "wrist_roll": (5, "sts3215"),
          "gripper": (6, "sts3215")}

follower_arm = feetech_arm(driver_port="/dev/ttyACM0", calibration_file="examples/main_follower.json" )
target_gpos=[ 0.2071 , 0   ,      0.13, -1.57    ,   1.4  ,       0    ]
target_y= 0.0  # Initialize target yaw angle
target_x= 0.20
t = 0
try:
    # Headless MuJoCo loop (no viewer)
    start = time.time()
    
    while time.time() - start < 1000:
        t += 1
        step_start = time.time()

        # Control logic (replace this section with your desired control algorithm)
        # Add more control logic as needed
        # Joint limit enforcement
        
        # for i in range(len(target_gpos)):
        #     if target_gpos[i] < control_glimit[0][i]:
        #         target_gpos[i] = control_glimit[0][i]
        #     if target_gpos[i] > control_glimit[1][i]:
        #         target_gpos[i] = control_glimit[1][i]
        # print("target_gpos:", [f"{x:.3f}" for x in target_gpos])
        # fd_qpos = np.concatenate(([0.0,], mjdata.qpos[qpos_indices][1:5]))
        target_gpos[0]=math.sqrt(target_x**2 + target_y**2)  # Update target_gpos[0] based on target_x and target_y
        fd_qpos = follower_arm.feedback()[1:5]
        # compute yaw angle from y/x using atan2
        target_qpos[0] =math.asin(target_y / target_gpos[0])
        
        # print(math.asin(target_y / target_gpos[0]))
        qpos_inv, IK_success = lerobot_IK(fd_qpos, target_gpos, robot=robot)
        # target_qpos[0] = math.atan2(target_y, target_gpos[0])
        # print("target_qpos[0]:", target_qpos[0])
        if np.all(qpos_inv != -1.0):  # Check if IK solution is valid
            target_qpos = np.concatenate((target_qpos[0:1], qpos_inv[:4], [1]))
            # target_qpos = np.concatenate((qpos_inv, target_qpos[5:]))
            # print("target_qpos:", [f"{x:.3f}" for x in target_qpos])
            # apply and step headlessly
            # mjdata.qpos[qpos_indices] = target_qpos
            # print("target_qpos:", [f"{x:.3f}" for x in target_qpos])
            # print("fd_qpos:",follower_arm.feedback())
            # mujoco.mj_step(mjmodel, mjdata)
            for i in range(len(target_qpos)):
                if target_qpos[i] < control_qlimit[0][i]:
                    target_qpos[i] = control_qlimit[0][i]
                elif target_qpos[i] > control_qlimit[1][i]:
                    target_qpos[i] = control_qlimit[1][i]
            follower_arm.action(target_qpos)
            target_gpos_last = target_gpos.copy()
        else:
            target_gpos = target_gpos_last.copy()

except KeyboardInterrupt:
    print("User interrupted the simulation.")
finally:
    follower_arm.disconnect()
