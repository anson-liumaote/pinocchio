import numpy as np
import math
import datetime

WHEEL_RADIUS = 0.08     # m
WHEEL_MASS = 0.695  # kg
WHEEL_DISTANCE = 0.355
URDF_PATH = "/home/crazydog/crazydog/crazydog_ws/src/lqr_control/lqr_control/robot_models/big bipedal robot v1/urdf/big bipedal robot v1.urdf"
MID_ANGLE = 0.045
TORQUE_CONSTRAIN = 1.5
MOTOR_INIT_POS = [None, 0.669, 3.815, None, 1.247+2*math.pi, 2.970]     # for unitree motors
INIT_ANGLE = [-2.42, 2.6]
LOCK_POS = [None, 2.76, 9.88, None, 5.44, -3.10]    # -2.75, 2.0
LOCK_COOR = [0.032606-0.0742, -0.228576]     # lock legs coordinate [x, y]
THIGH_LENGTH = 0.215
CALF_LENGTH = 0.215

def inverse_kinematics(x, y, L1=THIGH_LENGTH, L2=CALF_LENGTH):
        # 計算 d    
        d = np.sqrt(x**2 + y**2)
        # 計算 theta2
        cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
        theta2 = np.arccos(cos_theta2)
        
        # 計算 theta1
        theta1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))
    
        return theta1, theta2
    
def change_angle(x, y):
    theta1, theta2 = inverse_kinematics(x, y)
    print(theta1, theta2)
    theta1_err = theta1 - INIT_ANGLE[0]
    theta2_err = theta2 - INIT_ANGLE[1]
    print(f'theta1_err:{theta1_err}, theta2_err{theta2_err}')

change_angle(LOCK_COOR[0], LOCK_COOR[1])

print()