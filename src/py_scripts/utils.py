import numpy as np
import json
import rospy
from std_msgs.msg import Float32MultiArray, Float64
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from scipy.spatial.transform import Rotation as R
import pinocchio as pin
import time

def load_json(fname):
    with open(fname, 'r') as file:
        data = json.load(file)
    return data

def calc_brick_T(brick_name, x_on_plate, y_on_plate, z_on_plate, ori, plate_T, plate_dim, lego_lib, brick_tall=0.0096, P_len=0.008):
    brick_id = brick_name[1:].split('_')[0]
    brick_h, brick_width = lego_lib[brick_id]["height"], lego_lib[brick_id]["width"]
    ref_pose = plate_T
    topleft_offset = np.array([[1, 0, 0, -(plate_dim[0] * P_len) / 2.0],
                                  [0, 1, 0, -(plate_dim[1] * P_len) / 2.0],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1]])
    brick_offset = np.array([[1, 0, 0, x_on_plate * P_len],
                              [0, 1, 0, y_on_plate * P_len],
                              [0, 0, 1, z_on_plate * brick_tall],
                              [0, 0, 0, 1]])
    brick_center_offset = np.array([[1, 0, 0, (brick_h * P_len) / 2.0],
                                     [0, 1, 0, (brick_width * P_len) / 2.0],
                                     [0, 0, 1, 0],
                                     [0, 0, 0, 1]])
    z_90 = np.array([[0, -1, 0, 0],
                     [1, 0, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])
    
    brick_pose = ref_pose @ topleft_offset @ brick_offset @ brick_center_offset
    if(ori == 1):
        brick_center_offset[1, ] = -brick_center_offset[1, 3]
        brick_pose = ref_pose @ topleft_offset @ brick_offset @ z_90 @ brick_center_offset
    return brick_pose


def collision_free_env(brick_name, brick_T, objects_T, lego_lib, eps=0.02, P_len=0.008):
    brick_id = brick_name[1:].split('_')[0]
    brick_h, brick_width = lego_lib[brick_id]["height"], lego_lib[brick_id]["width"]
    brick_radius = 0.5 * np.sqrt((brick_h * P_len)**2 + (brick_width * P_len)**2)
    for k in objects_T.keys():
        other_T = objects_T[k]
        if("plate" in k):
            other_radius = 0.5 * np.sqrt((48 * P_len)**2 + (48 * P_len)**2)
        else:
            other_id = k[1:].split('_')[0]
            other_h, other_width = lego_lib[other_id]["height"], lego_lib[other_id]["width"]
            other_radius = 0.5 * np.sqrt((other_h * P_len)**2 + (other_width * P_len)**2)
        dist = np.linalg.norm(brick_T[:3, 3] - other_T[:3, 3])
        if(dist < (brick_radius + other_radius + eps)):
            return False
    return True