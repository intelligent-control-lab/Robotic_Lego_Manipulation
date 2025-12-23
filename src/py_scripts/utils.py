import numpy as np
import json
import rospy
from std_msgs.msg import Float32MultiArray, Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from scipy.spatial.transform import Rotation as R
from message_filters import ApproximateTimeSynchronizer, Subscriber
import pinocchio as pin
import time
import cv2
from collections import deque
from scipy.optimize import least_squares


def load_json(fname):
    with open(fname, 'r') as file:
        data = json.load(file)
    return data

def rot_x(q, deg=True):
    if(deg):
        q = q / 180 * np.pi
    rot_mtx = np.array([[1, 0, 0, 0],
                        [0, np.cos(q), -np.sin(q), 0],
                        [0, np.sin(q), np.cos(q), 0],
                        [0, 0, 0, 1]])
    return rot_mtx

def rot_y(q, deg=True):
    if(deg):
        q = q / 180 * np.pi
    rot_mtx = np.array([[np.cos(q), 0, np.sin(q), 0],
                        [0, 1, 0, 0]
                        [-np.sin(q), 0, np.cos(q), 0],
                        [0, 0, 0, 1]])
    return rot_mtx

def rot_z(q, deg=True):
    if(deg):
        q = q / 180 * np.pi
    rot_mtx = np.array([[np.cos(q), -np.sin(q), 0, 0],
                        [np.sin(q), np.cos(q), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
    return rot_mtx

def brick_T_on_plate(brick_name, x_on_plate, y_on_plate, z_on_plate, ori, plate_T, plate_dim, lego_lib, 
                     brick_tall=0.0096, P_len=0.008):
    brick_id = brick_name[1:].split('_')[0]
    brick_height, brick_width = lego_lib[brick_id]["height"], lego_lib[brick_id]["width"]
    ref_pose = plate_T
    topleft_offset = np.array([[1, 0, 0, -(plate_dim[0] * P_len) / 2.0],
                                  [0, 1, 0, -(plate_dim[1] * P_len) / 2.0],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1]])
    brick_offset = np.array([[1, 0, 0, x_on_plate * P_len],
                              [0, 1, 0, y_on_plate * P_len],
                              [0, 0, 1, z_on_plate * brick_tall],
                              [0, 0, 0, 1]])
    brick_center_offset = np.array([[1, 0, 0, (brick_height * P_len) / 2.0],
                                     [0, 1, 0, (brick_width * P_len) / 2.0],
                                     [0, 0, 1, 0],
                                     [0, 0, 0, 1]])
    
    brick_pose = ref_pose @ topleft_offset @ brick_offset @ brick_center_offset
    if(ori == 1):
        brick_center_offset[1, ] = -brick_center_offset[1, 3]
        brick_pose = ref_pose @ topleft_offset @ brick_offset @ rot_z(90) @ brick_center_offset
    return brick_pose

def attach_brick_T(brick_name, brick_T, press_side, press_offset, approach_dir, lego_lib, P_len=0.008):
    brick_id = brick_name[1:].split('_')[0]
    brick_height, brick_width = lego_lib[brick_id]["height"], lego_lib[brick_id]["width"]
    ref_pose = brick_T
    offset_T = np.eye(4)
    press_offset += 1
    if(press_side == 1):
        offset_T[0, 3] = -brick_height * P_len / 2
        offset_T[1, 3] = -brick_width * P_len / 2 + press_offset * P_len
    elif(press_side == 2):
        offset_T = offset_T @ rot_z(-90)
        offset_T = offset_T @ np.array([[1, 0, 0, -brick_width * P_len / 2],
                                        [0, 1, 0, -brick_height * P_len / 2 + press_offset * P_len],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]])
    elif(press_side == 3):
        offset_T = offset_T @ rot_z(90)
        offset_T = offset_T @ np.array([[1, 0, 0, -brick_width * P_len / 2],
                                        [0, 1, 0, brick_height * P_len / 2 - press_offset * P_len],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]])
    elif(press_side == 4):
        offset_T = offset_T @ rot_z(180)
        offset_T = offset_T @ np.array([[1, 0, 0, -brick_height * P_len / 2],
                                        [0, 1, 0, brick_width * P_len / 2 - press_offset * P_len],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]])
    else:
        raise TypeError("Unknown press_side!")
    attach_T = ref_pose @ offset_T
    if(approach_dir == -1):
        pre_pre_attach_T = attach_T @ np.array([[1, 0, 0, -P_len / 2],
                                            [0, 1, 0, -P_len / 2],
                                            [0, 0, 1, P_len / 2],
                                            [0, 0, 0, 1]])
    elif(approach_dir == 1):
        pre_pre_attach_T = attach_T @ np.array([[1, 0, 0, -P_len / 2],
                                            [0, 1, 0, P_len / 2],
                                            [0, 0, 1, P_len / 2],
                                            [0, 0, 0, 1]])
    else:
        raise TypeError("Unknown approach_dir!")
    pre_attach_T = attach_T @ np.array([[1, 0, 0, 0],
                                        [0, 1, 0, 0],
                                        [0, 0, 1, 0.0025],
                                        [0, 0, 0, 1]])

    return attach_T, pre_attach_T, pre_pre_attach_T


def collision_free_env(brick_name, brick_T, objects_T, storage_plate_dim, assemble_plate_dim, lego_lib, eps=0.02, P_len=0.008):
    brick_id = brick_name[1:].split('_')[0]
    brick_h, brick_width = lego_lib[brick_id]["height"], lego_lib[brick_id]["width"]
    brick_radius = 0.5 * np.sqrt((brick_h * P_len)**2 + (brick_width * P_len)**2)
    for k in objects_T.keys():
        other_T = objects_T[k]
        if("storage_plate" in k):
            other_radius = 0.5 * np.sqrt((storage_plate_dim[0] * P_len)**2 + (storage_plate_dim[1] * P_len)**2)
        elif("assemble_plate" in k):
            other_radius = 0.5 * np.sqrt((assemble_plate_dim[0] * P_len)**2 + (assemble_plate_dim[1] * P_len)**2)
        else:
            other_id = k[1:].split('_')[0]
            other_h, other_width = lego_lib[other_id]["height"], lego_lib[other_id]["width"]
            other_radius = 0.5 * np.sqrt((other_h * P_len)**2 + (other_width * P_len)**2)
        dist = np.linalg.norm(brick_T[:3, 3] - other_T[:3, 3])
        if(dist < (brick_radius + other_radius + eps)):
            return False
    return True