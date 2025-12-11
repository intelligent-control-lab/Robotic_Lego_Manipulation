import numpy as np
import json
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from scipy.spatial.transform import Rotation as R

def load_json(fname):
    with open(fname, 'r') as file:
        data = json.load(file)
    return data