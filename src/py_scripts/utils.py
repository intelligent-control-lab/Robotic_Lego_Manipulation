import rospy
import numpy as np
import time
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from sklearn.decomposition import PCA
import cv2
import copy