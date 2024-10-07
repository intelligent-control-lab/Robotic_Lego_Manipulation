from utils import *
import open3d as o3d

class Robot():
    def __init__(self, robot_properties_dir):
        self.dof = 6
        self.robot_state = np.zeros(self.dof)
        self.robot_state_v = np.zeros(self.dof)
        self.robot_state_a = np.zeros(self.dof)
        self.robot_goal_pub = rospy.Publisher('/stmotion_controller_bringup/robot_goal', Float32MultiArray, queue_size=6)
        self.robot_state_sub = rospy.Subscriber("/stmotion_controller_bringup/robot_state", Float32MultiArray, self.robot_state_callback)
        self.robot_travel_time_pub = rospy.Publisher('/stmotion_controller_bringup/jpc_travel_time', Float64, queue_size=1)
        self.depth_sub = rospy.Subscriber("/camera/wrist_camera/depth/image_raw", Image, self.depth_callback)
        self.color_sub = rospy.Subscriber("/camera/wrist_camera/image_raw", Image, self.color_callback)
        self.cam_mtx_sub = rospy.Subscriber("/camera/wrist_camera/camera_info", CameraInfo, self.cam_mtx_callback)
        self.cam_mtx = None
        self.depth_img = None
        self.color_img = None
        self.robot2cam = np.matrix([[1, 0, 0, 0.5],
                                    [0, 1, 0, 0.1],
                                    [0, 0, 1, 1],
                                    [0, 0, 0, 1]])
        rz = -90 / 180 * np.pi
        ry = 90 / 180 * np.pi
        self.robot2plane = self.robot2cam @ np.matrix([[np.cos(ry), 0, np.sin(ry), 0],
                                                        [0, 1, 0, 0],
                                                        [-np.sin(ry), 0, np.cos(ry), 0],
                                                        [0, 0, 0, 1]]) @ np.matrix([[1, 0, 0, 0.013],
                                                        [0, 1, 0, 0],
                                                        [0, 0, 1, 0.0305],
                                                        [0, 0, 0, 1]]) @ np.matrix([[np.cos(ry), 0, np.sin(ry), 0],
                                                        [0, 1, 0, 0],
                                                        [-np.sin(ry), 0, np.cos(ry), 0],
                                                        [0, 0, 0, 1]]) @ np.matrix([[np.cos(rz), -np.sin(rz), 0, 0],
                                                        [np.sin(rz), np.cos(rz), 0, 0],
                                                        [0, 0, 1, 0],
                                                        [0, 0, 0, 1]])

        self.robot_goal = np.zeros(self.dof)
        self.robot_goal_msg = Float32MultiArray()
        self.robot_travel_time = Float64()

        self.base = np.loadtxt(robot_properties_dir + "FANUC_base.txt")
        self.base = np.matrix([[1, 0, 0, self.base[0]],
                               [0, 1, 0, self.base[1]],
                               [0, 0, 1, self.base[2]],
                               [0, 0, 0, 1]])
        self.DH = np.loadtxt(robot_properties_dir + "FANUC_DH.txt")
        self.ee_inv = np.linalg.inv(np.matrix([[np.cos(0), -np.sin(0) * np.cos(0), np.sin(0) * np.sin(0), self.DH[5, 2] * np.cos(0)],
                                               [np.sin(0), np.cos(0) * np.cos(0), -np.cos(0) * np.sin(0), self.DH[5, 2] * np.sin(0)],
                                               [0, np.sin(0), np.cos(0), -self.DH[5, 1]],
                                               [0, 0, 0, 1]]))
        self.base_inv = np.linalg.inv(self.base)
        self.theta_max = np.matrix([[-120, 120],
                                    [-120, 90],
                                    [-90, 90],
                                    [-180, 180],
                                    [-120, 120],
                                    [-360, 360]], dtype=np.float32)
        self.theta_max_rad = np.copy(self.theta_max) * np.pi / 180

        # Gripper
        gripper_controller_topic = ["/fanuc_gazebo/joint0_finger_A_position_controller/command",
                                    "/fanuc_gazebo/joint1_finger_A_position_controller/command",
                                    "/fanuc_gazebo/joint2_finger_A_position_controller/command",
                                    "/fanuc_gazebo/joint3_finger_A_position_controller/command",
                                    "/fanuc_gazebo/joint0_finger_B_position_controller/command",
                                    "/fanuc_gazebo/joint1_finger_B_position_controller/command",
                                    "/fanuc_gazebo/joint2_finger_B_position_controller/command",
                                    "/fanuc_gazebo/joint3_finger_B_position_controller/command",
                                    "/fanuc_gazebo/joint0_finger_C_position_controller/command",
                                    "/fanuc_gazebo/joint1_finger_C_position_controller/command",
                                    "/fanuc_gazebo/joint2_finger_C_position_controller/command",
                                    "/fanuc_gazebo/joint3_finger_C_position_controller/command",]
        self.gripper = Gripper(gripper_controller_topic)

    def depth_callback(self, data):
        try:
            bridge = CvBridge()
            depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            self.depth_img = depth_image
        except CvBridgeError as e:
            pass

    def color_callback(self, data):
        try:
            bridge = CvBridge()
            color_image = bridge.imgmsg_to_cv2(data, "bgr8")
            self.color_img = color_image
        except CvBridgeError as e:
            pass
    
    def cam_mtx_callback(self, data):
        self.cam_mtx = np.reshape(np.asarray(data.K), (3, 3))
    
    def color_img(self):
        return self.color_img
    
    def depth_img(self):
        return self.depth_img
    
    def robot_state_callback(self, data):
        for i in range(self.dof):
            self.robot_state[i] = data.data[i*3]
            self.robot_state_v[i] = data.data[i*3 + 1]
            self.robot_state_a[i] = data.data[i*3 + 2]
    
    def set_robot_travel_time(self, t):
        self.robot_travel_time.data = t
        self.robot_travel_time_pub.publish(self.robot_travel_time)

    def drive_robot(self, goal):
        for i in range(self.dof):
            self.robot_goal[i] = goal[i]
        self.robot_goal_msg.data = goal
        self.robot_goal_pub.publish(self.robot_goal_msg)

    def goal_reached(self):
        for i in range(self.dof):
            if(abs(self.robot_state[i] - self.robot_goal[i]) > 0.01 or abs(self.robot_state_v[i]) > 0.001 or abs(self.robot_state_a[i] > 0.001)):
                return False
        return True
    
    def FK(self, q, joint_rad=0):
        trans_mtx = np.copy(self.base)
        DH_cur = np.copy(self.DH)
        q_rad = np.asarray(q, dtype=np.float32)
        if(joint_rad == 0):
            for i in range(q_rad.shape[0]):
                q_rad[i] *= np.pi / 180
        DH_cur[:, 0] += q_rad
        for i in range(DH_cur.shape[0]):
            tmp = np.matrix([[np.cos(DH_cur[i, 0]), -np.sin(DH_cur[i, 0]) * np.cos(DH_cur[i, 3]), np.sin(DH_cur[i, 0]) * np.sin(DH_cur[i, 3]), DH_cur[i, 2] * np.cos(DH_cur[i, 0])],
                             [np.sin(DH_cur[i, 0]), np.cos(DH_cur[i, 0]) * np.cos(DH_cur[i, 3]), -np.cos(DH_cur[i, 0]) * np.sin(DH_cur[i, 3]), DH_cur[i, 2] * np.sin(DH_cur[i, 0])],
                             [0, np.sin(DH_cur[i, 3]), np.cos(DH_cur[i, 3]), DH_cur[i, 1]],
                             [0, 0, 0, 1]])
            trans_mtx = np.matmul(trans_mtx, tmp)
        return trans_mtx
    
    def joint_in_range(self, theta, is_rad=0):
        theta_deg = np.asarray(theta, dtype=np.float32)
        if(is_rad):
            theta_deg = theta_deg / np.pi * 180
        for i in range(theta.shape[0]):
            if(theta_deg[i] < self.theta_max[i, 0] or theta_deg[i] > self.theta_max[i, 1]):
                return False
        return True
            
    def IK(self, cur_q, goal_T, joint_rad=0, eps=1e-10):
        status = False
        theta = np.asarray(cur_q, dtype=np.float32)
        DH_cur = np.copy(self.DH)
        min_diff = 10000
        if(joint_rad == 0):
            for i in range(cur_q.shape[0]):
                theta[i] *= np.pi / 180
        cur_theta = np.copy(theta)
        theta_tmp = np.copy(theta)
        T = self.base_inv @ goal_T @ self.ee_inv
        R = T[:3, :3]
        P = T[:3, 3]
        X, Y, Z = P[0, 0], P[1, 0], P[2, 0]
        r = np.sqrt(X**2 + Y**2 + Z**2)
        a1, a2, a3 = self.DH[0, 2], self.DH[1, 2], self.DH[2, 2]
        d4 = self.DH[3, 1]
        r2 = r**2
        a12, a22, a32 = a1**2, a2**2, a3**2
        d42 = d4**2
        m = a32 + d42
        e = 2 * r2
        c = 4 * a12
        l = a2 * (2 * m + 2 * a12 + 2 * a22 - c - e)
        l2 = l**2
        h = (c + e) * m - (m + a12 + a22)**2 + a12 * e + a22 * e + 4 * a12 * a22 - 4 * a12 * Z**2 - r**4

        cond1 = 4 * l2 + 16 * a22 * h
        th23_candidate_cnt, th1_candidate_cnt, all_candidate_cnt = 0, 0, 0
        th23_candidates = np.zeros((8, 2))
        if(cond1 >= 0):
            f1 = (-2 * l + np.sqrt(cond1)) / (8 * a22 + eps)
            cond2 = d42 + a32 - f1**2
            if(cond2 >= 0):
                # First candidate
                th3_tmp = 2 * np.arctan((-d4 + np.sqrt(cond2)) / (a3 + f1 + eps))

                f1 = -np.sin(th3_tmp) * d4 + a3 * np.cos(th3_tmp)
                f2 = np.cos(th3_tmp) * d4 + a3 * np.sin(th3_tmp)
                t1 = f1 + a2
                k = f1**2 + f2**2 + 2 * f1 * a2 + a12 + a22
                t2 = (r2 - k) / (2 * a1 + eps)
                t1 = f2
                t2 = -f1-a2
                t3 = Z
                th2_tmp1 = 2 * np.arctan((t2 + np.sqrt(t2**2 + t1**2 - t3**2)) / (t1 + t3 + eps)) + np.pi / 2
                th2_tmp2 = 2 * np.arctan((t2 - np.sqrt(t2**2 + t1**2 - t3**2)) / (t1 + t3 + eps)) + np.pi / 2

                if(th3_tmp < self.theta_max_rad[2, 1] and th3_tmp > self.theta_max_rad[2, 0]):
                    if(th2_tmp1 < self.theta_max_rad[1, 1] and th2_tmp1 > self.theta_max_rad[1, 0]):
                        th23_candidates[th23_candidate_cnt, :] = np.asarray([th2_tmp1, th3_tmp])
                        th23_candidate_cnt += 1
                    if(th2_tmp2 < self.theta_max_rad[1, 1] and th2_tmp2 > self.theta_max_rad[1, 0]):
                        th23_candidates[th23_candidate_cnt, :] = np.asarray([th2_tmp2, th3_tmp])
                        th23_candidate_cnt += 1

                # Second candidate
                th3_tmp = 2 * np.arctan((-d4 - np.sqrt(cond2)) / (a3 + f1 + eps))
                f1 = -np.sin(th3_tmp) * d4 + a3 * np.cos(th3_tmp)
                f2 = np.cos(th3_tmp) * d4 + a3 * np.sin(th3_tmp)
                t1 = f1 + a2
                k = f1**2 + f2**2 + 2 * f1 * a2 + a12 + a22
                t2 = (r2 - k) / (2 * a1 + eps)
                t1 = f2
                t2 = -f1-a2
                t3 = Z
                th2_tmp1 = 2 * np.arctan((t2 + np.sqrt(t2**2 + t1**2 - t3**2)) / (t1 + t3 + eps)) + np.pi / 2
                th2_tmp2 = 2 * np.arctan((t2 - np.sqrt(t2**2 + t1**2 - t3**2)) / (t1 + t3 + eps)) + np.pi / 2
                if(th3_tmp < self.theta_max_rad[2, 1] and th3_tmp > self.theta_max_rad[2, 0]):
                    if(th2_tmp1 < self.theta_max_rad[1, 1] and th2_tmp1 > self.theta_max_rad[1, 0]): 
                        th23_candidates[th23_candidate_cnt, :] = np.asarray([th2_tmp1, th3_tmp])
                        th23_candidate_cnt +=1
                    if(th2_tmp2 < self.theta_max_rad[1, 1] and th2_tmp2 > self.theta_max_rad[1, 0]):
                    
                        th23_candidates[th23_candidate_cnt, :] = np.asarray([th2_tmp2, th3_tmp])
                        th23_candidate_cnt +=1
                    
            f1 = (-2 * l - np.sqrt(cond1)) / (8 * a22 + eps)
            cond2 = d42 + a32 - f1**2
            if(cond2 >= 0):
                # Third candidate
                th3_tmp = 2 * np.arctan((-d4 + np.sqrt(cond2)) / (a3 + f1 + eps))
                f1 = -np.sin(th3_tmp) * d4 + a3 * np.cos(th3_tmp)
                f2 = np.cos(th3_tmp) * d4 + a3 * np.sin(th3_tmp)
                t1 = f1 + a2
                k = f1**2 + f2**2 + 2 * f1 * a2 + a12 + a22
                t2 = (r2 - k) / (2 * a1 + eps)
                t1 = f2
                t2 = -f1-a2
                t3 = Z
                th2_tmp1 = 2 * np.arctan((t2 + np.sqrt(t2**2 + t1**2 - t3**2)) / (t1 + t3 + eps)) + np.pi / 2
                th2_tmp2 = 2 * np.arctan((t2 - np.sqrt(t2**2 + t1**2 - t3**2)) / (t1 + t3 + eps)) + np.pi / 2
                if(th3_tmp < self.theta_max_rad[2, 1] and th3_tmp > self.theta_max_rad[2, 0]):
                    if(th2_tmp1 < self.theta_max_rad[1, 1] and th2_tmp1 > self.theta_max_rad[1, 0]):
                        th23_candidates[th23_candidate_cnt, :] = np.asarray([th2_tmp1, th3_tmp])
                        th23_candidate_cnt +=1
                    
                    if(th2_tmp2 < self.theta_max_rad[1, 1] and th2_tmp2 > self.theta_max_rad[1, 0]):
                        th23_candidates[th23_candidate_cnt, :] = np.asarray([th2_tmp2, th3_tmp])
                        th23_candidate_cnt +=1
                    
                # Fourth candidate
                th3_tmp = 2 * np.arctan((-d4 - np.sqrt(cond2)) / (a3 + f1 + eps))
                f1 = -np.sin(th3_tmp) * d4 + a3 * np.cos(th3_tmp)
                f2 = np.cos(th3_tmp) * d4 + a3 * np.sin(th3_tmp)
                t1 = f1 + a2
                k = f1**2 + f2**2 + 2 * f1 * a2 + a12 + a22
                t2 = (r2 - k) / (2 * a1 + eps)
                t1 = f2
                t2 = -f1-a2
                t3 = Z
                th2_tmp1 = 2 * np.arctan((t2 + np.sqrt(t2**2 + t1**2 - t3**2)) / (t1 + t3 + eps)) + np.pi / 2
                th2_tmp2 = 2 * np.arctan((t2 - np.sqrt(t2**2 + t1**2 - t3**2)) / (t1 + t3 + eps)) + np.pi / 2
                if(th3_tmp < self.theta_max_rad[2, 1] and th3_tmp > self.theta_max_rad[2, 0]):
                    if(th2_tmp1 < self.theta_max_rad[1, 1] and th2_tmp1 > self.theta_max_rad[1, 0]):
                        th23_candidates[th23_candidate_cnt, :] = np.asarray([th2_tmp1, th3_tmp])
                        th23_candidate_cnt +=1
                    if(th2_tmp2 < self.theta_max_rad[1, 1] and th2_tmp2 > self.theta_max_rad[1, 0]): 
                        th23_candidates[th23_candidate_cnt, :] = np.asarray([th2_tmp2, th3_tmp])
                        th23_candidate_cnt +=1
        else:
            status = False
            return cur_q, status
        
        th1_candidates = np.zeros((2, 1))
        candidates = np.zeros((32, cur_q.shape[0]))
        for i in range(th23_candidate_cnt):
            th2_tmp = th23_candidates[i, 0] - np.pi / 2
            th3_tmp = th23_candidates[i, 1]
            th1_candidate_cnt = 0

            g1 = f1 * np.cos(th2_tmp) + f2 * np.sin(th2_tmp) + a2 * np.cos(th2_tmp)
            g2 = f1 * np.sin(th2_tmp) - f2 * np.cos(th2_tmp) + a2 * np.sin(th2_tmp)
            q1 = g1+a1
            q2 = 0
            cond1 = q2**2 + q1**2 - X**2
            q1 = 0
            q2 = g1+a1
            th1_tmp1 = 2 * np.arctan((q2 + np.sqrt(q2**2 + q1**2 - Y**2)) / (q1 + Y + eps))
            th1_tmp2 = 2 * np.arctan((q2 - np.sqrt(q2**2 + q1**2 - Y**2)) / (q1 + Y + eps))

            if(th1_tmp1 < self.theta_max_rad[0, 1] and th1_tmp1 > self.theta_max_rad[0, 0]):
                th1_candidates[th1_candidate_cnt, 0] = th1_tmp1
                th1_candidate_cnt += 1
            if(th1_tmp2 < self.theta_max_rad[0, 1] and th1_tmp2 > self.theta_max_rad[0, 0]):
                th1_candidates[th1_candidate_cnt, 0] = th1_tmp2
                th1_candidate_cnt += 1

            for j in range(th1_candidate_cnt):
                theta_tmp[0] = th1_candidates[j, 0]
                theta_tmp[1] = th2_tmp + np.pi / 2
                theta_tmp[2] = th3_tmp
                DH_cur[:, 0] = self.DH[:, 0] + theta_tmp
                R03 = np.identity(3)
                a = DH_cur[:, 3]
                q = DH_cur[:, 0]
                for k in range(3):
                    tmp = np.matrix([[np.cos(q[k]), -np.sin(q[k]) * np.cos(a[k]), np.sin(q[k]) * np.sin(a[k])],
                                     [np.sin(q[k]), np.cos(q[k]) * np.cos(a[k]), -np.cos(q[k]) * np.sin(a[k])],
                                     [0, np.sin(a[k]), np.cos(a[k])]])
                    R03 = R03 @ tmp
                R36 = np.linalg.inv(R03) @ R
                th5_tmp = np.arccos(-R36[2, 2])
                s5 = np.sin(th5_tmp) + eps
                if(abs(s5) <= 0.000001):
                    th4_tmp = 0
                    th5_tmp = 0
                    th6_tmp = np.arctan(R36[0, 1], R36[0, 0])
                    theta_tmp[3] = th4_tmp
                    theta_tmp[4] = th5_tmp
                    theta_tmp[5] = th6_tmp
                    if(self.joint_in_range(theta_tmp, 1)):
                        candidates[all_candidate_cnt, :] = theta_tmp.transpose() 
                        all_candidate_cnt += 1
                    
                    th5_tmp = np.pi
                    th6_tmp = np.arctan2(R36[1, 0], -R36[1, 1])
                    theta_tmp[3] = th4_tmp
                    theta_tmp[4] = th5_tmp
                    theta_tmp[5] = th6_tmp
                    if(self.joint_in_range(theta_tmp, 1)):
                        candidates[all_candidate_cnt, :] = theta_tmp.transpose() 
                        all_candidate_cnt += 1
                else:
                    th4_1 = np.arctan2(R36[1, 2] / s5, R36[0, 2] / s5)
                    th6_1 = np.arctan2(R36[2, 1] / s5, R36[2, 0] / s5)
                    s5 = np.sin(-th5_tmp)
                    th4_2 = np.arctan2(R36[1, 2] / s5, R36[0, 2] / s5)
                    th6_2 = np.arctan2(R36[2, 1] / s5, R36[2, 0] / s5)

                    th4_tmp = th4_1
                    th6_tmp = th6_1
                    theta_tmp[3] = th4_tmp
                    theta_tmp[4] = th5_tmp
                    theta_tmp[5] = th6_tmp
                    if(self.joint_in_range(theta_tmp, 1)):
                        candidates[all_candidate_cnt, :] = theta_tmp.transpose() 
                        all_candidate_cnt += 1
                    
                    th5_tmp = -th5_tmp
                    th4_tmp = th4_2
                    th6_tmp = th6_2
                    theta_tmp[3] = th4_tmp
                    theta_tmp[4] = th5_tmp
                    theta_tmp[5] = th6_tmp
                    if(self.joint_in_range(theta_tmp, 1)):
                        candidates[all_candidate_cnt, :] = theta_tmp.transpose() 
                        all_candidate_cnt += 1

        status = False
        for i in range(all_candidate_cnt):
            theta_tmp = candidates[i, :]
            verify_T = self.FK(theta_tmp, 1)
            if(np.allclose(verify_T, goal_T, atol=1e-4) and np.linalg.norm(theta_tmp - cur_theta) < min_diff):
                theta = theta_tmp
                min_diff = np.linalg.norm(theta_tmp - cur_theta)
                status = True
        if(not status):
            return cur_q, status
        
        # Rad to Deg
        theta = np.round(theta * 180 / np.pi, decimals=5)
        return theta, status
    
    def find_brick(self, color):
        color_val = [0, 0, 255]
        if(color == "red"):
            color_val = [0, 0, 255]
        elif(color == "blue"):
            color_val = [255, 0, 0]
        elif(color == "green"):
            color_val = [0, 255, 0]
        color_val = np.asarray(color_val)

        mask = cv2.inRange(self.color_img, color_val-20, color_val+20)
        masked_color_img = cv2.bitwise_and(self.color_img, self.color_img, mask=mask)
        masked_depth_img = cv2.bitwise_and(self.depth_img, self.depth_img, mask=mask)
        center_y, center_x, _ = np.where(masked_color_img >= 1)

        points = np.concatenate((center_x.reshape(-1, 1), center_y.reshape(-1, 1)), axis=1)
        pca = PCA(n_components=2)
        pca.fit(points)
        block_rot_z = np.arctan2(pca.components_[0, 1], pca.components_[0, 0])
        while(block_rot_z < -np.pi / 2):
            block_rot_z += np.pi
        while(block_rot_z > np.pi / 2):
            block_rot_z -= np.pi

        center_x = int(np.average(center_x))
        center_y = int(np.average(center_y))

        depth = masked_depth_img[center_y, center_x]
        center_pt = np.array([[center_x * depth], [center_y * depth], [depth]])
        
        block_cam_pos = np.linalg.inv(self.cam_mtx) @ center_pt
        block_cam_pos = np.concatenate((block_cam_pos, np.asarray([[1]])))
        brick_pos = self.robot2plane @ block_cam_pos
        return brick_pos, block_rot_z


class Gripper():
    def __init__(self, gripper_controller_topic):
        self.pub_joint_sim = []
        for i in range(len(gripper_controller_topic)):
            self.pub_joint_sim.append(rospy.Publisher(gripper_controller_topic[i], Float64, queue_size=1))
        self.gripper_controller_topic = copy.copy(gripper_controller_topic)
        self.pub_joint = rospy.Publisher('Robotiq3FGripperJoint', Float32MultiArray, queue_size=12)
        self.sub_goal = rospy.Subscriber("Robotiq3FGripperGoal", Float32MultiArray, self.gripper_goal_callback)
        self.goal = np.zeros(4) # target POA, POB, POC, POS
        self.joint = np.zeros(12) # joint angle of 12 finger joints for gazebo and avp simulation 
        
    def gripper_goal_callback(self, data):
        for i in range(4):
            self.goal[i] = data.data[i]
        
    def drive(self, poa, pob, poc, pos):
        self.goal = [min(max(int(poa), 0), 255), min(max(int(pob), 0), 255), min(max(int(poc), 0), 255), min(max(int(pos), 0), 255)]
        self.gripper_publish_joint()
    
    def controlGripperRegister(self, POA = 0, POB = 0, POC = 0, POS = 137, ICF = False):
        '''
            POA degree of finger A (0, 255)
            POB degree of finger B (0, 255)
            POC degree of finger C (0, 255)
            POS degree of scissor (0, 255)
            ICF whether control the degree of each finger indenpendently, 
                when ICF is False, POB and POC is not used
        '''
        COEF_SCISSORS = 26 / 220
        COEF_JOINT_1 = 62 / 140
        COEF_JOINT_2 = 90 / 100
        
        # position of scissor axis
        fa0 = 0
        fb0 = COEF_SCISSORS * min(POS, 220) - 10
        fc0 = -fb0

        # position of finger A
        fa1 = COEF_JOINT_1 * min(POA, 140)
        fa2 = COEF_JOINT_2 * min(max(POA-140, 0), 100)
        fa3 = - COEF_JOINT_1 * min(POA, 110)

        # individual control of fingers
        if ICF == False:
            fb1, fb2, fb3 = fa1, fa2, fa3
            fc1, fc2, fc3 = fa1, fa2, fa3
        else:
            fb1 = COEF_JOINT_1 * min(POB, 140)
            fb2 = COEF_JOINT_2 * min(max(POB-140, 0), 100)
            fb3 = - COEF_JOINT_1 * min(POB, 110)
            fc1 = COEF_JOINT_1 * min(POC, 140)
            fc2 = COEF_JOINT_2 * min(max(POC-140, 0), 100)
            fc3 = - COEF_JOINT_1 * min(POC, 110)
            
        commandGripper = np.array([fa0, fa1, fa2, fa3,
                                   fb0, fb1, fb2, fb3,
                                   fc0, fc1, fc2, fc3])
    
        return commandGripper
    
    def gripper_publish_joint(self):
        self.joint = self.controlGripperRegister(POA = self.goal[0],
                                                    POB = self.goal[1],
                                                    POC = self.goal[2],
                                                    POS = self.goal[3],
                                                    ICF = True)

        for i in range(len(self.gripper_controller_topic)):
            joint_sim_msg = Float64()
            joint_sim_msg.data = self.joint[i] / 180 * np.pi
            self.pub_joint_sim[i].publish(joint_sim_msg)
            
        joint_msg = Float32MultiArray()
        joint_msg.data = self.joint
        self.pub_joint.publish(joint_msg)
        

    


if __name__ == '__main__':
    robot = Robot("./config/robot_properties/")

    init_q = np.asarray([0, 0, 0, 0, -90, 0])
    T = robot.FK(init_q)
    T[0, 3] = 0.3
    T[1, 3] = 0
    T[2, 3] = 0.3
    ts = time.time()
    q = robot.IK(init_q * 0, T)
    print(q, time.time() - ts)

