from FoundationPoseEstimator import *

class Robot():
    def __init__(self, config_fname):
        config = load_json(config_fname)
        self.robot_model = pin.buildModelFromUrdf(config["Robot_URDF_fname"])
        self.joint_ranges = {"lower": self.robot_model.lowerPositionLimit, "upper": self.robot_model.upperPositionLimit}
        self.waypoint_travel_time = config["Waypoint_Travel_Time"]
        travel_time_topic = config["Travel_Time_Topic"]
        robot_goal_topic = config["Robot_Goal_Topic"]
        robot_state_topic = config["Robot_State_Topic"]
        head_color_topic = config["Head_Color_Topic"]
        head_depth_topic = config["Head_Depth_Topic"]
        right_color_topic = config["Right_Color_Topic"]
        right_depth_topic = config["Right_Depth_Topic"]
        left_color_topic = config["Left_Color_Topic"]
        left_depth_topic = config["Left_Depth_Topic"]
        self.data_dir = config["Data_Dir"]
        self.robot_dof = self.robot_model.nq
        camera_info = load_json(config["Camera_Info_Dir"])
        self.head_cam_K = np.array([[camera_info["head_K"]["fx"], 0.0, camera_info["head_K"]["cx"]],
                                    [0.0, camera_info["head_K"]["fy"], camera_info["head_K"]["cy"]],
                                    [0.0, 0.0, 1.0]])
        self.right_cam_K = np.array([[camera_info["right_K"]["fx"], 0.0, camera_info["right_K"]["cx"]],
                                     [0.0, camera_info["right_K"]["fy"], camera_info["right_K"]["cy"]],
                                     [0.0, 0.0, 1.0]])
        self.left_cam_K = np.array([[camera_info["left_K"]["fx"], 0.0, camera_info["left_K"]["cx"]],
                                     [0.0, camera_info["left_K"]["fy"], camera_info["left_K"]["cy"]],
                                     [0.0, 0.0, 1.0]])
        self.foundationpose = FoundationPoseEstimator()

        rospy.init_node('robot_py', anonymous=True)

        # Create the pubs & subs
        self.robot_goal_pub = rospy.Publisher(robot_goal_topic, Float32MultiArray, queue_size=self.robot_dof)
        self.travel_time_pub = rospy.Publisher(travel_time_topic, Float64, queue_size=1)

        rospy.Subscriber(robot_state_topic, Float32MultiArray, self.robot_state_callback)
        robot_state_sub = Subscriber(robot_state_topic, Float32MultiArray)
        head_color_sub = Subscriber(head_color_topic, Image)
        head_depth_sub = Subscriber(head_depth_topic, Image)
        right_color_sub = Subscriber(right_color_topic, Image)
        right_depth_sub = Subscriber(right_depth_topic, Image)
        left_color_sub = Subscriber(left_color_topic, Image)  
        left_depth_sub = Subscriber(left_depth_topic, Image)
        ats = ApproximateTimeSynchronizer([robot_state_sub, 
                                           head_color_sub, head_depth_sub, 
                                           right_color_sub, right_depth_sub, 
                                           left_color_sub, left_depth_sub], 
                                          queue_size=10, slop=0.02, allow_headerless=True)
        ats.registerCallback(self.synced_callback)
        self.bridge = CvBridge()
        
        self.ros_hz = 1000
        self.robot_state = None
        queue_len = 10
        self.robot_state_buffer = deque(maxlen=queue_len)
        self.head_color_buffer = deque(maxlen=queue_len)
        self.head_depth_buffer = deque(maxlen=queue_len)
        self.right_color_buffer = deque(maxlen=queue_len)
        self.right_depth_buffer = deque(maxlen=queue_len)
        self.left_color_buffer = deque(maxlen=queue_len)
        self.left_depth_buffer = deque(maxlen=queue_len)
        self.set_travel_time(self.waypoint_travel_time)
        self.home_q = np.array([-25, 90, 65, 0, 90, -90, 90, 0, 45, 0, 90, -90, 90, 0, 45]) # degrees

    def synced_callback(self, robot_state_data, 
                        head_color_data, head_depth_data, 
                        right_color_data, right_depth_data, 
                        left_color_data, left_depth_data):
        try:
            self.robot_state_buffer.append(robot_state_data)
            self.head_color_buffer.append(head_color_data)
            self.head_depth_buffer.append(head_depth_data)
            self.right_color_buffer.append(right_color_data)
            self.right_depth_buffer.append(right_depth_data)
            self.left_color_buffer.append(left_color_data)
            self.left_depth_buffer.append(left_depth_data)
        except Exception as e:
            print("Synced callback error:", e)
        
    def get_observations(self):
        if len(self.head_color_buffer) == 0:
            print("No observation received yet.")
            return None, None, None, None, None, None
        robot_state_data = self.robot_state_buffer[-1]
        head_color_data = self.head_color_buffer[-1]
        head_depth_data = self.head_depth_buffer[-1]
        right_color_data = self.right_color_buffer[-1]
        right_depth_data = self.right_depth_buffer[-1]
        left_color_data = self.left_color_buffer[-1]
        left_depth_data = self.left_depth_buffer[-1]
        
        head_color = self.bridge.imgmsg_to_cv2(head_color_data, "rgb8")
        head_depth = self.bridge.imgmsg_to_cv2(head_depth_data, "32FC1")
        right_color = self.bridge.imgmsg_to_cv2(right_color_data, "rgb8")
        right_depth = self.bridge.imgmsg_to_cv2(right_depth_data, "32FC1")
        left_color = self.bridge.imgmsg_to_cv2(left_color_data, "rgb8")
        left_depth = self.bridge.imgmsg_to_cv2(left_depth_data, "32FC1")
        robot_state = robot_state_data.data
        return robot_state, head_color, head_depth, right_color, right_depth, left_color, left_depth

    def locate_object(self, object_name, # base32x32, b1x1, b1x2, b1x4, b1x6, b1x8, b2x2, b2x4, b2x6 
                      view, # head, right, left 
                      color=None, 
                      track=False,
                      vis_detection=False):
        robot_state, head_color, head_depth, right_color, right_depth, left_color, left_depth = self.get_observations()

        # Load CAD
        mesh = trimesh.load(self.data_dir + f'/meshes/{object_name}.obj')
        mesh.apply_scale(0.001)
        print("Loaded mesh:", self.data_dir + f'/meshes/{object_name}.obj')

        if(view == "head"):
            color = head_color
            depth = head_depth
            K = self.head_cam_K
        elif(view == "right"):
            color = right_color
            depth = right_depth
            K = self.right_cam_K
        elif(view == "left"):
            color = left_color
            depth = left_depth
            K = self.left_cam_K
        else:
            raise ValueError("Invalid view name")
        mask = np.ones([depth.shape[0], depth.shape[1]]).astype(bool)
        depth = depth / 1000.0  # convert to meters

        # hsv = cv2.cvtColor(color, cv2.COLOR_RGB2HSV)
        lower = np.array([100, 0, 0])
        upper = np.array([255, 25, 25])
        mask = cv2.inRange(color, lower, upper).astype(bool)

        pose = self.foundationpose.locate(mesh, color, depth, mask, K, track=track, vis_detection=vis_detection)
        return pose, robot_state
    

    def robot_state_callback(self, data):
        self.robot_state = data.data

    def FK(self, q, ee_name, input_deg=True):
        if(input_deg):
            q = q / 180 * np.pi
        data = self.robot_model.createData()
        pin.forwardKinematics(self.robot_model, data, q)
        pin.updateFramePlacements(self.robot_model, data)
        ee_id = self.robot_model.getFrameId(ee_name)
        T = np.eye(4)
        T[:3, :3] = data.oMf[ee_id].rotation
        T[:3, 3] = data.oMf[ee_id].translation
        return T
    
    def IK(self, cart_pt, q_init, ee_name, planning_joint_list, solver="opt",
           step_size=0.1, TOL=1e-3, MAX_ITERS=10000, input_deg=True, output_deg=True):
        if(solver == "opt"):
            return self.IK_opt(cart_pt, q_init, ee_name, planning_joint_list, 
                               TOL=TOL, input_deg=input_deg, output_deg=output_deg)
        elif(solver == "iter"):
            return self.IK_iter(cart_pt, q_init, ee_name, planning_joint_list, 
                                step_size=step_size, TOL=TOL, MAX_ITERS=MAX_ITERS, input_deg=input_deg, output_deg=output_deg)
        else:
            raise TypeError("Unknown IK solver!")
        
    def IK_iter(self, cart_pt, q_init, ee_name, planning_joint_list, 
                step_size=0.1, TOL=1e-3, MAX_ITERS=10000, input_deg=True, output_deg=True):
        ts = time.time()
        if(input_deg):
            q_init = q_init / 180 * np.pi
        data = self.robot_model.createData()
        ee_id = self.robot_model.getFrameId(ee_name)
        target_pose = pin.SE3(cart_pt[:3, :3], cart_pt[:3, 3])
        status = False
        q = np.copy(q_init)
        
        unlock_joint_ids = []
        for joint_name in planning_joint_list:
            joint_id = self.robot_model.getJointId(joint_name)
            idx_q = self.robot_model.joints[joint_id].idx_q
            unlock_joint_ids.append(idx_q)
        locked_joint_ids = np.setdiff1d(np.arange(q.shape[0]), unlock_joint_ids)

        for iters in range(MAX_ITERS):
            pin.forwardKinematics(self.robot_model, data, q)
            pin.updateFramePlacements(self.robot_model, data)
            current_pose = data.oMf[ee_id]
            err = pin.log6(current_pose.inverse() * target_pose).vector
            if np.linalg.norm(err) < TOL:
                status = True
                break
            J = pin.computeFrameJacobian(self.robot_model, data, q, ee_id, pin.LOCAL)
            J[:, locked_joint_ids] = 0
            dq = step_size * np.linalg.pinv(J) @ err
            q = pin.integrate(self.robot_model, q, dq)
            q = np.clip(q, self.joint_ranges["lower"], self.joint_ranges["upper"])
        else:
            q = np.copy(q_init)
        
        if(output_deg):
            q = q / np.pi * 180
        log = {"status": status,
               "time": time.time() - ts,
               "num_iter": iters,
               "error": err}
        return q, log
    
    def IK_opt(self, cart_pt, q_init, ee_name, planning_joint_list, 
               TOL=1e-3, input_deg=True, output_deg=True):
        ts = time.time()
        if(input_deg):
            q_init = q_init / 180 * np.pi

        data = self.robot_model.createData()
        ee_id = self.robot_model.getFrameId(ee_name)
        target_pose = pin.SE3(cart_pt[:3, :3], cart_pt[:3, 3])
        full_q = np.copy(q_init)
        status = False
        unlock_joint_ids = []
        for joint_name in planning_joint_list:
            joint_id = self.robot_model.getJointId(joint_name)
            idx_q = self.robot_model.joints[joint_id].idx_q
            unlock_joint_ids.append(idx_q)
        lower_bounds = self.joint_ranges["lower"][unlock_joint_ids]
        upper_bounds = self.joint_ranges["upper"][unlock_joint_ids]
        q0 = full_q[unlock_joint_ids] + 1e-8
        q0 = np.clip(q0, lower_bounds, upper_bounds)

        def ik_error(q_in):
            full_q[unlock_joint_ids] = q_in
            pin.forwardKinematics(self.robot_model, data, full_q)
            pin.updateFramePlacements(self.robot_model, data)
            current_pose = data.oMf[ee_id]
            error = pin.log6(current_pose.inverse() * target_pose).vector
            return error
        
        result = least_squares(
            ik_error,
            q0,
            bounds=(lower_bounds, upper_bounds),
            loss="huber",
        )

        full_q[unlock_joint_ids] = result.x
        err = ik_error(result.x)
        if np.linalg.norm(err) < TOL:
            status = True

        if(output_deg):
            full_q = full_q / np.pi * 180
        log = {"status": status,
               "time": time.time() - ts,
               "lsq_status": result.status,
               "error": err}
        return full_q, log
    
    def drive_robot(self, q_goal):
        msg = Float32MultiArray()
        msg.data = q_goal.tolist()
        robot_state = self.robot_state

        rate = rospy.Rate(self.ros_hz)
        for _ in range(self.ros_hz):
            self.robot_goal_pub.publish(msg)
            if(self.robot_state != robot_state):
                break
            rate.sleep()

    def set_travel_time(self, travel_time):
        self.waypoint_travel_time = travel_time
        msg = Float64()
        msg.data = travel_time
        rate = rospy.Rate(self.ros_hz)
        for _ in range(int(self.ros_hz * 0.3)):
            self.travel_time_pub.publish(msg)
            rate.sleep()

if __name__ == "__main__":
    robot = Robot("./config/user_config_sim.json")
    q = np.zeros(robot.robot_dof)
    robot.drive_robot(q)
    time.sleep(2)
   
    q_sol, log = robot.IK(np.array([[1, 0, 0, 0.7],
                                    [ 0, 1, 0, -0.3333],
                                    [ 0, 0, 1,  0.9],
                                    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]), 
                                    q, 
                                    "right_attach", 
                                    ["right_arm_joint1", "right_arm_joint2", "right_arm_joint3", 
                                     "right_arm_joint4", "right_arm_joint5", "right_arm_joint6"])
    print(q_sol, log["status"])
    robot.drive_robot(q_sol)
    time.sleep(2)
    
    for i in range(10):
        if(i < 2):
            robot.locate_object("b2x4", "right", track=False, vis_detection=1)
        else:
            robot.locate_object("b2x4", "right", track=True, vis_detection=1)