from utils import *

class Robot():
    def __init__(self, config_fname):
        config = load_json(config_fname)
        self.robot_model = pin.buildModelFromUrdf(config["Robot_URDF_fname"])
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
        self.data_dir = config["Data_dir"]
        self.robot_dof = self.robot_model.nq

        # Create the service client
        self.robot_goal_pub = rospy.Publisher(robot_goal_topic, Float32MultiArray, queue_size=self.robot_dof)
        self.travel_time_pub = rospy.Publisher(travel_time_topic, Float64, queue_size=1)
        rospy.Subscriber(robot_state_topic, Float32MultiArray, self.robot_state_callback)
        rospy.Subscriber(head_color_topic, Image, self.head_color_callback, queue_size=1)
        rospy.Subscriber(head_depth_topic, Image, self.head_depth_callback, queue_size=1)
        rospy.Subscriber(right_color_topic, Image, self.right_color_callback, queue_size=1)
        rospy.Subscriber(right_depth_topic, Image, self.right_depth_callback, queue_size=1)
        rospy.Subscriber(left_color_topic, Image, self.left_color_callback, queue_size=1)
        rospy.Subscriber(left_depth_topic, Image, self.left_depth_callback, queue_size=1)
        self.bridge = CvBridge()

        rospy.init_node('robot_py', anonymous=True)
        self.ros_hz = 1000
        self.robot_state = None
        self.head_color = None
        self.head_depth = None
        self.head_mask = None
        self.right_color = None
        self.right_depth = None
        self.right_mask = None
        self.left_color = None
        self.left_depth = None
        self.left_mask = None
        self.set_travel_time(self.waypoint_travel_time)

    def head_color_callback(self, data):
        try:
            self.head_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imwrite(self.data_dir + "/head_color.png", self.head_color)
            
            # Use SAM2 in the future
            hsv = cv2.cvtColor(self.head_color, cv2.COLOR_BGR2HSV)
            lower = np.array([35, 40, 40])
            upper = np.array([85, 255, 255])
            self.head_mask = cv2.inRange(hsv, lower, upper)
            cv2.imwrite(self.data_dir + "/head_mask.png", self.head_mask)
        except Exception as e:
            print("Head color callback error:", e)

    def head_depth_callback(self, data):
        try:
            self.head_depth = self.bridge.imgmsg_to_cv2(data, "32FC1")
            cv2.imwrite(self.data_dir + "/head_depth.png", self.head_depth)
            np.save(self.data_dir + "/head_depth.npy", self.head_depth)
        except Exception as e:
            print("Head depth callback error:", e)

    def right_color_callback(self, data):
        self.right_color = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def right_depth_callback(self, data):
        self.right_depth = self.bridge.imgmsg_to_cv2(data, "32FC1")
    
    def left_color_callback(self, data):
        self.left_color = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def left_depth_callback(self, data):
        self.left_depth = self.bridge.imgmsg_to_cv2(data, "32FC1")

    def robot_state_callback(self, data):
        self.robot_state = data.data

    def FK(self, q, ee_name):
        data = self.robot_model.createData()
        pin.forwardKinematics(self.robot_model, data, q)
        pin.updateFramePlacements(self.robot_model, data)
        ee_id = self.robot_model.getFrameId(ee_name)
        T = np.eye(4)
        T[:3, :3] = data.oMf[ee_id].rotation
        T[:3, 3] = data.oMf[ee_id].translation
        return T
    
    def IK(self, cart_pt, q_init, ee_name, planning_joint_list, step_size=0.1, TOL = 1e-3, MAX_ITERS=10000, deg=True):
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

        for _ in range(MAX_ITERS):
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
        else:
            q = np.copy(q_init)
        if(deg):
            q = q / np.pi * 180
        return q, status
    
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
    q = np.zeros(15)

    print(robot.FK(q, "left_arm_eoat"))
    q_sol, status = robot.IK(np.array([[1, 0, 0, 0.1],
                    [0, 1, 0, 0.33],
                    [0, 0, 1, 1.3],
                    [0, 0, 0, 1]]), q, "left_arm_eoat", 
                    ["left_arm_joint1", "left_arm_joint2", "left_arm_joint3", "left_arm_joint4", "left_arm_joint5", "left_arm_joint6"])
    print(q_sol, status)

    # robot.drive_robot(q_sol)