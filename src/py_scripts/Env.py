from utils import *

class Environment():
    def __init__(self, config_fname):
        # Load Config Settings
        config = load_json(config_fname)
        self.lego_lib = load_json(config["Lego_Library_fname"])
        self.workspace_range = config["Workspace_Range"]
        self.storage_on_plate = config["Storage_On_Plate"]
        self.objects_T = {}
        self.brick_tall = 0.0096
        self.plate_tall = 0.0016
        self.P_len = 0.008

        # Create the service client
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        rospy.wait_for_service("/gazebo/get_model_state")
        self.get_state_client = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        self.init_env(config["Env_Setup_fname"])

    
    def init_env(self, fname):
        env_setup = load_json(fname)

        storage_setup = env_setup["storage_plate"]
        self.storage_plate_dim = [storage_setup["height"], storage_setup["width"]]
        storage_plate_T = np.eye(4)
        r = R.from_euler('xyz', [storage_setup["roll"], storage_setup["pitch"], storage_setup["yaw"]])
        storage_plate_T[:3, :3] = r.as_matrix()
        storage_plate_T[:3, 3] = np.array([storage_setup["x"], storage_setup["y"], storage_setup["z"]])
        quat = r.as_quat()
        pose = [storage_setup["x"], storage_setup["y"], storage_setup["z"], quat[0], quat[1], quat[2], quat[3]]
        self.set_state("storage_plate", pose)
        self.objects_T["storage_plate"] = storage_plate_T

        assemble_setup = env_setup["assemble_plate"]

        assemble_plate_T = np.eye(4)
        self.assemble_plate_dim = [assemble_setup["height"], assemble_setup["width"]]
        r = R.from_euler('xyz', [assemble_setup["roll"], assemble_setup["pitch"], assemble_setup["yaw"]])
        assemble_plate_T[:3, :3] = r.as_matrix()
        assemble_plate_T[:3, 3] = np.array([assemble_setup["x"], assemble_setup["y"], assemble_setup["z"]])
        quat = r.as_quat()
        pose = [assemble_setup["x"], assemble_setup["y"], assemble_setup["z"], quat[0], quat[1], quat[2], quat[3]]
        self.set_state("assemble_plate", pose)
        self.objects_T["assemble_plate"] = assemble_plate_T

        for k in env_setup.keys():
            if(k == "storage_plate" or k == "assemble_plate"):
                continue
            brick_setup = env_setup[k]
            brick_T = np.eye(4)
            if(self.storage_on_plate):
                x_on_plate, y_on_plate, z_on_plate, ori = brick_setup["x"], brick_setup["y"], brick_setup["z"], brick_setup["ori"]
                brick_T = brick_T_on_plate(k, x_on_plate, y_on_plate, z_on_plate, ori, storage_plate_T, self.storage_plate_dim, self.lego_lib, P_len=self.P_len, brick_tall=self.brick_tall)
                r = R.from_matrix(brick_T[:3, :3])
                quat = r.as_quat()
                pose = [brick_T[0, 3], brick_T[1, 3], brick_T[2, 3], quat[0], quat[1], quat[2], quat[3]]
                self.set_state(k, pose)
            else:
                x, y = np.random.uniform(self.workspace_range["x_min"], self.workspace_range["x_max"]), np.random.uniform(self.workspace_range["y_min"], self.workspace_range["y_max"])
                z = self.workspace_range["z"] + self.brick_tall - self.plate_tall
                roll, pitch, yaw = storage_setup["roll"], storage_setup["pitch"], np.random.uniform(-np.pi, np.pi)
                r = R.from_euler('xyz', [roll, pitch, yaw])
                quat = r.as_quat()
                brick_T[:3, :3] = r.as_matrix()
                brick_T[:3, 3] = np.array([x, y, z])
                while(not collision_free_env(k, brick_T, self.objects_T, self.storage_plate_dim, self.assemble_plate_dim, self.lego_lib, P_len=self.P_len)):
                    x, y = np.random.uniform(self.workspace_range["x_min"], self.workspace_range["x_max"]), np.random.uniform(self.workspace_range["y_min"], self.workspace_range["y_max"])
                    brick_T[:3, 3] = np.array([x, y, z])
                pose = [x, y, z, quat[0], quat[1], quat[2], quat[3]]
                self.set_state(k, pose)
            self.objects_T[k] = brick_T

    def set_state(self, name, pose):
        state_msg = ModelState()
        state_msg.model_name = name
        state_msg.pose.position.x = pose[0]
        state_msg.pose.position.y = pose[1]
        state_msg.pose.position.z = pose[2]
        state_msg.pose.orientation.x = pose[3]
        state_msg.pose.orientation.y = pose[4]
        state_msg.pose.orientation.z = pose[5]
        state_msg.pose.orientation.w = pose[6]
        ret = self.set_state_client(state_msg)
        return ret
    
    def get_state(self, name, relative_to=""):
        resp = self.get_state_client(model_name=name, relative_entity_name=relative_to)
        pose = np.eye(4)
        if resp.success:
            p = resp.pose
            pose[:3, 3] = np.array([p.position.x, p.position.y, p.position.z])
            r = R.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
            pose[:3, :3] = r.as_matrix()
        else:
            print("Failed:", resp.status_message)
        return pose

if __name__ == "__main__":
    env = Environment("./config/user_config_sim.json")