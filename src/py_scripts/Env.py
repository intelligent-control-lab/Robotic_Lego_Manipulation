from utils import *

class Environment():
    def __init__(self, config_fname):
        # Load Config Settings
        config = load_json(config_fname)
        
        # Create the service client
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.init_env(config["Env_Setup_fname"])

    
    def init_env(self, fname):
        env_setup = load_json(fname)
        storage_setup = env_setup["storage_plate"]
        r = R.from_euler('xyz', [storage_setup["roll"], storage_setup["pitch"], storage_setup["yaw"]]).as_quat()
        pose = [storage_setup["x"], storage_setup["y"], storage_setup["z"], r[0], r[1], r[2], r[3]]
        self.set_state("storage_plate", pose)

        assemble_setup = env_setup["assemble_plate"]
        r = R.from_euler('xyz', [assemble_setup["roll"], assemble_setup["pitch"], assemble_setup["yaw"]]).as_quat()
        pose = [assemble_setup["x"], assemble_setup["y"], assemble_setup["z"], r[0], r[1], r[2], r[3]]
        self.set_state("assemble_plate", pose)




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

if __name__ == "__main__":
    env = Environment("./config/user_config_sim.json")