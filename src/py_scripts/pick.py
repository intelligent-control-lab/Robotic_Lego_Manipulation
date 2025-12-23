from Env import *
from Robot import *

def main():
    config_fname = "./config/user_config_sim.json"
    env = Environment(config_fname)
    robot = Robot(config_fname)
    home_q = robot.home_q
    robot.drive_robot(home_q)
    time.sleep(2)

    brick_name = "b2_1"
    brick_T = env.get_state(brick_name, relative_to="galaxea_r1_lite_/galaxea/")

    press_side = 4
    press_offset = 1
    approach_dir = -1
    moving_joints = ["right_arm_joint1", "right_arm_joint2", "right_arm_joint3", 
                     "right_arm_joint4", "right_arm_joint5", "right_arm_joint6"]
    ee_name = "right_attach"
    IK_solver = "opt"
    tool_T, pre_tool_T, pre_pre_tool_T = attach_brick_T(brick_name, brick_T, press_side, press_offset, approach_dir, env.lego_lib)

    q_sol, log1 = robot.IK(pre_pre_tool_T, home_q, ee_name, moving_joints, solver=IK_solver)
    robot.drive_robot(q_sol)
    time.sleep(2)

    q_sol, log2 = robot.IK(pre_tool_T, q_sol, ee_name, moving_joints, solver=IK_solver)
    robot.drive_robot(q_sol)
    time.sleep(2)

    q_sol, log3 = robot.IK(tool_T, q_sol, ee_name, moving_joints, solver=IK_solver)
    robot.drive_robot(q_sol)
    print(log1["status"], log2["status"], log3["status"])
    print(log1["time"], log2["time"], log3["time"])

    

if __name__ == "__main__":
    main()