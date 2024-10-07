from utils import *
from Robot import *





def main():
    goal_list = [("red", "blue")]

    rospy.init_node("block_assembly")
    fanuc = Robot("./config/robot_properties/")

    home_T = np.matrix([[0.5001711,  0.8659266, 0, 0.3],
                        [0.8659266, -0.5001711, 0, 0],
                        [0, 0, -1, 0.5],
                        [0, 0, 0, 1]])
    home_q, status = fanuc.IK(fanuc.robot_state, home_T)
    cur_goal = home_q
    fanuc.drive_robot(cur_goal)
    time.sleep(1)
    mode = 0
    task_idx = 0
    gripper_deg = 101
    while not rospy.is_shutdown():
        if(fanuc.goal_reached()):
            if(task_idx > len(goal_list)):
                break
            if(mode == 0):
                cur_goal = home_q
                fanuc.gripper.drive(0, 0, 0, 255)
            elif(mode == 1):
                pick_pos, pick_rz = fanuc.find_brick(goal_list[task_idx][0])
                place_pos, place_rz = fanuc.find_brick(goal_list[task_idx][1])
                goal_T = home_T
                goal_T[0, 3] = pick_pos[0]
                goal_T[1, 3] = pick_pos[1]
                goal_T[2, 3] = pick_pos[2] + 0.23
                goal_T = goal_T * np.matrix([[np.cos(pick_rz), -np.sin(pick_rz), 0, 0],
                                             [np.sin(pick_rz), np.cos(pick_rz), 0, 0],
                                             [0, 0, 1, 0],
                                             [0, 0, 0, 1]])
                cur_goal, status = fanuc.IK(fanuc.robot_state, goal_T)
                fanuc.gripper.drive(0, 0, 0, 255)
            elif(mode == 2):
                fanuc.gripper.drive(gripper_deg, gripper_deg, gripper_deg, 255)
                time.sleep(1.5)
            elif(mode == 3):
                cur_goal = home_q
                fanuc.gripper.drive(gripper_deg, gripper_deg, gripper_deg, 255)
            elif(mode == 4):
                goal_T = home_T
                goal_T[0, 3] = place_pos[0]
                goal_T[1, 3] = place_pos[1]
                goal_T[2, 3] = place_pos[2] + 0.25
                goal_T = goal_T * np.matrix([[np.cos(place_rz), -np.sin(place_rz), 0, 0],
                                             [np.sin(place_rz), np.cos(place_rz), 0, 0],
                                             [0, 0, 1, 0],
                                             [0, 0, 0, 1]])
                cur_goal, status = fanuc.IK(fanuc.robot_state, goal_T)
                fanuc.gripper.drive(gripper_deg, gripper_deg, gripper_deg, 255)
            elif(mode == 5):
                fanuc.gripper.drive(0, 0, 0, 255)
                time.sleep(1.5)
                cur_goal = home_q
            mode += 1
            if(mode > 5):
                mode = 0
                task_idx += 1
            time.sleep(0.01)
        fanuc.drive_robot(cur_goal)
            
if __name__ == '__main__':
    main()