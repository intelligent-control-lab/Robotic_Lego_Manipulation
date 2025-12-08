#include "Lego.hpp"
// HEADER FILES FOR SERVICE INTERFACE WITH YK ROBOT
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/WrenchStamped.h"
#include "lego_manipulation/SetPose.h"

// HEADER FILES FOR API INTERFACE WITH YK ROBOT
// #include "yk_api/yk_interface.h"  // Uncomment this if using yaskawa robot

using namespace std::chrono;

lego_manipulation::math::VectorJd robot1_q = Eigen::MatrixXd::Zero(6, 1);
lego_manipulation::math::VectorJd robot1_qd = Eigen::MatrixXd::Zero(6, 1);
lego_manipulation::math::VectorJd robot1_qdd = Eigen::MatrixXd::Zero(6, 1);
lego_manipulation::math::VectorJd robot2_q = Eigen::MatrixXd::Zero(6, 1);
lego_manipulation::math::VectorJd robot2_qd = Eigen::MatrixXd::Zero(6, 1);
lego_manipulation::math::VectorJd robot2_qdd = Eigen::MatrixXd::Zero(6, 1);

// Get robot state from stmotion controller.
void robot1StateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    robot1_q << msg->data[0], msg->data[3], msg->data[6], msg->data[9], msg->data[12], msg->data[15];
    robot1_qd << msg->data[1], msg->data[4], msg->data[7], msg->data[10], msg->data[13], msg->data[16];
    robot1_qdd << msg->data[2], msg->data[5], msg->data[8], msg->data[11], msg->data[14], msg->data[17];
}

void robot2StateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    robot2_q << msg->data[0], msg->data[3], msg->data[6], msg->data[9], msg->data[12], msg->data[15];
    robot2_qd << msg->data[1], msg->data[4], msg->data[7], msg->data[10], msg->data[13], msg->data[16];
    robot2_qdd << msg->data[2], msg->data[5], msg->data[8], msg->data[11], msg->data[14], msg->data[17];
}


int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "task_planning_node");
        ros::NodeHandle nh;
        ros::NodeHandle private_node_handle("~");
        std::string base_frame;
        private_node_handle.param<std::string>("base_frame", base_frame, "world");
        ROS_INFO_STREAM("namespace of task_planning nh = " << nh.getNamespace());
        unsigned int second = 1000000;
        usleep(1 * second);
        ros::AsyncSpinner async_spinner(1);
        async_spinner.start();
        ros::Rate loop_rate(150);

        std::string config_fname, root_pwd, task_fname, world_base_fname;
        std::string r1_DH_fname, r1_DH_tool_fname, r1_DH_tool_assemble_fname, r1_DH_tool_disassemble_fname, r1_DH_tool_alt_fname, r1_DH_tool_alt_assemble_fname, r1_robot_base_fname;
        std::string r2_DH_fname, r2_DH_tool_fname, r2_DH_tool_assemble_fname, r2_DH_tool_disassemble_fname, r2_DH_tool_alt_fname, r2_DH_tool_alt_assemble_fname, r2_robot_base_fname;
        std::string lego_lib_fname, env_setup_fname;
        std::string r1_robot_state_topic, r1_controller_joint_goal_topic, r1_controller_cart_goal_topic, r1_controller_time_topic;
        std::string r2_robot_state_topic, r2_controller_joint_goal_topic, r2_controller_cart_goal_topic, r2_controller_time_topic;
        std::string robot1_name, robot2_name;
        bool IK_status;
        
        ros::ServiceClient set_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        private_node_handle.getParam("config_fname", config_fname);
        private_node_handle.getParam("root_pwd", root_pwd);

        std::ifstream config_file(config_fname, std::ifstream::binary);
        Json::Value config;
        config_file >> config;
        world_base_fname = root_pwd + config["world_base_fname"].asString();
        r1_DH_fname = root_pwd + config["r1_DH_fname"].asString();
        r1_DH_tool_fname = root_pwd + config["r1_DH_tool_fname"].asString();
        r1_DH_tool_assemble_fname = root_pwd + config["r1_DH_tool_assemble_fname"].asString();
        r1_DH_tool_disassemble_fname = root_pwd + config["r1_DH_tool_disassemble_fname"].asString();
        r1_DH_tool_alt_fname = root_pwd + config["r1_DH_tool_alt_fname"].asString();
        r1_DH_tool_alt_assemble_fname = root_pwd + config["r1_DH_tool_alt_assemble_fname"].asString();
        r1_robot_base_fname = root_pwd + config["Robot1_Base_fname"].asString();
        r2_DH_fname = root_pwd + config["r2_DH_fname"].asString();
        r2_DH_tool_fname = root_pwd + config["r2_DH_tool_fname"].asString();
        r2_DH_tool_assemble_fname = root_pwd + config["r2_DH_tool_assemble_fname"].asString();
        r2_DH_tool_disassemble_fname = root_pwd + config["r2_DH_tool_disassemble_fname"].asString();
        r2_DH_tool_alt_fname = root_pwd + config["r2_DH_tool_alt_fname"].asString();
        r2_DH_tool_alt_assemble_fname = root_pwd + config["r2_DH_tool_alt_assemble_fname"].asString();
        r2_robot_base_fname = root_pwd + config["Robot2_Base_fname"].asString();
        
        env_setup_fname = root_pwd + config["Env_Setup_fname"].asString();
        task_fname = root_pwd + config["Task_Graph_fname"].asString();
        lego_lib_fname = root_pwd + config["lego_lib_fname"].asString();
        robot1_name = config["Robot1_name"].asString();
        robot2_name = config["Robot2_name"].asString();

        r1_robot_state_topic = "/" + robot1_name + "/robot_state";
        r1_controller_joint_goal_topic = "/" + robot1_name + "/robot_goal";
        r1_controller_cart_goal_topic = "/" + robot1_name + "/robot_cart_goal";
        r1_controller_time_topic = "/" + robot1_name + "/jpc_travel_time";
        r2_robot_state_topic = "/" + robot2_name + "/robot_state";
        r2_controller_joint_goal_topic = "/" + robot2_name + "/robot_goal";
        r2_controller_cart_goal_topic = "/" + robot2_name + "/robot_cart_goal";
        r2_controller_time_topic = "/" + robot2_name + "/jpc_travel_time";

        bool use_yk = config["Use_yk"].asBool();
        int twist_deg = config["Twist_Deg"].asInt();
        double jpc_travel_time = config["Waypoint_Travel_Time"].asDouble();

        std::ifstream task_file(task_fname, std::ifstream::binary);
        Json::Value task_json;
        task_file >> task_json;

        lego_manipulation::lego::Lego::Ptr lego_ptr = std::make_shared<lego_manipulation::lego::Lego>();
        lego_ptr->setup_dual_arm(env_setup_fname, lego_lib_fname, task_json, world_base_fname,
                                 r1_DH_fname, r1_DH_tool_fname, r1_DH_tool_disassemble_fname, r1_DH_tool_assemble_fname, r1_DH_tool_alt_fname, r1_DH_tool_alt_assemble_fname, r1_robot_base_fname, 
                                 r2_DH_fname, r2_DH_tool_fname, r2_DH_tool_disassemble_fname, r2_DH_tool_assemble_fname, r2_DH_tool_alt_fname, r2_DH_tool_alt_assemble_fname, r2_robot_base_fname, set_state_client);

        Eigen::MatrixXd twist_R = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd twist_T = Eigen::MatrixXd::Identity(4, 4);
        Eigen::Matrix4d cart_T = Eigen::Matrix4d::Identity(4, 4);
        int grab_brick;

        ros::ServiceClient client = nh.serviceClient<lego_manipulation::SetPose>("yk_set_pose");
        lego_manipulation::SetPose srv;
        ros::Publisher r1_goal_pub = nh.advertise<std_msgs::Float32MultiArray>(r1_controller_joint_goal_topic, lego_ptr->robot_dof_1());
        ros::Publisher r1_controller_time_pub = nh.advertise<std_msgs::Float64>(r1_controller_time_topic, 1);
        ros::Publisher r2_goal_pub = nh.advertise<std_msgs::Float32MultiArray>(r2_controller_joint_goal_topic, lego_ptr->robot_dof_2());
        ros::Publisher r2_controller_time_pub = nh.advertise<std_msgs::Float64>(r2_controller_time_topic, 1);
        ros::Subscriber r1_robot_state_sub = nh.subscribe(r1_robot_state_topic, lego_ptr->robot_dof_1() * 3, robot1StateCallback);
        ros::Subscriber r2_robot_state_sub = nh.subscribe(r2_robot_state_topic, lego_ptr->robot_dof_2() * 3, robot2StateCallback);

        std_msgs::Float32MultiArray r1_goal_msg;
        std_msgs::Float64 r1_controller_time_msg;
        std_msgs::Float32MultiArray r2_goal_msg;
        std_msgs::Float64 r2_controller_time_msg;
        // Modify stmotion controller runtime
        r1_controller_time_msg.data = jpc_travel_time;
        r1_controller_time_pub.publish(r1_controller_time_msg);
        r2_controller_time_msg.data = jpc_travel_time;
        r2_controller_time_pub.publish(r2_controller_time_msg);

        int main_arm = 0;
        int support_arm = 0;
        
        int num_tasks = task_json.size();

        // Setup predefined waypoints
        Eigen::MatrixXd home_q(lego_ptr->robot_dof_1(), 1);
        Eigen::MatrixXd zero_q(lego_ptr->robot_dof_1(), 1);
        Eigen::MatrixXd receive_q(lego_ptr->robot_dof_1(), 1);
        Eigen::MatrixXd home_receive_q(lego_ptr->robot_dof_1(), 1);
        Eigen::MatrixXd home_handover_q(lego_ptr->robot_dof_1(), 1);
        home_q.col(0) << 0, 0, 0, 0, -90, 0;
        zero_q.col(0) << 0, 0, 0, 0, 0, 0;
        home_receive_q.col(0) << 0, 0, 0, 0, 0, 180;
        receive_q.col(0) << 0, 0, 0, 0, 0, 180;
        home_handover_q.col(0) << 0, 0, 0, 0, -90, 0;

        // Home
        Eigen::Matrix4d home_T = lego_manipulation::math::FK(home_q, lego_ptr->robot_DH_r1(), lego_ptr->robot_base_r1(), false);
        home_T.col(3) << 0.2, 0, 0.4, 1; // Home X, Y, Z in base frame of the Flange
        home_T = lego_ptr->world_base_frame() * home_T; // To world frame
        home_q = lego_manipulation::math::IK(home_q, home_T.block(0, 3, 3, 1), home_T.block(0, 0, 3, 3),
                                             lego_ptr->robot_DH_r1(), lego_ptr->robot_base_r1(), 0, 10e6, 10e-4*5); // Home
        
        // Home to receive
        Eigen::Matrix4d home_receive_T = lego_manipulation::math::FK(home_receive_q, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), false);
        home_receive_T.col(3) << 0.35, 0, 0.35, 1;
        home_receive_T = lego_ptr->world_base_frame() * home_receive_T; 
        home_receive_q = lego_ptr->IK(home_receive_q, home_receive_T, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(), lego_ptr->robot_tool_inv_r1(), 0, IK_status);
        assert(IK_status);

        // Receive 
        Eigen::Matrix4d receive_T = lego_manipulation::math::FK(receive_q, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), false);
        receive_T.col(3) << 0.45, 0, 0.35, 1;
        receive_T = lego_ptr->world_base_frame() * receive_T; 
        receive_q = lego_ptr->IK(receive_q, receive_T, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(), lego_ptr->robot_tool_inv_r1(), 0, IK_status);
        assert(IK_status);

        // Home to handover
        Eigen::Matrix4d home_handover_T = lego_manipulation::math::FK(home_handover_q, lego_ptr->robot_DH_r1(), lego_ptr->robot_base_r1(), false);
        home_handover_T.col(3) << 0.2, 0, 0.5, 1;
        home_handover_T = lego_ptr->world_base_frame() * home_handover_T; 
        home_handover_q = lego_ptr->IK(home_handover_q, home_handover_T, lego_ptr->robot_DH_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(), lego_ptr->robot_ee_inv_r1(), 0, IK_status);


        Eigen::Matrix4d y_n90, y_p90, z_180;
        y_n90 << 0, 0, -1, 0, 
                0, 1, 0, 0,
                1, 0, 0, 0,
                0, 0, 0, 1;
        y_p90 << 0, 0, 1, 0,
                0, 1, 0, 0,
                -1, 0, 0, 0,
                0, 0, 0, 1;
        z_180 << -1, 0, 0, 0,
                 0, -1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;

        Eigen::Matrix4d support_T = Eigen::MatrixXd::Identity(4, 4);
        Eigen::Matrix4d support_T_down = Eigen::MatrixXd::Identity(4, 4);
        Eigen::Matrix4d support_T_down_pre = Eigen::MatrixXd::Identity(4, 4);
        Eigen::Matrix4d press_T = Eigen::MatrixXd::Identity(4, 4);
        Eigen::Matrix4d press_up_T = press_T;
        
        lego_manipulation::math::VectorJd pick_offset = Eigen::MatrixXd::Zero(6, 1);

        // Attack angle
        pick_offset << -0.005, 0.005, -0.005,  // place brick offset
                       -0.005, 0.005, -0.0028; // grab brick offset

        int mode = 0; // 0:home 1:pick tilt up 2:pick_up 3:pick_down, 4:pick_twist 5:pick_twist_up 6:home
                      // 7:drop tilt up 8:drop_up 9:drop_down 10:drop_twist 11:drop_twist_up 12:done
        int task_idx = 1;
        
        lego_manipulation::math::VectorJd r1_cur_goal = home_q;
        lego_manipulation::math::VectorJd r2_cur_goal = home_q;
        std::string brick_name;
        bool move_on_to_next = use_yk;
        int bx, by, bz, bid, bseq, bori, press_side, press_offset, manipulate_type, attack_dir, press_x, press_y, press_z, press_ori, support_x, support_y, support_z, support_ori;

        // Eigen::MatrixXd record(1000, 28);
        // int record_cnt = 0;

        while(ros::ok)
        {
            if (mode >= 4 && mode <= 9)
            {
                if (main_arm == 1) {
                    lego_ptr->update_bricks(robot1_q, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), false, brick_name, 0);
                }
                if (main_arm == 2) {
                    lego_ptr->update_bricks(robot2_q, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), false, brick_name, 0);
                }
            }
            else if (mode >= 19 && mode <= 24)
            {
                if (support_arm == 1) {
                    lego_ptr->update_bricks(robot1_q, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), false, brick_name, 0);
                }
                if (support_arm == 2) {
                    lego_ptr->update_bricks(robot2_q, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), false, brick_name, 0);
                }
            }
            else if (mode >= 25 && mode <= 32)
            {
                if (main_arm == 1) {
                    lego_ptr->update_bricks(robot1_q, lego_ptr->robot_DH_tool_alt_r1(), lego_ptr->robot_base_r1(), false, brick_name, 1);
                }
                if (main_arm == 2) {
                    lego_ptr->update_bricks(robot2_q, lego_ptr->robot_DH_tool_alt_r2(), lego_ptr->robot_base_r2(), false, brick_name, 1);
                }
            }
            if((use_yk && move_on_to_next) || 
               (!use_yk && lego_ptr->robot_reached_goal(robot1_q, r1_cur_goal, lego_ptr->robot_dof_1()) && lego_ptr->robot_is_static(robot1_qd, robot1_qdd, lego_ptr->robot_dof_1()) && 
                lego_ptr->robot_reached_goal(robot2_q, r2_cur_goal, lego_ptr->robot_dof_2()) && lego_ptr->robot_is_static(robot2_qd, robot2_qdd, lego_ptr->robot_dof_2())))
            {
                // Modify stmotion controller runtime
            r1_controller_time_msg.data = jpc_travel_time;
            r1_controller_time_pub.publish(r1_controller_time_msg);
            r2_controller_time_msg.data = jpc_travel_time;
            r2_controller_time_pub.publish(r2_controller_time_msg);

                ROS_INFO_STREAM("Next step");
                if (mode == 9 || mode == 24 || mode == 32)
                {
                    lego_ptr->update_brick_connection();
                }
                if (mode == 14 || mode == 36)
                {
                    if (task_idx >= num_tasks)
                    {
                        break;
                    }
                    mode = 0;
                    task_idx ++;
                }
                if(mode == 0)
                {
                    // chooose task
                    auto node = task_json[std::to_string(task_idx)];
                    bx = node["x"].asInt();
                    by = node["y"].asInt();
                    bz = node["z"].asInt();
                    bid = node["brick_id"].asInt();
                    bseq = node["brick_seq"].asInt();
                    bori = node["ori"].asInt();
                    press_side = node["press_side"].asInt();
                    press_offset = node["press_offset"].asInt();
                    manipulate_type = node["manipulate_type"].asInt();
                    attack_dir = node["attack_dir"].asInt();
                    press_x = node["press_x"].asInt();
                    press_y = node["press_y"].asInt();
                    press_z = node["press_z"].asInt();
                    press_ori = node["press_ori"].asInt();
                    support_x = node["support_x"].asInt();
                    support_y = node["support_y"].asInt();
                    support_z = node["support_z"].asInt();
                    support_ori = node["support_ori"].asInt();

                    if(manipulate_type == 0)
                    {
                        mode = 1;
                    }
                    else
                    {
                        mode = 16;
                    }
                    // No support needed
                    if(support_x == -1 || support_y == -1 || support_z == -1)
                    {
                        support_arm = 0;
                        if(press_side == 1 || press_side == 2)
                        {
                            main_arm = 1;
                        }
                        else{
                            main_arm = 2;
                        }
                    }
                    // Support needed
                    else
                    {
                        // From top
                        if(manipulate_type == 0)
                        {
                            if(support_ori == 0 || support_ori == 1)
                            {
                                support_arm = 1;
                                main_arm = 2;
                            }
                            else
                            {
                                support_arm = 2;
                                main_arm = 1;
                            }
                        }
                        // From bottom
                        else
                        {
                            if(press_side == 1 || press_side == 2)
                            {
                                main_arm = 1;
                                support_arm = 2;
                            }
                            else
                            {
                                main_arm = 2;
                                support_arm = 1;
                            }
                        }
                    }
                }
                else
                {
                    mode++;
                }
                ROS_INFO_STREAM("Mode: " << mode << " Task: " << task_idx << " Robot: " << main_arm << " Support: " << support_arm << " Manipulate: " << manipulate_type << " Brick: " << lego_ptr->get_brick_name_by_id(bid, bseq));

                if (mode == 0 || mode == 14 || mode == 36)
                {
                    r1_cur_goal = home_q;
                    r2_cur_goal = home_q;
                }
                else if (mode == 1)
                {
                    brick_name = lego_ptr->get_brick_name_by_id(bid, bseq);
                    grab_brick = 1;
                    lego_ptr->brick_pose_in_stock(brick_name, press_side, press_offset, cart_T);
                    
                    Eigen::Matrix4d offset_T = Eigen::MatrixXd::Identity(4, 4);
                    offset_T.col(3) << pick_offset(3), pick_offset(4), pick_offset(5) - abs(pick_offset(5)), 1;
                    offset_T = cart_T * offset_T;

                    if (main_arm == 1) {
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, offset_T, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_inv_r1(), 0, IK_status);
                        if(support_arm == 2){
                            
                            lego_ptr->support_pose_down_pre(support_x, support_y, support_z, support_ori, support_T_down_pre);
                            r2_cur_goal = lego_ptr->IK(r2_cur_goal, support_T_down_pre, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_inv_r2(), 0, IK_status);
                        }
                    }
                    else if(main_arm == 2){
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, offset_T, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_inv_r2(), 0, IK_status);
                        if(support_arm == 1){
                            
                            lego_ptr->support_pose_down_pre(support_x, support_y, support_z, support_ori, support_T_down_pre);
                            r1_cur_goal = lego_ptr->IK(r1_cur_goal, support_T_down_pre, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_inv_r1(), 0, IK_status);
                        }
                    }
                }
                else if (mode == 2)
                {
                    Eigen::Matrix4d up_T = Eigen::MatrixXd::Identity(4, 4);
                    up_T.col(3) << 0, 0, pick_offset(5), 1;
                    up_T = cart_T * up_T;
                    
                    if (main_arm == 1) {
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, up_T, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_inv_r1(), 0, IK_status);
                        if(support_arm == 2){
                            
                            lego_ptr->support_pose_down(support_x, support_y, support_z, support_ori, support_T_down);
                            r2_cur_goal = lego_ptr->IK(r2_cur_goal, support_T_down, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_inv_r2(), 0, IK_status);
                        }
                    }
                    else if(main_arm == 2){
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, up_T, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_inv_r2(), 0, IK_status);
                        if(support_arm == 1){
                            
                            lego_ptr->support_pose_down(support_x, support_y, support_z, support_ori, support_T_down);
                            r1_cur_goal = lego_ptr->IK(r1_cur_goal, support_T_down, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_inv_r1(), 0, IK_status);
                        }
                    }
                }
                else if (mode == 3)
                {
                    if (main_arm == 1) {
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, cart_T, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_inv_r1(), 0, IK_status);

                        if(support_arm == 2){
                            lego_ptr->support_pose(support_x, support_y, support_z, support_ori, support_T);
                            r2_cur_goal = lego_ptr->IK(r2_cur_goal, support_T, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_inv_r2(), 0, IK_status);
                        }
                    }
                    else if(main_arm == 2){
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, cart_T, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_inv_r2(), 0, IK_status);
                        if(support_arm == 1){
                            lego_ptr->support_pose(support_x, support_y, support_z, support_ori, support_T);
                            r1_cur_goal = lego_ptr->IK(r1_cur_goal, support_T, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_inv_r1(), 0, IK_status);
                        }
                    }  
                }
                else if (mode == 4)
                {
                    double twist_rad = twist_deg / 180.0 * PI;
                    twist_R << cos(twist_rad), 0, sin(twist_rad), 
                               0, 1, 0, 
                               -sin(twist_rad), 0, cos(twist_rad);
                    twist_T = Eigen::MatrixXd::Identity(4, 4);
                    twist_T.block(0, 0, 3, 3) << twist_R;

                    
                    if (main_arm == 1) {
                        cart_T = lego_manipulation::math::FK(r1_cur_goal, lego_ptr->robot_DH_tool_disassemble_r1(), lego_ptr->robot_base_r1(), false);
                        cart_T = cart_T * twist_T;
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, cart_T, lego_ptr->robot_DH_tool_disassemble_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_disassemble_inv_r1(), 0, IK_status);
                    }
                    else if(main_arm == 2){
                        cart_T = lego_manipulation::math::FK(r2_cur_goal, lego_ptr->robot_DH_tool_disassemble_r2(), lego_ptr->robot_base_r2(), false);
                        cart_T = cart_T * twist_T;
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, cart_T, lego_ptr->robot_DH_tool_disassemble_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_disassemble_inv_r2(), 0, IK_status);
                    }  
                }
                else if(mode == 5)
                {
                    if (main_arm == 1) {
                        cart_T = lego_manipulation::math::FK(r1_cur_goal, lego_ptr->robot_DH_tool_assemble_r1(), lego_ptr->robot_base_r1(), false);
                        cart_T(2, 3) = cart_T(2, 3) + 0.015;
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, cart_T, lego_ptr->robot_DH_tool_assemble_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_assemble_inv_r1(), 0, IK_status);
                    }
                    if (main_arm == 2) {
                        cart_T = lego_manipulation::math::FK(r2_cur_goal, lego_ptr->robot_DH_tool_assemble_r2(), lego_ptr->robot_base_r2(), false);
                        cart_T(2, 3) = cart_T(2, 3) + 0.015;
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, cart_T, lego_ptr->robot_DH_tool_assemble_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_assemble_inv_r2(), 0, IK_status);
                    }
                }
                else if(mode == 6)
                {
                    if (main_arm == 1) {
                        r1_cur_goal = home_q;
                    }
                    if (main_arm == 2) {
                        r2_cur_goal = home_q;
                    }
                }
                else if(mode == 7)
                {
                    lego_ptr->assemble_pose_from_top(press_x, press_y, press_z, press_ori, press_side, cart_T);
                    Eigen::Matrix4d offset_T = Eigen::MatrixXd::Identity(4, 4);
                    offset_T.col(3) << pick_offset(0), pick_offset(1) * attack_dir, pick_offset(2) - abs(pick_offset(2)), 1;
                    offset_T = cart_T * offset_T;

                    if (main_arm == 1) {
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, offset_T, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_inv_r1(), 0, IK_status);
                    }
                    if (main_arm == 2) {
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, offset_T, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_inv_r2(), 0, IK_status);
                    }
                }
                else if(mode == 8)
                {
                    Eigen::Matrix4d up_T = Eigen::MatrixXd::Identity(4, 4);
                    up_T.col(3) << 0, 0, pick_offset(2), 1;
                    up_T = cart_T * up_T;

                    if (main_arm == 1) {
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, up_T, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_inv_r1(), 0, IK_status);
                    }
                    if (main_arm == 2) {
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, up_T, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_inv_r2(), 0, IK_status);
                    }
                }
                else if(mode == 9)
                {
                    if (main_arm == 1) {
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, cart_T, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_inv_r1(), 0, IK_status);
                    }
                    if (main_arm == 2) {
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, cart_T, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_inv_r2(), 0, IK_status);
                    }
                }
                else if(mode == 10)
                {
                    double twist_rad = twist_deg / 180.0 * PI;
                    twist_R << cos(-twist_rad), 0, sin(-twist_rad), 
                               0, 1, 0, 
                               -sin(-twist_rad), 0, cos(-twist_rad);
                    twist_T = Eigen::MatrixXd::Identity(4, 4);
                    twist_T.block(0, 0, 3, 3) << twist_R;

                    if (main_arm == 1) {
                        cart_T = lego_manipulation::math::FK(r1_cur_goal, lego_ptr->robot_DH_tool_assemble_r1(), lego_ptr->robot_base_r1(), false);
                        cart_T = cart_T * twist_T;
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, cart_T, lego_ptr->robot_DH_tool_assemble_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_assemble_inv_r1(), 0, IK_status);
                    }
                    if (main_arm == 2) {
                        cart_T = lego_manipulation::math::FK(r2_cur_goal, lego_ptr->robot_DH_tool_assemble_r2(), lego_ptr->robot_base_r2(), false);
                        cart_T = cart_T * twist_T;
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, cart_T, lego_ptr->robot_DH_tool_assemble_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_assemble_inv_r2(), 0, IK_status);
                    }
                }
                else if(mode == 11)
                {
                    if (main_arm == 1) {
                        cart_T = lego_manipulation::math::FK(r1_cur_goal, lego_ptr->robot_DH_tool_assemble_r1(), lego_ptr->robot_base_r1(), false);
                        cart_T(2, 3) = cart_T(2, 3) + 0.015;
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, cart_T, lego_ptr->robot_DH_tool_assemble_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_assemble_inv_r1(), 0, IK_status);
                    }
                    if (main_arm == 2) {
                        cart_T = lego_manipulation::math::FK(r2_cur_goal, lego_ptr->robot_DH_tool_assemble_r2(), lego_ptr->robot_base_r2(), false);
                        cart_T(2, 3) = cart_T(2, 3) + 0.015;
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, cart_T, lego_ptr->robot_DH_tool_assemble_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_assemble_inv_r2(), 0, IK_status);
                    }
                }
                else if(mode == 12)
                {
                    if (support_arm == 1) {
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, support_T_down, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_inv_r1(), 0, IK_status);
                    }
                    if (support_arm == 2) {
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, support_T_down, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_inv_r2(), 0, IK_status);
                    }
                }
                else if(mode == 13)
                {
                    if(main_arm == 1){
                        r1_cur_goal = home_q;
                        if (support_arm == 2) {
                            r2_cur_goal = lego_ptr->IK(r2_cur_goal, support_T_down_pre, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_inv_r2(), 0, IK_status);
                        }
                    }
                    if(main_arm == 2){
                        r2_cur_goal = home_q;
                        if (support_arm == 1) {
                            r1_cur_goal = lego_ptr->IK(r1_cur_goal, support_T_down_pre, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_inv_r1(), 0, IK_status);
                        }
                    }
                }
                else if(mode == 16)
                {
                    brick_name = lego_ptr->get_brick_name_by_id(bid, bseq);
                    lego_ptr->brick_pose_in_stock(brick_name, press_side, press_offset, cart_T);
                    
                    Eigen::Matrix4d offset_T = Eigen::MatrixXd::Identity(4, 4);
                    offset_T.col(3) << pick_offset(3), pick_offset(4), pick_offset(5) - abs(pick_offset(5)), 1;
                    offset_T = cart_T * offset_T;
                    if(main_arm == 1)
                    {
                        r1_cur_goal = home_receive_q;
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, offset_T, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_inv_r2(), 0, IK_status);
                    }
                    else if(main_arm == 2)
                    {
                        r2_cur_goal = home_receive_q;
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, offset_T, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_inv_r1(), 0, IK_status);
                    }
                }
                else if(mode == 17)
                {
                    Eigen::Matrix4d up_T = Eigen::MatrixXd::Identity(4, 4);
                    up_T.col(3) << 0, 0, pick_offset(5), 1;
                    up_T = cart_T * up_T;
                    
                    if (main_arm == 1) {
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, up_T, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_inv_r2(), 0, IK_status);
                    }
                    else if(main_arm == 2){
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, up_T, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_inv_r1(), 0, IK_status);
                    }
                }
                else if(mode == 18)
                {
                    if (main_arm == 1) {
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, cart_T, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_inv_r2(), 0, IK_status);
                    }
                    else if(main_arm == 2){
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, cart_T, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_inv_r1(), 0, IK_status);
                    }
                }
                else if(mode == 19)
                {
                    double twist_rad = twist_deg / 180.0 * PI;
                    twist_R << cos(twist_rad), 0, sin(twist_rad), 
                               0, 1, 0, 
                               -sin(twist_rad), 0, cos(twist_rad);
                    twist_T = Eigen::MatrixXd::Identity(4, 4);
                    twist_T.block(0, 0, 3, 3) << twist_R;

                    if (main_arm == 1) {
                        cart_T = lego_manipulation::math::FK(r2_cur_goal, lego_ptr->robot_DH_tool_disassemble_r2(), lego_ptr->robot_base_r2(), false);
                        cart_T = cart_T * twist_T;
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, cart_T, lego_ptr->robot_DH_tool_disassemble_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_disassemble_inv_r2(), 0, IK_status);
                    }
                    else if(main_arm == 2){
                        cart_T = lego_manipulation::math::FK(r1_cur_goal, lego_ptr->robot_DH_tool_disassemble_r1(), lego_ptr->robot_base_r1(), false);
                        cart_T = cart_T * twist_T;
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, cart_T, lego_ptr->robot_DH_tool_disassemble_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_disassemble_inv_r1(), 0, IK_status);
                    }
                }
                else if(mode == 20)
                {
                    if (main_arm == 1) {
                        cart_T = lego_manipulation::math::FK(r2_cur_goal, lego_ptr->robot_DH_tool_assemble_r2(), lego_ptr->robot_base_r2(), false);
                        cart_T(2, 3) = cart_T(2, 3) + 0.015;
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, cart_T, lego_ptr->robot_DH_tool_assemble_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_assemble_inv_r2(), 0, IK_status);
                    }
                    else if (main_arm == 2) {
                        cart_T = lego_manipulation::math::FK(r1_cur_goal, lego_ptr->robot_DH_tool_assemble_r1(), lego_ptr->robot_base_r1(), false);
                        cart_T(2, 3) = cart_T(2, 3) + 0.015;
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, cart_T, lego_ptr->robot_DH_tool_assemble_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_assemble_inv_r1(), 0, IK_status);
                    }
                }
                else if(mode == 21)
                {
                    if(main_arm == 1){
                        r2_cur_goal = home_q;
                    }
                    else if(main_arm == 2){
                        r1_cur_goal = home_q;
                    }
                }
                else if(mode == 22)
                {
                    if(main_arm == 1){
                        r1_cur_goal = receive_q;
                        r2_cur_goal = home_handover_q;
                    }
                    else if(main_arm == 2){
                        r1_cur_goal = home_handover_q;
                        r2_cur_goal = receive_q;
                    }
                }
                else if(mode == 23)
                {
                    if(main_arm == 1){
                        cart_T = lego_manipulation::math::FK(r1_cur_goal, lego_ptr->robot_DH_tool_alt_r1(), lego_ptr->robot_base_r1(), false);
                        cart_T = cart_T * y_p90 * z_180;
                        Eigen::Matrix4d up_T = cart_T;
                        up_T(2, 3) = up_T(2, 3) + 0.015;
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, up_T, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_inv_r2(), 0, IK_status);
                    }
                    else if(main_arm == 2){
                        cart_T = lego_manipulation::math::FK(r2_cur_goal, lego_ptr->robot_DH_tool_alt_r2(), lego_ptr->robot_base_r2(), false);
                        cart_T = cart_T * y_p90 * z_180;
                        Eigen::Matrix4d up_T = cart_T;
                        up_T(2, 3) = up_T(2, 3) + 0.015;
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, up_T, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_inv_r1(), 0, IK_status);
                    }
                }
                else if(mode == 24)
                {
                    if(main_arm == 1){
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, cart_T, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_inv_r2(), 0, IK_status);
                    }
                    else if(main_arm == 2){
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, cart_T, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_inv_r1(), 0, IK_status);
                    }
                }
                else if(mode == 25)
                {
                    double twist_rad = twist_deg / 180.0 * PI;
                    twist_R << cos(-twist_rad), 0, sin(-twist_rad), 
                               0, 1, 0, 
                               -sin(-twist_rad), 0, cos(-twist_rad);
                    twist_T = Eigen::MatrixXd::Identity(4, 4);
                    twist_T.block(0, 0, 3, 3) << twist_R;

                    if (main_arm == 1) {
                        cart_T = lego_manipulation::math::FK(r2_cur_goal, lego_ptr->robot_DH_tool_assemble_r2(), lego_ptr->robot_base_r2(), false);
                        cart_T = cart_T * twist_T;
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, cart_T, lego_ptr->robot_DH_tool_assemble_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_assemble_inv_r2(), 0, IK_status);
                    }
                    else if (main_arm == 2) {
                        cart_T = lego_manipulation::math::FK(r1_cur_goal, lego_ptr->robot_DH_tool_assemble_r1(), lego_ptr->robot_base_r1(), false);
                        cart_T = cart_T * twist_T;
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, cart_T, lego_ptr->robot_DH_tool_assemble_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_assemble_inv_r1(), 0, IK_status);
                    }
                }
                else if(mode == 26)
                {
                    if(main_arm == 1){
                        cart_T = lego_manipulation::math::FK(r2_cur_goal, lego_ptr->robot_DH_tool_assemble_r2(), lego_ptr->robot_base_r2(), false);
                        Eigen::Matrix4d up_T = cart_T;
                        up_T(2, 3) = up_T(2, 3) + 0.015;
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, up_T, lego_ptr->robot_DH_tool_assemble_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_assemble_inv_r2(), 0, IK_status);
                    }
                    else if(main_arm == 2){
                        cart_T = lego_manipulation::math::FK(r1_cur_goal, lego_ptr->robot_DH_tool_assemble_r1(), lego_ptr->robot_base_r1(), false);
                        Eigen::Matrix4d up_T = cart_T;
                        up_T(2, 3) = up_T(2, 3) + 0.015;
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, up_T, lego_ptr->robot_DH_tool_assemble_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_assemble_inv_r1(), 0, IK_status);
                    }
                }
                else if(mode == 27)
                {
                    if(main_arm == 1){
                        r2_cur_goal = home_handover_q;
                    }
                    else if(main_arm == 2){
                        r1_cur_goal = home_handover_q;
                    }
                }
                else if(mode == 28)
                {
                    if(main_arm == 1){
                        r1_cur_goal = home_receive_q;
                        r2_cur_goal = home_q;
                    }
                    else if(main_arm == 2){
                        r1_cur_goal = home_q;
                        r2_cur_goal = home_receive_q;
                    }
                }
                else if(mode == 29)
                {
                    lego_ptr->assemble_pose_from_top(press_x, press_y, press_z+2, press_ori, press_side, cart_T);
                    cart_T = cart_T * y_p90 * z_180;
                    Eigen::Matrix4d pre_T = Eigen::MatrixXd::Identity(4, 4);
                    pre_T.col(3) << -(pick_offset(5) - abs(pick_offset(5))), attack_dir * (-pick_offset(4)), pick_offset(3) - 0.05, 1;
                    pre_T = cart_T * pre_T;
                    if(main_arm == 1){
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, pre_T, lego_ptr->robot_DH_tool_alt_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_alt_inv_r1(), 0, IK_status);
                    }
                    else if(main_arm == 2){
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, pre_T, lego_ptr->robot_DH_tool_alt_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_alt_inv_r2(), 0, IK_status);
                    }
                }
                else if(mode == 30)
                {
                    Eigen::Matrix4d offset_T = Eigen::MatrixXd::Identity(4, 4);
                    offset_T.col(3) << -(pick_offset(5) - abs(pick_offset(5))), attack_dir * (-pick_offset(4)), pick_offset(3), 1;
                    offset_T = cart_T * offset_T;

                    if(support_ori == 0){
                        lego_ptr->assemble_pose_from_top(support_x+1, support_y, support_z+1, support_ori, 1, press_T);
                    }
                    else if(support_ori == 1){
                        lego_ptr->assemble_pose_from_top(support_x, support_y+1, support_z+1, support_ori, 1, press_T);
                    }
                    press_T(2, 3) = press_T(2, 3) - lego_ptr->brick_height() + lego_ptr->lever_wall_height() + lego_ptr->knob_height();
                    press_up_T = press_T;
                    press_up_T(2, 3) = press_up_T(2, 3) + 0.015;

                    if(main_arm == 1){
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, offset_T, lego_ptr->robot_DH_tool_alt_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_alt_inv_r1(), 0, IK_status);
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, press_up_T, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_inv_r2(), 0, IK_status);
                    }
                    else if(main_arm == 2){
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, offset_T, lego_ptr->robot_DH_tool_alt_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_alt_inv_r2(), 0, IK_status);
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, press_up_T, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_inv_r1(), 0, IK_status);
                    }
                }
                else if(mode == 31)
                {
                    Eigen::Matrix4d down_T = Eigen::MatrixXd::Identity(4, 4);
                    down_T.col(3) << -pick_offset(5), 0, 0, 1;
                    down_T = cart_T * down_T;
                    
                    if (main_arm == 1) {
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, down_T, lego_ptr->robot_DH_tool_alt_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_alt_inv_r1(), 0, IK_status);
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, press_T, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_inv_r2(), 0, IK_status);
                    }
                    else if(main_arm == 2){
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, down_T, lego_ptr->robot_DH_tool_alt_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_alt_inv_r2(), 0, IK_status);
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, press_T, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_inv_r1(), 0, IK_status);
                    }
                }
                else if(mode == 32)
                {
                    if (main_arm == 1) {
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, cart_T, lego_ptr->robot_DH_tool_alt_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_alt_inv_r1(), 0, IK_status);
                    }
                    else if(main_arm == 2){
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, cart_T, lego_ptr->robot_DH_tool_alt_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_alt_inv_r2(), 0, IK_status);
                    }
                }
                else if(mode == 33)
                {
                    double twist_rad = twist_deg / 180.0 * PI;
                    twist_R << cos(twist_rad), 0, sin(twist_rad), 
                               0, 1, 0, 
                               -sin(twist_rad), 0, cos(twist_rad);
                    twist_T = Eigen::MatrixXd::Identity(4, 4);
                    twist_T.block(0, 0, 3, 3) << twist_R;

                    if (main_arm == 1) {
                        cart_T = lego_manipulation::math::FK(r1_cur_goal, lego_ptr->robot_DH_tool_alt_assemble_r1(), lego_ptr->robot_base_r1(), false);
                        cart_T = cart_T * twist_T;
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, cart_T, lego_ptr->robot_DH_tool_alt_assemble_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_alt_assemble_inv_r1(), 0, IK_status);
                    }
                    else if (main_arm == 2) {
                        cart_T = lego_manipulation::math::FK(r2_cur_goal, lego_ptr->robot_DH_tool_alt_assemble_r2(), lego_ptr->robot_base_r2(), false);
                        cart_T = cart_T * twist_T;
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, cart_T, lego_ptr->robot_DH_tool_alt_assemble_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_alt_assemble_inv_r2(), 0, IK_status);
                    }
                }
                else if(mode == 34)
                {
                    if(main_arm == 1){
                        cart_T = lego_manipulation::math::FK(r1_cur_goal, lego_ptr->robot_DH_tool_alt_assemble_r1(), lego_ptr->robot_base_r1(), false);
                        Eigen::Matrix4d down_T = Eigen::MatrixXd::Identity(4, 4);
                        down_T << 1, 0, 0, 0.015,
                                  0, 1, 0, 0,
                                  0, 0, 1, -0.015,
                                  0, 0, 0, 1;
                        down_T = cart_T * down_T;

                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, down_T, lego_ptr->robot_DH_tool_alt_assemble_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_alt_assemble_inv_r1(), 0, IK_status);
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, press_up_T, lego_ptr->robot_DH_tool_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_inv_r2(), 0, IK_status);
                    }
                    else if(main_arm == 2){
                        cart_T = lego_manipulation::math::FK(r2_cur_goal, lego_ptr->robot_DH_tool_alt_assemble_r2(), lego_ptr->robot_base_r2(), false);
                        Eigen::Matrix4d down_T = Eigen::MatrixXd::Identity(4, 4);
                        down_T << 1, 0, 0, 0.015,
                                  0, 1, 0, 0,
                                  0, 0, 1, -0.015,
                                  0, 0, 0, 1;
                        down_T = cart_T * down_T;
                        r2_cur_goal = lego_ptr->IK(r2_cur_goal, down_T, lego_ptr->robot_DH_tool_alt_assemble_r2(), lego_ptr->robot_base_r2(), lego_ptr->robot_base_inv_r2(),
                                                    lego_ptr->robot_tool_alt_assemble_inv_r2(), 0, IK_status);
                        r1_cur_goal = lego_ptr->IK(r1_cur_goal, press_up_T, lego_ptr->robot_DH_tool_r1(), lego_ptr->robot_base_r1(), lego_ptr->robot_base_inv_r1(),
                                                    lego_ptr->robot_tool_inv_r1(), 0, IK_status);
                    }
                }
                else if(mode == 35)
                {
                    if(main_arm == 1){
                        r1_cur_goal = home_q;
                        r2_cur_goal = home_q;
                    }
                    else if(main_arm == 2){
                        r1_cur_goal = home_q;
                        r2_cur_goal = home_q;
                    }
                }
                std::cout<<IK_status<<std::endl;
                assert(IK_status);
                
                
                
                
                Eigen::MatrixXd r1_cart_T_goal = lego_manipulation::math::FK(r1_cur_goal, lego_ptr->robot_DH_r1(), lego_ptr->robot_base_r1(), false);
                Eigen::Matrix3d r1_goal_rot = r1_cart_T_goal.block(0, 0, 3, 3);
                Eigen::Quaterniond r1_quat(r1_goal_rot);

                Eigen::MatrixXd r2_cart_T_goal = lego_manipulation::math::FK(r2_cur_goal, lego_ptr->robot_DH_r2(), lego_ptr->robot_base_r2(), false);
                Eigen::Matrix3d r2_goal_rot = r2_cart_T_goal.block(0, 0, 3, 3);
                Eigen::Quaterniond r2_quat(r2_goal_rot);

                // record.row(record_cnt) << use_r1, r1_cart_T_goal(0, 3), r1_cart_T_goal(1, 3), r1_cart_T_goal(2, 3), r1_quat.x(), r1_quat.y(), r1_quat.z(), r1_quat.w(),
                //                           r1_cur_goal(0), r1_cur_goal(1), r1_cur_goal(2), r1_cur_goal(3), r1_cur_goal(4), r1_cur_goal(5),
                //                           use_r2, r2_cart_T_goal(0, 3), r2_cart_T_goal(1, 3), r2_cart_T_goal(2, 3), r2_quat.x(), r2_quat.y(), r2_quat.z(), r2_quat.w(),
                //                           r2_cur_goal(0), r2_cur_goal(1), r2_cur_goal(2), r2_cur_goal(3), r2_cur_goal(4), r2_cur_goal(5);
                // record_cnt ++;
            }

            // Send to Stmotion controller
            r1_goal_msg.data.clear();
            for(int j=0; j<lego_ptr->robot_dof_1(); j++)
            {
                r1_goal_msg.data.push_back(r1_cur_goal(j));
            }
            r1_goal_pub.publish(r1_goal_msg);

            r2_goal_msg.data.clear();
            for(int j=0; j<lego_ptr->robot_dof_2(); j++)
            {
                r2_goal_msg.data.push_back(r2_cur_goal(j));
            }
            r2_goal_pub.publish(r2_goal_msg);

            Eigen::MatrixXd r1_cart_T_goal = lego_manipulation::math::FK(r1_cur_goal, lego_ptr->robot_DH_r1(), lego_ptr->robot_base_r1(), false);
            Eigen::Matrix3d r1_goal_rot = r1_cart_T_goal.block(0, 0, 3, 3);
            Eigen::Quaterniond r1_quat(r1_goal_rot);

            Eigen::MatrixXd r2_cart_T_goal = lego_manipulation::math::FK(r2_cur_goal, lego_ptr->robot_DH_r2(), lego_ptr->robot_base_r2(), false);
            Eigen::Matrix3d r2_goal_rot = r2_cart_T_goal.block(0, 0, 3, 3);
            Eigen::Quaterniond r2_quat(r2_goal_rot);

            // Send to yaskawa service
            if (use_yk)
            {
                geometry_msgs::Pose r1_goal_pose;
                geometry_msgs::Pose r2_goal_pose;
                if (move_on_to_next)
                {
                    r1_goal_pose.position.x = r1_cart_T_goal(0, 3);
                    r1_goal_pose.position.y = r1_cart_T_goal(1, 3);
                    r1_goal_pose.position.z = r1_cart_T_goal(2, 3);
                    r1_goal_pose.orientation.x = r1_quat.x();
                    r1_goal_pose.orientation.y = r1_quat.y();
                    r1_goal_pose.orientation.z = r1_quat.z();
                    r1_goal_pose.orientation.w = r1_quat.w();

                    r2_goal_pose.position.x = r2_cart_T_goal(0, 3);
                    r2_goal_pose.position.y = r2_cart_T_goal(1, 3);
                    r2_goal_pose.position.z = r2_cart_T_goal(2, 3);
                    r2_goal_pose.orientation.x = r2_quat.x();
                    r2_goal_pose.orientation.y = r2_quat.y();
                    r2_goal_pose.orientation.z = r2_quat.z();
                    r2_goal_pose.orientation.w = r2_quat.w();
                    move_on_to_next = false;
                    ROS_INFO_STREAM("Sending pose r1: " << r1_goal_pose);
                    ROS_INFO_STREAM("Sending pose r2: " << r2_goal_pose);
                }

                srv.request.base_frame = "base_link";
                srv.request.pose = r1_goal_pose;
                if (client.call(srv))
                {
                    move_on_to_next = true;
                    ROS_INFO_STREAM("Pose Set to: " << srv.response.pose);
                }
            }
        }
        // lego_manipulation::io::SaveMatToFile(record.block(0, 0, record_cnt, 28), root_pwd + "/record_steps.txt");
        usleep(1000 * second);
        ROS_INFO_STREAM("Task Execution Done!");
        ros::shutdown();
        return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}


