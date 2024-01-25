#include "Lego.hpp"
// HEADER FILES FOR SERVICE INTERFACE WITH YK ROBOT
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/WrenchStamped.h"
#include "lego_manipulation/SetPose.h"

// HEADER FILES FOR API INTERFACE WITH YK ROBOT
// #include "yk_api/yk_interface.h"  // Uncomment this if using yaskawa robot

using namespace std::chrono;

lego_manipulation::math::VectorJd robot_q = Eigen::MatrixXd::Zero(6, 1);
lego_manipulation::math::VectorJd robot_qd = Eigen::MatrixXd::Zero(6, 1);
lego_manipulation::math::VectorJd robot_qdd = Eigen::MatrixXd::Zero(6, 1);
int fts_buffer_size = 2;
Eigen::MatrixXd fts_buffer = Eigen::MatrixXd::Zero(6, fts_buffer_size);
int fts_buffer_idx = 0;
lego_manipulation::math::VectorJd fts_val = Eigen::MatrixXd::Zero(6, 1);
lego_manipulation::math::VectorJd fts_val_one_step = Eigen::MatrixXd::Zero(6, 1);

void ftsCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    double fx = msg->wrench.force.x;
    double fy = msg->wrench.force.y;
    double fz = msg->wrench.force.z;
    double tx = msg->wrench.torque.x;
    double ty = msg->wrench.torque.y;
    double tz = msg->wrench.torque.z;
    if(fts_buffer_idx < fts_buffer_size)
    {
        fts_buffer.col(fts_buffer_idx) << fx, fy, fz, tx, ty, tz;
        fts_buffer_idx++;
    }
    else
    {
        fts_buffer.block(0, 0, 6, fts_buffer_size - 1) << fts_buffer.block(0, 1, 6, fts_buffer_size - 1);
        fts_buffer.col(fts_buffer_size - 1) << fx, fy, fz, tx, ty, tz;
    }
    fts_val = fts_buffer.rowwise().mean();
    fts_val_one_step << fx, fy, fz, tx, ty, tz;
}

// Get robot state from stmotion controller.
void robotStateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    robot_q << msg->data[0], msg->data[3], msg->data[6], msg->data[9], msg->data[12], msg->data[15];
    robot_qd << msg->data[1], msg->data[4], msg->data[7], msg->data[10], msg->data[13], msg->data[16];
    robot_qdd << msg->data[2], msg->data[5], msg->data[8], msg->data[11], msg->data[14], msg->data[17];
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

        std::string config_fname, root_pwd, task_fname, DH_fname, DH_tool_fname, DH_tool_assemble_fname, DH_tool_disassemble_fname, lego_lib_fname, 
                    robot_base_fname, env_setup_fname, robot_state_topic, controller_joint_goal_topic, controller_cart_goal_topic, 
                    controller_tracking_space_topic, controller_time_topic;
        int tracking_space = 0; // 0: joint, 1: Cart
        bool IK_status;
        ros::ServiceClient set_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        private_node_handle.getParam("config_fname", config_fname);
        private_node_handle.getParam("root_pwd", root_pwd);

        std::ifstream config_file(config_fname, std::ifstream::binary);
        Json::Value config;
        config_file >> config;
        DH_fname = root_pwd + config["DH_fname"].asString();
        DH_tool_fname = root_pwd + config["DH_tool_fname"].asString();
        DH_tool_assemble_fname = root_pwd + config["DH_tool_assemble_fname"].asString();
        DH_tool_disassemble_fname = root_pwd + config["DH_tool_disassemble_fname"].asString();
        robot_base_fname = root_pwd + config["Robot_Base_fname"].asString();
        env_setup_fname = root_pwd + config["Env_Setup_fname"].asString();
        task_fname = root_pwd + config["Task_Graph_fname"].asString();
        lego_lib_fname = root_pwd + config["lego_lib_fname"].asString();
        robot_state_topic = config["ST_Controller_Topic"]["State"].asString();
        controller_joint_goal_topic = config["ST_Controller_Topic"]["Joint_Goal"].asString();
        controller_cart_goal_topic = config["ST_Controller_Topic"]["Cart_Goal"].asString();
        controller_time_topic = config["ST_Controller_Topic"]["Controller_Time"].asString();
        controller_tracking_space_topic = config["ST_Controller_Topic"]["Tracking_Space"].asString();
        bool infinite_tasks = config["Infinite_Tasks"].asBool();
        bool assemble = config["Start_with_Assemble"].asBool();
        bool fts_feedback = config["FTS_feedback"].asBool();
        bool use_yk = config["Use_yk"].asBool();
        int twist_deg = config["Twist_Deg"].asInt();
        int premanipulation_idx = -1;
        int reached_premanipulation = 0;


        std::ifstream task_file(task_fname, std::ifstream::binary);
        Json::Value task_json;
        task_file >> task_json;

        lego_manipulation::lego::Lego::Ptr lego_ptr = std::make_shared<lego_manipulation::lego::Lego>();
        lego_ptr->setup(env_setup_fname, lego_lib_fname, assemble, task_json, 
                        DH_fname, DH_tool_fname, DH_tool_disassemble_fname, DH_tool_assemble_fname, 
                        robot_base_fname, 1, set_state_client);
        Eigen::MatrixXd twist_R = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd twist_T = Eigen::MatrixXd::Identity(4, 4);
        Eigen::MatrixXd cart_T = Eigen::MatrixXd::Identity(4, 4);
        
        double incremental_deg = twist_deg;
        int twist_idx = 0;
        int twist_num = twist_deg / incremental_deg;
        int grab_brick;

        ros::ServiceClient client = nh.serviceClient<lego_manipulation::SetPose>("yk_set_pose");
        lego_manipulation::SetPose srv;
        ros::Publisher goal_pub = nh.advertise<std_msgs::Float32MultiArray>(controller_joint_goal_topic, lego_ptr->robot_dof());
        ros::Publisher controller_time_pub = nh.advertise<std_msgs::Float64>(controller_time_topic, 1);
        ros::Publisher controller_tracking_space_pub = nh.advertise<std_msgs::Int64>(controller_tracking_space_topic, 1);
        ros::Publisher reached_premanipulation_pub = nh.advertise<std_msgs::Int64>("/reached_premanipulation", 1);
        ros::Publisher premanipulation_idx_pub = nh.advertise<std_msgs::Int64>("/premanipulation_idx", 1);
        ros::Subscriber robot_state_sub = nh.subscribe(robot_state_topic, lego_ptr->robot_dof() * 3, robotStateCallback);
        ros::Subscriber fts_sub = nh.subscribe("/fts", 1, ftsCallback);
        ros::Publisher fts_avg_pub = nh.advertise<std_msgs::Float32MultiArray>("/fts_recv_avg", 6);
        ros::Publisher fts_one_step_pub = nh.advertise<std_msgs::Float32MultiArray>("/fts_recv_one_step", 6);
        std_msgs::Float32MultiArray goal_msg;
        std_msgs::Float64 controller_time_msg;
        std_msgs::Int64 controller_tracking_space_msg;
        std_msgs::Float32MultiArray fts_avg_msg;
        std_msgs::Float32MultiArray fts_one_step_msg;
        std_msgs::Int64 premanipulation_idx_msg;
        std_msgs::Int64 reached_premanipulation_msg;
        
        int num_tasks = task_json.size();
        Eigen::MatrixXd home_q(lego_ptr->robot_dof(), 1);
        home_q.col(0) << 0, 0, 0, 0, -90, 0;
        Eigen::Matrix4d home_T = lego_manipulation::math::FK(home_q, lego_ptr->robot_DH(), lego_ptr->robot_base(), false);
        home_T.col(3) << 0.3, 0, 0.4, 1; // Home X, Y, Z in base frame of the Flange
        home_q = lego_manipulation::math::IK(home_q, home_T.block(0, 3, 3, 1), home_T.block(0, 0, 3, 3),
                                             lego_ptr->robot_DH(), lego_ptr->robot_base(), 0, 10e6, 10e-3*5); // Home
        lego_manipulation::math::VectorJd pick_offset = Eigen::MatrixXd::Zero(6, 1);

        // Attack angle
        pick_offset << -0.005, 0.005, -0.005,  // place brick offset
                       -0.005, 0.005, -0.0028; // grab brick offset
        // FTS feedback param
        double nominal_x_force = 0.8;
        double nominal_y_force = 1.2;
        double nominal_z_force = -12;
        double force_tolerance = 3;

        int mode = 0; // 0:home 1:pick tilt up 2:pick_up 3:pick_down, 4:pick_twist 5:pick_twist_up 6:home
                      // 7:drop tilt up 8:drop_up 9:drop_down 10:drop_twist 11:drop_twist_up 12:done
        int task_idx;
        if(assemble)
        {
            task_idx = 1;
        }
        else
        {
            task_idx = num_tasks;
        }
        
        lego_manipulation::math::VectorJd cur_goal = home_q;
        std::string brick_name;
        bool move_on_to_next = use_yk;
        
        while(ros::ok)
        {
            if (mode >= 4 && mode <= 9)
            {
                lego_ptr->update_bricks(robot_q, lego_ptr->robot_DH_tool(), lego_ptr->robot_base(), false, brick_name);
            }
            if((use_yk && move_on_to_next) || (!use_yk && lego_ptr->robot_reached_goal(robot_q, cur_goal) && lego_ptr->robot_is_static(robot_qd, robot_qdd)))
            {
                if (mode == 9)
                {
                    lego_ptr->update_brick_connection();
                }
                if(mode == 2 || mode == 8)
                {
                    premanipulation_idx ++;
                    reached_premanipulation_msg.data = 1;
                    premanipulation_idx_msg.data = premanipulation_idx;
                }
                else
                {
                    reached_premanipulation_msg.data = 0;
                }
                if (mode == 12)
                {
                    if (task_idx >= num_tasks && !infinite_tasks && assemble)
                    {
                        break;
                    }
                    else if (task_idx <= 1 && !infinite_tasks && !assemble)
                    {
                        break;
                    }
                    else if (task_idx >= num_tasks && assemble)
                    {
                        task_idx++;
                        assemble = !assemble;
                    }
                    else if (task_idx <= 1 && !assemble)
                    {
                        task_idx--;
                        assemble = !assemble;
                        std::ifstream task_file(task_fname, std::ifstream::binary);
                        Json::Value task_json;
                        task_file >> task_json;
                        num_tasks = task_json.size();
                        ROS_INFO_STREAM("Num tasks: " << num_tasks);
                    }
                    mode = 0;
                    if (assemble)
                    {
                        task_idx++;
                    }
                    else
                    {
                        task_idx--;
                    }
                }
                else if (mode == 4 || mode == 10)
                {
                    twist_idx++;
                    if (twist_idx == twist_num)
                    {
                        mode++;
                        twist_idx = 0;
                    }
                }
                else if(mode == 3 || mode == 9)
                {
                    if(fts_feedback)
                    {
                        usleep(0.5 * second);
                        ROS_INFO_STREAM("Before pose: " << cart_T(0, 3) << " " << cart_T(1, 3) << " " << cart_T(2, 3));
                        ROS_INFO_STREAM("Force: " << fts_val_one_step(0) << " " << fts_val_one_step(1) << " " << fts_val_one_step(2));
                        if(abs(fts_val_one_step(2) - (nominal_z_force)) < force_tolerance)//abs(fts_val(0) - 1.2) < 0.1 && abs(fts_val(1) - 0.6) < 0.1 && abs(fts_val(2) - (-5)) < 0.1)
                        {
                            mode++;
                        }
                        else if(fts_val_one_step(2) < nominal_z_force) // Lift up
                        {
                            Eigen::Matrix4d dT = Eigen::MatrixXd::Identity(4, 4);
                            dT.col(3) << 0, 0, -0.0005, 1;
                            cart_T = cart_T * dT;
                        }
                        else if(fts_val_one_step(2) > nominal_z_force) // Press down
                        {
                            Eigen::Matrix4d dT = Eigen::MatrixXd::Identity(4, 4);
                            dT.col(3) << 0, 0, 0.0001, 1;
                            cart_T = cart_T * dT;
                        }
                        ROS_INFO_STREAM("After pose: " << cart_T(0, 3) << " " << cart_T(1, 3) << " " << cart_T(2, 3));
                    }
                    else{
                        mode++;
                    }
                }
                else
                {
                    mode++;
                }

                reached_premanipulation_pub.publish(reached_premanipulation_msg);
                premanipulation_idx_pub.publish(premanipulation_idx_msg);

                // Modify stmotion controller tracking space
                controller_tracking_space_msg.data = 0;
                controller_tracking_space_pub.publish(controller_tracking_space_msg);
                // Modify stmotion controller runtime
                controller_time_msg.data = 0.5;
                controller_time_pub.publish(controller_time_msg);
                
                ROS_INFO_STREAM("Mode: " << mode << " Task: " << task_idx);

                if (mode == 0 || mode == 6 || mode == 12)
                {
                    cur_goal = home_q;
                }
                else if (mode == 1)
                {
                    auto cur_graph_node = task_json[std::to_string(task_idx)];
                    brick_name = lego_ptr->get_brick_name_by_id(cur_graph_node["brick_id"].asInt(), cur_graph_node["brick_seq"].asInt());
                    grab_brick = 1;
                    lego_ptr->calc_brick_grab_pose(brick_name, assemble, grab_brick,
                                                          cur_graph_node["x"].asInt(),
                                                          cur_graph_node["y"].asInt(),
                                                          cur_graph_node["z"].asInt(),
                                                          cur_graph_node["ori"].asInt(),
                                                          cur_graph_node["press_side"].asInt(), cart_T);
                    Eigen::Matrix4d offset_T = Eigen::MatrixXd::Identity(4, 4);
                    offset_T.col(3) << pick_offset(3), pick_offset(4), pick_offset(5) - abs(pick_offset(5)), 1;
                    offset_T = cart_T * offset_T;
                    cur_goal = lego_manipulation::math::IK(cur_goal, offset_T.block(0, 3, 3, 1), offset_T.block(0, 0, 3, 3),
                                                            lego_ptr->robot_DH_tool(), lego_ptr->robot_base(), 0, 10e6, 10e-3*5);
                    // cur_goal =  lego_manipulation::math::IK_closed_form(cur_goal, offset_T, lego_ptr->robot_DH_tool(), 
                    //                                                     lego_ptr->robot_base_inv(), lego_ptr->robot_tool_inv(), 0, IK_status);
                    if(!assemble && lego_ptr->brick_instock(brick_name))
                    {
                        cur_goal = home_q;
                        mode = 12;
                    }
                }
                else if (mode == 2)
                {
                    Eigen::Matrix4d up_T = Eigen::MatrixXd::Identity(4, 4);
                    up_T.col(3) << 0, 0, pick_offset(5), 1;
                    up_T = cart_T * up_T;
                    cur_goal = lego_manipulation::math::IK(cur_goal, up_T.block(0, 3, 3, 1), up_T.block(0, 0, 3, 3),
                                                            lego_ptr->robot_DH_tool(), lego_ptr->robot_base(), 0, 10e6, 10e-3*5);
                    // cur_goal = lego_manipulation::math::IK_closed_form(cur_goal, up_T, lego_ptr->robot_DH_tool(), 
                    //                                                    lego_ptr->robot_base_inv(), lego_ptr->robot_tool_inv(), 0, IK_status);
                }
                else if (mode == 3)
                {
                    cur_goal = lego_manipulation::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                                            lego_ptr->robot_DH_tool(), lego_ptr->robot_base(), 0, 10e6, 10e-3*5);
                    // cur_goal =  lego_manipulation::math::IK_closed_form(cur_goal, cart_T, lego_ptr->robot_DH_tool(), 
                    //                                                     lego_ptr->robot_base_inv(), lego_ptr->robot_tool_inv(), 0, IK_status);
                }
                else if (mode == 4)
                {
                    double twist_rad = incremental_deg / 180.0 * PI;
                    twist_R << cos(twist_rad), 0, sin(twist_rad), 
                               0, 1, 0, 
                               -sin(twist_rad), 0, cos(twist_rad);
                    twist_T = Eigen::MatrixXd::Identity(4, 4);
                    twist_T.block(0, 0, 3, 3) << twist_R;
                    cart_T = lego_manipulation::math::FK(cur_goal, lego_ptr->robot_DH_tool_disassemble(), lego_ptr->robot_base(), false);
                    cart_T = cart_T * twist_T;
                    cur_goal = lego_manipulation::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                                            lego_ptr->robot_DH_tool_disassemble(), lego_ptr->robot_base(), 0, 10e6, 10e-3*5);
                    // cur_goal =  lego_manipulation::math::IK_closed_form(cur_goal, cart_T, lego_ptr->robot_DH_tool_disassemble(), 
                    //                                                     lego_ptr->robot_base_inv(), lego_ptr->robot_tool_disassemble_inv(),0,IK_status);
                }
                else if(mode == 5 || mode == 11)
                {
                    cart_T = lego_manipulation::math::FK(cur_goal, lego_ptr->robot_DH_tool_assemble(), lego_ptr->robot_base(), false);
                    cart_T(2, 3) = cart_T(2, 3) + 0.015;
                    cur_goal = lego_manipulation::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                                            lego_ptr->robot_DH_tool_assemble(), lego_ptr->robot_base(), 0, 10e6, 10e-3*5);
                    // cur_goal =  lego_manipulation::math::IK_closed_form(cur_goal, cart_T, lego_ptr->robot_DH_tool_assemble(), 
                    //                                                     lego_ptr->robot_base_inv(), lego_ptr->robot_tool_assemble_inv(),0,IK_status);
                }
                else if(mode == 7)
                {
                    auto cur_graph_node = task_json[std::to_string(task_idx)];
                    grab_brick = 0;
                    lego_ptr->calc_brick_grab_pose(brick_name, assemble, 0,
                                                          cur_graph_node["x"].asInt(),
                                                          cur_graph_node["y"].asInt(),
                                                          cur_graph_node["z"].asInt(),
                                                          cur_graph_node["ori"].asInt(),
                                                          cur_graph_node["press_side"].asInt(), cart_T);
                    Eigen::Matrix4d offset_T = Eigen::MatrixXd::Identity(4, 4);
                    offset_T.col(3) << pick_offset(0), pick_offset(1), pick_offset(2) - abs(pick_offset(2)), 1;
                    offset_T = cart_T * offset_T;
                    cur_goal = lego_manipulation::math::IK(cur_goal, offset_T.block(0, 3, 3, 1), offset_T.block(0, 0, 3, 3),
                                                            lego_ptr->robot_DH_tool(), lego_ptr->robot_base(), 0, 10e6, 10e-3*5);
                    // cur_goal =  lego_manipulation::math::IK_closed_form(cur_goal, offset_T, lego_ptr->robot_DH_tool(), 
                    //                                                     lego_ptr->robot_base_inv(), lego_ptr->robot_tool_inv(),0,IK_status);
                }
                else if (mode == 8)
                {   
                    Eigen::Matrix4d up_T = Eigen::MatrixXd::Identity(4, 4);
                    up_T.col(3) << 0, 0, pick_offset(2), 1;
                    up_T = cart_T * up_T;
                    cur_goal = lego_manipulation::math::IK(cur_goal, up_T.block(0, 3, 3, 1), up_T.block(0, 0, 3, 3),
                                                            lego_ptr->robot_DH_tool(), lego_ptr->robot_base(), 0, 10e6, 10e-3*5);
                    // cur_goal =  lego_manipulation::math::IK_closed_form(cur_goal, up_T, lego_ptr->robot_DH_tool(), 
                    //                                                     lego_ptr->robot_base_inv(), lego_ptr->robot_tool_inv(),0,IK_status);
                }
                else if(mode == 9)
                {
                    cur_goal = lego_manipulation::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                                            lego_ptr->robot_DH_tool(), lego_ptr->robot_base(), 0, 10e6, 10e-3*5);
                    // cur_goal =  lego_manipulation::math::IK_closed_form(cur_goal, cart_T, lego_ptr->robot_DH_tool(), 
                    //                                                     lego_ptr->robot_base_inv(), lego_ptr->robot_tool_inv(),0,IK_status);
                }
                else if(mode == 10)
                {
                    double twist_rad = incremental_deg / 180.0 * PI;
                    twist_R << cos(-twist_rad), 0, sin(-twist_rad), 
                               0, 1, 0, 
                               -sin(-twist_rad), 0, cos(-twist_rad);
                    twist_T = Eigen::MatrixXd::Identity(4, 4);
                    twist_T.block(0, 0, 3, 3) << twist_R;
                    cart_T = lego_manipulation::math::FK(cur_goal, lego_ptr->robot_DH_tool_assemble(), lego_ptr->robot_base(), false);
                    cart_T = cart_T * twist_T;
                    cur_goal = lego_manipulation::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                                            lego_ptr->robot_DH_tool_assemble(), lego_ptr->robot_base(), 0, 10e6, 10e-3*5);
                    // cur_goal =  lego_manipulation::math::IK_closed_form(cur_goal, cart_T, lego_ptr->robot_DH_tool_assemble(), 
                    //                                                     lego_ptr->robot_base_inv(), lego_ptr->robot_tool_assemble_inv(),0,IK_status);
                }
            }

            // Send to Stmotion controller
            goal_msg.data.clear();
            fts_one_step_msg.data.clear();
            fts_avg_msg.data.clear();
            for(int j=0; j<lego_ptr->robot_dof(); j++)
            {
                goal_msg.data.push_back(cur_goal(j));
                fts_one_step_msg.data.push_back(fts_val_one_step(j));
                fts_avg_msg.data.push_back(fts_val(j));
            }
            goal_pub.publish(goal_msg);
            fts_avg_pub.publish(fts_avg_msg);
            fts_one_step_pub.publish(fts_one_step_msg);


            Eigen::MatrixXd cart_T_goal = lego_manipulation::math::FK(cur_goal, lego_ptr->robot_DH(), lego_ptr->robot_base(), false);
            Eigen::Matrix3d goal_rot = cart_T_goal.block(0, 0, 3, 3);
            Eigen::Quaterniond quat(goal_rot);

            // Send to yaskawa service
            if (use_yk)
            {
                geometry_msgs::Pose goal_pose;
                if (move_on_to_next)
                {
                    goal_pose.position.x = cart_T_goal(0, 3);
                    goal_pose.position.y = cart_T_goal(1, 3);
                    goal_pose.position.z = cart_T_goal(2, 3);
                    goal_pose.orientation.x = quat.x();
                    goal_pose.orientation.y = quat.y();
                    goal_pose.orientation.z = quat.z();
                    goal_pose.orientation.w = quat.w();
                    move_on_to_next = false;
                    ROS_INFO_STREAM("Sending pose: " << goal_pose);
                }

                srv.request.base_frame = "base_link";
                srv.request.pose = goal_pose;
                if (client.call(srv))
                {
                    move_on_to_next = true;
                    ROS_INFO_STREAM("Pose Set to: " << srv.response.pose);
                }
            }
        }
        ROS_INFO_STREAM("Task Execution Done!");
        ros::shutdown();
        return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}



