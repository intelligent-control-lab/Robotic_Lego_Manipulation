#include "Lego.hpp"
// HEADER FILES FOR SERVICE INTERFACE WITH YK ROBOT
#include "geometry_msgs/Pose.h"
#include "lego_manipulation/SetPose.h"

// HEADER FILES FOR API INTERFACE WITH YK ROBOT
// #include "yk_api/yk_interface.h"  // Uncomment this if using yaskawa robot

using namespace std::chrono;

lego_manipulation::math::VectorJd robot_q = Eigen::MatrixXd::Zero(6, 1);
lego_manipulation::math::VectorJd robot_qd = Eigen::MatrixXd::Zero(6, 1);
lego_manipulation::math::VectorJd robot_qdd = Eigen::MatrixXd::Zero(6, 1);


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
        bool use_yk = config["Use_yk"].asBool();
        int twist_deg = config["Twist_Deg"].asInt();


        std::ifstream task_file(task_fname, std::ifstream::binary);
        Json::Value task_json;
        task_file >> task_json;

        lego_manipulation::lego::Lego::Ptr lego_ptr = std::make_shared<lego_manipulation::lego::Lego>();
        lego_ptr->setup(env_setup_fname, lego_lib_fname, assemble, task_json, DH_fname, DH_tool_fname, DH_tool_disassemble_fname, DH_tool_assemble_fname, 
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
        ros::Subscriber robot_state_sub = nh.subscribe(robot_state_topic, lego_ptr->robot_dof() * 3, robotStateCallback);
        std_msgs::Float32MultiArray goal_msg;
        std_msgs::Float64 controller_time_msg;
        std_msgs::Int64 controller_tracking_space_msg;
        
        int num_tasks = task_json.size();
        Eigen::MatrixXd home_q(lego_ptr->robot_dof(), 1);
        home_q.col(0) << 0, 0, 0, 0, -90, 0;
        Eigen::Matrix4d home_T = lego_manipulation::math::FK(home_q, lego_ptr->robot_DH(), lego_ptr->robot_base(), false);
        home_T.col(3) << 0.6, 0, 0.4, 1; // Home X, Y, Z in base frame of the Flange
        home_q = lego_manipulation::math::IK(home_q, home_T.block(0, 3, 3, 1), home_T.block(0, 0, 3, 3),
                                             lego_ptr->robot_DH(), lego_ptr->robot_base(), 0, 10e6, 10e-3); // Home

        int mode = 0; // 0: home, 1: pick up, 2, pick down 3: pick twist 4: pick twist up 5: home
                      // 6: drop up 7: drop down 8: drop twist 9: drop twist up 10: done
        int task_idx;
        if(assemble)
        {
            task_idx = 1;
        }
        else
        {
            task_idx = num_tasks;
        }
        
        int pre_mode = -1;
        lego_manipulation::math::VectorJd cur_goal = home_q;
        std::string brick_name;
        bool move_on_to_next = use_yk;
        
        while(ros::ok)
        {
            if(mode >= 3 && mode <= 7)
            {
                if(pre_mode != mode)
                {
                    lego_ptr->update_bricks(robot_q, lego_ptr->robot_DH_tool(), lego_ptr->robot_base(), false, brick_name);
                }
                else
                {
                    lego_ptr->update_bricks(robot_q, lego_ptr->robot_DH_tool(), lego_ptr->robot_base(), false, brick_name);
                }
            }
            pre_mode = mode;
            if((use_yk && move_on_to_next) || (!use_yk && lego_ptr->robot_reached_goal(robot_q, cur_goal) && lego_ptr->robot_is_static(robot_qd, robot_qdd)))
            {
                if(mode == 7)
                {
                    lego_ptr->update_brick_connection();
                }
                if(mode == 10){
                    if(task_idx >= num_tasks && !infinite_tasks && assemble)
                    {
                        break;
                    }
                    else if(task_idx <= 1 && !infinite_tasks && !assemble)
                    {
                        break;
                    }
                    else if(task_idx >= num_tasks && assemble)
                    {
                        task_idx ++;
                        assemble = !assemble;
                    }
                    else if(task_idx <= 1 && !assemble)
                    {
                        task_idx --;
                        assemble = !assemble;
                    }
                    mode = 0;
                    if(assemble)
                    {
                        task_idx ++;
                    }
                    else
                    {
                        task_idx --;
                    }
                }
                else if(mode == 3 || mode == 8)
                {
                    twist_idx ++;
                    if(twist_idx == twist_num)
                    {
                        mode ++;
                        twist_idx = 0;
                    }
                }
                else{
                    mode ++;
                }

                // Modify stmotion controller tracking space
                controller_tracking_space_msg.data = 0;
                controller_tracking_space_pub.publish(controller_tracking_space_msg);
                // Modify stmotion controller runtime
                controller_time_msg.data = 0.5;
                controller_time_pub.publish(controller_time_msg);
                
                ROS_INFO_STREAM("Mode: " << mode << " Task: " << task_idx);

                if(mode == 0 || mode == 5 || mode == 10)
                {
                    cur_goal = home_q;
                }
                else if(mode == 1)
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
                    Eigen::Matrix4d up_T = cart_T;
                    up_T(2, 3) = up_T(2, 3) + 0.015;
                    cur_goal =  lego_manipulation::math::IK_closed_form(cur_goal, up_T, lego_ptr->robot_DH_tool(), 
                                                                        lego_ptr->robot_base_inv(), lego_ptr->robot_tool_inv(), 0, IK_status);
                    if(!assemble && lego_ptr->brick_instock(brick_name))
                    {
                        cur_goal = home_q;
                        mode = 10;
                    }
                }
                else if(mode == 2)
                {
                    cur_goal =  lego_manipulation::math::IK_closed_form(cur_goal, cart_T, lego_ptr->robot_DH_tool(), 
                                                                        lego_ptr->robot_base_inv(), lego_ptr->robot_tool_inv(), 0, IK_status);
                }
                else if(mode == 3)
                {
                    double twist_rad = incremental_deg / 180.0 * PI;
                    twist_R << cos(twist_rad), 0, sin(twist_rad), 
                               0, 1, 0, 
                               -sin(twist_rad), 0, cos(twist_rad);
                    twist_T = Eigen::MatrixXd::Identity(4, 4);
                    twist_T.block(0, 0, 3, 3) << twist_R;
                    cart_T = lego_manipulation::math::FK(cur_goal, lego_ptr->robot_DH_tool_disassemble(), lego_ptr->robot_base(), false);
                    cart_T = cart_T * twist_T;
                    cur_goal =  lego_manipulation::math::IK_closed_form(cur_goal, cart_T, lego_ptr->robot_DH_tool_disassemble(), 
                                                                        lego_ptr->robot_base_inv(), lego_ptr->robot_tool_disassemble_inv(),0,IK_status);
                }
                else if(mode == 4 || mode == 9)
                {
                    cart_T = lego_manipulation::math::FK(cur_goal, lego_ptr->robot_DH_tool_assemble(), lego_ptr->robot_base(), false);
                    cart_T(2, 3) = cart_T(2, 3) + 0.015;
                    cur_goal =  lego_manipulation::math::IK_closed_form(cur_goal, cart_T, lego_ptr->robot_DH_tool_assemble(), 
                                                                        lego_ptr->robot_base_inv(), lego_ptr->robot_tool_assemble_inv(),0,IK_status);
                }
                else if(mode == 6)
                {
                    auto cur_graph_node = task_json[std::to_string(task_idx)];
                    grab_brick = 0;
                    lego_ptr->calc_brick_grab_pose(brick_name, assemble, 0,
                                                   cur_graph_node["x"].asInt(), 
                                                   cur_graph_node["y"].asInt(), 
                                                   cur_graph_node["z"].asInt(),
                                                   cur_graph_node["ori"].asInt(),
                                                   cur_graph_node["press_side"].asInt(), cart_T);
                    Eigen::Matrix4d up_T = cart_T;
                    up_T(2, 3) = up_T(2, 3) + 0.015;
                    cur_goal =  lego_manipulation::math::IK_closed_form(cur_goal, up_T, lego_ptr->robot_DH_tool(), 
                                                                        lego_ptr->robot_base_inv(), lego_ptr->robot_tool_inv(),0,IK_status);
                }
                else if(mode == 7)
                {
                    cur_goal =  lego_manipulation::math::IK_closed_form(cur_goal, cart_T, lego_ptr->robot_DH_tool(), 
                                                                        lego_ptr->robot_base_inv(), lego_ptr->robot_tool_inv(),0,IK_status);
                }
                else if(mode == 8)
                {
                    double twist_rad = incremental_deg / 180.0 * PI;
                    twist_R << cos(-twist_rad), 0, sin(-twist_rad), 
                               0, 1, 0, 
                               -sin(-twist_rad), 0, cos(-twist_rad);
                    twist_T = Eigen::MatrixXd::Identity(4, 4);
                    twist_T.block(0, 0, 3, 3) << twist_R;
                    cart_T = lego_manipulation::math::FK(cur_goal, lego_ptr->robot_DH_tool_assemble(), lego_ptr->robot_base(), false);
                    cart_T = cart_T * twist_T;
                    cur_goal =  lego_manipulation::math::IK_closed_form(cur_goal, cart_T, lego_ptr->robot_DH_tool_assemble(), 
                                                                        lego_ptr->robot_base_inv(), lego_ptr->robot_tool_assemble_inv(),0,IK_status);
                }
            }

            // Send to Stmotion controller
            goal_msg.data.clear();
            for(int j=0; j<lego_ptr->robot_dof(); j++)
            {
                goal_msg.data.push_back(cur_goal(j));
            }
            goal_pub.publish(goal_msg);


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



