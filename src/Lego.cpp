#include "Lego.hpp"

namespace lego_manipulation
{
namespace lego
{
Lego::Lego()
{
}

void Lego::setup_dual_arm(const std::string& env_setup_fname, const std::string& lego_lib_fname, const Json::Value& task_json, const std::string& world_base_fname,
                          const std::string& r1_DH_fname, const std::string& r1_DH_tool_fname, const std::string& r1_DH_tool_disassemble_fname, 
                          const std::string& r1_DH_tool_assemble_fname, const std::string& r1_DH_tool_alt_fname, const std::string& r1_DH_tool_alt_assemble_fname, const std::string& r1_base_fname, 
                          const std::string& r2_DH_fname, const std::string& r2_DH_tool_fname, const std::string& r2_DH_tool_disassemble_fname, 
                          const std::string& r2_DH_tool_assemble_fname, const std::string& r2_DH_tool_alt_fname, const std::string& r2_DH_tool_alt_assemble_fname, const std::string& r2_base_fname, const ros::ServiceClient& cli)
{
    client_ = cli;

    gazebo_msgs::ModelState brick_pose;
    std::ifstream config_file(env_setup_fname, std::ifstream::binary);
    Json::Value config;
    double x, y, z, roll, pitch, yaw;
    Eigen::Quaterniond quat(Eigen::Matrix3d::Identity(3, 3));
    Eigen::Matrix4d brick_pose_mtx;
    Eigen::Matrix3d z_90;
    z_90 << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix3d rot_mtx = Eigen::Matrix3d::Identity(3, 3);
    std::string brick_name;
    std::ifstream lego_lib_file(lego_lib_fname, std::ifstream::binary);
    Json::Value lego_library;

    thetamax_.resize(r1_robot_dof_, 2);
    thetamax_rad_.resize(r1_robot_dof_, 2);
    thetamax_ << -170, 170,
                 -110, 130,
                 -65, 200,
                 -200, 200,
                 -120, 120,
                 -450, 450;
    for(int i=0; i<r1_robot_dof_; i++)
    {
        thetamax_rad_.row(i) << thetamax_(i, 0) / 180 * PI, thetamax_(i, 1) / 180 * PI;
    }
    set_world_base(world_base_fname);
    set_robot_base(r1_base_fname, r2_base_fname);
    set_DH(r1_DH_fname, r2_DH_fname);
    set_DH_tool(r1_DH_tool_fname, r2_DH_tool_fname);
    set_DH_tool_assemble(r1_DH_tool_assemble_fname, r2_DH_tool_assemble_fname);
    set_DH_tool_disassemble(r1_DH_tool_disassemble_fname, r2_DH_tool_disassemble_fname);
    set_DH_tool_alt(r1_DH_tool_alt_fname, r2_DH_tool_alt_fname);
    set_DH_tool_alt_assemble(r1_DH_tool_alt_assemble_fname, r2_DH_tool_alt_assemble_fname);
    print_manipulation_property();
    config_file >> config;
    lego_lib_file >> lego_library;

    for(auto brick = config.begin(); brick != config.end(); brick++)
    {
        brick_pose.model_name = brick.name();
        if(brick.name().compare("storage_plate") == 0)
        {
            x = (*brick)["x"].asDouble();
            y = (*brick)["y"].asDouble();
            z = (*brick)["z"].asDouble();
            roll = (*brick)["roll"].asDouble();
            pitch = (*brick)["pitch"].asDouble();
            yaw = (*brick)["yaw"].asDouble();
            Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond quat_storage = yawAngle * pitchAngle * rollAngle;
            storage_plate_.pose = Eigen::Matrix4d::Identity(4, 4);
            storage_plate_.pose.block(0, 0, 3, 3) = quat_storage.matrix();
            storage_plate_.pose.col(3) << x, y, z, 1;
            storage_plate_.width = (*brick)["width"].asInt();
            storage_plate_.height = (*brick)["height"].asInt();
            storage_plate_.pose = world_base_frame_ * storage_plate_.pose;
            x = storage_plate_.pose(0, 3);
            y = storage_plate_.pose(1, 3);
            z = storage_plate_.pose(2, 3);
            Eigen::Matrix3d rot_mtx = storage_plate_.pose.block(0, 0, 3, 3);
            quat = rot_mtx;
        }
        else if(brick.name().compare("assemble_plate") == 0)
        {
            x = (*brick)["x"].asDouble();
            y = (*brick)["y"].asDouble();
            z = (*brick)["z"].asDouble();
            roll = (*brick)["roll"].asDouble();
            pitch = (*brick)["pitch"].asDouble();
            yaw = (*brick)["yaw"].asDouble();
            Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond quat_assemble = yawAngle * pitchAngle * rollAngle;
            assemble_plate_.pose = Eigen::Matrix4d::Identity(4, 4);
            assemble_plate_.pose.block(0, 0, 3, 3) = quat_assemble.matrix();
            assemble_plate_.pose.col(3) << x, y, z, 1;
            assemble_plate_.width = (*brick)["width"].asInt();
            assemble_plate_.height = (*brick)["height"].asInt();
            assemble_plate_.pose = world_base_frame_ * assemble_plate_.pose;
            x = assemble_plate_.pose(0, 3);
            y = assemble_plate_.pose(1, 3);
            z = assemble_plate_.pose(2, 3);
            Eigen::Matrix3d rot_mtx = assemble_plate_.pose.block(0, 0, 3, 3);
            quat = rot_mtx;
        }
        else{
            continue;
        }
        brick_pose.pose.position.x = x;
        brick_pose.pose.position.y = y;
        brick_pose.pose.position.z = z;
        brick_pose.pose.orientation.x = quat.x();
        brick_pose.pose.orientation.y = quat.y();
        brick_pose.pose.orientation.z = quat.z();
        brick_pose.pose.orientation.w = quat.w();
        setmodelstate_.request.model_state = brick_pose;
        client_.call(setmodelstate_);
    }

    for(auto brick = config.begin(); brick != config.end(); brick++)
    {
        brick_pose.model_name = brick.name();
        if(brick.name()[0] == 'b')
        {
            lego_brick l_brick;
            l_brick.brick_name = brick.name();
            brick_dimension_from_name(brick.name(), l_brick.height, l_brick.width, lego_library);
            calc_brick_loc(l_brick, storage_plate_, (*brick)["ori"].asInt(),
                           (*brick)["x"].asInt(), (*brick)["y"].asInt(), (*brick)["z"].asInt(),
                           brick_pose_mtx);
            x = brick_pose_mtx(0, 3);
            y = brick_pose_mtx(1, 3);
            z = brick_pose_mtx(2, 3);

            l_brick.x = x;
            l_brick.y = y;
            l_brick.z = z;
            l_brick.cur_x = x;
            l_brick.cur_y = y;
            l_brick.cur_z = z;
            l_brick.in_stock = true;
            rot_mtx = brick_pose_mtx.block(0, 0, 3, 3);
            quat = rot_mtx;
            l_brick.quat_x = quat.x();
            l_brick.quat_y = quat.y();
            l_brick.quat_z = quat.z();
            l_brick.quat_w = quat.w();
            l_brick.cur_quat = quat;
            brick_map_[brick.name()] = l_brick;
        }
        else
        {
            ROS_INFO_STREAM("Unknown brick type: " << brick.name() << " !");
            continue;
        }
        brick_pose.pose.position.x = x;
        brick_pose.pose.position.y = y;
        brick_pose.pose.position.z = z;
        brick_pose.pose.orientation.x = quat.x();
        brick_pose.pose.orientation.y = quat.y();
        brick_pose.pose.orientation.z = quat.z();
        brick_pose.pose.orientation.w = quat.w();
        setmodelstate_.request.model_state = brick_pose;
        client_.call(setmodelstate_);
    }
    update_brick_connection();
    usleep(1000 * 1000); 
}


void Lego::setup(const std::string& env_setup_fname, const std::string& lego_lib_fname, const bool& assemble, const Json::Value& task_json, const std::string& world_base_fname,
                   const std::string& r1_DH_fname, const std::string& r1_DH_tool_fname, const std::string& r1_DH_tool_disassemble_fname, 
                   const std::string& r1_DH_tool_assemble_fname, const std::string& r1_base_fname, 
                   const std::string& r2_DH_fname, const std::string& r2_DH_tool_fname, const std::string& r2_DH_tool_disassemble_fname, 
                   const std::string& r2_DH_tool_assemble_fname, const std::string& r2_base_fname,const bool& use_config_file, const ros::ServiceClient& cli)
{
    client_ = cli;

    gazebo_msgs::ModelState brick_pose;
    std::ifstream config_file(env_setup_fname, std::ifstream::binary);
    Json::Value config;
    double x, y, z;
    Eigen::Quaterniond quat(Eigen::Matrix3d::Identity(3, 3));
    Eigen::Matrix4d brick_pose_mtx;
    Eigen::Matrix3d z_90;
    z_90 << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix3d rot_mtx = Eigen::Matrix3d::Identity(3, 3);
    std::string brick_name;
    std::ifstream lego_lib_file(lego_lib_fname, std::ifstream::binary);
    Json::Value lego_library;

    set_world_base(world_base_fname);
    set_robot_base(r1_base_fname, r2_base_fname);
    set_DH(r1_DH_fname, r2_DH_fname);
    set_DH_tool(r1_DH_tool_fname, r2_DH_tool_fname);
    set_DH_tool_assemble(r1_DH_tool_assemble_fname, r2_DH_tool_assemble_fname);
    set_DH_tool_disassemble(r1_DH_tool_disassemble_fname, r2_DH_tool_disassemble_fname);
    print_manipulation_property();
    config_file >> config;
    lego_lib_file >> lego_library;

    for(auto brick = config.begin(); brick != config.end(); brick++)
    {
        brick_pose.model_name = brick.name();
        if(brick.name().compare("storage_plate") == 0)
        {
            if(use_config_file)
            {
                double x = (*brick)["x"].asDouble();
                double y = (*brick)["y"].asDouble();
                double z = (*brick)["z"].asDouble();
                double roll = (*brick)["roll"].asDouble();
                double pitch = (*brick)["pitch"].asDouble();
                double yaw = (*brick)["yaw"].asDouble();
                Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
                Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
                Eigen::Quaterniond quat = yawAngle * pitchAngle * rollAngle;
                storage_plate_.pose = Eigen::Matrix4d::Identity(4, 4);
                storage_plate_.pose.block(0, 0, 3, 3) = quat.matrix();
                storage_plate_.pose.col(3) << x, y, z, 1;
                storage_plate_.width = (*brick)["width"].asInt();
                storage_plate_.height = (*brick)["height"].asInt();
            }
            storage_plate_.pose = world_base_frame_ * storage_plate_.pose;
            x = storage_plate_.pose(0, 3);
            y = storage_plate_.pose(1, 3);
            z = storage_plate_.pose(2, 3);
            Eigen::Matrix3d rot_mtx = storage_plate_.pose.block(0, 0, 3, 3);
            quat = rot_mtx;
        }
        else if(brick.name().compare("assemble_plate") == 0)
        {
            if(use_config_file)
            {
                double x = (*brick)["x"].asDouble();
                double y = (*brick)["y"].asDouble();
                double z = (*brick)["z"].asDouble();
                double roll = (*brick)["roll"].asDouble();
                double pitch = (*brick)["pitch"].asDouble();
                double yaw = (*brick)["yaw"].asDouble();
                Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
                Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
                Eigen::Quaterniond quat = yawAngle * pitchAngle * rollAngle;
                assemble_plate_.pose = Eigen::Matrix4d::Identity(4, 4);
                assemble_plate_.pose.block(0, 0, 3, 3) = quat.matrix();
                assemble_plate_.pose.col(3) << x, y, z, 1;
                assemble_plate_.width = (*brick)["width"].asInt();
                assemble_plate_.height = (*brick)["height"].asInt();
            }
            assemble_plate_.pose = world_base_frame_ * assemble_plate_.pose;
            x = assemble_plate_.pose(0, 3);
            y = assemble_plate_.pose(1, 3);
            z = assemble_plate_.pose(2, 3);
            Eigen::Matrix3d rot_mtx = assemble_plate_.pose.block(0, 0, 3, 3);
            quat = rot_mtx;
        }
        else{
            continue;
        }
        brick_pose.pose.position.x = x;
        brick_pose.pose.position.y = y;
        brick_pose.pose.position.z = z;
        brick_pose.pose.orientation.x = quat.x();
        brick_pose.pose.orientation.y = quat.y();
        brick_pose.pose.orientation.z = quat.z();
        brick_pose.pose.orientation.w = quat.w();
        setmodelstate_.request.model_state = brick_pose;
        client_.call(setmodelstate_);
    }

    for(auto brick = config.begin(); brick != config.end(); brick++)
    {
        brick_pose.model_name = brick.name();
        if(brick.name()[0] == 'b')
        {
            lego_brick l_brick;
            l_brick.brick_name = brick.name();
            brick_dimension_from_name(brick.name(), l_brick.height, l_brick.width, lego_library);
            calc_brick_loc(l_brick, storage_plate_, (*brick)["ori"].asInt(),
                           (*brick)["x"].asInt(), (*brick)["y"].asInt(), (*brick)["z"].asInt(),
                           brick_pose_mtx);
            x = brick_pose_mtx(0, 3);
            y = brick_pose_mtx(1, 3);
            z = brick_pose_mtx(2, 3);

            l_brick.x = x;
            l_brick.y = y;
            l_brick.z = z;
            l_brick.cur_x = x;
            l_brick.cur_y = y;
            l_brick.cur_z = z;
            l_brick.in_stock = true;
            rot_mtx = brick_pose_mtx.block(0, 0, 3, 3);
            quat = rot_mtx;
            l_brick.quat_x = quat.x();
            l_brick.quat_y = quat.y();
            l_brick.quat_z = quat.z();
            l_brick.quat_w = quat.w();
            l_brick.cur_quat = quat;
            brick_map_[brick.name()] = l_brick;
        }
        else
        {
            ROS_INFO_STREAM("Unknown brick type: " << brick.name() << " !");
            continue;
        }
        brick_pose.pose.position.x = x;
        brick_pose.pose.position.y = y;
        brick_pose.pose.position.z = z;
        brick_pose.pose.orientation.x = quat.x();
        brick_pose.pose.orientation.y = quat.y();
        brick_pose.pose.orientation.z = quat.z();
        brick_pose.pose.orientation.w = quat.w();
        setmodelstate_.request.model_state = brick_pose;
        client_.call(setmodelstate_);
    }
    if(!assemble) // Starting from disassemble, build the structure first
    {
        for(int i=1; i<=task_json.size(); i++)
        {
            auto cur_graph_node = task_json[std::to_string(i)];
            brick_name = get_brick_name_by_id(cur_graph_node["brick_id"].asInt(), cur_graph_node["brick_seq"].asInt());
            calc_brick_loc(brick_map_[brick_name], assemble_plate_, cur_graph_node["ori"].asInt(),
                           cur_graph_node["x"].asInt(), cur_graph_node["y"].asInt(), cur_graph_node["z"].asInt(),
                           brick_pose_mtx);
            x = brick_pose_mtx(0, 3);
            y = brick_pose_mtx(1, 3);
            z = brick_pose_mtx(2, 3);
            rot_mtx = brick_pose_mtx.block(0, 0, 3, 3);

            brick_map_[brick_name].in_stock = assemble;
            quat = rot_mtx;
            brick_map_[brick_name].cur_x = x;
            brick_map_[brick_name].cur_y = y;
            brick_map_[brick_name].cur_z = z;
            brick_map_[brick_name].cur_quat = quat;

            brick_pose.model_name = brick_name;
            brick_pose.pose.position.x = x;
            brick_pose.pose.position.y = y;
            brick_pose.pose.position.z = z;
            brick_pose.pose.orientation.x = quat.x();
            brick_pose.pose.orientation.y = quat.y();
            brick_pose.pose.orientation.z = quat.z();
            brick_pose.pose.orientation.w = quat.w();
            setmodelstate_.request.model_state = brick_pose;
            client_.call(setmodelstate_);
        }
    }
    update_brick_connection();
    usleep(1000 * 1000); 
}

void Lego::brick_dimension_from_name(const std::string& b_name, int& height, int& width, const Json::Value& lego_lib)
{
    auto dash_id = b_name.find("_");
    std::string id = b_name.substr(1, dash_id - 1);
    height = lego_lib[id]["height"].asInt();
    width = lego_lib[id]["width"].asInt();
}

void Lego::set_world_base(const std::string& world_base_fname)
{
    ROS_INFO_STREAM("Load World Base from: " << world_base_fname);
    world_base_frame_ = io::LoadMatFromFile(world_base_fname);
    world_T_base_inv_ = math::PInv(world_base_frame_);
}

void Lego::set_robot_base(const std::string& r1_fname, const std::string& r2_fname)
{
    ROS_INFO_STREAM("Load Robot 1 Base from: " << r1_fname);
    r1_base_frame_ = world_base_frame_ * io::LoadMatFromFile(r1_fname);
    r1_T_base_inv_ = r1_base_frame_;
    r1_T_base_inv_ = math::PInv(r1_T_base_inv_);

    ROS_INFO_STREAM("Load Robot 2 Base from: " << r2_fname);
    r2_base_frame_ = world_base_frame_ * io::LoadMatFromFile(r2_fname);
    r2_T_base_inv_ = r2_base_frame_;
    r2_T_base_inv_ = math::PInv(r2_T_base_inv_);
}

void Lego::print_manipulation_property()
{
    ROS_INFO_STREAM("\nRobot 1 Base: \n" << r1_base_frame_);
    ROS_INFO_STREAM("\nRobot 1 DH: \n" << r1_DH_);
    ROS_INFO_STREAM("\nRobot 1 Tool DH: \n" << r1_DH_tool_);
    ROS_INFO_STREAM("\nRobot 1 Tool Disassemble DH: \n" << r1_DH_tool_disassemble_);
    ROS_INFO_STREAM("\nRobot 1 Tool Assemble DH: \n" << r1_DH_tool_assemble_);

    ROS_INFO_STREAM("\nRobot 2 Base: \n" << r2_base_frame_);
    ROS_INFO_STREAM("\nRobot 2 DH: \n" << r2_DH_);
    ROS_INFO_STREAM("\nRobot 2 Tool DH: \n" << r2_DH_tool_);
    ROS_INFO_STREAM("\nRobot 2 Tool Disassemble DH: \n" << r2_DH_tool_disassemble_);
    ROS_INFO_STREAM("\nRobot 2 Tool Assemble DH: \n" << r2_DH_tool_assemble_);
    std::cout << "\n" << std::endl;
}

void Lego::set_DH(const std::string& r1_fname, const std::string& r2_fname)
{
    ROS_INFO_STREAM("Load Robot 1 DH from: " << r1_fname);
    r1_DH_ = io::LoadMatFromFile(r1_fname);
    r1_ee_inv_ = Eigen::Matrix4d::Identity(4, 4); 
    r1_ee_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  r1_DH_(5, 2) * cos(0),
               sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  r1_DH_(5, 2) * sin(0),
               0,       sin(0),         cos(0),        -r1_DH_(5, 1),
               0,       0,              0,              1;
    r1_ee_inv_ = math::PInv(r1_ee_inv_);

    ROS_INFO_STREAM("Load Robot 2 DH from: " << r2_fname);
    r2_DH_ = io::LoadMatFromFile(r2_fname);
    r2_ee_inv_ = Eigen::Matrix4d::Identity(4, 4); 
    r2_ee_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  r2_DH_(5, 2) * cos(0),
               sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  r2_DH_(5, 2) * sin(0),
               0,       sin(0),         cos(0),        -r2_DH_(5, 1),
               0,       0,              0,              1;
    r2_ee_inv_ = math::PInv(r2_ee_inv_);
}


void Lego::set_DH_tool(const std::string& r1_fname, const std::string& r2_fname)
{
    ROS_INFO_STREAM("Load r1 DH tool from: " << r1_fname);
    r1_DH_tool_ = io::LoadMatFromFile(r1_fname);
    r1_tool_inv_ = Eigen::Matrix4d::Identity(4, 4); 
    r1_tool_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  r1_DH_tool_(5, 2) * cos(0),
                 sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  r1_DH_tool_(5, 2) * sin(0),
                 0,       sin(0),         cos(0),        -r1_DH_tool_(5, 1),
                 0,       0,              0,              1;
    r1_tool_inv_ = math::PInv(r1_tool_inv_);

    ROS_INFO_STREAM("Load r2 DH tool from: " << r2_fname);
    r2_DH_tool_ = io::LoadMatFromFile(r2_fname);
    r2_tool_inv_ = Eigen::Matrix4d::Identity(4, 4); 
    r2_tool_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  r2_DH_tool_(5, 2) * cos(0),
                 sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  r2_DH_tool_(5, 2) * sin(0),
                 0,       sin(0),         cos(0),        -r2_DH_tool_(5, 1),
                 0,       0,              0,              1;
    r2_tool_inv_ = math::PInv(r2_tool_inv_);
}

void Lego::set_DH_tool_assemble(const std::string& r1_fname, const std::string& r2_fname)
{
    ROS_INFO_STREAM("Load r1 DH tool for assemble from: " << r1_fname);
    r1_DH_tool_assemble_ = io::LoadMatFromFile(r1_fname);
    r1_tool_assemble_inv_ = Eigen::Matrix4d::Identity(4, 4); 
    r1_tool_assemble_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  r1_DH_tool_assemble_(5, 2) * cos(0),
                          sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  r1_DH_tool_assemble_(5, 2) * sin(0),
                          0,       sin(0),         cos(0),        -r1_DH_tool_assemble_(5, 1),
                          0,       0,              0,              1;
    r1_tool_assemble_inv_ = math::PInv(r1_tool_assemble_inv_);

    ROS_INFO_STREAM("Load r2 DH tool for assemble from: " << r2_fname);
    r2_DH_tool_assemble_ = io::LoadMatFromFile(r2_fname);
    r2_tool_assemble_inv_ = Eigen::Matrix4d::Identity(4, 4); 
    r2_tool_assemble_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  r2_DH_tool_assemble_(5, 2) * cos(0),
                          sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  r2_DH_tool_assemble_(5, 2) * sin(0),
                          0,       sin(0),         cos(0),        -r2_DH_tool_assemble_(5, 1),
                          0,       0,              0,              1;
    r2_tool_assemble_inv_ = math::PInv(r2_tool_assemble_inv_);
}

void Lego::set_DH_tool_disassemble(const std::string& r1_fname, const std::string& r2_fname)
{
    ROS_INFO_STREAM("Load r1 DH tool for disassemble from: " << r1_fname);
    r1_DH_tool_disassemble_ = io::LoadMatFromFile(r1_fname);
    r1_tool_disassemble_inv_ = Eigen::Matrix4d::Identity(4, 4);
    r1_tool_disassemble_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  r1_DH_tool_disassemble_(5, 2) * cos(0),
                             sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  r1_DH_tool_disassemble_(5, 2) * sin(0),
                             0,       sin(0),         cos(0),        -r1_DH_tool_disassemble_(5, 1),
                             0,       0,              0,              1;
    r1_tool_disassemble_inv_ = math::PInv(r1_tool_disassemble_inv_);

    ROS_INFO_STREAM("Load r2 DH tool for disassemble from: " << r2_fname);
    r2_DH_tool_disassemble_ = io::LoadMatFromFile(r2_fname);
    r2_tool_disassemble_inv_ = Eigen::Matrix4d::Identity(4, 4);
    r2_tool_disassemble_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  r2_DH_tool_disassemble_(5, 2) * cos(0),
                             sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  r2_DH_tool_disassemble_(5, 2) * sin(0),
                             0,       sin(0),         cos(0),        -r2_DH_tool_disassemble_(5, 1),
                             0,       0,              0,              1;
    r2_tool_disassemble_inv_ = math::PInv(r2_tool_disassemble_inv_);
}

void Lego::set_DH_tool_alt(const std::string& r1_fname, const std::string& r2_fname)
{
    ROS_INFO_STREAM("Load r1 DH tool alt from: " << r1_fname);
    r1_DH_tool_alt_ = io::LoadMatFromFile(r1_fname);
    r1_tool_alt_inv_ = Eigen::Matrix4d::Identity(4, 4); 
    r1_tool_alt_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  r1_DH_tool_alt_(5, 2) * cos(0),
                 sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  r1_DH_tool_alt_(5, 2) * sin(0),
                 0,       sin(0),         cos(0),        -r1_DH_tool_alt_(5, 1),
                 0,       0,              0,              1;
    r1_tool_alt_inv_ = math::PInv(r1_tool_alt_inv_);

    ROS_INFO_STREAM("Load r2 DH tool alt from: " << r2_fname);
    r2_DH_tool_alt_ = io::LoadMatFromFile(r2_fname);
    r2_tool_alt_inv_ = Eigen::Matrix4d::Identity(4, 4); 
    r2_tool_alt_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  r2_DH_tool_alt_(5, 2) * cos(0),
                 sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  r2_DH_tool_alt_(5, 2) * sin(0),
                 0,       sin(0),         cos(0),        -r2_DH_tool_alt_(5, 1),
                 0,       0,              0,              1;
    r2_tool_alt_inv_ = math::PInv(r2_tool_alt_inv_);
}

void Lego::set_DH_tool_alt_assemble(const std::string& r1_fname, const std::string& r2_fname)
{
    ROS_INFO_STREAM("Load r1 DH tool for assemble from: " << r1_fname);
    r1_DH_tool_alt_assemble_ = io::LoadMatFromFile(r1_fname);
    r1_tool_alt_assemble_inv_ = Eigen::Matrix4d::Identity(4, 4); 
    r1_tool_alt_assemble_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  r1_DH_tool_alt_assemble_(5, 2) * cos(0),
                          sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  r1_DH_tool_alt_assemble_(5, 2) * sin(0),
                          0,       sin(0),         cos(0),        -r1_DH_tool_alt_assemble_(5, 1),
                          0,       0,              0,              1;
    r1_tool_alt_assemble_inv_ = math::PInv(r1_tool_alt_assemble_inv_);

    ROS_INFO_STREAM("Load r2 DH tool for assemble from: " << r2_fname);
    r2_DH_tool_alt_assemble_ = io::LoadMatFromFile(r2_fname);
    r2_tool_alt_assemble_inv_ = Eigen::Matrix4d::Identity(4, 4); 
    r2_tool_alt_assemble_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  r2_DH_tool_alt_assemble_(5, 2) * cos(0),
                          sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  r2_DH_tool_alt_assemble_(5, 2) * sin(0),
                          0,       sin(0),         cos(0),        -r2_DH_tool_alt_assemble_(5, 1),
                          0,       0,              0,              1;
    r2_tool_alt_assemble_inv_ = math::PInv(r2_tool_alt_assemble_inv_);
}

void Lego::set_assemble_plate_pose(const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond quat = yawAngle * pitchAngle * rollAngle;
    assemble_plate_.pose = Eigen::Matrix4d::Identity(4, 4);
    assemble_plate_.pose.block(0, 0, 3, 3) = quat.matrix();
    assemble_plate_.pose.col(3) << x, y, z, 1;
}

void Lego::set_storage_plate_pose(const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond quat = yawAngle * pitchAngle * rollAngle;
    storage_plate_.pose = Eigen::Matrix4d::Identity(4, 4);
    storage_plate_.pose.block(0, 0, 3, 3) = quat.matrix();
    storage_plate_.pose.col(3) << x, y, z, 1;
}

void Lego::calc_brick_loc(const lego_brick& brick, const lego_plate& plate, const int& orientation,
                          const int& brick_loc_x, const int& brick_loc_y, const int& brick_loc_z, 
                          Eigen::Matrix4d& out_pose)
{
    int brick_height = brick.height;
    int brick_width = brick.width;
    Eigen::Matrix4d refpose = plate.pose;
    Eigen::Matrix4d topleft_offset = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d brick_offset = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d brick_center_offset = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d z_90;
    z_90 << 0, -1, 0, 0, 
             1, 0, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    brick_offset.col(3) << brick_loc_x * P_len_ - brick_len_offset_,
                           brick_loc_y * P_len_ - brick_len_offset_,
                           brick_loc_z * brick_height_m_,
                           1;
    brick_center_offset.col(3) << (brick_height * P_len_ - brick_len_offset_) / 2.0,
                                  (brick_width * P_len_ - brick_len_offset_) / 2.0,
                                  0,
                                  1;
    topleft_offset.col(3) << -(plate.width * P_len_ - brick_len_offset_) / 2.0,
                             -(plate.height * P_len_ - brick_len_offset_) / 2.0,
                             0,
                             1;
    out_pose = refpose * topleft_offset * brick_offset * brick_center_offset;
    if(orientation == 1)
    {
        brick_center_offset(1, 3) = -brick_center_offset(1, 3);
        out_pose = refpose * topleft_offset * brick_offset * z_90 * brick_center_offset ;
    }
}


void Lego::brick_pose_in_stock(const std::string& name, const int& press_side, const int& press_offset, Eigen::Matrix4d& T)
{
    lego_brick l_brick = brick_map_[name];
    int brick_height = l_brick.height;
    int brick_width = l_brick.width;
    brick_map_[name].press_side = press_side;
    brick_map_[name].press_offset = press_offset;
    
    Eigen::Quaterniond quat;
    double x, y, z;
    Eigen::Matrix4d y_180, z_180, z_90;
    y_180 << -1, 0, 0, 0, 
             0, 1, 0, 0, 
             0, 0, -1, 0,
             0, 0, 0, 1;
    z_180 << -1, 0, 0, 0,
             0, -1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    z_90 << 0, -1, 0, 0, 
            1, 0, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Eigen::Matrix3d rot_mtx = Eigen::Matrix3d::Identity(3, 3);
    Eigen::Matrix4d brick_pose_mtx = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d grab_offset_mtx = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d grab_pose_mtx = Eigen::Matrix4d::Identity(4, 4);
    
    quat.x() = l_brick.quat_x;
    quat.y() = l_brick.quat_y;
    quat.z() = l_brick.quat_z;
    quat.w() = l_brick.quat_w;
    rot_mtx = quat.normalized().toRotationMatrix();
    brick_pose_mtx.block(0, 0, 3, 3) << rot_mtx;
    brick_pose_mtx(0, 3) = l_brick.x;
    brick_pose_mtx(1, 3) = l_brick.y;
    brick_pose_mtx(2, 3) = l_brick.z;
    brick_pose_mtx = brick_pose_mtx * y_180;
    int center_press_offset = 0;

    if(press_side == 1)
    {
        grab_offset_mtx(0, 3) = (brick_height * P_len_ - brick_len_offset_) / 2.0;
        if(brick_width == 1)
        {
            grab_offset_mtx(1, 3) = (P_len_ - brick_len_offset_) / 2.0;
        }
        else
        {
            center_press_offset = (brick_width / 2) - 1;
            grab_offset_mtx(1, 3) = (press_offset - center_press_offset) * P_len_;
        }
        grab_offset_mtx = grab_offset_mtx * z_180;
    }
    else if(press_side == 2)
    {
        if(brick_height == 1)
        {
            grab_offset_mtx(0, 3) = -(P_len_ - brick_len_offset_) / 2.0;
        }
        else
        {
            center_press_offset = (brick_height / 2) - 1;
            grab_offset_mtx(0, 3) = (center_press_offset - press_offset) * P_len_;
        }
        grab_offset_mtx(1, 3) = (brick_width * P_len_ - brick_len_offset_) / 2.0;
        grab_offset_mtx = grab_offset_mtx * z_180 * z_90;
    }
    else if(press_side == 3)
    {
        if(brick_height == 1)
        {
            grab_offset_mtx(0, 3) = (P_len_ - brick_len_offset_) / 2.0;
        }
        else
        {
            center_press_offset = (brick_height / 2) - 1;
            grab_offset_mtx(0, 3) = (center_press_offset - press_offset) * P_len_;
        }
        grab_offset_mtx(1, 3) = -(brick_width * P_len_ - brick_len_offset_) / 2.0;
        grab_offset_mtx = grab_offset_mtx * z_90;
    }
    else if(press_side == 4)
    {
        grab_offset_mtx(0, 3) = -(brick_height * P_len_ - brick_len_offset_) / 2.0;
        if(brick_width == 1)
        {
            grab_offset_mtx(1, 3) = -(P_len_ - brick_len_offset_) / 2.0;
        }
        else
        {
            center_press_offset = (brick_width / 2) - 1;
            grab_offset_mtx(1, 3) = (press_offset - center_press_offset) * P_len_;
        }
        grab_offset_mtx = grab_offset_mtx;
    }
    grab_pose_mtx = brick_pose_mtx * grab_offset_mtx;  
    T = grab_pose_mtx;
}

void Lego::support_pose_down_pre(const int& x, const int& y, const int& z, const int& ori, Eigen::Matrix4d& T)
{
    Eigen::MatrixXd init_q(r1_robot_dof_, 1);
    Eigen::Matrix4d init_T, tmp_T;
    int goal_x, goal_y, goal_z, goal_ori, goal_press_side;
    goal_z = z - 1;
    if(ori == 0)
    {
        init_q.col(0) << 0, 0, 0, 0, 0, 0;
        goal_x = x - 3;
        goal_y = y;
        goal_ori = 0;
        goal_press_side = 1;
    }
    else if(ori == 1)
    {
        init_q.col(0) << -90, 0, 0, 0, 0, 0;
        goal_x = x;
        goal_y = y + 3;
        goal_ori = 1;
        goal_press_side = 4;
    }
    else if(ori == 2)
    {
        init_q.col(0) << 90, 0, 0, 0, 0, 0;
        goal_x = x;
        goal_y = y - 3;
        goal_ori = 1;
        goal_press_side = 1;
    }
    else
    {
        init_q.col(0) << 180, 0, 0, 0, 0, 0;
        goal_x = x + 3;
        goal_y = y;
        goal_ori = 0;
        goal_press_side = 4;
    }
    init_T = math::FK(init_q, robot_DH_tool_r1(), robot_base_r1(), false);
    assemble_pose_from_top(goal_x, goal_y, goal_z, goal_ori, goal_press_side, tmp_T);
    init_T.col(3) << tmp_T(0, 3), tmp_T(1, 3), tmp_T(2, 3), 1;
    T = init_T;
}

void Lego::support_pose_down(const int& x, const int& y, const int& z, const int& ori, Eigen::Matrix4d& T)
{
    Eigen::MatrixXd init_q(r1_robot_dof_, 1);
    Eigen::Matrix4d init_T, tmp_T;
    int goal_x, goal_y, goal_z, goal_ori, goal_press_side;
    goal_z = z - 1;
    if(ori == 0)
    {
        init_q.col(0) << 0, 0, 0, 0, 0, 0;
        goal_ori = 0;
        goal_x = x + 1;
        goal_y = y;
        goal_press_side = 1;
    }
    else if(ori == 1)
    {
        init_q.col(0) << -90, 0, 0, 0, 0, 0;
        goal_ori = 1;
        goal_x = x;
        goal_y = y - 1;
        goal_press_side = 4;
    }
    else if(ori == 2)
    {
        init_q.col(0) << 90, 0, 0, 0, 0, 0;
        goal_ori = 1;
        goal_x = x;
        goal_y = y + 1;
        goal_press_side = 1;
    }
    else
    {
        init_q.col(0) << 180, 0, 0, 0, 0, 0;
        goal_ori = 0;
        goal_x = x - 1;
        goal_y = y;
        goal_press_side = 4;
    }
    init_T = math::FK(init_q, robot_DH_tool_r1(), robot_base_r1(), false);
    assemble_pose_from_top(goal_x, goal_y, goal_z, goal_ori, goal_press_side, tmp_T);
    init_T.col(3) << tmp_T(0, 3), tmp_T(1, 3), tmp_T(2, 3) + (brick_height_m_ - (P_len_ - brick_len_offset_)), 1;
    Eigen::Matrix4d offset_T = Eigen::Matrix4d::Identity(4, 4);
    offset_T.col(3) << 0, 0, -lever_wall_height_, 1;
    T = init_T * offset_T;
}

void Lego::support_pose(const int& x, const int& y, const int& z, const int& ori, Eigen::Matrix4d& T)
{
    Eigen::MatrixXd init_q(r1_robot_dof_, 1);
    Eigen::Matrix4d init_T, tmp_T;
    int goal_x, goal_y, goal_z, goal_ori, goal_press_side;
    goal_z = z;

    if(ori == 0)
    {
        init_q.col(0) << 0, 0, 0, 0, 0, 0;
        goal_ori = 0;
        goal_press_side = 1;
        goal_x = x + 1;
        goal_y = y;
    }
    else if(ori == 1)
    {
        init_q.col(0) << -90, 0, 0, 0, 0, 0;
        goal_ori = 1;
        goal_press_side = 4;
        goal_x = x;
        goal_y = y - 1;
    }
    else if(ori == 2)
    {
        init_q.col(0) << 90, 0, 0, 0, 0, 0;
        goal_ori = 1;
        goal_press_side = 1;
        goal_x = x;
        goal_y = y + 1;
    }
    else
    {
        init_q.col(0) << 180, 0, 0, 0, 0, 0;
        goal_ori = 0;
        goal_press_side = 4;
        goal_x = x - 1;
        goal_y = y;
    }
    init_T = math::FK(init_q, robot_DH_tool_r1(), robot_base_r1(), false);
    assemble_pose_from_top(goal_x, goal_y, goal_z, goal_ori, goal_press_side, tmp_T);
    init_T.col(3) << tmp_T(0, 3), tmp_T(1, 3), tmp_T(2, 3) + (brick_height_m_ - (P_len_ - brick_len_offset_)), 1;
    Eigen::Matrix4d offset_T = Eigen::Matrix4d::Identity(4, 4);
    offset_T.col(3) << 0, 0, -lever_wall_height_, 1;
    T = init_T * offset_T;
}

void Lego::assemble_pose_from_top(const int& press_x, const int& press_y, const int& press_z, const int& press_ori, const int& press_side, Eigen::Matrix4d& T)
{
    lego_brick l_brick = brick_map_["b9_1"];
    
    Eigen::Quaterniond quat;
    double x, y, z;
    Eigen::Matrix4d y_180, z_180, z_90;
    y_180 << -1, 0, 0, 0, 
             0, 1, 0, 0, 
             0, 0, -1, 0,
             0, 0, 0, 1;
    z_180 << -1, 0, 0, 0,
             0, -1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    z_90 << 0, -1, 0, 0, 
            1, 0, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Eigen::Matrix3d rot_mtx = Eigen::Matrix3d::Identity(3, 3);
    Eigen::Matrix4d brick_pose_mtx = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d grab_offset_mtx = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d grab_pose_mtx = Eigen::Matrix4d::Identity(4, 4);
    int side;

    if(press_ori == 0){
        if(press_side == 1){
            side = 1;
        }
        else if(press_side == 2){
            side = 1;
        }
        else if(press_side == 3){
            side = 4;
        }
        else if(press_side == 4){
            side = 4;
        }
    }
    else if(press_ori == 1){
        if(press_side == 1){
            side = 1;
        }
        else if(press_side == 2){
            side = 4;
        }
        else if(press_side == 3){
            side = 1;
        }
        else if(press_side == 4){
            side = 4;
        }
    }
    calc_brick_loc(l_brick, assemble_plate_, press_ori, press_x, press_y, press_z-1, brick_pose_mtx);
    brick_pose_mtx = brick_pose_mtx * y_180;
    int brick_height = 1;
    int brick_width = 2;
    if(side == 1)
    {
        grab_offset_mtx(0, 3) = (brick_height * P_len_ - brick_len_offset_) / 2.0;
        grab_offset_mtx(1, 3) = 0;  
        grab_offset_mtx = grab_offset_mtx * z_180;
    }
    else if(side == 2)
    {
        grab_offset_mtx(0, 3) = 0;
        grab_offset_mtx(1, 3) = (brick_width * P_len_ - brick_len_offset_) / 2.0;
        grab_offset_mtx = grab_offset_mtx * z_180 * z_90;
    }
    else if(side == 3)
    {
        grab_offset_mtx(0, 3) = 0;
        grab_offset_mtx(1, 3) = -(brick_width * P_len_ - brick_len_offset_) / 2.0;
        grab_offset_mtx = grab_offset_mtx * z_90;
    }
    else if(side == 4)
    {
        grab_offset_mtx(0, 3) = -(brick_height * P_len_ - brick_len_offset_) / 2.0;
        grab_offset_mtx(1, 3) = 0;
    }
    grab_pose_mtx = brick_pose_mtx * grab_offset_mtx;
    T = grab_pose_mtx;
}

bool Lego::joint_in_range(const math::VectorJd& theta, const bool& is_rad)
{
    math::VectorJd theta_deg = theta;
    if(is_rad)
    {
        // Rad to Deg
        for(int i=0; i<theta.rows(); i++)
        {
            theta_deg(i) = theta_deg(i) / PI * 180;
        }
    }
    for(int i=0; i<theta.rows(); i++)
    {
        if(theta_deg(i) < thetamax_(i, 0) || theta_deg(i) > thetamax_(i, 1))
        {
            return false;
        }
    }
    return true;
}

math::VectorJd Lego::IK(const math::VectorJd& cur_q, const Eigen::Matrix4d& goal_T, const Eigen::MatrixXd& DH, const Eigen::Matrix4d& T_base, const Eigen::Matrix4d& T_base_inv,
                         const Eigen::Matrix4d& T_tool_inv, const bool& joint_rad, bool& status)
{
    double eps = 1e-10;
    status = false;
    math::VectorJd theta = cur_q;
    Eigen::MatrixXd DH_cur = DH;
    if(!joint_rad)
    {
        // Deg to Rad
        for(int i=0; i<cur_q.rows(); i++)
        {
            theta(i) = theta(i) * PI / 180;
        }
    }
    math::VectorJd cur_theta = theta;
    math::VectorJd theta_tmp = theta;
    Eigen::Matrix4d T = T_base_inv * goal_T * T_tool_inv;
    Eigen::Matrix3d R = T.block(0, 0, 3, 3);
    Eigen::Matrix<double, 3, 1> P = T.block(0, 3, 3, 1);
    double X, Y, Z, r, r2, a1, a12, a2, a22, a3, a32, d4, d42, m, e, c, l, l2, h, f1, f2, t1, t2, t3, k, g1, g2, q1, q2, min_diff;
    double th1_tmp, th2_tmp, th3_tmp, th4_tmp, th5_tmp, th6_tmp;
    X = P(0, 0);
    Y = P(1, 0);
    Z = P(2, 0);
    r = sqrt(pow(X, 2) + pow(Y, 2) + pow(Z, 2));
    a1 = DH(0, 2);
    a2 = DH(1, 2);
    a3 = DH(2, 2);
    d4 = DH(3, 1);
    r2 = pow(r, 2);
    a12 = pow(a1, 2);
    a22 = pow(a2, 2);
    a32 = pow(a3, 2);
    d42 = pow(d4, 2);
    m = a32 + d42;
    e = 2 * r2;
    c = 4 * a12;
    l = a2 * (2 * m + 2 * a12 + 2 * a22 - c - e);
    l2 = pow(l, 2);
    h = (c + e) * m - pow((m + a12 + a22), 2) + a12 * e + a22 * e + 4 * a12 * a22 - 4 * a12 * pow(Z, 2) - pow(r, 4);

    double cond1, cond2, th2_tmp1, th2_tmp2;
    cond1 = 4 * l2 + 16 * a22 * h;
    min_diff = 10000000;
    Eigen::MatrixXd th23_candidates;
    int th23_candidate_cnt, th1_candidate_cnt, all_candidate_cnt;
    th23_candidate_cnt = 0;
    th1_candidate_cnt = 0;
    all_candidate_cnt = 0;
    th23_candidates.resize(8, 2);

    if(cond1 < 0 && abs(cond1) < 0.0001)
    {
        cond1 = 0;
    }
    if(cond1 >= 0)
    {
        f1 = (-2 * l + sqrt(cond1)) / (8 * a22 + eps);
        cond2 = d42 + a32 - pow(f1, 2);
        if(cond2 >= 0)
        {
            // First candidate
            th3_tmp = 2 * atan((-d4 + sqrt(cond2)) / (a3 + f1 + eps));

            f1 = -sin(th3_tmp) * d4 + a3 * cos(th3_tmp);
            f2 = cos(th3_tmp) * d4 + a3 * sin(th3_tmp);
            t1 = f1 + a2;
            k = pow(f1, 2) + pow(f2, 2) + 2 * f1 * a2 + a12 + a22;
            t2 = (r2 - k) / (2 * a1 + eps);
            t1 = f2;
            t2 = -f1-a2;
            t3 = Z;
            th2_tmp1 = 2 * atan((t2 + sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + PI / 2;
            th2_tmp2 = 2 * atan((t2 - sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + PI / 2;

            if(th3_tmp < thetamax_rad_(2, 1) && th3_tmp > thetamax_rad_(2, 0))
            {
                if(th2_tmp1 < thetamax_rad_(1, 1) && th2_tmp1 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp1, th3_tmp;
                    th23_candidate_cnt ++;
                }
                if(th2_tmp2 < thetamax_rad_(1, 1) && th2_tmp2 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp2, th3_tmp;
                    th23_candidate_cnt ++;
                }
            }

            // Second candidate
            th3_tmp = 2 * atan((-d4 - sqrt(cond2)) / (a3 + f1 + eps));
            f1 = -sin(th3_tmp) * d4 + a3 * cos(th3_tmp);
            f2 = cos(th3_tmp) * d4 + a3 * sin(th3_tmp);
            t1 = f1 + a2;
            k = pow(f1, 2) + pow(f2, 2) + 2 * f1 * a2 + a12 + a22;
            t2 = (r2 - k) / (2 * a1 + eps);
            t1 = f2;
            t2 = -f1-a2;
            t3 = Z;
            th2_tmp1 = 2 * atan((t2 + sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + PI / 2;
            th2_tmp2 = 2 * atan((t2 - sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + PI / 2;
            if(th3_tmp < thetamax_rad_(2, 1) && th3_tmp > thetamax_rad_(2, 0))
            {
                if(th2_tmp1 < thetamax_rad_(1, 1) && th2_tmp1 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp1, th3_tmp;
                    th23_candidate_cnt ++;
                }
                if(th2_tmp2 < thetamax_rad_(1, 1) && th2_tmp2 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp2, th3_tmp;
                    th23_candidate_cnt ++;
                }
            }
        }
        f1 = (-2 * l - sqrt(cond1)) / (8 * a22 + eps);
        cond2 = d42 + a32 - pow(f1, 2);
        if(cond2)
        {
            // Third candidate
            th3_tmp = 2 * atan((-d4 + sqrt(cond2)) / (a3 + f1 + eps));
            f1 = -sin(th3_tmp) * d4 + a3 * cos(th3_tmp);
            f2 = cos(th3_tmp) * d4 + a3 * sin(th3_tmp);
            t1 = f1 + a2;
            k = pow(f1, 2) + pow(f2, 2) + 2 * f1 * a2 + a12 + a22;
            t2 = (r2 - k) / (2 * a1 + eps);
            t1 = f2;
            t2 = -f1-a2;
            t3 = Z;
            th2_tmp1 = 2 * atan((t2 + sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + PI / 2;
            th2_tmp2 = 2 * atan((t2 - sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + PI / 2;
            if(th3_tmp < thetamax_rad_(2, 1) && th3_tmp > thetamax_rad_(2, 0))
            {
                if(th2_tmp1 < thetamax_rad_(1, 1) && th2_tmp1 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp1, th3_tmp;
                    th23_candidate_cnt ++;
                }
                if(th2_tmp2 < thetamax_rad_(1, 1) && th2_tmp2 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp2, th3_tmp;
                    th23_candidate_cnt ++;
                }
            }
            
            // Fourth candidate
            th3_tmp = 2 * atan((-d4 - sqrt(cond2)) / (a3 + f1 + eps));
            f1 = -sin(th3_tmp) * d4 + a3 * cos(th3_tmp);
            f2 = cos(th3_tmp) * d4 + a3 * sin(th3_tmp);
            t1 = f1 + a2;
            k = pow(f1, 2) + pow(f2, 2) + 2 * f1 * a2 + a12 + a22;
            t2 = (r2 - k) / (2 * a1 + eps);
            t1 = f2;
            t2 = -f1-a2;
            t3 = Z;
            th2_tmp1 = 2 * atan((t2 + sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + PI / 2;
            th2_tmp2 = 2 * atan((t2 - sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + PI / 2;
            if(th3_tmp < thetamax_rad_(2, 1) && th3_tmp > thetamax_rad_(2, 0))
            {
                if(th2_tmp1 < thetamax_rad_(1, 1) && th2_tmp1 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp1, th3_tmp;
                    th23_candidate_cnt ++;
                }
                if(th2_tmp2 < thetamax_rad_(1, 1) && th2_tmp2 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp2, th3_tmp;
                    th23_candidate_cnt ++;
                }
            }
        }
    }
    else
    {
        std::cout<<"IK failed condition 1"<<std::endl;
        status = false;
        return cur_q;
    }
    
    Eigen::MatrixXd th1_candidates, candidates;
    Eigen::Matrix4d verify_T;
    th1_candidates.resize(2, 1);
    candidates.resize(32, r1_robot_dof_);
    double th1_tmp1, th1_tmp2;
    for(int i=0; i<th23_candidate_cnt; i++)
    {
        th2_tmp = th23_candidates(i, 0) - PI / 2;
        th3_tmp = th23_candidates(i, 1);
        th1_candidate_cnt = 0;

        g1 = f1 * cos(th2_tmp) + f2 * sin(th2_tmp) + a2 * cos(th2_tmp);
        g2 = f1 * sin(th2_tmp) - f2 * cos(th2_tmp) + a2 * sin(th2_tmp);
        q1 = g1+a1;
        q2 = 0;
        cond1 = pow(q2, 2) + pow(q1, 2) - pow(X, 2);
        q1 = 0;
        q2 = g1+a1;
        th1_tmp1 = 2 * atan((q2 + sqrt(pow(q2, 2) + pow(q1, 2) - pow(Y, 2))) / (q1 + Y + eps));
        th1_tmp2 = 2 * atan((q2 - sqrt(pow(q2, 2) + pow(q1, 2) - pow(Y, 2))) / (q1 + Y + eps));
        if(th1_tmp1 < thetamax_rad_(0, 1) && th1_tmp1 > thetamax_rad_(0, 0))
        {
            th1_candidates.row(th1_candidate_cnt) << th1_tmp1;
            th1_candidate_cnt ++;
        }
        if(th1_tmp2 < thetamax_rad_(0, 1) && th1_tmp2 > thetamax_rad_(0, 0))
        {
            th1_candidates.row(th1_candidate_cnt) << th1_tmp2;
            th1_candidate_cnt ++;
        }
        for(int j=0; j<th1_candidate_cnt; j++)
        {
            theta_tmp(0) = th1_candidates(j, 0);
            theta_tmp(1) = th2_tmp + PI / 2;;
            theta_tmp(2) = th3_tmp;
            DH_cur.col(0) = DH.col(0) + theta_tmp;
            Eigen::Matrix3d R03 = Eigen::Matrix3d::Identity(3, 3);
            Eigen::MatrixXd a = DH_cur.col(3);
            Eigen::MatrixXd q = DH_cur.col(0);
            Eigen::Matrix3d temp(3, 3); 
            for(int k=0; k<3; k++)
            {
                temp << cos(q(k)), -sin(q(k)) * cos(a(k)),  sin(q(k)) * sin(a(k)),  
                        sin(q(k)),  cos(q(k)) * cos(a(k)), -cos(q(k)) * sin(a(k)),  
                        0,          sin(a(k)),              cos(a(k));
                R03 = R03 * temp;
            }
            Eigen::Matrix3d R36 = math::PInv(R03) * R;
            th5_tmp = acos(-R36(2, 2));
            double s5 = sin(th5_tmp) + eps;

            if(abs(s5) <= 0.001)
            {
                th4_tmp = 0;
                th5_tmp = 0;
                th6_tmp = atan2(R36(0, 1), R36(0, 0));
                theta_tmp(3) = th4_tmp;
                theta_tmp(4) = th5_tmp;
                theta_tmp(5) = th6_tmp;
                if(joint_in_range(theta_tmp, 1))
                {
                    candidates.row(all_candidate_cnt) << theta_tmp.transpose(); 
                    all_candidate_cnt ++;
                }

                th5_tmp = PI;
                th6_tmp = atan2(R36(1, 0), -R36(1, 1));
                theta_tmp(3) = th4_tmp;
                theta_tmp(4) = th5_tmp;
                theta_tmp(5) = th6_tmp;
                if(joint_in_range(theta_tmp, 1))
                {
                    candidates.row(all_candidate_cnt) << theta_tmp.transpose(); 
                    all_candidate_cnt ++;
                }
            }
            else
            {
                double th4_1 = atan2(R36(1, 2) / s5, R36(0, 2) / s5);
                double th6_1 = atan2(R36(2, 1) / s5, R36(2, 0) / s5);
                double sum1 = sqrt(pow(th5_tmp, 2) + pow(th4_1, 2) + pow(th6_1, 2));
                s5 = sin(-th5_tmp);
                double th4_2 = atan2(R36(1, 2) / s5, R36(0, 2) / s5);
                double th6_2 = atan2(R36(2, 1) / s5, R36(2, 0) / s5);
                double sum2 = sqrt(pow(th5_tmp, 2) + pow(th4_2, 2) + pow(th6_2, 2));

                th4_tmp = th4_1;
                th6_tmp = th6_1;
                theta_tmp(3) = th4_tmp;
                theta_tmp(4) = th5_tmp;
                theta_tmp(5) = th6_tmp;
                if(joint_in_range(theta_tmp, 1))
                {
                    candidates.row(all_candidate_cnt) << theta_tmp.transpose(); 
                    all_candidate_cnt ++;
                }
                
                th5_tmp = -th5_tmp;
                th4_tmp = th4_2;
                th6_tmp = th6_2;
                theta_tmp(3) = th4_tmp;
                theta_tmp(4) = th5_tmp;
                theta_tmp(5) = th6_tmp;
                if(joint_in_range(theta_tmp, 1))
                {
                    candidates.row(all_candidate_cnt) << theta_tmp.transpose(); 
                    all_candidate_cnt ++;
                }
            } 
        }  
    }
    status = false;
    for(int i=0; i<all_candidate_cnt; i++)
    {   
        for(int j=-1; j<2; j++)
        {
            theta_tmp = candidates.row(i);
            theta_tmp(5) = theta_tmp(5) + j * 2 * PI;
            verify_T = math::FK(theta_tmp, DH, T_base, 1);
            
            if(verify_T.isApprox(goal_T, 0.1) && verify_T.col(3).isApprox(goal_T.col(3), 0.01) && (theta_tmp - cur_theta).norm() < min_diff && joint_in_range(theta_tmp, 1))
            {
                theta = theta_tmp;
                min_diff = (theta_tmp - cur_theta).norm();
                status = true;            
            }
        }
        
    }
    if(!status)
    {
        std::cout<<"IK failed! No valid candidate."<<std::endl;
        return cur_q;
    }

    // Rad to Deg
    for(int i=0; i<theta.rows(); i++)
    {
        theta(i) = theta(i) * 180 / PI;
    }
    return theta;
}

void Lego::calc_brick_grab_pose(const std::string& name, const bool& assemble_pose, const bool& take_brick,
                                       const int& brick_assemble_x, const int& brick_assemble_y, const int& brick_assemble_z, 
                                       const int& orientation, const int& press_side, Eigen::MatrixXd& T)
{
    lego_brick l_brick = brick_map_[name];
    int brick_height = l_brick.height;
    int brick_width = l_brick.width;
    brick_map_[name].press_side = press_side;
    
    Eigen::Quaterniond quat;
    double x, y, z;
    Eigen::Matrix4d y_180, z_180, z_90;
    y_180 << -1, 0, 0, 0, 
             0, 1, 0, 0, 
             0, 0, -1, 0,
             0, 0, 0, 1;
    z_180 << -1, 0, 0, 0,
             0, -1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    z_90 << 0, -1, 0, 0, 
            1, 0, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Eigen::Matrix3d rot_mtx = Eigen::Matrix3d::Identity(3, 3);
    Eigen::Matrix4d brick_pose_mtx = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d grab_offset_mtx = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d grab_pose_mtx = Eigen::Matrix4d::Identity(4, 4);
    
    // Get pose to grab the brick
    if(take_brick)
    {
        // Brick on storage plate
        if(l_brick.in_stock)
        {
            quat.x() = l_brick.quat_x;
            quat.y() = l_brick.quat_y;
            quat.z() = l_brick.quat_z;
            quat.w() = l_brick.quat_w;
            rot_mtx = quat.normalized().toRotationMatrix();
            brick_pose_mtx.block(0, 0, 3, 3) << rot_mtx;
            brick_pose_mtx(0, 3) = l_brick.x;
            brick_pose_mtx(1, 3) = l_brick.y;
            brick_pose_mtx(2, 3) = l_brick.z;
            brick_pose_mtx = brick_pose_mtx * y_180;

            if(press_side == 1)
            {
                grab_offset_mtx(0, 3) = (brick_height * P_len_ - brick_len_offset_) / 2.0;
                if(brick_width % 2 == 0)
                {
                    grab_offset_mtx(1, 3) = 0;
                }
                else
                {
                    grab_offset_mtx(1, 3) = (P_len_ - brick_len_offset_) / 2.0;
                }  
                grab_offset_mtx = grab_offset_mtx * z_180;
            }
            else if(press_side == 2)
            {
                if(brick_height % 2 == 0)
                {
                    grab_offset_mtx(0, 3) = 0;
                }
                else
                {
                    grab_offset_mtx(0, 3) = -(P_len_ - brick_len_offset_) / 2.0;
                }
                grab_offset_mtx(1, 3) = (brick_width * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx = grab_offset_mtx * z_180 * z_90;
            }
            else if(press_side == 3)
            {
                if(brick_height % 2 == 0)
                {
                    grab_offset_mtx(0, 3) = 0;
                }
                else
                {
                    grab_offset_mtx(0, 3) = (P_len_ - brick_len_offset_) / 2.0;
                }
                grab_offset_mtx(1, 3) = -(brick_width * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx = grab_offset_mtx * z_90;
            }
            else if(press_side == 4)
            {
                grab_offset_mtx(0, 3) = -(brick_height * P_len_ - brick_len_offset_) / 2.0;
                if(brick_width % 2 == 0)
                {
                    grab_offset_mtx(1, 3) = 0;
                }
                else
                {
                    grab_offset_mtx(1, 3) = -(P_len_ - brick_len_offset_) / 2.0;
                }  
                grab_offset_mtx = grab_offset_mtx;
            }
            grab_pose_mtx = brick_pose_mtx * grab_offset_mtx;
        }  
        // Brick on assemble plate
        else
        {
            quat.x() = l_brick.cur_quat.x();
            quat.y() = l_brick.cur_quat.y();
            quat.z() = l_brick.cur_quat.z();
            quat.w() = l_brick.cur_quat.w();
            rot_mtx = quat.normalized().toRotationMatrix();
            brick_pose_mtx.block(0, 0, 3, 3) << rot_mtx;
            brick_pose_mtx(0, 3) = l_brick.cur_x;
            brick_pose_mtx(1, 3) = l_brick.cur_y;
            brick_pose_mtx(2, 3) = l_brick.cur_z;
            brick_pose_mtx = brick_pose_mtx * y_180;

            if(press_side == 1)
            {
                grab_offset_mtx(0, 3) = (brick_height * P_len_ - brick_len_offset_) / 2.0;
                if(brick_width % 2 == 0)
                {
                    grab_offset_mtx(1, 3) = 0;
                }
                else
                {
                    grab_offset_mtx(1, 3) = (P_len_ - brick_len_offset_) / 2.0;
                }  
                grab_offset_mtx = grab_offset_mtx * z_180;
            }
            else if(press_side == 2)
            {
                if(brick_height % 2 == 0)
                {
                    grab_offset_mtx(0, 3) = 0;
                }
                else
                {
                    grab_offset_mtx(0, 3) = -(P_len_ - brick_len_offset_) / 2.0;
                }
                grab_offset_mtx(1, 3) = (brick_width * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx = grab_offset_mtx * z_180 * z_90;
            }
            else if(press_side == 3)
            {
                if(brick_height % 2 == 0)
                {
                    grab_offset_mtx(0, 3) = 0;
                }
                else
                {
                    grab_offset_mtx(0, 3) = (P_len_ - brick_len_offset_) / 2.0;
                }
                grab_offset_mtx(1, 3) = -(brick_width * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx = grab_offset_mtx * z_90;
            }
            else if(press_side == 4)
            {
                grab_offset_mtx(0, 3) = -(brick_height * P_len_ - brick_len_offset_) / 2.0;
                if(brick_width % 2 == 0)
                {
                    grab_offset_mtx(1, 3) = 0;
                }
                else
                {
                    grab_offset_mtx(1, 3) = -(P_len_ - brick_len_offset_) / 2.0;
                }  
                grab_offset_mtx = grab_offset_mtx;
            }
            grab_pose_mtx = brick_pose_mtx * grab_offset_mtx;
        }
    }
    // Get pose to place the brick
    else
    {
        // Place on storage plate
        if(!assemble_pose)
        {
            quat.x() = l_brick.quat_x;
            quat.y() = l_brick.quat_y;
            quat.z() = l_brick.quat_z;
            quat.w() = l_brick.quat_w;
            rot_mtx = quat.normalized().toRotationMatrix();
            brick_pose_mtx.block(0, 0, 3, 3) << rot_mtx;
            brick_pose_mtx(0, 3) = l_brick.x;
            brick_pose_mtx(1, 3) = l_brick.y;
            brick_pose_mtx(2, 3) = l_brick.z;
            brick_pose_mtx = brick_pose_mtx * y_180;

            if(press_side == 1)
            {
                grab_offset_mtx(0, 3) = (brick_height * P_len_ - brick_len_offset_) / 2.0;
                if(brick_width % 2 == 0)
                {
                    grab_offset_mtx(1, 3) = 0;
                }
                else
                {
                    grab_offset_mtx(1, 3) = (P_len_ - brick_len_offset_) / 2.0;
                }  
                grab_offset_mtx = grab_offset_mtx * z_180;
            }
            else if(press_side == 2)
            {
                if(brick_height % 2 == 0)
                {
                    grab_offset_mtx(0, 3) = 0;
                }
                else
                {
                    grab_offset_mtx(0, 3) = -(P_len_ - brick_len_offset_) / 2.0;
                }
                grab_offset_mtx(1, 3) = (brick_width * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx = grab_offset_mtx * z_180 * z_90;
            }
            else if(press_side == 3)
            {
                if(brick_height % 2 == 0)
                {
                    grab_offset_mtx(0, 3) = 0;
                }
                else
                {
                    grab_offset_mtx(0, 3) = (P_len_ - brick_len_offset_) / 2.0;
                }
                grab_offset_mtx(1, 3) = -(brick_width * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx = grab_offset_mtx * z_90;
            }
            else if(press_side == 4)
            {
                grab_offset_mtx(0, 3) = -(brick_height * P_len_ - brick_len_offset_) / 2.0;
                if(brick_width % 2 == 0)
                {
                    grab_offset_mtx(1, 3) = 0;
                }
                else
                {
                    grab_offset_mtx(1, 3) = -(P_len_ - brick_len_offset_) / 2.0;
                }  
                grab_offset_mtx = grab_offset_mtx;
            }
            grab_pose_mtx = brick_pose_mtx * grab_offset_mtx;
        }  
        // Place on assemble plate
        else
        {
            calc_brick_loc(l_brick, assemble_plate_, orientation, 
                           brick_assemble_x, brick_assemble_y, brick_assemble_z, brick_pose_mtx);

            brick_pose_mtx = brick_pose_mtx * y_180;

            if(press_side == 1)
            {
                grab_offset_mtx(0, 3) = (brick_height * P_len_ - brick_len_offset_) / 2.0;
                if(brick_width % 2 == 0)
                {
                    grab_offset_mtx(1, 3) = 0;
                }
                else
                {
                    grab_offset_mtx(1, 3) = (P_len_ - brick_len_offset_) / 2.0;
                }  
                grab_offset_mtx = grab_offset_mtx * z_180;
            }
            else if(press_side == 2)
            {
                if(brick_height % 2 == 0)
                {
                    grab_offset_mtx(0, 3) = 0;
                }
                else
                {
                    grab_offset_mtx(0, 3) = -(P_len_ - brick_len_offset_) / 2.0;
                }
                grab_offset_mtx(1, 3) = (brick_width * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx = grab_offset_mtx * z_180 * z_90;
            }
            else if(press_side == 3)
            {
                if(brick_height % 2 == 0)
                {
                    grab_offset_mtx(0, 3) = 0;
                }
                else
                {
                    grab_offset_mtx(0, 3) = (P_len_ - brick_len_offset_) / 2.0;
                }
                grab_offset_mtx(1, 3) = -(brick_width * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx = grab_offset_mtx * z_90;
            }
            else
            {
                grab_offset_mtx(0, 3) = -(brick_height * P_len_ - brick_len_offset_) / 2.0;
                if(brick_width % 2 == 0)
                {
                    grab_offset_mtx(1, 3) = 0;
                }
                else
                {
                    grab_offset_mtx(1, 3) = -(P_len_ - brick_len_offset_) / 2.0;
                }  
            }
            grab_pose_mtx = brick_pose_mtx * grab_offset_mtx;
        }
    }
    T = grab_pose_mtx;
}

void Lego::calc_brick_sup_pose(const std::string&name, const Eigen::MatrixXd& cart_T, const int &dx,
    const int &dy, const int &dz, const bool &shift, Eigen::MatrixXd &T)
{
    // copy the sup pose's orientation
    Eigen::Matrix4d sup_pose_mtx = Eigen::Matrix4d::Identity(4, 4);
    sup_pose_mtx.block(0, 0, 3, 3) = T.block(0, 0, 3, 3);

    // copy the lego's x y z
    sup_pose_mtx.col(3) << cart_T(0, 3), cart_T(1, 3), cart_T(2, 3), 1;

    // calculate the sup offset
    Eigen::Matrix4d grab_offset_mtx = Eigen::Matrix4d::Identity(4, 4);

    grab_offset_mtx(0, 3) = dx * P_len_;
    if (dx > 0) {
        grab_offset_mtx(0, 3) += 0.0002;
        if (shift) {
            grab_offset_mtx(0, 3) += 2 * P_len_;
        }
    }
    else if (dx < 0) {
        grab_offset_mtx(0, 3) -= 0.0002;
        if (shift) {
            grab_offset_mtx(0, 3) -= 2 * P_len_;
        }
    }
    grab_offset_mtx(1, 3) = dy * P_len_;
    if (dy > 0) {
        grab_offset_mtx(1, 3) += 0.0002;
        if (shift) {
            grab_offset_mtx(1, 3) += 2 * P_len_;
        }
    }
    else if (dy < 0) {
        grab_offset_mtx(1, 3) -= 0.0002;
        if (shift) {
            grab_offset_mtx(1, 3) -= 2 * P_len_;
        }
    }
    grab_offset_mtx(2, 3) = -dz * brick_height_m_ - (P_len_ - brick_len_offset_);

    sup_pose_mtx = grab_offset_mtx * sup_pose_mtx;

    T = sup_pose_mtx;
}


std::string Lego::get_brick_name_by_id(const int& id, const int& seq_id)
{
    std::string brick_name = "b" + std::to_string(id) + "_" + std::to_string(seq_id);
    if(brick_map_.find(brick_name) == brick_map_.end())
    {
        ROS_INFO_STREAM("No available brick! ID: " << id << ", Seq ID: " << seq_id);
    }
    return brick_name;
}


void Lego::get_brick_corners(const lego_brick& b1, double& lx, double& ly, double& rx, double& ry)
{
    // Landscape orientation
    if((math::ApproxEqNum(b1.cur_quat.x(), 0,EPS_*10000) && math::ApproxEqNum(b1.cur_quat.y(), 0,EPS_*10000) && math::ApproxEqNum(b1.cur_quat.z(), 0,EPS_*10000) && math::ApproxEqNum(b1.cur_quat.w(), 1,EPS_*10000)))
    {
        lx = b1.cur_x - (b1.height * P_len_ - brick_len_offset_) / 2.0;
        ly = b1.cur_y - (b1.width * P_len_ - brick_len_offset_) / 2.0;
        rx = b1.cur_x + (b1.height * P_len_ - brick_len_offset_) / 2.0;
        ry = b1.cur_y + (b1.width * P_len_ - brick_len_offset_) / 2.0;
    }
    // Vertcal orientation
    else
    {
        lx = b1.cur_x - (b1.width * P_len_ - brick_len_offset_) / 2.0;
        ly = b1.cur_y - (b1.height * P_len_ - brick_len_offset_) / 2.0;
        rx = b1.cur_x + (b1.width * P_len_ - brick_len_offset_) / 2.0;
        ry = b1.cur_y + (b1.height * P_len_ - brick_len_offset_) / 2.0;   
    }
}


bool Lego::bricks_overlap(const lego_brick& b1, const lego_brick& b2)
{
    double l1x, l1y, r1x, r1y, l2x, l2y, r2x, r2y;
    get_brick_corners(b1, l1x, l1y, r1x, r1y);
    get_brick_corners(b2, l2x, l2y, r2x, r2y);

    if(math::ApproxEqNum(l1x, r1x, EPS_) || 
       math::ApproxEqNum(l1y, r1y, EPS_) || 
       math::ApproxEqNum(l2x, r2x, EPS_) || 
       math::ApproxEqNum(l2y, r2y, EPS_))
    {
        return false;
    }
    if(l1x >= r2x || l2x >= r1x)
    {
        return false;
    }
    if(l2y >= r1y || l1y >= r2y)
    {
        return false;
    }
    return true;
}

bool Lego::is_top_connect(const lego_brick& b1, const lego_brick& b2)
{
    double b1x = b1.cur_x;
    double b1y = b1.cur_y;
    double b1z = b1.cur_z;
    double b2x = b2.cur_x;
    double b2y = b2.cur_y;
    double b2z = b2.cur_z;
    if(b2z - b1z < 0.1 * brick_height_m_ || (b2z - b1z) > 1.1 * brick_height_m_)
    {
        return false;
    }
    
    if(!bricks_overlap(b1, b2))
    {
        return false;
    }
    return true;
}

bool Lego::is_bottom_connect(const lego_brick& b1, const lego_brick& b2)
{
    double b1x = b1.cur_x;
    double b1y = b1.cur_y;
    double b1z = b1.cur_z;
    double b2x = b2.cur_x;
    double b2y = b2.cur_y;
    double b2z = b2.cur_z;
    if(b1z - b2z < 0.1 * brick_height_m_ || (b1z - b2z) > 1.1 * brick_height_m_)
    {
        return false;
    }
    
    if(!bricks_overlap(b1, b2))
    {
        return false;
    }
    return true;
}


void Lego::update_brick_connection()
{
    auto start = high_resolution_clock::now();
    for(auto b1:brick_map_)
    {
        brick_map_[b1.second.brick_name].top_connect.clear();
        brick_map_[b1.second.brick_name].bottom_connect.clear();
        b1.second.top_connect.clear();
        b1.second.bottom_connect.clear();
        for(auto b2:brick_map_)
        {
            if(b1.second.brick_name.compare(b2.second.brick_name) == 0)
            {
                continue;
            }
            if(is_top_connect(b1.second, b2.second))
            {
                brick_map_[b1.second.brick_name].top_connect[b2.second.brick_name] = b2.second.brick_name;
            }
            if(is_bottom_connect(b1.second, b2.second))
            {
                brick_map_[b1.second.brick_name].bottom_connect[b2.second.brick_name] = b2.second.brick_name;
            }
        }
    }
    
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    ROS_INFO_STREAM("\nUpdate brick connection time: " << duration.count() / 1000000.0 << " s");
}


void Lego::update_all_top_bricks(const std::string& brick_name, const Eigen::Matrix4d& dT)
{
    for(auto top_brick_n:brick_map_[brick_name].top_connect)
    {
        std::string top_brick_name = top_brick_n.second;
        lego_brick top_brick = brick_map_[top_brick_name];

        Eigen::Matrix4d cur_T = Eigen::Matrix4d::Identity(4, 4);
        cur_T.col(3) << top_brick.cur_x, top_brick.cur_y, top_brick.cur_z, 1;
        cur_T.block(0, 0, 3, 3) << top_brick.cur_quat.normalized().toRotationMatrix();
        Eigen::Matrix4d new_T = dT * cur_T;
        Eigen::Matrix3d new_rot = new_T.block(0, 0, 3, 3);
        Eigen::Quaterniond new_quat(new_rot);

        brick_map_[top_brick_name].cur_x = new_T.coeff(0, 3);
        brick_map_[top_brick_name].cur_y = new_T.coeff(1, 3);
        brick_map_[top_brick_name].cur_z = new_T.coeff(2, 3);
        brick_map_[top_brick_name].cur_quat = new_quat;
        // Update brick in stock status
        brick_map_[top_brick_name].in_stock = (math::ApproxEqNum(brick_map_[top_brick_name].x, brick_map_[top_brick_name].cur_x, EPS_ * 100) && 
                                               math::ApproxEqNum(brick_map_[top_brick_name].y, brick_map_[top_brick_name].cur_y, EPS_ * 100) && 
                                               math::ApproxEqNum(brick_map_[top_brick_name].z, brick_map_[top_brick_name].cur_z, EPS_ * 100)); // scale up eps_

        gazebo_msgs::ModelState new_pose;
        new_pose.model_name = top_brick_name;
        new_pose.pose.position.x = brick_map_[top_brick_name].cur_x;
        new_pose.pose.position.y = brick_map_[top_brick_name].cur_y;
        new_pose.pose.position.z = brick_map_[top_brick_name].cur_z;
        new_pose.pose.orientation.x = brick_map_[top_brick_name].cur_quat.x();
        new_pose.pose.orientation.y = brick_map_[top_brick_name].cur_quat.y();
        new_pose.pose.orientation.z = brick_map_[top_brick_name].cur_quat.z();
        new_pose.pose.orientation.w = brick_map_[top_brick_name].cur_quat.w();

        setmodelstate_.request.model_state = new_pose;
        client_.call(setmodelstate_);
        update_all_top_bricks(top_brick_name, dT);
    }

}

void Lego::update(const std::string& brick_name, const Eigen::Matrix4d& T_init)
{
    lego_brick cur_brick = brick_map_[brick_name];
    int brick_height = cur_brick.height;
    int brick_width = cur_brick.width;
    int press_offset = cur_brick.press_offset;
    
    Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d dT, new_brick_T;
    Eigen::Matrix4d cur_brick_T = Eigen::Matrix4d::Identity(4, 4);
    cur_brick_T.col(3) << cur_brick.cur_x, cur_brick.cur_y, cur_brick.cur_z, 1;
    cur_brick_T.block(0, 0, 3, 3) << cur_brick.cur_quat.normalized().toRotationMatrix();

    Eigen::Matrix4d y_180, z_90, z_180;
    z_90 << 0, -1, 0, 0, 
            1, 0, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    y_180 << -1, 0, 0, 0,
              0, 1, 0, 0, 
              0, 0, -1, 0, 
              0, 0, 0, 1;
    z_180 << -1, 0, 0, 0,
              0, -1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
    new_brick_T = T_init * y_180 * z_180;
    int center_press_offset = 0;
    if(cur_brick.press_side == 1)
    {
        center_press_offset = (brick_width / 2) - 1;
        if(brick_width % 2 == 0)
        {
            tmp.col(3) << (brick_height * P_len_ - brick_len_offset_) / 2.0, (center_press_offset - press_offset) * P_len_, 0, 1;
        }
        else
        {
            tmp.col(3) << (brick_height * P_len_ - brick_len_offset_) / 2.0, -(P_len_ - brick_len_offset_) / 2.0, 0, 1;
        }
        new_brick_T = new_brick_T * tmp;
        
    }
    if(cur_brick.press_side == 4)
    {
        center_press_offset = (brick_width / 2) - 1;
        if(brick_width % 2 == 0)
        {
            tmp.col(3) << -(brick_height * P_len_ - brick_len_offset_) / 2.0, (center_press_offset - press_offset) * P_len_, 0, 1;
        }
        else
        {
            tmp.col(3) << -(brick_height * P_len_ - brick_len_offset_) / 2.0, (P_len_ - brick_len_offset_) / 2.0, 0, 1;
        }
        new_brick_T = new_brick_T * z_180 * tmp;
    }
    else if(cur_brick.press_side == 2)
    {
        center_press_offset = (brick_height / 2) - 1;
        if(brick_height % 2 == 0)
        {
            tmp.col(3) << (press_offset - center_press_offset) * P_len_, -(brick_width * P_len_ - brick_len_offset_) / 2.0, 0, 1;
        }
        else
        {
            tmp.col(3) << -(P_len_ - brick_len_offset_) / 2.0, -(brick_width * P_len_ - brick_len_offset_) / 2.0,  0, 1;
        }
        new_brick_T = new_brick_T * z_90 * tmp ;
    }
    else if(cur_brick.press_side == 3)
    {
        
        if(brick_height % 2 == 0)
        {
            tmp.col(3) << (press_offset - center_press_offset) * P_len_, (brick_width * P_len_ - brick_len_offset_) / 2.0, 0, 1;
        }
        else
        {
            tmp.col(3) << (P_len_ - brick_len_offset_) / 2.0, (brick_width * P_len_ - brick_len_offset_) / 2.0,  0, 1;
        }
        new_brick_T = new_brick_T * z_90 * z_180 * tmp ;
    }
    dT = new_brick_T * math::PInv(cur_brick_T);
    
    update_all_top_bricks(brick_name, dT);
    Eigen::Matrix3d rot_mtx = new_brick_T.block(0, 0, 3, 3);
    Eigen::Quaterniond new_quat(rot_mtx);
    brick_map_[brick_name].cur_x = new_brick_T.coeff(0, 3);
    brick_map_[brick_name].cur_y = new_brick_T.coeff(1, 3);
    brick_map_[brick_name].cur_z = new_brick_T.coeff(2, 3);
    brick_map_[brick_name].cur_quat =  new_quat;
    
    // Update brick in stock status
    brick_map_[brick_name].in_stock = (math::ApproxEqNum(brick_map_[brick_name].x, brick_map_[brick_name].cur_x, EPS_ * 100) && 
                                       math::ApproxEqNum(brick_map_[brick_name].y, brick_map_[brick_name].cur_y, EPS_ * 100) && 
                                       math::ApproxEqNum(brick_map_[brick_name].z, brick_map_[brick_name].cur_z, EPS_ * 100));

    gazebo_msgs::ModelState new_pose;
    new_pose.model_name = brick_name;
    new_pose.pose.position.x = brick_map_[brick_name].cur_x;
    new_pose.pose.position.y = brick_map_[brick_name].cur_y;
    new_pose.pose.position.z = brick_map_[brick_name].cur_z;
    new_pose.pose.orientation.x = brick_map_[brick_name].cur_quat.x();
    new_pose.pose.orientation.y = brick_map_[brick_name].cur_quat.y();
    new_pose.pose.orientation.z = brick_map_[brick_name].cur_quat.z();
    new_pose.pose.orientation.w = brick_map_[brick_name].cur_quat.w();
    
    setmodelstate_.request.model_state = new_pose;
    client_.call(setmodelstate_);
}

void Lego::update_bricks(const math::VectorJd& robot_q, const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, 
                                const bool& joint_rad, const std::string& brick_name, const int& mode)
{
    Eigen::Matrix4d T = math::FK(robot_q, DH, base_frame, joint_rad);
    if(mode == 1)
    {
        Eigen::Matrix4d y_p90, z_180;
        y_p90 << 0, 0, 1, 0,
                0, 1, 0, 0,
                -1, 0, 0, 0,
                0, 0, 0, 1;
        z_180 << -1, 0, 0, 0,
                 0, -1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
        T = T * y_p90 * z_180;
    }
    std::string closest_brick_name = brick_name;
    update(brick_name, T);
}


bool Lego::robot_is_static(math::VectorJd robot_qd, math::VectorJd robot_qdd, const int& robot_dof)
{
    for(int i=0; i<robot_dof; i++)
    {
        if(abs(robot_qd(i)) > EPS_ || abs(robot_qdd(i)) > EPS_)
        {
            return false;
        }
    }
    return true;
}


bool Lego::robot_reached_goal(math::VectorJd robot_q, math::VectorJd goal, const int& robot_dof)
{
    for(int i=0; i<robot_dof; i++)
    {
        if(abs(robot_q(i) - goal(i)) > EPS_)
        {
            return false;
        }
    }
    return true;
}


}
}
