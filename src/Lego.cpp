#include "Lego.hpp"

namespace lego_manipulation
{
namespace lego
{
Lego::Lego()
{
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
    if(cur_brick.press_side == 1)
    {
        if(brick_width % 2 == 0)
        {
            tmp.col(3) << (brick_height * P_len_ - brick_len_offset_) / 2.0, 0, 0, 1;
        }
        else
        {
            tmp.col(3) << (brick_height * P_len_ - brick_len_offset_) / 2.0, -(P_len_ - brick_len_offset_) / 2.0, 0, 1;
        }
        new_brick_T = new_brick_T * tmp;
        
    }
    if(cur_brick.press_side == 4)
    {
        if(brick_width % 2 == 0)
        {
            tmp.col(3) << -(brick_height * P_len_ - brick_len_offset_) / 2.0, 0, 0, 1;
        }
        else
        {
            tmp.col(3) << -(brick_height * P_len_ - brick_len_offset_) / 2.0, (P_len_ - brick_len_offset_) / 2.0, 0, 1;
        }
        new_brick_T = new_brick_T * z_180 * tmp;
    }
    else if(cur_brick.press_side == 2)
    {
        
        if(brick_height % 2 == 0)
        {
            tmp.col(3) << 0, -(brick_width * P_len_ - brick_len_offset_) / 2.0, 0, 1;
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
            tmp.col(3) << 0, (brick_width * P_len_ - brick_len_offset_) / 2.0, 0, 1;
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
                                const bool& joint_rad, const std::string& brick_name)
{
    Eigen::Matrix4d T = math::FK(robot_q, DH, base_frame, joint_rad);
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
