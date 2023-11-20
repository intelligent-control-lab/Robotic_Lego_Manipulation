#include "Lego.hpp"

namespace lego_manipulation
{
namespace lego
{
Lego::Lego()
{
}
        
void Lego::setup(const std::string& env_setup_fname, const bool& assemble, const Json::Value& task_json, 
                 const std::string& DH_fname, const std::string& DH_tool_fname, const std::string& DH_tool_disassemble_fname, const std::string& DH_tool_assemble_fname, 
                 const std::string& base_fname, const ros::ServiceClient& cli)
{
    client_ = cli;

    gazebo_msgs::ModelState brick_pose;
    std::ifstream config_file(env_setup_fname, std::ifstream::binary);
    Json::Value config;
    double x, y, z;
    Eigen::Quaterniond quat(Eigen::Matrix3d::Identity(3, 3));
    brick_pose.pose.orientation.x = quat.x();
    brick_pose.pose.orientation.y = quat.y();
    brick_pose.pose.orientation.z = quat.z();
    brick_pose.pose.orientation.w = quat.w();

    set_robot_base(base_fname);
    set_DH(DH_fname);
    set_DH_tool(DH_tool_fname);
    set_DH_tool_assemble(DH_tool_assemble_fname);
    set_DH_tool_disassemble(DH_tool_disassemble_fname);
    print_manipulation_property();
    
    config_file >> config;
    storage_plate_x_ = config["storage_plate"]["x"].asDouble();
    storage_plate_y_ = config["storage_plate"]["y"].asDouble();
    storage_plate_z_ = config["storage_plate"]["z"].asDouble();
    storage_plate_width_ = config["storage_plate"]["width"].asInt();
    storage_plate_height_ = config["storage_plate"]["height"].asInt();
    assemble_plate_x_ = config["assemble_plate"]["x"].asDouble();
    assemble_plate_y_ = config["assemble_plate"]["y"].asDouble();
    assemble_plate_z_ = config["assemble_plate"]["z"].asDouble();
    assemble_plate_width_ = config["assemble_plate"]["width"].asInt();
    assemble_plate_height_ = config["assemble_plate"]["height"].asInt();

    for(auto brick = config.begin(); brick != config.end(); brick++)
    {
        brick_pose.model_name = brick.name();
        
        if(brick.name().compare("storage_plate") == 0)
        {
            storage_plate_x_ = (*brick)["x"].asDouble();
            storage_plate_y_ = (*brick)["y"].asDouble();
            storage_plate_z_ = (*brick)["z"].asDouble();
            storage_plate_width_ = (*brick)["width"].asInt();
            storage_plate_height_ = (*brick)["height"].asInt();
            x = storage_plate_x_;
            y = storage_plate_y_;
            z = storage_plate_z_;
        }
        else if(brick.name().compare("assemble_plate") == 0)
        {
            assemble_plate_x_ = (*brick)["x"].asDouble();
            assemble_plate_y_ = (*brick)["y"].asDouble();
            assemble_plate_z_ = (*brick)["z"].asDouble();
            assemble_plate_width_ = (*brick)["width"].asInt();
            assemble_plate_height_ = (*brick)["height"].asInt();
            x = assemble_plate_x_;
            y = assemble_plate_y_;
            z = assemble_plate_z_;
        }
        else if(brick.name()[0] == 'b')
        {
            calc_brick_loc(brick.name(), storage_plate_x_, storage_plate_y_, 0, storage_plate_width_, storage_plate_height_, 
                           (*brick)["x"].asInt(), (*brick)["y"].asInt(), x, y);
            z = storage_plate_z_ + (*brick)["z"].asInt() * brick_height_m_;
            lego_brick l_brick;
            l_brick.brick_name = brick.name();
            l_brick.height = std::stoi(brick.name().substr(1, 1));
            l_brick.width = std::stoi(brick.name().substr(2, 1));
            l_brick.x = x;
            l_brick.y = y;
            l_brick.z = z;
            l_brick.cur_x = x;
            l_brick.cur_y = y;
            l_brick.cur_z = z;
            l_brick.in_stock = true;
            Eigen::Matrix3d rot_mtx = Eigen::Matrix3d::Identity(3, 3);
            Eigen::Quaterniond quat(rot_mtx);
            l_brick.quat_x = quat.x();
            l_brick.quat_y = quat.y();
            l_brick.quat_z = quat.z();
            l_brick.quat_w = quat.w();
            l_brick.cur_quat = quat;
            brick_map_[brick.name()] = l_brick;
        }
        else
        {
            ROS_INFO_STREAM("Unknown brick type!");
            continue;
        }
        brick_pose.pose.position.x = x;
        brick_pose.pose.position.y = y;
        brick_pose.pose.position.z = z;
        setmodelstate_.request.model_state = brick_pose;
        client_.call(setmodelstate_);
    }
    if(!assemble)
    {
        std::string brick_name;
        double x, y, brick_z_m;
        Eigen::Matrix3d z_90;
        z_90 << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        Eigen::Matrix3d rot_mtx = Eigen::Matrix3d::Identity(3, 3);
        Eigen::Matrix3d rot_mtx1 = rot_mtx * z_90;
        Eigen::Quaterniond quat;
        for(int i=1; i<=task_json.size(); i++)
        {
            auto cur_graph_node = task_json[std::to_string(i)];
            brick_name = get_brick_name_by_id(cur_graph_node["brick_id"].asInt(), cur_graph_node["brick_seq"].asInt());
            calc_brick_loc(brick_name, assemble_plate_x_, assemble_plate_y_, cur_graph_node["ori"].asInt(), assemble_plate_width_, assemble_plate_height_, 
                           cur_graph_node["x"].asInt(), cur_graph_node["y"].asInt(), x, y);
            brick_z_m = assemble_plate_z_ + cur_graph_node["z"].asInt() * brick_height_m_;

            brick_map_[brick_name].in_stock = assemble;
            if(cur_graph_node["ori"].asInt() == 1)
            {
                quat = rot_mtx1;
            }
            else
            {
                quat = rot_mtx;
            }
            brick_map_[brick_name].cur_x = x;
            brick_map_[brick_name].cur_y = y;
            brick_map_[brick_name].cur_z = brick_z_m;
            brick_map_[brick_name].cur_quat = quat;

            brick_pose.model_name = brick_name;
            brick_pose.pose.position.x = x;
            brick_pose.pose.position.y = y;
            brick_pose.pose.position.z = brick_z_m;
            brick_pose.pose.orientation.x = quat.x();
            brick_pose.pose.orientation.y = quat.y();
            brick_pose.pose.orientation.z = quat.z();
            brick_pose.pose.orientation.w = quat.w();
            setmodelstate_.request.model_state = brick_pose;
            client_.call(setmodelstate_);
        }
    }
    usleep(1000 * 1000); 
}

void Lego::set_robot_base(const std::string& fname)
{
    ROS_INFO_STREAM("Load Robot Base from: " << fname);
    base_frame_ = io::LoadMatFromFile(fname);
    T_base_inv_ = Eigen::Matrix4d::Identity(4, 4);
    T_base_inv_.col(3) << base_frame_, 1;
    T_base_inv_ = math::PInv(T_base_inv_);
}

void Lego::print_manipulation_property()
{
    ROS_INFO_STREAM("\nRobot Base: \n" << base_frame_);
    ROS_INFO_STREAM("\nRobot DH: \n" << DH_);
    ROS_INFO_STREAM("\nRobot Tool DH: \n" << DH_tool_);
    ROS_INFO_STREAM("\nRobot Tool Disassemble DH: \n" << DH_tool_disassemble_);
    ROS_INFO_STREAM("\nRobot Tool Assemble DH: \n" << DH_tool_assemble_);
    std::cout << "\n" << std::endl;
}

void Lego::set_DH(const std::string& fname)
{
    ROS_INFO_STREAM("Load Robot DH from: " << fname);
    DH_ = io::LoadMatFromFile(fname);
    ee_inv_ = Eigen::Matrix4d::Identity(4, 4); 
    ee_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  DH_(5, 2) * cos(0),
               sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  DH_(5, 2) * sin(0),
               0,       sin(0),         cos(0),        -DH_(5, 1),
               0,       0,              0,              1;
    ee_inv_ = math::PInv(ee_inv_);
}


void Lego::set_DH_tool(const std::string& fname)
{
    ROS_INFO_STREAM("Load DH tool from: " << fname);
    DH_tool_ = io::LoadMatFromFile(fname);
    tool_inv_ = Eigen::Matrix4d::Identity(4, 4); 
    tool_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  DH_tool_(5, 2) * cos(0),
                 sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  DH_tool_(5, 2) * sin(0),
                 0,       sin(0),         cos(0),        -DH_tool_(5, 1),
                 0,       0,              0,              1;
    tool_inv_ = math::PInv(tool_inv_);
}

void Lego::set_DH_tool_assemble(const std::string& fname)
{
    ROS_INFO_STREAM("Load DH tool for assemble from: " << fname);
    DH_tool_assemble_ = io::LoadMatFromFile(fname);
    tool_assemble_inv_ = Eigen::Matrix4d::Identity(4, 4); 
    tool_assemble_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  DH_tool_assemble_(5, 2) * cos(0),
                          sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  DH_tool_assemble_(5, 2) * sin(0),
                          0,       sin(0),         cos(0),        -DH_tool_assemble_(5, 1),
                          0,       0,              0,              1;
    tool_assemble_inv_ = math::PInv(tool_assemble_inv_);
}

void Lego::set_DH_tool_disassemble(const std::string& fname)
{
    ROS_INFO_STREAM("Load DH tool for disassemble from: " << fname);
    DH_tool_disassemble_ = io::LoadMatFromFile(fname);
    tool_disassemble_inv_ = Eigen::Matrix4d::Identity(4, 4);
    tool_disassemble_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  DH_tool_disassemble_(5, 2) * cos(0),
                             sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  DH_tool_disassemble_(5, 2) * sin(0),
                             0,       sin(0),         cos(0),        -DH_tool_disassemble_(5, 1),
                             0,       0,              0,              1;
    tool_disassemble_inv_ = math::PInv(tool_disassemble_inv_);
}


void Lego::calc_brick_loc(const std::string name, const double& ref_x, const double& ref_y, const int& orientation,
                                 const int& plate_width, const int& plate_height, const int& brick_loc_x, const int& brick_loc_y, 
                                 double& out_x, double& out_y)
{
    int brick_height = std::stoi(name.substr(1, 1));
    int brick_width = std::stoi(name.substr(2, 1));
    double brick_offset_x_m = brick_loc_x * P_len_ - brick_len_offset_;
    double brick_offset_y_m = brick_loc_y * P_len_ - brick_len_offset_;

    double brick_topleft_x_m = std::max(brick_offset_x_m, 0.0) + (ref_x - (plate_width * P_len_ - brick_len_offset_) / 2.0);
    double brick_topleft_y_m = std::max(brick_offset_y_m, 0.0) + (ref_y - (plate_height * P_len_ - brick_len_offset_) / 2.0);

    if(orientation == 0)
    {
        out_x = brick_topleft_x_m + (brick_height * P_len_ - brick_len_offset_) / 2.0;
        out_y = brick_topleft_y_m + (brick_width * P_len_ - brick_len_offset_) / 2.0;
    }
    else
    {
        out_x = brick_topleft_x_m + (brick_width * P_len_ - brick_len_offset_) / 2.0;
        out_y = brick_topleft_y_m + (brick_height * P_len_ - brick_len_offset_) / 2.0;
    }
}



void Lego::calc_brick_grab_pose(const std::string& name, const bool& assemble_pose, const bool& take_brick,
                                       const int& brick_assemble_x, const int& brick_assemble_y, const int& brick_assemble_z, 
                                       const int& orientation, const int& press_side, Eigen::MatrixXd& T)
{
    lego_brick l_brick = brick_map_[name];
    int brick_height = l_brick.height;//::stoi(name.substr(1, 1));
    int brick_width = l_brick.width;//::stoi(name.substr(2, 1));
    brick_map_[name].press_side = press_side;
    
    Eigen::Quaterniond quat;
    double x, y, z;
    Eigen::Matrix3d y_180, z_180, z_90;
    y_180 << -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0;
    z_180 << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
    z_90 << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    Eigen::Matrix3d rot_mtx = Eigen::Matrix3d::Identity(3, 3);

    // Get pose to grab the brick
    if(take_brick)
    {
        quat.x() = l_brick.cur_quat.x();
        quat.y() = l_brick.cur_quat.y();
        quat.z() = l_brick.cur_quat.z();
        quat.w() = l_brick.cur_quat.w();
        rot_mtx = rot_mtx * y_180;
        
        // Brick on storage plate
        if(l_brick.in_stock)
        {
            quat.x() = l_brick.quat_x;
            quat.y() = l_brick.quat_y;
            quat.z() = l_brick.quat_z;
            quat.w() = l_brick.quat_w;
            rot_mtx = quat.normalized().toRotationMatrix();
            if(press_side == 1 || press_side == 4)
            {
                x = l_brick.x - (brick_height * P_len_ - brick_len_offset_) / 2.0;
                if(brick_width % 2 == 0)
                {
                    y = l_brick.y;
                }
                else
                {
                    y = l_brick.y + (P_len_ - brick_len_offset_) / 2.0;
                }  
                z = l_brick.z;
                rot_mtx = rot_mtx * y_180 * z_180;
            }
            else
            {
                if(brick_height % 2 == 0)
                {
                    x = l_brick.x;
                }
                else
                {
                    x = l_brick.x + (P_len_ - brick_len_offset_) / 2.0;
                }
                y = l_brick.y + (brick_width * P_len_ - brick_len_offset_) / 2.0;
                z = l_brick.z;
                rot_mtx = rot_mtx * y_180 * z_180 * z_90;
            }
        }  
        // Landscape orientation
        else if((math::ApproxEqNum(quat.x(), 0, EPS_) && math::ApproxEqNum(quat.y(), 0, EPS_) && math::ApproxEqNum(quat.z(), 0, EPS_)) ||
                (math::ApproxEqNum(quat.x(), 0, EPS_) && math::ApproxEqNum(quat.y(), 0, EPS_) && math::ApproxEqNum(quat.z(), 1, EPS_)))
        {
            
            if(press_side == 1)
            {
                rot_mtx = rot_mtx * z_180;
                x = l_brick.cur_x - (brick_height * P_len_ - brick_len_offset_) / 2.0;
                if(brick_width % 2 == 0)
                {
                    y = l_brick.cur_y;
                }
                else
                {
                    y = l_brick.cur_y + (P_len_ - brick_len_offset_) / 2.0;
                }  
                z = l_brick.cur_z;
            }
            else if(press_side == 2)
            {
                rot_mtx = rot_mtx * z_180 * z_90;
                if(brick_height % 2 == 0)
                {
                    x = l_brick.cur_x;
                }
                else
                {
                    x = l_brick.cur_x + (P_len_ - brick_len_offset_) / 2.0;
                }
                y = l_brick.cur_y + (brick_width * P_len_ - brick_len_offset_) / 2.0;
                z = l_brick.cur_z;
            }
            else if(press_side == 3)
            {
                if(brick_height % 2 == 0)
                {
                    x = l_brick.cur_x;
                }
                else
                {
                    x = l_brick.cur_x - (P_len_ - brick_len_offset_) / 2.0;
                }
                y = l_brick.cur_y - (brick_width * P_len_ - brick_len_offset_) / 2.0;
                z = l_brick.cur_z;
                rot_mtx = rot_mtx * z_90;
            }
            else
            {
                rot_mtx = rot_mtx;
                x = l_brick.cur_x + (brick_height * P_len_ - brick_len_offset_) / 2.0;
                if(brick_width % 2 == 0)
                {
                    y = l_brick.cur_y;
                }
                else
                {
                    y = l_brick.cur_y - (P_len_ - brick_len_offset_) / 2.0;
                }  
                z = l_brick.cur_z;
            }
        }
        // Vertical orientation
        else
        {
            if(press_side == 1)
            {   
                if(brick_width % 2 == 0)
                {
                    x = l_brick.cur_x;
                }
                else
                {
                    x = l_brick.cur_x - (P_len_ - brick_len_offset_) / 2.0;
                }
                y = l_brick.cur_y - (brick_height * P_len_ - brick_len_offset_) / 2.0;
                z = l_brick.cur_z;
                rot_mtx = rot_mtx * z_90;
            }
            if(press_side == 2)
            {   
                x = l_brick.cur_x - (brick_width * P_len_ - brick_len_offset_) / 2.0;
                if(brick_height % 2 == 0)
                {
                    y = l_brick.cur_y;
                }
                else
                {
                    y = l_brick.cur_y + (P_len_ - brick_len_offset_) / 2.0;
                }  
                z = l_brick.cur_z;
                rot_mtx = rot_mtx * z_180;
            }
            if(press_side == 3)
            {   
                x = l_brick.cur_x + (brick_width * P_len_ - brick_len_offset_) / 2.0;
                if(brick_height % 2 == 0)
                {
                    y = l_brick.cur_y;
                }
                else
                {
                    y = l_brick.cur_y - (P_len_ - brick_len_offset_) / 2.0;
                }  
                z = l_brick.cur_z;
            }
            if(press_side == 4)
            {   
                if(brick_width % 2 == 0)
                {
                    x = l_brick.cur_x;
                }
                else
                {
                    x = l_brick.cur_x + (P_len_ - brick_len_offset_) / 2.0;
                }
                y = l_brick.cur_y + (brick_height * P_len_ - brick_len_offset_) / 2.0;
                z = l_brick.cur_z;
                rot_mtx = rot_mtx * z_180 * z_90;
            } 
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
            if(press_side == 1 || press_side == 4)
            {
                x = l_brick.x - (brick_height * P_len_ - brick_len_offset_) / 2.0;
                if(brick_width % 2 == 0)
                {
                    y = l_brick.y;
                }
                else
                {
                    y = l_brick.y + (P_len_ - brick_len_offset_) / 2.0;
                }  
                z = l_brick.z;
                rot_mtx = rot_mtx * y_180 * z_180;
            }
            else
            {
                if(brick_height % 2 == 0)
                {
                    x = l_brick.x;
                }
                else
                {
                    x = l_brick.x + (P_len_ - brick_len_offset_) / 2.0;
                }
                y = l_brick.y + (brick_width * P_len_ - brick_len_offset_) / 2.0;
                z = l_brick.z;
                rot_mtx = rot_mtx * y_180 * z_180 * z_90;
            }
        }  
        // Place on assemble plate
        else
        {
            calc_brick_loc(name, assemble_plate_x_, assemble_plate_y_, orientation, assemble_plate_width_, assemble_plate_height_, 
                           brick_assemble_x, brick_assemble_y, x, y);
            z = assemble_plate_z_ + brick_assemble_z * brick_height_m_;
            rot_mtx = rot_mtx * y_180;
            if(orientation == 0)
            {
                if(press_side == 1)
                {
                    x = x - (brick_height * P_len_ - brick_len_offset_) / 2.0;
                    if(brick_width % 2 == 0)
                    {
                        y = y;
                    }
                    else
                    {
                        y = y + (P_len_ - brick_len_offset_) / 2.0;
                    }  
                    rot_mtx = rot_mtx * z_180;
                }
                else if(press_side == 2)
                {
                    if(brick_height % 2 == 0)
                    {
                        x = x;
                    }
                    else
                    {
                        x = x + (P_len_ - brick_len_offset_) / 2.0;
                    }
                    y = y + (brick_width * P_len_ - brick_len_offset_) / 2.0;
                    rot_mtx = rot_mtx * z_180 * z_90;
                }
                else if(press_side == 3)
                {
                    if(brick_height % 2 == 0)
                    {
                        x = x;
                    }
                    else
                    {
                        x = x - (P_len_ - brick_len_offset_) / 2.0;
                    }
                    y = y - (brick_width * P_len_ - brick_len_offset_) / 2.0;
                    rot_mtx = rot_mtx * z_90;
                }
                else
                {
                    x = x + (brick_height * P_len_ - brick_len_offset_) / 2.0;
                    if(brick_width % 2 == 0)
                    {
                        y = y;
                    }
                    else
                    {
                        y = y - (P_len_ - brick_len_offset_) / 2.0;
                    }  
                }
            }
            else
            {
                if(press_side == 1)
                {   
                    if(brick_width % 2 == 0)
                    {
                        x = x;
                    }
                    else
                    {
                        x = x - (P_len_ - brick_len_offset_) / 2.0;
                    }
                    y = y - (brick_height * P_len_ - brick_len_offset_) / 2.0;
                    rot_mtx = rot_mtx * z_90;
                }
                if(press_side == 2)
                {   
                    x = x - (brick_width * P_len_ - brick_len_offset_) / 2.0;
                    if(brick_height % 2 == 0)
                    {
                        y = y;
                    }
                    else
                    {
                        y = y + (P_len_ - brick_len_offset_) / 2.0;
                    }  
                    rot_mtx = rot_mtx * z_180;
                }
                if(press_side == 3)
                {   
                    x = x + (brick_width * P_len_ - brick_len_offset_) / 2.0;
                    if(brick_height % 2 == 0)
                    {
                        y = y;
                    }
                    else
                    {
                        y = y - (P_len_ - brick_len_offset_) / 2.0;
                    }  
                }
                if(press_side == 4)
                {   
                    if(brick_width % 2 == 0)
                    {
                        x = x;
                    }
                    else
                    {
                        x = x + (P_len_ - brick_len_offset_) / 2.0;
                    }
                    y = y + (brick_height * P_len_ - brick_len_offset_) / 2.0;
                    rot_mtx = rot_mtx * z_180 * z_90;
                } 
            }
        }
    }
    
    // Update brick in stock status
    brick_map_[name].in_stock = (math::ApproxEqNum(l_brick.x, l_brick.cur_x,EPS_) && 
                                math::ApproxEqNum(l_brick.y, l_brick.cur_y,EPS_) && 
                                math::ApproxEqNum(l_brick.z, l_brick.cur_z,EPS_));

    T.block(0, 0, 3, 3) << rot_mtx;
    T.block(0, 3, 3, 1) << x, y, z;
}


std::string Lego::get_brick_name_by_id(const int& id, const int& seq_id)
{
    int brick_height = 0;
    int brick_width = 0;
    if(id == 2)
    {
        brick_height = 2;
        brick_width = 4;
    }
    else if(id == 3)
    {
        brick_height = 2;
        brick_width = 6;
    }
    else if(id == 4)
    {
        brick_height = 1;
        brick_width = 8;
    }
    else if(id == 5 || id == 7 || id == 8)
    {
        brick_height = 1;
        brick_width = 4;
    }
    else if(id == 6)
    {
        brick_height = 1;
        brick_width = 6;
    }
    else if(id == 9 || id == 11)
    {
        brick_height = 1;
        brick_width = 2;
    }
    else if(id == 10)
    {
        brick_height = 1;
        brick_width = 1;
    }
    else if(id == 12)
    {
        brick_height = 2;
        brick_width = 2;
    }
    std::string b = "b";
    std::string brick_name = b + std::to_string(brick_height) + std::to_string(brick_width) + "_" + std::to_string(seq_id);
    if(brick_map_.find(brick_name) == brick_map_.end())
    {
        ROS_INFO_STREAM("No available brick!");
    }
    return brick_name;
}


void Lego::get_brick_corners(const lego_brick& b1, double& lx, double& ly, double& rx, double& ry)
{
    // Landscape orientation
    if(math::ApproxEqNum(b1.cur_quat.x(), 0,EPS_) && math::ApproxEqNum(b1.cur_quat.y(), 0,EPS_) && math::ApproxEqNum(b1.cur_quat.z(), 0,EPS_) ||
       math::ApproxEqNum(b1.cur_quat.x(), 0,EPS_) && math::ApproxEqNum(b1.cur_quat.y(), 0,EPS_) && math::ApproxEqNum(b1.cur_quat.z(), 1,EPS_))
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
    int brick_height = cur_brick.height;//::stoi(brick_name.substr(1, 1));
    int brick_width = cur_brick.width;//::stoi(brick_name.substr(2, 1));
    
    Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d dT, new_brick_T;
    Eigen::Matrix4d cur_brick_T = Eigen::Matrix4d::Identity(4, 4);
    cur_brick_T.col(3) << cur_brick.cur_x, cur_brick.cur_y, cur_brick.cur_z, 1;
    cur_brick_T.block(0, 0, 3, 3) << cur_brick.cur_quat.normalized().toRotationMatrix();

    Eigen::Matrix3d y_180, z_90;
    z_90 << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    y_180 << -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0;
    Eigen::Matrix3d rot_mtx = T_init.block(0, 0, 3, 3) * y_180;
    if(cur_brick.press_side == 1 || cur_brick.press_side == 4)
    {
        if(brick_width % 2 == 0)
        {
            tmp.col(3) << (brick_height * P_len_ - brick_len_offset_) / 2.0, 0, 0, 1;
        }
        else
        {
            tmp.col(3) << (brick_height * P_len_ - brick_len_offset_) / 2.0, (P_len_ - brick_len_offset_) / 2.0, 0, 1;
        }
    }
    else
    {
        rot_mtx = rot_mtx * z_90;
        if(brick_height % 2 == 0)
        {
            tmp.col(3) << (brick_width * P_len_ - brick_len_offset_) / 2.0, 0, 0, 1;
        }
        else
        {
            tmp.col(3) << (brick_width * P_len_ - brick_len_offset_) / 2.0, (P_len_ - brick_len_offset_) / 2.0, 0, 1;
        }
    }
    new_brick_T = T_init * tmp;
    new_brick_T.block(0, 0, 3, 3) << rot_mtx;
    dT = new_brick_T * math::PInv(cur_brick_T);

    update_all_top_bricks(brick_name, dT);
    rot_mtx = new_brick_T.block(0, 0, 3, 3);
    Eigen::Quaterniond new_quat(rot_mtx);
    brick_map_[brick_name].cur_x = new_brick_T.coeff(0, 3);
    brick_map_[brick_name].cur_y = new_brick_T.coeff(1, 3);
    brick_map_[brick_name].cur_z = new_brick_T.coeff(2, 3);
    brick_map_[brick_name].cur_quat =  new_quat;

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
                                const bool& joint_rad, const int& task_mode, const std::string& brick_name)
{
    Eigen::Matrix4d T = math::FK(robot_q, DH, base_frame, joint_rad);
    Eigen::Matrix4d tmp;
    std::string closest_brick_name = brick_name;
    update(brick_name, T);
}


bool Lego::robot_is_static(math::VectorJd robot_qd, math::VectorJd robot_qdd)
{
    for(int i=0; i<robot_dof_; i++)
    {
        if(abs(robot_qd(i)) > EPS_ || abs(robot_qdd(i)) > EPS_)
        {
            return false;
        }
    }
    return true;
}


bool Lego::robot_reached_goal(math::VectorJd robot_q, math::VectorJd goal)
{
    for(int i=0; i<robot_dof_; i++)
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
