#pragma once
#include "Utils/Math.hpp"
#include "Utils/FileIO.hpp"
#include "gazebo_msgs/SetModelState.h"

namespace lego_manipulation
{
namespace lego
{
struct lego_brick{
    std::string brick_name;
    int height;
    int width;
    double x;
    double y;
    double z;
    double quat_x;
    double quat_y;
    double quat_z;
    double quat_w;
    double cur_x;
    double cur_y;
    double cur_z;
    int press_side;
    Eigen::Quaterniond cur_quat;
    bool in_stock;
    std::map<std::string, std::string> top_connect;
    std::map<std::string, std::string> bottom_connect;
};

struct lego_plate{
    int height;
    int width;
    Eigen::Matrix4d pose;
};

class Lego
{
    /* -------------------------------------------------------------------------- */
    /*                                   pointer                                  */
    /* -------------------------------------------------------------------------- */
    public:
        typedef std::shared_ptr<Lego> Ptr;
        typedef std::shared_ptr<Lego const> ConstPtr;

    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:
        std::map<std::string, lego_brick> brick_map_;
        ros::ServiceClient client_; 
        gazebo_msgs::SetModelState setmodelstate_;
        double brick_height_m_ = 0.0096;
        double brick_len_offset_ = 0.0002;
        double P_len_ = 0.008;
        double EPS_ = 0.00001;
        double knob_height_ = 0.0017;
        lego_plate assemble_plate_;
        lego_plate storage_plate_;
        int r1_robot_dof_ = 6;
        int r2_robot_dof_ = 6;
        Eigen::Matrix4d world_base_frame_;
        Eigen::Matrix4d world_T_base_inv_;

        Eigen::MatrixXd r1_DH_; // size = [n_joint + n_ee, 4]
        Eigen::Matrix4d r1_ee_inv_, r1_tool_inv_, r1_tool_assemble_inv_, r1_tool_disassemble_inv_;
        Eigen::MatrixXd r1_DH_tool_;
        Eigen::MatrixXd r1_DH_tool_assemble_;
        Eigen::MatrixXd r1_DH_tool_disassemble_;
        Eigen::MatrixXd r1_base_frame_;
        Eigen::Matrix4d r1_T_base_inv_;

        Eigen::MatrixXd r2_DH_; // size = [n_joint + n_ee, 4]
        Eigen::Matrix4d r2_ee_inv_, r2_tool_inv_, r2_tool_assemble_inv_, r2_tool_disassemble_inv_;
        Eigen::MatrixXd r2_DH_tool_;
        Eigen::MatrixXd r2_DH_tool_assemble_;
        Eigen::MatrixXd r2_DH_tool_disassemble_;
        Eigen::MatrixXd r2_base_frame_;
        Eigen::Matrix4d r2_T_base_inv_;

        void update_all_top_bricks(const std::string& brick_name, const Eigen::Matrix4d& dT);
        void update(const std::string& brick_name, const Eigen::Matrix4d& T);
        void calc_brick_loc(const lego_brick& brick, const lego_plate& plate, const int& orientation,
                            const int& brick_loc_x, const int& brick_loc_y, const int& brick_loc_z, 
                            Eigen::Matrix4d& out_pose);
        bool is_top_connect(const lego_brick& b1, const lego_brick& b2);
        bool is_bottom_connect(const lego_brick& b1, const lego_brick& b2);
        bool bricks_overlap(const lego_brick& b1, const lego_brick& b2);
        void get_brick_corners(const lego_brick& b1, double& lx, double& ly, double& rx, double& ry);
        void brick_dimension_from_name(const std::string& b_name, int& height, int& width, const Json::Value& lego_lib);


    public:
        Lego();
        ~Lego(){}
                
        // Operations
        void setup(const std::string& env_setup_fname, const std::string& lego_lib_fname, const bool& assemble, const Json::Value& task_json, const std::string& world_base_fname,
                   const std::string& r1_DH_fname, const std::string& r1_DH_tool_fname, const std::string& r1_DH_tool_disassemble_fname, 
                   const std::string& r1_DH_tool_assemble_fname, const std::string& r1_base_fname, 
                   const std::string& r2_DH_fname, const std::string& r2_DH_tool_fname, const std::string& r2_DH_tool_disassemble_fname, 
                   const std::string& r2_DH_tool_assemble_fname, const std::string& r2_base_fname,const bool& use_config_file, const ros::ServiceClient& cli);
        void set_robot_base(const std::string& r1_base_fname, const std::string& r2_base_fname);
        void set_DH(const std::string& r1_DH_fname, const std::string& r2_DH_fname);
        void set_DH_tool(const std::string& r1_DH_tool_fname, const std::string& r2_DH_tool_fname);
        void set_DH_tool_assemble(const std::string& r1_DH_tool_assemble_fname, const std::string& r2_DH_tool_assemble_fname);
        void set_DH_tool_disassemble(const std::string& r1_DH_tool_disassemble_fname, const std::string& r2_DH_tool_disassemble_fname);
        void print_manipulation_property();
        void set_assemble_plate_pose(const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw);
        void set_storage_plate_pose(const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw);
        void set_world_base(const std::string& world_base_fname);

        void update_bricks(const math::VectorJd& robot_q, const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, 
                           const bool& joint_rad, const std::string& brick_name);
        std::string get_brick_name_by_id(const int& id, const int& seq_id);
        void update_brick_connection();
        void calc_brick_grab_pose(const std::string& name, const bool& assemble_pose, const bool& take_brick,
                                  const int& brick_assemble_x, const int& brick_assemble_y, const int& brick_assemble_z, 
                                  const int& orientation, const int& press_side, Eigen::MatrixXd& T);
        void calc_brick_sup_pose(const std::string&name, const Eigen::MatrixXd& cart_T, const int& press_side, const bool& offset, Eigen::MatrixXd &T);
        int brick_instock(const std::string& name) {return brick_map_[name].in_stock;};
        int robot_dof_1() {return r1_robot_dof_;};
        int robot_dof_2() {return r2_robot_dof_;};
        Eigen::MatrixXd world_base_frame() {return world_base_frame_;};
        Eigen::MatrixXd world_base_inv() {return world_T_base_inv_;};
        Eigen::MatrixXd robot_DH_r1() {return r1_DH_;};
        Eigen::MatrixXd robot_DH_tool_r1() {return r1_DH_tool_;};
        Eigen::MatrixXd robot_DH_tool_assemble_r1() {return r1_DH_tool_assemble_;};
        Eigen::MatrixXd robot_DH_tool_disassemble_r1() {return r1_DH_tool_disassemble_;};
        Eigen::MatrixXd robot_base_r1() {return r1_base_frame_;};
        Eigen::Matrix4d robot_base_inv_r1() {return r1_T_base_inv_;};
        Eigen::Matrix4d robot_ee_inv_r1() {return r1_ee_inv_;};
        Eigen::Matrix4d robot_tool_inv_r1() {return r1_tool_inv_;};
        Eigen::Matrix4d robot_tool_assemble_inv_r1() {return r1_tool_assemble_inv_;};
        Eigen::Matrix4d robot_tool_disassemble_inv_r1() {return r1_tool_disassemble_inv_;};
        Eigen::MatrixXd robot_DH_r2() {return r2_DH_;};
        Eigen::MatrixXd robot_DH_tool_r2() {return r2_DH_tool_;};
        Eigen::MatrixXd robot_DH_tool_assemble_r2() {return r2_DH_tool_assemble_;};
        Eigen::MatrixXd robot_DH_tool_disassemble_r2() {return r2_DH_tool_disassemble_;};
        Eigen::MatrixXd robot_base_r2() {return r2_base_frame_;};
        Eigen::Matrix4d robot_base_inv_r2() {return r2_T_base_inv_;};
        Eigen::Matrix4d robot_ee_inv_r2() {return r2_ee_inv_;};
        Eigen::Matrix4d robot_tool_inv_r2() {return r2_tool_inv_;};
        Eigen::Matrix4d robot_tool_assemble_inv_r2() {return r2_tool_assemble_inv_;};
        Eigen::Matrix4d robot_tool_disassemble_inv_r2() {return r2_tool_disassemble_inv_;};

        bool robot_is_static(math::VectorJd robot_qd, math::VectorJd robot_qdd, const int& robot_dof);
        bool robot_reached_goal(math::VectorJd robot_q, math::VectorJd goal, const int& robot_dof);
        Eigen::Matrix4d assemble_plate_pose() {return assemble_plate_.pose;};
        Eigen::Matrix4d storage_plate_pose() {return storage_plate_.pose;};
        double brick_height() {return brick_height_m_;};
};
}
}