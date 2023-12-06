#pragma once
#include "Utils/Math.hpp"
#include "Utils/FileIO.hpp"

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
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    Eigen::Quaterniond quat;
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
        double storage_plate_x_ = 0.0;
        double storage_plate_y_ = 0.0;
        double storage_plate_z_ = 0.0;
        int storage_plate_width_ = 0;
        int storage_plate_height_ = 0;
        double assemble_plate_x_ = 0.0;
        double assemble_plate_y_ = 0.0;
        double assemble_plate_z_ = 0.0;
        lego_plate assemble_plate_;
        lego_plate storage_plate_;
        int assemble_plate_width_ = 0;
        int assemble_plate_height_ = 0;
        int robot_dof_ = 6;

        Eigen::MatrixXd DH_; // size = [n_joint + n_ee, 4]
        Eigen::Matrix4d ee_inv_, tool_inv_, tool_assemble_inv_, tool_disassemble_inv_;
        Eigen::MatrixXd DH_tool_;
        Eigen::MatrixXd DH_tool_assemble_;
        Eigen::MatrixXd DH_tool_disassemble_;
        Eigen::MatrixXd base_frame_;
        Eigen::Matrix4d T_base_inv_;

        void update_all_top_bricks(const std::string& brick_name, const Eigen::Matrix4d& dT);
        void update(const std::string& brick_name, const Eigen::Matrix4d& T);
        void calc_brick_loc(const std::string name, const lego_plate& plate, const int& orientation,
                            const int& brick_loc_x, const int& brick_loc_y, const int& brick_loc_z, 
                            Eigen::Matrix4d& out_pose);
        bool is_top_connect(const lego_brick& b1, const lego_brick& b2);
        bool is_bottom_connect(const lego_brick& b1, const lego_brick& b2);
        bool bricks_overlap(const lego_brick& b1, const lego_brick& b2);
        void get_brick_corners(const lego_brick& b1, double& lx, double& ly, double& rx, double& ry);


    public:
        Lego();
        ~Lego(){}
                
        // Operations
        void setup(const std::string& env_setup_fname, const bool& assemble, const Json::Value& task_json, 
                   const std::string& DH_fname, const std::string& DH_tool_fname, const std::string& DH_tool_disassemble_fname, const std::string& DH_tool_assemble_fname, 
                   const std::string& base_fname, const ros::ServiceClient& cli);
        void set_robot_base(const std::string& base_fname);
        void set_DH(const std::string& DH_fname);
        void set_DH_tool(const std::string& DH_tool_fname);
        void set_DH_tool_assemble(const std::string& DH_tool_assemble_fname);
        void set_DH_tool_disassemble(const std::string& DH_tool_disassemble_fname);
        void print_manipulation_property();

        void update_bricks(const math::VectorJd& robot_q, const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, 
                           const bool& joint_rad, const int& task_mode, const std::string& brick_name);
        std::string get_brick_name_by_id(const int& id, const int& seq_id);
        void update_brick_connection();
        void calc_brick_grab_pose(const std::string& name, const bool& assemble_pose, const bool& take_brick,
                                  const int& brick_assemble_x, const int& brick_assemble_y, const int& brick_assemble_z, 
                                  const int& orientation, const int& press_side, Eigen::MatrixXd& T);
        int brick_instock(const std::string& name) {return brick_map_[name].in_stock;};
        int robot_dof() {return robot_dof_;};
        Eigen::MatrixXd robot_DH() {return DH_;};
        Eigen::MatrixXd robot_DH_tool() {return DH_tool_;};
        Eigen::MatrixXd robot_DH_tool_assemble() {return DH_tool_assemble_;};
        Eigen::MatrixXd robot_DH_tool_disassemble() {return DH_tool_disassemble_;};
        Eigen::MatrixXd robot_base() {return base_frame_;};
        Eigen::Matrix4d robot_base_inv() {return T_base_inv_;};
        Eigen::Matrix4d robot_ee_inv() {return ee_inv_;};
        Eigen::Matrix4d robot_tool_inv() {return tool_inv_;};
        Eigen::Matrix4d robot_tool_assemble_inv() {return tool_assemble_inv_;};
        Eigen::Matrix4d robot_tool_disassemble_inv() {return tool_disassemble_inv_;};
        bool robot_is_static(math::VectorJd robot_qd, math::VectorJd robot_qdd);
        bool robot_reached_goal(math::VectorJd robot_q, math::VectorJd goal);
};
}
}