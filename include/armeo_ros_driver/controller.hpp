#pragma once
#include <Eigen/Dense> 
#include "xpcapi.hpp"

struct ArmeoState {
    struct {
        uint64_t ns;
        double sec;
    } timestamp;

    Eigen::Vector<double, 6> joint_angles;
    Eigen::Vector<double, 3> ee_position;
    Eigen::Quaternion<double> ee_rot;
    Eigen::Matrix<double, 4, 4> ee_pose;
    
    Eigen::Matrix<double, 6, 6> jacobian;
    Eigen::Vector<double, 6> joint_velocities;
    Eigen::Vector<double, 6> ee_velocities;


    double gripper_joint_angle;
    double gripper_joint_vel;
};

class ArmeoController {
public:
    ArmeoController(std::shared_ptr<ArmeoState> state, int xpc_port) :
        state_(state),
        xpc_port_(xpc_port)
    {}

    
    virtual void update() = 0;

private:
    std::shared_ptr<ArmeoState> state_;
    int xpc_port_;

    rclcpp::Node* node_;
};