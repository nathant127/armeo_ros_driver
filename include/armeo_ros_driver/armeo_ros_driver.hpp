#pragma once
#include <rclcpp/rclcpp.hpp>

#include "controller.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <vector>
#include <memory>
#include <string>
#include <array>



class ArmeoRosDriver : public rclcpp::Node {
public:
    ArmeoRosDriver();
    ~ArmeoRosDriver();

private:

    void initialize_armeo();

    void update();

    void publish_state();

    void load_config();

    void SP(int idx, double val);
    bool is_estop_pressed();

private:

    std::shared_ptr<ArmeoState> state_;
    int xpc_port_;

    rclcpp::TimerBase::SharedPtr update_timer_;

    std::unique_ptr<ArmeoController> controller_;

    const std::vector<std::string> joint_names_ = {
        "Base joint",
    	"Shoulder joint",
    	"Shoulder twist joint",
    	"Elbow joint",
    	"Arm Twist Joint",
    	"Wrist joint",
    	"Grip joint"
    };

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    sensor_msgs::msg::JointState joint_states_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jacobian_publisher_;
    std_msgs::msg::Float64MultiArray jacobian_msg_;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    
};