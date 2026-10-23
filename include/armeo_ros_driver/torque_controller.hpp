#pragma once

#include "controller.hpp"
#include "xpc_def.hpp"
#include "xpcapi.hpp"

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"

constexpr int torque_id_base = 467;


class TorqueController : public ArmeoController {
public:

    TorqueController(std::shared_ptr<ArmeoState> state, int xpc_port, rclcpp::Node* node) :
        ArmeoController(state, xpc_port),
        node_(node)
    {
        sub_torque_cmd_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>("torque_cmd", 10,
            std::bind(&TorqueController::cmd_callback, this, std::placeholders::_1)
        );

        prev_cmd_.resize(7);
        for (int i = 0; i < 7; i++) {
            prev_cmd_[i] = 0.0;
            xpcapi::xPCSetParam(this->xpc_port_, torque_id_base, &prev_cmd_[i]);
        }

        double buf;

        buf = 0.0;
        xpcapi::xPCSetParam(this->xpc_port_, SAFETY_ENABLE, &buf);
        buf = OMDirectTorque;
        xpcapi::xPCSetParam(this->xpc_port_, OP_MODE, &buf);
    }

    static std::string get_name() {return "torque_controller";}

    void update() {
        if (!cmd_valid_) {
            return;
        }
        if (!cmd_ready_) {
            auto duration = node_->now() - cmd_stamp_;
            if (duration.nanoseconds() < 500000000ll) {
                return;
            }

            for (int i = 0; i < 7; i++) {
                prev_cmd_[i] = 0.0;
                xpcapi::xPCSetParam(this->xpc_port_, torque_id_base, &prev_cmd_[i]);
            }

            cmd_valid_ = false;
        }

        if (cmd_buf_.data.size() != 7) {
            cmd_ready_ = false;
            return;
        }

        for (int i = 0; i < 7; i++) {
            xpcapi::xPCSetParam(this->xpc_port_, torque_id_base, &cmd_buf_.data[i]);
        }

        prev_cmd_ = cmd_buf_.data;

        cmd_ready_ = false;

    }

private:

    void cmd_callback(const std_msgs::msg::Float64MultiArray& msg) {
        cmd_buf_ = msg;
        cmd_stamp_ = node_->now();
        cmd_ready_ = true;
        cmd_valid_ = true;
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_torque_cmd_;
    rclcpp::Time cmd_stamp_;
    std_msgs::msg::Float64MultiArray cmd_buf_;
    bool cmd_ready_ = false;
    bool cmd_valid_ = false;

    std::vector<double> prev_cmd_;

    rclcpp::Node* node_;
};