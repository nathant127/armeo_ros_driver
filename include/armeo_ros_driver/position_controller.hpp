#pragma once

#include <array>
#include <memory>

#include "controller.hpp"
#include "xpc_def.hpp"
#include "xpcapi.hpp"

#include <rclcpp/rclcpp.hpp>
#include "armeo_msgs/msg/trajectory.hpp"

#include "std_msgs/msg/bool.hpp"

#define REPLAY 530
#define RECORD 529
#define REPLAY_SPEED 531


class TorqueController : public ArmeoController {
public:

    TorqueController(std::shared_ptr<ArmeoState> state, int xpc_port, rclcpp::Node* node) :
        ArmeoController(state, xpc_port),
        node_(node)
    {
        sub_targets_ = this->create_subscriber<armeo_msgs::msg::Trajectory>("trajectory", 10,
            std::bind(&TorqueController::cmd_callback, std::placeholder::_1, this)
        );

        sub_startstop_ = this->create_subscriber<std_msgs::msg::Bool>("startstop", 10,
            std::bind(&TorqueController::startstop_callback, std::placeholder::_1, this)
        );

        double buf;
        buf = 0.0;
        xpcapi::xPCSetParam(this->xpc_port_, REPLAY, &buf);
        buf = 0.0;
        xpcapi::xPCSetParam(this->xpc_port_, RECORD, &buf);
        buf = 1.0;
        xpcapi::xPCSetParam(this->xpc_port_, REPLAY_SPEED, &buf);
        buf = 0.0;
        xpcapi::xPCSetParam(this->xpc_port_, SAFETY_ENABLE, &buf);
        buf = OMMobilization;
        xpcapi::xPCSetParam(this->xpc_port_, OP_MODE, &buf);
    }

    static int get_target_id(int target_num, int joint_num) {
        constexpr std::array<int, 20> offset_lookup = {
            1,12,14,15,16,17,18,19,20,2,3,4,5,6,7,8,9,10,11,13
        };
        return get_num_targets_id(joint_num) + offset_lookup[target_num];
    }

    static int get_num_targets_id(int joint_num) {
        constexpr std::array<int, 7> base_lookup = {
            1659, 1681, 1703, 1725, 1747, 1769, 1791
        };
        return base_lookup[joint_num];
    }

    void update() {
        if (!cmd_ready_) {
            return;
        }

        double buf[2];
        buf[0] = 0.0;
        xpcapi::xPCSetParam(this->xpc_port_, REPLAY, &buf);

        for (int joint = 0; joint < targets_buf_.trajectories.size(); joint++) {
            const auto& traj = targets_buf_.trajectories[joint];
            buf[0] = traj.targets.size()
            xpcapi::xPCSetParam(this->xpc_port_, get_num_targets_id(joint), &buf);

            for (int target_idx = 0; target_idx < traj.targets.size(); target_idx++) {
                buf[0] = traj.targets[target_idx].joint_angle;
                buf[1] = traj.targets[target_idx].duration_sec;
                xpcapi::xPCSetParam(this->xpc_port_, get_target_id(target_idx, joint), &buf);
            }

        }
        



        target_set_ = true;
        cmd_ready_ = false;
    }

private:

    void cmd_callback(const armeo_msgs::msg::Trajectory::SharedPtr msg) {
        targets_buf_ = *msg;
        cmd_stamp_ = node_->now();
        cmd_ready_ = true;
        cmd_valid_ = true;
    }

    void startstop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (target_set_ && msg->data) {
            double buf = 1.0;
            xpcapi::xPCSetParam(this->xpc_port_, REPLAY, &buf);
        }
        if (!msg->data) {
            double buf = 0.0;
            xpcapi::xPCSetParam(this->xpc_port_, REPLAY, &buf);
        }
    }

    rclcpp::Subscription<armeo_msgs::msg::Trajectory>::SharedPtr sub_targets_;
    armeo_msgs::msg::Trajectory targets_buf_;
    bool target_set_ = false;
    rclcpp::Time cmd_stamp_;
    bool cmd_ready_ = false;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_startstop_;

    rclcpp::Node* node_;
}