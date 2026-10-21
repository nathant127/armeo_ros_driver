#pragma once
#include <rclcpp/rclcpp.hpp>

#include "controller.hpp"



class ArmeoRosDriver : public rclcpp::Node {
public:
    ArmeoRosDriver();
    ~ArmeoRosDriver();

private:

    void update();

    void publish_state();

    void load_config();

private:

    std::shared_ptr<ArmeoState> state_;
    int xpc_port_;

    rclcpp::TimerBase::SharedPtr update_timer_;

    std::unique_ptr<ArmeoController> controller_;
};