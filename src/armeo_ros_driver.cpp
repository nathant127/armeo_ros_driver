#include "armeo_ros_driver.hpp"
#include "xpcapi.hpp"

#include <string>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <exception>
#include <stdexcept>

#include "controller.hpp"

using namespace xpcapi;


ArmeoRosDriver::ArmeoRosDriver() :
    Node("ArmeoRosDriver")
{
    constexpr char addr[] = "10.10.10.11";
    constexpr char net_port[] = "22222";
    this->xpc_port_ = xPCOpenTcpIpPort(addr, net_port);

    
    char name[100];
    // Check if an app is loaded
    // If no app is loaded, the app is called "loader"
    xPCGetAppName(xpc_port_, name);
    if (std::strcmp("loader", name) != 0) {
        // Stop app if it is running
        if (xPCIsAppRunning(xpc_port_)) {
            xPCStopApp(xpc_port_);
        }
        xPCUnloadApp(xpc_port_);
    }
    std::string path;
    path = ament_index_cpp::get_package_share_directory("armeo_ros_driver") + "/models";
    constexpr char target_model[] = "AM2_3_1_01HWfullV2_PCIe";
    xPCLoadApp(xpc_port_, path.c_str(), target_model);    
    // Start the app
    xPCStartApp(xpc_port_);
}

ArmeoRosDriver::~ArmeoRosDriver() {
    if (xPCIsAppRunning(xpc_port_)) {
        xPCStopApp(xpc_port_);
    }
    xPCUnloadApp(xpc_port_);
}

constexpr int num_signals = 7 + 3 + 36 + 7;

static std::array<int, num_signals> generate_sig_ids(int port) {
    std::array<int, num_signals> arr;
    int idx = 0;

    int base_id = xPCGetSignalIdx(port, "Output/SoftwareSignals/JointAngle/s1");
    for (int i = 0; i < 7; i++) {
        arr[idx] = base_id + i;
        idx++;
    }
    base_id = xPCGetSignalIdx(port, "Output/SoftwareSignals/EndEffector/s1");
    for (int i = 0; i < 3; i++) {
        arr[idx] = base_id + i;
        idx++;
    }
    base_id = xPCGetSignalIdx(port, "Kinematics/Jacobian EE/s1");
    for (int i = 0; i < 36; i++) {
        arr[idx] = base_id + i;
        idx++;
    }
    base_id = xPCGetSignalIdx(port, "Output/SoftwareSignals/JointVelocity/s1");
    for (int i = 0; i < 7; i++) {
        arr[idx] = base_id + i;
        idx++;
    }

    return arr;
}

void ArmeoRosDriver::update() {

    static std::array<int, num_signals> sig_ids = generate_sig_ids(this->xpc_port_);
    double sig_data[num_signals];
    xPCGetSignals(this->xpc_port_, num_signals, sig_ids.data(), sig_data);
    auto time = this->now();

    state_->timestamp.ns = time.nanoseconds();
    state_->timestamp.sec = time.seconds();
    
    double* q = &sig_data[0];
    state_->joint_angles << q[0], q[1], q[2], q[3], q[4], q[5];
    state_->gripper_joint_angle = q[6];
    double* ee = &sig_data[7];
    state_->ee_position << ee[0], ee[1], ee[2];
    double* j = &sig_data[10]; // 6x6 matrix, col major ordering
    state_->jacobian <<
        j[0], j[6],  j[12], j[18], j[24], j[30],
        j[1], j[7],  j[13], j[19], j[25], j[31],
        j[2], j[8],  j[14], j[20], j[26], j[32],
        j[3], j[9],  j[15], j[21], j[27], j[33],
        j[4], j[10], j[16], j[22], j[28], j[34],
        j[5], j[11], j[17], j[23], j[29], j[35];

    double* qv = &sig_data[46]; // joint velocities, size 7
    state_->joint_velocities << qv[0], qv[1], qv[2], qv[3], qv[4], qv[5];
    state_->gripper_joint_vel = qv[6];

}

void ArmeoRosDriver::load_config() {

}

void ArmeoRosDriver::publish_state() {

}


int main(int argc, char** argv) {
    int rc = 0;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmeoRosDriver>();
    try {
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "Unhandled exception: %s", e.what());
        rc = 1;
        
    }
    rclcpp::shutdown();
    
    return rc;
}