#include "armeo_ros_driver.hpp"
#include "xpcapi.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "std_msgs/msg/multi_array_dimension.hpp"

#include <exception>
#include <stdexcept>

#include "fkine.hpp"

#include "position_controller.hpp"
#include "torque_controller.hpp"

using namespace xpcapi;

#define OP_MODE 141
#define SAFETY_RESET 155
#define SAFETY_ENABLE 154
#define CALIBRATE_ENCODERS 136
#define AC_WATCHDOG 135
#define SEND_UDP_PACKETS 138
#define MANOVO_POWER 424

#define S_WATCHDOG_OK 4831

#define OMEStopTest 6.000000
#define OMWatchdogTest 9.000000

#define STATUS_ESTOP_BIT 2

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

    using namespace std::chrono_literals;
    update_timer_ = this->create_wall_timer(
        100ms, 
        std::bind(&ArmeoRosDriver::update, this)
    );

    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    joint_states_.name = joint_names_;
    joint_states_.position.resize(7, 0.0);
    joint_states_.velocity.resize(7, 0.0);
    joint_states_.effort.resize(7, 0.0);


    // Jacobian Publisher
    jacobian_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("jacobian", 10);

    std_msgs::msg::MultiArrayDimension dim1;
    dim1.label = "rows";
    dim1.size = 6;
    dim1.stride = 36;

    std_msgs::msg::MultiArrayDimension dim2;
    dim2.label = "columns";
    dim2.size = 6;
    dim2.stride = 6;
    jacobian_msg_.layout.dim.push_back(dim1);
    jacobian_msg_.layout.dim.push_back(dim2);
    jacobian_msg_.layout.data_offset = 0;
    jacobian_msg_.data.resize(36, 0.0);

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("ee_pose", 10);

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("ee_vel", 10);

    state_ = std::make_shared<ArmeoState>();

    load_config();

    if (controller_ != nullptr) {
        initialize_armeo();
    }
}

ArmeoRosDriver::~ArmeoRosDriver() {
    if (xPCIsAppRunning(xpc_port_)) {
        xPCStopApp(xpc_port_);
    }
    xPCUnloadApp(xpc_port_);
}

struct SensorQ {
    int param_id;
    double param_vals[20];
};
const std::array<SensorQ, 22> sensor_qs = {
    SensorQ {444, {-3.855432,-3.748824,-3.642305,-3.535644,-3.428935,-3.322148,-3.215532,-3.108886,-3.002224,-2.895702,-2.789009,-2.682456,-2.575903,-2.469319,-2.362594,-2.256104,-2.149613,-2.043107,-1.936554,-1.779770}},
    SensorQ {432, { 1.727295, 1.848145, 1.970215, 2.097168, 2.215576, 2.349854, 2.479248, 2.600098, 2.725830, 2.851562, 2.968750, 3.090820, 3.215332, 3.343506, 3.470459, 3.590088, 3.719482, 3.845215, 3.966064, 4.145508}},
    SensorQ {445, {-2.151669,-2.012279,-1.942179,-1.872262,-1.802201,-1.732140,-1.662209,-1.592095,-1.522190,-1.452102,-1.382210,-1.312384,-1.242309,-1.172482,-1.102407,-1.032345,-0.962295,-0.892298,-0.822313,-0.752237}},
    SensorQ {433, { 1.492920, 1.707764, 1.817627, 1.933594, 2.048340, 2.160645, 2.274170, 2.382812, 2.496338, 2.600098, 2.714844, 2.827148, 2.937012, 3.046875, 3.161621, 3.271484, 3.378906, 3.488770, 3.598633, 3.704834}},
    SensorQ {446, {-1.591740,-1.668824,-1.745878,-1.822957,-1.899839,-1.976683,-2.053693,-2.130586,-2.207422,-2.284449,-2.361265,-2.438091,-2.515070,-2.592009,-2.668883,-2.745835,-2.822812,-2.899710,-2.976743,-3.146142}},
    SensorQ {434, { 4.989014, 5.041504, 5.100098, 5.158691, 5.218506, 5.277100, 5.335693, 5.396729, 5.456543, 5.516357, 5.577393, 5.635986, 5.699463, 5.756836, 5.819092, 5.880127, 5.936279, 6.002197, 6.054688, 6.193848}},
    SensorQ {441, {-0.022689, 0.069008, 0.160769, 0.252529, 0.344248, 0.435973, 0.527702, 0.619403, 0.711092, 0.802893, 0.894729, 0.986505, 1.078284, 1.169926, 1.261742, 1.353438, 1.445199, 1.537026, 1.628763, 1.811146}},
    SensorQ {429, { 3.560791, 3.673096, 3.817139, 3.953857, 4.097900, 4.243164, 4.387207, 4.532471, 4.672852, 4.809570, 4.948730, 5.093994, 5.244141, 5.388184, 5.533447, 5.668945, 5.811768, 5.961914, 6.114502, 6.361084}},
    SensorQ {442, {-2.738422,-2.622116,-2.505824,-2.389646,-2.273303,-2.157111,-2.041008,-1.924867,-1.808750,-1.692393,-1.576174,-1.460020,-1.343751,-1.227571,-1.111239,-0.995121,-0.878927,-0.762607,-0.646424,-0.432083}},
    SensorQ {430, { 1.004639, 1.367188, 1.744385, 2.062988, 2.473145, 2.894287, 3.319092, 3.747559, 4.171143, 4.598389, 5.024414, 5.443115, 5.864258, 6.276855, 6.685791, 7.114258, 7.531738, 7.937012, 8.294678, 8.978271}},
    SensorQ {443, {-1.104793,-0.995651,-0.886550,-0.777354,-0.668103,-0.558845,-0.449637,-0.340249,-0.230886,-0.121610,-0.012291, 0.096904, 0.206087, 0.315401, 0.424527, 0.533658, 0.643027, 0.752266, 0.861495, 1.071141}},
    SensorQ {431, { 3.165283, 3.320312, 3.486328, 3.651123, 3.823242, 3.991699, 4.157715, 4.323730, 4.492188, 4.665527, 4.841309, 5.009766, 5.180664, 5.358887, 5.527344, 5.693359, 5.869141, 6.038818, 6.214600, 6.533203}},
    SensorQ {447, { 0.275398, 0.053463, 0.000987,-0.051494,-0.103910,-0.156280,-0.208709,-0.261142,-0.313555,-0.365922,-0.418387,-0.470834,-0.523217,-0.575603,-0.627991,-0.680495,-0.732903,-0.785282,-0.837747,-0.890118}},
    SensorQ {435, { 4.689941, 4.986572, 5.048828, 5.098877, 5.147705, 5.218506, 5.302734, 5.375977, 5.451660, 5.522461, 5.592041, 5.646973, 5.684814, 5.756836, 5.834961, 5.917969, 5.992432, 6.058350, 6.124268, 6.134033}},
    SensorQ {448, {-1.367030,-1.205922,-1.099209,-0.992480,-0.885908,-0.779163,-0.672449,-0.565862,-0.459227,-0.352481,-0.245752,-0.139022,-0.032434, 0.074280, 0.180916, 0.287426, 0.394189, 0.500717, 0.607281, 0.713840}},
    SensorQ {436, { 4.588623, 4.779053, 4.898682, 5.023193, 5.151367, 5.278320, 5.404053, 5.524902, 5.653076, 5.776367, 5.900879, 6.020508, 6.142578, 6.263428, 6.390381, 6.518555, 6.640625, 6.763916, 6.889648, 7.008057}},
    SensorQ {449, { 0.685914, 0.755902, 0.825808, 0.895651, 0.965521, 1.035600, 1.105653, 1.175575, 1.245627, 1.315628, 1.385602, 1.455446, 1.525460, 1.595435, 1.665279, 1.735280, 1.805360, 1.875348, 1.945441, 2.082957}},
    SensorQ {437, { 5.897217, 5.993652, 6.102295, 6.215820, 6.324463, 6.433105, 6.547852, 6.655273, 6.757812, 6.868896, 6.982422, 7.087402, 7.200928, 7.312012, 7.423096, 7.525635, 7.633057, 7.739258, 7.849121, 8.074951}},
    SensorQ {450, {-0.001404,-0.168500,-0.245408,-0.322344,-0.399180,-0.476217,-0.553142,-0.630169,-0.706993,-0.783918,-0.860888,-0.937869,-1.014923,-1.091958,-1.168837,-1.245735,-1.322677,-1.399591,-1.476517,-1.553343}},
    SensorQ {438, { 3.739014, 3.873291, 3.930664, 3.989258, 4.047852, 4.108887, 4.167480, 4.227295, 4.283447, 4.346924, 4.403076, 4.468994, 4.522705, 4.586182, 4.648438, 4.709473, 4.763184, 4.822998, 4.884033, 4.930420}},
    SensorQ {451, { 0.890118, 0.837667, 0.785305, 0.732857, 0.680411, 0.627987, 0.575567, 0.523148, 0.470683, 0.418216, 0.365707, 0.313211, 0.260754, 0.208370, 0.155987, 0.103510, 0.051089,-0.001396,-0.053788,-0.261675}},
    SensorQ {439, { 3.909912, 3.922119, 4.008789, 4.062500, 4.132080, 4.204102, 4.278564, 4.351807, 4.410400, 4.462891, 4.536133, 4.598389, 4.654541, 4.735107, 4.808350, 4.865723, 4.934082, 4.984131, 5.035400, 5.333252}},
};

// Helper for single elem params
inline void SP(int port, int idx, double val) {
    xPCSetParam(port, idx, &val);
}

// Helper for checking estop state
inline bool is_estop_pressed(int port) {
    return ((uint32_t)xPCGetSignal(port, 696)) & (1 << STATUS_ESTOP_BIT);
}

void ArmeoRosDriver::initialize_armeo() {
    SP(port, SAFETY_RESET, 0.0);
    SP(port, SAFETY_RESET, 1.0);
    SP(port, SAFETY_RESET, 0.0);

    SP(port, OP_MODE, OMEStopTest);

    bool check_estop = this->declare_parameter<bool>("skip_estop_check", false);
    if (is_estop_pressed(port)) {
        RCLCPP_INFO(this->get_logger(), "Please release the estop\n");
        while(is_estop_pressed(port)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    if (check_estop) {
        RCLCPP_INFO(this->get_logger(), "Please press the estop\n");
        while(!is_estop_pressed(port)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        RCLCPP_INFO(this->get_logger(), "Please release the estop again\n");
        while(is_estop_pressed(port)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    SP(port, SAFETY_ENABLE, 1.0);
    // Calib encoders
    SP(port, CALIBRATE_ENCODERS, 0.0);
    SP(port, CALIBRATE_ENCODERS, 1.0);
    SP(port, CALIBRATE_ENCODERS, 0.0);

    SP(port, SAFETY_RESET, 0.0);
    SP(port, SAFETY_RESET, 1.0);
    SP(port, SAFETY_RESET, 0.0);
    SP(port, SAFETY_RESET, 0.0);
    SP(port, SAFETY_RESET, 1.0);
    SP(port, SAFETY_RESET, 0.0);

    SP(port, OP_MODE, OMWatchdogTest);
    while (xPCGetSignal(port, S_WATCHDOG_OK) != 1.0) {
        std::this_thread::sleep_for(10ms);
    }

    SP(port, SAFETY_RESET, 0.0);
    SP(port, SAFETY_RESET, 1.0);
    SP(port, AC_WATCHDOG, 0.0);
    SP(port, SAFETY_RESET, 0.0);
    SP(port, OP_MODE, OMEStopTest);

    // Set sensor values
    for (size_t i = 0; i < sensor_qs.size(); i++) {
        xPCSetParam(port, sensor_qs[i].param_id, sensor_qs[i].param_vals);
    }

    SP(port, CALIBRATE_ENCODERS, 0.0);
    SP(port, CALIBRATE_ENCODERS, 1.0);
    SP(port, AC_WATCHDOG, 1.0);
    SP(port, CALIBRATE_ENCODERS, 0.0);
    SP(port, SEND_UDP_PACKETS, 1.0);
    SP(port, MANOVO_POWER, 1.0);

    SP(port, SAFETY_ENABLE, 0.0);
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

    // RCLCPP_INFO(this->get_logger(), "Q: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", state_->joint_angles[0], state_->joint_angles[1], state_->joint_angles[2], state_->joint_angles[3], state_->joint_angles[4], state_->joint_angles[5], state_->joint_angles[6]);

    // Perform calculations
    state_->ee_velocities = state_->jacobian * state_->joint_velocities;
    state_->ee_pose = fkine(state_->joint_angles);
    state_->ee_rot = state_->ee_pose.block<3,3>(0,0); // Grab rotation matrix from pose

    if (controller_ != nullptr) {
        controller_->update();
    }
    publish_state();
}

void ArmeoRosDriver::load_config() {
    std::string controller_name = this->declare_parameter<std::string>("controller_name");

    if (controller_name == TorqueController::get_name()) {
        this->controller_ = std::make_unique<TorqueController>(this->state_, this->xpc_port_, this);
    }
    if (controller_name == PositionController::get_name()) {
        this->controller_ = std::make_unique<PositionController>(this->state_, this->xpc_port_, this);
    }
}

void ArmeoRosDriver::publish_state() {

    // Joint States

    joint_states_.header.stamp = this->get_clock()->now();
    for (int i = 0; i < 6; i++) {
        joint_states_.position[i] = state_->joint_angles[i];
        joint_states_.velocity[i] = state_->joint_velocities[i];
    }
    joint_states_.position[6] = state_->gripper_joint_angle;
    joint_states_.velocity[6] = state_->gripper_joint_vel;
    joint_state_publisher_->publish(joint_states_);

    // Jacobian

    int index = 0;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            jacobian_msg_.data[index++] = state_->jacobian(i, j);
        }
    }

    jacobian_publisher_->publish(jacobian_msg_);

    // Pose

    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = state_->ee_position[0];
    pose_msg.position.y = state_->ee_position[1];
    pose_msg.position.z = state_->ee_position[2];
    pose_msg.orientation.x = state_->ee_rot.x();
    pose_msg.orientation.y = state_->ee_rot.y();
    pose_msg.orientation.z = state_->ee_rot.z();
    pose_msg.orientation.w = state_->ee_rot.w();

    pose_publisher_->publish(pose_msg);

    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = state_->ee_velocities[0];
    twist_msg.linear.y = state_->ee_velocities[1];
    twist_msg.linear.z = state_->ee_velocities[2];
    twist_msg.angular.x = state_->ee_velocities[3];
    twist_msg.angular.y = state_->ee_velocities[4];
    twist_msg.angular.z = state_->ee_velocities[5];

    twist_publisher_->publish(twist_msg);




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