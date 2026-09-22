#include "armeo_ros_driver.hpp"
#include "xpcapi.hpp"

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace xpcapi;

constexpr char target_model[] = "AM2_3_1_01HWfullV2_PCIe";

int main() {

    int port = xPCOpenTcpIpPort("127.0.0.1", "22222");
    try {
        std::string path = ament_index_cpp::get_package_share_directory("armeo_ros_driver") + "/models";
    } catch (const ament_index_cpp::PackageNotFoundError & e) {
        return 1;
    }
    
    xPCLoadApp(port, path, target_model);
}
