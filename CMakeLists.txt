cmake_minimum_required(VERSION 3.21)
project(armeo_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(armeo_msgs REQUIRED)


add_subdirectory(xpctarget_api_cpp)

add_executable(armeo_ros_driver
    src/armeo_ros_driver.cpp
)
target_link_libraries(armeo_ros_driver xpcapi Eigen3::Eigen)
ament_target_dependencies(armeo_ros_driver rclcpp sensor_msgs geometry_msgs std_msgs armeo_msgs)

target_include_directories(armeo_ros_driver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(armeo_ros_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS armeo_ros_driver
    DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY launch
#     DESTINATION share/${PROJECT_NAME}
# )
install(DIRECTORY models
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()