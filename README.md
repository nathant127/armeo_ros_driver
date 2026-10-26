# armeo_ros_driver

This ROS2 Driver is designed to create an interface through which to control the ArmeoPower Robotic Exoskeleton. 
It currently has two controllers implemented that directly correspond to operation modes within the Armeo Power model.

The Armeo Power is a 7 link robot, with the 7th being an optional "grip" joint which does not contribute to the end effector or jacobian

For more detail about the Aremo Power see [armeo_docs](https://github.com/nathant127/armeo_docs)

This ros driver uses the message types from [armeo_msgs](https://github.com/nathant127/armeo_msgs)

# Features
The Aremo Ros Driver is capable of publishing the current state of the Armeo, it does this even if no controller is loaded.

| Topic Name | Type | Desciption |
|------------|------|------------|
| /joint_states | sensor_msgs::msg::JointState | Contains the joint angle (rad) and joint velocity (rad/s) of the Armeo, effort is currently left empty |
| /jacobian | std_msgs::msg::Float64MultiArray | Contains the 6x6 jacobian of the Armeo, in row major order |
| /ee_pose | geometry_msgs::msg::Pose | Contains the end effector pose |
| /ee_vel | geometry_msgs::msg::Twist | Contains the end effector velocity |

## Controllers

The Ros Driver currently has two different controllers that correspond directly to the internal operation modes "Mobilization" ([Mobilization Mode](https://github.com/nathant127/armeo_docs/blob/main/docs/mobilization_mode.md)) and "Direct Torque" ([Direct Torque Mode](https://github.com/nathant127/armeo_docs/blob/main/docs/direct_torque_mode.md)).

The reason the controllers correspond directly to operation modes is for simplicity, allowing a user to understand the underlying control interaface so that they can create their own wrapper/abstraction as needed.

### Position Controller

The [position_controller](/include/armeo_ros_driver/position_controller.hpp) uses Mobilization mode, and creates two subscriptions:

| Topic Name | Type | Desciption |
|------------|------|------------|
| /trajectory | armeo_msgs::msg::Trajectory | A type that allows defining up to 20 "targets", comprised of a joint angle (rad) and time duration (s) for each joint as a trajectory to follow |
| /startstop | std_msgs::msg::Bool | Tells controller to start/stop moving (true/false) |

The idea is you send a trajectory to /trajectory, then send a start command to /startstop. For safety reasons, when you send a new trajectory, it will stop and require a new start command to be sent.

Note that only one trajectory can be active at a time, and it will loop back to the start of the trajectory at the end, repeating forever.

### Torque Controller
The [torque_controller](/include/armeo_ros_driver/torque_controller.hpp) uses Direct Torque mode, and creates one subscriptions:

| Topic Name | Type | Desciption |
|------------|------|------------|
| /torque_cmd | std_msgs::msg::Float64MultiArray | A 7 element array of torques for each joint |

Once the controller is loaded, the robot will default to a torque of 0, you should then constantly send new torques in a loop, if a new torque is not sent for a period of time the controller will stop until it recieves a new one for safety.

# Build/Installation

## Dependancies

- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- Eigen3
- [armeo_msgs](https://github.com/nathant127/armeo_msgs)
- [xpctarget_api_cpp](https://github.com/nathant127/xpctarget_api_cpp) (included as a submodule)

## Build

First clone and setup the repository in a ros2 workspace
```bash
mkdir -p armeo_workspace/src
cd armeo_workspace/src

git clone https://github.com/nathant127/armeo_ros_driver
cd armeo_ros_driver
git submodule update --init

cd ..
git clone https://github.com/nathant127/armeo_msgs
```

Now Build
```bash
colcon build --symlink-install
```

# Usage

Plug in the Armeo Power Target PC to your device over ethernet, the Armeo Power Target PC address is 10.10.10.11:22222. 

So that your device knows how to route traffic to the Armeo Power's Subnet, ensure that the connection you use is set to a static IP with subnet 10.10.10.XX, preferably 10.10.10.10 as that is the address that the offical Armeo Power PC uses to conenct.
How this is done depends on the operating system, but you can do this with nmcli like so:
```bash
sudo nmcli connection modify "ETHERNET CONNECTION NAME" \
  ipv4.method manual \
  ipv4.addresses "10.10.10.10/24" \
  ipv4.gateway "" \
  ipv4.never-default yes
```

Run the package with a specific controller (default controller is none)
```bash
ros2 run armeo_ros_driver armeo_ros_driver --ros-args -p controller_name:=torque_controller 
```
