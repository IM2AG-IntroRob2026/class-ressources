# Lab 1: ROS2 Basics

## 1. Installation

### ROS2 Jazzy (Ubuntu 24.04)
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt upgrade
sudo apt install ros-jazzy-desktop
```

### ROS2 Humble (Ubuntu 22.04)
Follow similar steps but replace `jazzy` with `humble` in the package name:
```bash
sudo apt install ros-humble-desktop
```

## 2. Environment Setup
Add to `~/.bashrc`:
```bash
source /opt/ros/jazzy/setup.bash
```

## 3. ROS2 CLI Tools & Demos

### Nodes & Topics
ROS2 applications are composed of **Nodes** that communicate via **Topics**. A topic is a bus over which nodes exchange messages.

Start a C++ talker node:
```bash
ros2 run demo_nodes_cpp talker
```
Start a Python listener node in another terminal to see the communication:
```bash
ros2 run demo_nodes_py listener
```

### Turtlesim Basics
`turtlesim` is a lightweight simulator for learning ROS2 basics.

Start the simulator:
```bash
ros2 run turtlesim turtlesim_node
```
Run a node that automatically moves the turtle in a square:
```bash
ros2 run turtlesim draw_square
```

### Topic Inspection
Use these commands to discover and inspect active data streams.

List all active topics:
```bash
ros2 topic list
```
Get details about a specific topic (type, publishers, subscribers):
```bash
ros2 topic info /turtle1/cmd_vel
```
View the actual data being published on a topic:
```bash
ros2 topic echo /turtle1/pose
```
Show the structure of a message type:
```bash
ros2 interface show geometry_msgs/msg/Twist
```

### Services
Services follow a **Request/Response** pattern, ideal for discrete calls like "spawn a robot" or "trigger a sensor".

List available services:
```bash
ros2 service list
```
Call a service to spawn a second turtle:
```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.2, name: 'turtle2'}"
```

### Actions
Actions are for long-running tasks. They provide a **Goal**, intermediate **Feedback**, and a final **Result**.

List active action servers:
```bash
ros2 action list
```
Send a goal to rotate the turtle; notice the feedback as it turns:
```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
```

## 4. Development Implementation

### Create Workspace
Everything in ROS2 is built inside a workspace.
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Create Package
A package is the unit of software in ROS2.
```bash
ros2 pkg create --build-type ament_cmake my_robot_controller --dependencies rclcpp geometry_msgs
```

### Package Implementation Checklist
Once a package is created, verify these files to ensure it's detectable and functional:

#### 1. `package.xml`
- **Dependencies**: Ensure `<depend>` tags match your requirements (e.g., `rclcpp`, `geometry_msgs`).
- **Metadata**: Update version, maintainer, and license fields.

#### 2. `CMakeLists.txt`
- **find_package**: Must include all dependencies mentioned in `package.xml`.
- **add_executable**: Define the binary name and source file.
- **ament_target_dependencies**: Link your executable to ROS2 libraries.
- **install**: Specify where the binary and other files (launch, config) should be installed.
- **ament_package()**: Must be the last line to finalize the package.

#### 3. Custom Interfaces (Messages/Services/Actions)
If creating a dedicated interface package:
- Add `.msg`, `.srv`, or `.action` files in the respective subdirectories.
- In `package.xml`: Add `<build_depend>`, `<exec_depend>`, and `<member_of_group>` for `rosidl_default_generators`.
- In `CMakeLists.txt`: Use `rosidl_generate_interfaces(${PROJECT_NAME} "msg/MyMsg.msg" ...)` and add dependencies.

### Parameters
Parameters are configuration values for nodes. You can declare them in code and override them at runtime.

In your node (C++):
```cpp
this->declare_parameter("my_param", 10);
int val = this->get_parameter("my_param").as_int();
```

Run with a parameter override:
```bash
ros2 run my_robot_controller controller_node --ros-args -p my_param:=20
```
