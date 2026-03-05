# Lab 5: Square IRL with Create 3

The objective of this lab is to get familiar with the Create3 robot by trying to have it drive in a square using the code created for turtlesim.  

## Connect with the Create 3 

The following instructions are adapted from [irobot web site](https://iroboteducation.github.io/create3_docs/) and from experience. 

### Pre-requisite

#### A native ubuntu device 

In theory, any device (e.g., Raspberry Pi, etc) with ubuntu should work too. If your team does not have any Ubuntu device, The school will lend you one. 

Virtual machines or containers on an OS other than Ubuntu have not yet been reported to work successfully, though if a team with special skills in network bridges manages to design a robust and reproducible solution, that would be appreciated. 

#### ros-iron-cyclone via docker (or not)

The robot communicates most reliably with ROS2 Iron version using the CycloneDDS DDS implementation. For convenience, a docker image, ros-iron-cyclone, is available on [the class docker repo](https://github.com/IM2AG-IntroRob2026/docker_images). When testing, make sure to build your code and run it on this image. 

In principle, ROS2 Iron and CycloneDDS could be installed on the laptop as well as the main ROS2 version instead of Jazzy, to avoid the use of docker.  

#### A Wifi hotspot

The robot and the laptop will need to be connected to the same Wifi network. If you can, use a phone to create one. Otherwise, you can use the following:

- SSID: wifi-F218
- Password: im2ag-intro-rob

When using the shared wifi hotspot, consider adjusting `ROS_DOMAIN_ID` to something different than 0 to avoid interference with other groups. 

### Steps to connect the robot

1. Boot the robot by setting it on its (plugged) dock. Wait until it sings.
2. Start and connect to the wifi hot spot *on the robot*: press buttons 1+2 for a few seconds until it sings and glows blue. Connect your laptop wifi to the robot's when done 
3. Connect to the web service provided by the robot at 192.168.10.1
4. Checks and update if needed the robot namespace and `ROS_DOMAIN_ID`
4. In the Connect tab, enter the Wifi name (e.g., `wifi-f218`) and password that you will use with the robot and the laptop, click Connect. Wait until it sings and glows white again
5. When both laptop and robot are connected to the same wifi, checks that the robots topics, services and action are visible from ROS2 Iron install (either docker with --net=host or native)

## Test and explore the Robot ROS2 API

Go to [Create3 documentation for the ROS2 API](https://iroboteducation.github.io/create3_docs/api/ros2/).

If needed, you can connect back to the robot web service by going to its IP address on the shared wifi hotspot.

## Adapt and run the `draw_square` action on the robot

Should be as simple as undocking the robot and running the `draw_square` action with remapping the `cmd_vel` topic to that of the robot.