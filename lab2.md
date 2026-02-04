# Lab 2: Docker & DDS Debugging

## 1. ROS2 in Docker

### Running Official Images
```bash
docker run -it --rm --net=host ros:jazzy
```
Note: `--net=host` is required for ROS2 discovery across containers/host without complex configuration.

### Multi-Container Communication
Create a bridge network:
```bash
docker network create ros_net
docker run -it --rm --name talker --net ros_net ros:jazzy ros2 run demo_nodes_cpp talker
docker run -it --rm --name listener --net ros_net ros:jazzy ros2 run demo_nodes_py listener
```

## 2. DDS Under the Hood

ROS2 uses DDS (Data Distribution Service) for discovery and data transport.
Key concepts:
- **Participant**: A process in the DDS network.
- **Topic**: The data channel.
- **QoS (Quality of Service)**: Reliability, Durability, History, Liveliness.

### Discovery Mechanism
DDS uses **Simple Discovery Protocol** (SDP) via multicast (default 239.255.0.1).

### Troubleshooting Discovery
If nodes don't see each other:
1. Check `ROS_DOMAIN_ID`:
   ```bash
   echo $ROS_DOMAIN_ID
   ```
2. Check Multicast:
   ```bash
   ros2 doctor --report
   ```

## 3. DDS Debugging & Introspection

### ros2 doctor
Check system health and middleware configuration:
```bash
ros2 doctor
ros2 doctor --report
```

### DDS GUI Tools: PlotJuggler
While CLI tools are powerful, GUI tools help visualize traffic and QoS.
**PlotJuggler** is commonly used for ROS2 data visualization.
```bash
sudo apt install ros-jazzy-plotjuggler-ros
ros2 run plotjuggler plotjuggler
```
Select the **ROS2 Topic Re-publisher** plugin to start streaming data.

### RMW (ROS Middleware) Implementation
You can switch the underlying DDS implementation without changing your code.
Example: Switching to CycloneDDS:
```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 4. QoS (Quality of Service) Demos

QoS settings control how data is delivered. Mismatched QoS between a publisher and subscriber will prevent communication.

### Illustrating Reliability (Best Effort vs Reliable)
Use `ros2 topic pub` with specific QoS profiles.

**Scenario: Simulated Packet Loss**
In a constrained network (or simulated via `tc`), a **Best Effort** subscriber will drop messages, while a **Reliable** one will retry.

### Illustrating History & Depth (Queue Saturation)
If a publisher is much faster than a subscriber:
- **Keep Last (Depth: 1)**: The subscriber only ever gets the most recent message.
- **Keep All**: The system might consume significant memory or slow down to ensure delivery.

**Demo Command:**
You can use the `demo_nodes_cpp` specialized for QoS:
```bash
# Publisher with specific QoS
ros2 run demo_nodes_cpp talker --ros-args -p reliability:=best_effort

# Subscriber with incompatible QoS (will fail to receive)
ros2 run demo_nodes_cpp listener --ros-args -p reliability:=reliable
```
Check why it fails using `--verbose`:
```bash
ros2 topic info /chatter --verbose
```

## 5. Network Configuration

### xml Configuration
DDS can be configured via XML files for non-multicast environments (e.g., cloud, restrictive subnets).
Example `FASTRTPS_DEFAULT_PROFILES_FILE` or `CYCLONEDDS_URI`.
