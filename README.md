# robot_hardware_interface

<p align="center">
	<a href="https://github.com/hexfellow/robot_hardware_interface/stargazers"><img src="https://img.shields.io/github/stars/hexfellow/robot_hardware_interface?colorA=363a4f&colorB=b7bdf8&style=for-the-badge"></a>
	<a href="https://github.com/hexfellow/robot_hardware_interface/issues"><img src="https://img.shields.io/github/issues/hexfellow/robot_hardware_interface?colorA=363a4f&colorB=f5a97f&style=for-the-badge"></a>
	<a href="https://github.com/hexfellow/robot_hardware_interface/contributors"><img src="https://img.shields.io/github/contributors/hexfellow/robot_hardware_interface?colorA=363a4f&colorB=a6da95&style=for-the-badge"></a>
</p>

## <a name="overview"></a> **Overview**
This is a ROS package that provides ROS interfaces for hex series device SDKs (compatible with both ROS1 and ROS2). The supported hardware list is as follows:
- [✅] **[hex_vehicle](#hex_vehicle)**
- [ ] **[hex_lift](#hex_lift)**
- [ ] **[hex_arm](#hex_arm)**

## <a name="hex_vehicle"></a> For vehicle <small><sup>[overview ▲](#overview)</sup></small>

### Chassis Translation Module (`chassis_trans.py`)

The `chassis_trans.py` module provides a ROS interface for vehicle chassis control using the hex_vehicle SDK. It serves as a bridge between ROS topics and the underlying vehicle hardware API.

#### Features

- **Dual Control Modes**:
  - **Simple Control**: Direct vehicle velocity control (x, y, yaw)
  - **Complex Control**: Individual motor velocity control
- **Real-time Data Publishing**: Motor states (position, velocity, torque) and vehicle velocity
- **Error Monitoring**: Continuous motor error detection and logging
- **WebSocket Communication**: Connects to vehicle hardware via WebSocket URL

#### Installation & Usage

**Prerequisites:**
1. Install the [hex_vehicle_python_lib](https://github.com/hexfellow/hex_vehicle_python_lib) library.

**Build:**
1. Navigate to your workspace and build the package:
   - For ROS1: `catkin_make`
   - For ROS2: `colcon build`

**Launch:**
Use the provided launch files to start the chassis interface:

**ROS1:**
```bash
roslaunch robot_hardware_interface chassis_bringup.launch
```

**ROS2:**
```bash
ros2 launch robot_hardware_interface chassis_bringup.launch.py
```

#### ROS Interface

**Published Topics:**
| Topic Name       | Message Type                    | Description |
| ---------------- | ------------------------------- | ----------- |
| `/motor_states`  | `sensor_msgs/JointState`        | Motor states read from WebSocket: position range [-3.14rad ~ 3.14rad], velocity (rad/s), effort (Nm) |
| `/real_vel`      | `geometry_msgs/TwistStamped`    | Vehicle velocity feedback from WebSocket: linear (m/s) and angular (rad/s) velocities |

**Subscribed Topics:**
| Topic Name       | Message Type                    | Description |
| ---------------- | ------------------------------- | ----------- |
| `/joint_ctrl`    | `sensor_msgs/JointState`        | Individual motor velocity commands for complex control mode |
| `/cmd_vel`       | `geometry_msgs/TwistStamped`    | Vehicle velocity commands for simple control mode (x, y, yaw) |

**Parameters:**
| Parameter Name | Data Type | Default Value            | Description |
| -------------- | --------- | ------------------------ | ----------- |
| `rate_ros`     | `uint`    | `300`                    | ROS topic publishing frequency (Hz) |
| `rate_state`   | `uint`    | `200`                    | Hardware state reading frequency (Hz) |
| `frame_id`     | `str`     | `base_link`              | Reference frame for published topics |
| `simple_mode`  | `bool`    | `true`                   | Enable simple control mode (vehicle-level commands) |
| `ws_url`       | `str`     | `ws://127.0.0.1:8439`   | WebSocket URL for hardware communication |

#### Parameter Configuration

You can configure parameters in two ways:

1. **Static Configuration**: Modify the launch file directly
2. **Runtime Arguments**: Pass parameters when launching

**Example with runtime arguments:**
```bash
ros2 launch robot_hardware_interface chassis_bringup.launch.py ws_url:="ws://172.18.23.92:8439" simple_mode:=true
```

## <a name="hex_lift"></a> For lift <small><sup>[overview ▲](#overview)</sup></small>
waiting...

## <a name="hex_arm"></a> For Arm <small><sup>[overview ▲](#overview)</sup></small>
waiting...


<p align="center">
	Copyright &copy; 2025-present <a href="https://github.com/hexfellow" target="_blank">Hexfellow Org</a>
</p>

<p align="center">
	<a href="https://github.com/hexfellow/robot_hardware_interface/blob/main/LICENSE"><img src="https://img.shields.io/static/v1.svg?style=for-the-badge&label=License&message=Apache&logoColor=d9e0ee&colorA=363a4f&colorB=b7bdf8"/></a>
</p>