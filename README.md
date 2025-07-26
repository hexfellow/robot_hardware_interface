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

### Chassis Key Control Module (`chassis_key_control.py`)
This module provides a method to quickly publish cmd_vel messages via the keyboard.
It can only be invoked using the `rosrun` or `ros2` run commands.
**ROS1:**
```bash
rosrun robot_hardware_interface chassis_key_control.py
```
**ROS2:**
```bash
ros2 run robot_hardware_interface chassis_key_control
```

#### ROS Interface

**Published Topics:**
| Topic Name       | Message Type                    | Description |
| ---------------- | ------------------------------- | ----------- |
| `/cmd_vel`  | `geometry_msgs/Twist`        | the speed command for control chassis. |

#### Parameter Configuration
**Parameters:**
| Parameter Name | Data Type | Default Value | Description |
| -------------- | --------- | ------------- | ----------- |
| `speed` | `float` | `0.5` | Default linear velocity scale factor |
| `turn` | `float` | `1.0` | Default angular velocity scale factor |
| `speed_limit` | `float` | `4.0` | Maximum linear velocity limit |
| `turn_limit` | `float` | `1000.0` | Maximum angular velocity limit |
| `repeat_rate` | `float` | `100.0` | Publishing frequency in Hz |
| `key_timeout` | `float` | `0.2` | Keyboard input timeout in seconds |
| `stamped` | `bool` | `false` | Use TwistStamped messages if true |
| `frame_id` | `string` | `''` | Frame ID for stamped messages |

Quickly setting example:
```bash
rosrun robot_hardware_interface chassis_key_control.py _speed:=0.9 _turn:=0.8
```
```bash
ros2 run robot_hardware_interface chassis_key_control --ros-args -p repeat_rate:=1.0
```

#### Keyboard Controls
**Movement Keys:**
| Key | Action | Description |
|-----|--------|-------------|
| `i` | Forward | Move robot forward |
| `,` | Backward | Move robot backward |
| `j` | Turn Left | Rotate robot counterclockwise |
| `l` | Turn Right | Rotate robot clockwise |
| `u` | Forward + Left | Move forward while turning left |
| `o` | Forward + Right | Move forward while turning right |
| `m` | Backward + Left | Move backward while turning left |
| `.` | Backward + Right | Move backward while turning right |
| `k` | Stop | Stop all movement |

**Speed Control:**
| Key | Action | Description |
|-----|--------|-------------|
| `q` | Increase All Speeds | Increase both linear and angular speeds by 10% |
| `z` | Decrease All Speeds | Decrease both linear and angular speeds by 10% |
| `w` | Increase Linear Speed | Increase only linear speed by 10% |
| `x` | Decrease Linear Speed | Decrease only linear speed by 10% |
| `e` | Increase Angular Speed | Increase only angular speed by 10% |
| `c` | Decrease Angular Speed | Decrease only angular speed by 10% |

**Exit:**
| Key | Action | Description |
|-----|--------|-------------|
| `Ctrl+C` | Quit | Terminate the program safely |


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