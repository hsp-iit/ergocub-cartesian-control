# Control Interface

This document describes how to use the cartesian controllers provided by this project. The controllers allow you to command the robot arms to reach desired poses in Cartesian space using high-level commands.

## Overview

The project provides two main executables:

- **`ergocub-cartesian-control`**: For controlling ergoCub robots
- **`r1-cartesian-control`**: For controlling R1 (R1SN003)

Both controllers provide the same interface through YARP RPC commands and implement a Thrift service for easy integration.

## Starting the Controllers

The user can start the controllers using the provided configurations

### ergoCub Controller

```bash
# For right arm
ergocub-cartesian-control --from config_right.ini

# For left arm  
ergocub-cartesian-control --from config_left.ini

# For simulation (right arm)
ergocub-cartesian-control --from config_right_sim.ini

# For arms without torso control
ergocub-cartesian-control --from config_right_no_torso.ini
```

### R1 Controller

```bash
# For right arm
r1-cartesian-control --from config_right.ini

# For left arm
r1-cartesian-control --from config_left.ini

# For simulation (right_arm)
r1-cartesian-control --from config_right_sim_r1.ini
```

## RPC Interface

The controllers expose an RPC interface for sending commands. You can connect to the RPC port and send commands directly.

### Connecting to the Controller

```bash
# Connect to the controller's RPC port
yarp rpc /gb-ergocub-cartesian-control/right_arm/rpc:i
```

### Available Commands

#### 1. Go to Pose
Move the end-effector to a specific pose (position + orientation):

```
go_to_pose <x> <y> <z> <qx> <qy> <qz> <qw> <duration>
```

- `x, y, z`: Target position in meters (relative to robot's base frame)
- `qx, qy, qz, qw`: Target orientation as quaternion
- `duration`: Trajectory duration in seconds

**Example:**
```
go_to_pose 0.3 -0.2 0.1 0.0 0.0 0.0 1.0 5.0
```

#### 2. Go to Position
Move the end-effector to a specific position (keeping current orientation):

```
go_to_position <x> <y> <z> <duration>
```

**Example:**
```
go_to_position 0.4 -0.15 0.2 3.0
```

#### 3. Rotate Around Axis
Rotate the end-effector around a specified axis:

```
rotate_rad <angle> <axis_x> <axis_y> <axis_z> <duration>
rotate_deg <angle> <axis_x> <axis_y> <axis_z> <duration>
```

- `angle`: Rotation angle (radians for `rotate_rad`, degrees for `rotate_deg`)
- `axis_x, axis_y, axis_z`: Rotation axis (unit vector)

**Example:**
```
rotate_deg 45.0 0.0 0.0 1.0 4.0  # Rotate 45° around Z-axis
```

#### 4. Go Home
Return to the predefined home position:

```
go_home
```

#### 5. Get Current Pose
Retrieve the current end-effector pose:

```
get_pose
```

Returns a 4x4 homogeneous transformation matrix.

#### 6. Motion Status
Check if the current motion is completed:

```
is_motion_done
```

Returns `true` if the motion is finished, `false` otherwise.

#### 7. Stop Motion
Stop the current motion:

```
stop
```

#### 8. Reachability Analysis
Check if a pose is reachable by the robot:

```
is_pose_reachable <x> <y> <z> <qx> <qy> <qz> <qw>
ask_reachability_evaluation <pose_matrix>
retrieve_reachable_pose
```

## Thrift Service Interface

For programmatic access, the controllers implement a Thrift service with the same functionality. This allows easy integration from C++, Python, or other languages.

### C++ Example

```cpp
#include <gb-ergocub-cartesian-service/ergoCubCartesianService.h>
#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>

// Connect to the service
yarp::os::Network yarp;
yarp::os::RpcClient client;
client.open("/client");
yarp.connect("/client", "/gb-ergocub-cartesian-control/right_arm/rpc:i");

// Create service proxy
ergoCubCartesianService service;
service.yarp().attachAsClient(client);

// Send commands
bool success = service.go_to_position(0.3, -0.2, 0.1, 5.0);
```

### Python Example

```python
import yarp

# Initialize YARP
yarp.Network.init()

# Connect to service
client = yarp.RpcClient()
client.open("/python_client")
yarp.Network.connect("/python_client", "/gb-ergocub-cartesian-control/right_arm/rpc:i")

# Send command
cmd = yarp.Bottle()
cmd.addString("go_to_position")
cmd.addFloat64(0.3)
cmd.addFloat64(-0.2) 
cmd.addFloat64(0.1)
cmd.addFloat64(5.0)

reply = yarp.Bottle()
client.write(cmd, reply)
```

## Coordinate Frames

The controllers work in the robot's coordinate frame. Tipically they are directed in the following way:

- **X-axis**: Forward direction of the robot
- **Y-axis**: Left direction of the robot  
- **Z-axis**: Upward direction
- **Origin**: `root_link` for ergoCub and `mobile_base_body_link` for R1


## Configuration

### General Configuration Parameters

#### Control Parameters
- **`rate`** (default: `100.0`): Control loop frequency in Hz. Higher values provide smoother control but require more computational resources.
- **`traj_duration`** (default: `5.0`): Default trajectory duration in seconds when not specified in commands.

#### Communication
- **`rpc_local_port_name`** (e.g: `/gb-ergocub-cartesian-control/right_arm/rpc:i`): Local YARP port name for RPC communication with the controller.

#### Convergence and Limits
- **`position_error_th`** (default: `0.0005`): Position convergence threshold in meters. The controller considers a motion complete when the position error is below this value.
- **`max_iteration`** (default: `10000`): Maximum number of iterations for the inverse kinematics solver before giving up.

#### Debugging and Logging
- **`module_logging`** (default: `true`): Enable data logging for analysis and debugging.
- **`module_verbose`** (default: `true`): Enable verbose console output for debugging purposes.
- **`qp_verbose`** (default: `false`): Enable verbose console output for the QP solver.

### ARM Section Parameters

#### Robot Configuration
- **`name`** (example: `left_arm`, `right_arm`): Identifier name for the arm being controlled.
- **`joint_axes_list`** (example: `(torso_yaw_joint l_shoulder_pitch l_shoulder_roll l_shoulder_yaw l_elbow l_wrist_yaw l_wrist_roll l_wrist_pitch)`): Complete ordered list of joint names to control. Order matters for the control algorithm.
- **`joint_ports_list`** (example: `(/cer/torso /cer/left_arm)`): List of YARP port names for communicating with the robot's joint interfaces.
- **`joint_local_port`** (example: `/r1-cartesian-control/left_arm`): Local YARP port prefix for this controller instance.

### IK_PARAM Section Parameters

#### Joint Limits and Constraints
- **`limits_param`** (default: `0.90`): Safety factor for joint limits (0 < value < 1). Reduces effective joint range to avoid hitting limits.
- **`max_joint_position_variation`** (default: `25.0`): Maximum allowed joint position change per control cycle in degrees.
- **`max_joint_position_track_error`** (default: `1.5`): Maximum acceptable tracking error between desired and actual joint positions in degrees.

#### Control Weights and Gains
- **`joint_vel_weight`** (default: `(5.0 0.0)`): Joint velocity cost weight and offset for the QP solver `(weight, offset)`.
- **`position_param`** (default: `(20.0 0.5)`): Position control parameters `(cost_weight, proportional_gain)`.
- **`orientation_param`** (default: `(10.0 0.5)`): Orientation control parameters `(cost_weight, proportional_gain)`.
- **`joint_pos_param`** (default: `(2.5 10.0 5.0)`): Joint position control gains `(cost_weight, proportional_gain, derivative_gain)`.

#### Reference Configuration
- **`joint_ref`** (example: `(0.0 0.0 0.1 0.0 0.1 0.0 0.0 0.0)`): Reference joint configuration used for null-space projection. Must match the number and order of joints in `joint_axes_list`.

### FK_PARAM Section Parameters

#### Kinematic Chain Definition
- **`root_frame_name`** (default: `mobile_base_body_link` for R1, `root_link` for ergoCub): Name of the root frame for the kinematic chain.
- **`ee_frame_name`** (example: `l_hand_palm`, `r_hand_palm`): Name of the end-effector frame that will be controlled.

### FSM_PARAM Section Parameters

#### Finite State Machine
- **`stop_speed`** (default: `0.001`): Velocity threshold in rad/s for determining when motion has stopped (approximately 0.057 degrees/s).

## Tuning Guidelines

### Performance Tuning
- **Increase `rate`** for smoother control, but monitor CPU usage
- **Adjust `position_param` and `orientation_param` weights** to balance position vs orientation tracking
- **Tune `joint_pos_param`** gains for better joint space control

### Safety Tuning  
- **Reduce `limits_param`** for additional safety margin from joint limits
- **Lower `max_joint_position_variation`** for slower, safer motions
- **Decrease `max_joint_position_track_error`** for stricter tracking requirements



## Safety Considerations

1. **Joint Limits**: The controllers automatically enforce joint position and velocity limits
2. **Collision Avoidance**: No built-in collision avoidance - ensure trajectories are safe
3. **Emergency Stop**: Use the `stop` command to halt motion immediately
4. **Reachability**: Use reachability checks before commanding unreachable poses
5. **Monitoring**: Monitor `is_motion_done` to ensure commands complete successfully

## Troubleshooting

### Common Issues

1. **Controller not responding**: Check YARP network and port connections
2. **Motion not starting or wrong motion**: Verify pose is reachable and within joint limits
3. **Jerky motion**: Increase trajectory duration for smoother movements
4. **High tracking error**: Tune controller gains in configuration file

### Debugging

Enable verbose output in the configuration:
```ini
module_verbose true
module_logging true
```

Check controller status through RPC:
```
is_motion_done
get_pose
```


