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

## Configuration

TODO: Add complete reference to parameters

### General configuration parameters

```ini
rate 100.0                              # Control rate in Hz
traj_duration 5.0                       # Default trajectory duration
rpc_local_port_name /controller/rpc:i   # RPC command port
position_error_th 0.0005               # Position convergence threshold
max_iteration 10000                     # Maximum IK iterations
module_logging true                     # Enable logging
module_verbose true                     # Enable verbose output
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

The controllers work in the robot's base coordinate frame:
- **X-axis**: Forward direction of the robot
- **Y-axis**: Left direction of the robot  
- **Z-axis**: Upward direction
- **Origin**: Typically at the robot's base

For arm-specific controllers:
- **Right arm**: Negative Y values are towards the robot's right side
- **Left arm**: Positive Y values are towards the robot's left side

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


