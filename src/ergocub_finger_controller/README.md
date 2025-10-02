# ErgoCub Finger Controller

A simple YARP-based controller for managing finger joint positions on the ergoCub robot.

## Overview

This controller reads finger joint target positions from a YARP port and applies them to the robot's finger encoders. It's designed to be lightweight and easy to integrate with other control systems.

## Features

- Reads finger joint commands via YARP port
- Applies position control to finger joints
- Configurable control period
- Support for both hands (18 finger joints total)
- Real-time encoder feedback

## Usage

### Running the Controller

```bash
ergocub_finger_controller --from ergocub_finger_controller.ini
```

### Configuration

The controller uses a configuration file (`ergocub_finger_controller.ini`) with the following parameters:

- `robot`: Robot name (default: "ergoCub")
- `name`: Controller name for port naming (default: "/ergocub_finger_controller")
- `period`: Control loop period in seconds (default: 0.01)
- `finger_joints`: List of finger joint indices to control

### Command Interface

Send finger commands via the YARP port `/ergocub_finger_controller/finger_commands:i`.

Commands should be sent as YARP bottles containing the target positions for all finger joints in radians:

```bash
# Example: Send finger positions (18 values for all finger joints)
echo "0.0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8" | yarp write ... /ergocub_finger_controller/finger_commands:i
```

### Python Interface Example

```python
import yarp

# Initialize YARP
yarp.Network.init()

# Create port
port = yarp.BufferedPortBottle()
port.open("/finger_commander")

# Connect to controller
yarp.Network.connect("/finger_commander", "/ergocub_finger_controller/finger_commands:i")

# Send command
bottle = port.prepare()
bottle.clear()
for i in range(18):  # 18 finger joints
    bottle.addFloat64(0.1 * i)  # Example positions
port.write()

# Cleanup
port.close()
yarp.Network.fini()
```

## Joint Mapping

The controller assumes the following joint indices for finger control:

### Left Hand (9 joints)
- Joints 7-15: Thumb, index, middle, ring, little finger joints

### Right Hand (9 joints) 
- Joints 23-31: Thumb, index, middle, ring, little finger joints

**Note**: Adjust the `finger_joints` parameter in the configuration file to match your specific robot setup.

## Dependencies

- YARP (>= 3.8.0)
- CMake (>= 3.16)
- C++17 compatible compiler

## Building

```bash
mkdir build && cd build
cmake ..
make -j4
```

## Installation

```bash
make install
```

This will install:
- The executable to `bin/ergocub_finger_controller`
- Configuration files to `share/ergocub_finger_controller/`

## Troubleshooting

### Connection Issues
- Ensure YARP server is running (`yarp server`)
- Check that the robot's controlboard is accessible
- Verify port names match the configuration

### Control Issues
- Ensure finger joints are in position control mode
- Check joint index configuration matches your robot
- Verify target positions are within joint limits
