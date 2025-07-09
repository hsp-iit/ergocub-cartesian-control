# Ergocub Cartesian Control

A collection of controllers for the ergoCub and the R1 (R1SN003) robot.

## Installation

### Docker

```
docker build -t hsp/ergocub-cartesian-control:latest .
```

### Source

We recommend installing the libraries directly on the robot because the control motion may be sensitive to internet lag.

#### Dependencies

- `robotology-superbuild` latest build

#### Instructions

We recommend installing the repository in the superbuild install directory, but of course you can choose based on your needs

```
git clone https://github.com/hsp-iit/ergocub-cartesian-control.git
cd ergocub-cartesian-control
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=<your-path-to-robotology-superbuild>/build/install
make install
```

## Use this project

This project provides two executables, `r1-cartesian-control` and `ergocub-cartesian-control` that directly control the robot arm. You should use `r1-cartesian-control` for R1 and `ergocub-cartesian-control` for ergoCub. More details regarding how to use the controllers can be found in [Control Interface](ControlInterface.md)

### CMake

This project provides CMake configuration files that allow easy integration into other CMake-based projects. After installation, you can use `find_package()` to locate and link against the provided libraries.

#### Basic Integration

To use this project in your CMake-based project, add the following to your `CMakeLists.txt`:

```cmake
find_package(ergocub-cartesian-control REQUIRED)

# Link against specific components
target_link_libraries(your_target 
    ergocub-cartesian-control::utils
    ergocub-cartesian-control::cub-joint-control
    ergocub-cartesian-control::trajectory-generator
)
```

#### Available Components

The project provides the following CMake components that can be used independently:

- **`utils`**: Utility functions and helper classes
- **`cub-joint-control`**: Joint position control for iCub, ergoCub and R1 robots
- **`trajectory-generator`**: Library for generating robot trajectories
- **`gb-ergocub-cartesian-service`**: Thrift interface definitions for the controllers

#### Component-based Usage

You can specify only the components you need:

```cmake
find_package(ergocub-cartesian-control REQUIRED COMPONENTS utils trajectory-generator)

target_link_libraries(your_target 
    ergocub-cartesian-control::utils
    ergocub-cartesian-control::trajectory-generator
)
```

#### Dependencies

The project automatically handles its dependencies when found via CMake:
- YARP (dev, os, sig components)
- Eigen3

Make sure these dependencies are available in your system or specify their paths if installed in non-standard locations.

#### Example Integration

Here's a complete example of how to integrate this project into your CMake project:

```cmake
cmake_minimum_required(VERSION 3.20)
project(my_robot_project)

# Find the ergocub-cartesian-control package
find_package(ergocub-cartesian-control REQUIRED COMPONENTS utils cub-joint-control)

# Create your executable
add_executable(my_robot_app src/main.cpp)

# Link against the required components
target_link_libraries(my_robot_app 
    ergocub-cartesian-control::utils
    ergocub-cartesian-control::cub-joint-control
)
```

#### Installation Path Configuration

If you installed the project to a custom location, you may need to help CMake find it:

```cmake
# Option 1: Set CMAKE_PREFIX_PATH
set(CMAKE_PREFIX_PATH "/path/to/your/install/prefix" ${CMAKE_PREFIX_PATH})
find_package(ergocub-cartesian-control REQUIRED)

# Option 2: Set the package-specific path
set(ergocub-cartesian-control_DIR "/path/to/your/install/prefix/lib/cmake/ergocub-cartesian-control")
find_package(ergocub-cartesian-control REQUIRED)
```

## Project layout

    src/
        cub_joint_control  # Source code for control in joint position for iCub, ergoCub and R1.
        r1_cartesian_control # Current controller used by r1 
        ergocub_cartesian_control # Current controller used by ergoCub
        ergocub_cartesian_service # Thrift interface definition for the controllers
        trajectory_generator # Library to generate trajectories
        utils # Library that provides utility functions


# Another section