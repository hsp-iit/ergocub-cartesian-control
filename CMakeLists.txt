cmake_minimum_required(VERSION 3.20)

project(ergocub-cartesian-control
        LANGUAGES CXX
        VERSION 0.0.1)

include(GNUInstallDirs)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
            message(STATUS "Setting build type to 'Release' as none was specified.")
            set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

# Add common libraries cub_joint_control and utils
add_subdirectory(src/cub_joint_control)
add_subdirectory(src/utils)
add_subdirectory(src/trajectory_generator)

# Add thrift service to define rpc interface
add_subdirectory(src/ergocub_cartesian_service)

# Add module
add_subdirectory(src/ergocub_cartesian_control)


# Add standard uninstall target
include(AddUninstallTarget)