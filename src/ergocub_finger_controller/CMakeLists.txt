cmake_minimum_required(VERSION 3.16)

set(EXE_TARGET_NAME ergocub-finger-controller)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(YARP 3.8.0 REQUIRED COMPONENTS os sig dev)

# Include directories
include_directories(include ${YARP_INCLUDE_DIRS})

# Define the executable
add_executable(${EXE_TARGET_NAME}
    main.cpp
    module.cpp
)

# Target include directories
target_include_directories(${EXE_TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link libraries
target_link_libraries(${EXE_TARGET_NAME}
    ${YARP_LIBRARIES}
)

# Set target properties
set_target_properties(${EXE_TARGET_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Install the executable
install(TARGETS ${EXE_TARGET_NAME}
        RUNTIME DESTINATION bin
)

# Install all configuration files
file(GLOB CONFIG_FILES ${CMAKE_CURRENT_SOURCE_DIR}/config/*.ini)
install(FILES ${CONFIG_FILES}
        DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/ergocub_finger_controller)
