set(LIBRARY_TARGET_NAME cub-joint-control)

# YARP
find_package(YARP REQUIRED COMPONENTS idl_tools sig)

# Eigen3
find_package(Eigen3 REQUIRED)

set(${LIBRARY_TARGET_NAME}_HDR
    include/cub-joint-control/jointControl.h
    include/cub-joint-control/cubJointControl.h
)

set(${LIBRARY_TARGET_NAME}_SRC
    src/cubJointControl.cpp
)

# Add library
add_library(${LIBRARY_TARGET_NAME} STATIC ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})

# Library properties
set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES VERSION       ${${PROJECT_NAME}_VERSION}
                                                        PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_HDR}")

# Include directories
target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

# Linker configuration
target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC YARP::YARP_os YARP::YARP_sig Eigen3::Eigen utils)
