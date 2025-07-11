# SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
# SPDX-License-Identifier: BSD-3-Clause

set(LIBRARY_TARGET_NAME utils)

# YARP
find_package(YARP REQUIRED COMPONENTS idl_tools sig)

# Eigen3
find_package(Eigen3 REQUIRED)

set(${LIBRARY_TARGET_NAME}_HDR
    include/utils/utils.h
    include/utils/utils.hpp
)

set(${LIBRARY_TARGET_NAME}_SRC
    src/utils.cpp
)

# Add library
add_library(${LIBRARY_TARGET_NAME} STATIC ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})

# Library properties
set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES
    VERSION       ${${PROJECT_NAME}_VERSION}
    PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_HDR}"
    EXPORT_NAME ${LIBRARY_TARGET_NAME}  # Set consistent export name
)


# Include directories
target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

# Linker configuration
target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC YARP::YARP_os YARP::YARP_sig  Eigen3::Eigen)

# Install the headers explicitly to ensure they're available
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h"
)


# Install the targets
install(TARGETS ${LIBRARY_TARGET_NAME}
    EXPORT  ${PROJECT_NAME}Targets  # Use the parent project's export name
    LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}"
)
