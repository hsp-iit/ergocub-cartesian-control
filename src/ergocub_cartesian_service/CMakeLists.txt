# SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
# SPDX-License-Identifier: BSD-3-Clause

set(LIBRARY_TARGET_NAME gb-ergocub-cartesian-service)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Build a shared library
option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

# Include helpers
include(AddInstallRPATHSupport)
include(InstallBasicPackageFiles)

# Enable RPATH
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

# YARP
find_package(YARP REQUIRED COMPONENTS idl_tools sig)

# Generate source files from thrift
yarp_idl_to_dir(INPUT_FILES ${LIBRARY_TARGET_NAME}/service.thrift
                OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}
                SEPARATE
)

set(${LIBRARY_TARGET_NAME}_HDR
include/${LIBRARY_TARGET_NAME}/ergoCubCartesianService.h
include/${LIBRARY_TARGET_NAME}/ReachabilityEvaluationState.h
)

set(${LIBRARY_TARGET_NAME}_SRC
    src/ergoCubCartesianService.cpp
    src/ReachabilityEvaluationState.cpp
)

# Add library
add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})

# Library properties
set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES
    PUBLIC_HEADER "${headers}"
    EXPORT_NAME ${LIBRARY_TARGET_NAME}  # Set consistent export name
)

# Include directories
target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

# Linker configuration
target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC YARP::YARP_os YARP::YARP_sig)

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
