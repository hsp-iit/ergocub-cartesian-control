set(LIBRARY_TARGET_NAME gb-ergocub-cartesian-service)

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
set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES VERSION       ${${PROJECT_NAME}_VERSION}
                                                        PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_HDR}")

# Include directories
target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

# Linker configuration
target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC YARP::YARP_os YARP::YARP_sig)

# Specify installation targets, type and destination folders.
install(TARGETS ${LIBRARY_TARGET_NAME}
        EXPORT  ${LIBRARY_TARGET_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                   COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                   COMPONENT lib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}"        COMPONENT dev
)
set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_TARGETS ${LIBRARY_TARGET_NAME})

# Install the library
set(DEPENDENCIES "YARP COMPONENTS os sig")
install_basic_package_files(${LIBRARY_TARGET_NAME}
                            VERSION ${${PROJECT_NAME}_VERSION}
                            COMPATIBILITY ExactVersion
                            EXPORT ${LIBRARY_TARGET_NAME}
                            NO_SET_AND_CHECK_MACRO
                            VARS_PREFIX ${LIBRARY_TARGET_NAME}
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO
                            DEPENDENCIES ${DEPENDENCIES})