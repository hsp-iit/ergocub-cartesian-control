set(LIBRARY_TARGET_NAME cub-joint-control)

# YARP
find_package(YARP REQUIRED COMPONENTS dev os)

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
set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES 
    VERSION       ${${PROJECT_NAME}_VERSION}
    PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_HDR}"
    EXPORT_NAME   cub-joint-control  # Set consistent export name
)

# Include directories
target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

# Linker configuration
target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC YARP::YARP_os YARP::YARP_sig Eigen3::Eigen 
                                             PRIVATE utils)

             
# Install the headers explicitly to ensure they're available
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h"
)
                                             

# Export the target to a different export set than the one used for installation
install(TARGETS ${LIBRARY_TARGET_NAME}
    EXPORT  ${PROJECT_NAME}Targets  # Use the parent project's export name
    LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                          COMPONENT shlib
    ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                          COMPONENT lib
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}" COMPONENT dev
)
