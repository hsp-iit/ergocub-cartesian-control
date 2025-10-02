set(EXE_TARGET_NAME mc-ergocub-head-controller)

# Eigen3
find_package(Eigen3 REQUIRED)

# YARP
find_package(YARP REQUIRED COMPONENTS
                           dev
                           eigen
                           os
                           sig
)

# iDynTree
find_package(iDynTree REQUIRED)

# ProxSuite
find_package(proxsuite REQUIRED)

# Executable
add_executable(${EXE_TARGET_NAME}
                include/inverseKinematicsParameters.h
                include/inverseKinematics.h
                include/forwardKinematics.h
                include/io.h
                include/module.h
                src/inverseKinematicsParameters.cpp
                src/inverseKinematics.cpp
                src/forwardKinematics.cpp
                src/io.cpp
                src/main.cpp
                src/module.cpp
                src/module_service.cpp
)

target_include_directories(${EXE_TARGET_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${EXE_TARGET_NAME} YARP::YARP_init
                                         YARP::YARP_dev
                                         YARP::YARP_eigen
                                         YARP::YARP_os
                                         YARP::YARP_sig
                                         Eigen3::Eigen
                                         proxsuite::proxsuite
                                         mc-ergocub-head-service
                                         ${iDynTree_LIBRARIES}
                                         trajectory-generator
)

# Install the executable
install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

# Install configuration and application files
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/config.ini DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/${EXE_TARGET_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/config_sim.ini DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/${EXE_TARGET_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/common.ini DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/${EXE_TARGET_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/ik_parameters.ini DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/${EXE_TARGET_NAME})
