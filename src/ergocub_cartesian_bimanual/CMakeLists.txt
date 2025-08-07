set(EXE_TARGET_NAME mc-ergocub-cartesian-bimanual)

# YARP
find_package(YARP REQUIRED COMPONENTS
                           dev
                           os
                           sig
                           idl_tools
                           eigen
)

# Eigen3
find_package(Eigen3 REQUIRED)

# iDynTree
find_package(iDynTree REQUIRED)

# ProxSuite
find_package(proxsuite REQUIRED)

# BipedalLocomotionFramework (for logging)
find_package(BipedalLocomotionFramework 0.18.0
  COMPONENTS  VectorsCollection
              ParametersHandlerYarpImplementation REQUIRED)

# Executable
add_executable(${EXE_TARGET_NAME}
                include/DifferentialInverseKinematicsQP.h
                include/Integrator.h
                include/ForwardKinematics.h
                include/ForwardKinematicsiDynTree.h
                include/module.h
                src/DifferentialInverseKinematicsQP.cpp
                src/Integrator.cpp
                src/ForwardKinematicsiDynTree.cpp
                src/module.cpp
                src/module_service.cpp
                src/main.cpp
)

target_include_directories(${EXE_TARGET_NAME} PRIVATE
                                              ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${EXE_TARGET_NAME} PRIVATE
                                         YARP::YARP_dev
                                         YARP::YARP_init
                                         YARP::YARP_os
                                         YARP::YARP_sig
                                         YARP::YARP_eigen
                                         Eigen3::Eigen 
                                         ${iDynTree_LIBRARIES}
                                         proxsuite::proxsuite
                                         cub-joint-control
                                         mc-ergocub-bimanual-service
                                         utils
                                         BipedalLocomotion::VectorsCollection
                                         BipedalLocomotion::ParametersHandlerYarpImplementation
)

# Install the executable
install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)


# Install configuration and application files
add_subdirectory(app)
