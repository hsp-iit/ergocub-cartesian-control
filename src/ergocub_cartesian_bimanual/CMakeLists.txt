cmake_minimum_required(VERSION 4.1)
project(ergocub_bimanual LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EXE_TARGET_NAME mc-ergocub-cartesian-bimanual)

# Option to build the YARP-based executable. Default OFF when building via scikit-build/pip.
option(BUILD_BIMANUAL_EXECUTABLE "Build the mc-ergocub-cartesian-bimanual executable" ON)
if(DEFINED SKBUILD AND SKBUILD)
  set(BUILD_BIMANUAL_EXECUTABLE OFF)
endif()

## Only locate YARP and BLF if we are actually building the executable
if(BUILD_BIMANUAL_EXECUTABLE)
  # YARP
  find_package(YARP REQUIRED COMPONENTS
                             dev
                             os
                             sig
                             idl_tools
                             eigen
  )
endif()

# Eigen3
find_package(Eigen3 REQUIRED)

# iDynTree
find_package(iDynTree REQUIRED)

# ProxSuite
find_package(proxsuite REQUIRED)

if(BUILD_BIMANUAL_EXECUTABLE)
  # BipedalLocomotionFramework (for logging)
  find_package(BipedalLocomotionFramework 0.18.0
    COMPONENTS  VectorsCollection
                ParametersHandlerYarpImplementation REQUIRED)
endif()

# Python bindings 
option(BUILD_PYTHON "Build Python bindings for bimanual IK" ON)
if(BUILD_PYTHON)
  # YARP logs are used inside the core; link YARP OS for the Python module too
  find_package(YARP REQUIRED COMPONENTS os)
  find_package(pybind11 CONFIG REQUIRED)
endif()

# Core library (IK + FK), reused by both the executable and the Python module
add_library(ergocub_cartesian_bimanual_core
  include/DifferentialInverseKinematicsQP.h
  include/Integrator.h
  include/ForwardKinematics.h
  include/ForwardKinematicsiDynTree.h
  src/DifferentialInverseKinematicsQP.cpp
  src/Integrator.cpp
  src/ForwardKinematicsiDynTree.cpp
)

set_target_properties(ergocub_cartesian_bimanual_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(ergocub_cartesian_bimanual_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(ergocub_cartesian_bimanual_core PUBLIC
  Eigen3::Eigen
  ${iDynTree_LIBRARIES}
  proxsuite::proxsuite
)

# Install the core library - REQUIRED for the executable to work!
install(TARGETS ergocub_cartesian_bimanual_core
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

## Executable (guarded)
if(BUILD_BIMANUAL_EXECUTABLE)
  # Ensure required targets from the superproject exist, otherwise skip
  if(NOT (TARGET cub-joint-control AND TARGET mc-ergocub-bimanual-service AND TARGET utils))
    message(STATUS "Skipping ${EXE_TARGET_NAME}: required targets not found (cub-joint-control, mc-ergocub-bimanual-service, utils)")
    set(BUILD_BIMANUAL_EXECUTABLE OFF)
  endif()
endif()

if(BUILD_BIMANUAL_EXECUTABLE)
  add_executable(${EXE_TARGET_NAME}
    include/module.h
    src/module.cpp
    src/module_service.cpp
    src/main.cpp
  )

  target_include_directories(${EXE_TARGET_NAME} PRIVATE
                                                ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(${EXE_TARGET_NAME} PRIVATE
                                           ergocub_cartesian_bimanual_core
                                           YARP::YARP_dev
                                           YARP::YARP_init
                                           YARP::YARP_os
                                           YARP::YARP_sig
                                           YARP::YARP_eigen
                                           cub-joint-control
                                           mc-ergocub-bimanual-service
                                           utils
                                           BipedalLocomotion::VectorsCollection
                                           BipedalLocomotion::ParametersHandlerYarpImplementation
  )

  # YARP logging is always enabled via includes; no special defines needed

  # Install the executable
  install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

  # Install configuration and application files only with the executable
  add_subdirectory(app)
endif()


# Python module (pybind11)
if(BUILD_PYTHON)
  # Bindings source lives in the python package; facade in the py subproject
  # Reuse the original IK core from this target and compile the facade locally
  set(PYBIND_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/python/pybind_module.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/BimanualIK.cpp
  )
  pybind11_add_module(pysquale MODULE ${PYBIND_SRC})
  target_include_directories(pysquale PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(pysquale PRIVATE ergocub_cartesian_bimanual_core YARP::YARP_os)

  # Place the module at the wheel root if built via scikit-build; else libdir
  if(DEFINED SKBUILD AND SKBUILD)
    set(_PY_DEST ".")
  else()
    include(GNUInstallDirs)
    set(_PY_DEST ${CMAKE_INSTALL_LIBDIR})
  endif()
  install(TARGETS pysquale DESTINATION ${_PY_DEST})
endif()
