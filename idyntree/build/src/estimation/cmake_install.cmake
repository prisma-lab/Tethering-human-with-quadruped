# Install script for directory: /home/jcacace/quad_ws/src/idyntree/src/estimation

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xshlibx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-estimation.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-estimation.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-estimation.so"
         RPATH "$ORIGIN/:$ORIGIN/../lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/jcacace/quad_ws/src/idyntree/build/lib/libidyntree-estimation.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-estimation.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-estimation.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-estimation.so"
         OLD_RPATH "/home/jcacace/quad_ws/src/idyntree/build/lib:"
         NEW_RPATH "$ORIGIN/:$ORIGIN/../lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-estimation.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xshlibx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xruntimex" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/iDynTree/Estimation" TYPE FILE FILES
    "/home/jcacace/quad_ws/src/idyntree/src/estimation/include/iDynTree/Estimation/BerdyHelper.h"
    "/home/jcacace/quad_ws/src/idyntree/src/estimation/include/iDynTree/Estimation/ExternalWrenchesEstimation.h"
    "/home/jcacace/quad_ws/src/idyntree/src/estimation/include/iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h"
    "/home/jcacace/quad_ws/src/idyntree/src/estimation/include/iDynTree/Estimation/SimpleLeggedOdometry.h"
    "/home/jcacace/quad_ws/src/idyntree/src/estimation/include/iDynTree/Estimation/BerdySparseMAPSolver.h"
    "/home/jcacace/quad_ws/src/idyntree/src/estimation/include/iDynTree/Estimation/SchmittTrigger.h"
    "/home/jcacace/quad_ws/src/idyntree/src/estimation/include/iDynTree/Estimation/ContactStateMachine.h"
    "/home/jcacace/quad_ws/src/idyntree/src/estimation/include/iDynTree/Estimation/BipedFootContactClassifier.h"
    "/home/jcacace/quad_ws/src/idyntree/src/estimation/include/iDynTree/Estimation/GravityCompensationHelpers.h"
    "/home/jcacace/quad_ws/src/idyntree/src/estimation/include/iDynTree/Estimation/ExtendedKalmanFilter.h"
    "/home/jcacace/quad_ws/src/idyntree/src/estimation/include/iDynTree/Estimation/AttitudeEstimator.h"
    "/home/jcacace/quad_ws/src/idyntree/src/estimation/include/iDynTree/Estimation/AttitudeMahonyFilter.h"
    "/home/jcacace/quad_ws/src/idyntree/src/estimation/include/iDynTree/Estimation/AttitudeQuaternionEKF.h"
    "/home/jcacace/quad_ws/src/idyntree/src/estimation/include/iDynTree/Estimation/KalmanFilter.h"
    )
endif()

