# Install script for directory: /home/jcacace/quad_ws/src/idyntree/src/core

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
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-core.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-core.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-core.so"
         RPATH "$ORIGIN/:$ORIGIN/../lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/jcacace/quad_ws/src/idyntree/build/lib/libidyntree-core.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-core.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-core.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-core.so"
         OLD_RPATH ":::::::::::::::::::::::"
         NEW_RPATH "$ORIGIN/:$ORIGIN/../lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-core.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xshlibx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xruntimex" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/iDynTree/Core" TYPE FILE FILES
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/Axis.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/ArticulatedBodyInertia.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/ClassicalAcc.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/Direction.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/EigenSparseHelpers.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/EigenMathHelpers.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/EigenHelpers.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/InertiaNonLinearParametrization.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/MatrixDynSize.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/MatrixFixSize.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/Position.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/PositionRaw.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/Rotation.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/RotationRaw.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/RotationalInertiaRaw.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/SpatialAcc.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/SpatialForceVector.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/SpatialInertiaRaw.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/SpatialInertia.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/SpatialMomentum.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/SpatialMotionVector.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/TestUtils.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/Transform.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/TransformDerivative.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/Twist.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/Utils.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/VectorFixSize.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/VectorDynSize.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/Wrench.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/PrivateUtils.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/PrivatePreProcessorUtils.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/GeomVector3.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/SpatialVector.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/SparseMatrix.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/Triplets.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/CubicSpline.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/Span.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/SO3Utils.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/MatrixView.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/AngularForceVector3.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/AngularMotionVector3.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/ForceVector3.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/LinearForceVector3.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/LinearMotionVector3.h"
    "/home/jcacace/quad_ws/src/idyntree/src/core/include/iDynTree/Core/MotionVector3.h"
    "/home/jcacace/quad_ws/src/idyntree/build/src/core/CoreExport.h"
    )
endif()

