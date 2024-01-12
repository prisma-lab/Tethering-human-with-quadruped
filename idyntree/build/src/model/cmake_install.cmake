# Install script for directory: /home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model

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
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-model.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-model.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-model.so"
         RPATH "$ORIGIN/:$ORIGIN/../lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/build/lib/libidyntree-model.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-model.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-model.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-model.so"
         OLD_RPATH "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/build/lib:"
         NEW_RPATH "$ORIGIN/:$ORIGIN/../lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libidyntree-model.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xshlibx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xruntimex" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/iDynTree/Model" TYPE FILE FILES
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/ContactWrench.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/DenavitHartenberg.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/FixedJoint.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/ForwardKinematics.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/FreeFloatingState.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/FreeFloatingMatrices.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/IJoint.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/Dynamics.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/DynamicsLinearization.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/DynamicsLinearizationHelpers.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/Indices.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/Jacobians.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/JointState.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/LinkTraversalsCache.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/Link.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/LinkState.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/Model.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/ModelTransformers.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/MovableJointImpl.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/RevoluteJoint.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/PrismaticJoint.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/SolidShapes.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/SubModel.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/Traversal.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src/model/include/iDynTree/Model/ModelTestUtils.h"
    "/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/build/src/model/ModelExport.h"
    )
endif()

