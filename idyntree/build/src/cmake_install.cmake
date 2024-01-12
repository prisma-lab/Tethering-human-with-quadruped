# Install script for directory: /home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/src

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

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/build/src/core/cmake_install.cmake")
  include("/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/build/src/model/cmake_install.cmake")
  include("/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/build/src/sensors/cmake_install.cmake")
  include("/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/build/src/model_io/cmake_install.cmake")
  include("/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/build/src/estimation/cmake_install.cmake")
  include("/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/build/src/solid-shapes/cmake_install.cmake")
  include("/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/build/src/high-level/cmake_install.cmake")
  include("/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/build/src/inverse-kinematics/cmake_install.cmake")
  include("/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/build/src/optimalcontrol/cmake_install.cmake")
  include("/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/build/src/visualization/cmake_install.cmake")
  include("/home/jcacace/dev/quad_ws/src/tethered_dog/idyntree/build/src/tools/cmake_install.cmake")

endif()

