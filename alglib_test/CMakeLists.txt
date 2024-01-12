cmake_minimum_required(VERSION 2.8.3)
project(kdl_ros_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


include_directories(
 include

)


## Declare a C++ library
add_library( alglib_lib
    src/alglib/alglibinternal.cpp
    src/alglib/alglibmisc.cpp
    src/alglib/ap.cpp
    src/alglib/linalg.cpp
    src/alglib/optimization.cpp
    src/alglib/solvers.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable( test_node src/test.cpp src/alglib/alglibinternal.cpp
    src/alglib/alglibmisc.cpp
    src/alglib/ap.cpp
    src/alglib/linalg.cpp
    src/alglib/optimization.cpp
    src/alglib/solvers.cpp)



