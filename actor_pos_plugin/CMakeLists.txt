cmake_minimum_required(VERSION 2.8.3)
project(actor_pos_plugin)

find_package( Gazebo REQUIRED)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  message_generation
  roscpp
  roslib
  roslib
  tf
  tf_conversions
	tf2_msgs
)
include_directories( /usr/include/eigen3 )

include_directories(
 ${GAZEBO_INCLUDE_DIRS}
 ${GAZEBO_PROTO_INCLUDE_DIRS}
 ${SDF_INCLUDE_DIRS}
)

#add_message_files(
#	FILES
#  arva_data.msg
#  arva.msg
#)

#generate_messages(
#  DEPENDENCIES
#	std_msgs
#  geometry_msgs
#)


catkin_package(
  DEPENDS 
  roscpp 
	gazebo_ros 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(actor_pos_plugin src/actor_pos_plugin.cpp)
target_link_libraries(actor_pos_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


add_library(cable_plugin src/cable.cpp)
target_link_libraries(cable_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})



add_executable( set_laser_pose src/set_laser_pose.cpp)
target_link_libraries( set_laser_pose  ${catkin_LIBRARIES} )


