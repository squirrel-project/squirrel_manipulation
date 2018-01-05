cmake_minimum_required(VERSION 2.8.3)
project(squirrel_object_manipulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  actionlib_msgs
  tf
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  visualization_msgs
  squirrel_manipulation_msgs
  kclhand_control
)

#catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  roscpp
  rospy
  actionlib
  actionlib_msgs
  tf
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  visualization_msgs
  squirrel_manipulation_msgs
  squirrel_motion_planner_msgs
  kclhand_control
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(squirrel_object_manipulation_server src/squirrel_object_manipulation_server.cpp)
target_link_libraries(squirrel_object_manipulation_server ${catkin_LIBRARIES})
add_dependencies(squirrel_object_manipulation_server ${catkin_EXPORTED_TARGETS})

#install(PROGRAMS
#   scripts/squirrel_grasping_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#install(FILES
#   launch/squirrel_grasping_metahand.launch
#   launch/squirrel_grasping_softhand.launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
