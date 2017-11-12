cmake_minimum_required(VERSION 2.8.3)
project(squirrel_grasping_new)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  squirrel_manipulation_msgs
  actionlib
  actionlib_msgs
  tf
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  visualization_msgs
)

#catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  roscpp
  rospy
  squirrel_manipulation_msgs
  actionlib
  actionlib_msgs
  tf
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(squirrel_grasp_server src/squirrel_grasp_server.cpp)
target_link_libraries(squirrel_grasp_server ${catkin_LIBRARIES})
add_dependencies(squirrel_grasp_server ${catkin_EXPORTED_TARGETS})

#install(PROGRAMS
#   scripts/squirrel_grasping_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#install(FILES
#   launch/squirrel_grasping_metahand.launch
#   launch/squirrel_grasping_softhand.launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
