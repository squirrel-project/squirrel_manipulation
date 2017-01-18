cmake_minimum_required(VERSION 2.8.3)
project(squirrel_placement)

find_package(catkin REQUIRED
  rospy
  squirrel_manipulation_msgs
  squirrel_ptp_server
  actionlib
  actionlib_msgs
  squirrel_softhand
  kclhand_control
  geometry_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package(
)

install(PROGRAMS
   scripts/squirrel_placement_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(FILES
   launch/squirrel_placement_metahand.launch
   launch/squirrel_placement_softhand.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
