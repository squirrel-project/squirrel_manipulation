cmake_minimum_required(VERSION 2.8.3)
project(squirrel_moveit_utils)

find_package(catkin REQUIRED
  rospy
  geometry_msgs
  moveit_commander
)

catkin_package(
)

 install(PROGRAMS
   scripts/ground_emulation.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(FILES
   launch/squirrel_moveit_utils.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
