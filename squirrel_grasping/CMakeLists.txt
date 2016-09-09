cmake_minimum_required(VERSION 2.8.3)
project(squirrel_grasping)

find_package(catkin REQUIRED
  rospy
  squirrel_manipulation_msgs
  actionlib
  actionlib_msgs
  mongodb_store
  tf
  kclhand_control
  geometry_msgs
  dynamic_reconfigure
)

catkin_python_setup()

catkin_package(
)

 install(PROGRAMS
   scripts/squirrel_blind_grasping_node.py
   scripts/squirrel_haf_grasping_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(FILES
   launch/squirrel_blind_grasping_node.launch
   launch/squirrel_haf_grasping_node.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
