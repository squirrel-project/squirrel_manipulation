cmake_minimum_required(VERSION 2.8.3)
project(squirrel_moveit_utils)

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    cmake_modules
    geometric_shapes
    moveit_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
)

catkin_package(
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(add_case src/add_case.cpp)
target_link_libraries(add_case ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS add_case DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(add_ground src/add_ground.cpp)
target_link_libraries(add_ground ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS add_ground DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
   launch/squirrel_moveit_utils.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
