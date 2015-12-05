cmake_minimum_required(VERSION 2.8.3)
project(squirrel_kinesthetic_teaching)

AUX_SOURCE_DIRECTORY(src KIN_TEACHING_SRC)

find_package(catkin REQUIRED 
  COMPONENTS 
    roscpp 
    roslib 
    std_msgs 
    moveit_core 
    moveit_ros_planning 
    moveit_ros_planning_interface
    pluginlib
)

find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
)

set(CMAKE_VERBOSE_MAKEFILE on)

include_directories(
  SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include
)

add_executable(squirrel_kinesthetic_teaching_node
  ${KIN_TEACHING_SRC} include
)

target_link_libraries(squirrel_kinesthetic_teaching_node
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)
