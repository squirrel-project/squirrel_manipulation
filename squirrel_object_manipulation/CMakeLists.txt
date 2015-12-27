cmake_minimum_required(VERSION 2.8.3)
project(squirrel_object_manipulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  tf2
  tf
  squirrel_rgbd_mapping_msgs
  genmsg
  actionlib_msgs
  actionlib
  squirrel_manipulation_msgs
  mongodb_store
  cv_bridge
  image_transport
  base_local_planner
  cmake_modules
  control_toolbox
  costmap_2d
  nav_core
  pluginlib
  tf_conversions
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES squirrel_mhand_msgs
  CATKIN_DEPENDS
    squirrel_rgbd_mapping_msgs
    roscpp
    sensor_msgs
    std_msgs
    nav_msgs
    tf2
    squirrel_manipulation_msgs
    cv_bridge
    image_transport
    base_local_planner
    control_toolbox
    costmap_2d
    nav_core
    pluginlib
    roscpp
    tf
    tf_conversions
)

find_package(Boost REQUIRED COMPONENTS
filesystem
)

find_package(Eigen3 REQUIRED)
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

#find_package(rostest REQUIRED)
#find_package(gtest)

catkin_add_gtest(${PROJECT_NAME}_manipulation test/manipulation.cpp)
target_link_libraries(${PROJECT_NAME}_manipulation ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_library(robotino
   src/RobotinoControl.cpp
)
add_library(pushlib
   src/pushing/src/PushPlanner.cpp
   src/pushing/src/SimplePathFollowing.cpp
   src/pushing/src/SimplePush.cpp
   src/pushing/src/BangBangPush.cpp
   src/pushing/src/PushRecoverySM.cpp
)

add_library(utils
   src/utils/conversion_utils.cpp
   src/utils/math_utils.cpp
)
## Declare a cpp executable
add_executable(ardemo src/ARDemo.cpp)
target_link_libraries(ardemo robotino ${catkin_LIBRARIES})

add_executable(push src/pushing/pushing.cpp)
target_link_libraries(push robotino utils pushlib ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pushc src/pushing/pushing_core.cpp)
target_link_libraries(pushc robotino utils pushlib ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(pickup src/pickup.cpp)
target_link_libraries(pickup robotino ${catkin_LIBRARIES})

add_executable(smash src/smash.cpp)
target_link_libraries(smash robotino ${catkin_LIBRARIES})

#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest_gtest(test_mynode test/test.launch test/manipulation.cpp)
#  target_link_libraries(test_mynode ${catkin_LIBRARIES})
#endif()

