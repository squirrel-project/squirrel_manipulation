cmake_minimum_required(VERSION 2.8.3)
project(squirrel_object_manipulation)

find_package(catkin REQUIRED COMPONENTS
  tf
  tf2
  rospy
  roscpp
  genmsg
  kukadu
  std_msgs
  nav_msgs
  actionlib
  cv_bridge
  sensor_msgs
  mongodb_store
  geometry_msgs
  actionlib_msgs
  image_transport
  squirrel_manipulation_msgs
  squirrel_rgbd_mapping_msgs

)

catkin_package(
  CATKIN_DEPENDS
    squirrel_rgbd_mapping_msgs
    squirrel_manipulation_msgs
    roscpp
    sensor_msgs
    std_msgs
    nav_msgs
    tf
    tf_conversions
    #tf2
    #cv_bridge
    #image_transport
    #base_local_planner
    #costmap_2d
    #nav_core
    control_toolbox
    pluginlib
)

find_package(Boost REQUIRED COMPONENTS
filesystem
)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

catkin_add_gtest(${PROJECT_NAME}_manipulation test/manipulation.cpp)
target_link_libraries(${PROJECT_NAME}_manipulation ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_library(robotino
   src/RobotinoControl.cpp
)
add_library(pushlib
   src/pushing/src/PushPlanner.cpp
   src/pushing/src/VisualisePushPlanner.cpp
   src/pushing/src/SimplePathFollowing.cpp
   src/pushing/src/PIDPathFollowing.cpp
   src/pushing/src/SimplePush.cpp
   src/pushing/src/BangBangPush.cpp
   src/pushing/src/PIDSimplePush.cpp
   src/pushing/src/PIDPush.cpp
   src/pushing/src/PIDObjectPush.cpp
)

add_library(utils
   src/utils/conversion_utils.cpp
   src/utils/math_utils.cpp
   src/utils/gnuplot-cpp/gnuplot_i.cpp
)

add_executable(ardemo src/ARDemo.cpp)
target_link_libraries(ardemo robotino ${catkin_LIBRARIES})

add_executable(push src/pushing/pushing.cpp)
target_link_libraries(push robotino utils pushlib ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(smash src/smash.cpp)
target_link_libraries(smash robotino ${catkin_LIBRARIES})

#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest_gtest(test_mynode test/test.launch test/manipulation.cpp)
#  target_link_libraries(test_mynode ${catkin_LIBRARIES})
#endif()

