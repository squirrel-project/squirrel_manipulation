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
  squirrel_object_perception_msgs
)

catkin_package(
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

add_library(conversions
   src/utils/conversion_utils.cpp
)

add_executable(ardemo src/ARDemo.cpp)
target_link_libraries(ardemo robotino ${catkin_LIBRARIES})

add_executable(push src/pushing.cpp)
target_link_libraries(push robotino conversions ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pickup src/pickup.cpp)
target_link_libraries(pickup robotino ${catkin_LIBRARIES})
