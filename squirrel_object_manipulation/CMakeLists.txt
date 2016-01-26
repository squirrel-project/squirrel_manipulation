cmake_minimum_required(VERSION 2.8.3)
project(squirrel_object_manipulation)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cmake_modules
  control_toolbox
  costmap_2d
  cv_bridge
  genmsg
  geometry_msgs
  image_transport
  kukadu
  mongodb_store
  nav_msgs
  roscpp
  sensor_msgs
  squirrel_manipulation_msgs
  squirrel_object_perception_msgs
  squirrel_rgbd_mapping_msgs
  std_msgs
  tf
  tf2
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS
filesystem
)

catkin_package(
  CATKIN_DEPENDS
    control_toolbox
    cv_bridge
    geometry_msgs
    image_transport
    nav_msgs
    roscpp
    sensor_msgs
    squirrel_manipulation_msgs
    squirrel_object_perception_msgs
    squirrel_rgbd_mapping_msgs
    std_msgs
    tf
    tf2
    visualization_msgs
)

include_directories(
	include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
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

add_library(conversions
   src/utils/conversion_utils.cpp
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

add_dependencies(robotino ${catkin_EXPORTED_TARGETS})
add_dependencies(pushlib ${catkin_EXPORTED_TARGETS})
add_dependencies(conversions ${catkin_EXPORTED_TARGETS})
add_dependencies(utils ${catkin_EXPORTED_TARGETS})
add_dependencies(ardemo ${catkin_EXPORTED_TARGETS})
add_dependencies(push ${catkin_EXPORTED_TARGETS})
add_dependencies(smash ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
	catkin_add_gtest(${PROJECT_NAME}_manipulation test/manipulation.cpp)
endif()

install(TARGETS ardemo push smash RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS robotino pushlib conversions utils ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
