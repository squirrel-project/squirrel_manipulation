cmake_minimum_required(VERSION 2.8.3)
project(squirrel_object_manipulation)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  std_msgs
  nav_msgs
  actionlib
  sensor_msgs
  geometry_msgs
  control_toolbox
  cmake_modules
  costmap_2d
  mongodb_store
  squirrel_manipulation_msgs
  squirrel_rgbd_mapping_msgs
  squirrel_object_perception_msgs
)


find_package(Boost REQUIRED COMPONENTS
filesystem
)

catkin_package(
    CATKIN_DEPENDS tf roscpp std_msgs nav_msgs actionlib sensor_msgs geometry_msgs control_toolbox cmake_modules costmap_2d mongodb_store squirrel_manipulation_msgs squirrel_rgbd_mapping_msgs squirrel_object_perception_msgs
)


include_directories(
        include
        /src/pushing/include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}

)

catkin_add_gtest(${PROJECT_NAME}_manipulation test/manipulation.cpp)
target_link_libraries(${PROJECT_NAME}_manipulation ${catkin_LIBRARIES} ${GTEST_LIBRARIES} utils)

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
   src/pushing/src/DipoleField.cpp
   src/pushing/src/CentroidAlignment.cpp
   src/pushing/src/DynamicPush.cpp
)

add_library(utils
   src/utils/conversion_utils.cpp
   src/utils/math_utils.cpp
   src/utils/gnuplot-cpp/gnuplot_i.cpp
)

target_link_libraries(robotino utils)
target_link_libraries(pushlib utils)

add_executable(ardemo src/ARDemo.cpp)
target_link_libraries(ardemo robotino utils ${catkin_LIBRARIES})

add_executable(push src/pushing/pushing.cpp)
target_link_libraries(push robotino utils pushlib ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(smash src/smash.cpp)
target_link_libraries(smash robotino utils ${catkin_LIBRARIES})


