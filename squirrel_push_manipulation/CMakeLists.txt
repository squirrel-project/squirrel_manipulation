cmake_minimum_required(VERSION 2.8.3)
project(squirrel_push_manipulation)

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
  robotino_msgs
  squirrel_manipulation_msgs
  squirrel_object_perception_msgs
  squirrel_view_controller_msgs
  squirrel_navigation_msgs
)


find_package(Boost REQUIRED COMPONENTS
filesystem
)

catkin_package(
    CATKIN_DEPENDS tf roscpp std_msgs nav_msgs actionlib sensor_msgs geometry_msgs robotino_msgs control_toolbox cmake_modules costmap_2d mongodb_store squirrel_manipulation_msgs squirrel_rgbd_mapping_msgs squirrel_object_perception_msgs squirrel_view_controller_msgs squirrel_navigation_msgs
)

include_directories(${ARMADILLO_INCLUDE_DIRS})

include_directories(
        include
        /src/pushing/include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}

)

catkin_add_gtest(${PROJECT_NAME}_manipulation test/manipulation.cpp)
target_link_libraries(${PROJECT_NAME}_manipulation ${catkin_LIBRARIES} ${GTEST_LIBRARIES}  ${ARMADILLO_LIBRARIES})

add_library(robotino
   src/RobotinoControl.cpp
   src/RobotinoBaseControl.cpp
)

add_library(pushlib
   src/pushing/src/PushPlanner.cpp
   src/pushing/src/VisualisePushPlanner.cpp
   src/pushing/src/DynamicPush.cpp
)

add_library(push_utils
   src/utils/conversion_utils.cpp
   src/utils/math_utils.cpp
   src/utils/gnuplot-cpp/gnuplot_i.cpp
)


add_executable(push src/pushing/pushing.cpp)
target_link_libraries(push robotino push_utils pushlib ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(smash src/smash.cpp)
target_link_libraries(smash robotino push_utils ${catkin_LIBRARIES})

