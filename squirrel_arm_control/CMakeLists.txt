cmake_minimum_required(VERSION 2.8.3)
project(squirrel_arm_control)

find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    kukadu
    pcl_ros
    std_msgs
    iis_fast_ik
    sensor_msgs
    geometry_msgs
    iis_robot_dep
    pcl_conversions
    trajectory_msgs
    uibk_planning_node
)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(Armadillo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS kukadu uibk_planning_node
  DEPENDS ${Boost_LIBRARIES} ${ARMADILLO_LIBRARIES} ${PCL_LIBRARIES}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

AUX_SOURCE_DIRECTORY(src/ SQUIRRLCONTSRC)

add_library(${PROJECT_NAME}
    ${SQUIRRLCONTSRC}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_squirrel_control test/test_squirrel_control.cpp)
target_link_libraries(test_squirrel_control ${PROJECT_NAME} ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
