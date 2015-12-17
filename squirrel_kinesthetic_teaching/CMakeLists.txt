cmake_minimum_required(VERSION 2.8.3)
project(squirrel_kinesthetic_teaching)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    moveit_core
    moveit_msgs
    uibk_planning_node
    moveit_ros_planning_interface
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

AUX_SOURCE_DIRECTORY(src/ KINTEACHSRC)

add_library(${PROJECT_NAME}
    ${KINTEACHSRC}
)

add_executable(testkin test/test_kinesthetic_teaching.cpp)
target_link_libraries(testkin ${catkin_LIBRARIES} ${PROJECT_NAME})
