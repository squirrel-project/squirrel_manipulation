cmake_minimum_required(VERSION 2.8.3)
project(squirrel_handovers)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  squirrel_manipulation_msgs
)

find_package(Boost REQUIRED COMPONENTS
filesystem
)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs actionlib squirrel_manipulation_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(handover src/handover.cpp)
target_link_libraries(handover ${catkin_LIBRARIES})

