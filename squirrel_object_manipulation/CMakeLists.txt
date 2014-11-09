cmake_minimum_required(VERSION 2.8.3)
project(squirrel_object_manipulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
sensor_msgs
std_msgs
nav_msgs
tf2
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	${catkin_INCLUDE_DIRS}
)

#find_package(rostest REQUIRED)
#find_package(gtest)

catkin_add_gtest(${PROJECT_NAME}_manipulation test/manipulation.cpp)
target_link_libraries(${PROJECT_NAME}_manipulation ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_library(librobotino
   src/RobotinoControl.cpp
)

## Declare a cpp executable
add_executable(testrobotino src/TestRobotino.cpp)
target_link_libraries(testrobotino librobotino ${catkin_LIBRARIES})

add_executable(ardemo src/ARDemo.cpp)
target_link_libraries(ardemo librobotino ${catkin_LIBRARIES})

#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest_gtest(test_mynode test/test.launch test/manipulation.cpp)
#  target_link_libraries(test_mynode ${catkin_LIBRARIES})
#endif()

