cmake_minimum_required(VERSION 2.8.3)
project(squirrel_ptp_server)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCPP11SUPPORTED)
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCPP11SUPPORTED)
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
  squirrel_manipulation_msgs
  uibk_moveit_config
  uibk_robot_driver
  kukadu
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES squirrel_ptp_server
  CATKIN_DEPENDS actionlib actionlib_msgs roscpp squirrel_manipulation_msgs uibk_moveit_config uibk_robot_driver kukadu
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(squirrel_ptp src/ptp_node.cpp)
target_link_libraries(squirrel_ptp ${catkin_LIBRARIES})
add_dependencies(squirrel_ptp ${catkin_EXPORTED_TARGETS})
install(TARGETS squirrel_ptp RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
