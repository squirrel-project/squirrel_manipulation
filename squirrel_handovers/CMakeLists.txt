cmake_minimum_required(VERSION 2.8.3)
project(squirrel_handovers)

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
add_dependencies(handover squirrel_manipulation_msgs_gencpp)

#add_executable(handover_collect src/handover_data_collect.cpp)
#target_link_libraries(handover_collect ${catkin_LIBRARIES})
