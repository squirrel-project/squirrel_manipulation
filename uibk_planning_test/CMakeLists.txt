cmake_minimum_required(VERSION 2.8.3)
project(uibk_planning_test)

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
  kukadu
  roscpp
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uibk_planning_test
   CATKIN_DEPENDS kukadu roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(uibk_planning_test src/main.cpp)
add_dependencies(uibk_planning_test kukadu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(uibk_planning_test
  ${catkin_LIBRARIES}
)

install(TARGETS uibk_planning_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
