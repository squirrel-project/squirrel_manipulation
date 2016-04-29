cmake_minimum_required(VERSION 2.8.3)
project(uibk_arm_controller)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uibk_arm_controller
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
  DEPENDS dxl rt
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(uibk_arm_controller
   src/arm_controller.cpp
)

add_executable(uibk_arm_demo src/uibk_arm_demo.cpp)

add_dependencies(uibk_arm_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(uibk_arm_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} uibk_arm_controller)

## Specify libraries to link a library or executable target against
target_link_libraries(uibk_arm_controller
  ${catkin_LIBRARIES}
  dxl
  rt
)

target_link_libraries(uibk_arm_demo
  ${catkin_LIBRARIES}
  uibk_arm_controller
  dxl
  rt
)
