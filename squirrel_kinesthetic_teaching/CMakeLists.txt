cmake_minimum_required(VERSION 2.8.3)
project(squirrel_kinesthetic_teaching)

AUX_SOURCE_DIRECTORY(src KIN_TEACHING_SRC)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_generation
)

find_package(Armadillo REQUIRED)
find_package(Boost REQUIRED)

set(CMAKE_VERBOSE_MAKEFILE on)

add_service_files(
  FILES
  StartTeaching.srv
  StopTeaching.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include
  )

add_executable(squirrel_kinesthetic_teaching_node
  ${KIN_TEACHING_SRC} ${KIN_TEACHING_HDRS} include
  )

target_link_libraries(squirrel_kinesthetic_teaching_node
  ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES} ${Boost_LIBRARIES}
  )
