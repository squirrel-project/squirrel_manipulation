cmake_minimum_required(VERSION 2.8.3)
project(squirrel_arm_control)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    kukadu
    moveit_msgs
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf
    nav_msgs
    trajectory_msgs
    uibk_planning_node
)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(Armadillo REQUIRED)

catkin_package(
  INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS geometry_msgs kukadu moveit_msgs sensor_msgs std_msgs tf trajectory_msgs uibk_planning_node DEPENDS ${Boost_LIBRARIES} ${ARMADILLO_LIBRARIES} ${PCL_LIBRARIES}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${ARMADILLO_INCLUDE_DIRS})

AUX_SOURCE_DIRECTORY(src/ SQUIRRLCONTSRC)

add_library(${PROJECT_NAME}
    ${SQUIRRLCONTSRC}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES})

add_executable(test_squirrel_control test/test_squirrel_control.cpp)
target_link_libraries(test_squirrel_control ${PROJECT_NAME} ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_squirrel_control ${catkin_EXPORTED_TARGETS})
install(TARGETS test_squirrel_control RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
