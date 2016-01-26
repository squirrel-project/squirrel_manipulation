cmake_minimum_required(VERSION 2.8.3)
project(squirrel_kinesthetic_teaching)

find_package(catkin REQUIRED COMPONENTS
    moveit_core
    moveit_msgs
    moveit_ros_planning_interface
    roscpp
    std_msgs
    trajectory_msgs
    uibk_planning_node
)

find_package(cmake_modules REQUIRED)
find_package(Armadillo REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS moveit_msgs roscpp std_msgs trajectory_msgs uibk_planning_node
    DEPENDS ${ARMADILLO_LIBRARIES}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    ${KINTEACHSRC}
)

target_link_libraries(${PROJECT_NAME} ${ARMADILLO_LIBRARIES})

add_executable(testkin test/test_kinesthetic_teaching.cpp)
target_link_libraries(testkin ${catkin_LIBRARIES} ${PROJECT_NAME} ${ARMADILLO_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(testkin ${catkin_EXPORTED_TARGETS})
install(TARGETS testkin RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

AUX_SOURCE_DIRECTORY(src/ KINTEACHSRC)
