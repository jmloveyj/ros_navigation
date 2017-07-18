cmake_minimum_required(VERSION 2.8.3)
project(move_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  tf
  laser_geometry
  std_msgs
  geometry_msgs
  sensor_msgs
  grid_map_core
  nav_msgs
  grid_map_ros
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_python_setup()

# add_message_files(
#    FILES
#  )
add_message_files(
  DIRECTORY msg
  FILES
  Histogram.msg
)


generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    message_runtime
    roscpp
    rospy
    tf
    std_msgs
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(mapTest 
src/nav_node.cpp
src/map_provider.cpp
src/range_map_updater.cpp
src/laser_map_updater.cpp
src/rrt_planner.cpp
src/vfh.cpp
src/steerer.cpp)
add_dependencies(mapTest ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(mapTest ${catkin_LIBRARIES})

add_executable(mapTest_vfh
src/nav_only_vfh_node.cpp
src/map_provider.cpp
src/range_map_updater.cpp
src/laser_map_updater.cpp
src/vfh.cpp
src/steerer.cpp)
add_dependencies(mapTest_vfh ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(mapTest_vfh ${catkin_LIBRARIES})


add_executable(mapTest_graph
src/nav_graph_node.cpp
src/vfh.cpp
src/steerer.cpp
src/map_provider.cpp
src/range_map_updater.cpp
src/laser_map_updater.cpp
src/astar_planner.cpp)
add_dependencies(mapTest_graph ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(mapTest_graph ${catkin_LIBRARIES})
# install(TARGETS cleanMoveServer snakeMoveService fullCoveragePathPlan stuckRecovery
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(TARGETS cleanMoveLib
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         )
