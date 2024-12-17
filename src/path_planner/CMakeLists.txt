cmake_minimum_required(VERSION 3.0.2)
project(path_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  nodelet
  pluginlib
  roscpp
  std_msgs
  tf
  tf2_ros
  nav_msgs
)

find_package(ompl REQUIRED)

aux_source_directory(src/planner PLANNER_SRC_LIST)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES path_planner
#  CATKIN_DEPENDS costmap_2d nodelet pluginlib roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(path_planner_node src/path_planner_node.cpp src/path_planner_ros.cpp ${PLANNER_SRC_LIST})
target_link_libraries(path_planner_node ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
