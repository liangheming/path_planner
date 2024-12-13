cmake_minimum_required(VERSION 3.0.2)
project(map_manager)

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
  nav_msgs
  roscpp
  std_msgs
  # zouyu_srvs
  nodelet
  pluginlib
  tf
)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_manager
#  CATKIN_DEPENDS nav_msgs roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(map_manager_node src/map_manager_node.cpp src/map_manager_ros.cpp)
target_link_libraries(map_manager_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)

add_library(map_manager_nodelet src/map_manager_nodelet.cpp src/map_manager_ros.cpp)
target_link_libraries(map_manager_nodelet ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)

