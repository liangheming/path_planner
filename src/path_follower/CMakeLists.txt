cmake_minimum_required(VERSION 3.0.2)
project(path_follower)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  nav_msgs
  geometry_msgs
  tf2_ros
  costmap_2d
)
# ${G2O_INCLUDE_DIR} ${G2O_LIBRARIES}
find_package(G2O REQUIRED)
# ${SUITESPARSE_INCLUDE_DIRS}  ${SUITESPARSE_LIBRARIES}
find_package(SUITESPARSE REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES path_follower
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(src/teb_follower TEB_SRC)
file(GLOB EDGES_HPP src/teb_follower/g2o_edges/*.hpp)
set(NANOFLANN_HEADER src/teb_follower/third_party/nanoflann.hpp)

add_executable(path_follower_node src/path_follower_node.cpp src/path_follower_ros.cpp ${TEB_SRC} ${EDGES_HPP} ${NANOFLANN_HEADER})
target_include_directories(path_follower_node PRIVATE ${G2O_INCLUDE_DIR} ${SUITESPARSE_INCLUDE_DIRS})
target_link_libraries(path_follower_node ${catkin_LIBRARIES} ${G2O_LIBRARIES} ${SUITESPARSE_LIBRARIES})


add_executable(tf_pub_once_node src/tf_pub_once_node.cpp)
target_link_libraries(tf_pub_once_node ${catkin_LIBRARIES})

# add_executable(test_g2o_demo src/test_g2o_demo.cpp ${TEB_SRC} ${EDGES_HPP})
# target_include_directories(test_g2o_demo PRIVATE ${G2O_INCLUDE_DIR} ${SUITESPARSE_INCLUDE_DIRS})
# target_link_libraries(test_g2o_demo ${catkin_LIBRARIES} ${G2O_LIBRARIES} ${SUITESPARSE_LIBRARIES})
