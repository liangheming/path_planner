cmake_minimum_required(VERSION 3.0.2)
project(path_follower)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
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

aux_source_directory(src/teb_local_planner TEB_SRC)

add_executable(main src/main.cpp ${TEB_SRC})
