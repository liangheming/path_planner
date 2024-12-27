cmake_minimum_required(VERSION 3.0.2)

project(path_follower)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  message(STATUS "CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE}")
endif()

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

find_package(Eigen3 REQUIRED)


LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
)

# ${G2O_INCLUDE_DIR} ${G2O_LIBRARIES}
find_package(G2O REQUIRED)
# ${SUITESPARSE_INCLUDE_DIRS}  ${SUITESPARSE_LIBRARIES}
find_package(SUITESPARSE REQUIRED)

catkin_package(

)

aux_source_directory(src/teb_follower SOURCE_DIR)
set(THIRD_SRC src/third_party/nanoflann.hpp)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${G2O_INCLUDE_DIR}
  ${SUITESPARSE_INCLUDE_DIRS}
)


add_executable(single_test src/single_test.cpp ${SOURCE_DIR})
target_link_libraries(single_test ${catkin_LIBRARIES} ${G2O_LIBRARIES} ${SUITESPARSE_LIBRARIES})