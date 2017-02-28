cmake_minimum_required(VERSION 2.8.3)
project(combine_path)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  move_base
  costmap_2d
  geometry_msgs
  navfn
  roscpp
  std_msgs
  tf
  nav_core
  pluginlib
)

find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DNEW_YAMLCPP)
endif()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    pluginlib
    costmap_2d
    nav_core
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
  src
)

add_library(combine_path src/combine_path.cpp)
target_link_libraries(combine_path
  ${catkin_LIBRARIES}
  ${yaml-cpp_LIBRARIES}
)
