cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(CERES_ICP)

find_package(Boost REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
    ${CERES_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${BOOST_HEADER_DIR})
include_directories(
  include
  include/yaml-cpp
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(ceres_icp   src/main.cpp src/ceres_icp.cpp src/lidarCeres.cpp)
target_link_libraries(ceres_icp 
    ${PCL_LIBRARIES} 
    ${CERES_LIBRARIES}
    ${BOOST_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/libs/libyaml-cpp.a)