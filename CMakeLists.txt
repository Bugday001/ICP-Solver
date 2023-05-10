cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(CERES_ICP)

set(g2o_DIR /usr/local/lib/cmake/g2o)
find_package(g2o REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(FMT REQUIRED)
FIND_PACKAGE(GTSAM REQUIRED)

include_directories(
    ${CERES_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${BOOST_HEADER_DIR}
    ${g2o_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR})
include_directories(
  include
  include/yaml-cpp
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(G2O_LIBS /usr/local/lib/libg2o_cli.so /usr/local/lib/libg2o_ext_freeglut_minimal.so /usr/local/lib/libg2o_simulator.so /usr/local/lib/libg2o_solver_slam2d_linear.so /usr/local/lib/libg2o_types_icp.so /usr/local/lib/libg2o_types_slam2d.so /usr/local/lib/libg2o_core.so /usr/local/lib/libg2o_interface.so /usr/local/lib/libg2o_solver_csparse.so /usr/local/lib/libg2o_solver_structure_only.so /usr/local/lib/libg2o_types_sba.so /usr/local/lib/libg2o_types_slam3d.so /usr/local/lib/libg2o_csparse_extension.so /usr/local/lib/libg2o_opengl_helper.so /usr/local/lib/libg2o_solver_dense.so /usr/local/lib/libg2o_stuff.so /usr/local/lib/libg2o_types_sclam2d.so /usr/local/lib/libg2o_parser.so /usr/local/lib/libg2o_solver_pcg.so /usr/local/lib/libg2o_types_data.so /usr/local/lib/libg2o_types_sim3.so cxsparse )

add_executable(ceres_icp   src/main.cpp src/ceres_icp.cpp src/g2o_icp.cpp src/gtsam_icp.cpp src/lidarCeres.cpp)
target_link_libraries(ceres_icp 
    ${PCL_LIBRARIES} 
    ${CERES_LIBRARIES}
    ${BOOST_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/libs/libyaml-cpp.a
    ${Sophus_LIBRARIES} fmt::fmt
    ${G2O_LIBS}
    ${Boost_LIBRARIES} -lgtsam -ltbb)