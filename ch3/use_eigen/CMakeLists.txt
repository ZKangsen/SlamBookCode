cmake_minimum_required(VERSION 3.5)
project(use_eigen)
find_package(Eigen3 REQUIRED)
set(CMAKE_CXX_STANDARD 11)
set(GLOG_LIB glog)
set(GFLAGS_LIB gflags)

add_executable(UseEigen use_eigen.cpp)
target_link_libraries(UseEigen ${Eigen_LIBRARY_DIR} ${GLOG_LIB} ${GFLAGS_LIB})