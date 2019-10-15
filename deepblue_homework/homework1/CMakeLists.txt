cmake_minimum_required(VERSION 3.5)
project(update_rotation)

include_directories("/usr/include/eigen3")
find_package(Sophus REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(GLOG_LIB glog)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} Sophus_LIBRARIES GLOG_LIB)