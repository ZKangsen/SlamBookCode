cmake_minimum_required(VERSION 3.5)
project(feature_extraction)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)
set(SRC_LIST main.cpp)
set(GLOG_LIB glog)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${GLOG_LIB})
