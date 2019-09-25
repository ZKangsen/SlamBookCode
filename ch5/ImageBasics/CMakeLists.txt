cmake_minimum_required(VERSION 3.5)
project(ImageBasics)
find_package(OpenCV REQUIRED)
set(CMAKE_CXX_STANDARD 11)
set(SRC_LIST main.cpp)
set(GLOG_LIB glog)

add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${GLOG_LIB})