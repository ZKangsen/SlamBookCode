cmake_minimum_required(VERSION 3.5)
project(ceres_curve_fitting)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
set(CMAKE_CXX_STANDARD 11)
set(SRC_LIST main.cpp)
set(GLOG_LIB glog)
set(CERES_LIB ceres)

add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${CERES_LIB} ${GLOG_LIB})