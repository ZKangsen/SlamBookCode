cmake_minimum_required(VERSION 3.5)
project(vo_prj)

find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
include_directories(/usr/include/eigen3)
include_directories(./include)
set(CMAKE_CXX_STANDARD 11)
set(SRC_LIST test/main_test.cc src/camera.cc src/frame.cc src/map_point.cc src/map.cc src/config.cc)
set(GLOG_LIB glog)

add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${GLOG_LIB})
