cmake_minimum_required(VERSION 3.5)
project(pose_estimation)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)
set(SRC_LIST main.cpp pose_estimate/pose_estimate.cc feature_extraction/extract_feature.cc)
set(GLOG_LIB glog)

add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${GLOG_LIB})
