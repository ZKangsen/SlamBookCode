cmake_minimum_required(VERSION 3.5)
project(useSophus)

#find_package(Eigen3 REQUIRED)                  # need to icnlude eigen/Eigen3/Core
find_package(Sophus REQUIRED)
include_directories(/usr/include/eigen3)        # only if include Eigen3/Core
include_directories(${Sophus_INCLUDE_DIR})
set(CMAKE_CXX_STANDARD 11)
set(SRC_LIST main.cpp)
set(GLOG_LIB glog)

add_executable(${PROJECT_NAME} ${SRC_LIST})
#target_link_libraries(${PROJECT_NAME} ${Eigen_LIBRARY_DIR} ${GLOG_LIB})
target_link_libraries(${PROJECT_NAME} ${GLOG_LIB} ${Sophus_LIBRARIES})
