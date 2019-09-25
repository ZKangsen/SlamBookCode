cmake_minimum_required(VERSION 3.5)
project(g2o_curve_fitting)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV REQUIRED)
find_package(G2O REQUIRED)

include_directories(${G2O_INCLUDE_DIRS} /usr/include/eigen3)

set(CMAKE_CXX_STANDARD 11)
set(SRC_LIST main.cpp)
set(G2O_CORE_LIB g2o_core)
set(G2O_STUFF_LIB g2o_stuff)
set(G2O_TYPES g2o_types_sba)
set(GLOG_LIB glog)

add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${G2O_CORE_LIB} ${G2O_STUFF_LIB} ${GLOG_LIB}
        ${G2O_TYPES})
