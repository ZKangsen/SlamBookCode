cmake_minimum_required(VERSION 3.5)
project(icp)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(CMAKE_CXX_STANDARD 11)
find_package(G2O REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${G20_INCLUDE/DIRS} /usr/include/eigen3)
set(SRC_LIST main.cc feature_extraction/extract_feature.cc utils/utils.cc svd_icp/svd.cc)
include_directories(/usr/include/eigen)
set(GLOG_LIB glog)
# g2o
set(G2O_CORE_LIB g2o_core)
set(G2O_STUFF_LIB g2o_stuff)
set(G2O_TYPE_LIB g2o_types_sba)

add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${GLOG_LIB}
        ${G2O_CORE_LIB} ${G2O_STUFF_LIB} ${G2O_TYPE_LIB})
