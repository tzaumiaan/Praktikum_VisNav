cmake_minimum_required(VERSION 2.8)
project(ex3)

set(CMAKE_CXX_FLAGS "-std=c++11")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# eigen
include_directories(/usr/local/include/eigen3/)

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(gn-curve-fitting src/gn-curve-fitting.cpp)
target_link_libraries(gn-curve-fitting ${OpenCV_LIBS})

add_executable(g2o-curve-fitting src/g2o-curve-fitting.cpp)
target_link_libraries(g2o-curve-fitting ${OpenCV_LIBS} g2o_core g2o_stuff)

add_executable(ceres-curve-fitting src/ceres-curve-fitting.cpp)
target_link_libraries(ceres-curve-fitting ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(gn-pose-estimation src/gn-pose-estimate.cpp)
target_link_libraries(gn-pose-estimation ${OpenCV_LIBS})
