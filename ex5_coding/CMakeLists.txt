cmake_minimum_required(VERSION 2.8)
project(ex5)
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# eigen
include_directories("/usr/local/include/eigen3")  # change this if your eigen is not installed by apt-get

# sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# g2o or ceres if needed
# g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
# ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable(BAL src/BAL.cpp)
target_link_libraries(BAL ${CERES_LIBRARIES} g2o_core g2o_stuff)

add_executable(PBA src/PBA.cpp)
target_link_libraries(PBA ${OpenCV_LIBS} ${CERES_LIBRARIES} ${Pangolin_LIBRARIES} g2o_core g2o_stuff g2o_types_sba)
