cmake_minimum_required(VERSION 2.8)
project(ex4)
set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -march=native")
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

add_executable(computeORB src/computeORB.cpp)
target_link_libraries(computeORB ${OpenCV_LIBS})

add_executable(LKFlow src/optical_flow.cpp)
target_link_libraries(LKFlow ${OpenCV_LIBS})

add_executable(direct_method src/direct_method.cpp)
target_link_libraries(direct_method ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
