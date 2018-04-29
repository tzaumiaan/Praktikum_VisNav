cmake_minimum_required(VERSION 2.6)
project(ex2_coding)

# compile options
set(CMAKE_CXX_STANDARD 11)

# dependencies
## Eigen
include_directories(/usr/local/include/eigen3/)
## Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
## Pangolin
find_package(Pangolin)
include_directories(${Pangolin_INCLUDE_DIRS})
## OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# executables
add_executable(ex2_task1 src/left_jacobian.cpp)
target_link_libraries(ex2_task1 ${Sophus_LIBRARIES})

add_executable(ex2_task2 src/draw_trajectory.cpp)
target_link_libraries(ex2_task2 ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES})

add_executable(ex2_task3_1 src/undistort_image.cpp)
target_link_libraries(ex2_task3_1 ${OpenCV_LIBS})

add_executable(ex2_task3_2 src/disparity.cpp)
target_link_libraries(ex2_task3_2 ${OpenCV_LIBS} ${Pangolin_LIBRARIES})

add_executable(ex2_task3_3 src/build_map.cpp)
target_link_libraries(ex2_task3_3 ${Sophus_LIBRARIES} ${OpenCV_LIBS} ${Pangolin_LIBRARIES})

