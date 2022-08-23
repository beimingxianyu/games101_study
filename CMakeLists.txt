cmake_minimum_required(VERSION 3.16)
project(games101)

set(CMAKE_CXX_STANDARD 17)

set(Eigen3_DIR "D:/c/vs/Eigen3/share/eigen3/cmake")

set(OpenCV_DIR "D:/c/vs/opencv/build")

find_package(Eigen3 REQUIRED NO_MODULE)

find_package(OpenCV REQUIRED NO_MODULE)

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${EIGEN3_INCLUDE_DIR})


add_subdirectory(homework0)

add_subdirectory(homework1)

add_subdirectory(homework2)

add_subdirectory(homework3)

add_subdirectory(homework4)

add_subdirectory(homework5)

add_subdirectory(homework6)

add_subdirectory(homework7)

add_subdirectory(position_base_fluids)

add_executable(test test.cpp)

target_link_libraries(test Eigen3::Eigen)


