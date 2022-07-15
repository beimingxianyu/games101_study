cmake_minimum_required(VERSION 3.16)
project(games101)

set(CMAKE_CXX_STANDARD 11)

set(Eigen3_DIR "D:/c/Eigen3/share/eigen3/cmake")

find_package(Eigen3 REQUIRED NO_MODULE)

include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(./housework0)

