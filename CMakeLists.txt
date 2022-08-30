cmake_minimum_required(VERSION 3.16)
project(games101)

set(CMAKE_CXX_STANDARD 17)

add_library(test2 SHARED test2.cpp)

add_executable(test test.cpp)
target_link_libraries(test PUBLIC test2)


# set(CMAKE_BUILD_TYPE Release)


