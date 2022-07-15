project(housework0)

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/cmake-build-debug/${PROJECT_NAME}/lib)

set(EXECUTABLE_OUTPUT_PATH  ${CMAKE_BINARY_DIR}/cmake-build-debug/${PROJECT_NAME}/bin)

#set(Eigen3_DIR "D:/c/Eigen3/share/eigen3/cmake")
#
#find_package(Eigen3 REQUIRED NO_MODULE)
#
#include_directories(${EIGEN3_INCLUDE_DIR})

list(APPEND ${EXE_PROJECTS} main.cpp)

foreach(EXE_PROJECT IN LISTS ${EXE_PROJECTS})
    set_target_properties(${EXE_PROJECT} PROPERTIES LINK_FLAGS "-Wl,-rpath,../lib")
endforeach()

add_executable(main ./main.cpp)

target_link_libraries(main Eigen3::Eigen)





