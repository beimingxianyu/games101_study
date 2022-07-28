project(Rasterizer3)

file(COPY models DESTINATION ${CMAKE_BINARY_DIR}/homework3)



add_library(Triangle SHARED Triangle.cpp)

add_library(Texture SHARED Texture.cpp)
target_link_libraries(Texture PUBLIC ${OpenCV_LIBS})

add_library(Shader SHARED Shader.cpp)
target_link_libraries(Shader PUBLIC Texture)

add_library(OBJ_Loader SHARED OBJ_Loader.cpp)

add_library(rasterizer SHARED rasterizer.cpp)
target_link_libraries(rasterizer PUBLIC Triangle Shader)

include_directories(${PROJECT_SOURCE_DIR})

add_executable(Rasterizer3 main.cpp global.hpp)
target_link_libraries(Rasterizer3 ${OpenCV_LIBS} rasterizer OBJ_Loader)
#target_compile_options(Rasterizer PUBLIC -Wall -Wextra -pedantic)