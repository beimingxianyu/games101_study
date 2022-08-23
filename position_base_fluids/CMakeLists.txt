project(RayTracing3)

file(COPY models DESTINATION ${CMAKE_BINARY_DIR}/position_base_fluids)

add_library(OBj_loader SHARED OBj_loader.cpp)
target_include_directories(OBj_loader PUBLIC ${PROJECT_SOURCE_DIR})

add_library(Object SHARED Object.cpp)
target_include_directories(Object PUBLIC ${PROJECT_SOURCE_DIR})

add_library(Material SHARED Material.cpp)
target_include_directories(Material PUBLIC ${PROJECT_SOURCE_DIR})

add_library(Triangle2 SHARED Triangle.cpp)
target_include_directories(Triangle2 PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(Triangle2 PUBLIC OBJ_Loader Object Material)

add_library(Sphere SHARED Sphere.cpp)
target_include_directories(Sphere PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(Sphere PUBLIC OBJ_Loader Material)

add_library(shader SHARED shader.cpp)
target_include_directories(shader PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(shader PUBLIC Material)

add_library(rasterizer2 SHARED rasterizer.cpp)
target_include_directories(rasterizer2 PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(rasterizer2 PUBLIC Material Object Triangle2 Sphere)

add_library(position_base SHARED position_base.cpp)
target_include_directories(position_base PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(position_base PUBLIC Material Object Triangle2 Sphere)

add_library(scene SHARED Scene.cpp)
target_include_directories(scene PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(scene PUBLIC Sphere shader Triangle2 Material Object rasterizer2 position_base)

add_executable(position_base_fluids main.cpp)
target_link_libraries(position_base_fluids PUBLIC OBJ_Loader Object Material Triangle2 Sphere shader scene ${OpenCV_LIBS})
