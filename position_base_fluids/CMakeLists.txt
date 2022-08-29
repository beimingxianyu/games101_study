project(RayTracing3)

file(COPY models DESTINATION ${CMAKE_BINARY_DIR}/position_base_fluids)

add_library(PBDF_OBj_loader SHARED ${PROJECT_SOURCE_DIR}/OBj_loader.cpp)
target_include_directories(PBDF_OBj_loader PUBLIC ${PROJECT_SOURCE_DIR})

add_library(PBDF_Object SHARED ${PROJECT_SOURCE_DIR}/Object.cpp)
target_include_directories(PBDF_Object PUBLIC ${PROJECT_SOURCE_DIR})

add_library(PBDF_Material SHARED ${PROJECT_SOURCE_DIR}/Material.cpp)
target_include_directories(PBDF_Material PUBLIC ${PROJECT_SOURCE_DIR})

add_library(PBDF_Triangle SHARED ${PROJECT_SOURCE_DIR}/Triangle.cpp)
target_include_directories(PBDF_Triangle PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(PBDF_Triangle PUBLIC PBDF_OBj_loader PBDF_Object PBDF_Material)

add_library(PBDF_Sphere SHARED ${PROJECT_SOURCE_DIR}/Sphere.cpp)
target_include_directories(PBDF_Sphere PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(PBDF_Sphere PUBLIC PBDF_OBj_loader PBDF_Material)

add_library(PBDF_shader SHARED ${PROJECT_SOURCE_DIR}/shader.cpp)
target_include_directories(PBDF_shader PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(PBDF_shader PUBLIC PBDF_Material)

add_library(PBDF_rasterizer SHARED ${PROJECT_SOURCE_DIR}/rasterizer.cpp)
target_include_directories(PBDF_rasterizer PUBLIC ${PROJECT_SOURCE_DIR})	
target_link_libraries(PBDF_rasterizer PUBLIC PBDF_Material PBDF_Object PBDF_Triangle PBDF_Sphere)

add_library(position_base SHARED ${PROJECT_SOURCE_DIR}/position_base.cpp)
target_include_directories(position_base PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(position_base PUBLIC PBDF_Material PBDF_Object PBDF_Triangle PBDF_Sphere)

add_library(PBDF_scene SHARED ${PROJECT_SOURCE_DIR}/Scene.cpp)
target_include_directories(PBDF_scene PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(PBDF_scene PUBLIC PBDF_Sphere PBDF_shader PBDF_Triangle PBDF_rasterizer position_base)

add_executable(position_base_fluids ${PROJECT_SOURCE_DIR}/main.cpp)
target_link_libraries(position_base_fluids PUBLIC PBDF_OBj_loader PBDF_Object PBDF_Material PBDF_Triangle PBDF_Sphere PBDF_shader PBDF_scene ${OpenCV_LIBS})
