project(RayTracing1)

add_executable(RayTracing1 main.cpp Object.hpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp Scene.hpp Light.hpp Renderer.cpp)
#target_compile_options(RayTracing1 PUBLIC -Wall -Wextra -pedantic -Wshadow -Wreturn-type -fsanitize=undefined)
#target_compile_features(RayTracing1 PUBLIC cxx_std_17)
#target_link_libraries(RayTracing1 PUBLIC -fsanitize=undefined)
