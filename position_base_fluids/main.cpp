#include <chrono>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>


#include "Material.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Scene.h"



// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{
    float eye_fov = 45.0f, aspect_ratio = 2, zNear = -965.68f, zFar = -1365.68f;
    Eigen::Vector3f eye_position{0.0f, 0.0f, 965.68f};
    // Change the definition here to change resolution
    Material* white = new Material(DIFFUSE, Eigen::Vector3f(255.0f, 255.0f, 255.0f));
    Material* blue  = new Material(DIFFUSE, Eigen::Vector3f(5.0f, 39.0f, 175.0f));
    Scene scene;

    // 设置M，V，P变换矩阵
    Rasterizer& rasterizer = scene.getRasterizer();
    rasterizer.setModelMatrix(get_model_matrix());
    rasterizer.setViewMatrix(get_view_matrix(eye_position));
    rasterizer.setProjectionMatrix(get_projection_matrix(eye_fov, aspect_ratio, zNear, zFar));

    // 添加绘制对象
    for (float x = -499.5f; x <= 499.5f; x += 20.0f) {
        for (float y = -199.5f; y <= 199.5f; y += 20.0f) {
            for (float z = -199.5f; z <= 199.5f; z += 20.0f) {
                Sphere *sphere = new Sphere(Eigen::Vector3f(x, y, z), 2, blue);
                scene.addObject(sphere);
            }
        }
    }
//    Sphere *sphere = new Sphere(Eigen::Vector3f(0, 0, 10000), 200, blue);
//    scene.addObject(sphere);

    // 展示
    int key = 0;
    int k = 0;
    while(key != 27) {
        scene.clean(Clean::g_buffer | Clean::z_buffer | Clean::frame_buffer);
        scene.draw();

        cv::Mat image(400, 800, CV_32FC3, scene.getFrameBuffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        key = cv::waitKey(10);
        std::cout << k++ << '\n';
    }


    delete white;
    delete blue;
    return 0;
}