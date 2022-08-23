//
// Created by 北冥咸鱼 on 2022/8/21.
//

#ifndef GAMES101_RASTERIZER_HPP
#define GAMES101_RASTERIZER_HPP
#include <Eigen/Eigen>


#include "global.hpp"
#include "Object.hpp"
#include "Material.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"


Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos);


Eigen::Matrix4f get_model_matrix(float angle = 0.0f);

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar);

class Rasterizer {
public:
    Rasterizer(const int& width, const int& height, const std::vector<Object*>& objects,
               const std::size_t& transparent_flag, std::vector<Material>& g_buffer, std::vector<float>& z_buffer);

    void setViewMatrix(const Eigen::Matrix4f& m);
    void setModelMatrix(const Eigen::Matrix4f& m);
    void setProjectionMatrix(const Eigen::Matrix4f& m);
    void updataMVPMatrix();

    std::size_t getIndex(int x, int y) const;

    float sphereZ(float x, float y, float radius2, const Sphere sphere) {
        float z_abs = std::sqrt(radius2 - std::pow(x - sphere.center.x(), 2) - std::pow(y - sphere.center.y(), 2));
        if (std::pow(width_ / 2 - sphere.center.x(), 2) + std::pow(height_ / 2 - sphere.center.y(), 2) + std::pow(-sphere.center.z(), 2) < sphere.radius2) {
            return sphere.center.z() - z_abs;
        }
        return sphere.center.z() + z_abs;
    }

    void draw();

    void drawTriangle(Object& object);
    void drawSphere(Object& object);

    void rasterizerTriangle(const Triangle& triangle);
    void rasterizerSphere(const Sphere& sphere);


private:
    int width_;
    int height_;
    const std::vector<Object*>& objects_;
    std::size_t transparent_flag_;
    std::vector<Material>& g_buffer_;
    std::vector<float>& z_buffer_;
    Eigen::Matrix4f M_;
    Eigen::Matrix4f V_;
    Eigen::Matrix4f P_;
    Eigen::Matrix4f MVP_;
};


#endif //GAMES101_RASTERIZER_HPP
