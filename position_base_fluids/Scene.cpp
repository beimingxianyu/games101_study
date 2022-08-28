//
// Created by 北冥咸鱼 on 2022/8/20.
//

#include "Scene.h"

int Scene::width() {
    return width_;
}

int Scene::height() {
    return height_;
}

Scene::Scene(const int &width, const int &height, const Shader &shader) :
             width_(width), height_(height), shader_(shader),
             g_buffer_(std::vector<Material>(width_ * height_, Material(DIFFUSE, Eigen::Vector3f(255.0f, 255.0f, 255.0f)))),
             frame_buffer_(std::vector<Eigen::Vector3f>(width_ * height_, Eigen::Vector3f(255.0f, 255.0f, 255.0f))),
             z_buffer_(std::vector<float>(width_ * height_, std::numeric_limits<float>::max() * -1.0f)),
             rasterizer_(Rasterizer(width_, height_, objects_, transparent_flag_, g_buffer_, z_buffer_)), 
             position_base_(objects_, transparent_flag_, Bound3D(200.0f, -200.0f, 400.0f, -400.0f, 200.0f, -200.0f)) {}

void Scene::clean(Clean buffer) {
    if ((buffer & Clean::frame_buffer) == Clean::frame_buffer) {
        std::fill(frame_buffer_.begin(), frame_buffer_.end(), Eigen::Vector3f(255.0f, 255.0f, 255.0f));
    }
    if ((buffer & Clean::g_buffer) == Clean::g_buffer) {
        std::fill(g_buffer_.begin(), g_buffer_.end(), Material(DIFFUSE, Eigen::Vector3f(255.0f, 255.0f, 255.0f)));
    }
    if ((buffer & Clean::z_buffer) == Clean::z_buffer) {
        std::fill(z_buffer_.begin(), z_buffer_.end(), std::numeric_limits<float>::max() * -1.0f);
    }
}

std::vector<Object*>& Scene::getObjects() {
    return objects_;
}


void Scene::draw() {
    rasterizer_.draw();
    shader_.draw(g_buffer_, frame_buffer_);
    // TODO 怎加PBF方法
    
}

std::vector<Vector3f> & Scene::getFrameBuffer() {
    return frame_buffer_;
}

Scene &Scene::addObject(Object* object) {
    objects_.push_back(object);
    return *this;
}

Rasterizer & Scene::getRasterizer() {
    return rasterizer_;
}

Scene::~Scene() {
    for (auto object: objects_) {
        delete object;
    }
}


