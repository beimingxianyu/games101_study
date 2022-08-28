//
// Created by 北冥咸鱼 on 2022/8/20.
//

#ifndef GAMES101_SCENE_H
#define GAMES101_SCENE_H
#include "Object.hpp"
#include "shader.h"
#include "rasterizer.hpp"
#include "position_base.hpp"


enum class Clean{
    frame_buffer,
    g_buffer,
    z_buffer
};

inline Clean operator|(Clean a, Clean b)
{
    return Clean((int)a | (int)b);
}

inline Clean operator&(Clean a, Clean b)
{
    return Clean((int)a & (int)b);
}
class Scene {
public:
    Scene(const int& width = 800, const int& height = 400, const Shader& shader = Shader());
    int width();
    int height();
    void clean(Clean);
    std::vector<Object *> & getObjects();
    Scene& addObject(Object* object);
    Rasterizer& getRasterizer();
    void draw();
    std::vector<Vector3f>& getFrameBuffer();
    ~Scene();

private:
    int width_;
    int height_;
    Shader shader_;
    Rasterizer rasterizer_;
    std::vector<Object*> objects_;
    std::size_t transparent_flag_;
    std::vector<Material> g_buffer_;
    std::vector<float> z_buffer_;
    std::vector<Vector3f> frame_buffer_;
    PositionBase position_base_;
};


#endif //GAMES101_SCENE_H
