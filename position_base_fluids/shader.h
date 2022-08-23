//
// Created by 北冥咸鱼 on 2022/8/21.
//

#ifndef GAMES101_SHADER_H
#define GAMES101_SHADER_H
#include <Eigen/Eigen>

#include "Material.hpp"

class Shader
{
public:
    void draw(std::vector<Material>& g_buffer, std::vector<Eigen::Vector3f>& fram_buffer);
private:
};


#endif //GAMES101_SHADER_H
