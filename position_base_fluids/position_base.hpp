//
// Created by 北冥咸鱼 on 2022/8/21.
//

#ifndef GAMES101_POSITION_BASE_HPP
#define GAMES101_POSITION_BASE_HPP
#include "global.hpp"
#include "Object.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"


class PositionBase {
public:
    PositionBase(std::vector<Object*> & objects, const Bound3D& bound = Bound3D(
            std::numeric_limits<float>::max() * -1.0f, std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max() * -1.0f, std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max() * -1.0f, std::numeric_limits<float>::max()),
                 const float & r = 0.02, const float& h = 20, const float & p0 = 1000, const float & g0 = 9.8);

private:
    float p0_;                        // 水的密度
    float m0_;                        // 模拟水的小球的质量
    float h_;                         // 密度影响半径
    float g0_;                        // 重力加速度
    std::vector<Object*> & objects_;  // 运动物体列表
    Bound3D bound3D_;                 // 物体运动的范围
};


#endif //GAMES101_POSITION_BASE_HPP
