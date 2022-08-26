//
// Created by 北冥咸鱼 on 2022/8/21.
//

#ifndef GAMES101_POSITION_BASE_HPP
#define GAMES101_POSITION_BASE_HPP
#include "global.hpp"
#include "Object.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "Eigen/Eigen"


class PositionBase {
public:
    PositionBase(std::vector<Object*> & objects, const std::size_t transparent_flag, const Bound3D& bound = Bound3D(
                 std::numeric_limits<float>::max() * -1.0f, std::numeric_limits<float>::max(),
                 std::numeric_limits<float>::max() * -1.0f, std::numeric_limits<float>::max(),
                 std::numeric_limits<float>::max() * -1.0f, std::numeric_limits<float>::max()),
                 const float & r = 0.02, const float& h = 20, const float & p0 = 1000, const float & g0 = 9.8);

private:
    // 获取相邻点
    std::vector<std::vector<std::size_t> > proximityPoind(const std::size_t& beg, const std::size_t& end);
    // 使用Poly6光滑核计算W    
    float wPoly6(const Eigen::Vector3f& r);
    // 计算Poly6光滑核的导数
    Eigen::Vector3f derivativeOfWPoly6(const std::size_t& i, const std::size_t& j);
    // 使用Spiky光滑核计算
    float wSpiky(const Eigen::Vector3f& r);
    // 计算使用Spiky光滑核的导数（当k为i时）
    Eigen::Vector3f derivativeOfWSpikyWhenTheKEqualI(const std::size_t& i);
    // 计算使用Spiky光滑核的导数（当k为j时）
    Eigen::Vector3f derivativeOfWSpikyWhenTheKEqualJ(const std::size_t& i, const std::size_t& j);
    // 计算使用Spiky光滑核的导数
    float derivativeOfWSpiky(const std::size_t& i);
    // 计算物体i的密度
    float densityI(const std::size_t& i);
    // 计算约束函数的值
    float constraintFunction(const std::size_t& i);
    // CMF参数（一般为很小的正数，暂时不看论文了，这里设置为EPSILON)
    float CMF();
    // 引入拉伸不稳定性
    float S_corr(const std::size_t& i, const std::size_t& j, const float& k = 0.1f, const int& n = 4);
	// 求Ramta的值
	float Ramta(const std::size_t& i);
    // 位置更新向量
    Eigen::Vector3f deltaPotion(const std::size_t& i);
    
    
    

private:
    float density_0_;                                     // 水的密度
    float m0_;                                            // 模拟水的小球的质量
    float h_;                                             // 密度影响半径
    float g0_;                                            // 重力加速度
    std::vector<Object*> & objects_;                      // 运动物体列表
    std::size_t transparent_flag_;                        // 透明标志
    Bound3D bound3D_;                                     // 物体运动的范围
    std::vector<std::vector<std::size_t> > proximity_poind_;  // 相邻点列表
};


#endif //GAMES101_POSITION_BASE_HPP
