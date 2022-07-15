//
// Created by 北冥咸鱼 on 2022/7/13.
//
#include <iostream>
#include <Eigen/Core>
#include <cmath>


int main(int argc, char** argv) {
    Eigen::Vector2f p(2, 1), ap(1, 2);
    Eigen::Matrix2f m, t;
//    m << static_cast<float>(std::cos(acos(-1)/2)) << static_cast<float >(std::sin(acos(-1)/2))
//      << static_cast<float >(-1 * std::sin(acos(-1)/2)) << static_cast<float>(std::cos(acos(-1)/2));
    m << std::cos(acos(-1)/2), -1 * std::sin(acos(-1)/2), std::sin(acos(-1)/2), std::cos(acos(-1)/2);
    p = m * p;
    p += ap;
    std::cout << p << std::endl;
    t << 1, 2, 3, 4;
    std::cout << t << std::endl;
    return 0;
}