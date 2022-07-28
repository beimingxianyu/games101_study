//
// Created by 北冥咸鱼 on 2022/7/20.
//
#include <Eigen/Eigen>

void f(const Eigen::VEctor4f* v) {
    for (int i = 0; i < 3; ++i) {
        v[i] = {3, 2, 1};
    }
}


int main() {
    Eigen::Vector4f v[] = {{1, 2, 3, 4}, {1, 2, 3, 4}, {1, 2, 3, 4}};
    for (auto& i: v) {
        std::cout << i << '\n';
    }
    return 0;
}