//
// Created by 北冥咸鱼 on 2022/8/21.
//

#include "shader.h"

void Shader::draw(std::vector<Material> &g_buffer, std::vector<Eigen::Vector3f> &fram_buffer) {
    for (std::size_t i = 0; i < g_buffer.size(); ++i) {
        fram_buffer[i] = g_buffer[i].m_color;
    }
}
