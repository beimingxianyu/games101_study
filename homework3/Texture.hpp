//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    static Eigen::Vector3f makeColor(cv::Vec<uchar, 3> color) {
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v) {
        u *= width; v = (1 - v) * height;
        int pos_u0 = static_cast<int>(u - 0.5), pos_v0 = static_cast<int>(v - 0.5);
        Eigen::Vector3f return_color(Eigen::Vector3f::Zero());
        float s = u - pos_u0 - 0.5;
        float t = v - pos_v0 - 0.5;
        Eigen::Vector3f u00 = makeColor(image_data.at<cv::Vec3b>(pos_v0, pos_u0)),
                u01 = makeColor(image_data.at<cv::Vec3b>(pos_v0 - 1, pos_u0)),
                u10 = makeColor(image_data.at<cv::Vec3b>(pos_v0, pos_u0 + 1)),
                u11 = makeColor(image_data.at<cv::Vec3b>(pos_v0 - 1, pos_u0 + 1));
        if (pos_u0 != -1 && pos_v0 != -1 && pos_u0 != width - 1 && pos_v0 != height - 1) {
            return_color += (1 - s) * u00 + (1 - t) * u00;
            return_color += (1 - s) * u01 + t * u01;
            return_color += s * u10 + (1 -t) * u10;
            return_color += s * u11 + t * u11;
            return return_color / 4;
        }
        if (pos_u0 == -1 && pos_v0 == -1) {
            return_color += makeColor(image_data.at<cv::Vec3b>(0,height - 1));
            return return_color;
        }
        if (pos_u0 == width - 1 && pos_v0 == height - 1) {
            return_color += makeColor(image_data.at<cv::Vec3b>(pos_u0, pos_v0));
            return return_color;
        }
        if (pos_u0 == -1 && pos_v0 == height - 1) {
            return_color += makeColor(image_data.at<cv::Vec3b>(0, pos_v0));
            return return_color;
        }
        if (pos_v0 == -1 && pos_u0 == width - 1) {
            return_color += makeColor(image_data.at<cv::Vec3b >(pos_u0, 0));
            return return_color;
        }
        if (pos_u0 == -1) {
            return_color += (1 - t) * u00 + t * u01;
            return return_color;
        }
        if (pos_v0 == -1) {
            return_color += (1 - s) * u01 + s * u11;
            return return_color;
        }
        if (pos_v0 == height - 1) {
            return_color += (1 - s) * u00 + s * u10;
            return return_color;
        }
        return_color += (1 - t) * u10 + t * u11;
        return return_color;
    }
};
#endif //RASTERIZER_TEXTURE_H
