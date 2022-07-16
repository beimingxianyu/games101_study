#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity(), model2 = Eigen::Matrix4f::Identity();
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    model << std::cos(rotation_angle/180*MY_PI), -1 * std::sin(rotation_angle/180*MY_PI), 0, 0,
             std::sin(rotation_angle/180*MY_PI), std::cos(rotation_angle/180*MY_PI), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity(), Mtrans = Eigen::Matrix4f::Identity(),
                    Mscale = Eigen::Matrix4f::Identity(), MP20 = Eigen::Matrix4f::Identity();
    float ForY = eye_fov / 180 * MY_PI;
    float t(std::abs(zNear) * std::tan(ForY / 2));
    float r(t * aspect_ratio);
    Mscale << 1/r, 0, 0, 0,
            0, 1/t, 0, 0,
            0, 0, 2/(zNear-zFar), 0,
            0, 0, 0, 1;
    Mtrans << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, -(zNear+zFar)/2,
            0, 0, 0, 1;
    MP20 << zNear, 0, 0, 0, 0, zNear, 0, 0, 0, 0, zNear + zFar, -1 * zNear * zFar, 0, 0, 1, 0;
    projection = Mscale * Mtrans * MP20;
    return projection;
}

Eigen::Matrix4f toMatrix4f(Eigen::Matrix3f m)
{
    Eigen::Matrix4f tRet = Eigen::Matrix4f::Zero();
    tRet.block<3,3>(0,0) = m;
    tRet.row(3) = (Vector4f){0,0,0,1};
    return tRet;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    Eigen::Matrix3f N;
    N << 0, -axis.z(), axis.y(), axis.z(), 0, -axis.x(), -axis.y(), axis.x(), 0;
    Eigen::Matrix3f rotation(std::cos(angle / 180 *MY_PI) * Eigen::Matrix3f::Identity() +
                          (1 - std::cos(angle / 180 *MY_PI)) * (axis * axis.transpose()) +
                          std::sin(angle / 180 *MY_PI) * N);
    Eigen::Matrix4f model;
    model << rotation(0, 0), rotation(0, 1), rotation(0, 2), 0,
             rotation(1, 0), rotation(1, 1), rotation(1, 2), 0,
             rotation(2, 0), rotation(2, 1), rotation(2, 2), 0,
             0, 0, 0, 1;
    return model;
}


//Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
//{
//    float a = angle*MY_PI/180.0;
//    float cosa = cosf(a), sina = sinf(a);
//    Eigen::Matrix3f  I = Eigen::Matrix3f::Identity();
//    axis = axis.normalized();
//    Eigen::Matrix3f nhat;
//    nhat << 0, -axis.z(), axis.y(),
//            axis.z(), 0, -axis.x(),
//            -axis.y(), axis.x(), 0;
//    return toMatrix4f(I + (nhat*nhat)*(1-cosa) + sina*nhat);
//    // return toMatrix4f(cosa*I + (1-cosa)*(axis*axis.transpose()) + sina*nhat);
//}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }
    Eigen::Vector3f x(1.0, 0.0, 0.0);
    Eigen::Vector3f y(0.0, 1.0, 0.0);
    Eigen::Vector3f z(0.0, 0.0, 1.0);
    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
//        r.set_model(get_rotation(y, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
