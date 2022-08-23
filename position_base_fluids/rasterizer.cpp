//
// Created by 北冥咸鱼 on 2022/8/21.
//

#include "rasterizer.hpp"

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
            0,1,0,-eye_pos[1],
            0,0,1,-eye_pos[2],
            0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * M_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
            0, 1, 0, 0,
            -sin(angle), 0, cos(angle), 0,
            0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

//Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
//{
//    float n = zNear;
//    float f = zFar;
//    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
//    float t = -tan((eye_fov/360)*M_PI)*(abs(n)); //top
//    float r = t/aspect_ratio;
//
//    Eigen::Matrix4f Mp;//透视矩阵
//    Mp <<
//            n, 0, 0,   0,
//            0, n, 0,   0,
//            0, 0, n+f, -n*f,
//            0, 0, 1,   0;
//    Eigen::Matrix4f Mo_tran;//平移矩阵
//    Mo_tran <<
//            1, 0, 0, 0,
//            0, 1, 0, 0,  //b=-t;
//            0, 0, 1, -(n+f)/2 ,
//            0, 0, 0, 1;
//    Eigen::Matrix4f Mo_scale;//缩放矩阵
//    Mo_scale <<
//             1/r,     0,       0,       0,
//            0,       1/t,     0,       0,
//            0,       0,       2/(n-f), 0,
//            0,       0,       0,       1;
//    projection = (Mo_scale*Mo_tran)* Mp;//投影矩阵
//    //这里一定要注意顺序，先透视再正交;正交里面先平移再缩放；否则做出来会是一条直线！
//    return projection;
//}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    float n = zNear, f = zFar;
    float tanFov = std::tan(eye_fov / 2 / 180 * M_PI);
    float t = std::abs(n) * tanFov, r = aspect_ratio * t, b = -t, l = -r;
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    // 投影矩阵
    Eigen::Matrix4f M_P;
    M_P << n, 0, 0, 0,
           0, n, 0, 0,
           0, 0, n+f, -n*f,
           0, 0, 1, 0;

    // 位移矩阵
    Eigen::Matrix4f M_tran;
    M_tran << 1, 0, 0, -(r+l)/2,
              0, 1, 0, -(t+b)/2,
              0, 0, 1, -(n+f)/2,
              0, 0, 0, 1;

    // 缩放矩阵
    Eigen::Matrix4f M_scale;
    M_scale << 2/(r-l), 0, 0, 0,
               0, 2/(t-b), 0, 0,
               0, 0, 2/(n-f), 0,
               0, 0, 0, 1;

    projection = (M_scale*M_tran)* M_P;
    return projection;
}

//static bool insideTriangle(float x, float y, const Vector4f* _v)
//{
//    Vector3f Q = {x,y,0};
//
//    Vector3f p0p1 = _v[1].head<3>() - _v[0].head<3>();
//    Vector3f p0Q = Q - _v[0].head<3>();
//
//    Vector3f p1p2 = _v[2].head<3>() - _v[1].head<3>();
//    Vector3f p1Q = Q - _v[1].head<3>();
//
//    Vector3f p2p0 = _v[0].head<3>() - _v[2].head<3>();
//    Vector3f p2Q = Q - _v[2].head<3>();
//
//    //类定义里面已经定义是逆时针，所以只用考虑同正情况。
//    return p0p1.cross(p0Q).z() > 0 && p1p2.cross(p1Q).z() > 0 && p2p0.cross(p2Q).z()>0;
//}

Vector4f toVector4f(const Vector3f v) {
    return Vector4f{v.x(), v.y(), v.z(), 1.0f};
}

Rasterizer::Rasterizer(const int &width, const int &height, const std::vector<Object *> &objects,
                       const size_t &transparent_flag, std::vector<Material> &g_buffer, std::vector<float>& z_buffer) :
                       width_(width), height_(height), objects_(objects), transparent_flag_(transparent_flag),
                       g_buffer_(g_buffer), z_buffer_(z_buffer) {}


std::size_t Rasterizer::getIndex(int x, int y) const {
    return (height_-y - 1)*width_ + x;
}

void Rasterizer::updataMVPMatrix() {
    MVP_ = P_ * V_ * M_;
}

void Rasterizer::setViewMatrix(const Matrix4f &m) {
    V_ = m;
    updataMVPMatrix();
}

void Rasterizer::setModelMatrix(const Matrix4f &m) {
    M_ = m;
    updataMVPMatrix();
}

void Rasterizer::setProjectionMatrix(const Eigen::Matrix4f& m) {
    P_ = m;
    updataMVPMatrix();
}

void Rasterizer::draw() {
    for (Object *object: objects_) {
        if (object->getType() == Triangle_type) {
            drawTriangle(*object);
        } else {
            drawSphere(*object);
        }
    }
}

void Rasterizer::drawTriangle(Object &object) {
    Triangle triangle = dynamic_cast<Triangle&>(object);
    float f1 = std::abs((-1365.68f - -965.68f) / 2.0);
    float f2 = (-1365.68f + -965.68f) / 2.0;
    Eigen::Vector4f v[] = {
            MVP_ * toVector4f(triangle.v0),
            MVP_ * toVector4f(triangle.v1),
            MVP_ * toVector4f(triangle.v2)
    };

    for (auto& vec : v) {
        vec.x()/=vec.w();
        vec.y()/=vec.w();
        vec.z()/=vec.w();
    }

    for (auto & vert : v)
    {
        vert.x() = 0.5*width_*(vert.x()+1.0);
        vert.y() = 0.5*height_*(vert.y()+1.0);
        vert.z() = vert.z() * f1 + f2;
    }

    triangle.v0 = v[0].head<3>();
    triangle.v1 = v[1].head<3>();
    triangle.v2 = v[2].head<3>();

    rasterizerTriangle(triangle);
}

void Rasterizer::drawSphere(Object &object) {
    Sphere sphere = dynamic_cast<Sphere&>(object);
    float f1 = std::abs((-1365.68f - -965.68f) / 2.0);
    float f2 = (-1365.68f + -965.68f) / 2.0;

    Vector4f v = MVP_ * toVector4f(sphere.center);

    v.x() /= v.w();
    v.y() /= v.w();
    v.z() /= v.w();

    v.x() = 0.5*width_*(v.x()+1.0);
    v.y() = 0.5*height_*(v.y()+1.0);
    v.z() = v.z() * f1 + f2;

    sphere.center = v.head<3>();

    rasterizerSphere(sphere);
}

void Rasterizer::rasterizerTriangle(const Triangle &triangle) {
    Bound bound = triangle.calculateBound();

    if (bound.bottom_ > height_ || bound.top_ < 0 || bound.right_ < 0 || bound.right_ > width_ ) {
        return;
    }

    int top = std::min(height_ - 1, static_cast<int>(std::floor(bound.top_ - 0.5))),
        bottom = std::max(0, static_cast<int>(std::floor(bound.bottom_ + 0.5))),
        right = std::min(width_ - 1, static_cast<int>(std::floor(bound.right_ - 0.5))),
        left = std::max(0, static_cast<int>(std::floor(bound.left_ + 0.5)));

    std::array<Eigen::Vector3f, 3> x_sort{triangle.v0, triangle.v1, triangle.v2};
    std::array<Eigen::Vector3f, 3> y_sort{triangle.v0, triangle.v1, triangle.v2};
    std::sort(x_sort.begin(), x_sort.end(), [](auto a, auto b) {return a.x() < b.x();});
    std::sort(y_sort.begin(), y_sort.end(), [](auto a, auto b) {return a.y() < b.y();});



    for (int y = bottom; y <= top; ++y) {
        for (int x = left; x <= right; ++x) {
            // todo mm

        }
    }

}

void Rasterizer::rasterizerSphere(const Sphere &sphere) {
    Bound bound = sphere.calculateBound();

    if (bound.bottom_ > height_ || bound.top_ < 0 || bound.right_ < 0 || bound.right_ > width_ ) {
        return;
    }

    int top = std::min(height_ - 1, static_cast<int>(std::floor(bound.top_ - 0.5))),
        bottom = std::max(0, static_cast<int>(std::floor(bound.bottom_ + 0.5))),
        right = std::min(width_ - 1, static_cast<int>(std::floor(bound.right_ - 0.5))),
        left = std::max(0, static_cast<int>(std::floor(bound.left_ + 0.5)));
//    float top = std::min(static_cast<float>(height_ - 1), std::floor(bound.top_ - 0.5f)),
//          bottom = std::max(0.0f, std::floor(bound.top_ + 0.5f)),
//          right = std::min(static_cast<float>(width_ - 1), std::floor(bound.right_ - 0.5f)),
//          left = std::max(0.0f, std::floor(bound.left_ + 0.5f));
    for (int y = bottom; y <= top; ++y) {
        for (int x = left; x <= right; ++x) {
            float x_pos = x + 0.5f, y_pos = y + 0.5f;
//            if (std::pow(x_pos, 2) + std::pow(y_pos, 2) < sphere.radius2 &&
//                 z_pos > z_buffer_[getIndex(x, y)]) {
//                g_buffer_[getIndex(x, y)] = *sphere.m;
//                z_buffer_[getIndex(x, y)] = z_pos;
//            }
            if (std::pow(x_pos - sphere.center.x(), 2) + std::pow(y_pos - sphere.center.y(), 2) < sphere.radius2) {
                float z_pos = sphereZ(x_pos, y_pos, sphere.radius2, sphere);
                if (z_pos > z_buffer_[getIndex(x, y)]) {
                    g_buffer_[getIndex(x, y)] = *sphere.m;
                    z_buffer_[getIndex(x, y)] = z_pos;
                }
            }
        }
    }
}


