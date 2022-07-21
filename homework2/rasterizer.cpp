// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

Eigen::Vector4f to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


//static bool insideTriangle(int x, int y, const Eigen::Vector3f* _v) {
//    Eigen::Vector3f v1, v2, temp1(0, 0, 0), temp2(0, 0, 0);
//    for (int i = 0; i < 3; ++i) {
//        v1 = {_v[i].x() - (float)x, _v[i].y() - (float)y, 0};
//        v2 = {_v[(i + 1) % 3].x() - _v[i].x(), _v[(i + 1) % 3].y() - _v[i].y(), 0};
//        temp2 = v1.cross(v2);
//        if (i == 0) {
//            temp1 = temp2;
//            continue;
//        }
//        if (temp1.dot(temp2) < 0.01) {
//            return false;
//        }
//        temp1 = temp2;
//    }
//    return true;
//}

//static bool insideTriangle(int x, int y, const Vector3f* _v)
//{

//    std::vector<Vector3f> ver, ver2;
//
//    ver.push_back({_v[1].x()-_v[0].x(),_v[1].y()-_v[0].y(),0}); ver2.push_back({x-_v[0].x(),y-_v[0].y(),0});
//    ver.push_back({_v[2].x()-_v[1].x(),_v[2].y()-_v[1].y(),0}); ver2.push_back({x-_v[1].x(),y-_v[1].y(),0});
//    ver.push_back({_v[0].x()-_v[2].x(),_v[0].y()-_v[2].y(),0}); ver2.push_back({x-_v[2].x(),y-_v[2].y(),0});//对应存储接下来要进行叉乘的向量
//
//    for(int i=0;i<3;i++){//逆时针叉乘，因为右手定则，只要有一个叉乘结果z坐标为负，则说明在三角形外
//        if(ver[i].cross(ver2[i]).z() < 0)
//            return false;
//    }
//    return true;
//}
static bool insideTriangle(float x, float y, const Vector3f* _v)
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f Q = {x,y,0};

    Vector3f p0p1 = _v[1] - _v[0];
    Vector3f p0Q = Q - _v[0];

    Vector3f p1p2 = _v[2] - _v[1];
    Vector3f p1Q = Q - _v[1];

    Vector3f p2p0 = _v[0] - _v[2];
    Vector3f p2Q = Q - _v[2];

    //类定义里面已经定义是逆时针，所以只用考虑同正情况。
    return p0p1.cross(p0Q).z() > 0 && p1p2.cross(p1Q).z() > 0 && p2p0.cross(p2Q).z()>0;
}


static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;  // 整体扩大z的值，所以没影响
        }

        for (int j = 0; j < 3; ++j)
        {
//            t.setVertex(i, v[i].head<3>());
//            t.setVertex(i, v[i].head<3>());
            t.setVertex(j, v[j].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        // rasterize_triangle(t);
        MSAA_rasterize_triangle(t);
    }
}

//Screen space rasterization
//void rst::rasterizer::rasterize_triangle(const Triangle& t) {
//    auto v = t.toVector4();
//
//
//    // iterate through the pixel and find if the current pixel is inside the triangle
//    int top    = std::max(std::max(t.v[0].y(), t.v[1].y()), t.v[2].y()),
//        bottom = std::min(std::min(t.v[0].y(), t.v[1].y()), t.v[2].y()),
//        right  = std::max(std::max(t.v[0].x(), t.v[1].x()), t.v[2].x()),
//        left   = std::min(std::min(t.v[0].x(), t.v[1].x()), t.v[2].y());
//    float x, y;
//    for (float pos_x = left; pos_x <= right; pos_x++) {
//        for (float pos_y = bottom; pos_y <= top; pos_y++) {
//            x = pos_x + 0.5;
//            y = pos_y + 0.5;
//            if (insideTriangle(x, y, t.v)) {
//                auto value = computeBarycentric2D(x, y, t.v);
////                [alpha, beta, gamma]
//                auto alpha = std::get<0>(value), beta = std::get<1>(value), gamma = std::get<2>(value) ;
//                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
//                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
//                z_interpolated *= w_reciprocal;
//                if (z_interpolated < depth_buf[get_index(x, y)]) {
//                    depth_buf[get_index(x, y)] = z_interpolated;
//                    set_pixel({pos_x, pos_y, z_interpolated}, t.getColor());
//                }
//            }
//        }
//    }
//}

void rst::rasterizer::MSAA_rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    int top    = std::max(std::max(std::max(t.v[0].y(), (float)0), t.v[1].y()), t.v[2].y()),
        bottom = std::min(std::min(t.v[0].x(), t.v[1].y()), t.v[2].y()),
        right  = std::max(std::max(std::max(t.v[0].x(), (float)0), t.v[1].x()), t.v[2].x()),
        left   = std::min(std::min(t.v[0].x(), t.v[1].x()), t.v[2].y());
    if (bottom < 0) {
        bottom = 0;
    }
    if (left < 0) {
        left = 0;
    }
    if (top > height) {
        top = height;
    }
    if (right < width) {
        right = width;
    }
    // 偏移量
    std::vector<std::array<float, 2> > offsets = {{0.25, 0.25}, {0.75, 0.25},
                                                  {0.25, 0.75}, {0.75, 0.75}};
    int inside_point;  // 每个像素内有效点的个数
    float point_depth;
    float x, y;

    for (float pos_x = left; pos_x <= right; pos_x++) {
        for (float pos_y = bottom; pos_y <= top; pos_y++) {
            for (int i = 0; i < 4; ++i) {
                x = pos_x + offsets[i][0];
                y = pos_y + offsets[i][1];
                if (insideTriangle(x, y, t.v)) {
                    ++inside_point;
                    auto value = computeBarycentric2D(x, y, t.v);
//                [alpha, beta, gamma]
                    auto alpha = std::get<0>(value), beta = std::get<1>(value), gamma = std::get<2>(value) ;
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    if (z_interpolated < super_depth_buf[get_index(x, y)][i]) {
                        super_depth_buf[get_index(x, y)][i] = z_interpolated;
                        set_pixel({pos_x, pos_y, z_interpolated}, t.getColor(), i);
                    }
                }
            }
        }
    }

}

//void rst::rasterizer::MSAA_rasterize_triangle(const Triangle& t) {
//    auto v = t.toVector4();
//    int top    = std::max(std::max(t.v[0].y(), t.v[1].y()), t.v[2].y()),
//            bottom = std::min(std::min(t.v[0].y(), t.v[1].y()), t.v[2].y()),
//            right  = std::max(std::max(t.v[0].x(), t.v[1].x()), t.v[2].x()),
//            left   = std::min(std::min(t.v[0].x(), t.v[1].x()), t.v[2].y());
//    std::vector<Eigen::Vector2f> super_sample_step
//            {
//                    {0.25,0.25},
//                    {0.75,0.25},
//                    {0.25,0.75},
//                    {0.75,0.75},
//            };
//    // iterate through the pixel and find if the current pixel is inside the triangle
//    for (int x = left; x <= right; x++)
//    {
//        for (int y = bottom; y <= height; y++)
//        {
//            int count = 0;
//            float minDepth = FLT_MAX;
//            for (int i = 0; i < 4; i++)
//            {
//                if (insideTriangle(x + super_sample_step[i][0], y + super_sample_step[i][1], t.v))
//                {
//                    count++;
//                }
//            }
//            if (count > 0)
//                //若像素的四个样本中有一个在三角形内，就要对这个像素进行深度测试，然后颜色直接就是所占比例
//            {
//                auto tup = computeBarycentric2D(x + 0.5, y + 0.5, t.v);
//                float alpha;
//                float beta;
//                float gamma;
//                std::tie(alpha, beta, gamma) = tup;
//                float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
//                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
//                z_interpolated *= w_reciprocal;
//                //深度测试，通过便着色，并同时将深度存入缓存
//                //这里有个细节之前没注意，就是buf的取值要用get_index函数
//                if (depth_buf[get_index(x, y)] > z_interpolated)
//                {
//                    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
//                    //深度存入缓存
//                    depth_buf[get_index(x, y)] = z_interpolated;
//                    Vector3f point = { (float)x,(float)y,z_interpolated };
//                    Vector3f color = t.getColor()*count/4; //t.getColor()* count / 4.0f + (4 - count) * frame_buf[get_index(x, y)] / 4.0f
//                    //着色
//                    set_pixel(point, color);
//                }
//            }
//        }
//    }
//}


void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{

    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::array<Eigen::Vector3f, 4> frame{Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
                                             Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()};
        std::fill(super_frame_buf.begin(), super_frame_buf.end(), frame);
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::array<float, 4> value{std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(),
                                   std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()};
        std::fill(super_depth_buf.begin(), super_depth_buf.end(), value);
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    super_frame_buf.resize(w * h);
    super_depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color, const int& index)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    super_frame_buf[ind][index] = color;

}

std::vector<Eigen::Vector3f>& rst::rasterizer::frame_buffer() {
    for (int i = 0; i < super_frame_buf.size(); ++i) {
        frame_buf[i] = (super_frame_buf[i][0] + super_frame_buf[i][1] + super_frame_buf[i][2] + super_frame_buf[i][4]) / 4;
    }
    return frame_buf;
}

// clang-format on