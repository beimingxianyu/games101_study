#pragma once


#include <cassert>
#include <array>
#include <memory>

#include "Material.hpp"
#include "OBJ_Loader.hpp"
#include "Object.hpp"



using namespace Eigen;


class Triangle : public Object
{
public:
    Vector3f v0, v1, v2; // vertices A, B ,C , counter-clockwise order
//    Vector3f e1, e2;     // 2 edges v1-v0, v2-v0;
    Bound bound;
//    Vector3f normal;
//    float area;
    Material* m;
    Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2, Material* _m = nullptr)
        : v0(_v0), v1(_v1), v2(_v2), m(_m)
    {
        updata();
    }

    Triangle& updata() {
//        e1 = v1 - v0;
//        e2 = v2 - v0;
//        normal = e1 * e2;
//        normal = e1.cross(e2).normalized();
//        area = e1.cross(e2).norm() * 0.5f;
        calculateBound();
        return *this;
    }

    Bound& calculateBound() override {
        bound.left_   = std::min(v0.x(), std::min(v1.x(), v2.x())),
        bound.right_  = std::max(v0.x(), std::max(v1.x(), v2.x())),
        bound.bottom_ = std::min(v0.y(), std::min(v1.y(), v2.y())),
        bound.top_    = std::max(v0.y(), std::max(v1.y(), v1.y()));
        return bound;
    }

    Bound calculateBound() const {
        Bound bound__;
        bound__.left_   = std::min(v0.x(), std::min(v1.x(), v2.x())),
        bound__.right_  = std::max(v0.x(), std::max(v1.x(), v2.x())),
        bound__.bottom_ = std::min(v0.y(), std::min(v1.y(), v2.y())),
        bound__.top_    = std::max(v0.y(), std::max(v1.y(), v1.y()));
        return bound__;
    }

    Bound getBound() const override {
        return bound;
    }

//    float getArea() const override {
//        return area;
//    }

    Vector3f getColor() const override {
        return m->getColor();
    }

    Triangle& movePositionX(const float& x) override {
        v0.x() += x;
        v1.x() += x;
        v2.x() += x;
        bound.left_ += x;
        bound.right_ += x;
        return *this;
    }

    Triangle& movePositionY(const float& y) override {
        v0.y() += y;
        v1.y() += y;
        v2.y() += y;
        bound.bottom_ += y;
        bound.top_ += y;
        return *this;
    }

    Triangle& movePositionZ(const float& z) override {
        v0.z() += z;
        v1.z() += z;
        v2.z() += z;
        return *this;
    }

    Triangle& movePosition(const Vector3f& dir) override {
        movePositionX(dir.x());
        movePositionY(dir.y());
        movePositionZ(dir.z());
        return *this;
    }

    Object_Type getType() const override{
        return Triangle_type;
    }
};

class MeshTriangle : public Object
{
public:
    MeshTriangle(const std::string& filename, Material *mt = new Material())
    {
        objl::Loader loader;
        loader.LoadFile(filename);
        m = mt;
        assert(loader.LoadedMeshes.size() == 1);
        auto mesh = loader.LoadedMeshes[0];

        Vector3f min_vert = Vector3f{std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity()};
        Vector3f max_vert = Vector3f{-std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity()};
        for (int i = 0; i < mesh.Vertices.size(); i += 3) {
            std::array<Vector3f, 3> face_vertices;

            for (int j = 0; j < 3; j++) {
                auto vert = Vector3f(mesh.Vertices[i + j].Position.X,
                                     mesh.Vertices[i + j].Position.Y,
                                     mesh.Vertices[i + j].Position.Z);
                face_vertices[j] = vert;
                min_vert = Vector3f(std::min(min_vert.x(), vert.x()),
                                    std::min(min_vert.y(), vert.y()),
                                    std::min(min_vert.z(), vert.z()));
                max_vert = Vector3f(std::max(max_vert.x(), vert.x()),
                                    std::max(max_vert.y(), vert.y()),
                                    std::max(max_vert.z(), vert.z()));
            }

            triangles.emplace_back(face_vertices[0], face_vertices[1],
                                   face_vertices[2], mt);
        }
        calculateBound();
    }

    Object_Type getType() const override{
        return Mesh_triangle_type;
    }

    Bound& calculateBound() override {
        for (const auto& triangle: triangles) {
            bound.left_   = std::min(bound.left_, triangle.bound.left_);
            bound.right_  = std::max(bound.right_, triangle.bound.right_);
            bound.bottom_ = std::min(bound.bottom_, triangle.bound.bottom_);
            bound.top_    = std::max(bound.top_, triangle.bound.top_);
        }
        return bound;
    }

    Bound getBound() const override {
        return bound;
    }

    MeshTriangle& movePositionX(const float& x) override {
        for (auto& triangle: triangles) {
            triangle.movePositionX(x);
        }
        bound.left_ += x;
        bound.right_ += x;
        return *this;
    }

    MeshTriangle& movePositionY(const float& y) override {
        for (auto& triangle: triangles) {
            triangle.movePositionY(y);
        }
        bound.top_ += y;
        bound.bottom_ += y;
        return *this;
    }

    MeshTriangle& movePositionZ(const float& z) override {
        for (auto& triangle: triangles) {
            triangle.movePositionZ(z);
        }
        return *this;
    }

    MeshTriangle& movePosition(const Vector3f& dir) override {
        movePositionX(dir.x());
        movePositionY(dir.y());
        movePositionZ(dir.z());
        return *this;
    }

    Vector3f getColor() const override {
        return m->getColor();
    }

    Bound bound;
    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;

    std::vector<Triangle> triangles;

    Material* m;
};


