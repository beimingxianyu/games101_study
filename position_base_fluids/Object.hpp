//
// Created by LEI XU on 5/13/19.
//
#pragma once
#ifndef RAYTRACING_OBJECT_H
#define RAYTRACING_OBJECT_H
#include <Eigen/Eigen>


#include "global.hpp"

using namespace Eigen;

enum Object_Type{
    Triangle_type,
    Sphere_type,
    Mesh_triangle_type
};

class Bound;

class Object
{
public:
    Object() {}
    virtual ~Object() {}
    virtual Object_Type getType() const = 0;
//    virtual float getArea() const = 0;
    virtual Bound& calculateBound() = 0;
    virtual Bound getBound() const = 0;
    virtual Object& movePositionX(const float& x) = 0;
    virtual Object& movePositionY(const float& y) = 0;
    virtual Object& movePositionZ(const float& z) = 0;
    virtual Object& movePosition(const Vector3f& dir) = 0;
    virtual Vector3f getColor() const = 0;
};

class Bound {
public:
    Bound() : left_(std::numeric_limits<float>::max()), right_(std::numeric_limits<float>::max() * -1.0f),
              bottom_(std::numeric_limits<float>::max()), top_(std::numeric_limits<float>::max() * -1.0f){}
    Bound(Object* obj) {obj->calculateBound();}
    float left_, right_, bottom_, top_;
};

class Bound3D {
public:
    Bound3D() : left_(std::numeric_limits<float>::max()), right_(std::numeric_limits<float>::max() * -1.0f),
                bottom_(std::numeric_limits<float>::max()), top_(std::numeric_limits<float>::max() * -1.0f),
                near_(std::numeric_limits<float>::max()), far_(std::numeric_limits<float>::max() * -1.0f) {}

    Bound3D(const float & top, const float & bottom, const float & left,
            const float & right, const float & near, const float & far) :
            top_(top), bottom_(bottom), left_(left), right_(right), near_(near), far_(far) {}

    float left_, right_, bottom_, top_, near_, far_;
};



#endif //RAYTRACING_OBJECT_H
