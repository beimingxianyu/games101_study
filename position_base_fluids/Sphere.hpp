//
// Created by LEI XU on 5/13/19.
//

#ifndef RAYTRACING_SPHERE_H
#define RAYTRACING_SPHERE_H

#include "Object.hpp"
#include "Material.hpp"

class Sphere : public Object{
public:
    Vector3f center;
    float radius, radius2;
    Material *m;
    Bound bound;
//    float area;
    Sphere(const Vector3f &c, const float &r, Material* mt = new Material()) : center(c), radius(r), radius2(r * r), m(mt), bound(this)  // area(4 * M_PI *r *r)
    {}

    Sphere() = default;

    Object_Type getType() const override {
        return Sphere_type;
    }

    Bound & calculateBound() override {
        bound.left_   = center.x() - radius;
        bound.right_  = center.x() + radius;
        bound.top_    = center.y() + radius;
        bound.bottom_ = center.y() - radius;
        return bound;
    }

    [[nodiscard]] Bound calculateBound() const {
        Bound bound_;
        bound_.left_   = center.x() - radius;
        bound_.right_  = center.x() + radius;
        bound_.top_    = center.y() + radius;
        bound_.bottom_ = center.y() - radius;
        return bound_;
    }

    [[nodiscard]] Bound getBound() const override {
        return bound;
    }

    Sphere& movePositionX(const float &x) override {
        center.x() += x;
        bound.left_ += x;
        bound.right_ += x;
        return *this;
    }

    Sphere& movePositionY(const float &y) override {
        center.y() += y;
        bound.top_ += y;
        bound.bottom_ += y;
        return *this;
    }

    Sphere& movePositionZ(const float &z) override {
        center.z() += z;
        return *this;
    }

    Sphere& movePosition(const Vector3f &dir) override {
        movePositionX(dir.x());
        movePositionX(dir.y());
        movePositionX(dir.z());
        return *this;
    }

//    float getArea() const override {
//        return area;
//    }

    Vector3f getColor() const override {
        return m->getColor();
    }
};




#endif //RAYTRACING_SPHERE_H
