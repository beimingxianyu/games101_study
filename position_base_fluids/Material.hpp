//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include <Eigen/Eigen>

using namespace Eigen;

enum MaterialType { DIFFUSE};

class Material{
private:

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected


    Vector3f toWorld(const Vector3f &a, const Vector3f &N){
        Vector3f B, C;
        if (std::fabs(N.x()) > std::fabs(N.y())){
            float invLen = 1.0f / std::sqrt(N.x() * N.x() + N.z() * N.z());
            C = Vector3f(N.z() * invLen, 0.0f, -N.x() *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y() * N.y() + N.z() * N.z());
            C = Vector3f(0.0f, N.z() * invLen, -N.y() *invLen);
        }
        B = C.cross(N);
        return a.x() * B + a.y() * C + a.z() * N;
    }

public:
    MaterialType m_type;
    Vector3f m_color;
    //Texture tex;

    inline Material(MaterialType t=DIFFUSE,const Vector3f& color = Vector3f(0, 0, 0));
    inline MaterialType getType();
    inline Vector3f getColor() {return m_color;};
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);

};

Material::Material(MaterialType t, const Vector3f& color){
    m_type = t;
    m_color = color;
}

MaterialType Material::getType(){return m_type;}
///Vector3f Material::getColor(){return m_color;}

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}


#endif //RAYTRACING_MATERIAL_H
