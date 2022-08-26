//
// Created by 北冥咸鱼 on 2022/8/21.
//

#include "position_base.hpp"

PositionBase::PositionBase(std::vector<Object*>& objects, const std::size_t transparent_flag, const Bound3D& bound, const float& r, const float& h,
                           const float &p0, const float &g0)
                           : objects_(objects), transparent_flag_(transparent_flag), bound3D_(bound), h_(h), density_0_(p0), g0_(g0), 
                             m0_(r * r * density_0_) {}



std::vector<std::vector<std::size_t>> PositionBase::proximityPoind(
    const std::size_t& beg,
    const std::size_t& end) {
    std::vector<std::vector<std::size_t>> result(end - beg);
    std::size_t index1 = 0, index2 = 1;
    for (; beg + index1 != end; ++index1, index2 = index1 + 1) {
        Sphere sphere1 = dynamic_cast<Sphere&>(*(objects_[beg + index1]));
        for (; beg + index2 != end; ++index2) {
            if ((dynamic_cast<Sphere&>(*(objects_[beg + index2])).center - sphere1.center).norm() < h_) {
                result[index1].push_back(beg + index2);
                result[index2].push_back(beg + index1);
            }
        }
    }
    return result;
}


float PositionBase::wPoly6(const Eigen::Vector3f& r) {
    float K_Poly6 = 315.0f / 64.0f / M_PI / std::pow(h_, 9);
    float r_norm(r.norm());
    if (r_norm < 0 || r_norm > h_) {
        return 0;
    }
    return K_Poly6 * std::pow((std::pow(h_, 2) - std::pow(r_norm, 2)), 3);
}

Eigen::Vector3f PositionBase::derivativeOfWPoly6(const std::size_t& i, const std::size_t& j) {
	Eigen::Vector3f rr = dynamic_cast<Sphere&>(*objects_[i]).center - dynamic_cast<Sphere&>(*objects_[j]).center;
    return -1125 / 32 / M_PI / std::pow(h_, 9) * std::pow(std::pow(h_, 2) - std::pow(rr.norm(), 2), 2) * rr;
}


float PositionBase::wSpiky(const Eigen::Vector3f& r) {
    float r_norm(r.norm());
    float K_Spiky = 15 / (M_PI * std::pow(h_, 6));
    return K_Spiky * std::pow(h_ - r_norm, 3);
}


Eigen::Vector3f PositionBase::derivativeOfWSpikyWhenTheKEqualI(const std::size_t& i) {
    float K_Spiky = 15 / (M_PI * std::pow(h_, 6));
    Sphere& sphere_i = dynamic_cast<Sphere&>(*objects_[i]);
    float temp = 0;
    for (const auto& index : proximity_poind_[i - transparent_flag_]) {
        temp += std::pow(h_ - (sphere_i.center - dynamic_cast<Sphere&>(*objects_[index]).center).norm(), 2);
    }
    return -3 * K_Spiky * temp * sphere_i.center.normalized() / density_0_;
}


Eigen::Vector3f PositionBase::derivativeOfWSpikyWhenTheKEqualJ(const std::size_t& i, const std::size_t& j) {
	float K_Spiky = 15 / (M_PI * std::pow(h_, 6));
	Sphere& sphere_i = dynamic_cast<Sphere&>(*objects_[i]);
    Sphere& sphere_j = dynamic_cast<Sphere&>(*objects_[j]);
    return -3 * K_Spiky * std::pow(h_ - (sphere_i.center - sphere_j.center).norm(), 2) * sphere_j.center.normalized() / density_0_;
}


float PositionBase::derivativeOfWSpiky(const std::size_t& i) {
    float result(0.0f);
    result += std::pow(derivativeOfWSpikyWhenTheKEqualI(i).norm(), 2);
    for (const auto& index : proximity_poind_[i - transparent_flag_]) {
        result += std::pow(derivativeOfWSpikyWhenTheKEqualJ(i, index).norm(), 2);
    }
    return result + CMF();
}


float PositionBase::densityI(const std::size_t& i) {
    float result = 0.0f;
    for (const auto& index : proximity_poind_[i - transparent_flag_]) {
        result += m0_ * wPoly6(dynamic_cast<Sphere&>(*objects_[i]).center - dynamic_cast<Sphere&>(*objects_[index]).center);
    }
    return result;
}


float PositionBase::constraintFunction(const std::size_t& i) {
    return densityI(i) / density_0_ - 1;
}

float PositionBase::CMF() {
    // TODO 以后有时间更新一下
	return EPSILON;
}


float PositionBase::S_corr(const std::size_t& i, const std::size_t& j, const float& k, const int& n) {
    float w_1 = wPoly6(dynamic_cast<Sphere&>(*(objects_[i])).center - dynamic_cast<Sphere&>(*(objects_[j])).center);
    float w_2 = wPoly6(Eigen::Vector3f(0.2 * h_, 0, 0));  // 0.2 * h_为徳塔q的模，可换为0.1 * h_ 至 0.3 * h_中间的数
    return -k * std::pow(w_1 / w_2, n);
}

float PositionBase::Ramta(const std::size_t& i) {
    float c_1(constraintFunction(i));
    float c_2(derivativeOfWSpiky(i));
    return c_1 / c_2;
}

Eigen::Vector3f PositionBase::deltaPotion(const std::size_t& i) {
    float ramta_i(Ramta(i));
    Eigen::Vector3f result(Eigen::Vector3f::Zero());
    for (const auto& index : proximity_poind_[i - transparent_flag_]) {
        result += (ramta_i + Ramta(index) + S_corr(i, index)) * derivativeOfWPoly6(i, index);
    }
    return result / density_0_;
}



